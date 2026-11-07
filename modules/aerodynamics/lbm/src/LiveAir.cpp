// See LiveAir.hpp and rotor_aero.md §8.
#include "lagrange/aerodynamics/LiveAir.hpp"
#include "lagrange/fields/Field.hpp"
#include "lagrange/gpu/Device.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <numbers>

namespace lagrange::aerodynamics {

namespace {

// The box moves when the drone has drifted this many cells from where it was centred, so a
// hovering drone's box stays put and the flow is not copied every step.
constexpr double kDriftCells = 6.0;
// How often the velocity comes back from the GPU, s of air: the rotors and links read it.
constexpr double kSyncInterval = 0.02;
// How much air is stepped at once, s: the rotors' push is held over it.
constexpr double kStepBatch = 0.005;
// The calibration disc's run, s of air: the starting vortex leaves and the jet settles
// (test_lbm_box).
constexpr double kCalibrationSeconds = 0.6;
constexpr double kCalibrationThrust = 5.0;  // N, about a hovering 2 kg quadcopter's rotor
// The box reaches down to the ground when its bottom would be closer to it than this, m: the
// outwash needs the ground in the box.
constexpr double kGroundSnap = 0.5;

using Vec3 = LiveAir::Vec3;

double dot(const Vec3& a, const Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

}  // namespace

Vec3 discMeanOf(const LbmBox::Rotor& rotor, const std::function<Vec3(const Vec3&)>& velocityAt);

LiveAir::LiveAir() = default;
LiveAir::~LiveAir() {
    box_.reset();  // before the device it runs on
    device_.reset();
}

bool LiveAir::start(const Settings& settings, std::string& why) {
    settings_ = settings;
    if (settings.cpu) {
        return true;
    }
    std::string error;
    device_ = gpu::Device::create(gpu::DeviceDesc{}, error);
    if (!device_) {
        why = "no GPU (" + error + ")";
        return false;
    }
    if (device_->info().isSoftware) {
        why = "only a software GPU (" + device_->info().name + "), too slow for live air";
        device_.reset();
        return false;
    }
    return true;
}

void LiveAir::setGround(bool has, double z) {
    hasGround_ = has;
    groundZ_ = z;
}

void LiveAir::setObstacles(std::vector<Downwash::Box> boxes) {
    obstacles_ = std::move(boxes);
}

LbmBoxConfig LiveAir::configAt(bool floor, double height) const {
    LbmBoxConfig config;
    const double dx = settings_.cell;
    config.nx = 2 * static_cast<std::size_t>(std::lround(settings_.width / (2.0 * dx)));
    config.ny = config.nx;
    config.nz = static_cast<std::size_t>(std::ceil(height / dx - 1e-9));
    config.cellSize = dx;
    config.maxSpeed = settings_.maxSpeed;
    config.airDensity = settings_.airDensity;
    config.floor = floor;
    return config;
}

LiveAir::Placement LiveAir::placementFor(const Vec3& centre) const {
    const double dx = settings_.cell;
    const LbmBoxConfig probe = configAt(false, 1.0);
    Placement p;
    // On the grid of cells, so a moved box's cells line up with the old one's.
    p.origin = {std::round((centre[0] - 0.5 * probe.nx * dx) / dx) * dx,
                std::round((centre[1] - 0.5 * probe.ny * dx) / dx) * dx,
                std::round((centre[2] - settings_.below) / dx) * dx};
    if (hasGround_ && p.origin[2] < groundZ_ + kGroundSnap) {
        p.origin[2] = groundZ_;
        p.floor = true;
    }
    p.height = centre[2] + settings_.above - p.origin[2];
    return p;
}

void LiveAir::rebuild(const Vec3& centre) {
    const double dx = settings_.cell;
    const Placement place = placementFor(centre);
    const Vec3 origin = place.origin;
    const bool floor = place.floor;
    auto next = std::make_unique<LbmBox>(configAt(floor, place.height));
    const LbmBoxConfig& config = next->config();
    if (!obstacles_.empty()) {
        Downwash shapes;
        shapes.setObstacles(obstacles_);
        for (std::size_t z = 0; z < config.nz; ++z) {
            for (std::size_t y = 0; y < config.ny; ++y) {
                for (std::size_t x = 0; x < config.nx; ++x) {
                    const Vec3 at{origin[0] + (static_cast<double>(x) + 0.5) * dx,
                                  origin[1] + (static_cast<double>(y) + 0.5) * dx,
                                  origin[2] + (static_cast<double>(z) + 0.5) * dx};
                    if (shapes.insideObstacle(at)) {
                        next->setSolid(x, y, z);
                    }
                }
            }
        }
    }
    // New cells start in the wind; where the old box was, its flow is carried over.
    next->setAmbient(ambient_);
    next->fillWithAmbient();
    if (box_) {
        box_->syncPopulations();
        box_->syncVelocity();
        const std::array<long, 3> offset{std::lround((origin[0] - origin_[0]) / dx),
                                         std::lround((origin[1] - origin_[1]) / dx),
                                         std::lround((origin[2] - origin_[2]) / dx)};
        next->carryOver(*box_, offset);
    }
    box_.reset();
    if (device_) {
        next->useGpu(*device_);
    }
    box_ = std::move(next);
    origin_ = origin;
    sinceSync_ = 0.0;
}

void LiveAir::calibrate(const LbmBox::Rotor& rotor) {
    // A lone disc of this radius in still air, in cells of this size: the share of momentum
    // theory's induced velocity the box makes through it, so airAtRotor can take it off.
    LbmBoxConfig config;
    config.nx = 2 * static_cast<std::size_t>(std::ceil(6.0 * rotor.radius / settings_.cell));
    config.ny = config.nx;
    config.nz = static_cast<std::size_t>(std::ceil(1.6 / settings_.cell));
    config.cellSize = settings_.cell;
    config.maxSpeed = settings_.maxSpeed;
    config.airDensity = settings_.airDensity;
    LbmBox lone(config);
    if (device_) {
        lone.useGpu(*device_);
    }
    LbmBox::Rotor disc = rotor;
    disc.axis = {0.0, 0.0, 1.0};
    disc.thrust = kCalibrationThrust;
    disc.centre = {0.5 * static_cast<double>(config.nx) * config.cellSize,
                   0.5 * static_cast<double>(config.ny) * config.cellSize,
                   static_cast<double>(config.nz) * config.cellSize - 0.4};
    const std::vector<LbmBox::Rotor> one{disc};
    lone.setRotors(one);
    lone.step(static_cast<std::size_t>(std::lround(kCalibrationSeconds / lone.timeStep())));
    lone.syncVelocity();
    const Vec3 mean = discMeanOf(disc, [&lone](const Vec3& at) { return lone.velocityAt(at); });
    const double area = std::numbers::pi * disc.radius * disc.radius;
    const double induced = std::sqrt(disc.thrust / (2.0 * settings_.airDensity * area));
    isolatedShare_ = -mean[2] / induced;
}

void LiveAir::advance(double dt, const Vec3& centre, std::span<const LbmBox::Rotor> rotors) {
    const double dx = settings_.cell;
    if (isolatedShare_ < 0.0 && !rotors.empty()) {
        calibrate(rotors.front());
    }
    if (!box_) {
        rebuild(centre);
    } else {
        // Move the box when the drone has drifted from where it was placed, it has come near
        // the ground or left it, or its top no longer has the air over the drone.
        const LbmBoxConfig& c = box_->config();
        const Placement want = placementFor(centre);
        const double top = origin_[2] + static_cast<double>(c.nz) * dx;
        const double limit = kDriftCells * dx;
        const bool moved = std::abs(want.origin[0] - origin_[0]) > limit ||
                           std::abs(want.origin[1] - origin_[1]) > limit ||
                           (!c.floor && std::abs(want.origin[2] - origin_[2]) > limit) ||
                           want.floor != c.floor ||
                           (c.floor && std::abs(centre[2] + settings_.above - top) > limit);
        if (moved) {
            rebuild(centre);
        }
    }
    std::vector<LbmBox::Rotor> local(rotors.begin(), rotors.end());
    for (LbmBox::Rotor& r : local) {
        for (std::size_t k = 0; k < 3; ++k) {
            r.centre.at(k) -= origin_.at(k);
        }
    }
    box_->setRotors(local);
    box_->setAmbient(ambient_);
    pending_ += dt;
    // The GPU's steps go in batches: a submit per module step costs more than the steps.
    const auto steps = pending_ >= kStepBatch
                           ? static_cast<std::size_t>(pending_ / box_->timeStep())
                           : std::size_t{0};
    if (steps > 0) {
        box_->step(steps);
        pending_ -= static_cast<double>(steps) * box_->timeStep();
    }
    sinceSync_ += dt;
    sinceField_ += dt;
    if (sinceSync_ >= kSyncInterval) {
        box_->syncVelocity();
        sinceSync_ = 0.0;
    }
}

bool LiveAir::contains(const Vec3& world) const {
    if (!box_) {
        return false;
    }
    const LbmBoxConfig& c = box_->config();
    const double dx = settings_.cell;
    const std::array<double, 3> size{c.nx * dx, c.ny * dx, c.nz * dx};
    for (std::size_t k = 0; k < 3; ++k) {
        const double local = world.at(k) - origin_.at(k);
        if (local < 0.0 || local > size.at(k)) {
            return false;
        }
    }
    return true;
}

Vec3 LiveAir::velocityAt(const Vec3& world) const {
    if (!box_) {
        return {};
    }
    const Vec3 v =
        box_->velocityAt({world[0] - origin_[0], world[1] - origin_[1], world[2] - origin_[2]});
    return {v[0] - ambient_[0], v[1] - ambient_[1], v[2] - ambient_[2]};
}

Vec3 LiveAir::discMean(const LbmBox::Rotor& rotor) const {
    return discMeanOf(rotor, [this](const Vec3& at) { return velocityAt(at); });
}

Vec3 discMeanOf(const LbmBox::Rotor& rotor, const std::function<Vec3(const Vec3&)>& velocityAt) {
    // Area-weighted points on rings over the disc, as tools/validation/lbm_hover.py samples.
    const Vec3& n = rotor.axis;
    Vec3 a = std::abs(n[0]) < 0.9 ? Vec3{1.0, 0.0, 0.0} : Vec3{0.0, 1.0, 0.0};
    const double an = dot(a, n);
    a = {a[0] - an * n[0], a[1] - an * n[1], a[2] - an * n[2]};
    const double al = std::sqrt(dot(a, a));
    a = {a[0] / al, a[1] / al, a[2] / al};
    const Vec3 b{n[1] * a[2] - n[2] * a[1], n[2] * a[0] - n[0] * a[2], n[0] * a[1] - n[1] * a[0]};
    constexpr int kRings = 6;
    constexpr int kSpokes = 16;
    Vec3 sum{};
    double weights = 0.0;
    for (int i = 1; i <= kRings; ++i) {
        const double r = rotor.radius * i / kRings;
        for (int j = 0; j < kSpokes; ++j) {
            const double t = 2.0 * std::numbers::pi * j / kSpokes;
            Vec3 at{};
            for (std::size_t k = 0; k < 3; ++k) {
                at.at(k) = rotor.centre.at(k) + r * (std::cos(t) * a.at(k) + std::sin(t) * b.at(k));
            }
            const Vec3 v = velocityAt(at);
            for (std::size_t k = 0; k < 3; ++k) {
                sum.at(k) += r * v.at(k);
            }
            weights += r;
        }
    }
    return {sum[0] / weights, sum[1] / weights, sum[2] / weights};
}

Vec3 LiveAir::airAtRotor(const LbmBox::Rotor& rotor) const {
    const Vec3 mean = discMean(rotor);
    const double area = std::numbers::pi * rotor.radius * rotor.radius;
    const double alone =
        rotor.thrust > 0.0 && isolatedShare_ > 0.0
            ? isolatedShare_ * std::sqrt(rotor.thrust / (2.0 * settings_.airDensity * area))
            : 0.0;
    // Alone, the disc's air goes against its thrust at `alone`; what is left is from elsewhere.
    return {mean[0] + alone * rotor.axis[0], mean[1] + alone * rotor.axis[1],
            mean[2] + alone * rotor.axis[2]};
}

std::shared_ptr<const fields::Field> LiveAir::field(double interval) {
    if (!box_ || (field_ && sinceField_ < interval)) {
        return field_;
    }
    sinceField_ = 0.0;
    const LbmBoxConfig& c = box_->config();
    constexpr int kStride = 2;
    fields::DenseBox dense;
    dense.size = kStride * settings_.cell;
    dense.origin = {origin_[0] + 0.5 * dense.size, origin_[1] + 0.5 * dense.size,
                    origin_[2] + 0.5 * dense.size};
    dense.count = {static_cast<int>(c.nx) / kStride, static_cast<int>(c.ny) / kStride,
                   static_cast<int>(c.nz) / kStride};
    field_ =
        fields::vectorField("air_box", dense, [this](const Vec3& at) { return velocityAt(at); });
    return field_;
}

}  // namespace lagrange::aerodynamics
