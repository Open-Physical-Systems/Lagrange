// An automotive radar by ray launching (Radar.hpp).
#include "lagrange/random/Philox.hpp"
#include "lagrange/sensors/Radar.hpp"
#include "lagrange/sensors/ShapeCaster.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numbers>
#include <tuple>

namespace lagrange::sensors {

namespace {

using namespace literals;
using Vec3 = std::array<double, 3>;

constexpr double kSpeedOfLight = 299792458.0;  // m/s (exact)
constexpr double kDefaultRcsDensity = 0.1;     // m²/m², SensorMaterialAPI_1's fallback
constexpr double kNormalStep = 1e-3;           // m, finite differences for surface normals
constexpr double kBackOff = 0.01;              // m, off the surface for the normal

SignalSpec spec(std::string name, SignalType type, Unit unit, std::string frame = {}) {
    SignalSpec s;
    s.name = std::move(name);
    s.type = type;
    s.unit = unit;
    s.frame = std::move(frame);
    return s;
}

double setting(const World::Sensors& sensors, std::uint32_t sensor, const char* key,
               double fallback) {
    const auto& map = sensors.settings[sensor];
    const auto it = map.find(key);
    return it == map.end() ? fallback : it->second;
}

Vec3 sub(const Vec3& a, const Vec3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
double dot(const Vec3& a, const Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

class Radar final : public IModule {
public:
    Radar(const World& world, std::uint32_t sensor, bool groundTruth)
        : groundTruth_(groundTruth)
        , sensor_(sensor)
        , name_(world.sensors.names[sensor])
        , label_("radar_" + world.sensors.names[sensor])
        , body_(world.sensors.body[sensor].index)
        , mount_(world.sensors.poseInBody[sensor])
        , step_(Duration{static_cast<std::int64_t>(
              std::llround(1e9 / std::max(world.sensors.rateHz[sensor], 1e-3)))}) {}

    [[nodiscard]] ModuleInfo info() const override {
        return {"sensors", label_, 0, 1, groundTruth_, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return step_; }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

    Status declare(const WorldView& world, const ModuleConfig&, SignalDeclarer& d) override {
        const World::Sensors& s = world.sensors();
        maxRange_ = setting(s, sensor_, "max_range", 0.0);
        azimuthFov_ = setting(s, sensor_, "azimuth_fov", 0.0);
        elevationFov_ = setting(s, sensor_, "elevation_fov", 0.0);
        resolution_ = setting(s, sensor_, "azimuth_resolution", 0.035);
        rangeResolution_ = setting(s, sensor_, "range_resolution", 0.2);
        velocityResolution_ = setting(s, sensor_, "velocity_resolution", 0.1);
        txPower_ = setting(s, sensor_, "tx_power", 0.0);
        gain_ = setting(s, sensor_, "antenna_gain", 1.0);
        noiseFloor_ = setting(s, sensor_, "noise_floor", 0.0);
        threshold_ = setting(s, sensor_, "detection_threshold", 10.0);
        frequency_ = setting(s, sensor_, "carrier_frequency", 77e9);
        if (maxRange_ <= 0.0 || azimuthFov_ <= 0.0 || elevationFov_ <= 0.0 || txPower_ <= 0.0 ||
            noiseFloor_ <= 0.0 || resolution_ <= 0.0) {
            return Status::error(StatusCode::InvalidConfig,
                                 "radar " + name_ +
                                     " needs its range, fields of view, power, "
                                     "noise floor and resolution");
        }
        // Rays at half the resolution, over the field of view.
        const double spacing = resolution_ / 2.0;
        const int columns = std::max(1, static_cast<int>(std::ceil(azimuthFov_ / spacing)));
        const int rows = std::max(1, static_cast<int>(std::ceil(elevationFov_ / spacing)));
        dAzimuth_ = azimuthFov_ / columns;
        dElevation_ = elevationFov_ / rows;
        rays_.clear();
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < columns; ++c) {
                const double az = -azimuthFov_ / 2.0 + (c + 0.5) * dAzimuth_;
                const double el = -elevationFov_ / 2.0 + (r + 0.5) * dElevation_;
                rays_.push_back(
                    {az,
                     el,
                     {std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el)}});
            }
        }
        const World& raw = world.raw();
        const std::vector<bool> moving = movingBodies(raw);
        rest_ = restPoses(raw);
        poses_.assign(moving.size(), SignalId{});
        twists_.assign(moving.size(), SignalId{});
        const auto watch = [&](std::uint32_t b) {
            if (moving[b] && !poses_[b].valid()) {
                poses_[b] = d.input(spec("body/" + raw.bodies.names[b] + "/pose", SignalType::Pose,
                                         units::m, "world_local"));
                twists_[b] = d.input(spec("body/" + raw.bodies.names[b] + "/twist",
                                          SignalType::Wrench, units::m_per_s, "world_local"));
            }
        };
        watch(body_);
        for (const BodyId owner : raw.colliders.body) {
            if (owner != kWorldBody) {
                watch(owner.index);
            }
        }
        // Radar cross-section densities by body (SensorMaterialAPI_1 on a body's prim).
        density_.assign(raw.bodies.names.size(), kDefaultRcsDensity);
        for (const SchemaInstance* m : world.instancesOf("SensorMaterialAPI_1")) {
            for (std::size_t b = 0; b < raw.bodies.prim.size(); ++b) {
                if (raw.bodies.prim[b] == m->prim) {
                    density_[b] =
                        m->real("sensmat:radarCrossSectionDensity").value_or(kDefaultRcsDensity);
                }
            }
        }
        SignalSpec out =
            spec("sensor/" + name_ + "/detections", SignalType::ScalarArray, units::one);
        out.arrayLength = static_cast<std::uint32_t>(kRadarMaxDetections * kRadarFields);
        detections_ = d.output(out);
        count_ = d.output(spec("sensor/" + name_ + "/count", SignalType::Int, units::one));
        return {};
    }

    Status init(const WorldView& world, const ModuleConfig&, SignalBus& bus) override {
        const World::Colliders& colliders = world.colliders();
        shapes_.clear();
        for (std::size_t c = 0; c < colliders.body.size(); ++c) {
            const BodyId owner = colliders.body[c];
            if (colliders.shape[c] == ShapeType::HeightField) {
                continue;  // terrain is ground clutter, as the ground plane is (Radar.hpp)
            }
            Shape shape{colliders.shape[c],
                        colliders.size[c],
                        colliders.poseInBody[c],
                        owner == kWorldBody ? -1 : static_cast<int>(owner.index),
                        colliders.shape[c] == ShapeType::ConvexHull
                            ? &world.raw().hulls.at(colliders.hull[c])
                            : nullptr,
                        owner == kWorldBody ? kDefaultRcsDensity : density_[owner.index]};
            if (owner == kWorldBody) {
                shape.moving = -1;
            } else if (!poses_[owner.index].valid()) {
                shape.pose = compose(rest_[owner.index], colliders.poseInBody[c]);
                shape.moving = -1;
            } else {
                shape.moving = static_cast<int>(owner.index);
            }
            shapes_.push_back(shape);
        }
        placed_.resize(shapes_.size());
        casterShapes_.resize(shapes_.size());
        write(bus, {});
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        if (groundTruth_) {
            write(bus, {});
            return {};
        }
        const Pose sensorPose = compose(pose(bus, body_), mount_);
        const Vec3 sensorVelocity = pointVelocity(bus, body_, sensorPose.position);
        for (std::size_t k = 0; k < shapes_.size(); ++k) {
            placed_[k] = shapes_[k].moving < 0
                             ? shapes_[k].pose
                             : compose(pose(bus, static_cast<std::uint32_t>(shapes_[k].moving)),
                                       shapes_[k].pose);
            casterShapes_[k] = {shapes_[k].type, shapes_[k].size, placed_[k], shapes_[k].hull};
        }
        caster_.update(casterShapes_);
        // Echoes by object and range cell (an object's angular extent is one detection).
        struct Cell {
            double power{0.0};                                               // Σσ (m²)
            double range{0.0}, azimuth{0.0}, elevation{0.0}, velocity{0.0};  // σ-weighted sums
        };
        std::map<std::pair<int, long>, Cell> cells;
        for (const Ray& ray : rays_) {
            const Vec3 dir = rotate(sensorPose.orientation, ray.direction);
            const services::RayHit hit = caster_.castHit(sensorPose.position, dir);
            if (!hit.hit() || hit.distance > maxRange_) {
                continue;
            }
            const Shape& shape = shapes_[hit.shape];
            // The ground plane is clutter, not a target: a real radar's CFAR and clutter map
            // remove the road surface's return, which this model does not simulate.
            if (shape.type == ShapeType::Plane) {
                continue;
            }
            const Vec3 p{sensorPose.position[0] + dir[0] * hit.distance,
                         sensorPose.position[1] + dir[1] * hit.distance,
                         sensorPose.position[2] + dir[2] * hit.distance};
            // Incidence from the surface normal: the gradient of the distance just outside.
            const Vec3 out{p[0] - dir[0] * kBackOff, p[1] - dir[1] * kBackOff,
                           p[2] - dir[2] * kBackOff};
            const ConvexHull* hull = shape.hull;
            const auto distance = [&](const Vec3& q) {
                return distanceToShape(shape.type, shape.size, placed_[hit.shape], q, hull);
            };
            Vec3 normal{distance({out[0] + kNormalStep, out[1], out[2]}) -
                            distance({out[0] - kNormalStep, out[1], out[2]}),
                        distance({out[0], out[1] + kNormalStep, out[2]}) -
                            distance({out[0], out[1] - kNormalStep, out[2]}),
                        distance({out[0], out[1], out[2] + kNormalStep}) -
                            distance({out[0], out[1], out[2] - kNormalStep})};
            const double n = std::sqrt(dot(normal, normal));
            const double cosine = n > 0.0 ? std::abs(dot(normal, dir)) / n : 1.0;
            const double solidAngle = dAzimuth_ * dElevation_ * std::cos(ray.elevation);
            const double rcs = shape.density * hit.distance * hit.distance * solidAngle * cosine;
            if (rcs <= 0.0) {
                continue;
            }
            // Radial velocity of the hit point relative to the radar (receding positive).
            const Vec3 target =
                shape.moving < 0 ? Vec3{}
                                 : pointVelocity(bus, static_cast<std::uint32_t>(shape.moving), p);
            const double radial = dot(sub(target, sensorVelocity), dir);
            const int object = shape.moving >= 0 ? shape.moving : -1 - static_cast<int>(hit.shape);
            Cell& cell = cells[{object, std::lround(hit.distance / rangeResolution_)}];
            cell.power += rcs;
            cell.range += rcs * hit.distance;
            cell.azimuth += rcs * ray.azimuth;
            cell.elevation += rcs * ray.elevation;
            cell.velocity += rcs * radial;
        }
        std::vector<std::array<double, kRadarFields>> found;
        for (const auto& [key, cell] : cells) {
            const double range = cell.range / cell.power;
            const double snr =
                radarSnr(txPower_, gain_, frequency_, cell.power, range, noiseFloor_);
            if (snr < threshold_) {
                continue;
            }
            found.push_back({range, cell.azimuth / cell.power, cell.elevation / cell.power,
                             cell.velocity / cell.power, cell.power, snr});
        }
        // Strongest first, at most kRadarMaxDetections, with measurement noise.
        std::sort(found.begin(), found.end(),
                  [](const auto& a, const auto& b) { return a[5] > b[5]; });
        if (found.size() > kRadarMaxDetections) {
            found.resize(kRadarMaxDetections);
        }
        random::Stream rng(context.seed, context.stepIndex, 0);
        for (auto& f : found) {
            const double spread = 1.0 / std::sqrt(2.0 * f[5]);
            f[0] += rangeResolution_ * spread * rng.normal();
            f[1] += resolution_ * spread * rng.normal();
            f[2] += resolution_ * spread * rng.normal();
            f[3] += velocityResolution_ * spread * rng.normal();
        }
        write(bus, found);
        return {};
    }

private:
    struct Ray {
        double azimuth, elevation;
        Vec3 direction;
    };
    struct Shape {
        ShapeType type;
        std::array<double, 3> size;
        Pose pose;  // world pose when not moving, else pose in its body
        int moving;
        const ConvexHull* hull;
        double density;
    };

    Pose pose(const SignalBus& bus, std::uint32_t body) const {
        if (!poses_[body].valid()) {
            return rest_[body];
        }
        const std::span<const double> v = bus.readN(poses_[body]);
        return {{v[0], v[1], v[2]}, {v[3], v[4], v[5], v[6]}};
    }

    /// The velocity of point `p` (world) of body `body`: v + ω × (p − origin).
    Vec3 pointVelocity(const SignalBus& bus, std::uint32_t body, const Vec3& p) const {
        if (!twists_[body].valid()) {
            return {};
        }
        const std::span<const double> t = bus.readN(twists_[body]);
        const Pose at = pose(bus, body);
        const Vec3 w{t[3], t[4], t[5]};
        const Vec3 r = sub(p, at.position);
        const Vec3 spin = cross(w, r);
        return {t[0] + spin[0], t[1] + spin[1], t[2] + spin[2]};
    }

    void write(SignalBus& bus, const std::vector<std::array<double, kRadarFields>>& found) {
        std::vector<double> out(kRadarMaxDetections * kRadarFields,
                                std::numeric_limits<double>::quiet_NaN());
        for (std::size_t i = 0; i < found.size(); ++i) {
            std::copy(found[i].begin(), found[i].end(), out.begin() + i * kRadarFields);
        }
        bus.writeN(detections_, out);
        bus.write(count_, static_cast<double>(found.size()));
    }

    bool groundTruth_;
    std::uint32_t sensor_;
    std::string name_;
    std::string label_;
    std::uint32_t body_;
    Pose mount_;
    Duration step_;
    double maxRange_{0.0}, azimuthFov_{0.0}, elevationFov_{0.0}, resolution_{0.0};
    double rangeResolution_{0.2}, velocityResolution_{0.1}, txPower_{0.0}, gain_{1.0};
    double noiseFloor_{0.0}, threshold_{10.0}, frequency_{77e9};
    double dAzimuth_{0.0}, dElevation_{0.0};
    std::vector<Ray> rays_;
    std::vector<Pose> rest_;
    std::vector<SignalId> poses_, twists_;
    std::vector<double> density_;
    std::vector<Shape> shapes_;
    std::vector<Pose> placed_;
    std::vector<ShapeCaster::Shape> casterShapes_;
    ShapeCaster caster_;
    SignalId detections_, count_;
};

}  // namespace

double radarSnr(double txPower, double gain, double frequency, double rcs, double range,
                double noiseFloor) {
    const double lambda = kSpeedOfLight / frequency;
    const double fourPi = 4.0 * std::numbers::pi;
    return txPower * gain * gain * lambda * lambda * rcs /
           (fourPi * fourPi * fourPi * std::pow(range, 4) * noiseFloor);
}

std::unique_ptr<IModule> createRadar(const World& world, std::uint32_t sensor, bool groundTruth) {
    return std::make_unique<Radar>(world, sensor, groundTruth);
}

}  // namespace lagrange::sensors
