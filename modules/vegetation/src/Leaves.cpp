#include "lagrange/random/Philox.hpp"
#include "lagrange/vegetation/Leaves.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <numbers>

namespace lagrange::vegetation {

namespace {

using Vec3 = std::array<double, 3>;
constexpr double kPi = std::numbers::pi;
/// m, the side of a bucket of LandingSurfaces.
constexpr double kBucket = 4.0;
/// A surface over this many buckets a side is checked everywhere instead.
constexpr std::int64_t kMaxBuckets = 64;
/// Under this, a falling particle with nothing under it is dropped where it is (m below 0).
constexpr double kLostDepth = 1000.0;
/// Random streams of the stripping: entities from here on, one per tree (the particles' ids stay
/// far below).
constexpr std::uint32_t kStripEntity = 0xF0000000U;
/// The blade's length over √(leaf area): a 5 cm × 6 cm blade is some 6 cm long (drawing level).
constexpr double kBladeLength = 1.1;
/// Carried fractions this close under 1 count as 1 (three thirds make one leaf).
constexpr double kCarryRound = 1e-9;

double dot(const Vec3& a, const Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

Vec3 airAt(const Vec3& point, const Vec3& wind, std::span<const RotorWake> rotors) {
    Vec3 air = wind;
    for (const RotorWake& r : rotors) {
        const Vec3 w = rotorWake(r, point);
        air = {air[0] + w[0], air[1] + w[1], air[2] + w[2]};
    }
    return air;
}

bool upright(const Pose& pose) {
    constexpr double kUpright = 0.999;  // cos of some 2.6°
    return rotate(pose.orientation, {0.0, 0.0, 1.0})[2] > kUpright;
}

}  // namespace

double Leaf::terminalSpeed(double gravity) const {
    return drag > 0.0 ? std::sqrt(gravity / drag) : std::numeric_limits<double>::infinity();
}

std::array<double, 2> Leaf::orientation(double tiltAmplitude) const {
    const double tilt = settled > 0.0 ? 0.0 : tiltAmplitude * std::sin(phase);
    const double turned = tilt >= 0.0 ? yaw : yaw + kPi;
    return {std::remainder(turned, 2.0 * kPi), 0.5 * kPi - std::abs(tilt)};
}

Vec3 rotorWake(const RotorWake& rotor, const Vec3& point) {
    const double vi = std::abs(rotor.induced);
    const double norm = std::hypot(rotor.thrustAxis[0], rotor.thrustAxis[1], rotor.thrustAxis[2]);
    const double radius = rotor.radius;
    if (!(vi > 0.0) || !(norm > 0.0) || !(radius > 0.0)) {
        return {};
    }
    const Vec3 n{rotor.thrustAxis[0] / norm, rotor.thrustAxis[1] / norm,
                 rotor.thrustAxis[2] / norm};
    const Vec3 d{point[0] - rotor.hub[0], point[1] - rotor.hub[1], point[2] - rotor.hub[2]};
    const double s = -dot(d, n);  // downstream of the disc
    if (s < -kInflowRadii * radius) {
        return {};
    }
    const Vec3 across{d[0] + s * n[0], d[1] + s * n[1], d[2] + s * n[2]};
    const double r = std::hypot(across[0], across[1], across[2]);
    // The vortex cylinder on the axis (Leishman 2006, §2.7), top hat by continuity below the disc.
    const auto cylinder = [&](double at) { return vi * (1.0 + at / std::hypot(at, radius)); };
    double w = 0.0;
    double width = radius;
    const double end = kNearWakeRadii * radius;
    if (s <= end) {
        w = cylinder(s);
        if (s > 0.0) {
            width = radius * std::sqrt(vi / w);
        }
    } else {
        // A round jet that keeps its momentum flux w² width²: w ∝ 1/s, width ∝ s.
        const double wEnd = cylinder(end);
        w = wEnd * end / s;
        width = radius * std::sqrt(vi / wEnd) * s / end;
    }
    if (r > width) {
        return {};
    }
    return {-w * n[0], -w * n[1], -w * n[2]};
}

std::uint64_t LandingSurfaces::bucketKey(std::int64_t x, std::int64_t y) {
    return (static_cast<std::uint64_t>(x) << 32U) ^ (static_cast<std::uint64_t>(y) & 0xFFFFFFFFU);
}

void LandingSurfaces::build(const World& world) {
    surfaces_.clear();
    everywhere_.clear();
    buckets_.clear();
    fields_.clear();
    const World::Colliders& c = world.colliders;
    for (std::size_t i = 0; i < c.body.size(); ++i) {
        if (c.body[i] != kWorldBody) {
            continue;
        }
        Surface s;
        s.type = c.shape[i];
        s.pose = c.poseInBody[i];
        s.size = c.size[i];
        double reach = 0.0;  // the footprint's radius
        switch (s.type) {
            case ShapeType::Plane:
                if (rotate(s.pose.orientation, {0.0, 0.0, 1.0})[2] < 0.1) {
                    continue;  // a wall
                }
                reach = std::numeric_limits<double>::infinity();
                break;
            case ShapeType::HeightField:
                if (!upright(s.pose) || c.hull[i] >= world.heightfields.size()) {
                    continue;
                }
                s.field = static_cast<std::uint32_t>(fields_.size());
                fields_.push_back(world.heightfields[c.hull[i]]);
                reach = std::numeric_limits<double>::infinity();
                break;
            case ShapeType::Box:
                if (!upright(s.pose)) {
                    continue;
                }
                reach = std::hypot(s.size[0], s.size[1]);
                break;
            case ShapeType::Cylinder:
                if (!upright(s.pose)) {
                    continue;
                }
                reach = s.size[0];
                break;
            default:
                continue;
        }
        const auto index = static_cast<std::uint32_t>(surfaces_.size());
        surfaces_.push_back(s);
        const double x0 = std::floor((s.pose.position[0] - reach) / kBucket);
        const double x1 = std::floor((s.pose.position[0] + reach) / kBucket);
        const double y0 = std::floor((s.pose.position[1] - reach) / kBucket);
        const double y1 = std::floor((s.pose.position[1] + reach) / kBucket);
        if (!std::isfinite(reach) || x1 - x0 >= kMaxBuckets || y1 - y0 >= kMaxBuckets) {
            everywhere_.push_back(index);
            continue;
        }
        for (auto x = static_cast<std::int64_t>(x0); x <= static_cast<std::int64_t>(x1); ++x) {
            for (auto y = static_cast<std::int64_t>(y0); y <= static_cast<std::int64_t>(y1); ++y) {
                buckets_[bucketKey(x, y)].push_back(index);
            }
        }
    }
}

double LandingSurfaces::top(const Surface& s, double x, double y, bool& over) const {
    over = false;
    const Vec3& p = s.pose.position;
    if (s.type == ShapeType::Plane) {
        const Vec3 n = rotate(s.pose.orientation, {0.0, 0.0, 1.0});
        over = true;
        return p[2] - ((n[0] * (x - p[0])) + (n[1] * (y - p[1]))) / n[2];
    }
    const Vec3 q = rotate(conjugate(s.pose.orientation), {x - p[0], y - p[1], 0.0});
    switch (s.type) {
        case ShapeType::HeightField: {
            const std::optional<double> h = fields_[s.field].heightAt(q[0], q[1]);
            over = h.has_value();
            return over ? p[2] + *h : 0.0;
        }
        case ShapeType::Box:
            over = std::abs(q[0]) <= s.size[0] && std::abs(q[1]) <= s.size[1];
            return p[2] + s.size[2];
        case ShapeType::Cylinder:
            over = std::hypot(q[0], q[1]) <= s.size[0];
            return p[2] + s.size[1];
        default:
            return 0.0;
    }
}

double LandingSurfaces::below(double x, double y, double z) const {
    double best = -std::numeric_limits<double>::infinity();
    const auto consider = [&](std::uint32_t index) {
        bool over = false;
        const double t = top(surfaces_[index], x, y, over);
        if (over && t <= z + 1e-9) {
            best = std::max(best, t);
        }
    };
    for (const std::uint32_t index : everywhere_) {
        consider(index);
    }
    const auto it = buckets_.find(bucketKey(static_cast<std::int64_t>(std::floor(x / kBucket)),
                                            static_cast<std::int64_t>(std::floor(y / kBucket))));
    if (it != buckets_.end()) {
        for (const std::uint32_t index : it->second) {
            consider(index);
        }
    }
    return best;
}

void LeafField::reset(std::size_t trees) {
    leaves_.clear();
    cutCarry_.assign(trees, 0.0);
    stripCarry_.assign(trees, 0.0);
    stripped_.assign(trees, 0.0);
    nextId_ = 0.0;
    spawnedThisStep_ = 0;
    spawnStep_ = ~std::uint64_t{0};
}

void LeafField::add(std::uint32_t tree, const Vec3& position, const Vec3& velocity, double leafArea,
                    double jitter, std::uint64_t seed, std::uint64_t step) {
    if (step != spawnStep_) {
        spawnStep_ = step;
        spawnedThisStep_ = 0;
    }
    ++spawnedThisStep_;
    Leaf leaf;
    leaf.id = nextId_;
    nextId_ += 1.0;
    random::Stream rng(seed, step, static_cast<std::uint32_t>(std::fmod(leaf.id, 4294967296.0)));
    const auto spread = [&]() { return 1.0 + params_.spread * (2.0 * rng.uniform() - 1.0); };
    const double speed = params_.terminalSpeed * spread();
    leaf.drag = speed > 0.0 ? params_.gravity / (speed * speed) : 0.0;
    leaf.size = kBladeLength * std::sqrt(std::max(leafArea, 0.0)) * spread();
    leaf.yaw = 2.0 * kPi * rng.uniform();
    leaf.phase = 2.0 * kPi * rng.uniform();
    leaf.spin = params_.yawRate * (2.0 * rng.uniform() - 1.0);
    for (std::size_t k = 0; k < 3; ++k) {
        leaf.position.at(k) = position.at(k) + jitter * (2.0 * rng.uniform() - 1.0);
    }
    leaf.velocity = velocity;
    leaf.tree = tree;
    if (leaves_.size() >= kLeavesCapacity) {
        leaves_.erase(leaves_.begin());  // the oldest goes
    }
    leaves_.push_back(leaf);
}

std::uint32_t LeafField::spawnFromCut(std::uint32_t tree, const TreeSpec& spec,
                                      std::span<const CutCell> cells,
                                      std::span<const RotorWake> rotors, const Vec3& wind,
                                      std::uint64_t seed, std::uint64_t step) {
    if (tree >= cutCarry_.size() || !(spec.leafArea > 0.0)) {
        return 0;
    }
    constexpr double kCellVolume = kTreeCutCell * kTreeCutCell * kTreeCutCell;
    const double perCell = kCellVolume * spec.leafAreaDensity / spec.leafArea;
    std::uint32_t started = 0;
    for (const CutCell& cell : cells) {
        cutCarry_[tree] += perCell;
        while (cutCarry_[tree] >= 1.0 - kCarryRound) {
            cutCarry_[tree] -= 1.0;
            if (step == spawnStep_ && spawnedThisStep_ >= params_.maxSpawnPerStep) {
                continue;  // over the step's cap: not drawn
            }
            const Vec3 air = airAt(cell.centre, wind, rotors);
            const double e = params_.restitution;
            add(tree, cell.centre,
                {e * cell.bladeVelocity[0] + air[0], e * cell.bladeVelocity[1] + air[1],
                 e * cell.bladeVelocity[2] + air[2]},
                spec.leafArea, 0.5 * kTreeCutCell, seed, step);
            ++started;
        }
    }
    return started;
}

std::uint32_t LeafField::strip(std::uint32_t tree, const TreeSpec& spec, double cutVolume,
                               const Vec3& air, const Vec3& sway, std::span<const RotorWake> rotors,
                               const Vec3& wind, double dt, std::uint64_t seed,
                               std::uint64_t step) {
    if (tree >= stripCarry_.size() || !(params_.stripSpeed > 0.0) || !(spec.leafArea > 0.0)) {
        return 0;
    }
    const double ratio = dot(air, air) / (params_.stripSpeed * params_.stripSpeed);
    if (ratio <= 1.0) {
        return 0;
    }
    const double remaining =
        std::max(0.0, ((spec.crownVolume() - cutVolume) * spec.leafAreaDensity / spec.leafArea) -
                          stripped_[tree]);
    stripCarry_[tree] += params_.stripRate * (ratio - 1.0) * remaining * dt;
    random::Stream rng(seed, step, kStripEntity + tree);
    std::uint32_t started = 0;
    while (stripCarry_[tree] >= 1.0 - kCarryRound) {
        stripCarry_[tree] -= 1.0;
        stripped_[tree] += 1.0;
        if (step == spawnStep_ && spawnedThisStep_ >= params_.maxSpawnPerStep) {
            continue;
        }
        // A point of the crown's surface: a direction uniform on the sphere, on the ellipsoid.
        Vec3 u{rng.normal(), rng.normal(), rng.normal()};
        const double l = std::hypot(u[0], u[1], u[2]);
        if (!(l > 0.0)) {
            continue;
        }
        const Vec3 q{spec.crownRadius * u[0] / l, spec.crownRadius * u[1] / l,
                     spec.centreHeight() + (spec.halfHeight() * u[2] / l)};
        const Vec3 o = rotate(spec.pose.orientation, q);
        const Vec3 at{spec.pose.position[0] + o[0] + sway[0],
                      spec.pose.position[1] + o[1] + sway[1],
                      spec.pose.position[2] + o[2] + sway[2]};
        add(tree, at, airAt(at, wind, rotors), spec.leafArea, 0.0, seed, step);
        ++started;
    }
    return started;
}

bool LeafField::step(double dt, const Vec3& wind, std::span<const RotorWake> rotors,
                     const LandingSurfaces& ground) {
    if (!(dt > 0.0)) {
        return false;
    }
    bool moved = false;
    const double g = params_.gravity;
    const double omega = 2.0 * kPi * params_.flutterHz;
    for (Leaf& leaf : leaves_) {
        if (leaf.settled > 0.0) {
            continue;
        }
        moved = true;
        const Vec3 air = airAt(leaf.position, wind, rotors);
        const Vec3 u{leaf.velocity[0] - air[0], leaf.velocity[1] - air[1],
                     leaf.velocity[2] - air[2]};
        const double kv = leaf.drag * std::hypot(u[0], u[1], u[2]);
        leaf.phase = std::remainder(leaf.phase + (omega * dt), 2.0 * kPi);
        leaf.yaw = std::remainder(leaf.yaw + (leaf.spin * dt), 2.0 * kPi);
        const double glide = params_.glide * g * std::sin(leaf.phase);
        const Vec3 a{glide * std::cos(leaf.yaw), glide * std::sin(leaf.yaw), -g};
        for (std::size_t k = 0; k < 3; ++k) {
            leaf.velocity.at(k) =
                (leaf.velocity.at(k) + dt * (a.at(k) + kv * air.at(k))) / (1.0 + dt * kv);
        }
        const double before = leaf.position[2];
        for (std::size_t k = 0; k < 3; ++k) {
            leaf.position.at(k) += dt * leaf.velocity.at(k);
        }
        const double floor = ground.below(leaf.position[0], leaf.position[1], before);
        if (leaf.position[2] <= floor || leaf.position[2] < -kLostDepth) {
            leaf.position[2] = std::max(floor, leaf.position[2]);
            leaf.velocity = {};
            leaf.settled = 1.0;
        }
    }
    return moved;
}

std::uint32_t LeafField::flying(std::uint32_t tree) const {
    return static_cast<std::uint32_t>(std::ranges::count_if(leaves_, [&](const Leaf& l) {
        return l.settled == 0.0 && l.tree == static_cast<double>(tree);
    }));
}

void LeafField::write(std::vector<double>& out, double version) const {
    out.assign(kLeavesLength, 0.0);
    out[0] = version;
    out[1] = static_cast<double>(leaves_.size());
    for (std::size_t k = 0; k < leaves_.size(); ++k) {
        const Leaf& l = leaves_[k];
        const std::array<double, 2> o = l.orientation(params_.tiltAmplitude);
        const std::size_t at = kLeavesHeader + (kLeavesFields * k);
        out[at] = l.position[0];
        out[at + 1] = l.position[1];
        out[at + 2] = l.position[2];
        out[at + 3] = o[0];
        out[at + 4] = o[1];
        out[at + 5] = l.size;
    }
}

std::vector<double> LeafField::state() const {
    std::vector<double> out;
    out.push_back(nextId_);
    out.push_back(static_cast<double>(cutCarry_.size()));
    for (const std::vector<double>* v : {&cutCarry_, &stripCarry_, &stripped_}) {
        out.insert(out.end(), v->begin(), v->end());
    }
    out.push_back(static_cast<double>(leaves_.size()));
    for (const Leaf& l : leaves_) {
        std::array<double, kLeafDoubles> raw{};
        std::memcpy(raw.data(), &l, sizeof(Leaf));
        out.insert(out.end(), raw.begin(), raw.end());
    }
    return out;
}

bool LeafField::assign(std::span<const double> state) {
    std::size_t at = 0;
    const auto next = [&](double& v) {
        if (at >= state.size()) {
            return false;
        }
        v = state[at++];
        return true;
    };
    double id = 0.0;
    double trees = 0.0;
    if (!next(id) || !next(trees) || trees < 0.0) {
        return false;
    }
    const auto n = static_cast<std::size_t>(trees);
    std::vector<double> carry(n);
    std::vector<double> strip(n);
    std::vector<double> stripped(n);
    for (std::vector<double>* v : {&carry, &strip, &stripped}) {
        for (double& x : *v) {
            if (!next(x)) {
                return false;
            }
        }
    }
    double count = 0.0;
    if (!next(count) || count < 0.0 || count > kLeavesCapacity) {
        return false;
    }
    const auto c = static_cast<std::size_t>(count);
    if (state.size() - at != c * kLeafDoubles) {
        return false;
    }
    std::vector<Leaf> leaves(c);
    for (Leaf& l : leaves) {
        std::memcpy(static_cast<void*>(&l), state.data() + at, sizeof(Leaf));
        at += kLeafDoubles;
    }
    nextId_ = id;
    cutCarry_ = std::move(carry);
    stripCarry_ = std::move(strip);
    stripped_ = std::move(stripped);
    leaves_ = std::move(leaves);
    spawnedThisStep_ = 0;
    spawnStep_ = ~std::uint64_t{0};
    return true;
}

}  // namespace lagrange::vegetation
