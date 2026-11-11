#pragma once
// Private to lagrange_sensors: what the environment sensor kinds share (sensors.md §9): the
// sensor's own motion from its body's signals, optional inputs from other domains that read NaN
// while their domain is off, and the colliders for a ray or two.

#include "KindModule.hpp"
#include "lagrange/random/Philox.hpp"
#include "lagrange/sensors/ShapeCaster.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <span>
#include <string>
#include <vector>

namespace lagrange::sensors::kinds {
// Its own inline namespace: the kinds' private headers may name helpers alike (ODR).
inline namespace env_common {

using Vec3 = std::array<double, 3>;

inline constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

inline Vec3 sub(const Vec3& a, const Vec3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
inline double dot(const Vec3& a, const Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
inline double norm(const Vec3& a) {
    return std::sqrt(dot(a, a));
}
inline bool finite(const Vec3& a) {
    return std::isfinite(a[0]) && std::isfinite(a[1]) && std::isfinite(a[2]);
}

/// A Vec3 signal's three components.
inline Vec3 readVec3(const SignalBus& bus, SignalId id) {
    const std::span<const double> v = bus.readN(id);
    return {v[0], v[1], v[2]};
}

/// An input another domain may not write: unconnected, it reads NaN, so a sensor can tell "no
/// weather" from a zero (sensors.md §9: such outputs are NaN with `valid` 0).
inline SignalSpec optionalNaN(std::string name, SignalType type, Unit unit,
                              std::string frame = {}) {
    SignalSpec s = kindSpec(std::move(name), type, unit, std::move(frame));
    s.optional = true;
    s.defaultValue = kNaN;
    return s;
}

/// Whether `name` is a rotor of the run (a RotorAPI_1 joint, by its signal name): an `air_rotor`
/// that names none would read NaN for ever, taken as still air.
inline bool isRotor(const WorldView& view, std::string_view name) {
    for (const SchemaInstance* rotor : view.instancesOf("RotorAPI_1")) {
        if (view.signalInstanceName(rotor->prim) == name) {
            return true;
        }
    }
    return false;
}

/// The body a sensor rides on: its pose and twist inputs when it moves, its rest pose otherwise.
class BodyMotion {
public:
    void declare(const World& world, std::uint32_t body, SignalDeclarer& d, bool twist) {
        body_ = body;
        rest_ = restPoses(world).at(body);
        pose_ = twist_ = SignalId{};
        if (movingBodies(world)[body]) {
            const std::string base = "body/" + world.bodies.names[body];
            pose_ = d.input(kindSpec(base + "/pose", SignalType::Pose, units::m, "world_local"));
            if (twist) {
                twist_ = d.input(
                    kindSpec(base + "/twist", SignalType::Wrench, units::m_per_s, "world_local"));
            }
        }
    }
    [[nodiscard]] Pose pose(const SignalBus& bus) const {
        if (!pose_.valid()) {
            return rest_;
        }
        const std::span<const double> v = bus.readN(pose_);
        return {{v[0], v[1], v[2]}, {v[3], v[4], v[5], v[6]}};
    }
    /// World velocity of the body's point `p` (world): v + ω × (p − origin).
    [[nodiscard]] Vec3 velocityAt(const SignalBus& bus, const Vec3& p) const {
        if (!twist_.valid()) {
            return {};
        }
        const std::span<const double> t = bus.readN(twist_);
        const Vec3 spin = cross({t[3], t[4], t[5]}, sub(p, pose(bus).position));
        return {t[0] + spin[0], t[1] + spin[1], t[2] + spin[2]};
    }
    [[nodiscard]] std::uint32_t body() const { return body_; }

private:
    std::uint32_t body_{0};
    Pose rest_;
    SignalId pose_, twist_;
};

/// Every collider at its body's pose, for a few rays per step (CPU ShapeCaster, exact and
/// deterministic on every tier). Declares the pose inputs of the moving bodies it needs.
class ColliderScene {
public:
    void declare(const World& world, SignalDeclarer& d) {
        const std::vector<bool> moving = movingBodies(world);
        rest_ = restPoses(world);
        poses_.assign(moving.size(), SignalId{});
        for (const BodyId owner : world.colliders.body) {
            if (owner != kWorldBody && moving[owner.index] && !poses_[owner.index].valid()) {
                poses_[owner.index] =
                    d.input(kindSpec("body/" + world.bodies.names[owner.index] + "/pose",
                                     SignalType::Pose, units::m, "world_local"));
            }
        }
    }
    void init(const World& world) {
        const World::Colliders& c = world.colliders;
        owner_.assign(c.body.size(), -1);
        local_.assign(c.body.size(), Pose{});
        shapes_.assign(c.body.size(), ShapeCaster::Shape{});
        for (std::size_t k = 0; k < c.body.size(); ++k) {
            ShapeCaster::Shape& s = shapes_[k];
            s.type = c.shape[k];
            s.size = c.size[k];
            s.hull = c.shape[k] == ShapeType::ConvexHull ? &world.hulls.at(c.hull[k]) : nullptr;
            s.field =
                c.shape[k] == ShapeType::HeightField ? &world.heightfields.at(c.hull[k]) : nullptr;
            local_[k] = c.poseInBody[k];
            if (c.body[k] == kWorldBody) {
                s.pose = c.poseInBody[k];
            } else {
                owner_[k] = static_cast<int>(c.body[k].index);
                s.pose = compose(rest_[c.body[k].index], c.poseInBody[k]);
            }
        }
        placed_ = false;
    }
    /// Places the moving colliders at the poses on `bus`.
    void update(const SignalBus& bus) {
        if (shapes_.empty()) {
            return;
        }
        for (std::size_t k = 0; k < shapes_.size(); ++k) {
            const int b = owner_[k];
            if (b >= 0 && poses_[static_cast<std::size_t>(b)].valid()) {
                const std::span<const double> v = bus.readN(poses_[static_cast<std::size_t>(b)]);
                shapes_[k].pose =
                    compose({{v[0], v[1], v[2]}, {v[3], v[4], v[5], v[6]}}, local_[k]);
            }
        }
        caster_.update(shapes_);
        placed_ = true;
    }
    /// Distance (m) to the first collider along unit `dir` from `origin`, skipping the colliders
    /// of body `skip` (the sensor's own: a ray from inside one leaves it first), or +inf.
    [[nodiscard]] double cast(const Vec3& origin, const Vec3& dir, std::uint32_t skip) const {
        if (!placed_) {
            return std::numeric_limits<double>::infinity();
        }
        constexpr int kMaxSkips = 16;    // own colliders passed through, at most
        constexpr double kNudge = 1e-6;  // m past a skipped surface
        double travelled = 0.0;
        Vec3 from = origin;
        for (int i = 0; i < kMaxSkips; ++i) {
            const services::RayHit hit = caster_.castHit(from, dir);
            if (!hit.hit()) {
                return std::numeric_limits<double>::infinity();
            }
            if (owner_[hit.shape] != static_cast<int>(skip)) {
                return travelled + hit.distance;
            }
            const double step = hit.distance + kNudge;
            travelled += step;
            from = {from[0] + dir[0] * step, from[1] + dir[1] * step, from[2] + dir[2] * step};
        }
        return std::numeric_limits<double>::infinity();
    }

private:
    std::vector<Pose> rest_;
    std::vector<SignalId> poses_;
    std::vector<int> owner_;  // body per collider, -1 for the world
    std::vector<Pose> local_;
    std::vector<ShapeCaster::Shape> shapes_;
    ShapeCaster caster_;
    bool placed_{false};
};

/// A per-step Gaussian draw, from the run's seed (invariant 7).
inline double gaussian(const StepContext& context, std::uint32_t stream) {
    random::Stream rng(context.seed, context.stepIndex, stream);
    return rng.normal();
}

/// `value` rounded to a multiple of `resolution` (> 0), else itself.
inline double quantize(double value, double resolution) {
    return resolution > 0.0 && std::isfinite(value) ? std::round(value / resolution) * resolution
                                                    : value;
}

}  // namespace env_common
}  // namespace lagrange::sensors::kinds
