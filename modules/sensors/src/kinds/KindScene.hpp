#pragma once
// Private to lagrange_sensors: what the ray-casting kinds (cliff, safety_scanner) share. Every
// collider placed at the poses of the bodies that carry it, read as declared inputs (invariant
// 1), and cast on the CPU with ShapeCaster: the same exact, deterministic tests as the Tier 0
// lidar (sensors.md §6). These kinds cast a few hundred rays at most, so the CPU is the tier
// they need on every machine.

#include "KindModule.hpp"
#include "lagrange/sensors/ShapeCaster.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <array>
#include <span>
#include <vector>

namespace lagrange::sensors::kinds {
// Its own inline namespace: the kinds' private headers may name helpers alike (ODR).
inline namespace amr_scene {

/// A body's pose: its pose input when it moves, else its rest pose.
class BodyPoses {
public:
    /// Declares the pose input of body `b` when it moves (once).
    void watch(const World& world, SignalDeclarer& d, std::uint32_t b) {
        if (rest_.empty()) {
            rest_ = restPoses(world);
            moving_ = movingBodies(world);
            poses_.assign(rest_.size(), SignalId{});
        }
        if (b < moving_.size() && moving_[b] && !poses_[b].valid()) {
            poses_[b] = d.input(kindSpec("body/" + world.bodies.names[b] + "/pose",
                                         SignalType::Pose, units::m, "world_local"));
        }
    }
    [[nodiscard]] bool moves(std::uint32_t b) const { return poses_.at(b).valid(); }
    [[nodiscard]] Pose rest(std::uint32_t b) const { return rest_.at(b); }
    [[nodiscard]] Pose at(const SignalBus& bus, std::uint32_t b) const {
        if (!poses_.at(b).valid()) {
            return rest_.at(b);
        }
        const std::span<const double> v = bus.readN(poses_[b]);
        return {{v[0], v[1], v[2]}, {v[3], v[4], v[5], v[6]}};
    }

private:
    std::vector<Pose> rest_;
    std::vector<bool> moving_;
    std::vector<SignalId> poses_;
};

/// The colliders of a world as a ray-casting scene, optionally without one body's own.
class KindScene {
public:
    /// Declares the pose inputs of every moving body with a collider and of `sensorBody`.
    /// `skipBody`: a body whose colliders the rays pass through (the sensor's own housing), or
    /// kNone.
    static constexpr std::uint32_t kNone = 0xFFFF'FFFFU;
    void declare(const World& world, SignalDeclarer& d, std::uint32_t sensorBody,
                 std::uint32_t skipBody) {
        skip_ = skipBody;
        poses_.watch(world, d, sensorBody);
        for (const BodyId owner : world.colliders.body) {
            if (owner != kWorldBody && owner.index != skip_) {
                poses_.watch(world, d, owner.index);
            }
        }
    }

    void init(const World& world) {
        shapes_.clear();
        owner_.clear();
        local_.clear();
        const World::Colliders& c = world.colliders;
        for (std::size_t k = 0; k < c.body.size(); ++k) {
            const BodyId owner = c.body[k];
            if (owner != kWorldBody && owner.index == skip_) {
                continue;
            }
            ShapeCaster::Shape shape;
            shape.type = c.shape[k];
            shape.size = c.size[k];
            shape.hull = c.shape[k] == ShapeType::ConvexHull ? &world.hulls.at(c.hull[k]) : nullptr;
            shape.field =
                c.shape[k] == ShapeType::HeightField ? &world.heightfields.at(c.hull[k]) : nullptr;
            if (owner == kWorldBody) {
                shape.pose = c.poseInBody[k];
                owner_.push_back(-1);
            } else if (!poses_.moves(owner.index)) {
                shape.pose = compose(poses_.rest(owner.index), c.poseInBody[k]);
                owner_.push_back(-1);
            } else {
                owner_.push_back(static_cast<int>(owner.index));
            }
            local_.push_back(c.poseInBody[k]);
            shapes_.push_back(shape);
        }
        placed_ = false;
    }

    /// Places the moving shapes at the poses on the bus; the first call builds the hierarchy.
    void place(const SignalBus& bus) {
        bool moved = !placed_;
        for (std::size_t k = 0; k < shapes_.size(); ++k) {
            if (owner_[k] >= 0) {
                shapes_[k].pose =
                    compose(poses_.at(bus, static_cast<std::uint32_t>(owner_[k])), local_[k]);
                moved = true;
            }
        }
        if (moved) {
            caster_.update(shapes_);
            placed_ = true;
        }
    }

    /// Distance along the unit `dir` to the nearest shape, or kNoHit (+inf).
    [[nodiscard]] double cast(const std::array<double, 3>& origin,
                              const std::array<double, 3>& dir) const {
        return caster_.cast(origin, dir);
    }

    [[nodiscard]] const BodyPoses& poses() const { return poses_; }

private:
    BodyPoses poses_;
    std::uint32_t skip_{kNone};
    std::vector<ShapeCaster::Shape> shapes_;
    std::vector<int> owner_;  // body index of a moving shape, -1 for fixed ones
    std::vector<Pose> local_;
    ShapeCaster caster_;
    bool placed_{false};
};

/// The pose of `p` seen from frame `frame`: frame⁻¹ ∘ p.
[[nodiscard]] inline Pose relative(const Pose& frame, const Pose& p) {
    const std::array<double, 4> inv = conjugate(frame.orientation);
    const std::array<double, 3> d{p.position[0] - frame.position[0],
                                  p.position[1] - frame.position[1],
                                  p.position[2] - frame.position[2]};
    return {rotate(inv, d), multiply(inv, p.orientation)};
}

/// Yaw (rad) of a unit quaternion (w, x, y, z), ZYX convention.
[[nodiscard]] inline double yawOf(const std::array<double, 4>& q) {
    return std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
}

/// The joint named `name` (World::Joints::names), or the joint whose child is `body` when the
/// name is empty; kNone when there is none.
[[nodiscard]] inline std::uint32_t findJoint(const World& world, const std::string& name,
                                             std::uint32_t body) {
    for (std::uint32_t j = 0; j < world.joints.names.size(); ++j) {
        if (name.empty() ? world.joints.child[j].index == body : world.joints.names[j] == name) {
            return j;
        }
    }
    return KindScene::kNone;
}

}  // namespace amr_scene
}  // namespace lagrange::sensors::kinds
