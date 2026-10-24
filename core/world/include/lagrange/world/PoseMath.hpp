#pragma once

#include "lagrange/world/World.hpp"

#include <array>
#include <cmath>
#include <vector>

namespace lagrange {

/// The frame of a USD camera (looking along -z, y up) in a sensor's frame (looking along x, z
/// up, the URDF convention): a camera sensor's mount is its UsdGeomCamera's pose composed with
/// the conjugate of this.
inline constexpr std::array<double, 4> kSensorFromUsdCamera = {0.5, 0.5, -0.5, -0.5};

/// A camera's optical frame (z forward, x right, y down; ROS REP 103 "_optical") in its sensor
/// frame (x forward, z up): roll -90°, then yaw -90°.
inline constexpr std::array<double, 4> kSensorFromOptical = {0.5, -0.5, 0.5, -0.5};

/// Quaternion product a b, (w, x, y, z).
[[nodiscard]] inline std::array<double, 4> multiply(const std::array<double, 4>& a,
                                                    const std::array<double, 4>& b) {
    return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}

/// v rotated by the unit quaternion q.
[[nodiscard]] inline std::array<double, 3> rotate(const std::array<double, 4>& q,
                                                  const std::array<double, 3>& v) {
    // v' = v + 2 w (u x v) + 2 u x (u x v), with u the vector part of q.
    const std::array<double, 3> u{q[1], q[2], q[3]};
    const std::array<double, 3> t{2.0 * (u[1] * v[2] - u[2] * v[1]),
                                  2.0 * (u[2] * v[0] - u[0] * v[2]),
                                  2.0 * (u[0] * v[1] - u[1] * v[0])};
    return {v[0] + q[0] * t[0] + (u[1] * t[2] - u[2] * t[1]),
            v[1] + q[0] * t[1] + (u[2] * t[0] - u[0] * t[2]),
            v[2] + q[0] * t[2] + (u[0] * t[1] - u[1] * t[0])};
}

/// The conjugate: the inverse rotation of a unit quaternion.
[[nodiscard]] inline std::array<double, 4> conjugate(const std::array<double, 4>& q) {
    return {q[0], -q[1], -q[2], -q[3]};
}

/// a then b: the pose of b's frame in a's parent frame.
[[nodiscard]] inline Pose compose(const Pose& a, const Pose& b) {
    const std::array<double, 3> moved = rotate(a.orientation, b.position);
    return {{a.position[0] + moved[0], a.position[1] + moved[1], a.position[2] + moved[2]},
            multiply(a.orientation, b.orientation)};
}

/// World pose of every body with all joints at zero (composing poseInParent up the tree).
[[nodiscard]] inline std::vector<Pose> restPoses(const World& world) {
    const World::Bodies& bodies = world.bodies;
    std::vector<Pose> out(bodies.names.size());
    std::vector<bool> done(bodies.names.size(), false);
    for (bool progress = true; progress;) {
        progress = false;
        for (std::size_t b = 0; b < out.size(); ++b) {
            if (done[b]) {
                continue;
            }
            const BodyId parent = bodies.parent[b];
            if (parent == kWorldBody) {
                out[b] = bodies.poseInParent[b];
            } else if (parent.index < out.size() && done[parent.index]) {
                out[b] = compose(out[parent.index], bodies.poseInParent[b]);
            } else {
                continue;
            }
            done[b] = true;
            progress = true;
        }
    }
    return out;
}

/// Bodies that move: those with a non-fixed joint to their parent, and everything below them.
/// Only these have pose signals; the others stay at their rest pose.
[[nodiscard]] inline std::vector<bool> movingBodies(const World& world) {
    const World::Bodies& bodies = world.bodies;
    std::vector<bool> jointed(bodies.names.size(), false);
    for (std::size_t j = 0; j < world.joints.names.size(); ++j) {
        if (world.joints.type[j] != JointType::Fixed) {
            jointed.at(world.joints.child[j].index) = true;
        }
    }
    std::vector<bool> moving(bodies.names.size(), false);
    for (bool changed = true; changed;) {
        changed = false;
        for (std::size_t b = 0; b < bodies.names.size(); ++b) {
            const BodyId parent = bodies.parent[b];
            const bool value = jointed[b] || (parent != kWorldBody &&
                                              parent.index < moving.size() && moving[parent.index]);
            if (value && !moving[b]) {
                moving[b] = true;
                changed = true;
            }
        }
    }
    return moving;
}

}  // namespace lagrange
