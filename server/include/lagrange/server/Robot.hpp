#pragma once

#include "lagrange/module/Expected.hpp"
#include "lagrange/world/Model.hpp"
#include "lagrange/world/PoseMath.hpp"
#include "lagrange/world/World.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace lagrange::server {

using lagrange::boxMesh;
using lagrange::cylinderMesh;
using lagrange::kNoIndex;
using lagrange::Scene;
using lagrange::SceneMaterial;
using lagrange::ScenePrim;
using lagrange::sphereMesh;
using lagrange::TriMesh;
using Robot = lagrange::Model;

struct LoadOptions {
    double groundFriction{1.0};   // Coulomb coefficient of the ground plane
    double surfaceFriction{1.0};  // of every robot collider (URDF carries no friction)
    double dropHeightM{0.001};    // the robot starts this far above the ground
    /// Where a free robot's root link starts, in world ENU: x, y and heading as given, and its
    /// lowest collision shape dropHeightM above the height z (the ground under it).
    Pose spawn;
};

/// Distance from the region origin beyond which recentre() moves it (huge-worlds.md §1.2).
inline constexpr double kRebaseDistanceM = 1000.0;

/// Reads a URDF file into world tables and a scene (the v0.2 path until the USD importer of
/// roadmap 0.2-G1: open question Q4).
///
/// Links become bodies (inertia rotated into the link frame), joints the body tree, collision
/// boxes, cylinders, spheres and capsules the colliders, visuals the scene. A root link named
/// "world" fixes the robot to the ground; otherwise the root gets a free joint and the robot is
/// placed with its lowest collision shape just above a ground plane. Joints with a command
/// interface in the file's <ros2_control> block are actuated.
[[nodiscard]] Expected<Robot, std::string> loadUrdf(const std::string& path,
                                                    const LoadOptions& options = {});

/// Adds another URDF to a loaded robot: a second robot, or static scenery (a root link named
/// "world", as in assets/worlds). Its prims go under their own top prim, after the others.
///
/// Sensors come from the Gazebo <sensor> blocks of the file (<gazebo reference="link">): types
/// imu, ray, gpu_ray, lidar and gpu_lidar, with update_rate, pose, horizontal scan samples and
/// angles, range min/max and the Gaussian noise stddev.
[[nodiscard]] Expected<void, std::string> addUrdf(Robot& robot, const std::string& path,
                                                  const LoadOptions& options = {});

/// Moves the region-local frame (the floating origin, huge-worlds.md §1.2) to the first body
/// when it is farther than `rebaseDistanceM` from it, so that physics, sensors, signals and the
/// viewer work with small coordinates however far out the world was authored. Every position
/// held in the world frame (root bodies, world colliders, rest poses, scenery prims; unbounded
/// planes and the ground slab only along their normal) loses the same whole-metre shift, which is
/// exact in double, and world.region records it. Call it once, after the robot and the world are
/// loaded. Returns the shift (zeros when nothing moved).
std::array<double, 3> recentre(Robot& robot, double rebaseDistanceM = kRebaseDistanceM);

/// A robot from a URDF, an SDFormat or MJCF file (lagrange_importers, told apart by their root
/// element) or a USD stage (.usd, .usda, .usdc: lagrange_stage), with the ground added unless
/// the file has a ground plane of its own.
[[nodiscard]] Expected<Robot, std::string> loadModel(const std::string& path,
                                                     const LoadOptions& options = {});

/// Adds a second file to a loaded robot (the world's scenery, or another robot), in any of the
/// formats loadModel reads. Its own ground plane is left out.
[[nodiscard]] Expected<void, std::string> addModel(Robot& robot, const std::string& path,
                                                   const LoadOptions& options = {});

/// Whether a path names a USD layer.
[[nodiscard]] bool isUsd(const std::string& path);

}  // namespace lagrange::server
