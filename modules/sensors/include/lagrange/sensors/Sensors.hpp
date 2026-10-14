#pragma once

#include "lagrange/module/IModule.hpp"
#include "lagrange/world/World.hpp"

#include <array>
#include <limits>
#include <memory>
#include <vector>

namespace lagrange::sensors {

inline constexpr double kNoHit = std::numeric_limits<double>::infinity();

/// Distance along the unit direction `dir` from `origin` to the first point of a collision
/// shape at `shapePose` (world frame), or kNoHit. Exact for every ShapeType (sizes as in
/// World::Colliders). A ray starting inside a shape reports where it leaves it.
[[nodiscard]] double raycast(ShapeType shape, const std::array<double, 3>& size,
                             const Pose& shapePose, const std::array<double, 3>& origin,
                             const std::array<double, 3>& dir);

/// One module per sensor of the world (sensors.md §3): its rate is the sensor's update rate,
/// so its signals have that native rate. Domain "sensors", name "<type>_<sensor name>".
///
/// Lidar (2D, raycast level): one ray per sample from min_angle to max_angle in the sensor's xy
/// plane, against every collider; Gaussian range noise; no return beyond max_range or below
/// min_range (+inf). Output sensor/<n>/ranges, ScalarArray of `samples`, m.
///
/// IMU: specific force and angular rate in the sensor frame from the body's proper
/// acceleration and twist, plus white Gaussian noise (accel_noise, gyro_noise); the lever arm
/// between body origin and sensor is neglected. Outputs sensor/<n>/specific_force (Vec3 m/s²),
/// angular_rate (Vec3 rad/s), orientation (Quat, ground truth, world_local).
///
/// Noise comes from Philox keyed by run, sensor, step and beam, so it is deterministic.
[[nodiscard]] std::vector<std::unique_ptr<IModule>> createSensorModules(const World& world);

}  // namespace lagrange::sensors
