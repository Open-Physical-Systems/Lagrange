#pragma once

#include "lagrange/gpu/Raster.hpp"
#include "lagrange/gpu/RayCaster.hpp"
#include "lagrange/module/IModule.hpp"
#include "lagrange/world/Model.hpp"
#include "lagrange/world/World.hpp"

#include <array>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace lagrange::sensors {

inline constexpr double kNoHit = std::numeric_limits<double>::infinity();

/// Distance along the unit direction `dir` from `origin` to the first point of a collision
/// shape at `shapePose` (world frame), or kNoHit. Exact for every ShapeType (sizes as in
/// World::Colliders). A ray starting inside a shape reports where it leaves it.
[[nodiscard]] double raycast(ShapeType shape, const std::array<double, 3>& size,
                             const Pose& shapePose, const std::array<double, 3>& origin,
                             const std::array<double, 3>& dir);

/// What cameras draw: every mesh prim of a model's scene, on one GPU device (a software device
/// on a machine without a GPU, sensors.md §6). Made once per run and shared by its cameras.
struct CameraScene {
    struct Drawable {
        std::uint32_t mesh{0};
        std::uint32_t body{kNoIndex};  // kNoIndex: static, `offset` is its world pose
        Pose offset;                   // in the body
        std::array<float, 3> scale{1, 1, 1};
        std::array<float, 4> color{0.8F, 0.8F, 0.8F, 1.0F};
        std::uint32_t prim{0};  // scene prim index: the instance id in sensor/<n>/instance
    };
    std::shared_ptr<gpu::RasterScene> raster;
    std::vector<Drawable> drawables;
};

/// Uploads `model`'s meshes to the device named by `preferredDevice` (DeviceDesc semantics).
/// Returns nullptr with a reason in `error` when there is no usable Vulkan device.
[[nodiscard]] std::shared_ptr<CameraScene> createCameraScene(const Model& model,
                                                             const std::string& preferredDevice,
                                                             std::string& error);

/// One module per sensor of the world (sensors.md §3): its rate is the sensor's update rate,
/// so its signals have that native rate. Domain "sensors", name "<type>_<sensor name>".
///
/// Lidar (raycast level): one ray per beam of its LidarLayout, `samples` columns from min_angle
/// to max_angle about the sensor's z, in `vertical_samples` rings (1: a 2D scanner in the xy
/// plane), against every collider; Gaussian range noise; no return beyond max_range or below
/// min_range (+inf). Output sensor/<n>/ranges, ScalarArray of samples x vertical_samples, m,
/// ring after ring (LidarLayout).
///
/// IMU: specific force and angular rate in the sensor frame from the body's proper
/// acceleration and twist, plus white Gaussian noise (accel_noise, gyro_noise); the lever arm
/// between body origin and sensor is neglected. Outputs sensor/<n>/specific_force (Vec3 m/s²),
/// angular_rate (Vec3 rad/s), orientation (Quat, ground truth, world_local).
///
/// Camera (raster level): a pinhole of `width` x `height` pixels and `horizontal_fov`, square
/// pixels, principal point at the image centre, looking along the sensor's x (z up, the URDF
/// convention; the image is in the optical frame, z forward, x right, y down). Draws
/// `cameraScene` with the shared mesh shader (gpu/kernels/camera/mesh.slang). Outputs, as
/// Image buffers: sensor/<n>/image (rgba8, display-encoded), depth (r32f, m along the optical
/// axis, +inf where nothing is nearer than far_clip) and instance (r32u, the scene prim index,
/// 0xFFFFFFFF for none). Its period is rounded to whole milliseconds (30 Hz runs at 33 ms), so
/// that it lines up with the other modules' steps. Not deterministic (a GPU path, invariant 7).
///
/// Noise comes from Philox keyed by run, sensor, step and beam, so it is deterministic.
///
/// With `rayContext`, lidars cast on the GPU (Tier 1, gpu/kernels/lidar/bvh_traverse.slang) in
/// a frame centred on the sensor, in float; range limits and noise are applied afterwards
/// exactly as on the CPU. Such a lidar is not deterministic (invariant 7: the CPU path is the
/// reference) and needs a module config of Tier 1 or more.
///
/// `groundTruthCameras`: cameras as their ground-truth substitute while the other sensors are
/// simulated (sensors.md §5, "simulate the lidar, skip the cameras").
[[nodiscard]] std::vector<std::unique_ptr<IModule>> createSensorModules(
    const World& world, std::shared_ptr<gpu::RayContext> rayContext = nullptr,
    std::shared_ptr<const CameraScene> cameraScene = nullptr, bool groundTruthCameras = false);

/// The sensors domain's "ground_truth" substitute (ideal kind, CLAUDE.md §11.2): one module per
/// sensor with the same signals and rates, but no rendering. A lidar reports no returns (+inf);
/// an IMU reports the true specific force and angular rate, without noise; a camera publishes
/// empty images that carry their size and encoding.
[[nodiscard]] std::vector<std::unique_ptr<IModule>> createSensorSubstitutes(const World& world);

}  // namespace lagrange::sensors
