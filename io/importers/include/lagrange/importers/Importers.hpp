#pragma once

#include "lagrange/module/Expected.hpp"
#include "lagrange/world/Model.hpp"

#include <string>

namespace lagrange::importers {

struct ImportOptions {
    bool addGround{true};  // add the ground plane and slab when the file has no ground plane
    double groundFriction{1.0};
};

/// Reads an SDFormat file (libsdformat 16) into world tables and a scene: the importers'
/// front end for SDFormat -> USD (roadmap 0.2-G3), and what lagrange_server loads directly.
///
/// - A file with a <model> gives one top prim; a <world> gives one per model, with the world's
///   gravity. Nested models are flattened, their links named "outer::inner" with "::" as "_".
/// - Links become bodies (inertia from <inertial>), joints the body tree: revolute (continuous
///   when unlimited), continuous, prismatic, ball and fixed; other types are welded, with a
///   note. A link that is no joint's child floats, unless the model is static.
/// - Collisions: box, sphere, cylinder, capsule and plane (the ground, on static links), with
///   the ODE friction mu. Visuals: those shapes, ellipsoids, and a placeholder for mesh files.
/// - Sensors: imu, lidar and gpu_lidar (alias ray, gpu_ray), as the URDF loader reads them.
/// - Joints named in a model <plugin> (gz-sim's DiffDrive <left_joint>, JointController
///   <joint_name>, ...) are actuated.
///
/// model:// URIs resolve through GZ_SIM_RESOURCE_PATH and the file's directory.
[[nodiscard]] Expected<Model, std::string> importSdf(const std::string& path,
                                                     const ImportOptions& options = {});

/// Reads an MJCF file (MuJoCo 3.12's compiler) into world tables and a scene (roadmap 0.2-G4).
///
/// - Bodies keep MuJoCo's compiled inertia (from <inertial> or the geoms). Joints: free, ball,
///   slide and hinge (continuous unless limited); a body with several joints gets a 1 g helper
///   frame per extra joint. Bodies without joints are welded to their parent.
/// - Geoms with contype or conaffinity collide (box, sphere, capsule, cylinder, and planes on
///   static bodies as the ground); geoms in groups 0-2 with a visible colour are drawn, meshes
///   with their compiled triangles.
/// - Joints driven by an actuator are actuated, with the force range (or the control range
///   times the gear) as the effort limit.
/// - A site with a gyro or an accelerometer becomes an IMU at the physics rate (MJCF sensors
///   have no rate of their own), with the sensors' noise.
[[nodiscard]] Expected<Model, std::string> importMjcf(const std::string& path,
                                                      const ImportOptions& options = {});

enum class Format : std::uint8_t { Unknown, Urdf, Sdf, Mjcf, Usd };

/// The format of a robot or world file: USD by extension, the XML formats by their root element
/// (<robot>, <sdf>, <mujoco>).
[[nodiscard]] Format formatOf(const std::string& path);

}  // namespace lagrange::importers
