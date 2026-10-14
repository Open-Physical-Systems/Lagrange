#pragma once

#include "lagrange/module/Expected.hpp"
#include "lagrange/world/PoseMath.hpp"
#include "lagrange/world/World.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace lagrange::server {

inline constexpr std::uint32_t kNoIndex = 0xFFFF'FFFFU;

/// Triangle mesh for the viewer: float32 positions and normals, uint32 indices.
struct TriMesh {
    std::vector<float> positions;  // x, y, z per vertex
    std::vector<float> normals;
    std::vector<std::uint32_t> indices;
};

[[nodiscard]] TriMesh boxMesh(double sizeX, double sizeY, double sizeZ);
/// A cylinder along z, centred at the origin (the URDF convention).
[[nodiscard]] TriMesh cylinderMesh(double radius, double length, int segments = 32);
[[nodiscard]] TriMesh sphereMesh(double radius, int segments = 32, int rings = 16);

/// One prim of the scene the viewer shows. Its world pose is the pose of `body` composed with
/// `offset`, or `offset` alone when `body` is kNoIndex (static scenery).
struct ScenePrim {
    std::string name;
    std::string path;
    std::uint32_t parent{kNoIndex};
    std::string typeName;  // "Xform", "Mesh"
    std::vector<std::string> apiSchemas;
    std::uint32_t mesh{kNoIndex};
    std::uint32_t material{kNoIndex};
    std::uint32_t body{kNoIndex};
    Pose offset;
    std::array<float, 3> scale{1.0F, 1.0F, 1.0F};
};

struct SceneMaterial {
    std::string name;
    std::array<float, 4> rgba{0.7F, 0.7F, 0.72F, 1.0F};
};

struct Scene {
    std::vector<ScenePrim> prims;
    std::vector<TriMesh> meshes;
    std::vector<SceneMaterial> materials;
};

/// A robot ready to simulate and to show: the world tables for the physics, and the scene.
struct Robot {
    std::string name;
    std::string sourcePath;
    World world;
    Scene scene;
    std::vector<Pose> bodyRestPose;           // world pose of each body with all joints at zero
    std::vector<std::uint32_t> bodyPrim;      // scene prim of each body
    std::vector<std::uint32_t> jointPrim;     // scene prim of each joint's child link
    std::vector<std::uint32_t> sensorPrim;    // scene prim of each sensor
    std::vector<std::string> actuatedJoints;  // joints driven by torque, in file order
    std::vector<std::string> notes;           // what was skipped or approximated, for the user
};

struct LoadOptions {
    double groundFriction{1.0};   // Coulomb coefficient of the ground plane
    double surfaceFriction{1.0};  // of every robot collider (URDF carries no friction)
    double dropHeightM{0.001};    // the robot starts this far above the ground
};

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

}  // namespace lagrange::server
