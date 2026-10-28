#pragma once

#include "lagrange/world/World.hpp"
#include "lagrange/world/streaming/TileSelector.hpp"

#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace lagrange {

inline constexpr std::uint32_t kNoIndex = 0xFFFF'FFFFU;

/// Triangle mesh for the viewer: float32 positions and normals, uint32 indices.
struct TriMesh {
    std::vector<float> positions;  // x, y, z per vertex
    std::vector<float> normals;
    std::vector<std::uint32_t> indices;
};

[[nodiscard]] TriMesh boxMesh(double sizeX, double sizeY, double sizeZ);
/// A box of half extents `half` about its origin as a convex hull: 8 corners, 12 triangles.
[[nodiscard]] ConvexHull boxHull(const std::array<double, 3>& half);
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
    // A part that turns on its body about `hingeAxis` (unit, in the prim's frame, through its
    // origin) by the angle in the scalar signal `hinge` (rad), such as a folding propeller blade;
    // empty: rigid on its body.
    std::string hinge;
    std::array<double, 3> hingeAxis{0.0, 1.0, 0.0};
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

/// A model ready to simulate and to show: the world tables for the physics, and the scene the
/// viewer draws. Made by the URDF loader (server) and the USD stage loader (core/stage).
struct Model {
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
    /// The stage's tiles (WorldTileAPI_1), left unloaded for streaming; bounds in world ENU.
    std::vector<streaming::TileInfo> tiles;
};

/// The visible ground: a 50 m slab in the floor colour of the mock server, as the last root prim
/// (viewers take the first root prim as the robot). Its collision is the infinite plane that the
/// loaders add as the first collider.
void addGroundPrim(Model& model);

/// Adds `other` to `model`: a second robot, or a world's scenery loaded separately. Indices are
/// renumbered; `other`'s ground (plane colliders on the world and its root prim "ground") is
/// left out, since `model` has one. Its prims go after `model`'s, before the ground slab.
void appendModel(Model& model, const Model& other);

/// How much of a model's static content there is: what a streamed tile is appended after.
struct ModelExtent {
    std::size_t colliders{0};
    std::size_t hulls{0};
    std::size_t prims{0};
    std::size_t meshes{0};
    std::size_t materials{0};
};
[[nodiscard]] ModelExtent extentOf(const Model& model);
/// Drops everything appended after `extent` (the tiles), keeping what came before.
void truncateModel(Model& model, const ModelExtent& extent);

/// Adds a streamed tile's static content (stage::loadStage with ReadOptions::tile, in world ENU)
/// to `model`, moved into the region frame whose origin is `regionOrigin`: its colliders on the
/// world, and its scene prims under `model`'s prim of the same tile path (left in the scene,
/// empty, while the tile was unloaded). Bodies, joints, sensors and schema instances of the
/// tile are not taken (a tile is scenery). Appended at the end, after the ground slab, so that
/// truncateModel() takes it out again. Scene prims whose origin lies in one of `covered`
/// (world ENU boxes, min x, min y, max x, max y: loaded finer tiles over a coarse one) are left
/// out with what is under them, so the finer tiles replace the coarse one where they are.
void appendTile(Model& model, const Model& tile, const std::string& tilePath,
                const std::array<double, 3>& regionOrigin,
                std::span<const std::array<double, 4>> covered = {});

}  // namespace lagrange
