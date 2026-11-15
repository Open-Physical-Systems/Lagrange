#pragma once

#include "lagrange/world/World.hpp"
#include "lagrange/world/streaming/TileSelector.hpp"

#include <array>
#include <cstdint>
#include <optional>
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
/// The convex hull of a mesh's vertices, scaled, in the mesh's frame. More than kMaxHullPoints
/// vertices are first cut down to the farthest ones in that many directions. Closed, wound
/// outward, a sphere (V - E + F = 2), and convex to within 1e-6 of its size (a point that near a
/// face counts as on it; rarely, for a nearly flat set, a few times that), the same bits for the
/// same vertices. Empty when the vertices are flat or fewer than four: no volume to collide
/// with; when a vertex (or the scale) is NaN or infinite; or, never seen, when no hull closes
/// round them within a bounded amount of work (about a second).
inline constexpr std::size_t kMaxHullPoints = 256;
[[nodiscard]] std::optional<ConvexHull> convexHullOf(const TriMesh& mesh,
                                                     const std::array<double, 3>& scale = {1.0, 1.0,
                                                                                           1.0});
/// An ellipsoid of these radii as a convex hull, about 1 % smaller than the real shape.
[[nodiscard]] ConvexHull ellipsoidHull(const std::array<double, 3>& radii);
/// How much of `hull` the closed mesh fills, 0 to 1: well under 1 is a concave mesh, whose hull
/// also blocks what fits in its hollows. Meaningless for an open mesh.
[[nodiscard]] double hullFill(const TriMesh& mesh, const std::array<double, 3>& scale,
                              const ConvexHull& hull);
/// Below this fill a mesh is called concave (illustrative: a solid part with some detail is above
/// it, a room, a U or two parts in one file are below it).
inline constexpr double kConcaveFill = 0.6;
/// A triangle surface as a flat-shaded mesh, to draw it.
[[nodiscard]] TriMesh flatMesh(const ConvexHull& surface);
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

/// How a surface looks: glTF's metallic-roughness model, which UsdPreviewSurface and SDFormat's
/// <pbr> also use. Roughness 0 is a mirror, 1 fully matte.
struct SurfaceLook {
    float metallic{0.0F};
    float roughness{0.6F};
    std::array<float, 3> emissive{0.0F, 0.0F, 0.0F};  // linear RGB, added to what is reflected
    friend bool operator==(const SurfaceLook&, const SurfaceLook&) = default;
};

/// The look of a named kind of surface: "steel", "rubber", "glass", ... Illustrative values in the
/// ranges of the usual PBR material charts, not measurements. False for a name it does not know.
[[nodiscard]] bool surfaceLookPreset(std::string_view name, SurfaceLook& out);

struct SceneMaterial {
    std::string name;
    std::array<float, 4> rgba{0.7F, 0.7F, 0.72F, 1.0F};
    /// The look the file gave. Empty: from a word of the material's name that names a kind of
    /// surface ("rubber", "front_steel_plate"), else the default.
    std::optional<SurfaceLook> look{};
};

/// What is drawn for a material: its look, or the one its name suggests.
[[nodiscard]] SurfaceLook lookOf(const SceneMaterial& material);

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
    /// False for a world with its own ground (terrain tiles): the infinite ground plane of the
    /// model it joins is taken away (appendModel). From the layer's customLayerData
    /// `lagrangeGroundPlane`.
    bool groundPlane{true};
    /// True while the first collider is Lagrange's own ground plane (addGroundPlane, the URDF
    /// loader's and the empty model's): the only plane removeGroundPlane takes away. A file's own
    /// planes stay.
    bool lagrangePlane{false};
};

/// The visible ground: a 50 m slab in the floor colour of the mock server, as the last root prim
/// (viewers take the first root prim as the robot). Its collision is the infinite plane that the
/// loaders add as the first collider.
void addGroundPrim(Model& model);

/// The narrowest a level box may be, in x and in y (m), to count as a file's own floor.
inline constexpr double kFloorMinWidthM = 10.0;

/// How far (m) a level slab's top may be above what stands on it, or above the lowest other
/// static geometry, and still count as the floor (a robot's wheels just touching it, a wall's
/// foot just sunk into it).
inline constexpr double kFloorTopToleranceM = 0.1;

/// Whether the model has a floor of its own (one ground per run, docs/guide/cli.md): static
/// geometry that is a plane facing up, a height field, or a level box (turned about z only) at
/// least kFloorMinWidthM wide in x and in y and no thicker than a quarter of its narrower width
/// (a slab, not a building) whose top is the lowest ground there is: at most
/// kFloorTopToleranceM above z = 0 (where Lagrange's ground is and robots spawn), above
/// `standing` (the lowest point of what must stand on it; by default the lowest point of the
/// model's moving bodies) and above the bottom of every other static collider that is not such
/// a slab. So a building's roof, a mezzanine or a bridge deck is never taken for the floor, even
/// with a robot spawned on it (it would fall forever off its edge). Static: on the
/// world, or on a body welded to the world by fixed joints (an MJCF worldbody geom, an
/// SDFormat static model, a URDF world's links). With `countPlanes` false, planes are left out
/// (Lagrange's own ground is a plane on the world).
[[nodiscard]] bool hasOwnFloor(const Model& model, bool countPlanes = true,
                               std::optional<double> standing = std::nullopt);

/// The lowest point (world z, m) of the model's moving bodies at rest: their colliders' bottoms,
/// or their origins when they have no colliders; nothing when every body is static.
[[nodiscard]] std::optional<double> lowestMovingPoint(const Model& model);

/// Lagrange's ground: the infinite plane facing up at z = 0 as the first collider, and the slab
/// (addGroundPrim). The loaders call it for a file that has no floor of its own (hasOwnFloor).
void addGroundPlane(Model& model, double friction);

/// Takes Lagrange's ground away: its plane (the first collider while `lagrangePlane`; a file's
/// own planes stay) and the root prim "ground" without a body (the slab), renumbering the scene
/// prims after it.
void removeGroundPlane(Model& model);

/// Adds `other` to `model`: a second robot, or a world's scenery loaded separately. Indices are
/// renumbered; `other`'s ground (plane colliders on the world and its root prim "ground") is
/// left out, since `model` has one. Its prims go after `model`'s, before the ground slab. When
/// `other` has groundPlane false, or a floor of its own other than a plane (hasOwnFloor: a level
/// slab, a height field), `model`'s plane and ground slab are taken away instead.
void appendModel(Model& model, const Model& other);

/// Makes a robot's names unique before it joins another model (a second robot of the same
/// kind): bodies, joints and sensors get `prefix` + "_" + their name (signals follow:
/// body/<prefix>_base_link/pose), its top prim and every path under it take `prefix` as the
/// top prim's name, and with `passive` its joints are no longer actuated (no controller drives
/// them). `prefix` must be a valid prim and signal name.
void prefixModel(Model& model, const std::string& prefix, bool passive);

/// How much of a model's static content there is: what a streamed tile is appended after.
struct ModelExtent {
    std::size_t colliders{0};
    std::size_t hulls{0};
    std::size_t heightfields{0};
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
