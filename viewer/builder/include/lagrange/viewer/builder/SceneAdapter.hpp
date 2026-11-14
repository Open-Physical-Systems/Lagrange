#pragma once

// The document as a SceneFrame for the viewer's own SceneRenderer (plan decisions 1 and 2).
// Owner: WP-D. The builder draws only on the Lit pipeline; no new pipeline, no new shader.

#include "lagrange/viewer/SceneStore.hpp"
#include "lagrange/viewer/builder/Document.hpp"
#include "lagrange/viewer/builder/Kinematics.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace lagrange::viewer::builder {

/// What a document edit produces: the meshes, materials, prims and colliders. Rebuilt only on
/// a document edit (History::revision changes), never while a slider moves (R15).
struct BuiltScene {
    std::shared_ptr<const SceneStatics> statics;
    /// Per prim of statics->prims (same index): the name of the link or sensor it draws, so a
    /// pick (pickPrim) selects the document item.
    std::vector<std::string> primItem;
    /// Per prim: the index into Document::links of the link it moves with (-1: none, at the
    /// identity). poseScene places each prim at its link's frame.
    std::vector<int> primLink;
    /// Per prim: its pose in its link's frame (a shape's origin, a sensor's mount; identity for
    /// the link's own prim). Added by WP-D: a shape's origin cannot live in the mesh without
    /// loosening pickPrim's box test for turned shapes.
    std::vector<Frame> primOffset;
    /// Per prim: the index into Document::sensors it draws (-1: a link or shape prim).
    std::vector<int> primSensor;
    /// Per prim: the index into its link's shapes it draws (-1: the link's own prim or a
    /// sensor). Added by Addendum A, so a pick can select one shape (a collision-only shape
    /// placed by hand, choice 4).
    std::vector<int> primShape;
};

/// What buildStatics draws besides the document's visuals (Addendum A).
struct BuildOptions {
    /// Every collision shape translucent (the view's collision toggle): primitives as
    /// themselves, a mesh as the convex hull the server collides. Collision-only shapes are
    /// drawn translucent either way.
    bool collision{false};
};

/// Default colour of a shape with no rgba (Shape::rgba all zero): a neutral light blue-grey,
/// linear 0..1. The colour of the object, not a UI token.
inline constexpr std::array<float, 4> kDefaultShapeColor{0.62F, 0.68F, 0.78F, 1.0F};
/// Colour of collision geometry drawn translucent (Addendum A): a cyan that reads against the
/// default shape colour, at 0.35 alpha. The colour of an object, not a UI token.
inline constexpr std::array<float, 4> kCollisionColor{0.25F, 0.70F, 0.90F, 0.35F};
/// Edge of the cube drawn for a sensor's mount (m), so a sensor can be seen and picked. A view
/// marker only: it has no mass and does not collide.
inline constexpr double kSensorMarkerSize = 0.03;

/// One prim per link, a child prim per visual shape (MeshData from Primitives.hpp; a mesh file
/// one per material, read by lagrange_meshes, scale baked in), one collider per collision shape
/// (a mesh's is its server hull), and the translucent collision prims of `options` last
/// (typeName "Collision"); `generation` goes into SceneStatics::generation and must grow with
/// every rebuild, so the renderer uploads the meshes again. Relative mesh paths are read
/// against meshBase() (MeshShapes.hpp).
[[nodiscard]] BuiltScene buildStatics(const Document& document, std::uint64_t generation,
                                      const BuildOptions& options = {});

/// The frame to draw: `scene.statics` unchanged (same pointer), transforms from `links`
/// (linkFrames, in the root link's frame; the root sits at the world origin). Changing only
/// this per preview change keeps meshes on the GPU (plan decision 2).
[[nodiscard]] std::shared_ptr<const SceneFrame> poseScene(const BuiltScene& scene,
                                                          const std::vector<Frame>& links);

/// Each link's frame for the joint preview (in the root link's frame), in document order.
/// linkFrames (WP-A) when its tree is ok; until WP-A lands (its buildTree is a stub), a local
/// forward kinematics over the same rules: parent before child, mimic joints follow their
/// leader, missing joints at 0, links no joint reaches at the identity. Added by WP-D.
[[nodiscard]] std::vector<Frame> previewFrames(const Document& document, const Preview& preview);

/// The index of the prim that draws `item` (a link's own prim, or a sensor's), or kNone.
[[nodiscard]] std::uint32_t primOf(const BuiltScene& scene, std::string_view item);

}  // namespace lagrange::viewer::builder
