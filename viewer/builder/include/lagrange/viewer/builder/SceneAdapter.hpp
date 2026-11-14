#pragma once

// The document as a SceneFrame for the viewer's own SceneRenderer (plan decisions 1 and 2).
// Owner: WP-D. The builder draws only on the Lit pipeline; no new pipeline, no new shader.

#include "lagrange/viewer/SceneStore.hpp"
#include "lagrange/viewer/builder/Document.hpp"
#include "lagrange/viewer/builder/Kinematics.hpp"

#include <cstdint>
#include <memory>
#include <string>
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
};

/// One prim per link, a child prim per visual shape (MeshData from Primitives.hpp), one
/// collider per collision shape; `generation` goes into SceneStatics::generation and must grow
/// with every rebuild, so the renderer uploads the meshes again.
[[nodiscard]] BuiltScene buildStatics(const Document& document, std::uint64_t generation);

/// The frame to draw: `scene.statics` unchanged (same pointer), transforms from `links`
/// (linkFrames, in the root link's frame; the root sits at the world origin). Changing only
/// this per preview change keeps meshes on the GPU (plan decision 2).
[[nodiscard]] std::shared_ptr<const SceneFrame> poseScene(const BuiltScene& scene,
                                                          const std::vector<Frame>& links);

}  // namespace lagrange::viewer::builder
