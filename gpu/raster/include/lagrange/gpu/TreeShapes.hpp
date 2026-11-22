#pragma once
// Trees (TreeAPI_1; docs/physics/ground.md "Trees") for the plain mesh pipeline (mesh.slang):
// the viewer's Lit and other raster shadings and the camera sensor's simple look draw each tree
// as a trunk and a crown of leaf clumps, from the same numbers and seed as the realistic view's
// procedural tree (realistic.slang vsTree), so the clumps stand where its leaves are. The crown
// sways with tree/<n>/sway as there (a point at height z moves by sway·(z / centre height)^1.5),
// and a clump a propeller cut into shrinks by the share of its volume cut away
// (RealisticTreeCut), so a cut shows as a bite out of the crown. Invariant 13: one function makes
// the parts for both the viewport and the camera.

#include "lagrange/gpu/Raster.hpp"
#include "lagrange/gpu/Realistic.hpp"

#include <array>
#include <cstdint>
#include <span>
#include <vector>

namespace lagrange::gpu {

/// The meshes the parts are drawn with, to be uploaded after a scene's own meshes: a unit
/// sphere (a clump of leaves) and a unit trunk (radius 1 at z = 0 tapering to kTreeTrunkTaper
/// at z = 1).
inline constexpr std::uint32_t kTreeMeshCount = 2;
inline constexpr float kTreeTrunkTaper = 0.28F;  // realistic.slang trunkRadius: 1 - 0.72 at top
[[nodiscard]] std::span<const MeshSource> treeMeshSources();

/// Clumps of a tree's crown: realistic.slang kTreeClumps.
inline constexpr std::uint32_t kTreeClumpCount = 28;

/// Clump k's centre (the crown's frame: from its centre, tree axes, m) and radius (m), as
/// realistic.slang treeClump draws it. The clump is an ellipsoid of semi-axes r, r and 0.8 r.
[[nodiscard]] std::array<float, 4> treeClump(const RealisticTree& tree, std::uint32_t k);

/// Appends tree `index`'s parts (a trunk, then its clumps) to `out`, as instances of the meshes
/// treeMeshSources() uploaded from `firstMesh` on, with instance id `id`. `cuts` may hold every
/// tree's cut cells; only those of `index` count.
void appendTreeParts(const RealisticTree& tree, std::uint32_t index,
                     std::span<const RealisticTreeCut> cuts, std::uint32_t firstMesh,
                     std::uint32_t id, std::vector<RasterInstance>& out);

}  // namespace lagrange::gpu
