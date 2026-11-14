#pragma once

// Triangle meshes of the primitive shapes, for the Builder view (plan decision 1). Owner: WP-D.
// The same tessellation as core/world/include/lagrange/world/Model.hpp (cylinderMesh 32
// segments, sphereMesh 32 segments × 16 rings), copied because the viewer may not link
// lagrange_core (R14), so the builder and the tested robot look the same.

#include "lagrange/viewer/SceneStore.hpp"

#include <array>

namespace lagrange::viewer::builder {

/// Centred on the origin. Sizes in m (box: full edge lengths x y z; cylinder: along z).
/// Positions and normals in the mesh frame, with the aabb filled.
[[nodiscard]] MeshData boxMesh(const std::array<double, 3>& size);
[[nodiscard]] MeshData cylinderMesh(double radius, double length, int segments = 32);
[[nodiscard]] MeshData sphereMesh(double radius, int segments = 32, int rings = 16);

}  // namespace lagrange::viewer::builder
