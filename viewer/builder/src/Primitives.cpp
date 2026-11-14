// Stub (RFC 0016 WP0): WP-D replaces this file with the tessellation of
// core/world/include/lagrange/world/Model.hpp (R14).
#include "lagrange/viewer/builder/Primitives.hpp"

namespace lagrange::viewer::builder {

MeshData boxMesh(const std::array<double, 3>& /*size*/) {
    return {};  // not implemented yet (WP-D): nothing to draw
}

MeshData cylinderMesh(double /*radius*/, double /*length*/, int /*segments*/) {
    return {};  // not implemented yet (WP-D)
}

MeshData sphereMesh(double /*radius*/, int /*segments*/, int /*rings*/) {
    return {};  // not implemented yet (WP-D)
}

}  // namespace lagrange::viewer::builder
