#pragma once

// Mass, centre of mass and inertia from shapes (RFC 0016 §4). Owner: WP-A.
//
// Closed forms for uniform solids (box, cylinder along z, sphere); a link's shapes combine by
// rotating each tensor into the link frame (R I Rᵀ) and moving it to the common COM (parallel
// axis theorem). Capsules: a cylinder and two hemispheres. Meshes (Addendum A): the closed mesh's
// volume integrals (Mirtich 1996), or its convex hull's when it is not closed (MeshShapes.hpp).

#include "lagrange/viewer/builder/Document.hpp"

#include <array>
#include <optional>

namespace lagrange::viewer::builder {

/// Mass properties of a body in a stated frame.
struct MassProperties {
    double mass{0};                   ///< kg
    std::array<double, 3> com{};      ///< m, in the frame stated by the function
    std::array<double, 6> inertia{};  ///< ixx ixy ixz iyy iyz izz, kg·m², about `com`, axes of
                                      ///< that frame (URDF's `<inertia>` order)
};

/// One shape of density `density` (kg/m³) as a uniform solid, in the link frame (the shape's
/// origin applied). Mass 0 for density <= 0 and for a mesh that does not load (MeshShapes.hpp;
/// relative paths against meshBase()).
[[nodiscard]] MassProperties shapeMass(const Shape& shape, double density);

/// The link's mass properties in the link frame, from its solid shapes and
/// `link.inertial.density`, with the overrides applied: a mass override scales the computed
/// tensor by m_override / m_computed; an inertia override replaces the tensor; a com override
/// replaces the COM. nullopt when nothing gives the link a mass (no solid shape, no density and
/// no mass override), which validation reports.
[[nodiscard]] std::optional<MassProperties> linkMass(const Link& link);

/// What the engines need of an inertia tensor (ixx ixy ixz iyy iyz izz, kg·m²).
enum class InertiaCheck {
    Ok,
    NotPositiveDefinite,  ///< some principal moment <= 0 (or not symmetric positive definite)
    TriangleInequality,   ///< principal moments break Ixx + Iyy >= Izz (or a permutation)
};
[[nodiscard]] InertiaCheck checkInertia(const std::array<double, 6>& inertia);

}  // namespace lagrange::viewer::builder
