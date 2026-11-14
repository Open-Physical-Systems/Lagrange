// Stub (RFC 0016 WP0): WP-A replaces this file with the closed forms of RFC 0016 §4.
#include "lagrange/viewer/builder/MassProperties.hpp"

namespace lagrange::viewer::builder {

MassProperties shapeMass(const Shape& /*shape*/, double /*density*/) {
    return {};  // not implemented yet (WP-A): no mass
}

std::optional<MassProperties> linkMass(const Link& /*link*/) {
    return std::nullopt;  // not implemented yet (WP-A)
}

InertiaCheck checkInertia(const std::array<double, 6>& /*inertia*/) {
    // Not implemented yet (WP-A). Never claims a tensor is fine without checking it.
    return InertiaCheck::NotPositiveDefinite;
}

}  // namespace lagrange::viewer::builder
