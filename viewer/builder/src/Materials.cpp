// Stub (RFC 0016 WP0): WP-A fills the table from the thermal presets (plan §6).
#include "lagrange/viewer/builder/Materials.hpp"

namespace lagrange::viewer::builder {

std::span<const DensityPreset> densityPresets() {
    return {};  // not implemented yet (WP-A): the panel offers "custom" only
}

}  // namespace lagrange::viewer::builder
