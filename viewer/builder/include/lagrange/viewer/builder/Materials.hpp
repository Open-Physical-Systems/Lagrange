#pragma once

// Density presets for the Add panel's material menu (RFC 0016 open question 9, plan §6).
// Owner: WP-A. The names and numbers are those of the thermal presets of RFC 0012
// (modules/thermal/conduction/src/Materials.cpp), copied because the viewer cannot link
// lagrange_thermal (invariant 14); a check compares the two tables. Every value stays
// TODO(verify) until its primary source is read.

#include <span>
#include <string_view>

namespace lagrange::viewer::builder {

struct DensityPreset {
    std::string_view name;     ///< "aluminium_6061", the thermal preset's name
    double density;            ///< kg/m³
    std::string_view source;   ///< as cited, for the menu's tooltip
    std::string_view label{};  ///< as the Add panel shows it ("PLA", reference.png)
};

/// In menu order. "custom" is not a preset: the panel asks for a number.
[[nodiscard]] std::span<const DensityPreset> densityPresets();

}  // namespace lagrange::viewer::builder
