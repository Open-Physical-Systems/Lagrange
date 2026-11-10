#pragma once

#include <optional>
#include <string_view>

namespace lagrange::thermal {

/// A solid's thermal properties near 300 K (RFC 0012). Orthotropic when conductivityAlong is
/// set: `conductivityAlong` along the material's axis, `conductivity` in the two directions
/// across it (a circuit board: in-plane `conductivity`, through-plane `conductivityAlong` with
/// the axis normal to the board).
struct ThermalMaterial {
    double conductivity{0.0};                 // W/(m·K)
    double density{0.0};                      // kg/m³
    double specificHeat{0.0};                 // J/(kg·K)
    double emissivity{0.0};                   // 1
    std::optional<double> conductivityAlong;  // W/(m·K), along the axis
};

/// The preset of that name (docs/physics/heat-conduction.md lists them with their sources), or
/// nothing: aluminium_6061, aluminium_cast, copper, steel_304, fr4, pla, abs, petg, nylon_pa12,
/// polycarbonate, cfrp_epoxy, glass, silicone, thermal_pad.
[[nodiscard]] std::optional<ThermalMaterial> thermalMaterialPreset(std::string_view name);

}  // namespace lagrange::thermal
