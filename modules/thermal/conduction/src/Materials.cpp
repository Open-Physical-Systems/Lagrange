// Thermal material presets (RFC 0012), near 300 K. Every row names its source; a value marked
// TODO(verify) was not read from the primary source and should be checked against it.
#include "lagrange/thermal/Materials.hpp"

#include <array>

namespace lagrange::thermal {

namespace {

struct Row {
    std::string_view name;
    ThermalMaterial material;
};

// k W/(m·K), ρ kg/m³, c_p J/(kg·K), ε, k along the axis (orthotropic rows).
const std::array<Row, 14> kPresets{{
    // Aluminium 6061-T6: ρ, c_p from the ASM/MatWeb data sheet; k 167 (MatWeb, as quoted by
    // nTop's material library, TODO(verify) on MatWeb); ε bare, rolled (Engineering ToolBox,
    // "Emissivity Coefficients"; anodised is about 0.8, LBL spectral measurements).
    {"aluminium_6061", {167.0, 2700.0, 896.0, 0.09, std::nullopt}},
    // Cast aluminium A356-T6: Xometry A356 data sheet, Matmatch.
    {"aluminium_cast", {151.0, 2680.0, 963.0, 0.09, std::nullopt}},
    // Pure copper: Incropera et al., Fundamentals of Heat and Mass Transfer, 7th ed., Table A.1
    // (300 K); ε polished, Table A.11.
    {"copper", {401.0, 8933.0, 385.0, 0.03, std::nullopt}},
    // Stainless steel AISI 304: Incropera Table A.1 (300 K); ε polished, Engineering ToolBox
    // (TODO(verify): sources range 0.08 to 0.6 with finish).
    {"steel_304", {14.9, 7900.0, 477.0, 0.075, std::nullopt}},
    // FR-4: in-plane about 0.81, through-plane 0.29 (Wikipedia "FR-4", from IPC data); ρ 1850;
    // c_p 1100 (TODO(verify): measured 760 to 1180 over 300 to 430 K, LIST study); ε 0.9
    // (TODO(verify)). Axis: through the board.
    {"fr4", {0.81, 1850.0, 1100.0, 0.9, 0.29}},
    // PLA: k 0.13 (makeitfrom), ρ 1240, c_p 1800 (TODO(verify)), ε 0.9 (plastics, Engineering
    // ToolBox); glass transition 55-60 °C (Prusament PLA TDS).
    {"pla", {0.13, 1240.0, 1800.0, 0.9, std::nullopt}},
    // ABS: ρ 1100 (Ultimaker ABS TDS v5); k 0.17, c_p 1400 (TODO(verify)); ε 0.9.
    {"abs", {0.17, 1100.0, 1400.0, 0.9, std::nullopt}},
    // PETG: k 0.2 (sources give 0.16 to 0.29; Prusament PETG TDS, eSun TDS), ρ 1270, c_p 1300
    // (TODO(verify)), ε 0.9.
    {"petg", {0.2, 1270.0, 1300.0, 0.9, std::nullopt}},
    // Nylon PA12 (MJF/SLS): k 0.23 (ISO 22007), ρ 1010, c_p 1600 (Weerg and Xometry data sheets).
    {"nylon_pa12", {0.23, 1010.0, 1600.0, 0.9, std::nullopt}},
    // Polycarbonate: ρ 1190 (Ultimaker PC TDS v5); k 0.2, c_p 1200 (TODO(verify)); ε 0.9.
    {"polycarbonate", {0.2, 1190.0, 1200.0, 0.9, std::nullopt}},
    // CFRP, quasi-isotropic epoxy laminate: in-plane about 4 (TODO(verify)), through-thickness
    // 0.6 (0.5-0.8: PMC10339322; Southampton ICCM21), ρ 1600, c_p 750, ε 0.85 (TODO(verify)).
    // Axis: through the laminate.
    {"cfrp_epoxy", {4.0, 1600.0, 750.0, 0.85, 0.6}},
    // Soda-lime glass: Incropera Table A.3 (300 K) and A.11.
    {"glass", {1.4, 2500.0, 750.0, 0.92, std::nullopt}},
    // Silicone rubber: k about 0.2, ρ 1200, c_p 1270 (TODO(verify)), ε 0.9.
    {"silicone", {0.2, 1200.0, 1270.0, 0.9, std::nullopt}},
    // Silicone gap pad: k 2.4-5.0, ρ 3100-3600, c_p 850-1000 (Bergquist Gap Pad 2500 and
    // TGP 3000M data sheets); the middle of each range.
    {"thermal_pad", {3.0, 3300.0, 900.0, 0.9, std::nullopt}},
}};

}  // namespace

std::optional<ThermalMaterial> thermalMaterialPreset(std::string_view name) {
    for (const Row& row : kPresets) {
        if (row.name == name) {
            return row.material;
        }
    }
    return std::nullopt;
}

}  // namespace lagrange::thermal
