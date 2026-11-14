// Density presets for the Add panel's material menu (RFC 0016 open question 9, plan §6).
//
// The names and densities are those of the thermal presets of RFC 0012
// (modules/thermal/conduction/src/Materials.cpp), so a link's mass and its thermal material
// agree; copied because the viewer cannot link lagrange_thermal (invariant 14). The sources are
// as the repository cites them; none was read from its primary source for this table, so every
// value is TODO(verify) (plan §6 names the document to read for each).
//
// Not here, for want of any source in the repository (plan §6, open question 9), never invented:
// TODO(verify) carbon steel (AISI 1018; candidate Incropera Table A.1, "plain carbon steel"),
// TODO(verify) natural rubber (candidate a compound data sheet, ISO 2781),
// TODO(verify) pine (candidate USDA FPL Wood Handbook, FPL-GTR-190, Table 5-3a, by species at
// 12 % moisture).
#include "lagrange/viewer/builder/Materials.hpp"

#include <array>

namespace lagrange::viewer::builder {

namespace {

constexpr std::array<DensityPreset, 5> kPresets{{
    // TODO(verify): ASM Aerospace Specification Metals, 6061-T6.
    {"aluminium_6061", 2700.0, "Aluminium 6061-T6: ASM/MatWeb data sheet (TODO(verify))",
     "Aluminium 6061"},
    // TODO(verify): Incropera Table A.1.
    {"steel_304", 7900.0,
     "Stainless steel AISI 304: Incropera et al., Fundamentals of Heat and Mass Transfer, 7th "
     "ed., Table A.1, 300 K (TODO(verify))",
     "Steel 304"},
    // The thermal preset names no source for this density (only for k and Tg).
    // TODO(verify): candidate NatureWorks Ingeo 4043D technical data sheet (ASTM D792).
    {"pla", 1240.0, "PLA: no source cited for the density yet (TODO(verify))", "PLA"},
    // TODO(verify): read the TDS.
    {"abs", 1100.0, "ABS: Ultimaker ABS technical data sheet v5 (TODO(verify))", "ABS"},
    // TODO(verify); PA6 or PA66 injection grades are denser.
    {"nylon_pa12", 1010.0, "Nylon PA12 (MJF/SLS): Weerg and Xometry data sheets (TODO(verify))",
     "Nylon PA12"},
}};

}  // namespace

std::span<const DensityPreset> densityPresets() {
    return kPresets;
}

}  // namespace lagrange::viewer::builder
