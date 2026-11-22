#pragma once

// The solenoid of RFC 0020 §4.6, free of the bus so that tests check it against its closed forms:
// a plunger in a coil whose inductance L(x) grows as the plunger goes in. With the flux linkage
// λ = L(x)·i as the coil's state,
//
//   dλ/dt = v − R·i,   i = λ / L(x)
//   F = ½·i²·dL/dx − (F0 + k·x)            (the magnetic pull by the co-energy, the return spring)
//
// so a plunger that moves in at a given flux raises L and drops the current: the dip a valve
// driver watches for. x is the joint's position, 0 at rest (fully out) and `stroke` fully in;
// the pull is along +x. SI units throughout.

#include <string>
#include <vector>

namespace lagrange::actuators {

/// SolenoidAPI_1's attributes.
struct SolenoidParams {
    std::vector<double> position;    // m, the table's strokes, rising
    std::vector<double> inductance;  // H at each
    double resistance{0.0};          // ohm
    double springRate{0.0};          // N/m
    double springPreload{0.0};       // N, the spring's push at x = 0
    double stroke{0.0};              // m
    double ratedVoltage{0.0};        // V
};

/// What is wrong with `p` (empty when nothing): a table of at least two rising strokes with
/// positive inductances covering 0 to the stroke, a positive resistance, stroke and voltage, and
/// a spring that does not pull.
[[nodiscard]] std::string checkSolenoid(const SolenoidParams& p);

/// L(x), linear between the table's points, held at its ends outside them. H.
[[nodiscard]] double solenoidInductance(const SolenoidParams& p, double x);

/// dL/dx: the slope of the table's segment that holds x (the right one on a point; the end
/// segments' beyond the table). H/m.
[[nodiscard]] double solenoidInductanceSlope(const SolenoidParams& p, double x);

/// The magnetic pull ½·i²·dL/dx at current `current` and stroke x, N along +x.
[[nodiscard]] double solenoidPull(const SolenoidParams& p, double current, double x);

/// The return spring's force −(F0 + k·x), N along +x.
[[nodiscard]] double solenoidSpring(const SolenoidParams& p, double x);

/// The flux linkage after `dt` with `voltage` across the coil and the plunger at x through the
/// step: the exact R–L solution at L(x), di/dt = (v − R i)/L, from i = λ/L(x). A plunger that
/// moved since the last step keeps its flux (λ is continuous), so its current jumps by the ratio
/// of the inductances. Wb.
[[nodiscard]] double stepSolenoidFlux(const SolenoidParams& p, double flux, double voltage,
                                      double x, double dt);

}  // namespace lagrange::actuators
