#pragma once

// The holding brake of RFC 0020 "Also worth adding", free of the bus so that tests check it
// against its closed forms: a spring-applied, power-off brake on a joint, released by a coil
// that pulls its armature off the friction disc. The coil is an R–L circuit with a freewheeling
// diode across it (the usual suppression), so its current rises as V/R·(1 − e^(−t/τ)) when
// powered and decays as e^(−t/τ) when not, τ = L/R. The armature lets go when the current
// reaches the release current and the springs apply it again when the current falls to the
// dropout current (a hysteresis: pull-in needs more than holding off). While applied, the brake
// is a Coulomb friction on the joint: its static holding torque at rest, its dynamic torque
// while the joint slips. SI units; N·m on a hinge, N on a slide.

namespace lagrange::actuators {

/// BrakeAPI_1's attributes.
struct BrakeParams {
    double holding{0.0};         // N·m (or N): static, the most it holds at rest
    double dynamic{0.0};         // N·m (or N): while slipping
    double coilResistance{0.0};  // ohm
    double coilInductance{0.0};  // H; 0: the current follows the voltage at once
    double ratedVoltage{0.0};    // V
    double releaseCurrent{0.0};  // A: the armature pulls in at or above it
    double dropoutCurrent{0.0};  // A: the springs apply it at or below it
};

/// The brake's state between steps.
struct BrakeState {
    double current{0.0};  // A
    bool engaged{true};   // a power-off brake starts applied
};

/// Joint speeds below this count as at rest for the static torque, rad/s (or m/s): a drawing
/// line between stiction and slip, not a device value.
inline constexpr double kBrakeRestSpeed = 1e-3;

/// One step of `dt` with `voltage` across the coil (0: off, the diode freewheeling): the exact
/// R–L solution, then the armature's hysteresis.
void stepBrake(const BrakeParams& p, BrakeState& s, double voltage, double dt);

/// The friction limit the applied brake puts on a joint at `speed`; 0 when released.
[[nodiscard]] double brakeFriction(const BrakeParams& p, const BrakeState& s, double speed);

}  // namespace lagrange::actuators
