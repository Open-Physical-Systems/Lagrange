#pragma once

#include "lagrange/world/World.hpp"

#include <string>

namespace lagrange::mavlink {

/// A rotor's top speed under load, rad/s at its joint: what an autopilot's full output (1, or
/// 2000 µs) asks of the bridge's ESCs, so that the outputs between span what the motor can do.
struct RotorTopSpeed {
    double speed{0.0};  // rad/s at the rotor joint
    std::string from;   // how it was found, for the log
};

/// The rotor's loaded top speed, worked out from what the world says of it:
///
/// - With the ElectricalMotorAPI_1 that drives its joint (`elec:joint`) and a battery: the speed
///   at which the motor, turning the propeller (torque CP·ρ·D⁵·ω²/(8π³), its drive's current
///   with i = τ/(N·g·Kt)), runs out of voltage at the battery's full-charge open-circuit voltage
///   (V/√3 for pmsm and bldc in the peak dq convention, V for dc_brushed; R·i, back-EMF Ke·ω and
///   the cross term ω_e·Lq·i, the electrical domain's model in steady state), or reaches its
///   maxCurrent, whichever comes first. The battery is the one whose port leads straight to the
///   motor's elec:powerPort, else the one of the highest voltage. Battery sag, the gear's and
///   inverter's losses and field weakening are left out.
/// - Without a motor (none, or the electrical domain off): the speed at which the rotors together
///   lift twice the robot's weight (CT·ρ·n²·D⁴ each), a common multicopter margin.
/// - Without either: 850 rad/s, the sample quadcopter's.
[[nodiscard]] RotorTopSpeed rotorTopSpeed(const WorldView& world, const SchemaInstance& rotor,
                                          double airDensity);

}  // namespace lagrange::mavlink
