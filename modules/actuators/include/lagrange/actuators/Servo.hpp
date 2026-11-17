#pragma once

// The servo board of RFC 0020 §4.3, free of the bus so that tests check it against its closed
// forms: a hobby PWM servo's pulse map, deadband and proportional loop, a smart servo's P/I/D
// loop, velocity and current modes, profile and shutdown rules, and the motor-less `ideal`
// fidelity's torque line. SI units throughout: angles in rad at the joint, duty cycles in −1…1.

#include <cstdint>

namespace lagrange::actuators {

/// ServoAPI_1's attributes.
struct ServoParams {
    enum class Kind : std::uint8_t { Pwm, Smart };
    enum class OperatingMode : std::uint8_t { Position, Velocity, Current };
    Kind kind{Kind::Pwm};
    OperatingMode mode{OperatingMode::Position};
    double pulseMin{0.0}, pulseMax{0.0};  // s
    double angleMin{0.0}, angleMax{0.0};  // rad, the joint positions those pulses map to
    double deadband{0.0};                 // rad
    double kp{0.0};                       // 1/rad, duty per error
    double ki{0.0};                       // 1/(rad·s)
    double kd{0.0};                       // s/rad
    double velocityKp{0.0};               // s/rad
    double velocityKi{0.0};               // 1/rad
    double profileVelocity{0.0};          // rad/s, 0: none
    double profileAcceleration{0.0};      // rad/s², 0: none
    double resolution{0.0};               // rad, 0: exact sensing
    double stallTorque{0.0};              // N·m at the joint, full duty, standstill
    double noLoadSpeed{0.0};              // rad/s at the joint, full duty, no load
    double overloadTime{0.0};             // s, 0: no overload shutdown
    double maxTemperature{0.0};           // K, 0: no overheating shutdown
};

/// The joint position a pulse width asks for, by the servo's own linear map (pulseMin → angleMin,
/// pulseMax → angleMax), not held to the map's ends: a pulse outside them asks for more, as on a
/// real servo. NaN when the map is degenerate.
[[nodiscard]] double pulseToAngle(const ServoParams& p, double pulse);

/// `x` as a sensor with steps of `step` reads it (round to nearest); `x` itself for step ≤ 0.
[[nodiscard]] double quantise(double x, double step);

/// The board's view of an error: 0 inside ±band, the error less the band outside (a dead zone,
/// continuous, so the servo stops in the band without hunting, and holds a load at the band's
/// edge plus the error the load needs).
[[nodiscard]] double deadband(double error, double band);

/// The position loop's integrator, carried between steps.
struct LoopState {
    double integral{0.0};  // rad·s (position) or rad (velocity mode)
};

/// One step of the position loop of §4.3: u = Kp·e + Ki·∫e + Kd·(ω* − ω), with e the error after
/// the deadband, measured position quantised by the resolution, held to [uMin, uMax] (−1…1, or
/// tighter under a current limit). The integrator stops while the output is held (anti-windup)
/// and inside the deadband. `targetRate`: the set point's velocity (the profile's), rad/s.
[[nodiscard]] double positionDuty(const ServoParams& p, LoopState& s, double target,
                                  double targetRate, double position, double velocity, double dt,
                                  double uMin = -1.0, double uMax = 1.0);

/// One step of the velocity loop: u = Kvp·(ω* − ω) + Kvi·∫(ω* − ω), held to [uMin, uMax].
[[nodiscard]] double velocityDuty(const ServoParams& p, LoopState& s, double goal, double velocity,
                                  double dt, double uMin = -1.0, double uMax = 1.0);

/// The motor-less (`ideal`) fidelity's joint torque at duty `u` and joint speed ω: the DC motor's
/// line through the datasheet's two numbers, τ = τ_stall·(u − ω/ω0). At u = 1 it is
/// τ_stall·(1 − ω/ω0): stall torque at rest, zero at the no-load speed; back-driven (ω against
/// u) it brakes with more than τ_stall, as a DC motor does.
[[nodiscard]] double idealTorque(const ServoParams& p, double u, double velocity);

/// A brushed motor through its gear, as the duty limits see it, at the joint: armature
/// resistance R (Ω) and back-EMF per joint speed k = N·Ke (V·s/rad; Ke = Kt for a brushed motor).
struct Armature {
    double resistance{0.0};    // Ω
    double backEmf{0.0};       // V·s/rad at the joint
    double maxCurrent{0.0};    // A, 0: no limit
    double torquePerAmp{0.0};  // N·m/A at the joint, motoring: N·Kt·η
};

/// The duty range in which the armature current (u·V − k·ω)/R stays within ±maxCurrent, held to
/// −1…1 (a smart servo's current limit on a bare H-bridge). {−1, 1} without a limit.
void currentLimitedDuty(const Armature& a, double busVoltage, double velocity, double& uMin,
                        double& uMax);

/// Current mode: the duty that drives `current` through the armature at speed ω,
/// (R·i + k·ω)/V, held to −1…1.
[[nodiscard]] double currentDuty(const Armature& a, double busVoltage, double current,
                                 double velocity);

/// The stall torque and no-load speed at the joint for a bus voltage: τ_stall = N·Kt·η·V/R,
/// ω0 = V/(N·Ke). Writing them out is how a servo is checked against its datasheet.
[[nodiscard]] double stallTorqueOf(const Armature& a, double busVoltage);
[[nodiscard]] double noLoadSpeedOf(const Armature& a, double busVoltage);

/// The set point on its way to the goal under a profile's velocity and acceleration limits
/// (Dynamixel's Profile Velocity and Profile Acceleration): trapezoidal (triangular when the move
/// is short), stopping on the goal. Without limits it steps to the goal.
struct Profile {
    double position{0.0};  // rad
    double velocity{0.0};  // rad/s
};
void stepProfile(Profile& s, double goal, double maxVelocity, double maxAcceleration, double dt);

/// The shutdown rules of a smart servo (RFC 0020 open question 6): overload when the output stays
/// saturated for overloadTime, overheating at maxTemperature. Either latches the torque off until
/// torque_enable goes off and on again.
struct Shutdown {
    double saturatedFor{0.0};  // s
    bool overload{false};
    bool overheated{false};
    [[nodiscard]] bool tripped() const { return overload || overheated; }
};
void stepShutdown(const ServoParams& p, Shutdown& s, bool saturated, double temperature, double dt);

}  // namespace lagrange::actuators
