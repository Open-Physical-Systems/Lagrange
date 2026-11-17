#pragma once

// A two-phase hybrid stepper on a chopper driver (RFC 0020 §4.5), free of the bus and the
// scheduler so that the validation tests can drive it directly. SI units throughout.
//
//   v_a = R·i_a + L·di_a/dt − K_m·ω·sin(N_r·θ)
//   v_b = R·i_b + L·di_b/dt + K_m·ω·cos(N_r·θ)
//   τ   = −K_m·i_a·sin(N_r·θ) + K_m·i_b·cos(N_r·θ) − T_d·sin(4·N_r·θ)
//
// θ and ω are the rotor's angle and speed (motor shaft), N_r its teeth, K_m the torque per phase
// ampere, T_d the detent torque. The driver's microstep table puts i_a* = I·cos φ and
// i_b* = I·sin φ on the phases, φ = k·(π/2)/μ at microstep k, I the run current while it steps
// and the hold current at standstill. Its chopper is averaged over the PWM period: each phase
// gets the voltage that brings its current to the reference by the end of the step, held to
// ±V_bus, so at speed the current lags and falls (L/R and the back-EMF) and the torque with it.
#include "lagrange/electrical/Models.hpp"

#include <cstdint>

namespace lagrange::electrical {

struct StepperParams {
    /// R, L (inductanceQ), K_m (torqueConstant), the gear, the current limit (maxCurrent: the run
    /// current), the winding's temperature coefficient. dq is false; polePairs unused.
    MotorParams motor;
    int rotorTeeth{50};        // N_r
    int microsteps{1};         // μ, per full step
    double detentTorque{0.0};  // N·m, T_d
    double holdCurrent{0.0};   // A, peak per phase at standstill

    /// Electrical radians per joint unit (rad, or m on a slide): N_r·gearRatio.
    [[nodiscard]] double electricalPerJoint() const;
    /// Microsteps per joint unit: one full step is π/2 electrical.
    [[nodiscard]] double microstepsPerJoint() const;
};

struct StepperState {
    double ia{0.0}, ib{0.0};  // A, phase currents
    /// The rotor as the driver's local model integrates it between physics updates (joint
    /// units: rad or m, rad/s or m/s), and where step 0 is (the joint's position at start).
    double position{0.0}, velocity{0.0};
    double origin{0.0};
    double steps{0.0};     // microsteps commanded so far (an integer)
    double stillFor{0.0};  // s since the step count last changed
};

struct StepperInputs {
    double busVoltage{0.0};
    double temperature{298.15};  // K, of the winding
    bool enabled{true};          // false: no current (the driver's enable pin)
    /// With an inertia (kg·m², or kg on a slide) the rotor is advanced here, under the drive's
    /// torque and `loadTorque` (N·m or N at the joint, opposing positive motion when positive);
    /// without one, `position` and `velocity` stay as the caller set them.
    double inertia{0.0};
    double loadTorque{0.0};
    /// Above 0: a voltage-mode driver instead of the chopper, the table's cosine and sine times
    /// this amplitude (V, held to the bus) on the phases, with no current control. The steady
    /// state then has a closed form (the pull-out test's reference).
    double voltageAmplitude{0.0};
};

struct StepperOutputs {
    double jointTorque{0.0};  // N·m or N, the step's mean
    double motorTorque{0.0};  // N·m at the shaft
    double va{0.0}, vb{0.0};  // V, applied
    double busCurrent{0.0};   // A
    double lossPower{0.0};    // W, copper loss
};

/// The shaft torque at phase currents `ia`, `ib` and electrical angle `electrical` = N_r·θ.
[[nodiscard]] double stepperTorque(const StepperParams& p, double ia, double ib, double electrical);

/// The current the driver asks for: the hold current once still for `kStandstillDelay`.
[[nodiscard]] double stepperCurrent(const StepperParams& p, const StepperState& s);

/// One step: the chopper's phase voltages, the currents (the exact R-L solution with the voltage
/// and back-EMF held), the torque, and with an inertia the rotor.
StepperOutputs stepStepper(const StepperParams& p, StepperState& s, const StepperInputs& in,
                           double dt);

/// Full steps the rotor at `position` (joint units) is behind the step count, in whole electrical
/// periods: a slip lands on the next equilibrium, four full steps away, so this is a multiple of
/// 4. Positive: behind (the rotor did fewer steps than commanded).
[[nodiscard]] std::int64_t lostSteps(const StepperParams& p, const StepperState& s,
                                     double position);

/// How long a driver waits at standstill before it drops to the hold current, s. A modelling
/// choice (drivers make it a setting), not a hardware value.
inline constexpr double kStandstillDelay = 0.2;

}  // namespace lagrange::electrical
