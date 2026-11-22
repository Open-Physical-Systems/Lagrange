// The stepper and its chopper driver of RFC 0020 §4.5; the equations are in Stepper.hpp.
#include "lagrange/electrical/Stepper.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace lagrange::electrical {

namespace {

constexpr double kQuarterTurn = std::numbers::pi / 2.0;  // electrical rad per full step
constexpr double kTurn = 2.0 * std::numbers::pi;
// Rotor sub-steps per electrical step: the torque is re-evaluated as the rotor turns within the
// step (a 1.8° stepper's stiffness K_m·I·N_r on a light load rings at a few hundred hertz).
constexpr int kRotorSubsteps = 4;
// Detent torque's harmonic: four detent periods per electrical period (one per full step).
constexpr double kDetentHarmonic = 4.0;

/// One phase: the voltage that brings the current from `current` to `reference` by the end of
/// the step under `emf` (the back-EMF term moved to the source side), held to ±limit; then the
/// current at the end of the step and its mean over the step.
struct Phase {
    double voltage, end, mean;
};

Phase chop(double current, double reference, double emf, double resistance, double inductance,
           double limit, double dt, double forced) {
    double voltage = 0.0;
    double end = 0.0;
    double mean = 0.0;
    if (inductance <= 0.0 || resistance <= 0.0) {
        voltage =
            std::clamp(std::isnan(forced) ? resistance * reference - emf : forced, -limit, limit);
        end = mean = resistance > 0.0 ? (voltage + emf) / resistance : reference;
        return {voltage, end, mean};
    }
    const double tau = inductance / resistance;
    const double decay = std::exp(-dt / tau);
    voltage = std::isnan(forced) ? resistance * (reference - decay * current) / (1.0 - decay) - emf
                                 : forced;
    voltage = std::clamp(voltage, -limit, limit);
    const double settled = (voltage + emf) / resistance;
    end = settled + (current - settled) * decay;
    mean = settled + (current - settled) * tau / dt * (1.0 - decay);
    return {voltage, end, mean};
}

}  // namespace

double StepperParams::electricalPerJoint() const {
    return rotorTeeth * motor.gearRatio;
}

double StepperParams::microstepsPerJoint() const {
    return electricalPerJoint() / kQuarterTurn * microsteps;
}

double stepperTorque(const StepperParams& p, double ia, double ib, double electrical) {
    const double km = p.motor.torqueConstant;
    return -km * ia * std::sin(electrical) + km * ib * std::cos(electrical) -
           p.detentTorque * std::sin(kDetentHarmonic * electrical);
}

double stepperCurrent(const StepperParams& p, const StepperState& s) {
    return s.stillFor >= kStandstillDelay ? p.holdCurrent : p.motor.maxCurrent;
}

StepperOutputs stepStepper(const StepperParams& p, StepperState& s, const StepperInputs& in,
                           double dt) {
    StepperOutputs out;
    const double r = p.motor.resistanceAt(in.temperature);
    const double l = p.motor.inductanceQ;
    const double km = p.motor.torqueConstant;
    const double n = p.motor.gearRatio;
    const double perJoint = p.electricalPerJoint();
    const double electrical = perJoint * (s.position - s.origin);
    const double speed = n * s.velocity;  // rad/s at the shaft
    const double current = in.enabled ? stepperCurrent(p, s) : 0.0;
    const double phi = s.steps * kQuarterTurn / p.microsteps;
    const double limit = std::max(0.0, in.busVoltage);
    // Back-EMF on the source side: L·di_a/dt = v_a − R·i_a + K_m·ω·sin θe, and for b −K_m·ω·cos θe.
    // A voltage-mode driver forces the table's voltages; the chopper finds them (NaN).
    const bool forced = in.voltageAmplitude > 0.0;
    const double amplitude = in.enabled ? in.voltageAmplitude : 0.0;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Phase a = chop(s.ia, current * std::cos(phi), km * speed * std::sin(electrical), r, l,
                         limit, dt, forced ? amplitude * std::cos(phi) : nan);
    const Phase b = chop(s.ib, current * std::sin(phi), -km * speed * std::cos(electrical), r, l,
                         limit, dt, forced ? amplitude * std::sin(phi) : nan);
    s.ia = a.end;
    s.ib = b.end;
    out.va = a.voltage;
    out.vb = b.voltage;
    out.lossPower = r * (a.mean * a.mean + b.mean * b.mean);
    out.busCurrent = limit > 0.0 ? (a.voltage * a.mean + b.voltage * b.mean) / limit : 0.0;

    const auto jointTorque = [&](double shaft, double jointSpeed) {
        return n * p.motor.gearFactor(shaft * jointSpeed >= 0.0) * shaft;
    };
    if (in.inertia <= 0.0) {
        out.motorTorque = stepperTorque(p, a.mean, b.mean, electrical);
        out.jointTorque = jointTorque(out.motorTorque, s.velocity);
        return out;
    }
    // The rotor under the drive's torque and the load: semi-implicit Euler in sub-steps.
    const double h = dt / kRotorSubsteps;
    for (int k = 0; k < kRotorSubsteps; ++k) {
        const double angle = perJoint * (s.position - s.origin);
        const double shaft = stepperTorque(p, a.mean, b.mean, angle);
        const double torque = jointTorque(shaft, s.velocity);
        s.velocity += (torque - in.loadTorque) / in.inertia * h;
        s.position += s.velocity * h;
        out.motorTorque += shaft / kRotorSubsteps;
        out.jointTorque += torque / kRotorSubsteps;
    }
    return out;
}

std::int64_t lostSteps(const StepperParams& p, const StepperState& s, double position) {
    const double phi = s.steps * kQuarterTurn / p.microsteps;
    const double electrical = p.electricalPerJoint() * (position - s.origin);
    return 4 * static_cast<std::int64_t>(std::llround((phi - electrical) / kTurn));
}

}  // namespace lagrange::electrical
