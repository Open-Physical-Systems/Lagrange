// The averaged models against closed forms (docs/design/domains/electrical.md §8).
#include "lagrange/electrical/Models.hpp"

#include <cmath>
#include <numbers>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::electrical {
namespace {

/// The sample AMR's drive (assets/robots/amr_diffdrive/amr_diffdrive.overlay.yaml).
MotorParams sampleDrive() {
    MotorParams p;
    p.resistance = 0.12;
    p.inductanceD = p.inductanceQ = 0.00031;
    p.torqueConstant = 0.085;
    p.polePairs = 4;
    p.gearRatio = 20.0;
    p.gearEfficiency = 0.92;
    p.maxCurrent = 8.0;
    return p;
}

TEST(MotorModel, RlStepResponseMatchesTheClosedForm) {
    // A voltage step on each axis of a standing motor: i(t) = V/R·(1 − e^(−t·R/L)), sampled at
    // 10 steps per time constant. Required within 0.1 %; the exact step integration gives
    // rounding error only.
    const MotorParams p = sampleDrive();
    const double tau = p.inductanceD / p.resistance;
    const double dt = tau / 10.0;
    constexpr double kVolts = 2.0;
    MotorState s;
    for (int n = 1; n <= 50; ++n) {
        advanceCurrents(p, s, kVolts, kVolts, 0.0, p.resistance, dt);
        const double expected = kVolts / p.resistance * (1.0 - std::exp(-n * dt / tau));
        EXPECT_NEAR(s.id, expected, 1e-3 * std::abs(expected)) << "step " << n;
        EXPECT_NEAR(s.iq, expected, 1e-3 * std::abs(expected)) << "step " << n;
    }
}

TEST(MotorModel, TheConstantsFollowThePeakDqConvention) {
    const MotorParams p = sampleDrive();
    EXPECT_DOUBLE_EQ(p.fluxLinkage(), 0.085 / (1.5 * 4));  // Kt = 1.5·p·λ
    EXPECT_DOUBLE_EQ(p.backEmfConstant(), 0.085 / 1.5);    // Ke = p·λ
    // Converted to the datasheet form: Kt (N·m/A RMS) = √3 · Ke (V RMS line-line per rad/s).
    const double ktRms = p.torqueConstant * std::sqrt(2.0);
    const double keLineLineRms = p.backEmfConstant() * std::sqrt(3.0) / std::sqrt(2.0);
    EXPECT_NEAR(ktRms, std::sqrt(3.0) * keLineLineRms, 1e-12);
    MotorParams brushed = p;
    brushed.dq = false;
    EXPECT_DOUBLE_EQ(brushed.backEmfConstant(), 0.085);  // Ke = Kt in SI units
}

/// Runs the drive at a fixed joint speed until it settles and returns the last outputs.
MotorOutputs settle(const MotorParams& p, double torqueCommand, double jointVelocity,
                    double busVoltage, MotorState* state = nullptr) {
    MotorState s;
    MotorInputs in;
    in.torqueCommand = torqueCommand;
    in.jointVelocity = jointVelocity;
    in.busVoltage = busVoltage;
    MotorOutputs out;
    for (int n = 0; n < 20000; ++n) {  // 2 s at 100 µs: hundreds of time constants
        out = stepMotor(p, s, in, 1e-4);
    }
    if (state != nullptr) {
        *state = s;
    }
    return out;
}

TEST(MotorModel, BrushedDcTorqueSpeedLineMatchesTheClosedForm) {
    // Brushed DC, torque asked beyond what the bus allows: the drive saturates at the full bus
    // voltage and the steady state is i = (V − Kt·ω)/R, τ = Kt·i. No-load speed V/Kt, stall
    // current V/R. Required within 0.5 %.
    MotorParams p;
    p.dq = false;
    p.resistance = 0.5;
    p.inductanceD = p.inductanceQ = 0.0005;
    p.torqueConstant = 0.05;
    p.maxCurrent = 100.0;  // above the stall current: the voltage limits
    p.currentLoopBandwidth = 3000.0;
    constexpr double kVolts = 24.0;
    const double noLoad = kVolts / p.torqueConstant;
    for (const double fraction : {0.0, 0.25, 0.5, 0.75, 0.95}) {
        const double w = fraction * noLoad;
        const MotorOutputs out = settle(p, 1e3, w, kVolts);
        const double current = (kVolts - p.torqueConstant * w) / p.resistance;
        EXPECT_NEAR(out.motorTorque, p.torqueConstant * current,
                    5e-3 * p.torqueConstant * kVolts / p.resistance)
            << "at " << fraction << " of the no-load speed";
    }
    // At the no-load speed it can make no torque.
    EXPECT_NEAR(settle(p, 1e3, noLoad, kVolts).motorTorque, 0.0, 1e-6);
    // Within its capability it delivers the torque asked.
    EXPECT_NEAR(settle(p, 0.5, 0.3 * noLoad, kVolts).motorTorque, 0.5, 5e-3 * 0.5);
}

// --- lead screws (RFC 0020 §4.4) -------------------------------------------------------------

/// A brushed motor on a 2 mm-lead screw: gearRatio 2π/lead rad/m, η forward 0.27, self-locking.
MotorParams screwDrive(double backEfficiency) {
    MotorParams p;
    p.dq = false;
    p.resistance = 0.5;
    p.inductanceD = p.inductanceQ = 0.0005;
    p.torqueConstant = 0.1;
    p.maxCurrent = 15.0;
    p.currentLoopBandwidth = 3000.0;
    p.gearRatio = 2.0 * std::numbers::pi / 0.002;
    p.gearEfficiency = 0.27;
    p.gearBackEfficiency = backEfficiency;
    return p;
}

TEST(LeadScrew, ThrustIsTwoPiEtaTorqueOverLead) {
    // Driving the load (and at standstill), F = 2π·η·T/lead: the motor's torque asked through
    // the forward efficiency, and the joint's force from the torque it makes.
    const MotorParams p = screwDrive(0.0);
    constexpr double kForce = 800.0;  // N, asked
    const MotorOutputs still = settle(p, kForce, 0.0, 24.0);
    EXPECT_NEAR(still.jointTorque,
                2.0 * std::numbers::pi * p.gearEfficiency * still.motorTorque / 0.002,
                1e-9 * kForce);
    EXPECT_NEAR(still.jointTorque, kForce, 1e-3 * kForce) << "the drive gives what is asked";
    EXPECT_NEAR(still.motorTorque, kForce * 0.002 / (2.0 * std::numbers::pi * 0.27), 1e-3 * 0.95);
    const MotorOutputs lifting = settle(p, kForce, 0.01, 24.0);  // 10 mm/s with the force
    EXPECT_NEAR(lifting.jointTorque,
                2.0 * std::numbers::pi * p.gearEfficiency * lifting.motorTorque / 0.002,
                1e-9 * kForce);
}

TEST(LeadScrew, BackDrivenItPassesItsMotorTorqueAndTheGearFrictionTakesTheRest) {
    // Back-driven (sinking under the load), T = η_back·F·lead/(2π): the motor's torque reaches
    // the joint whole (N·T) and the rest of the load, (1 − η_back)·F, is the gear's friction.
    constexpr double kBack = 0.6;
    const MotorParams p = screwDrive(kBack);
    constexpr double kLoad = -500.0;  // N, the weight, down the axis
    const double holding =
        kBack * 500.0 * 0.002 / (2.0 * std::numbers::pi);   // N·m, the back-driven torque
    const double ask = holding * p.gearRatio;               // N the drive is asked for, up
    const MotorOutputs out = settle(p, ask, -0.005, 24.0);  // sinking at 5 mm/s
    EXPECT_NEAR(out.jointTorque, p.gearRatio * out.motorTorque, 1e-9 * ask) << "no 1/η";
    EXPECT_NEAR(out.motorTorque, holding, 1e-3 * holding);
    const double friction = screwFrictionLimit(p, out.jointTorque, kLoad, -0.005);
    EXPECT_DOUBLE_EQ(friction, (1.0 - kBack) * 500.0);
    // In balance: the motor's force and the friction hold the weight, so it sinks steadily.
    EXPECT_NEAR(out.jointTorque + friction + kLoad, 0.0, 1e-3 * 500.0);
}

TEST(LeadScrew, TheFrictionFollowsWhoDrives) {
    const MotorParams screw = screwDrive(0.0);
    constexpr double kLoad = -981.0;  // N, 100 kg on a vertical screw
    EXPECT_DOUBLE_EQ(screwFrictionLimit(screw, 0.0, kLoad, 0.0), 981.0)
        << "unpowered and self-locking: it holds the whole load";
    EXPECT_DOUBLE_EQ(screwFrictionLimit(screw, 1200.0, kLoad, 0.0), 0.0)
        << "a motor that out-pulls the load lifts it through the forward efficiency";
    EXPECT_DOUBLE_EQ(screwFrictionLimit(screw, 1200.0, kLoad, 0.01), 0.0) << "lifting";
    EXPECT_DOUBLE_EQ(screwFrictionLimit(screw, 1200.0, kLoad, -0.01), 981.0)
        << "still sinking: the load drives until the motor has stopped it";
    EXPECT_DOUBLE_EQ(screwFrictionLimit(screw, 500.0, kLoad, 0.0), 981.0)
        << "too weak to lift: the screw holds";
    EXPECT_DOUBLE_EQ(screwFrictionLimit(screw, 500.0, kLoad, 0.01), 0.0)
        << "still going up: the motor drives while it slows";
    EXPECT_DOUBLE_EQ(screwFrictionLimit(screw, 1200.0, kLoad, -1e-6), 0.0)
        << "creeping down a µm/s is still: a motor that out-pulls the load starts to lift";
    EXPECT_DOUBLE_EQ(screwFrictionLimit(screw, -300.0, kLoad, -0.01), 981.0)
        << "pushed down with the load";
    EXPECT_DOUBLE_EQ(screwFrictionLimit(screwDrive(0.9), 0.0, 100.0, 0.0), 10.0)
        << "a ball screw, the load either way";
    EXPECT_DOUBLE_EQ(screwFrictionLimit(screwDrive(-1.0), 0.0, kLoad, 0.0), 0.0)
        << "not a screw: no friction";
}

TEST(LeadScrew, WithoutABackEfficiencyTheGearIsAsBefore) {
    // elec:gearBackEfficiency's fallback −1: back-driven, the joint sees the motor torque over η.
    MotorParams p = screwDrive(-1.0);
    EXPECT_FALSE(p.screw());
    EXPECT_DOUBLE_EQ(p.gearFactor(true), 0.27);
    EXPECT_DOUBLE_EQ(p.gearFactor(false), 1.0 / 0.27);
    p.gearBackEfficiency = 0.0;
    EXPECT_TRUE(p.screw());
    EXPECT_DOUBLE_EQ(p.gearFactor(false), 1.0);
}

TEST(MotorModel, PmsmDeliversTheCommandAndConservesPower) {
    // The sample drive at 10 rad/s at the wheel with 8 N·m asked: the joint gets 8 N·m, and the
    // electrical input power is the mechanical power plus the copper loss. This holds only if the
    // back-EMF and the torque come from the same flux linkage.
    const MotorParams p = sampleDrive();
    const MotorOutputs out = settle(p, 8.0, 10.0, 51.2);
    EXPECT_NEAR(out.jointTorque, 8.0, 5e-3 * 8.0);
    const double mechanical = out.motorTorque * p.gearRatio * 10.0;  // at the motor shaft
    const double electrical = out.busCurrent * 51.2;
    EXPECT_NEAR(electrical, mechanical + out.lossPower, 5e-3 * electrical);
    // Regenerating: the joint pushes the motor; power flows back and the gear loses on the way.
    const MotorOutputs back = settle(p, -8.0, 10.0, 51.2);
    EXPECT_NEAR(back.jointTorque, -8.0, 5e-3 * 8.0);
    EXPECT_LT(back.busCurrent, 0.0);
    // The current limit caps the torque: 0.085·8·20·0.92 = 12.5 N·m.
    EXPECT_NEAR(settle(p, 50.0, 0.0, 51.2).jointTorque, 0.085 * 8.0 * 20.0 * 0.92, 1e-3);
}

TEST(MotorModel, ALongStepCapsTheCurrentLoopInsteadOfDiverging) {
    // At the network's largest step (1 ms) a 1 kHz loop (6283 rad/s) would be 6 steps of
    // bandwidth per step and diverge; capped at 1/dt it settles on the command.
    const MotorParams p = sampleDrive();
    MotorState s;
    MotorInputs in;
    in.torqueCommand = 5.0;
    in.busVoltage = 51.2;
    MotorOutputs out;
    for (int n = 0; n < 200; ++n) {
        out = stepMotor(p, s, in, 1e-3);
    }
    EXPECT_NEAR(out.jointTorque, 5.0, 5e-3 * 5.0);
    EXPECT_NEAR(s.iq, 5.0 / (20.0 * 0.92 * 0.085), 1e-3);
}

TEST(MotorModel, AnOffDriveMakesNoTorque) {
    const MotorParams p = sampleDrive();
    MotorState s;
    s.iq = 5.0;
    MotorInputs in;
    in.torqueCommand = 5.0;
    in.busVoltage = 48.0;
    in.enabled = false;
    const MotorOutputs out = stepMotor(p, s, in, 1e-4);
    EXPECT_EQ(out.jointTorque, 0.0);
    EXPECT_EQ(out.busCurrent, 0.0);
    EXPECT_EQ(s.iq, 0.0);
}

/// The drive turning a flywheel J (kg·m², at the joint): the drive steps at 100 µs, the
/// flywheel at `mechanicalDt` with the drive's last torque, as physics does.
std::vector<double> spinUp(double mechanicalDt, double electricalDt, double seconds) {
    const MotorParams p = sampleDrive();
    constexpr double kInertia = 0.05;
    MotorState s;
    double velocity = 0.0;
    double torque = 0.0;
    std::vector<double> speeds;
    const int mechanicalSteps = static_cast<int>(std::lround(seconds / mechanicalDt));
    const int sub = static_cast<int>(std::lround(mechanicalDt / electricalDt));
    for (int k = 0; k < mechanicalSteps; ++k) {
        for (int e = 0; e < sub; ++e) {
            MotorInputs in;
            in.torqueCommand = 10.0;
            in.jointVelocity = velocity;
            in.busVoltage = 51.2;
            torque = stepMotor(p, s, in, electricalDt).jointTorque;
        }
        velocity += torque / kInertia * mechanicalDt;
        speeds.push_back(velocity);
    }
    return speeds;
}

TEST(MotorModel, CoSimulatedWithPhysicsAtOneMillisecondItTracksTheTightlyCoupledSolution) {
    // Architecture §8 coupling test: drive at 100 µs, flywheel at 1 ms (the server's split),
    // against both at 10 µs as the reference. It accelerates into the voltage limit, so the
    // back-EMF coupling matters. Required within 2 % RMS.
    const std::vector<double> coSim = spinUp(1e-3, 1e-4, 1.5);
    const std::vector<double> tight = spinUp(1e-5, 1e-5, 1.5);
    ASSERT_EQ(tight.size(), 150000U);
    double error = 0.0;
    double scale = 0.0;
    for (std::size_t k = 0; k < coSim.size(); ++k) {
        const double reference = tight[(k + 1) * 100 - 1];
        error += (coSim[k] - reference) * (coSim[k] - reference);
        scale += reference * reference;
    }
    EXPECT_LT(std::sqrt(error / scale), 0.02);
    // And it did reach the voltage limit: the speed levels off below 51.2/√3/Ke/N.
    const MotorParams p = sampleDrive();
    const double limit = 51.2 / std::sqrt(3.0) / p.backEmfConstant() / p.gearRatio;
    EXPECT_GT(coSim.back(), 0.8 * limit);
    EXPECT_LT(coSim.back(), limit);
}

BatteryParams sampleBattery() {
    BatteryParams p;
    p.seriesCells = 16;
    p.parallelCells = 2;
    p.capacity = 72000.0;
    p.ocvSoc = {0.0, 0.1, 0.5, 0.9, 1.0};
    p.ocvVoltage = {2.80, 3.20, 3.28, 3.33, 3.45};
    p.r0 = 0.0015;
    p.r1 = 0.0008;
    p.c1 = 12000.0;
    p.cutoffVoltage = 44.8;
    p.maxChargeCurrent = 10.0;
    return p;
}

TEST(BatteryModel, PulseDischargeMatchesTheClosedForm) {
    // A constant-current pulse from rest: per cell V = OCV(SoC(t)) − i·r0 − i·r1·(1 − e^(−t/τ1)),
    // τ1 = r1·c1, i = I/parallelCells, SoC(t) = SoC0 − I·t/Q; the pack is seriesCells cells.
    const BatteryParams p = sampleBattery();
    BatteryState s;
    s.soc = 0.7;
    constexpr double kCurrent = 30.0;  // A, pack
    constexpr double kDt = 0.01;
    const double tau = p.r1 * p.c1;
    const double cell = kCurrent / p.parallelCells;
    for (int n = 1; n <= 3000; ++n) {
        advanceBattery(p, s, kCurrent, kDt);
        const double t = n * kDt;
        const Thevenin th = theveninOf(p, s, 298.15);
        const double terminal = th.emf - kCurrent * th.resistance;
        const double soc = 0.7 - kCurrent * t / p.capacity;
        const double expected = p.openCircuitVoltage(soc) - p.seriesCells * cell * p.r0 -
                                p.seriesCells * cell * p.r1 * (1.0 - std::exp(-t / tau));
        ASSERT_NEAR(terminal, expected, 1e-9) << "at " << t << " s";
    }
    EXPECT_FALSE(s.cut);
}

TEST(BatteryModel, OcvInterpolatesAndTheResistanceFollowsTemperature) {
    BatteryParams p = sampleBattery();
    EXPECT_DOUBLE_EQ(p.openCircuitVoltage(0.3), 16 * (3.20 + 0.5 * (3.28 - 3.20)));
    EXPECT_DOUBLE_EQ(p.openCircuitVoltage(1.5), 16 * 3.45);
    EXPECT_DOUBLE_EQ(p.openCircuitVoltage(-1.0), 16 * 2.80);
    p.r0TempCoeff = 0.01;
    const BatteryState s;
    EXPECT_NEAR(theveninOf(p, s, 308.15).resistance, 16 * 0.0015 * 1.1 / 2, 1e-15);
    // An RC pair without capacitance is a plain resistance.
    p.r0TempCoeff = 0.0;
    p.c1 = 0.0;
    EXPECT_NEAR(theveninOf(p, s, 298.15).resistance, 16 * (0.0015 + 0.0008) / 2, 1e-15);
}

TEST(BatteryModel, AnEmptyPackCutsOut) {
    const BatteryParams p = sampleBattery();
    BatteryState s;
    s.soc = 1e-4;
    advanceBattery(p, s, 72000.0, 1.0);  // more than what is left
    EXPECT_EQ(s.soc, 0.0);
    EXPECT_TRUE(s.cut);
}

// RFC 0020 §4.3: the position loop a drive in elec:controlMode position runs for a smart servo.
TEST(ServoLoop, DutyFollowsTheGainsDeadZoneAndSensingStep) {
    ServoLoop loop;
    loop.kp = 20.0;
    loop.ki = 4.0;
    loop.kd = 0.5;
    loop.deadband = 0.01;
    double integral = 0.0;
    // Inside the dead zone: off, the integral held.
    EXPECT_EQ(servoDuty(loop, integral, 0.005, 0.0, 0.0, 1e-3, -1.0, 1.0), 0.0);
    EXPECT_EQ(integral, 0.0);
    // Outside: Kp (e − band) − Kd ω, then the integral grows by (e − band) dt.
    EXPECT_NEAR(servoDuty(loop, integral, 0.03, 0.0, 0.1, 1e-3, -1.0, 1.0), 20.0 * 0.02 - 0.05,
                1e-12);
    EXPECT_NEAR(integral, 0.02e-3, 1e-15);
    // Held at the limit: no integration (anti-windup).
    const double before = integral;
    EXPECT_EQ(servoDuty(loop, integral, 1.0, 0.0, 0.0, 1e-3, -0.3, 0.3), 0.3);
    EXPECT_EQ(integral, before);
    // The sensing step rounds the measured position first.
    ServoLoop coarse;
    coarse.kp = 1.0;
    coarse.resolution = 0.1;
    double none = 0.0;
    EXPECT_NEAR(servoDuty(coarse, none, 0.5, 0.26, 0.0, 1e-3, -1.0, 1.0), 0.2, 1e-12);
}

TEST(ServoLoop, CurrentLimitBoundsTheArmatureCurrent) {
    MotorParams p;
    p.dq = false;
    p.torqueConstant = 0.01;
    p.gearRatio = 100.0;
    const double resistance = 2.0;
    double uMin = 0.0;
    double uMax = 0.0;
    currentLimitedDuty(p, resistance, 12.0, 3.0, 1.5, uMin, uMax);
    // i = (u V − Kt N ω)/R at the window's ends is ±1.5 A.
    EXPECT_NEAR((uMax * 12.0 - 0.01 * 100.0 * 3.0) / resistance, 1.5, 1e-12);
    EXPECT_NEAR((uMin * 12.0 - 0.01 * 100.0 * 3.0) / resistance, -1.5, 1e-12);
    currentLimitedDuty(p, resistance, 12.0, 3.0, 0.0, uMin, uMax);  // no limit
    EXPECT_EQ(uMin, -1.0);
    EXPECT_EQ(uMax, 1.0);
}

}  // namespace
}  // namespace lagrange::electrical
