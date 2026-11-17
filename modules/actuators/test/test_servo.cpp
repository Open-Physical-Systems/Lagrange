// The servo board of RFC 0020 §4.3 against closed forms and datasheet-style numbers: the pulse
// map; the torque line through stall torque and no-load speed; a smart servo's stall torque and
// no-load speed from its motor at three supply voltages (ROBOTIS's XM430-W350 table); the step
// response at full duty against the first-order motor's exact solution (the "s/60°" speed
// figure); holding torque (stiffness Kp·τ_stall, back-driven speed beyond stall); a deadband
// that stops without hunting; a trapezoidal profile; the current limit and current mode; the
// overload shutdown.
#include "lagrange/actuators/Servo.hpp"

#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

namespace lagrange::actuators {
namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kRpm = 2.0 * kPi / 60.0;  // rad/s per rev/min

// A standard-size hobby servo, illustrative numbers of the size its datasheet gives (not one
// product): 0.3 N·m stall, 60° in 0.12 s with no load, a 1.0–2.0 ms pulse over ±45°.
ServoParams hobby() {
    ServoParams p;
    p.kind = ServoParams::Kind::Pwm;
    p.pulseMin = 1.0e-3;
    p.pulseMax = 2.0e-3;
    p.angleMin = -kPi / 4.0;
    p.angleMax = kPi / 4.0;
    p.kp = 20.0;  // full duty from 2.9° of error
    p.stallTorque = 0.3;
    p.noLoadSpeed = (kPi / 3.0) / 0.12;  // the s/60° figure as a top speed: 8.73 rad/s
    return p;
}

// One joint: J·ω̇ = τ_servo(u, ω) + τ_ext, the board sampling every `board` s (1 ms, as the
// module), the plant integrated at 10 µs.
struct Plant {
    double inertia{2.0e-5};  // kg·m², a servo horn and a small arm at the output
    double theta{0.0}, omega{0.0};
    double external{0.0};  // N·m
};

struct Trace {
    double timeToReach{-1.0};  // s, first time θ ≥ `reach`
    int signChanges{0};        // of the duty over the second half of the run
    double u{0.0};
};

Trace run(const ServoParams& p, Plant& plant, double target, double seconds, double reach = 1e9,
          LoopState* loop = nullptr) {
    LoopState own;
    LoopState& s = loop != nullptr ? *loop : own;
    constexpr double kBoard = 1.0e-3;
    constexpr double kPlant = 1.0e-5;
    const int sub = static_cast<int>(std::lround(kBoard / kPlant));
    Trace trace;
    double lastSign = 0.0;
    for (double t = 0.0; t < seconds; t += kBoard) {
        const double u = positionDuty(p, s, target, 0.0, plant.theta, plant.omega, kBoard);
        trace.u = u;
        if (u != 0.0 && t > 0.5 * seconds) {
            const double sign = u > 0.0 ? 1.0 : -1.0;
            if (lastSign != 0.0 && sign != lastSign) {
                ++trace.signChanges;
            }
            lastSign = sign;
        }
        for (int i = 0; i < sub; ++i) {
            const double torque = idealTorque(p, u, plant.omega) + plant.external;
            plant.omega += torque / plant.inertia * kPlant;  // semi-implicit Euler
            plant.theta += plant.omega * kPlant;
            if (trace.timeToReach < 0.0 && plant.theta >= reach) {
                trace.timeToReach = t + (i + 1) * kPlant;
            }
        }
    }
    return trace;
}

TEST(Servo, PulseMapIsLinearAndNotHeldToItsEnds) {
    const ServoParams p = hobby();
    EXPECT_DOUBLE_EQ(pulseToAngle(p, 1.5e-3), 0.0);
    EXPECT_DOUBLE_EQ(pulseToAngle(p, 1.0e-3), -kPi / 4.0);
    EXPECT_DOUBLE_EQ(pulseToAngle(p, 2.0e-3), kPi / 4.0);
    // 0.2 ms past the end: 18° further, as a real servo goes.
    EXPECT_NEAR(pulseToAngle(p, 2.2e-3), kPi / 4.0 + 0.2 * kPi / 2.0, 1e-12);
    ServoParams flat = p;
    flat.pulseMax = flat.pulseMin;
    EXPECT_TRUE(std::isnan(pulseToAngle(flat, 1.5e-3)));
}

TEST(Servo, DeadbandAndSensingStep) {
    EXPECT_EQ(deadband(0.009, 0.01), 0.0);
    EXPECT_EQ(deadband(-0.01, 0.01), 0.0);
    EXPECT_NEAR(deadband(0.03, 0.01), 0.02, 1e-15);  // a dead zone: the error less the band
    EXPECT_NEAR(deadband(-0.03, 0.01), -0.02, 1e-15);
    EXPECT_DOUBLE_EQ(quantise(0.0123, 0.005), 0.01);
    EXPECT_DOUBLE_EQ(quantise(0.0123, 0.0), 0.0123);
    // XM430: 4096 pulses per turn, 0.088° each.
    const double step = 2.0 * kPi / 4096.0;
    EXPECT_NEAR(step * 180.0 / kPi, 0.088, 0.0005);
}

TEST(Servo, IdealTorqueIsTheLineThroughStallAndNoLoad) {
    const ServoParams p = hobby();
    EXPECT_DOUBLE_EQ(idealTorque(p, 1.0, 0.0), p.stallTorque);
    EXPECT_NEAR(idealTorque(p, 1.0, p.noLoadSpeed), 0.0, 1e-15);
    EXPECT_NEAR(idealTorque(p, 1.0, 0.5 * p.noLoadSpeed), 0.5 * p.stallTorque, 1e-15);
    EXPECT_NEAR(idealTorque(p, -1.0, 0.0), -p.stallTorque, 1e-15);
    EXPECT_NEAR(idealTorque(p, 0.5, 0.0), 0.5 * p.stallTorque, 1e-15);
    // Back-driven at a fifth of ω0 against full duty: 1.2 τ_stall of braking.
    EXPECT_NEAR(idealTorque(p, 1.0, -0.2 * p.noLoadSpeed), 1.2 * p.stallTorque, 1e-12);
}

// ROBOTIS e-Manual, XM430-W350 (read 2026-11-17): at 12.0 V a stall torque of 4.1 N·m at 2.3 A
// and 46 rev/min no load; at 11.1 V 3.8 N·m, 2.1 A, 43 rev/min; at 14.8 V 4.8 N·m, 2.7 A,
// 57 rev/min. Its gear is 353.5:1 (same page). A brushed motor fitted to the 12 V row: R from the
// stall current, N·Kt from the no-load speed, the gear's efficiency from the stall torque.
Armature xm430() {
    constexpr double kVolts = 12.0;
    constexpr double kStallCurrent = 2.3;
    constexpr double kStallTorque = 4.1;
    const double noLoad = 46.0 * kRpm;
    Armature a;
    a.resistance = kVolts / kStallCurrent;
    a.backEmf = kVolts / noLoad;                    // N·Ke = N·Kt
    a.torquePerAmp = kStallTorque / kStallCurrent;  // N·Kt·η
    a.maxCurrent = 0.0;
    return a;
}

TEST(Servo, SmartServoStallAndNoLoadAgainstItsDatasheet) {
    const Armature a = xm430();
    const double eta = a.torquePerAmp / a.backEmf;
    EXPECT_GT(eta, 0.0);
    EXPECT_LT(eta, 1.0);  // a gear that loses power: 0.716
    EXPECT_NEAR(stallTorqueOf(a, 12.0), 4.1, 0.02 * 4.1);
    EXPECT_NEAR(noLoadSpeedOf(a, 12.0) / kRpm, 46.0, 0.02 * 46.0);
    // The other rows follow from the same motor, linear in the voltage, within 2 %, but for the
    // stall torque at 14.8 V: the table gives 4.8 N·m (and 2.7 A), the linear motor 5.06 N·m
    // (2.84 A), 5 % more. A winding warmer at the higher current, or a supply limit, is not in
    // the model (TODO(verify): the datasheet does not say how its stall is measured).
    EXPECT_NEAR(stallTorqueOf(a, 11.1), 3.8, 0.02 * 3.8);
    EXPECT_NEAR(noLoadSpeedOf(a, 11.1) / kRpm, 43.0, 0.02 * 43.0);
    EXPECT_NEAR(noLoadSpeedOf(a, 14.8) / kRpm, 57.0, 0.02 * 57.0);
    EXPECT_NEAR(stallTorqueOf(a, 14.8), 4.8, 0.06 * 4.8);
    // The ideal fidelity with the datasheet's two numbers agrees with this motor's line
    // τ = N·Kt·η·(V − N·Ke·ω)/R at full duty within 2 % (exactly, with no other losses).
    ServoParams ideal;
    ideal.stallTorque = 4.1;
    ideal.noLoadSpeed = 46.0 * kRpm;
    for (const double fraction : {0.0, 0.25, 0.5, 0.75, 0.95}) {
        const double w = fraction * ideal.noLoadSpeed;
        const double motor = a.torquePerAmp * (12.0 - a.backEmf * w) / a.resistance;
        EXPECT_NEAR(idealTorque(ideal, 1.0, w), motor, 0.02 * ideal.stallTorque) << fraction;
    }
}

TEST(Servo, StepResponseAtFullDutyFollowsTheMotorsClosedForm) {
    // Saturated (u = 1) the joint obeys J ω̇ = τs (1 − ω/ω0): ω = ω0 (1 − e^(−t/τm)),
    // θ = ω0 (t − τm (1 − e^(−t/τm))), τm = J ω0 / τs. It stays saturated until the error is
    // under 1/Kp; the time to reach that point is checked against the closed form within 1 %.
    const ServoParams p = hobby();
    Plant plant;
    const double target = kPi / 3.0;  // a 60° step
    const double reach = target - 1.0 / p.kp;
    const Trace trace = run(p, plant, target, 1.0, reach);
    ASSERT_GT(trace.timeToReach, 0.0);
    const double tm = plant.inertia * p.noLoadSpeed / p.stallTorque;
    // Solve θ(t) = reach by Newton.
    double t = reach / p.noLoadSpeed + tm;
    for (int i = 0; i < 50; ++i) {
        const double theta = p.noLoadSpeed * (t - tm * (1.0 - std::exp(-t / tm)));
        const double omega = p.noLoadSpeed * (1.0 - std::exp(-t / tm));
        t -= (theta - reach) / omega;
    }
    EXPECT_NEAR(trace.timeToReach, t, 0.01 * t);
    // With so small a load the whole 60° takes about the datasheet's 0.12 s (it accelerates in
    // τm = 0.6 ms): within 5 %.
    EXPECT_NEAR(trace.timeToReach, 0.12, 0.05 * 0.12);
    // And it settles on the goal (no deadband: within a hundredth of a degree).
    EXPECT_NEAR(plant.theta, target, 2e-4);
}

TEST(Servo, HoldsATorqueByItsStiffnessAndYieldsPastStall) {
    // Held against τ_ext under its stall torque, the servo sits where Kp·e·τs = τ_ext: an error
    // of τ_ext / (Kp τs), without integral action.
    const ServoParams p = hobby();
    {
        Plant plant;
        plant.external = -0.5 * p.stallTorque;
        run(p, plant, 0.0, 1.0);
        const double expected = 0.5 * p.stallTorque / (p.kp * p.stallTorque);  // 0.025 rad
        EXPECT_NEAR(-plant.theta, expected, 0.01 * expected);
        EXPECT_NEAR(plant.omega, 0.0, 1e-6);
    }
    {
        // Just under stall it still holds, at an error past full duty's 1/Kp.
        Plant plant;
        plant.external = -0.95 * p.stallTorque;
        run(p, plant, 0.0, 1.0);
        EXPECT_NEAR(-plant.theta, 0.95 / p.kp, 0.01 * 0.95 / p.kp);
    }
    {
        // Past stall it is back-driven at the speed where the line brakes the excess:
        // τs (1 − ω/ω0) = 1.2 τs at ω = −0.2 ω0.
        Plant plant;
        plant.external = -1.2 * p.stallTorque;
        run(p, plant, 0.0, 0.5);
        EXPECT_NEAR(plant.omega, -0.2 * p.noLoadSpeed, 0.01 * 0.2 * p.noLoadSpeed);
    }
}

TEST(Servo, DeadbandStopsWithoutHunting) {
    ServoParams p = hobby();
    p.deadband = 0.02;  // about 1.1°
    {
        // A step inside the deadband: nothing moves.
        Plant plant;
        const Trace trace = run(p, plant, 0.5 * p.deadband, 0.2);
        EXPECT_EQ(plant.theta, 0.0);
        EXPECT_EQ(trace.u, 0.0);
    }
    {
        // A 20° step: it stops within the deadband, at rest, and the drive stays off.
        Plant plant;
        const double target = 20.0 * kPi / 180.0;
        const Trace trace = run(p, plant, target, 1.0);
        // (Unloaded it creeps up to the band's edge, the duty going to zero with the error.)
        EXPECT_LE(std::abs(target - plant.theta), p.deadband + 1e-9);
        EXPECT_NEAR(plant.omega, 0.0, 1e-9);
        EXPECT_NEAR(trace.u, 0.0, 1e-9);
        EXPECT_EQ(trace.signChanges, 0);
    }
    {
        // Without a deadband and with too much gain for its 1 ms loop it hunts: the test above
        // is not passing for want of a way to fail.
        ServoParams hot = hobby();
        hot.kp = 4000.0;
        Plant plant;
        plant.inertia = 2.0e-6;
        LoopState loop;
        const Trace trace = run(hot, plant, kPi / 8.0, 0.5, 1e9, &loop);
        EXPECT_GT(trace.signChanges, 10);
    }
}

TEST(Servo, DeadbandHoldsALoadAtItsEdge) {
    // Under a load the servo sits at the band's edge plus the error the load needs:
    // e = band + τ_ext / (Kp τs), continuous, so it holds still rather than chattering in and out.
    ServoParams p = hobby();
    p.deadband = 0.02;
    Plant plant;
    plant.external = -0.3 * p.stallTorque;
    const Trace trace = run(p, plant, 0.0, 1.0);
    const double expected = p.deadband + 0.3 / p.kp;
    EXPECT_NEAR(-plant.theta, expected, 0.01 * expected);
    EXPECT_EQ(trace.signChanges, 0);
    EXPECT_NEAR(plant.omega, 0.0, 1e-6);
}

TEST(Servo, ProfileIsTrapezoidal) {
    // Dynamixel's Profile Velocity and Acceleration: the set point accelerates at a to v, cruises,
    // and brakes at a onto the goal; the move takes d/v + v/a.
    const double v = 2.0;   // rad/s
    const double a = 10.0;  // rad/s²
    const double goal = 1.5;
    const double dt = 1.0e-3;
    Profile s;
    double peak = 0.0;
    double t = 0.0;
    double firstSlope = 0.0;
    while ((s.position != goal || t == 0.0) && t < 5.0) {
        const double before = s.velocity;
        stepProfile(s, goal, v, a, dt);
        t += dt;
        if (t <= 0.1 + 1e-9) {
            firstSlope = (s.velocity - before) / dt;
        }
        peak = std::max(peak, s.velocity);
    }
    EXPECT_NEAR(peak, v, 0.01 * v);
    EXPECT_NEAR(firstSlope, a, 0.01 * a);
    EXPECT_NEAR(t, goal / v + v / a, 0.01 * (goal / v + v / a));
    EXPECT_EQ(s.position, goal);
    EXPECT_EQ(s.velocity, 0.0);
    // A short move is triangular: peak √(a d) under the limit, time 2√(d/a).
    Profile shortMove;
    double tShort = 0.0;
    double peakShort = 0.0;
    while ((shortMove.position != 0.1 || tShort == 0.0) && tShort < 5.0) {
        stepProfile(shortMove, 0.1, v, a, dt);
        tShort += dt;
        peakShort = std::max(peakShort, shortMove.velocity);
    }
    EXPECT_NEAR(peakShort, std::sqrt(a * 0.1), 0.02 * std::sqrt(a * 0.1));
    EXPECT_NEAR(tShort, 2.0 * std::sqrt(0.1 / a), 0.02 * 2.0 * std::sqrt(0.1 / a));
    // No limits: a step.
    Profile step;
    stepProfile(step, 0.7, 0.0, 0.0, dt);
    EXPECT_EQ(step.position, 0.7);
}

TEST(Servo, CurrentLimitAndCurrentMode) {
    Armature a = xm430();
    a.maxCurrent = 1.0;
    double uMin = 0.0;
    double uMax = 0.0;
    // At rest the duty that gives 1 A is R·1/V.
    currentLimitedDuty(a, 12.0, 0.0, uMin, uMax);
    EXPECT_NEAR(uMax, a.resistance / 12.0, 1e-12);
    EXPECT_NEAR(uMin, -a.resistance / 12.0, 1e-12);
    // Turning, the back-EMF moves the window: the current at either end is ±1 A.
    const double w = 2.0;
    currentLimitedDuty(a, 12.0, w, uMin, uMax);
    EXPECT_NEAR((uMax * 12.0 - a.backEmf * w) / a.resistance, 1.0, 1e-12);
    EXPECT_NEAR((uMin * 12.0 - a.backEmf * w) / a.resistance, -1.0, 1e-12);
    // Current mode drives the asked current through the armature at any speed.
    const double u = currentDuty(a, 12.0, 0.5, w);
    EXPECT_NEAR((u * 12.0 - a.backEmf * w) / a.resistance, 0.5, 1e-12);
    // No limit: the full duty.
    a.maxCurrent = 0.0;
    currentLimitedDuty(a, 12.0, w, uMin, uMax);
    EXPECT_EQ(uMin, -1.0);
    EXPECT_EQ(uMax, 1.0);
}

TEST(Servo, VelocityLoopReachesItsGoal) {
    ServoParams p = hobby();
    p.velocityKp = 0.02;  // duty per rad/s: a loop gain of 0.3 per 1 ms sample on this inertia
    p.velocityKi = 2.0;
    LoopState s;
    double omega = 0.0;
    const double inertia = 2.0e-5;
    for (int k = 0; k < 2000; ++k) {
        const double u = velocityDuty(p, s, 4.0, omega, 1.0e-3);
        for (int i = 0; i < 100; ++i) {
            omega += idealTorque(p, u, omega) / inertia * 1.0e-5;
        }
    }
    EXPECT_NEAR(omega, 4.0, 1e-3);  // the integral takes the line's droop out
}

TEST(Servo, OverloadShutsDownAfterItsTimeAndOverheatingAtItsTemperature) {
    ServoParams p;
    p.overloadTime = 0.5;
    p.maxTemperature = 353.15;
    Shutdown s;
    for (int k = 0; k < 499; ++k) {
        stepShutdown(p, s, true, 300.0, 1.0e-3);
    }
    EXPECT_FALSE(s.tripped());
    stepShutdown(p, s, false, 300.0, 1.0e-3);  // a break resets the count
    for (int k = 0; k < 499; ++k) {
        stepShutdown(p, s, true, 300.0, 1.0e-3);
    }
    EXPECT_FALSE(s.tripped());
    stepShutdown(p, s, true, 300.0, 1.0e-3);
    EXPECT_TRUE(s.overload);
    Shutdown hot;
    stepShutdown(p, hot, false, 353.2, 1.0e-3);
    EXPECT_TRUE(hot.overheated);
    ServoParams never;
    Shutdown none;
    for (int k = 0; k < 10000; ++k) {
        stepShutdown(never, none, true, 1000.0, 1.0e-3);
    }
    EXPECT_FALSE(none.tripped());
}

}  // namespace
}  // namespace lagrange::actuators
