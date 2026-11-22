// The holding brake against its closed forms: the coil's R–L rise and freewheeling decay give the
// release time −τ·ln(1 − I_r/I_0) after power on and the engage time −τ·ln(I_d/I_0) after power
// off; the armature's hysteresis; static and dynamic friction.
#include "lagrange/actuators/Brake.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace lagrange::actuators {
namespace {

// The sample's brake (assets/robots/brake_arm): illustrative, not a product's.
BrakeParams sample() {
    BrakeParams p;
    p.holding = 5.0;
    p.dynamic = 4.0;
    p.coilResistance = 48.0;
    p.coilInductance = 0.5;
    p.ratedVoltage = 24.0;
    p.releaseCurrent = 0.35;
    p.dropoutCurrent = 0.15;
    return p;
}

/// The time of the first step after which `done(state)`, stepping `dt` at `volts`.
template <typename Done>
double timeUntil(const BrakeParams& p, BrakeState& s, double volts, double dt, Done done) {
    for (int k = 1; k < 100000; ++k) {
        stepBrake(p, s, volts, dt);
        if (done(s)) {
            return k * dt;
        }
    }
    return -1.0;
}

TEST(Brake, ReleaseAndEngageTimesAreTheCoilsClosedForms) {
    const BrakeParams p = sample();
    const double tau = p.coilInductance / p.coilResistance;  // 10.4 ms
    const double full = p.ratedVoltage / p.coilResistance;   // 0.5 A
    const double release = -tau * std::log(1.0 - p.releaseCurrent / full);
    const double engage = -tau * std::log(p.dropoutCurrent / full);
    for (const double dt : {1e-5, 1e-4, 1e-3}) {
        BrakeState s;
        EXPECT_TRUE(s.engaged);
        const double up =
            timeUntil(p, s, p.ratedVoltage, dt, [](const BrakeState& b) { return !b.engaged; });
        // Within one step: the switch is seen at the end of the step it happens in.
        EXPECT_GE(up, release - 1e-12) << "dt " << dt;
        EXPECT_LT(up, release + dt + 1e-12) << "dt " << dt;
        // Settled, then off.
        for (int k = 0; k < static_cast<int>(20.0 * tau / dt); ++k) {
            stepBrake(p, s, p.ratedVoltage, dt);
        }
        EXPECT_NEAR(s.current, full, 1e-6);
        const double down = timeUntil(p, s, 0.0, dt, [](const BrakeState& b) { return b.engaged; });
        EXPECT_GE(down, engage - 1e-12) << "dt " << dt;
        EXPECT_LT(down, engage + dt + 1e-12) << "dt " << dt;
    }
    std::printf("brake: release %.2f ms, engage %.2f ms\n", release * 1e3, engage * 1e3);
}

TEST(Brake, TheArmatureHasHysteresis) {
    const BrakeParams p = sample();
    BrakeState s;
    // Held at a current between dropout and release: it stays as it was.
    s.current = 0.25;
    stepBrake(p, s, 0.25 * p.coilResistance, 1e-3);
    EXPECT_TRUE(s.engaged);  // not enough to pull in
    s.engaged = false;
    stepBrake(p, s, 0.25 * p.coilResistance, 1e-3);
    EXPECT_FALSE(s.engaged);  // enough to stay off
}

TEST(Brake, StaticAtRestDynamicWhileSlippingNoneWhenReleased) {
    const BrakeParams p = sample();
    BrakeState s;
    EXPECT_EQ(brakeFriction(p, s, 0.0), 5.0);
    EXPECT_EQ(brakeFriction(p, s, -0.2), 4.0);
    s.engaged = false;
    EXPECT_EQ(brakeFriction(p, s, 0.0), 0.0);
}

}  // namespace
}  // namespace lagrange::actuators
