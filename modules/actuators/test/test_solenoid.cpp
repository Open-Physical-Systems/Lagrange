// RFC 0020 §4.6's solenoid against its closed forms: the pull ½·i²·dL/dx at three strokes, the
// coil current's rise V/R·(1 − e^(−t/τ)) with the plunger held, and the current's dip when the
// plunger moves in at a given flux (i·L kept, so i falls by L(x0)/L(x1)).
#include "lagrange/actuators/Solenoid.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace lagrange::actuators {
namespace {

// The sample's numbers (assets/robots/solenoid_bench): illustrative, not a product's.
SolenoidParams sample() {
    SolenoidParams p;
    p.position = {0.0, 0.002, 0.004, 0.006, 0.008, 0.009, 0.010};
    p.inductance = {0.030, 0.033, 0.037, 0.044, 0.057, 0.070, 0.090};
    p.resistance = 24.0;
    p.springRate = 5.0;
    p.springPreload = 0.1;
    p.stroke = 0.010;
    p.ratedVoltage = 12.0;
    return p;
}

/// The current after `seconds` of `volts` at x held, from none, in steps of `dt`.
double currentAfter(const SolenoidParams& p, double volts, double x, double seconds, double dt) {
    double flux = 0.0;
    const int steps = static_cast<int>(std::lround(seconds / dt));
    for (int k = 0; k < steps; ++k) {
        flux = stepSolenoidFlux(p, flux, volts, x, dt);
    }
    return flux / solenoidInductance(p, x);
}

TEST(Solenoid, PullAtThreeStrokesIsTheCoEnergysClosedForm) {
    const SolenoidParams p = sample();
    ASSERT_EQ(checkSolenoid(p), "");
    // Held at each stroke until the current settles (20 time constants), then the pull.
    for (const double x : {0.001, 0.005, 0.0095}) {
        const double tau = solenoidInductance(p, x) / p.resistance;
        const double i = currentAfter(p, p.ratedVoltage, x, 20.0 * tau, 1e-4);
        EXPECT_NEAR(i, p.ratedVoltage / p.resistance, 1e-6 * i);
        // The table's segment slope by hand: (L1 − L0)/(x1 − x0).
        const double slope = x < 0.002   ? (0.033 - 0.030) / 0.002
                             : x < 0.006 ? (0.044 - 0.037) / 0.002
                                         : (0.090 - 0.070) / 0.001;
        const double expected = 0.5 * 0.5 * 0.5 * slope;
        EXPECT_NEAR(solenoidPull(p, i, x), expected, 0.01 * expected) << "x " << x;
    }
    EXPECT_NEAR(solenoidSpring(p, 0.004), -(0.1 + 5.0 * 0.004), 1e-12);
}

TEST(Solenoid, CurrentRisesAsAnRlCircuitWithThePlungerHeld) {
    const SolenoidParams p = sample();
    const double x = 0.0;
    const double tau = solenoidInductance(p, x) / p.resistance;  // 1.25 ms
    for (const double dt : {1e-4, 1e-3}) {
        for (const double t : {0.5 * tau, tau, 3.0 * tau}) {
            // At a whole number of steps.
            const double seconds = std::round(t / dt) * dt;
            const double expected = 0.5 * (1.0 - std::exp(-seconds / tau));
            EXPECT_NEAR(currentAfter(p, 12.0, x, seconds, dt), expected, 0.01 * expected)
                << "dt " << dt << " t " << seconds;
        }
    }
}

TEST(Solenoid, MovingThePlungerInDropsTheCurrent) {
    const SolenoidParams p = sample();
    // Settled at rest, 0.5 A; then the plunger jumps to the stop in one step of 1 µs: the flux
    // stays, so the current falls by L(0)/L(stroke) = 0.03/0.09.
    double flux = 0.5 * solenoidInductance(p, 0.0);
    flux = stepSolenoidFlux(p, flux, 12.0, p.stroke, 1e-6);
    const double after = flux / solenoidInductance(p, p.stroke);
    EXPECT_NEAR(after, 0.5 * 0.030 / 0.090, 0.01 * 0.5 * 0.030 / 0.090);
    // Then it climbs back to V/R with the closed position's time constant.
    const double tau = 0.090 / 24.0;
    double i = after;
    for (int k = 0; k < static_cast<int>(10.0 * tau / 1e-4); ++k) {
        flux = stepSolenoidFlux(p, flux, 12.0, p.stroke, 1e-4);
        const double next = flux / solenoidInductance(p, p.stroke);
        EXPECT_GE(next, i);
        i = next;
    }
    EXPECT_NEAR(i, 0.5, 0.001);
    // Moving slowly (over many time constants) the current hardly dips.
    flux = 0.5 * solenoidInductance(p, 0.0);
    double lowest = 0.5;
    for (int k = 0; k <= 1000; ++k) {
        const double x = p.stroke * k / 1000.0;  // 10 mm in 1 s
        flux = stepSolenoidFlux(p, flux, 12.0, x, 1e-3);
        lowest = std::min(lowest, flux / solenoidInductance(p, x));
    }
    EXPECT_GT(lowest, 0.49);
}

TEST(Solenoid, BadParametersAreNamed) {
    SolenoidParams p = sample();
    p.position = {0.0, 0.008};
    p.inductance = {0.03, 0.05};
    EXPECT_NE(checkSolenoid(p).find("cover the stroke"), std::string::npos);
    p = sample();
    p.position[3] = p.position[2];
    EXPECT_NE(checkSolenoid(p).find("rise"), std::string::npos);
    p = sample();
    p.resistance = 0.0;
    EXPECT_NE(checkSolenoid(p).find("resistance"), std::string::npos);
    p = sample();
    p.inductance.pop_back();
    EXPECT_NE(checkSolenoid(p).find("points"), std::string::npos);
}

}  // namespace
}  // namespace lagrange::actuators
