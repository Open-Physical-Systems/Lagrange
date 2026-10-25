// Blade-element momentum theory against momentum theory and against measured full-scale
// propeller data (NACA Report 640), aerodynamics.md §7.
#include "lagrange/rotor_aero/Propeller.hpp"

#include <cmath>
#include <numbers>
#include <span>
#include <string>

#include <gtest/gtest.h>

namespace lagrange::rotor_aero {
namespace {

constexpr double kDeg = std::numbers::pi / 180.0;
const Air kSeaLevel{1.225, 0.0};

Propeller naca5868(double bladeAngle) {
    std::string error;
    const Propeller p =
        loadPropeller(std::string(LAGRANGE_PROPELLERS) + "/naca_5868-9.prop.yaml", error);
    EXPECT_TRUE(error.empty()) << error;
    return withBladeAngle(p, (bladeAngle - 25.0) * kDeg);
}

/// Measured CT and CP of propeller 5868-9 (2 blades, Clark Y), read off NACA Report 640,
/// figures 8 (CT) and 6 (CP) to about ±0.001 (aerodynamics.md §7.1). Only the attached-flow
/// range: below J ≈ 0.5 at 25° the inner blade stalls and the measured propeller keeps more
/// thrust than blade-element theory gives (stall delay on rotating blades, not modelled).
struct Point {
    double j, ct, cp;
};
constexpr Point kAt15[] = {{0.3, 0.0776, 0.0393},
                           {0.4, 0.0637, 0.0370},
                           {0.5, 0.0498, 0.0320},
                           {0.6, 0.0350, 0.0260},
                           {0.7, 0.0192, 0.0170}};
constexpr Point kAt25[] = {{0.5, 0.1031, 0.0856}, {0.6, 0.0966, 0.0830}, {0.7, 0.0871, 0.0795},
                           {0.8, 0.0750, 0.0730}, {0.9, 0.0605, 0.0650}, {1.0, 0.0450, 0.0540},
                           {1.1, 0.0305, 0.0380}};

void compare(const Propeller& p, std::span<const Point> points) {
    for (const Point& m : points) {
        // Tip speed of the tests (115 mph tunnel at these advance ratios): about 150 m/s; no
        // compressibility correction, as the report's coefficients have none.
        const PropellerCoefficients c = coefficientsAt(p, m.j, 150.0, kSeaLevel);
        EXPECT_NEAR(c.ct, m.ct, std::max(0.08 * m.ct, 0.003)) << "J = " << m.j;
        EXPECT_NEAR(c.cp, m.cp, std::max(0.06 * m.cp, 0.003)) << "J = " << m.j;
    }
}

TEST(NacaReport640, ThrustAndPowerAt15Degrees) {
    compare(naca5868(15.0), kAt15);
}

TEST(NacaReport640, ThrustAndPowerAt25Degrees) {
    compare(naca5868(25.0), kAt25);
}

TEST(NacaReport640, ZeroThrustAdvanceRatios) {
    // Figure 8: thrust vanishes at J = 0.82 (15°) and 1.29 (25°).
    for (const auto [angle, zero] : {std::pair{15.0, 0.82}, std::pair{25.0, 1.29}}) {
        const Propeller p = naca5868(angle);
        EXPECT_GT(coefficientsAt(p, zero - 0.03, 150.0, kSeaLevel).ct, 0.0) << angle;
        EXPECT_LT(coefficientsAt(p, zero + 0.03, 150.0, kSeaLevel).ct, 0.0) << angle;
    }
    // Beyond it the propeller windmills: the air drives it.
    EXPECT_LT(coefficientsAt(naca5868(15.0), 1.2, 150.0, kSeaLevel).cp, 0.0);
}

/// A rotor of many thin blades with ideal twist (blade angle ∝ 1/r) and no profile drag has
/// uniform inflow: its induced velocity and power are those of momentum theory,
/// v = √(T / 2ρA) and P = T·v, A the swept annulus.
TEST(Bem, IdealRotorMeetsMomentumTheory) {
    Propeller p;
    p.radius = 1.0;
    p.blades = 40;  // tip loss vanishes with many blades
    p.hand = "ccw";
    Aerofoil thin{2.0 * std::numbers::pi, 0.0, 1.5, -1.5, 0.0, 0.0, 0.0};
    for (double r = 0.2; r <= 1.0001; r += 0.05) {
        p.stations.push_back({r, 0.005, 0.12 / r, 0.1, thin});
    }
    const double omega = 100.0;
    const BemLoad l = bemLoad(p, omega, 0.0, kSeaLevel);
    const double area = std::numbers::pi * (1.0 - 0.2 * 0.2);
    const double v = std::sqrt(l.thrust / (2.0 * kSeaLevel.density * area));
    ASSERT_GT(l.thrust, 0.0);
    EXPECT_NEAR(l.inducedVelocity, v, 0.03 * v);
    EXPECT_NEAR(l.torque * omega, l.thrust * v, 0.04 * l.thrust * v);  // figure of merit ≈ 1
    EXPECT_FALSE(l.vortexRing);
}

TEST(Bem, TurningTheWrongWayGivesLittleThrustForMorePower) {
    const Propeller p = naca5868(15.0);
    const double omega = 50.0;  // rad/s, tip speed 76 m/s
    const BemLoad right = bemLoad(p, omega, 0.0, kSeaLevel);
    const BemLoad wrong = bemLoad(p, -omega, 0.0, kSeaLevel);
    EXPECT_GT(right.thrust, 0.0);
    EXPECT_LT(std::abs(wrong.thrust), 0.5 * right.thrust);
    // Power is absorbed either way, more of it per newton the wrong way.
    EXPECT_GT(right.torque * omega, 0.0);
    EXPECT_GT(wrong.torque * -omega, 0.0);
    EXPECT_GT(wrong.torque * -omega / std::abs(wrong.thrust),
              1.5 * right.torque * omega / right.thrust);
}

TEST(Bem, SlowDescentIsTheVortexRingState) {
    const Propeller p = naca5868(15.0);
    const BemLoad hover = bemLoad(p, 50.0, 0.0, kSeaLevel);
    EXPECT_FALSE(hover.vortexRing);
    EXPECT_TRUE(bemLoad(p, 50.0, -0.5 * hover.inducedVelocity, kSeaLevel).vortexRing);
}

TEST(Section, LinearThenStalledThenAFlatPlate) {
    const Aerofoil a{5.787, -5.3 * kDeg, 1.45, -0.5, 0.0093, 0.2, 0.01};
    EXPECT_NEAR(sectionCoefficients(a, -5.3 * kDeg, 0.0).cl, 0.0, 1e-12);
    EXPECT_NEAR(sectionCoefficients(a, 4.7 * kDeg, 0.0).cl, 5.787 * 10.0 * kDeg, 1e-9);
    EXPECT_NEAR(sectionCoefficients(a, -5.3 * kDeg, 0.0).cd, 0.0093 + 0.01 * 0.04, 1e-12);
    // Prandtl–Glauert: lift slope up by 1/√(1 − M²).
    EXPECT_NEAR(sectionCoefficients(a, 0.0, 0.6).cl / sectionCoefficients(a, 0.0, 0.0).cl, 1.25,
                1e-9);
    // Continuous through stall, then a flat plate: no lift at 90°, drag 2 + cd_min.
    const double stall = -5.3 * kDeg + 1.45 / 5.787;
    EXPECT_NEAR(sectionCoefficients(a, stall + 1e-9, 0.0).cl, 1.45, 1e-6);
    EXPECT_NEAR(sectionCoefficients(a, 90.0 * kDeg, 0.0).cl, 0.0, 1e-9);
    EXPECT_NEAR(sectionCoefficients(a, 90.0 * kDeg, 0.0).cd, 2.0 + 0.0093, 1e-9);
    // Reversed flow (the trailing edge first) is a flat plate too.
    EXPECT_NEAR(sectionCoefficients(a, 175.0 * kDeg, 0.0).cl,
                2.0 * std::sin(175.0 * kDeg) * std::cos(175.0 * kDeg), 1e-9);
}

TEST(PropellerTable, GivesWhatBemGives) {
    const Propeller p = naca5868(15.0);
    const PropellerTable table = PropellerTable::build(p, false);
    for (const double omega : {80.0, -30.0, 0.0}) {
        for (const double v : {0.0, 20.0, 70.0, -4.0}) {
            const BemLoad direct = bemLoad(p, omega, v, kSeaLevel);
            const BemLoad looked = table.at(omega, v, kSeaLevel);
            const double scale =
                std::abs(direct.thrust) + 0.01 * kSeaLevel.density * (omega * omega * 2.3 + v * v);
            EXPECT_NEAR(looked.thrust, direct.thrust, 0.01 * scale) << omega << " " << v;
            EXPECT_NEAR(looked.torque, direct.torque, 0.01 * (std::abs(direct.torque) + scale))
                << omega << " " << v;
        }
    }
    // With compressibility: the table's Mach rows against the direct solution.
    const PropellerTable fast = PropellerTable::build(p, true);
    const Air air{1.225, 340.0};
    const BemLoad direct = bemLoad(p, 150.0, 30.0, air);  // tip Mach 0.68
    EXPECT_NEAR(fast.at(150.0, 30.0, air).thrust, direct.thrust, 0.01 * direct.thrust);
    EXPECT_GT(direct.thrust, bemLoad(p, 150.0, 30.0, kSeaLevel).thrust);
}

TEST(Propeller, RejectsAnIncompleteFile) {
    std::string error;
    const Propeller p = loadPropeller("/nonexistent.prop.yaml", error);
    EXPECT_EQ(p.radius, 0.0);
    EXPECT_FALSE(error.empty());
}

}  // namespace
}  // namespace lagrange::rotor_aero
