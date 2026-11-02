// Rotor downwash and outwash against their references (Downwash.hpp; rotor_aero.md §2):
// actuator-disc momentum theory, the drone jet measured by Bauersfeld et al. (2024), and the
// outwash measured by Tanner et al. (2015) and in flight tests they quote.
#include "lagrange/aerodynamics/Aerodynamics.hpp"
#include "lagrange/aerodynamics/Downwash.hpp"

#include <cmath>
#include <numbers>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::aerodynamics {
namespace {

using Vec3 = Downwash::Vec3;

constexpr double kRho = 1.2;       // kg/m³, the air Bauersfeld et al. computed U_H with
constexpr double kGravity = 9.81;  // m/s², theirs

double inducedOf(double thrust, double radius) {
    return std::sqrt(thrust / (2.0 * kRho * std::numbers::pi * radius * radius));
}

/// Bauersfeld et al.'s "Offboard 1": 0.572 kg, 12.95 cm propellers, 26.6 cm motor distance,
/// U_H = 6.66 m/s (their Table I). Four rotors in an X, hovering at `height`, group 0.
std::vector<Downwash::Rotor> offboard(double height, std::size_t group = 0, double x = 0.0) {
    constexpr double kMass = 0.572;
    constexpr double kRadius = 0.1295 / 2.0;
    constexpr double kHalfDiagonal = 0.266 / 2.0;
    std::vector<Downwash::Rotor> rotors;
    for (int i = 0; i < 4; ++i) {
        const double a = std::numbers::pi / 4.0 + i * std::numbers::pi / 2.0;
        Downwash::Rotor r;
        r.hub = {x + kHalfDiagonal * std::cos(a), kHalfDiagonal * std::sin(a), height};
        r.thrustAxis = {0.0, 0.0, 1.0};
        r.radius = kRadius;
        r.thrust = kMass * kGravity / 4.0;
        r.induced = inducedOf(r.thrust, kRadius);
        r.group = group;
        rotors.push_back(r);
    }
    return rotors;
}

TEST(Downwash, NearWakeIsTheActuatorDisc) {
    // Momentum theory: v_i at the disc, 2 v_i far below, the slipstream contracted to R/√2.
    EXPECT_NEAR(nearWakeSpeed(5.0, 0.1, 1e-12), 5.0, 1e-9);
    EXPECT_NEAR(nearWakeSpeed(5.0, 0.1, 100.0), 10.0, 1e-5);
    EXPECT_NEAR(nearWakeRadius(5.0, 0.1, 100.0), 0.1 / std::numbers::sqrt2, 1e-6);
    // One radius below: (1 + 1/√2) v_i.
    EXPECT_NEAR(nearWakeSpeed(5.0, 0.1, 0.1), 5.0 * (1.0 + 1.0 / std::numbers::sqrt2), 1e-12);
}

TEST(Downwash, HoverInducedVelocityIsBauersfelds) {
    Downwash d;
    d.update(offboard(10.0));
    ASSERT_EQ(d.groups().size(), 1U);
    EXPECT_NEAR(d.groups()[0].hoverInduced, 6.66, 0.01 * 6.66);  // their Table I
    EXPECT_NEAR(d.groups()[0].length, 0.266, 1e-9);
}

TEST(Downwash, FarJetFollowsTheMeasuredDecayAndSpread) {
    Downwash d;
    d.update(offboard(10.0));
    const double uh = d.groups()[0].hoverInduced;
    const double l = 0.266;
    for (const double sl : {2.5, 4.0, 6.0, 10.0}) {
        const double z = 10.0 - sl * l;
        // On the axis: U_C = U_H·10.11/(s/l + 5.817) (their eqs. 5 and 9).
        const Vec3 v = d.velocityAt({0.0, 0.0, z});
        EXPECT_NEAR(-v[2], uh * 10.11 / (sl + 5.817), 1e-9) << sl;
        // At the half-width r½ = 0.07668 (s + 5.817 l): half the centreline speed.
        const double half = 0.07668 * (sl + 5.817) * l;
        const Vec3 w = d.velocityAt({half, 0.0, z});
        EXPECT_NEAR(w[2] / v[2], 0.5, 1e-9) << sl;
    }
}

TEST(Downwash, FarJetCarriesTheThrustAsMomentum) {
    // Not fitted: a free jet's momentum flux ∫ρu²dA equals the thrust that made it. Bauersfeld
    // et al.'s decay and spread constants, fitted separately, give that within a few per cent.
    Downwash d;
    const auto rotors = offboard(10.0);
    d.update(rotors);
    const double thrust = 0.572 * kGravity;
    const double l = 0.266;
    for (const double sl : {3.0, 6.0, 12.0}) {
        const double z = 10.0 - sl * l;
        const double half = farJetHalfWidth(l, sl * l);
        double flux = 0.0;
        constexpr int kRings = 4000;
        const double dr = 12.0 * half / kRings;
        for (int i = 0; i < kRings; ++i) {
            const double r = (i + 0.5) * dr;
            const double u = -d.velocityAt({r, 0.0, z})[2];
            flux += kRho * u * u * 2.0 * std::numbers::pi * r * dr;
        }
        EXPECT_NEAR(flux / thrust, 1.0, 0.05) << sl;
    }
}

TEST(Downwash, OwnGroupIsLeftOut) {
    Downwash d;
    d.update(offboard(10.0));
    const Vec3 v = d.velocityAt({0.0, 0.0, 9.0}, 0);
    EXPECT_EQ(v[2], 0.0);
    EXPECT_LT(d.velocityAt({0.0, 0.0, 9.0})[2], 0.0);
    // Nothing above the rotors.
    EXPECT_EQ(d.velocityAt({0.0, 0.0, 10.5})[2], 0.0);
}

TEST(Downwash, OutwashMatchesTheModelRotorAndFlightTests) {
    // A single rotor 1 R above the ground (Tanner et al. 2015): peak mean outwash 1.9 V_h at
    // 1.7–1.8 R, 0.02–0.08 R above the ground; at 1.5 R flight tests found 1.7 V_h (CH-47D) to
    // 2.1 V_h (CH-53E), 1.9 V_h for the HH-60H and V-22 (the paper's Refs. 14, 16, 40, 41).
    constexpr double kR = 1.0;
    constexpr double kVh = 5.0;
    Downwash d;
    d.setGround({0.0, 0.0, 0.0}, {0.0, 0.0, 1.0});
    Downwash::Rotor r;
    r.hub = {0.0, 0.0, kR};
    r.thrustAxis = {0.0, 0.0, 1.0};
    r.radius = kR;
    r.induced = kVh;
    r.thrust = 1.0;
    d.update(std::span<const Downwash::Rotor>(&r, 1));
    double best = 0.0;
    double where = 0.0;
    double height = 0.0;
    for (int i = 1; i <= 300; ++i) {
        for (int k = 1; k <= 60; ++k) {
            const double x = i * 0.01;
            const double z = k * 0.005;
            const double u = d.velocityAt({x, 0.0, z})[0];
            if (u > best) {
                best = u;
                where = x;
                height = z;
            }
        }
    }
    EXPECT_NEAR(best / kVh, 1.9, 0.05);
    EXPECT_GE(where, 1.65);
    EXPECT_LE(where, 1.85);
    EXPECT_GE(height, 0.02);
    EXPECT_LE(height, 0.08);
    double at15 = 0.0;
    for (int k = 1; k <= 60; ++k) {
        at15 = std::max(at15, d.velocityAt({1.5, 0.0, k * 0.005})[0]);
    }
    EXPECT_GE(at15 / kVh, 1.7 * 0.97);
    EXPECT_LE(at15 / kVh, 2.1);
    // Above the layer (0.15 R deep at the peak) it has halved.
    EXPECT_LT(d.velocityAt({1.7, 0.0, 0.16})[0], 0.5 * best + 0.05 * kVh);
    EXPECT_NEAR(d.peakOutwash() / kVh, 1.9, 0.05);
    // No downward flow through the ground.
    EXPECT_NEAR(d.velocityAt({0.0, 0.0, 0.0})[2], 0.0, 1e-12);
}

TEST(Downwash, WakeLeansWithTheWind) {
    // Leishman's skew: tan χ = V/v_i for a hovering rotor in a crosswind V.
    Downwash d;
    const auto rotors = offboard(10.0);
    const double vi = rotors[0].induced;
    const std::vector<Vec3> air{{vi, 0.0, 0.0}};  // the air going past at v_i: χ = 45°
    d.update(rotors, air);
    const Vec3& a = d.groups()[0].wakeAxis;
    EXPECT_NEAR(std::atan2(a[0], -a[2]), std::numbers::pi / 4.0, 1e-3);
    // Its centreline is downwind now.
    const Vec3 under = d.velocityAt({0.0, 0.0, 9.0});
    const Vec3 downwind = d.velocityAt({1.0, 0.0, 9.0});
    EXPECT_LT(downwind[2], under[2]);
}

TEST(Downwash, WindAtAddsTheProfileGustAndWakes) {
    Wind wind;
    wind.speed = 5.0;
    wind.fromDegrees = 270.0;  // from the west: blows towards +x
    wind.referenceHeight = 10.0;
    wind.roughnessLength = 0.03;
    Downwash d;
    d.update(offboard(10.0));
    const Vec3 x{0.0, 0.0, 9.0};
    const Vec3 v = windAt(wind, 0.0, {0.0, 1.0, 0.0}, d, x);
    EXPECT_NEAR(v[0], meanWindAt(wind, 9.0), 1e-9);
    EXPECT_NEAR(v[1], 1.0, 1e-9);
    EXPECT_NEAR(v[2], d.velocityAt(x)[2], 1e-12);
}

TEST(Downwash, ARoofStopsTheWakeAndCarriesTheOutwash) {
    // A 4 m × 4 m building 10 m tall, the drone 1 m over its roof.
    Downwash d;
    d.setGround({0.0, 0.0, 0.0}, {0.0, 0.0, 1.0});
    d.setObstacles({Downwash::Box{{0.0, 0.0, 5.0}, {1.0, 0.0, 0.0, 0.0}, {2.0, 2.0, 5.0}}});
    d.update(offboard(11.0));
    ASSERT_TRUE(d.groups()[0].hits);
    EXPECT_EQ(d.groups()[0].hitBox, 0);
    EXPECT_NEAR(d.groups()[0].hitPoint[2], 10.0, 1e-9);
    EXPECT_LT(d.velocityAt({0.0, 0.0, 10.5})[2], -1.0);  // the jet above the roof
    EXPECT_EQ(d.velocityAt({0.0, 0.0, 9.0})[2], 0.0);    // inside: still
    EXPECT_EQ(d.velocityAt({3.0, 0.0, 5.0})[2], 0.0);    // beside it, under roof level
    EXPECT_GT(d.velocityAt({0.8, 0.0, 10.02})[0], 0.5);  // outwash over the roof
    EXPECT_EQ(d.velocityAt({2.5, 0.0, 10.02})[0], 0.0);  // past the roof's edge
    EXPECT_EQ(d.peakOutwash(), 0.0);                     // no dust lifts from a roof
}

TEST(Downwash, AWallBesideTheWakeMirrorsIt) {
    // A wall 0.6 m from the drone's axis (its face at x = 0.6), 20 m tall and wide.
    Downwash open;
    open.update(offboard(10.0));
    Downwash walled;
    walled.setObstacles({Downwash::Box{{5.6, 0.0, 10.0}, {1.0, 0.0, 0.0, 0.0}, {5.0, 10.0, 10.0}}});
    walled.update(offboard(10.0));
    ASSERT_EQ(walled.groups()[0].walls.size(), 1U);
    const Vec3 nearWall{0.55, 0.0, 9.0};
    // The image jet adds to the real one next to the wall, where the air cannot go through.
    EXPECT_LT(walled.velocityAt(nearWall)[2], open.velocityAt(nearWall)[2] - 0.1);
    EXPECT_EQ(walled.velocityAt({0.7, 0.0, 9.0})[2], 0.0);  // in the wall
    // Far on the other side, the wall hardly matters.
    EXPECT_NEAR(walled.velocityAt({-1.5, 0.0, 8.0})[2], open.velocityAt({-1.5, 0.0, 8.0})[2], 0.05);
    Vec3 normal{};
    EXPECT_NEAR(walled.nearestWall({0.0, 0.0, 10.0}, normal), 0.6, 1e-9);
    EXPECT_NEAR(normal[0], -1.0, 1e-12);
    EXPECT_TRUE(std::isinf(open.nearestWall({0.0, 0.0, 10.0}, normal)));
}

}  // namespace
}  // namespace lagrange::aerodynamics
