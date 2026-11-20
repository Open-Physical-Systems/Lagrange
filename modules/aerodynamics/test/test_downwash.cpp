// Rotor downwash and outwash against their references (Downwash.hpp; rotor_aero.md §2):
// actuator-disc momentum theory, the drone jet measured by Bauersfeld et al. (2024), and the
// outwash measured by Tanner et al. (2015) and in flight tests they quote.
#include "lagrange/aerodynamics/Aerodynamics.hpp"
#include "lagrange/aerodynamics/Downwash.hpp"
#include "lagrange/gpu/GroundAir.hpp"

#include <array>
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

// The fountain's fit against the `lagrange foam-hover` runs it came from (rotor_aero.md §7):
// the change of the force on the sample quadcopter's frame from 2 m up, as a share of its 19.62 N
// thrust, at h/l = 1, 1.5, 2 and 3; within 0.25 N of each, nothing from 4 diagonals up.
TEST(Downwash, TheFountainIsTheHoverCasesFit) {
    constexpr double kThrust = 4.0 * 4.905;
    constexpr double kDiagonal = 0.5006;
    const std::vector<std::array<double, 2>> cases{
        {0.5, 0.634 + 1.861}, {0.75, -0.407 + 1.861}, {1.0, -1.336 + 1.861}, {1.5, -1.667 + 1.861}};
    for (const auto& [height, change] : cases) {
        EXPECT_NEAR(fountainLift(height / kDiagonal) * kThrust, change, 0.25) << height << " m";
    }
    EXPECT_EQ(fountainLift(kFountainReach), 0.0);
    EXPECT_NEAR(fountainLift(kFountainReach - 1e-9), 0.0, 1e-9) << "no step at its reach";
    EXPECT_GT(fountainLift(1.0), fountainLift(1.5)) << "stronger lower";
    EXPECT_EQ(fountainLift(0.2), fountainLift(kFountainLowest)) << "held below the lowest case";
}

// --- Over water (Downwash::setWater; downwash.md "Over water") ----------------------------

/// A pond 1.2 m deep whose surface is at z = 1.2 over the ground at 0, 6 m a side.
Downwash pond() {
    Downwash d;
    d.setGround({0.0, 0.0, 0.0}, {0.0, 0.0, 1.0});
    d.setWater({Downwash::WaterSurface{"pond", {0.0, 0.0, 1.2}, 3.0, 3.0}});
    return d;
}

TEST(Downwash, AWakeStopsOnWaterAsOnTheGround) {
    // The same drone 1 m over the pond and 1 m over bare ground: the same jet arrives and the
    // same outwash spreads; nothing moves under the surface, and no dust lifts from water.
    Downwash water = pond();
    water.update(offboard(2.2));
    Downwash ground;
    ground.setGround({0.0, 0.0, 0.0}, {0.0, 0.0, 1.0});
    ground.update(offboard(1.0));
    const Downwash::Group& g = water.groups()[0];
    ASSERT_TRUE(g.hits);
    EXPECT_EQ(g.hitWater, 0);
    EXPECT_EQ(g.hitBox, -1);
    EXPECT_NEAR(g.hitPoint[2], 1.2, 1e-9);
    EXPECT_NEAR(g.arriveSpeed, ground.groups()[0].arriveSpeed, 1e-12);
    for (const double r : {0.2, 0.5, 1.0, 2.0}) {
        EXPECT_NEAR(water.velocityAt({r, 0.0, 1.21})[0], ground.velocityAt({r, 0.0, 0.01})[0],
                    1e-12)
            << r;
    }
    EXPECT_EQ(water.velocityAt({0.0, 0.0, 1.0})[2], 0.0);  // under the surface
    EXPECT_EQ(water.peakOutwash(), 0.0);
    EXPECT_GT(ground.peakOutwash(), 0.0);
    // Beside the pond the wake goes down to the ground.
    Downwash beside = pond();
    beside.update(offboard(2.2, 0, 5.0));
    EXPECT_EQ(beside.groups()[0].hitWater, -1);
    EXPECT_NEAR(beside.groups()[0].hitPoint[2], 0.0, 1e-9);
}

TEST(Downwash, TheSurfaceCarriesTheJetsMomentumAsPressure) {
    // Not fitted: a jet stopped by a surface pushes it with its whole momentum flux, the thrust
    // (∫p dA = T), and the stagnation pressure on the axis is ½ρU² of the arriving jet.
    for (const double height : {0.5, 1.0, 2.0, 4.0}) {
        Downwash d = pond();
        d.update(offboard(1.2 + height));
        const double thrust = 0.572 * kGravity;
        double force = 0.0;
        constexpr int kCells = 600;
        const double span = 3.0;
        const double cell = 2.0 * span / kCells;
        for (int i = 0; i < kCells; ++i) {
            for (int j = 0; j < kCells; ++j) {
                const Vec3 x{-span + (i + 0.5) * cell, -span + (j + 0.5) * cell, 1.2};
                force += d.surfacePressure(x, kRho, 0) * cell * cell;
            }
        }
        // Within the near wake's and far jet's own momentum budgets (0.93 at 0.5 m, where the
        // jets merge; 1.02 from 1 m down).
        EXPECT_NEAR(force / thrust, 1.0, 0.08) << height;
        EXPECT_EQ(d.surfacePressure({0.0, 0.0, 1.2}, kRho, -1), 0.0);  // not the ground's
    }
    Downwash high = pond();
    high.update(offboard(1.2 + 10.0 * 0.266));
    const double u = high.groups()[0].arriveSpeed;
    EXPECT_NEAR(high.surfacePressure({0.0, 0.0, 1.2}, kRho, 0), 0.5 * kRho * u * u, 1e-9);
}

TEST(Downwash, TheOutwashsFrictionVelocityOnWater) {
    Downwash d = pond();
    d.update(offboard(2.2));
    for (const double r : {0.3, 0.6, 1.2}) {
        const Vec3 u = d.surfaceOutwash({r, 0.0, 1.2}, 0);
        const Vec3 f = d.surfaceFriction({r, 0.0, 1.2}, 0);
        EXPECT_GT(u[0], 0.0) << r;  // outwards
        EXPECT_NEAR(f[0], kWallJetFriction * u[0], 1e-12);
        // The wall jet's peak, the largest across the layer there.
        const Downwash::Group& g = d.groups()[0];
        EXPECT_NEAR(u[0],
                    outwashSpeed(g.arriveSpeed, g.arriveRadius, r,
                                 kOutwashPeakHeight * g.arriveRadius /
                                     (1.0 / std::sqrt(1.0 + 1.0 / std::numbers::sqrt2))),
                    1e-12);
    }
}

}  // namespace
}  // namespace lagrange::aerodynamics

namespace lagrange::aerodynamics {

// The renderer's ground air (lagrange/gpu/GroundAir.hpp, realistic.slang's groundAir) is a port
// of the outwash here: it must give the same horizontal air over level ground.
namespace {

std::vector<gpu::AirRotor> asAir(const std::vector<Downwash::Rotor>& rotors) {
    std::vector<gpu::AirRotor> out;
    for (const Downwash::Rotor& r : rotors) {
        out.push_back({r.hub, r.thrustAxis, r.radius, r.induced, r.thrust, r.group});
    }
    return out;
}

void expectSameOutwash(const std::vector<Downwash::Rotor>& rotors, double groundZ) {
    Downwash d;
    d.setGround({0.0, 0.0, groundZ}, {0.0, 0.0, 1.0});
    d.update(rotors);
    const std::vector<gpu::AirRotor> air = asAir(rotors);
    const std::vector<gpu::AirJet> jets = gpu::groundJets(air, groundZ);
    ASSERT_EQ(jets.size(), d.groups().size());
    for (std::size_t k = 0; k < jets.size(); ++k) {
        EXPECT_NEAR(jets[k].speed, d.groups()[k].arriveSpeed, 1e-9);
        EXPECT_NEAR(jets[k].radius, d.groups()[k].arriveRadius, 1e-9);
    }
    const double peak = gpu::outwashPeakHeight(jets.front().radius);
    for (const double rho : {0.1, 0.3, 0.6, 1.0, 2.0, 4.0}) {
        for (const double z : {peak, 2.0 * peak, 0.15, 0.3}) {
            const Vec3 at{jets.front().hit[0] + rho, jets.front().hit[1], groundZ + z};
            const Vec3 v = d.velocityAt(at);
            const std::array<double, 2> g = gpu::groundOutwash(jets, at);
            EXPECT_NEAR(g[0], v[0], 1e-9 + 1e-9 * std::abs(v[0])) << rho << " " << z;
            EXPECT_NEAR(g[1], v[1], 1e-9) << rho << " " << z;
        }
    }
}

}  // namespace

TEST(GroundAir, MatchesTheOutwashOfASingleRotor) {
    Downwash::Rotor r;
    r.hub = {0.0, 0.0, 1.0};
    r.thrustAxis = {0.0, 0.0, 1.0};
    r.radius = 1.0;
    r.induced = 5.0;
    r.thrust = 1.0;
    expectSameOutwash({r}, 0.0);
}

TEST(GroundAir, MatchesTheOutwashOfAQuadcopterAtSeveralHeights) {
    for (const double height : {0.3, 1.0, 2.5}) {
        expectSameOutwash(offboard(height + 0.4), 0.4);
    }
}

TEST(GroundAir, NoRotorsNoOutwash) {
    const std::vector<gpu::AirJet> jets = gpu::groundJets({}, 0.0);
    EXPECT_TRUE(jets.empty());
    const std::array<double, 2> g = gpu::groundOutwash(jets, {1.0, 2.0, 0.0});
    EXPECT_EQ(g[0], 0.0);
    EXPECT_EQ(g[1], 0.0);
    // Idle rotors: no wake.
    Downwash::Rotor idle;
    idle.hub = {0.0, 0.0, 1.0};
    idle.thrustAxis = {0.0, 0.0, 1.0};
    idle.radius = 0.12;
    EXPECT_TRUE(gpu::groundJets(asAir({idle}), 0.0).empty());
}

TEST(GroundAir, BladesBelowThePeakFeelThePeak) {
    Downwash::Rotor r;
    r.hub = {0.0, 0.0, 1.0};
    r.thrustAxis = {0.0, 0.0, 1.0};
    r.radius = 0.2;
    r.induced = 6.0;
    r.thrust = 5.0;
    const std::vector<gpu::AirJet> jets = gpu::groundJets(asAir({r}), 0.0);
    ASSERT_EQ(jets.size(), 1U);
    const double peak = gpu::outwashPeakHeight(jets[0].radius);
    EXPECT_DOUBLE_EQ(gpu::groundOutwash(jets, {0.5, 0.0, 0.0})[0],
                     gpu::groundOutwash(jets, {0.5, 0.0, peak})[0]);
    EXPECT_GT(gpu::groundOutwash(jets, {0.5, 0.0, 0.0})[0], 0.5);
}

}  // namespace lagrange::aerodynamics
