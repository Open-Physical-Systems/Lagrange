// Flying leaves (docs/physics/ground.md "Flying leaves") against their closed forms: the terminal
// fall speed of the quadratic drag, the drift at the wind's speed, the vortex cylinder's wake, the
// spawn count of a cut (cut volume × LAD / leaf area, the fraction carried), the stripping rate;
// they land on the highest surface under them and lie flat; the same cuts twice are bit for bit,
// and a saved state carries on as the run would.
#include "lagrange/vegetation/Leaves.hpp"
#include "lagrange/vegetation/Vegetation.hpp"

#include <cmath>
#include <cstdio>
#include <numbers>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::vegetation {
namespace {

using Vec3 = std::array<double, 3>;
constexpr Pose kLevel{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}};
constexpr std::uint64_t kSeed = 0x1234ABCDULL;

LandingSurfaces groundAtZero() {
    World world;
    world.addCollider(kWorldBody, ShapeType::Plane, {0.0, 0.0, 0.0}, kLevel, 0.8);
    LandingSurfaces ground;
    ground.build(world);
    return ground;
}

TreeSpec oak() {
    TreeSpec t;
    t.name = "oak";
    t.pose.position = {5.0, 0.0, 0.0};
    t.height = 10.0;
    t.crownRadius = 3.0;
    t.crownBase = 3.0;  // crown centre at 6.5 m
    t.leafAreaDensity = 1.0;
    t.crownMass = 2.0 * t.crownVolume();
    t.naturalFrequency = 0.5;
    t.leafArea = 0.003;
    return t;
}

TEST(Leaves, InStillAirALeafFallsAtItsTerminalSpeedAndLiesFlat) {
    LeafParams params;
    params.glide = 0.0;  // straight down: the closed form
    LeafField field(params);
    field.reset(1);
    field.add(0, {0.0, 0.0, 20.0}, {0.0, 0.0, 0.0}, 0.003, 0.0, kSeed, 0);
    const Leaf& leaf = field.leaves().front();
    const double vt = leaf.terminalSpeed(params.gravity);
    // Its own terminal speed, drawn within ±20 % of 1.5 m/s.
    EXPECT_GE(vt, 0.8 * params.terminalSpeed - 1e-12);
    EXPECT_LE(vt, 1.2 * params.terminalSpeed + 1e-12);
    const LandingSurfaces ground = groundAtZero();
    const double dt = 1e-3;
    for (int k = 0; k < 3000; ++k) {  // 3 s: twenty times v_t/g
        field.step(dt, {}, {}, ground);
    }
    const Leaf& falling = field.leaves().front();
    std::printf("terminal speed %.6f m/s (closed form %.6f), at z %.3f m\n", -falling.velocity[2],
                vt, falling.position[2]);
    EXPECT_NEAR(-falling.velocity[2], vt, 1e-9 * vt);
    EXPECT_EQ(falling.velocity[0], 0.0);
    EXPECT_EQ(falling.settled, 0.0);
    for (int k = 0; k < 20000 && field.leaves().front().settled == 0.0; ++k) {
        field.step(dt, {}, {}, ground);
    }
    const Leaf& down = field.leaves().front();
    EXPECT_EQ(down.settled, 1.0);
    EXPECT_EQ(down.position[2], 0.0);
    EXPECT_EQ(down.velocity, (Vec3{}));
    EXPECT_DOUBLE_EQ(down.orientation(params.tiltAmplitude)[1], std::numbers::pi / 2.0);  // flat
    EXPECT_EQ(field.flying(0), 0U);
    // Settled, nothing moves any more.
    EXPECT_FALSE(field.step(dt, {}, {}, ground));
}

TEST(Leaves, TumblingItGlidesFromSideToSideAndFallsNoFaster) {
    LeafField field;  // glide on
    field.reset(1);
    field.add(0, {0.0, 0.0, 30.0}, {}, 0.003, 0.0, kSeed, 0);
    const double vt = field.leaves().front().terminalSpeed(field.params().gravity);
    const LandingSurfaces ground = groundAtZero();
    double fastestSide = 0.0;
    double tiltMin = 10.0;
    double tiltMax = -10.0;
    for (int k = 0; k < 5000; ++k) {
        field.step(1e-3, {}, {}, ground);
        const Leaf& l = field.leaves().front();
        fastestSide = std::max(fastestSide, std::hypot(l.velocity[0], l.velocity[1]));
        const double pitch = l.orientation(field.params().tiltAmplitude)[1];
        tiltMin = std::min(tiltMin, pitch);
        tiltMax = std::max(tiltMax, pitch);
    }
    const Leaf& l = field.leaves().front();
    std::printf(
        "tumbling: falls at %.3f m/s (v_t %.3f), sideways up to %.3f m/s, pitch %.2f-%.2f\n",
        -l.velocity[2], vt, fastestSide, tiltMin, tiltMax);
    EXPECT_GT(fastestSide, 0.05);
    EXPECT_LE(-l.velocity[2], vt * (1.0 + 1e-9));
    EXPECT_GT(-l.velocity[2], 0.8 * vt);
    EXPECT_LT(tiltMin, std::numbers::pi / 2.0 - 0.3);
}

TEST(Leaves, InASteadyWindItDriftsAtTheWindsSpeed) {
    const Vec3 wind{5.0, -2.0, 0.0};
    LeafParams straight;
    straight.glide = 0.0;
    LeafField field(straight);
    field.reset(1);
    field.add(0, {0.0, 0.0, 100.0}, {}, 0.003, 0.0, kSeed, 0);
    const LandingSurfaces ground = groundAtZero();
    for (int k = 0; k < 5000; ++k) {
        field.step(1e-3, wind, {}, ground);
    }
    const Leaf& l = field.leaves().front();
    EXPECT_NEAR(l.velocity[0], wind[0], 1e-9);
    EXPECT_NEAR(l.velocity[1], wind[1], 1e-9);
    EXPECT_NEAR(-l.velocity[2], l.terminalSpeed(straight.gravity), 1e-9);
    // Tumbling, it drifts at the wind's speed on average.
    LeafField tumbling;
    tumbling.reset(1);
    tumbling.add(0, {0.0, 0.0, 100.0}, {}, 0.003, 0.0, kSeed, 0);
    for (int k = 0; k < 2000; ++k) {
        tumbling.step(1e-3, wind, {}, ground);
    }
    const Vec3 start = tumbling.leaves().front().position;
    constexpr int kSteps = 20000;  // 20 s: thirty flutters
    for (int k = 0; k < kSteps; ++k) {
        tumbling.step(1e-3, wind, {}, ground);
    }
    const Vec3 end = tumbling.leaves().front().position;
    const double seconds = kSteps * 1e-3;
    std::printf("tumbling in a 5.39 m/s wind: drifts at (%.3f, %.3f) m/s\n",
                (end[0] - start[0]) / seconds, (end[1] - start[1]) / seconds);
    EXPECT_NEAR((end[0] - start[0]) / seconds, wind[0], 0.05);
    EXPECT_NEAR((end[1] - start[1]) / seconds, wind[1], 0.05);
}

TEST(Leaves, TheRotorsWakeIsTheVortexCylinderThenARoundJet) {
    RotorWake r;
    r.hub = {1.0, 2.0, 3.0};
    r.thrustAxis = {0.0, 0.0, 1.0};
    r.radius = 0.1;
    r.induced = 6.0;
    // At the disc v_i, down; 2 v_i far below in the near wake, s/√(s² + R²) of the way.
    EXPECT_NEAR(rotorWake(r, {1.0, 2.0, 3.0})[2], -6.0, 1e-12);
    const double s = 0.3;
    const double w = 6.0 * (1.0 + s / std::hypot(s, 0.1));
    EXPECT_NEAR(rotorWake(r, {1.0, 2.0, 3.0 - s})[2], -w, 1e-12);
    // Its radius by continuity: inside R √(v_i/w), not beyond.
    const double width = 0.1 * std::sqrt(6.0 / w);
    EXPECT_NEAR(rotorWake(r, {1.0 + 0.99 * width, 2.0, 3.0 - s})[2], -w, 1e-12);
    EXPECT_EQ(rotorWake(r, {1.0 + 1.01 * width, 2.0, 3.0 - s})[2], 0.0);
    // Above: the inflow, v_i (1 − h/√(h² + R²)), and nothing beyond two radii.
    EXPECT_NEAR(rotorWake(r, {1.0, 2.0, 3.1})[2], -6.0 * (1.0 - 0.1 / std::hypot(0.1, 0.1)), 1e-12);
    EXPECT_EQ(rotorWake(r, {1.0, 2.0, 3.25})[2], 0.0);
    // Past four radii a round jet keeps its momentum flux: w × width constant.
    const double atEnd = -rotorWake(r, {1.0, 2.0, 3.0 - 0.4})[2];
    EXPECT_NEAR(-rotorWake(r, {1.0, 2.0, 3.0 - 0.8})[2], atEnd / 2.0, 1e-12);
    // No induced velocity, no wake.
    r.induced = 0.0;
    EXPECT_EQ(rotorWake(r, {1.0, 2.0, 2.9})[2], 0.0);
}

TEST(Leaves, ACutStartsVolumeTimesLadOverLeafAreaLeaves) {
    const TreeSpec t = oak();
    TreeCutMap cuts;
    LeafField field;
    field.reset(1);
    double volume = 0.0;
    std::uint32_t started = 0;
    const Vec3 none{};
    const Vec3 spin{0.0, 0.0, 800.0};  // rad/s about +z
    // A 10-inch disc stepping through the crown, level, 13 positions.
    for (int k = 0; k < 13; ++k) {
        std::vector<Vec3> centres;
        const Vec3 hub{3.0 + (0.35 * k), 0.2, 6.55 + (0.07 * k)};
        volume += cuts.sweep(t, 0, hub, {0.0, 0.0, 1.0}, 0.127, none, &centres);
        std::vector<CutCell> cells;
        for (const Vec3& c : centres) {
            const Vec3 r{c[0] - hub[0], c[1] - hub[1], 0.0};
            cells.push_back({c, {-spin[2] * r[1], spin[2] * r[0], 0.0}});
        }
        const std::size_t before = field.leaves().size();
        started += field.spawnFromCut(0, t, cells, {}, {0.0, 1.0, 0.0}, kSeed, k);
        // Each new leaf leaves within half a cell of a cut cell's centre, at 0.3 of the blade's
        // velocity there plus the air.
        for (std::size_t n = before; n < field.leaves().size(); ++n) {
            const Leaf& l = field.leaves()[n];
            bool near = false;
            for (const CutCell& c : cells) {
                bool inside = true;
                for (std::size_t a = 0; a < 3; ++a) {
                    inside = inside && std::abs(l.position.at(a) - c.centre.at(a)) <=
                                           0.5 * kTreeCutCell + 1e-12;
                }
                const bool moving =
                    std::abs(l.velocity[0] - 0.3 * c.bladeVelocity[0]) < 1e-9 &&
                    std::abs(l.velocity[1] - (0.3 * c.bladeVelocity[1] + 1.0)) < 1e-9;
                near = near || (inside && moving);
            }
            EXPECT_TRUE(near) << "leaf " << n;
        }
    }
    const double expected = std::floor((volume * t.leafAreaDensity / t.leafArea) + 1e-9);
    std::printf("%.4f m³ cut: %u leaves (volume × LAD / leaf area %.2f)\n", volume, started,
                volume * t.leafAreaDensity / t.leafArea);
    EXPECT_GT(volume, 0.0);
    EXPECT_EQ(static_cast<double>(started), expected);
    EXPECT_EQ(field.leaves().size(), started);
    EXPECT_EQ(field.flying(0), started);
}

TEST(Leaves, StrongAirStripsTheCrownAtItsRate) {
    const TreeSpec t = oak();
    LeafField field;
    field.reset(1);
    const Vec3 air{15.0, 0.0, 0.0};
    const double dt = 1e-3;
    std::uint32_t started = 0;
    for (int k = 0; k < 1000; ++k) {  // 1 s
        started += field.strip(0, t, 0.0, air, {}, {}, air, dt, kSeed, k);
    }
    const double leaves = t.crownVolume() * t.leafAreaDensity / t.leafArea;
    const double rate = 1e-3 * ((15.0 * 15.0 / 100.0) - 1.0) * leaves;  // per s
    std::printf("15 m/s on the oak: %u leaves in 1 s (rate %.1f per s)\n", started, rate);
    EXPECT_NEAR(started, rate, 1.0);
    // Each from the crown's surface.
    for (const Leaf& l : field.leaves()) {
        const Vec3 q{(l.position[0] - 5.0) / 3.0, l.position[1] / 3.0, (l.position[2] - 6.5) / 3.5};
        EXPECT_NEAR(std::hypot(q[0], q[1], q[2]), 1.0, 1e-9);
    }
    // Under the threshold, nothing.
    EXPECT_EQ(field.strip(0, t, 0.0, {9.9, 0.0, 0.0}, {}, {}, {}, dt, kSeed, 2000), 0U);
}

TEST(Leaves, TheyLandOnTheHighestSurfaceUnderThem) {
    World world;
    world.addCollider(kWorldBody, ShapeType::Plane, {}, kLevel, 0.8);
    world.addCollider(kWorldBody, ShapeType::Box, {1.0, 1.0, 1.0},
                      Pose{{10.0, 0.0, 1.0}, {1.0, 0.0, 0.0, 0.0}}, 0.8);  // a roof at 2 m
    LandingSurfaces ground;
    ground.build(world);
    EXPECT_EQ(ground.below(10.5, 0.5, 5.0), 2.0);
    EXPECT_EQ(ground.below(10.5, 0.5, 1.5), 0.0);  // under the roof: the ground
    EXPECT_EQ(ground.below(12.0, 0.0, 5.0), 0.0);
    LeafParams straight;
    straight.glide = 0.0;
    LeafField field(straight);
    field.reset(1);
    field.add(0, {10.5, 0.5, 6.0}, {}, 0.003, 0.0, kSeed, 0);
    field.add(0, {12.5, 0.5, 6.0}, {}, 0.003, 0.0, kSeed, 0);
    for (int k = 0; k < 10000; ++k) {
        field.step(1e-3, {}, {}, ground);
    }
    EXPECT_EQ(field.leaves()[0].position[2], 2.0);
    EXPECT_EQ(field.leaves()[1].position[2], 0.0);
}

/// A run with cuts: a disc sweeps through the crown in the wake of its own rotor, in a breeze.
std::vector<double> runWithCuts(int steps, LeafField& field, TreeCutMap& cuts, int from = 0) {
    const TreeSpec t = oak();
    const LandingSurfaces ground = groundAtZero();
    const Vec3 wind{2.0, 1.0, 0.0};
    for (int k = from; k < from + steps; ++k) {
        const Vec3 hub{3.0 + (0.004 * k), 0.0, 6.5};
        const std::array<RotorWake, 1> wake{RotorWake{hub, {0.0, 0.0, 1.0}, 0.127, 7.0}};
        std::vector<Vec3> centres;
        cuts.sweep(t, 0, hub, {0.0, 0.0, 1.0}, 0.127, {}, &centres);
        std::vector<CutCell> cells;
        for (const Vec3& c : centres) {
            cells.push_back({c, {-800.0 * (c[1] - hub[1]), 800.0 * (c[0] - hub[0]), 0.0}});
        }
        field.spawnFromCut(0, t, cells, wake, wind, kSeed, k);
        field.strip(0, t, 0.0, {12.0, 0.0, 0.0}, {}, wake, wind, 1e-3, kSeed, k);
        field.step(1e-3, wind, wake, ground);
    }
    return field.state();
}

TEST(Leaves, TheSameCutsTwiceAreBitForBitAndASavedStateCarriesOn) {
    LeafField a;
    LeafField b;
    a.reset(1);
    b.reset(1);
    TreeCutMap cutsA;
    TreeCutMap cutsB;
    const std::vector<double> first = runWithCuts(1500, a, cutsA);
    const std::vector<double> second = runWithCuts(1500, b, cutsB);
    ASSERT_GT(a.leaves().size(), 20U);
    EXPECT_EQ(first, second);
    std::uint32_t down = 0;
    for (const Leaf& l : a.leaves()) {
        down += l.settled > 0.0 ? 1U : 0U;
    }
    std::printf("after 1.5 s: %zu leaves, %u on the ground\n", a.leaves().size(), down);
    // Saved half way, restored into a new field, it carries on as the first did.
    LeafField c;
    ASSERT_TRUE(c.assign(first));
    TreeCutMap cutsC;
    ASSERT_TRUE(cutsC.assign(cutsA.values()));
    const std::vector<double> carriedOn = runWithCuts(1500, c, cutsC, 1500);
    const std::vector<double> straight = runWithCuts(1500, a, cutsA, 1500);
    EXPECT_EQ(carriedOn, straight);
    // The signal's layout.
    std::vector<double> out;
    a.write(out, 7.0);
    ASSERT_EQ(out.size(), kLeavesLength);
    EXPECT_EQ(out[0], 7.0);
    EXPECT_EQ(out[1], static_cast<double>(a.leaves().size()));
    EXPECT_EQ(out[kLeavesHeader + 2], a.leaves()[0].position[2]);
    EXPECT_FALSE(c.assign(std::vector<double>{0.0, 1.0, 0.0}));  // one tree, its carries short
}

TEST(Leaves, PastTheCapacityTheOldestGo) {
    LeafField field;
    field.reset(1);
    for (std::uint32_t k = 0; k < kLeavesCapacity + 10; ++k) {
        field.add(0, {0.0, 0.0, 1.0}, {}, 0.003, 0.0, kSeed, k);
    }
    ASSERT_EQ(field.leaves().size(), kLeavesCapacity);
    EXPECT_EQ(field.leaves().front().id, 10.0);
}

}  // namespace
}  // namespace lagrange::vegetation
