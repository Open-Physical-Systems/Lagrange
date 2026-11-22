// Vegetation's stem drag against its closed form (Nepf 1999, eq. 1): quadratic in the speed,
// opposite the motion, zero at rest, outside the canopy and without one.
#include "lagrange/vegetation/Vegetation.hpp"

#include <cmath>
#include <numbers>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::vegetation {
namespace {

TEST(StemDrag, ClosedForm) {
    const double a = frontalAreaDensity(1.5, 4.0);
    EXPECT_DOUBLE_EQ(a, kLeafProjection * 4.0 / 1.5);
    const std::array<double, 3> v{0.6, -0.8, 0.0};  // 1 m/s
    const std::array<double, 3> f = stemDrag(1.2, 1.0, a, 0.05, v);
    const double expected = 0.5 * 1.2 * 1.0 * a * 0.05 * 1.0;
    EXPECT_NEAR(std::hypot(f[0], f[1], f[2]), expected, 1e-15);
    // Opposite the motion.
    EXPECT_NEAR(f[0] * v[0] + f[1] * v[1] + f[2] * v[2], -expected, 1e-15);
    EXPECT_NEAR(f[0] * v[1] - f[1] * v[0], 0.0, 1e-15);
}

TEST(StemDrag, QuadraticInSpeedAndZeroAtRest) {
    const double a = frontalAreaDensity(0.8, 3.0);
    const std::array<double, 3> slow = stemDrag(1.2, 1.0, a, 0.1, {0.5, 0.0, 0.0});
    const std::array<double, 3> fast = stemDrag(1.2, 1.0, a, 0.1, {2.0, 0.0, 0.0});
    EXPECT_NEAR(fast[0] / slow[0], 16.0, 1e-12);
    const std::array<double, 3> rest = stemDrag(1.2, 1.0, a, 0.1, {0.0, 0.0, 0.0});
    EXPECT_EQ(rest[0], 0.0);
    EXPECT_EQ(rest[1], 0.0);
    EXPECT_EQ(rest[2], 0.0);
    EXPECT_EQ(frontalAreaDensity(0.0, 4.0), 0.0);
    EXPECT_EQ(stemDrag(1.2, 1.0, frontalAreaDensity(0.0, 4.0), 0.1, {1.0, 0.0, 0.0})[0], 0.0);
}

// A reed bank: a fixed box with the `reeds` preset. Its canopy is the layer over its top.
TEST(Canopy, LayerOverTheColliderTop) {
    World world;
    SurfaceMaterial reeds;
    ASSERT_TRUE(surfaceMaterialPreset("reeds", reeds));
    ASSERT_GT(reeds.canopyHeight, 0.0);
    world.surfaceMaterials.push_back(reeds);
    world.addCollider(kWorldBody, ShapeType::Box, {2.0, 5.0, 0.5},
                      Pose{{10.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}}, 0.8);
    world.colliders.material.back() = 0;
    // A plain box with no canopy, and a sphere with one, give no canopy.
    world.addCollider(kWorldBody, ShapeType::Box, {1.0, 1.0, 1.0},
                      Pose{{-10.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}}, 0.8);
    world.addCollider(kWorldBody, ShapeType::Sphere, {1.0, 1.0, 1.0},
                      Pose{{-20.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}}, 0.8);
    world.colliders.material.back() = 0;
    const std::vector<Canopy> canopies = canopiesOf(world);
    ASSERT_EQ(canopies.size(), 1U);
    const Canopy& c = canopies.front();
    EXPECT_TRUE(insideCanopy(c, {10.0, 0.0, 0.5 + reeds.canopyHeight / 2.0}));
    EXPECT_FALSE(insideCanopy(c, {10.0, 0.0, 0.5 + reeds.canopyHeight + 0.01}));  // over it
    EXPECT_FALSE(insideCanopy(c, {10.0, 0.0, 0.25}));                             // the soil
    EXPECT_FALSE(insideCanopy(c, {13.0, 0.0, 1.0}));                              // beside it
}

TEST(Canopy, PresetsHaveCanopies) {
    for (const char* name : {"reeds", "shrub", "tall_grass"}) {
        SurfaceMaterial m;
        ASSERT_TRUE(surfaceMaterialPreset(name, m)) << name;
        EXPECT_GT(m.canopyHeight, 0.0) << name;
        EXPECT_GT(m.leafAreaIndex, 0.0) << name;
    }
}

// Cutting (ground.md "Cutting the plants"): a level disc 0.25 m over tall grass (0.4 m) cuts the
// cells under it down to 0.25 m and no others; the area it reports is the disc's, to the cells'
// rounding; the same sweep again cuts nothing; a lower one cuts again.
Canopy tallGrass() {
    Canopy c;
    c.type = ShapeType::Plane;
    c.height = 0.4;
    c.leafAreaIndex = 3.0;
    c.collider = 7;
    return c;
}

TEST(Cutting, ALevelDiscCutsTheCellsUnderIt) {
    const Canopy grass = tallGrass();
    CutMap cuts;
    const double radius = 0.127;  // the sample quadcopter's 10-inch propellers
    const double area = cuts.sweep(grass, {1.0, 2.0, 0.25}, {0.0, 0.0, 1.0}, radius);
    const double disc = std::numbers::pi * radius * radius;
    EXPECT_GT(area, 0.9 * disc);
    EXPECT_LT(area, 1.8 * disc);  // cells reaching past the rim count whole
    EXPECT_NEAR(cuts.top(7, 1.0, 2.0), 0.25, 1e-12);
    EXPECT_NEAR(cuts.top(7, 1.1, 2.0), 0.25, 1e-12);
    EXPECT_TRUE(std::isinf(cuts.top(7, 1.3, 2.0)));  // beyond the rim
    EXPECT_TRUE(std::isinf(cuts.top(8, 1.0, 2.0)));  // another collider
    EXPECT_EQ(cuts.values()[1], static_cast<double>(cuts.cells()));
    const double version = cuts.values()[0];
    EXPECT_GT(version, 0.0);
    // Again at the same height (and less than a piece lower): nothing more.
    EXPECT_EQ(cuts.sweep(grass, {1.0, 2.0, 0.245}, {0.0, 0.0, 1.0}, radius), 0.0);
    EXPECT_EQ(cuts.values()[0], version);
    // Lower: cut again, and the plants never grow back above it.
    EXPECT_GT(cuts.sweep(grass, {1.0, 2.0, 0.1}, {0.0, 0.0, 1.0}, radius), 0.9 * disc);
    EXPECT_NEAR(cuts.top(7, 1.0, 2.0), 0.1, 1e-12);
    EXPECT_EQ(cuts.sweep(grass, {1.0, 2.0, 0.3}, {0.0, 0.0, 1.0}, radius), 0.0);
    EXPECT_NEAR(cuts.top(7, 1.0, 2.0), 0.1, 1e-12);
}

TEST(Cutting, AboveThePlantsNothingAndIntoTheSoilToTheGround) {
    const Canopy grass = tallGrass();
    CutMap cuts;
    EXPECT_EQ(cuts.sweep(grass, {0.0, 0.0, 1.2}, {0.0, 0.0, 1.0}, 0.127), 0.0);
    EXPECT_EQ(cuts.cells(), 0U);
    EXPECT_GT(cuts.sweep(grass, {0.0, 0.0, -0.05}, {0.0, 0.0, 1.0}, 0.127), 0.0);
    EXPECT_EQ(cuts.top(7, 0.0, 0.0), 0.0);
}

TEST(Cutting, ATiltedDiscCutsASlope) {
    const Canopy grass = tallGrass();
    CutMap cuts;
    const double tilt = 20.0 * std::numbers::pi / 180.0;  // about y: the +x side dips
    cuts.sweep(grass, {0.0, 0.0, 0.2}, {std::sin(tilt), 0.0, std::cos(tilt)}, 0.127);
    const double low = cuts.top(7, 0.1, 0.0);
    const double high = cuts.top(7, -0.1, 0.0);
    EXPECT_LT(low, 0.2);
    EXPECT_GT(high, 0.2);
    EXPECT_NEAR(high - low, 0.2 * std::tan(tilt), 0.03);
}

TEST(Cutting, OnlyOverTheBoxAndTheStemsStopAtTheCut) {
    World world;
    SurfaceMaterial reeds;
    ASSERT_TRUE(surfaceMaterialPreset("reeds", reeds));
    world.surfaceMaterials.push_back(reeds);
    world.addCollider(kWorldBody, ShapeType::Box, {1.0, 1.0, 0.5},
                      Pose{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}}, 0.8);
    world.colliders.material.back() = 0;
    const std::vector<Canopy> canopies = canopiesOf(world);
    ASSERT_EQ(canopies.size(), 1U);
    EXPECT_EQ(canopies.front().collider, 0U);
    CutMap cuts;
    // A disc over the box's corner, 1 m over its top: only the quarter over the box is cut.
    const double area = cuts.sweep(canopies.front(), {1.0, 1.0, 1.5}, {0.0, 0.0, 1.0}, 0.2);
    const double quarter = std::numbers::pi * 0.2 * 0.2 / 4.0;
    EXPECT_GT(area, 0.8 * quarter);
    EXPECT_LT(area, 1.6 * quarter);
    EXPECT_TRUE(std::isinf(cuts.top(0, 1.1, 1.1)));
    EXPECT_NEAR(cuts.top(0, 0.9, 0.9), 1.0, 1e-12);
    // Under the cut the reeds still stand; above it, no more.
    EXPECT_TRUE(insideCanopy(canopies.front(), {0.9, 0.9, 1.3}, &cuts));
    EXPECT_FALSE(insideCanopy(canopies.front(), {0.9, 0.9, 1.7}, &cuts));
    EXPECT_TRUE(insideCanopy(canopies.front(), {0.9, 0.9, 1.7}));
    EXPECT_TRUE(insideCanopy(canopies.front(), {0.0, 0.0, 1.7}, &cuts));  // not cut there
    // What a lidar reads from the signal is what the map says.
    CanopyCuts seen;
    seen.load(cuts.values());
    EXPECT_NEAR(seen.top(0, 0.9, 0.9), 1.0, 1e-12);
    EXPECT_TRUE(std::isinf(seen.top(0, 0.0, 0.0)));
    // And a saved map comes back whole.
    CutMap back;
    ASSERT_TRUE(back.assign(cuts.values()));
    EXPECT_NEAR(back.top(0, 0.9, 0.9), 1.0, 1e-12);
    EXPECT_EQ(back.cells(), cuts.cells());
}

// Trees (ground.md "Trees"): the crown's ellipsoid, its sway as a damped oscillator in the air,
// and a disc cutting through it.
TreeSpec oak() {
    TreeSpec t;
    t.name = "oak";
    t.pose.position = {5.0, 0.0, 0.0};
    t.height = 10.0;
    t.crownRadius = 3.0;
    t.crownBase = 3.0;  // crown centre at 6.5 m, half height 3.5 m
    t.leafAreaDensity = 1.0;
    t.dragCoefficient = 0.3;
    t.naturalFrequency = 0.5;
    t.dampingRatio = 0.1;
    t.crownMass = 2.0 * t.crownVolume();
    t.cutEnergy = 2000.0;
    t.leafArea = 0.003;
    return t;
}

TEST(Trees, TheCrownIsAnEllipsoidOverItsBase) {
    const TreeSpec t = oak();
    const std::array<double, 3> none{};
    EXPECT_TRUE(inLeaves(t, 0, {5.0, 0.0, 6.5}, none, nullptr));
    EXPECT_TRUE(inLeaves(t, 0, {7.9, 0.0, 6.5}, none, nullptr));
    EXPECT_FALSE(inLeaves(t, 0, {8.1, 0.0, 6.5}, none, nullptr));
    EXPECT_TRUE(inLeaves(t, 0, {5.0, 0.0, 9.9}, none, nullptr));
    EXPECT_FALSE(inLeaves(t, 0, {5.0, 0.0, 2.9}, none, nullptr));  // under the crown: the trunk
    // Swayed 0.5 m in +x, the crown's edge moves with it.
    EXPECT_TRUE(inLeaves(t, 0, {8.4, 0.0, 6.5}, {0.5, 0.0, 0.0}, nullptr));
    EXPECT_NEAR(t.crownVolume(), 4.0 / 3.0 * std::numbers::pi * 9.0 * 3.5, 1e-9);
}

TEST(Trees, ASteadyWindBendsTheCrownByForceOverStiffness) {
    const TreeSpec t = oak();
    constexpr double kAir = 1.225;
    const std::array<double, 2> wind{8.0, 0.0};
    Sway sway;
    for (int k = 0; k < 120000; ++k) {  // 120 s at 1 ms: many times 1/(ζω) = 3.2 s
        stepSway(t, sway, crownForce(t, kAir, wind, sway.velocity), 1e-3);
    }
    const double omega = 2.0 * std::numbers::pi * t.naturalFrequency;
    const double force = 0.5 * kAir * t.dragCoefficient * t.frontalArea() * 64.0;
    const double expected = force / (t.crownMass * omega * omega);
    std::printf("8 m/s on the oak: %.1f N, the crown %.3f m over (F/k %.3f)\n", force,
                sway.position[0], expected);
    EXPECT_NEAR(sway.position[0], expected, 1e-4 * expected);
    EXPECT_NEAR(sway.position[1], 0.0, 1e-12);
}

TEST(Trees, LetGoTheCrownSwingsAtItsFrequencyAndDampingRatio) {
    const TreeSpec t = oak();
    Sway sway;
    sway.position = {0.3, 0.0};
    const double dt = 1e-4;
    // Upward zero crossings of the velocity mark the peaks: their period and decrement.
    std::vector<double> peaks;
    std::vector<double> times;
    double previous = 0.0;
    for (int k = 0; k < 200000; ++k) {  // 20 s
        stepSway(t, sway, {0.0, 0.0}, dt);
        if (previous > 0.0 && sway.velocity[0] <= 0.0) {
            peaks.push_back(sway.position[0]);
            times.push_back(k * dt);
        }
        previous = sway.velocity[0];
    }
    ASSERT_GE(peaks.size(), 4U);
    const double period = (times[3] - times[0]) / 3.0;
    const double zeta = t.dampingRatio;
    const double expected = 1.0 / (t.naturalFrequency * std::sqrt(1.0 - zeta * zeta));
    EXPECT_NEAR(period, expected, 0.002 * expected);
    // The logarithmic decrement δ = ln(x_n / x_(n+1)) = 2πζ/√(1 − ζ²).
    const double delta = std::log(peaks[0] / peaks[1]);
    EXPECT_NEAR(delta, 2.0 * std::numbers::pi * zeta / std::sqrt(1.0 - zeta * zeta), 0.01 * delta);
}

TEST(Trees, ADiscCutsTheLeavesItPassesThrough) {
    const TreeSpec t = oak();
    TreeCutMap cuts;
    const std::array<double, 3> none{};
    // A 10-inch propeller, level, at the crown's centre: a disc of cells one cell thick.
    const double radius = 0.127;
    const double volume = cuts.sweep(t, 0, {5.0, 0.0, 6.55}, {0.0, 0.0, 1.0}, radius, none);
    // Cells of 10 cm a side whose centres... the disc meets the cells it passes over: from the
    // disc's area (0.0507 m²) to that of the cells round it, one cell thick.
    const double cell = kTreeCutCell;
    EXPECT_GT(volume, std::numbers::pi * radius * radius * cell);
    EXPECT_LT(volume, std::numbers::pi * (radius + 1.5 * cell) * (radius + 1.5 * cell) * cell);
    EXPECT_FALSE(inLeaves(t, 0, {5.0, 0.0, 6.55}, none, &cuts.cuts()));
    EXPECT_TRUE(inLeaves(t, 0, {5.0, 0.0, 6.85}, none, &cuts.cuts()));  // above the cut
    EXPECT_TRUE(inLeaves(t, 0, {5.5, 0.0, 6.55}, none, &cuts.cuts()));  // beyond the disc
    // The same sweep again cuts nothing more; outside the crown nothing.
    EXPECT_EQ(cuts.sweep(t, 0, {5.0, 0.0, 6.55}, {0.0, 0.0, 1.0}, radius, none), 0.0);
    EXPECT_EQ(cuts.sweep(t, 0, {5.0, 0.0, 11.0}, {0.0, 0.0, 1.0}, radius, none), 0.0);
    // The map round-trips through its signal.
    TreeCutMap back;
    ASSERT_TRUE(back.assign(cuts.values()));
    EXPECT_FALSE(inLeaves(t, 0, {5.0, 0.0, 6.55}, none, &back.cuts()));
}

TEST(Trees, TreeApiReadsWithItsFallbacks) {
    World w;
    w.schemaInstances.push_back({});
    SchemaInstance& s = w.schemaInstances.back();
    s.schema = "TreeAPI_1";
    s.prim = w.addPrim("/oak", "oak");
    w.prims.poseInBody.at(s.prim.index).position = {1.0, 2.0, 0.0};
    s.reals = {{"tree:height", 8.0}, {"tree:crownRadius", 2.0}};
    std::string error;
    const std::vector<TreeSpec> trees = treesOf(w, error);
    ASSERT_EQ(error, "");
    ASSERT_EQ(trees.size(), 1U);
    EXPECT_EQ(trees[0].name, "oak");
    EXPECT_DOUBLE_EQ(trees[0].crownBase, kTreeCrownBaseShare * 8.0);
    EXPECT_DOUBLE_EQ(trees[0].crownMass, kTreeCrownDensity * trees[0].crownVolume());
    EXPECT_DOUBLE_EQ(trees[0].pose.position[1], 2.0);
    s.reals["tree:crownBase"] = 9.0;  // above the top
    EXPECT_TRUE(treesOf(w, error).empty());
    EXPECT_NE(error.find("crownBase"), std::string::npos);
}

}  // namespace
}  // namespace lagrange::vegetation
