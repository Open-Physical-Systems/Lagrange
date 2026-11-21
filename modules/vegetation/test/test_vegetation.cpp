// Vegetation's stem drag against its closed form (Nepf 1999, eq. 1): quadratic in the speed,
// opposite the motion, zero at rest, outside the canopy and without one.
#include "lagrange/vegetation/Vegetation.hpp"

#include <cmath>
#include <numbers>

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

}  // namespace
}  // namespace lagrange::vegetation
