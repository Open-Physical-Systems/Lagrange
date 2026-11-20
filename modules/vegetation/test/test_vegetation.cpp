// Vegetation's stem drag against its closed form (Nepf 1999, eq. 1): quadratic in the speed,
// opposite the motion, zero at rest, outside the canopy and without one.
#include "lagrange/vegetation/Vegetation.hpp"

#include <cmath>

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

}  // namespace
}  // namespace lagrange::vegetation
