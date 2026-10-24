// The Tier 0 caster (ShapeCaster: Embree culls, the exact tests decide) must give exactly what
// the brute-force reference gives, every shape against every ray, bit for bit (invariant 7).
#include "lagrange/sensors/Sensors.hpp"
#include "lagrange/sensors/ShapeCaster.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <random>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::sensors {
namespace {

std::array<double, 4> randomRotation(std::mt19937_64& random) {
    std::normal_distribution<double> normal;
    std::array<double, 4> q{normal(random), normal(random), normal(random), normal(random)};
    const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    for (double& v : q) {
        v /= n;
    }
    return q;
}

/// A ground plane, a tilted plane, and `count` shapes of every kind within 20 m of `centre`.
std::vector<ShapeCaster::Shape> scene(std::mt19937_64& random, int count,
                                      const std::array<double, 3>& centre) {
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::uniform_real_distribution<double> spread(-20.0, 20.0);
    std::vector<ShapeCaster::Shape> shapes;
    shapes.push_back(
        {ShapeType::Plane, {0, 0, 0}, {{centre[0], centre[1], centre[2]}, {1, 0, 0, 0}}});
    shapes.push_back(
        {ShapeType::Plane,
         {0, 0, 0},
         {{centre[0] + 40.0, centre[1], centre[2]}, {std::cos(0.7), 0.0, std::sin(0.7), 0.0}}});
    for (int k = 0; k < count; ++k) {
        const auto type = static_cast<ShapeType>(1 + static_cast<int>(unit(random) * 4.0) % 4);
        shapes.push_back(
            {type,
             {0.05 + 1.2 * unit(random), 0.05 + 1.2 * unit(random), 0.05 + 1.2 * unit(random)},
             {{centre[0] + spread(random), centre[1] + spread(random),
               centre[2] + 3.0 * unit(random)},
              randomRotation(random)}});
    }
    return shapes;
}

double reference(const std::vector<ShapeCaster::Shape>& shapes, const std::array<double, 3>& origin,
                 const std::array<double, 3>& dir) {
    double nearest = kNoHit;
    for (const ShapeCaster::Shape& s : shapes) {
        nearest = std::min(nearest, raycast(s.type, s.size, s.pose, origin, dir));
    }
    return nearest;
}

std::array<double, 3> randomDirection(std::mt19937_64& random) {
    std::normal_distribution<double> normal;
    std::array<double, 3> d{normal(random), normal(random), normal(random)};
    const double n = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    for (double& v : d) {
        v /= n;
    }
    return d;
}

/// Rays from `origins` in random directions; counts the ones that differ in any bit.
int mismatches(const ShapeCaster& caster, const std::vector<ShapeCaster::Shape>& shapes,
               const std::vector<std::array<double, 3>>& origins, std::mt19937_64& random,
               int raysPerOrigin, int& hits) {
    int differ = 0;
    for (const auto& origin : origins) {
        for (int r = 0; r < raysPerOrigin; ++r) {
            const std::array<double, 3> dir = randomDirection(random);
            const double expected = reference(shapes, origin, dir);
            const double got = caster.cast(origin, dir);
            hits += expected < kNoHit ? 1 : 0;
            if (std::memcmp(&expected, &got, sizeof(double)) != 0) {
                if (differ < 5) {
                    ADD_FAILURE() << "ray from (" << origin[0] << ", " << origin[1] << ", "
                                  << origin[2] << ") along (" << dir[0] << ", " << dir[1] << ", "
                                  << dir[2] << "): reference " << expected << ", caster " << got;
                }
                ++differ;
            }
        }
    }
    return differ;
}

TEST(ShapeCaster, IsBitIdenticalToEveryShapeAgainstEveryRay) {
    std::mt19937_64 random(20261024);
    const auto shapes = scene(random, 400, {0.0, 0.0, 0.0});
    ShapeCaster caster;
    caster.update(shapes);
    // From inside the clutter (some origins inside shapes: the exit point counts).
    std::vector<std::array<double, 3>> origins;
    std::uniform_real_distribution<double> spread(-15.0, 15.0);
    for (int k = 0; k < 20; ++k) {
        origins.push_back({spread(random), spread(random), 0.5});
    }
    int hits = 0;
    EXPECT_EQ(mismatches(caster, shapes, origins, random, 5000, hits), 0);
    EXPECT_GT(hits, 50'000) << "most of the 100 000 rays hit something";
}

TEST(ShapeCaster, GrazingRaysAreNotCulled) {
    // Rays aimed at the rim of spheres and boxes, where a culling volume in float could cut
    // off a hit that the exact test in double finds.
    std::mt19937_64 random(7);
    std::vector<ShapeCaster::Shape> shapes;
    for (int k = 0; k < 50; ++k) {
        shapes.push_back({k % 2 == 0 ? ShapeType::Sphere : ShapeType::Box,
                          {0.3, 0.3, 0.3},
                          {{5.0 + 2.0 * k, 0.0, 0.0}, {1, 0, 0, 0}}});
    }
    ShapeCaster caster;
    caster.update(shapes);
    int differ = 0;
    int hits = 0;
    for (int k = 0; k < 50; ++k) {
        for (int j = -200; j <= 200; ++j) {
            // From the origin to points spread around the top edge of shape k.
            const double x = 5.0 + 2.0 * k;
            const double z = 0.3 + j * 1e-7;
            const double n = std::hypot(x, z);
            const std::array<double, 3> dir{x / n, 0.0, z / n};
            const double expected = reference(shapes, {0, 0, 0}, dir);
            const double got = caster.cast({0, 0, 0}, dir);
            hits += expected < kNoHit ? 1 : 0;
            differ += std::memcmp(&expected, &got, sizeof(double)) != 0 ? 1 : 0;
        }
    }
    EXPECT_EQ(differ, 0);
    EXPECT_GT(hits, 0);
}

TEST(ShapeCaster, StaysExactWhenShapesMove) {
    std::mt19937_64 random(11);
    auto shapes = scene(random, 200, {0.0, 0.0, 0.0});
    ShapeCaster caster;
    caster.update(shapes);
    std::uniform_real_distribution<double> step(-2.0, 2.0);
    for (int frame = 0; frame < 5; ++frame) {
        // Every third shape moves and turns, as the robot's own bodies do between scans.
        for (std::size_t k = 2; k < shapes.size(); k += 3) {
            for (double& v : shapes[k].pose.position) {
                v += step(random);
            }
            shapes[k].pose.orientation = randomRotation(random);
        }
        caster.update(shapes);
        int hits = 0;
        EXPECT_EQ(
            mismatches(caster, shapes, {{0.2, -0.1, 0.5}, {6.0, 3.0, 1.0}}, random, 2000, hits), 0)
            << "frame " << frame;
    }
}

TEST(ShapeCaster, IsExactFarFromTheRegionOrigin) {
    // The region-local frame rebases at 1 km (huge-worlds.md), so shapes stay within a few km
    // of its origin; here 2.5 km out.
    std::mt19937_64 random(13);
    const std::array<double, 3> far{2500.0, -1800.0, 40.0};
    const auto shapes = scene(random, 300, far);
    ShapeCaster caster;
    caster.update(shapes);
    int hits = 0;
    EXPECT_EQ(mismatches(caster, shapes, {{far[0], far[1], far[2] + 0.5}}, random, 20'000, hits),
              0);
    EXPECT_GT(hits, 10'000);
}

TEST(ShapeCaster, BoundingRadiusContainsEveryShape) {
    EXPECT_EQ(boundingRadius(ShapeType::Sphere, {0.5, 0, 0}), 0.5);
    EXPECT_EQ(boundingRadius(ShapeType::Capsule, {0.5, 1.0, 0}), 1.5);
    EXPECT_DOUBLE_EQ(boundingRadius(ShapeType::Cylinder, {0.3, 0.4, 0}), 0.5);
    EXPECT_DOUBLE_EQ(boundingRadius(ShapeType::Box, {1.0, 2.0, 2.0}), 3.0);
    EXPECT_TRUE(std::isinf(boundingRadius(ShapeType::Plane, {1, 1, 1})));
}

}  // namespace
}  // namespace lagrange::sensors
