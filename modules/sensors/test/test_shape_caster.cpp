// The Tier 0 caster (ShapeCaster: Embree culls, the exact tests decide) must give exactly what
// the brute-force reference gives, every shape against every ray, bit for bit (invariant 7).
#include "lagrange/random/Philox.hpp"
#include "lagrange/sensors/Sensors.hpp"
#include "lagrange/sensors/ShapeCaster.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <numbers>
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

TEST(ShapeCaster, StaysExactWhenShapesStartAndStopMoving) {
    // Shapes that have never moved sit in a hierarchy built once; a shape's first move takes it
    // to the one rebuilt. Shapes start moving at different scans (a parked car pulling out),
    // some stop again, and a frame moves nothing.
    std::mt19937_64 random(12);
    auto shapes = scene(random, 300, {0.0, 0.0, 0.0});
    ShapeCaster caster;
    caster.update(shapes);
    std::uniform_real_distribution<double> step(-1.5, 1.5);
    for (int frame = 0; frame < 8; ++frame) {
        for (std::size_t k = 0; k < shapes.size(); ++k) {
            const auto start = static_cast<int>(k % 7);  // first frame it moves
            const bool moves = frame >= start && frame != 5 && (k % 5 != 0 || frame < 3);
            if (!moves) {
                continue;
            }
            for (double& v : shapes[k].pose.position) {
                v += step(random);
            }
            shapes[k].pose.orientation = randomRotation(random);
        }
        caster.update(shapes);
        int hits = 0;
        EXPECT_EQ(mismatches(caster, shapes, {{0.2, -0.1, 0.5}, {6.0, 3.0, 1.0}, {-4.0, 8.0, 0.3}},
                             random, 1500, hits),
                  0)
            << "frame " << frame;
        EXPECT_GT(hits, 0);
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

// Roadmap 0.2-C4: the SceneQuery side of the caster.
TEST(SceneQuery, RayHitsNameTheShapeBruteForceFinds) {
    std::mt19937_64 random(17);
    const auto shapes = scene(random, 300, {0.0, 0.0, 0.0});
    ShapeCaster caster;
    caster.update(shapes);
    int hits = 0;
    for (int r = 0; r < 20'000; ++r) {
        const std::array<double, 3> origin{0.3, -0.2, 0.5};
        const std::array<double, 3> dir = randomDirection(random);
        double nearest = kNoHit;
        std::uint32_t shape = services::RayHit::kNone;
        for (std::uint32_t k = 0; k < shapes.size(); ++k) {
            const double t = raycast(shapes[k].type, shapes[k].size, shapes[k].pose, origin, dir);
            if (t < nearest) {  // ascending k: the lowest index wins a tie
                nearest = t;
                shape = k;
            }
        }
        const services::RayHit hit = caster.castHit(origin, dir);
        ASSERT_EQ(hit.shape, shape) << "ray " << r;
        ASSERT_EQ(hit.distance, nearest) << "ray " << r;
        hits += hit.hit() ? 1 : 0;
        // Through the interface, with a range limit.
        const services::SceneQuery& query = caster;
        const services::RayHit limited = query.ray(origin, dir, 5.0);
        EXPECT_EQ(limited.hit(), hit.hit() && hit.distance <= 5.0);
    }
    EXPECT_GT(hits, 10'000);
}

TEST(SceneQuery, RayGivesTheSurfaceNormalAtTheHit) {
    // A plane sloped 10° about y and a box: straight down onto each, the outward normal.
    const double slope = 10.0 * std::numbers::pi / 180.0;
    std::vector<ShapeCaster::Shape> shapes(2);
    shapes[0].type = ShapeType::Plane;
    shapes[0].size = {50.0, 50.0, 1.0};
    shapes[0].pose.orientation = {std::cos(slope / 2.0), 0.0, std::sin(slope / 2.0), 0.0};
    shapes[1].type = ShapeType::Box;
    shapes[1].size = {1.0, 1.0, 1.0};
    shapes[1].pose.position = {10.0, 0.0, 3.0};
    ShapeCaster caster;
    caster.update(shapes);
    const services::SceneQuery& query = caster;
    const services::RayHit plane = query.ray({0.0, 0.0, 5.0}, {0.0, 0.0, -1.0}, 10.0);
    ASSERT_EQ(plane.shape, 0U);
    EXPECT_NEAR(plane.normal[0], std::sin(slope), 1e-12);
    EXPECT_NEAR(plane.normal[1], 0.0, 1e-12);
    EXPECT_NEAR(plane.normal[2], std::cos(slope), 1e-12);
    const services::RayHit box = query.ray({10.2, 0.3, 8.0}, {0.0, 0.0, -1.0}, 10.0);
    ASSERT_EQ(box.shape, 1U);
    EXPECT_NEAR(box.distance, 4.0, 1e-12);
    EXPECT_NEAR(box.normal[2], 1.0, 1e-12);
    // Without a hit there is no normal.
    const services::RayHit none = query.ray({0.0, 0.0, 5.0}, {0.0, 0.0, 1.0}, 10.0);
    EXPECT_FALSE(none.hit());
    EXPECT_EQ(none.normal[2], 0.0);
}

TEST(SceneQuery, DistanceToEachShapeKind) {
    const Pose at{{1.0, 2.0, 3.0}, {1, 0, 0, 0}};
    // Turned 90 degrees about z: local x is world y.
    const Pose turned{
        {0.0, 0.0, 0.0},
        {std::cos(0.25 * std::numbers::pi), 0.0, 0.0, std::sin(0.25 * std::numbers::pi)}};
    EXPECT_NEAR(distanceToShape(ShapeType::Sphere, {0.5, 0, 0}, at, {1.0, 2.0, 5.0}), 1.5, 1e-12);
    EXPECT_EQ(distanceToShape(ShapeType::Sphere, {0.5, 0, 0}, at, {1.2, 2.0, 3.0}), 0.0);
    // Box with half extents 2 (x) and 0.5 (y), turned: it is long along world y.
    EXPECT_NEAR(distanceToShape(ShapeType::Box, {2.0, 0.5, 0.5}, turned, {0.0, 3.0, 0.0}), 1.0,
                1e-12);
    EXPECT_NEAR(distanceToShape(ShapeType::Box, {2.0, 0.5, 0.5}, turned, {3.0, 0.0, 0.0}), 2.5,
                1e-12);
    // Corner: from (1.5, 2.5, 0) to the corner (0.5, 2, 0) of the turned box.
    EXPECT_NEAR(distanceToShape(ShapeType::Box, {2.0, 0.5, 0.5}, turned, {1.5, 2.5, 0.0}),
                std::hypot(1.0, 0.5), 1e-12);
    // Capsule radius 0.2, half length 1 along z: beyond the cap, and beside the side.
    EXPECT_NEAR(distanceToShape(ShapeType::Capsule, {0.2, 1.0, 0}, Pose{}, {0, 0, 2.0}), 0.8,
                1e-12);
    EXPECT_NEAR(distanceToShape(ShapeType::Capsule, {0.2, 1.0, 0}, Pose{}, {1.0, 0, 0.5}), 0.8,
                1e-12);
    // Cylinder radius 0.2, half length 1: off the rim diagonally.
    EXPECT_NEAR(distanceToShape(ShapeType::Cylinder, {0.2, 1.0, 0}, Pose{}, {0.5, 0, 1.4}),
                std::hypot(0.3, 0.4), 1e-12);
    // The ground: height above it, zero below.
    EXPECT_NEAR(distanceToShape(ShapeType::Plane, {0, 0, 0}, Pose{}, {7, -3, 0.25}), 0.25, 1e-12);
    EXPECT_EQ(distanceToShape(ShapeType::Plane, {0, 0, 0}, Pose{}, {7, -3, -1.0}), 0.0);
}

TEST(SceneQuery, OverlapsMatchEveryShapeTested) {
    std::mt19937_64 random(19);
    const auto shapes = scene(random, 400, {0.0, 0.0, 0.0});
    ShapeCaster caster;
    caster.update(shapes);
    std::uniform_real_distribution<double> spread(-22.0, 22.0);
    std::uniform_real_distribution<double> reach(0.0, 3.0);
    std::size_t found = 0;
    for (int q = 0; q < 2000; ++q) {
        const std::array<double, 3> center{spread(random), spread(random), 0.2 + reach(random)};
        const double radius = reach(random);
        std::vector<std::uint32_t> expected;
        for (std::uint32_t k = 0; k < shapes.size(); ++k) {
            if (distanceToShape(shapes[k].type, shapes[k].size, shapes[k].pose, center) <= radius) {
                expected.push_back(k);
            }
        }
        ASSERT_EQ(caster.overlapSphere(center, radius), expected) << "query " << q;
        found += expected.size();
    }
    EXPECT_GT(found, 2000U) << "the queries reach shapes, not only the ground";
}

TEST(ShapeCaster, BoundingRadiusContainsEveryShape) {
    EXPECT_EQ(boundingRadius(ShapeType::Sphere, {0.5, 0, 0}), 0.5);
    EXPECT_EQ(boundingRadius(ShapeType::Capsule, {0.5, 1.0, 0}), 1.5);
    EXPECT_DOUBLE_EQ(boundingRadius(ShapeType::Cylinder, {0.3, 0.4, 0}), 0.5);
    EXPECT_DOUBLE_EQ(boundingRadius(ShapeType::Box, {1.0, 2.0, 2.0}), 3.0);
    EXPECT_TRUE(std::isinf(boundingRadius(ShapeType::Plane, {1, 1, 1})));
}

// Lidar over grass: the mean shortening over many beams is the closed form of the gap model,
// grazing beams end nearer the blade tops than steep ones, no canopy changes nothing, and the
// draws are the same on every run (counter-based streams).
TEST(CanopyReturn, MeanShorteningMatchesTheGapModel) {
    constexpr int kBeams = 200000;
    for (const double h : {0.05, 0.4}) {
        for (const double c : {1.0, 0.5, 0.1}) {
            double sum = 0.0;
            double sumSq = 0.0;
            for (int i = 0; i < kBeams; ++i) {
                random::Stream rng(7, 3, static_cast<std::uint32_t>(i));
                const double d = canopyShortening(h, c, 20.0, rng.uniform());
                EXPECT_GE(d, 0.0);
                EXPECT_LE(d, h / c + 1e-12);
                sum += d;
                sumSq += d * d;
            }
            const double mean = sum / kBeams;
            const double sd = std::sqrt(sumSq / kBeams - mean * mean);
            const double mu = kCanopyProjection * kCanopyLeafAreaIndex / h;
            const double path = h / c;
            const double expected = path - (1.0 - std::exp(-mu * path)) / mu;
            EXPECT_NEAR(canopyMeanShortening(h, c, 20.0), expected, 1e-12);
            EXPECT_NEAR(mean, expected, 5.0 * sd / std::sqrt(kBeams)) << h << " " << c;
        }
    }
    // A grazing beam's return is nearer the blade tops: the share of the path it goes into the
    // canopy, (path - shortening) / path, is smaller than a steep beam's.
    const double h = 0.05;
    const double grazing = 1.0 - canopyMeanShortening(h, 0.1, 20.0) / (h / 0.1);
    const double steep = 1.0 - canopyMeanShortening(h, 1.0, 20.0) / h;
    EXPECT_LT(grazing, steep);
    // The path is bounded by the range: a lidar inside the canopy.
    EXPECT_LE(canopyMeanShortening(0.4, 0.01, 1.5), 1.5);
}

TEST(CanopyReturn, NoCanopyNoChangeAndDeterministic) {
    for (int i = 0; i < 1000; ++i) {
        random::Stream rng(1, 2, static_cast<std::uint32_t>(i));
        EXPECT_EQ(canopyShortening(0.0, 0.3, 12.0, rng.uniform()), 0.0);
    }
    EXPECT_EQ(canopyMeanShortening(0.0, 0.3, 12.0), 0.0);
    EXPECT_EQ(canopyShortening(0.05, 0.3, std::numeric_limits<double>::infinity(), 0.5), 0.0);
    for (int i = 0; i < 1000; ++i) {
        random::Stream a(11, 5, static_cast<std::uint32_t>(i));
        random::Stream b(11, 5, static_cast<std::uint32_t>(i));
        const double x = canopyShortening(0.05, 0.2, 9.0, a.uniform());
        const double y = canopyShortening(0.05, 0.2, 9.0, b.uniform());
        EXPECT_EQ(std::memcmp(&x, &y, sizeof x), 0);
    }
}

// The surface's own leaf area index: denser canopies return nearer their top, and the default
// overload is LAI 3.
TEST(CanopyReturn, LeafAreaIndexOfTheSurface) {
    const double h = 1.5;
    for (const double lai : {1.0, 4.0, 6.0}) {
        const double mu = kCanopyProjection * lai / h;
        const double path = h / 0.5;
        EXPECT_NEAR(canopyMeanShortening(h, lai, 0.5, 20.0),
                    path - (1.0 - std::exp(-mu * path)) / mu, 1e-12);
    }
    EXPECT_LT(canopyMeanShortening(h, 1.0, 0.5, 20.0), canopyMeanShortening(h, 6.0, 0.5, 20.0));
    EXPECT_EQ(canopyMeanShortening(0.4, 3.0, 0.3, 9.0), canopyMeanShortening(0.4, 0.3, 9.0));
    EXPECT_EQ(canopyShortening(0.4, 0.0, 0.3, 9.0, 0.9), 0.0);
}

// Reeds and shrubs: a level beam over a reed bank (a box whose top is the soil, canopy 1.5 m
// over it) crosses the canopy volume for the bank's width; the share of beams that stop in it is
// 1 - exp(-mu L), mu = G LAI / h, and the mean depth of the stops is the closed form of a
// truncated exponential. Above the canopy and beside the bank nothing stops.
TEST(CanopyVolume, LevelBeamOverAReedBank) {
    const std::array<double, 3> half{2.0, 5.0, 0.5};  // 4 m wide bank, top at z = 0.5
    const Pose bank{{10.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}};
    const double h = 1.5;
    const double lai = 4.0;
    double enter = 0.0;
    double leave = 0.0;
    // A drone's lidar 1 m over the soil, looking along +x at the bank.
    ASSERT_TRUE(canopyVolumeSpan(ShapeType::Box, half, bank, {0.0, 0.0, 1.5}, {1.0, 0.0, 0.0}, h,
                                 50.0, enter, leave));
    EXPECT_NEAR(enter, 8.0, 1e-12);
    EXPECT_NEAR(leave, 12.0, 1e-12);
    // Clipped by the hit.
    ASSERT_TRUE(canopyVolumeSpan(ShapeType::Box, half, bank, {0.0, 0.0, 1.5}, {1.0, 0.0, 0.0}, h,
                                 9.0, enter, leave));
    EXPECT_NEAR(leave, 9.0, 1e-12);
    // Over the canopy's top, and beside the bank.
    EXPECT_FALSE(canopyVolumeSpan(ShapeType::Box, half, bank, {0.0, 0.0, 2.1}, {1.0, 0.0, 0.0}, h,
                                  50.0, enter, leave));
    EXPECT_FALSE(canopyVolumeSpan(ShapeType::Box, half, bank, {0.0, 6.0, 1.5}, {1.0, 0.0, 0.0}, h,
                                  50.0, enter, leave));
    EXPECT_FALSE(canopyVolumeSpan(ShapeType::Box, half, bank, {0.0, 0.0, 1.5}, {1.0, 0.0, 0.0}, 0.0,
                                  50.0, enter, leave));
    // A sphere has no canopy volume.
    EXPECT_FALSE(canopyVolumeSpan(ShapeType::Sphere, half, bank, {0.0, 0.0, 1.5}, {1.0, 0.0, 0.0},
                                  h, 50.0, enter, leave));

    constexpr int kBeams = 200000;
    const double mu = kCanopyProjection * lai / h;
    const double length = 4.0;
    int stopped = 0;
    double depth = 0.0;
    for (int i = 0; i < kBeams; ++i) {
        random::Stream rng(5, 9, static_cast<std::uint32_t>(i));
        const double at = canopyVolumeStop(8.0, 12.0, h, lai, rng.uniform());
        if (std::isfinite(at)) {
            EXPECT_GE(at, 8.0);
            EXPECT_LT(at, 12.0);
            ++stopped;
            depth += at - 8.0;
        }
    }
    const double p = 1.0 - std::exp(-mu * length);
    const double share = static_cast<double>(stopped) / kBeams;
    EXPECT_NEAR(share, p, 5.0 * std::sqrt(p * (1.0 - p) / kBeams));
    // E[s | s < L] = 1/mu - L exp(-mu L) / (1 - exp(-mu L)).
    const double expected = 1.0 / mu - length * std::exp(-mu * length) / p;
    EXPECT_NEAR(depth / stopped, expected, 0.01);
}

// Over a plane the canopy is a slab: a level beam inside it is bounded by its range, a beam
// from above crosses h / c of it.
TEST(CanopyVolume, SlabOverAPlane) {
    const Pose ground{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}};
    double enter = 0.0;
    double leave = 0.0;
    ASSERT_TRUE(canopyVolumeSpan(ShapeType::Plane, {1.0, 1.0, 1.0}, ground, {0.0, 0.0, 0.5},
                                 {0.0, 1.0, 0.0}, 1.0, 30.0, enter, leave));
    EXPECT_EQ(enter, 0.0);
    EXPECT_EQ(leave, 30.0);
    EXPECT_FALSE(canopyVolumeSpan(ShapeType::Plane, {1.0, 1.0, 1.0}, ground, {0.0, 0.0, 0.5},
                                  {0.0, 1.0, 0.0}, 1.0, std::numeric_limits<double>::infinity(),
                                  enter, leave));
    const double c = 0.5;
    const std::array<double, 3> down{std::sqrt(1.0 - c * c), 0.0, -c};
    ASSERT_TRUE(canopyVolumeSpan(ShapeType::Plane, {1.0, 1.0, 1.0}, ground, {0.0, 0.0, 3.0}, down,
                                 1.0, 100.0, enter, leave));
    EXPECT_NEAR(leave - enter, 1.0 / c, 1e-12);
    EXPECT_NEAR(enter, 2.0 / c, 1e-12);
}

}  // namespace
}  // namespace lagrange::sensors
