// v0.2 acceptance criterion 6: the Tier 1 lidar (compute kernel over a bounding-volume
// hierarchy, in float) agrees with the Tier 0 reference (exact casts in double) within
// docs/design/domains/sensors.md §7: the same shape hit for more than 99.9 % of rays, and the
// range within 1 mm.
#include "lagrange/gpu/RayCaster.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/sensors/Sensors.hpp"
#include "lagrange/world/Model.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <numbers>
#include <random>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::sensors {
namespace {

constexpr double kRangeToleranceM = 1e-3;     // sensors.md §7, tier agreement
constexpr double kSameShapeFraction = 0.999;  // sensors.md §7, tier agreement
// Beyond any lidar's reach (long-range automotive units: about 200-300 m); the module drops
// returns past max_range. A plane met at grazing incidence kilometres away is off by float's
// 1e-7 of the range on the GPU, which is no lidar's concern.
constexpr double kLongestRangeM = 300.0;

/// The device the GPU tests run on: LAGRANGE_TEST_GPU (as the viewer's tests), else automatic,
/// which is the software device (llvmpipe) on a CI runner without a GPU. Same kernel either way.
std::shared_ptr<gpu::RayContext> context() {
    static const std::shared_ptr<gpu::RayContext> made = [] {
        const char* preferred = std::getenv("LAGRANGE_TEST_GPU");
        std::string error;
        auto created = gpu::RayContext::create(preferred != nullptr ? preferred : "", error);
        if (!created) {
            std::fprintf(stderr, "no Vulkan device for the GPU lidar: %s\n", error.c_str());
        } else {
            std::fprintf(stderr, "GPU lidar on %s\n", created->deviceName().c_str());
        }
        return created;
    }();
    return made;
}

struct SceneShape {
    ShapeType type;
    std::array<double, 3> size;
    Pose pose;
};

std::array<double, 4> randomRotation(std::mt19937_64& random) {
    std::normal_distribution<double> normal;
    std::array<double, 4> q{normal(random), normal(random), normal(random), normal(random)};
    const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    for (double& v : q) {
        v /= n;
    }
    return q;
}

TEST(LidarTiers, RandomScenesAgreeWithTheCpuReference) {
    const auto gpuContext = context();
    if (!gpuContext) {
        GTEST_SKIP() << "no usable Vulkan device";
    }
    std::mt19937_64 random(20261018);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::uniform_real_distribution<double> spread(-20.0, 20.0);

    // A ground plane, a tilted plane far away, and 400 shapes of every kind around the sensor.
    std::vector<SceneShape> scene;
    scene.push_back({ShapeType::Plane, {0, 0, 0}, Pose{}});
    scene.push_back({ShapeType::Plane,
                     {0, 0, 0},
                     {{40.0, 0.0, 0.0}, {std::cos(0.7), 0.0, std::sin(0.7), 0.0}}});
    for (int k = 0; k < 400; ++k) {
        const auto type = static_cast<ShapeType>(1 + static_cast<int>(unit(random) * 4.0) % 4);
        const std::array<double, 3> size{0.05 + 1.2 * unit(random), 0.05 + 1.2 * unit(random),
                                         0.05 + 1.2 * unit(random)};
        scene.push_back(
            {type,
             size,
             {{spread(random), spread(random), 3.0 * unit(random)}, randomRotation(random)}});
    }
    // Far from the region origin's 0, as a robot that drove: positions still small after the
    // sensor-centred conversion.
    const Pose sensor{{0.37, -0.21, 0.52}, randomRotation(random)};

    // 100 000 beams in all directions, in the sensor frame.
    std::vector<std::array<float, 3>> beams(100'000);
    std::normal_distribution<double> normal;
    std::vector<std::array<double, 3>> beamsDouble(beams.size());
    for (std::size_t i = 0; i < beams.size(); ++i) {
        std::array<double, 3> d{normal(random), normal(random), normal(random)};
        const double n = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        for (std::size_t j = 0; j < 3; ++j) {
            d.at(j) /= n;
            beams[i].at(j) = static_cast<float>(d.at(j));
        }
        beamsDouble[i] = d;
    }

    // Tier 0: exact, in double, in the world frame.
    std::vector<double> cpuRange(beams.size(), kNoHit);
    std::vector<std::uint32_t> cpuShape(beams.size(), gpu::kNoShape);
    for (std::size_t i = 0; i < beams.size(); ++i) {
        // The beam as the GPU sees it (float), so that both tiers cast the same ray.
        const std::array<double, 3> local{beams[i][0], beams[i][1], beams[i][2]};
        const std::array<double, 3> dir = rotate(sensor.orientation, local);
        for (std::size_t k = 0; k < scene.size(); ++k) {
            const double t =
                raycast(scene[k].type, scene[k].size, scene[k].pose, sensor.position, dir);
            if (t < cpuRange[i]) {
                cpuRange[i] = t;
                cpuShape[i] = static_cast<std::uint32_t>(k);
            }
        }
    }

    // Tier 1: the kernel, in float, centred on the sensor.
    std::vector<gpu::CastShape> shapes(scene.size());
    for (std::size_t k = 0; k < scene.size(); ++k) {
        shapes[k].type = static_cast<std::uint32_t>(scene[k].type);
        shapes[k].size = scene[k].size;
        for (std::size_t j = 0; j < 3; ++j) {
            shapes[k].position.at(j) = scene[k].pose.position.at(j) - sensor.position.at(j);
        }
        shapes[k].orientation = scene[k].pose.orientation;
    }
    gpu::RayScan scan(gpuContext, beams);
    std::vector<gpu::CastHit> hits(beams.size());
    scan.cast(shapes, sensor.orientation, hits);

    std::size_t same = 0, returns = 0, outOfTolerance = 0;
    double worst = 0.0;
    for (std::size_t i = 0; i < beams.size(); ++i) {
        returns += std::isinf(cpuRange[i]) ? 0 : 1;
        if (hits[i].shape != cpuShape[i]) {
            continue;
        }
        ++same;
        if (std::isinf(cpuRange[i])) {
            EXPECT_TRUE(std::isinf(hits[i].range)) << "beam " << i;
            continue;
        }
        if (cpuRange[i] > kLongestRangeM) {
            continue;
        }
        const double error = std::abs(static_cast<double>(hits[i].range) - cpuRange[i]);
        worst = std::max(worst, error);
        outOfTolerance += error > kRangeToleranceM ? 1 : 0;
    }
    const double fraction = static_cast<double>(same) / static_cast<double>(beams.size());
    std::printf("%s: same shape for %.4f %% of %zu beams (%zu returns), worst range error %.3g m\n",
                gpuContext->deviceName().c_str(), 100.0 * fraction, beams.size(), returns, worst);
    EXPECT_GT(returns, beams.size() / 2) << "the scene is hit, not empty sky";
    EXPECT_GT(fraction, kSameShapeFraction);
    EXPECT_EQ(outOfTolerance, 0U) << "worst " << worst << " m";
}

/// An octahedron with half-diagonals `half`: a hull whose faces are not axis-aligned.
ConvexHull octahedron(const std::array<double, 3>& half) {
    ConvexHull hull;
    hull.points = {{half[0], 0, 0},  {-half[0], 0, 0}, {0, half[1], 0},
                   {0, -half[1], 0}, {0, 0, half[2]},  {0, 0, -half[2]}};
    for (const std::uint32_t x : {0U, 1U}) {
        for (const std::uint32_t y : {2U, 3U}) {
            for (const std::uint32_t z : {4U, 5U}) {
                hull.triangles.push_back({x, y, z});  // the winding does not matter
            }
        }
    }
    return hull;
}

// Convex hulls (sensors.md §7 tier agreement): the kernel's face-plane test against the CPU's.
TEST(LidarTiers, ConvexHullsAgreeWithTheCpuReference) {
    const auto gpuContext = context();
    if (!gpuContext) {
        GTEST_SKIP() << "no usable Vulkan device";
    }
    std::mt19937_64 random(20261025);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::uniform_real_distribution<double> spread(-10.0, 10.0);
    std::vector<ConvexHull> hulls;
    std::vector<Pose> poses;
    for (int k = 0; k < 200; ++k) {
        const std::array<double, 3> half{0.05 + unit(random), 0.05 + unit(random),
                                         0.05 + unit(random)};
        hulls.push_back(k % 2 == 0 ? octahedron(half) : boxHull(half));
        poses.push_back(
            {{spread(random), spread(random), 2.0 * unit(random)}, randomRotation(random)});
    }
    std::vector<std::vector<std::array<double, 4>>> faces(hulls.size());
    std::vector<gpu::CastShape> shapes(hulls.size());
    const Pose sensor{{0.37, -0.21, 0.52}, randomRotation(random)};
    for (std::size_t k = 0; k < hulls.size(); ++k) {
        facePlanes(hulls[k], faces[k]);
        shapes[k].type = static_cast<std::uint32_t>(ShapeType::ConvexHull);
        for (const auto& p : hulls[k].points) {
            for (std::size_t j = 0; j < 3; ++j) {
                shapes[k].size.at(j) = std::max(shapes[k].size.at(j), std::abs(p.at(j)));
            }
        }
        for (std::size_t j = 0; j < 3; ++j) {
            shapes[k].position.at(j) = poses[k].position.at(j) - sensor.position.at(j);
        }
        shapes[k].orientation = poses[k].orientation;
        shapes[k].faces = faces[k];
    }

    std::vector<std::array<float, 3>> beams(50'000);
    std::normal_distribution<double> normal;
    for (auto& beam : beams) {
        std::array<double, 3> d{normal(random), normal(random), normal(random)};
        const double n = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        for (std::size_t j = 0; j < 3; ++j) {
            beam.at(j) = static_cast<float>(d.at(j) / n);
        }
    }
    gpu::RayScan scan(gpuContext, beams);
    std::vector<gpu::CastHit> hits(beams.size());
    scan.cast(shapes, sensor.orientation, hits);

    std::size_t same = 0, returns = 0, outOfTolerance = 0;
    double worst = 0.0;
    for (std::size_t i = 0; i < beams.size(); ++i) {
        const std::array<double, 3> dir =
            rotate(sensor.orientation, {beams[i][0], beams[i][1], beams[i][2]});
        double range = kNoHit;
        std::uint32_t shape = gpu::kNoShape;
        for (std::size_t k = 0; k < hulls.size(); ++k) {
            const double t = raycast(ShapeType::ConvexHull, shapes[k].size, poses[k],
                                     sensor.position, dir, &hulls[k]);
            if (t < range) {
                range = t;
                shape = static_cast<std::uint32_t>(k);
            }
        }
        returns += std::isinf(range) ? 0 : 1;
        if (hits[i].shape != shape) {
            continue;
        }
        ++same;
        if (!std::isinf(range)) {
            const double error = std::abs(static_cast<double>(hits[i].range) - range);
            worst = std::max(worst, error);
            outOfTolerance += error > kRangeToleranceM ? 1 : 0;
        }
    }
    const double fraction = static_cast<double>(same) / static_cast<double>(beams.size());
    std::printf("hulls on %s: same shape for %.4f %% of %zu beams (%zu returns), worst %.3g m\n",
                gpuContext->deviceName().c_str(), 100.0 * fraction, beams.size(), returns, worst);
    EXPECT_GT(returns, beams.size() / 10) << "the hulls are hit";
    EXPECT_GT(fraction, kSameShapeFraction);
    EXPECT_EQ(outOfTolerance, 0U) << "worst " << worst << " m";
}

World room() {
    World world;
    world.addCollider(kWorldBody, ShapeType::Plane, {0, 0, 0}, Pose{}, 1.0);
    world.addCollider(kWorldBody, ShapeType::Box, {3.0, 0.1, 1.0}, {{0, 2.1, 1}}, 1.0);
    world.addCollider(kWorldBody, ShapeType::Box, {3.0, 0.1, 1.0}, {{0, -2.1, 1}}, 1.0);
    world.addCollider(kWorldBody, ShapeType::Box, {0.1, 2.0, 1.0}, {{3.1, 0, 1}}, 1.0);
    world.addCollider(kWorldBody, ShapeType::Box, {0.1, 2.0, 1.0}, {{-3.1, 0, 1}}, 1.0);
    world.addCollider(kWorldBody, ShapeType::Cylinder, {0.3, 0.5, 0}, {{1.5, -1.0, 0.5}}, 1.0);
    world.addCollider(kWorldBody, ShapeType::Sphere, {0.4, 0, 0}, {{-1.5, 1.0, 0.4}}, 1.0);
    world.addHullCollider(kWorldBody, octahedron({0.4, 0.3, 0.5}), {{1.0, 1.2, 0.5}}, 1.0);
    const BodyId post = world.addBody("post", kWorldBody, {{0.2, 0.1, 0}}, 0.0, {}, {});
    world.addSensor("scanner", SensorType::Lidar, post, {{0, 0, 0.5}}, 10.0,
                    {{"samples", 360.0},
                     {"min_angle", -std::numbers::pi},
                     {"max_angle", std::numbers::pi},
                     {"vertical_samples", 16.0},
                     {"vertical_min_angle", -0.26},
                     {"vertical_max_angle", 0.26},
                     {"min_range", 0.1},
                     {"max_range", 30.0},
                     {"range_noise", 0.01}});
    return world;
}

std::vector<double> scanOnce(const World& world, std::shared_ptr<gpu::RayContext> gpuContext) {
    Scheduler scheduler;
    ModuleConfigData config;
    config.tier = gpuContext ? CapabilityTier::Tier1 : CapabilityTier::Tier0;
    for (auto& module : createSensorModules(world, gpuContext)) {
        scheduler.add(std::move(module), ModuleConfig(config));
    }
    auto wired = scheduler.wire(WorldView(world));
    EXPECT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    EXPECT_TRUE(scheduler.stepTick().ok());
    const std::span<const double> r =
        scheduler.bus().readN(scheduler.bus().find("sensor/scanner/ranges"));
    return {r.begin(), r.end()};
}

// The module on either tier: the same ranges, and the same noise (Philox on the CPU after the
// cast), so a noisy scan differs between tiers only by the cast itself.
TEST(LidarTiers, TheGpuLidarModuleScansLikeTheCpuOne) {
    const auto gpuContext = context();
    if (!gpuContext) {
        GTEST_SKIP() << "no usable Vulkan device";
    }
    const World world = room();
    const std::vector<double> cpu = scanOnce(world, nullptr);
    const std::vector<double> gpu = scanOnce(world, gpuContext);
    ASSERT_EQ(cpu.size(), 360U * 16U);
    ASSERT_EQ(gpu.size(), cpu.size());
    for (std::size_t i = 0; i < cpu.size(); ++i) {
        ASSERT_FALSE(std::isinf(cpu[i])) << "a closed room: every beam returns";
        EXPECT_NEAR(gpu[i], cpu[i], kRangeToleranceM) << "beam " << i;
    }
}

TEST(LidarTiers, TheGpuLidarIsATier1ModuleAndNotDeterministic) {
    const auto gpuContext = context();
    if (!gpuContext) {
        GTEST_SKIP() << "no usable Vulkan device";
    }
    const World world = room();
    auto modules = createSensorModules(world, gpuContext);
    ASSERT_EQ(modules.size(), 1U);
    // The same name as on the CPU: the noise stream is keyed by it, so both tiers add the same
    // noise.
    EXPECT_EQ(modules[0]->info().name, "lidar_scanner");
    EXPECT_FALSE(modules[0]->info().deterministic);
    EXPECT_EQ(modules[0]->info().minimumTier, CapabilityTier::Tier1);
    // A deterministic run refuses it: the CPU path is the reference (invariant 7).
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    ModuleConfigData config;
    config.tier = CapabilityTier::Tier1;
    scheduler.add(std::move(modules[0]), ModuleConfig(config));
    EXPECT_FALSE(scheduler.wire(WorldView(world)));
}

}  // namespace
}  // namespace lagrange::sensors
