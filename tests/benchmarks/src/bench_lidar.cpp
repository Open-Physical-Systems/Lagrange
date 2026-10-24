// Lidar rays per second per capability tier (roadmap H6). Baselines: docs/design/benchmarks.md.
//
// The reference is every beam against every shape, exact, in double, on one thread. Tier 0 is
// what the lidar module runs on the CPU: the same exact tests, culled by Embree (ShapeCaster),
// beams in parallel with TBB; bit-identical to the reference. Tier 1 is the compute kernel over
// a bounding-volume hierarchy
// (gpu/kernels/lidar/bvh_traverse.slang). Both cast the same beams into the same random scene.
// LAGRANGE_TEST_GPU picks the Vulkan device, as in the tests.
#include "lagrange/gpu/RayCaster.hpp"
#include "lagrange/sensors/Sensors.hpp"
#include "lagrange/sensors/ShapeCaster.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <random>
#include <string>
#include <tbb/parallel_for.h>
#include <vector>

#include <benchmark/benchmark.h>

namespace lagrange::sensors {
namespace {

// A 32-channel spinning lidar at 1024 columns: one scan.
constexpr std::size_t kBeams = 32 * 1024;

struct SceneShape {
    ShapeType type;
    std::array<double, 3> size;
    Pose pose;
};

struct Scene {
    std::vector<SceneShape> shapes;
    Pose sensor;
    std::vector<std::array<float, 3>> beams;
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

/// A ground plane and `bounded` shapes of every kind within 20 m, beams in all directions.
Scene randomScene(int bounded) {
    std::mt19937_64 random(20261024);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::uniform_real_distribution<double> spread(-20.0, 20.0);
    Scene scene;
    scene.shapes.push_back({ShapeType::Plane, {0, 0, 0}, Pose{}});
    for (int k = 0; k < bounded; ++k) {
        const auto type = static_cast<ShapeType>(1 + static_cast<int>(unit(random) * 4.0) % 4);
        const std::array<double, 3> size{0.05 + 1.2 * unit(random), 0.05 + 1.2 * unit(random),
                                         0.05 + 1.2 * unit(random)};
        scene.shapes.push_back(
            {type,
             size,
             {{spread(random), spread(random), 3.0 * unit(random)}, randomRotation(random)}});
    }
    scene.sensor = {{0.37, -0.21, 0.52}, randomRotation(random)};
    std::normal_distribution<double> normal;
    scene.beams.resize(kBeams);
    for (auto& beam : scene.beams) {
        std::array<double, 3> d{normal(random), normal(random), normal(random)};
        const double n = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        for (std::size_t j = 0; j < 3; ++j) {
            beam.at(j) = static_cast<float>(d.at(j) / n);
        }
    }
    return scene;
}

void LidarReference(benchmark::State& state) {
    const Scene scene = randomScene(static_cast<int>(state.range(0)));
    std::vector<std::array<double, 3>> directions(scene.beams.size());
    for (std::size_t i = 0; i < scene.beams.size(); ++i) {
        const std::array<double, 3> local{scene.beams[i][0], scene.beams[i][1], scene.beams[i][2]};
        directions[i] = rotate(scene.sensor.orientation, local);
    }
    for (auto _ : state) {
        for (const auto& dir : directions) {
            double nearest = kNoHit;
            for (const SceneShape& shape : scene.shapes) {
                nearest = std::min(nearest, raycast(shape.type, shape.size, shape.pose,
                                                    scene.sensor.position, dir));
            }
            benchmark::DoNotOptimize(nearest);
        }
    }
    state.SetItemsProcessed(state.iterations() * static_cast<std::int64_t>(kBeams));
    state.SetLabel("cpu, 1 thread");
}
BENCHMARK(LidarReference)->ArgName("shapes")->Arg(50)->Arg(400)->Unit(benchmark::kMillisecond);

// One scan: shape update (hierarchy refit), then every beam, as the lidar module does.
void LidarTier0(benchmark::State& state) {
    const Scene scene = randomScene(static_cast<int>(state.range(0)));
    std::vector<std::array<double, 3>> directions(scene.beams.size());
    for (std::size_t i = 0; i < scene.beams.size(); ++i) {
        const std::array<double, 3> local{scene.beams[i][0], scene.beams[i][1], scene.beams[i][2]};
        directions[i] = rotate(scene.sensor.orientation, local);
    }
    std::vector<ShapeCaster::Shape> shapes;
    for (const SceneShape& shape : scene.shapes) {
        shapes.push_back({shape.type, shape.size, shape.pose});
    }
    ShapeCaster caster;
    caster.update(shapes);
    std::vector<double> nearest(directions.size());
    for (auto _ : state) {
        caster.update(shapes);
        tbb::parallel_for(std::size_t{0}, directions.size(), [&](std::size_t i) {
            nearest[i] = caster.cast(scene.sensor.position, directions[i]);
        });
        benchmark::DoNotOptimize(nearest.data());
    }
    state.SetItemsProcessed(state.iterations() * static_cast<std::int64_t>(kBeams));
    state.SetLabel("cpu, Embree culling, TBB");
}
BENCHMARK(LidarTier0)
    ->ArgName("shapes")
    ->Arg(50)
    ->Arg(400)
    ->Unit(benchmark::kMillisecond)
    ->UseRealTime();

std::shared_ptr<gpu::RayContext> context() {
    static const std::shared_ptr<gpu::RayContext> made = [] {
        const char* preferred = std::getenv("LAGRANGE_TEST_GPU");
        std::string error;
        return gpu::RayContext::create(preferred != nullptr ? preferred : "", error);
    }();
    return made;
}

// One scan: shape conversion, bounding-volume hierarchy build on the CPU, upload, dispatch and
// read-back, as the lidar module does every update.
void LidarTier1(benchmark::State& state) {
    const auto gpuContext = context();
    if (!gpuContext) {
        state.SkipWithError("no usable Vulkan device");
        return;
    }
    const Scene scene = randomScene(static_cast<int>(state.range(0)));
    gpu::RayScan scan(gpuContext, scene.beams);
    std::vector<gpu::CastHit> hits(scene.beams.size());
    std::vector<gpu::CastShape> shapes(scene.shapes.size());
    for (auto _ : state) {
        for (std::size_t k = 0; k < scene.shapes.size(); ++k) {
            const SceneShape& shape = scene.shapes[k];
            shapes[k] = {static_cast<std::uint32_t>(shape.type),
                         shape.size,
                         {shape.pose.position[0] - scene.sensor.position[0],
                          shape.pose.position[1] - scene.sensor.position[1],
                          shape.pose.position[2] - scene.sensor.position[2]},
                         shape.pose.orientation};
        }
        scan.cast(shapes, scene.sensor.orientation, hits);
        benchmark::DoNotOptimize(hits.data());
    }
    state.SetItemsProcessed(state.iterations() * static_cast<std::int64_t>(kBeams));
    state.SetLabel(gpuContext->deviceName());
}
BENCHMARK(LidarTier1)
    ->ArgName("shapes")
    ->Arg(50)
    ->Arg(400)
    ->Unit(benchmark::kMillisecond)
    ->UseRealTime();

}  // namespace
}  // namespace lagrange::sensors
