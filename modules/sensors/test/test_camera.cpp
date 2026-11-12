// The raster camera (sensors.md, camera level `raster`): what it sees of a known scene, its
// optical-frame convention, and its ground-truth substitute. The projection itself is checked
// against OpenCV in gpu/raster/test/test_raster.cpp.
#include "lagrange/gpu/Raster.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/sensors/CameraModel.hpp"
#include "lagrange/sensors/Sensors.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

#include <gtest/gtest.h>

namespace lagrange::sensors {
namespace {

using namespace literals;

constexpr std::uint32_t kWidth = 320;
constexpr std::uint32_t kHeight = 240;

/// A camera 1 m above the origin looking along +x, a 1 m cube centred 3 m ahead (its face at
/// 2.5 m) and a small cube ahead and to the left (+y).
Model scene() {
    Model model;
    World& w = model.world;
    const BodyId post = w.addBody("post", kWorldBody, Pose{}, 0.0, {}, {});
    w.addSensor("front_camera", SensorType::Camera, post, {{0, 0, 1}}, 30.0,
                {{"width", kWidth}, {"height", kHeight}, {"horizontal_fov", 1.2}});
    model.scene.meshes.push_back(boxMesh(1.0, 1.0, 1.0));
    model.scene.materials.push_back({"grey", {0.6F, 0.6F, 0.6F, 1.0F}});
    ScenePrim cube;
    cube.name = "cube";
    cube.mesh = 0;
    cube.material = 0;
    cube.offset = {{3, 0, 1}};
    model.scene.prims.push_back(cube);
    ScenePrim left = cube;
    left.name = "left";
    left.offset = {{3, 1.5, 1}};
    left.scale = {0.3F, 0.3F, 0.3F};
    model.scene.prims.push_back(left);
    return model;
}

struct Rig {
    Scheduler scheduler{SchedulerOptions{}};
    std::shared_ptr<const Buffer> image, depth, instance;

    void step() {
        ASSERT_TRUE(scheduler.stepTick().ok());
        const SignalBus& bus = scheduler.bus();
        image = bus.buffer(bus.readBuffer(bus.find("sensor/front_camera/image")));
        depth = bus.buffer(bus.readBuffer(bus.find("sensor/front_camera/depth")));
        instance = bus.buffer(bus.readBuffer(bus.find("sensor/front_camera/instance")));
    }
};

template <typename T>
T pixel(const Buffer& buffer, std::uint32_t u, std::uint32_t v) {
    T out{};
    std::memcpy(&out, buffer.bytes.data() + (std::size_t{v} * buffer.width + u) * sizeof(T),
                sizeof(T));
    return out;
}

TEST(Camera, SeesTheSceneInTheOpticalFrame) {
    const Model model = scene();
    std::string error;
    const char* gpu = std::getenv("LAGRANGE_TEST_GPU");
    const std::shared_ptr<CameraScene> cameras =
        createCameraScene(model, gpu != nullptr ? gpu : "", error);
    if (!cameras) {
        GTEST_SKIP() << "no Vulkan device: " << error;
    }
    Rig run;
    auto modules = createSensorModules(model.world, nullptr, cameras);
    ASSERT_EQ(modules.size(), 1U);
    EXPECT_EQ(modules[0]->preferredStep(), 33_ms);  // 30 Hz, on whole milliseconds
    EXPECT_FALSE(modules[0]->info().deterministic);
    run.scheduler.add(std::move(modules[0]));
    auto wired = run.scheduler.wire(WorldView(model.world));
    ASSERT_TRUE(wired) << to_string(wired.error().front());
    run.step();
    ASSERT_TRUE(run.image && run.depth && run.instance);
    EXPECT_EQ(run.image->encoding, "rgba8");
    EXPECT_EQ(run.depth->encoding, "r32f");
    EXPECT_EQ(run.instance->encoding, "r32u");
    ASSERT_EQ(run.image->bytes.size(), std::size_t{kWidth} * kHeight * 4);
    ASSERT_EQ(run.depth->width, kWidth);

    // The image centre is on the optical axis: the big cube's face, 2.5 m ahead.
    EXPECT_NEAR(pixel<float>(*run.depth, kWidth / 2, kHeight / 2), 2.5F, 1e-3F);
    EXPECT_EQ(pixel<std::uint32_t>(*run.instance, kWidth / 2, kHeight / 2), 0U);
    // +y of the sensor is left in the image: the small cube projects at u = cx - fx * 1.5 / 2.85.
    const double fx = kWidth / 2.0 / std::tan(0.6);
    const auto u = static_cast<std::uint32_t>(std::lround((kWidth - 1) / 2.0 - fx * 1.5 / 2.85));
    EXPECT_EQ(pixel<std::uint32_t>(*run.instance, u, kHeight / 2), 1U);
    EXPECT_NEAR(pixel<float>(*run.depth, u, kHeight / 2), 2.85F, 1e-3F);
    // The cube's top edge, 0.5 m above the axis, is at v = cy - fy * 0.5 / 2.5 = 72.7; above it
    // there is only sky.
    EXPECT_EQ(pixel<std::uint32_t>(*run.instance, kWidth / 2, 76), 0U);
    EXPECT_EQ(pixel<std::uint32_t>(*run.instance, kWidth / 2, 69), gpu::kNoInstance);
    EXPECT_TRUE(std::isinf(pixel<float>(*run.depth, kWidth / 2, 69)));
}

TEST(Camera, GroundTruthPublishesEmptyImagesThatKeepTheirSize) {
    const Model model = scene();
    Rig run;
    for (auto& module : createSensorSubstitutes(model.world)) {
        EXPECT_TRUE(module->info().deterministic);
        run.scheduler.add(std::move(module));
    }
    ASSERT_TRUE(run.scheduler.wire(WorldView(model.world)));
    run.step();
    ASSERT_TRUE(run.image);
    EXPECT_EQ(run.image->width, kWidth);
    EXPECT_EQ(run.image->height, kHeight);
    EXPECT_TRUE(run.image->bytes.empty());
}

TEST(Camera, WithoutADeviceItRefusesToStartAndSaysWhatToDo) {
    const Model model = scene();
    Scheduler scheduler;
    for (auto& module : createSensorModules(model.world)) {
        scheduler.add(std::move(module));
    }
    auto wired = scheduler.wire(WorldView(model.world));
    ASSERT_FALSE(wired);
    EXPECT_NE(to_string(wired.error().front()).find("ground_truth"), std::string::npos);
}

// The camera model on the GPU (gpu/kernels/camera/camera_model.slang) against the CPU
// reference, CameraModel::apply, on the same render: a checkerboard of colours seen through a
// barrel lens with vignetting, a rolling shutter while turning, and an exposure with noise. The
// remap is the same (instance ids and depth), and the noise draws the same random bits, so the
// codes agree but where float and double round apart.
class GpuCameraModel : public ::testing::TestWithParam<bool> {};

TEST_P(GpuCameraModel, AgreesWithTheCpuReference) {
    const bool exposure = GetParam();
    // Coloured squares on a plane 4 m ahead, the sky round them, and a tilted plate in front.
    std::vector<float> positions;
    std::vector<float> normals;
    std::vector<std::uint32_t> indices;
    const auto quad = [&](std::array<float, 3> a, std::array<float, 3> b, std::array<float, 3> c,
                          std::array<float, 3> d) {
        const auto base = static_cast<std::uint32_t>(positions.size() / 3);
        for (const auto& p : {a, b, c, d}) {
            positions.insert(positions.end(), p.begin(), p.end());
            normals.insert(normals.end(), {0, 0, -1});
        }
        indices.insert(indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
    };
    quad({-0.5F, -0.5F, 0}, {0.5F, -0.5F, 0}, {0.5F, 0.5F, 0}, {-0.5F, 0.5F, 0});
    const std::array<gpu::MeshSource, 1> meshes{gpu::MeshSource{positions, normals, indices}};
    std::string error;
    const char* device = std::getenv("LAGRANGE_TEST_GPU");
    auto scene = gpu::RasterScene::create(device != nullptr ? device : "", meshes, error);
    if (!scene) {
        GTEST_SKIP() << "no Vulkan device: " << error;
    }
    std::vector<gpu::RasterInstance> instances;
    for (int row = -1; row < 1; ++row) {
        for (int column = -2; column < 2; ++column) {
            gpu::RasterInstance square;
            square.position = {column + 0.5, row + 0.5, 4.0};
            square.color = {0.1F + 0.2F * static_cast<float>(column + 2),
                            0.2F + 0.4F * static_cast<float>(row + 1),
                            ((row + column) % 2 == 0) ? 0.9F : 0.05F, 1.0F};
            square.id = static_cast<std::uint32_t>(instances.size());
            instances.push_back(square);
        }
    }
    gpu::RasterInstance plate;
    plate.position = {0.3, 0.2, 2.0};
    plate.rotation = {std::cos(0.3), 0.0, std::sin(0.3), 0.0};
    plate.scale = {0.8F, 0.6F, 1.0F};
    plate.color = {0.6F, 0.6F, 0.6F, 1.0F};
    plate.id = 500;
    instances.push_back(plate);

    CameraModelConfig config;
    config.width = 320;
    config.height = 240;
    config.fx = config.fy = 260.0;
    config.cx = 159.5;
    config.cy = 119.5;
    config.lens.model = Lens::Model::PlumbBob;
    config.lens.k = {-0.12, 0.02, 0.001, -0.001, 0.0};
    config.vignetting = 0.4;
    config.rollingShutterTime = 0.03;
    if (exposure) {
        Exposure e;
        e.time = 0.01;
        e.autoExposure = true;
        e.electronRate = 5e7;  // bright: auto exposure shortens the next frame
        e.readNoise = 3.0;
        e.fullWell = 1e4;
        e.bitDepth = 12;
        config.exposure = e;
    }
    CameraModel cpuModel(config);
    CameraModel gpuModel(config);
    gpu::PinholeCamera render = cpuModel.renderCamera();
    render.near = 0.05;
    gpu::RasterCamera cpuCamera(scene, render.width, render.height);
    gpu::RasterCamera gpuCamera(scene, render.width, render.height);
    ASSERT_TRUE(gpuCamera.setEffects(gpuModel.gpuEffects(), error)) << error;
    const std::array<double, 3> omega{0.2, -0.8, 0.1};  // rad/s
    constexpr std::uint64_t kSeed = 0x1234'5678'9ABC'DEF0ULL;
    for (std::uint64_t step = 7; step < 9; ++step) {
        gpu::RasterImages rendered;
        gpu::RasterImages cpu;
        cpuCamera.render(instances, render, rendered);
        cpuModel.apply(rendered, omega, kSeed, step, 3U << 16U, cpu);
        gpu::RasterImages gpu;
        std::vector<float> signal;
        gpuCamera.render(instances, render, gpuModel.gpuFrame(omega, kSeed, step, 3U << 16U), gpu,
                         signal);
        gpuModel.afterGpuFrame(signal);
        const std::size_t pixels = std::size_t{config.width} * config.height;
        ASSERT_EQ(gpu.rgba.size(), pixels * 4);
        ASSERT_EQ(gpu.instance.size(), pixels);
        std::size_t sameInstance = 0;
        std::size_t seen = 0;
        double worstDepth = 0.0;
        for (std::size_t i = 0; i < pixels; ++i) {
            if (gpu.instance[i] != cpu.instance[i]) {
                continue;
            }
            ++sameInstance;
            if (cpu.instance[i] != gpu::kNoInstance) {
                ++seen;
                worstDepth = std::max(
                    worstDepth,
                    std::abs(static_cast<double>(gpu.depth[i]) - cpu.depth[i]) / cpu.depth[i]);
            }
        }
        // Only where the float and double remaps round to different render pixels.
        EXPECT_GE(sameInstance, pixels * 998 / 1000);
        EXPECT_GT(seen, pixels / 4);
        EXPECT_LT(seen, pixels);
        EXPECT_LT(worstDepth, 1e-4);
        std::size_t close = 0;
        std::size_t exact = 0;
        double sum = 0.0;
        for (std::size_t k = 0; k < gpu.rgba.size(); ++k) {
            const int d = static_cast<int>(gpu.rgba[k]) - static_cast<int>(cpu.rgba[k]);
            close += std::abs(d) <= 2 ? 1 : 0;
            exact += d == 0 ? 1 : 0;
            sum += d;
        }
        // On the RTX 4060 all but about 10 of 307 200 channels are the same code.
        EXPECT_GE(close, gpu.rgba.size() * 999 / 1000);
        EXPECT_GE(exact, gpu.rgba.size() * 99 / 100);
        EXPECT_LT(std::abs(sum / static_cast<double>(gpu.rgba.size())), 0.02);
        EXPECT_NEAR(gpuModel.exposureTime(), cpuModel.exposureTime(),
                    1e-4 * cpuModel.exposureTime() + 1e-12);
    }
}

INSTANTIATE_TEST_SUITE_P(ExposureOrNot, GpuCameraModel, ::testing::Bool());

}  // namespace
}  // namespace lagrange::sensors
