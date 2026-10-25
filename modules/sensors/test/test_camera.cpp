// The raster camera (sensors.md, camera level `raster`): what it sees of a known scene, its
// optical-frame convention, and its ground-truth substitute. The projection itself is checked
// against OpenCV in gpu/raster/test/test_raster.cpp.
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/sensors/Sensors.hpp"

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

}  // namespace
}  // namespace lagrange::sensors
