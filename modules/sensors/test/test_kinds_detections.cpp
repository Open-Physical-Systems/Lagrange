// detections (sensors.md §9): boxes at known poses in front of a camera give the 2D boxes of
// their projection within a pixel, the occlusion of the one behind, their classes from their
// names, their 3D boxes in the optical frame, and a semantic image. Renders on the software
// Vulkan device (llvmpipe) where there is no GPU; skipped without any device.
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/sensors/Kinds.hpp"
#include "lagrange/sensors/Sensors.hpp"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <numbers>
#include <string>

#include <gtest/gtest.h>

namespace lagrange::sensors {
namespace {

constexpr std::uint32_t kWidth = 320;
constexpr std::uint32_t kHeight = 240;
constexpr double kFov = 1.2;  // rad
constexpr std::size_t kFields = 16;

/// A camera 1 m up looking along +x; a 1 m crate centred 3 m ahead, a 0.4 m pole in front of
/// its middle, and a small tree to the left.
Model scene(std::map<std::string, std::string, std::less<>> text = {}) {
    Model model;
    World& w = model.world;
    const BodyId post = w.addBody("post", kWorldBody, Pose{}, 0.0, {}, {});
    w.addSensor("front_camera", SensorType::Camera, post, {{0, 0, 1}}, 30.0,
                {{"width", kWidth}, {"height", kHeight}, {"horizontal_fov", kFov}});
    text.emplace("camera", "front_camera");
    w.addCustomSensor("labels", "detections", post, Pose{}, 30.0, {}, std::move(text));
    model.scene.meshes.push_back(boxMesh(1.0, 1.0, 1.0));
    model.scene.materials.push_back({"grey", {0.6F, 0.6F, 0.6F, 1.0F}});
    ScenePrim crate;
    crate.name = "crate_big";
    crate.path = "/yard/crate_big";
    crate.mesh = 0;
    crate.material = 0;
    crate.offset = {{3, 0, 1}};
    model.scene.prims.push_back(crate);
    ScenePrim pole = crate;
    pole.name = "pole_front";
    pole.path = "/yard/pole_front";
    pole.offset = {{2, 0, 1}};
    pole.scale = {0.4F, 0.4F, 0.4F};
    model.scene.prims.push_back(pole);
    ScenePrim tree = crate;
    tree.name = "tree_left";
    tree.path = "/yard/tree_left";
    tree.offset = {{3, 1.5, 1}};
    tree.scale = {0.3F, 0.3F, 0.3F};
    model.scene.prims.push_back(tree);
    return model;
}

std::uint32_t pixel(const Buffer& b, std::uint32_t u, std::uint32_t v) {
    std::uint32_t out = 0;
    std::memcpy(&out, b.bytes.data() + (std::size_t{v} * b.width + u) * 4, 4);
    return out;
}

TEST(Detections, IsABuiltinKind) {
    EXPECT_TRUE(isBuiltinKind("detections"));
}

TEST(Detections, BoxesOcclusionClassesAndTheSemanticImage) {
    const Model model = scene();
    std::string error;
    const char* gpu = std::getenv("LAGRANGE_TEST_GPU");
    const std::shared_ptr<CameraScene> cameras =
        createCameraScene(model, gpu != nullptr ? gpu : "", error);
    if (!cameras) {
        GTEST_SKIP() << "no Vulkan device: " << error;
    }
    Scheduler scheduler{SchedulerOptions{}};
    for (auto& m : createSensorModules(model, nullptr, cameras)) {
        scheduler.add(std::move(m));
    }
    auto wired = scheduler.wire(WorldView(model.world));
    ASSERT_TRUE(wired) << to_string(wired.error().front());
    ASSERT_TRUE(scheduler.stepTick().ok());
    const SignalBus& bus = scheduler.bus();
    ASSERT_EQ(bus.read(bus.find("sensor/labels/count")), 3.0);
    const std::span<const double> rows = bus.readN(bus.find("sensor/labels/detections"));

    const double fx = kWidth / 2.0 / std::tan(kFov / 2.0);
    const double cx = (kWidth - 1) / 2.0;
    const double cy = (kHeight - 1) / 2.0;
    // Largest first: the crate. Its face at 2.5 m spans ±0.5 m: u = cx ± fx 0.5 / 2.5.
    const std::span<const double> crate = rows.subspan(0, kFields);
    EXPECT_EQ(crate[0], 0.0);  // scene prim 0
    EXPECT_EQ(crate[1], 8.0);  // "crate": class 8
    EXPECT_NEAR(crate[2], std::ceil(cx - fx * 0.2), 1.0);
    EXPECT_NEAR(crate[3], std::ceil(cy - fx * 0.2), 1.0);
    EXPECT_NEAR(crate[4], std::floor(cx + fx * 0.2), 1.0);
    EXPECT_NEAR(crate[5], std::floor(cy + fx * 0.2), 1.0);
    // The pole's face at 1.8 m covers the middle of the crate's: (0.2/1.8 / 0.2)² of it.
    const double hidden = std::pow(0.2 / 1.8 / 0.2, 2.0);
    EXPECT_NEAR(crate[7], hidden, 0.02);
    EXPECT_NEAR(crate[8], 0.0, 1e-9);  // wholly in the image
    // Its 3D box: 3 m ahead on the optical axis, 1 m each way, facing away (rotation_y -π/2).
    EXPECT_NEAR(crate[9], 0.0, 1e-9);
    EXPECT_NEAR(crate[10], 0.0, 1e-9);
    EXPECT_NEAR(crate[11], 3.0, 1e-6);
    EXPECT_NEAR(crate[12], 1.0, 1e-6);
    EXPECT_NEAR(crate[13], 1.0, 1e-6);
    EXPECT_NEAR(crate[14], 1.0, 1e-6);
    EXPECT_NEAR(crate[15], -std::numbers::pi / 2.0, 1e-9);

    const std::span<const double> pole = rows.subspan(kFields, kFields);
    EXPECT_EQ(pole[0], 1.0);
    EXPECT_EQ(pole[1], 7.0);  // "pole"
    EXPECT_NEAR(pole[2], std::ceil(cx - fx * 0.2 / 1.8), 1.0);
    EXPECT_NEAR(pole[4], std::floor(cx + fx * 0.2 / 1.8), 1.0);
    EXPECT_NEAR(pole[7], 0.0, 0.01);  // nothing in front of it
    EXPECT_NEAR(pole[11], 2.0, 1e-6);
    EXPECT_NEAR(pole[12], 0.4, 1e-6);

    const std::span<const double> tree = rows.subspan(2 * kFields, kFields);
    EXPECT_EQ(tree[0], 2.0);
    EXPECT_EQ(tree[1], 6.0);  // "tree"
    // 1.5 m to the left (optical x = -1.5), 0.3 m cube: its outer edge is its front face's (at
    // 2.85 m), its inner edge its side's far corner (1.35 m left at 3.15 m), which faces the
    // camera.
    EXPECT_NEAR(tree[2], std::ceil(cx - fx * 1.65 / 2.85), 1.0);
    EXPECT_NEAR(tree[4], std::floor(cx - fx * 1.35 / 3.15), 1.0);
    EXPECT_NEAR(tree[9], -1.5, 1e-6);
    EXPECT_NEAR(tree[11], 3.0, 1e-6);
    EXPECT_TRUE(std::isnan(rows[3 * kFields]));  // nothing more

    // The semantic image: the class of whatever each pixel shows, 0 for the sky.
    const std::shared_ptr<const Buffer> semantic =
        bus.buffer(bus.readBuffer(bus.find("sensor/labels/semantic")));
    ASSERT_TRUE(semantic);
    ASSERT_EQ(semantic->width, kWidth);
    ASSERT_EQ(semantic->encoding, "r32u");
    EXPECT_EQ(pixel(*semantic, kWidth / 2, kHeight / 2), 7U);
    EXPECT_EQ(pixel(*semantic, kWidth / 2, static_cast<std::uint32_t>(cy - fx * 0.18)), 8U);
    EXPECT_EQ(pixel(*semantic, static_cast<std::uint32_t>(cx - fx * 1.5 / 2.85), kHeight / 2), 6U);
    EXPECT_EQ(pixel(*semantic, kWidth / 2, 5), 0U);
}

TEST(Detections, ClassesFromTheSemanticParameter) {
    const Model model = scene({{"semantic", "crate=3;pole,tree=11"}});
    std::string error;
    const char* gpu = std::getenv("LAGRANGE_TEST_GPU");
    const std::shared_ptr<CameraScene> cameras =
        createCameraScene(model, gpu != nullptr ? gpu : "", error);
    if (!cameras) {
        GTEST_SKIP() << "no Vulkan device: " << error;
    }
    Scheduler scheduler{SchedulerOptions{}};
    for (auto& m : createSensorModules(model, nullptr, cameras)) {
        scheduler.add(std::move(m));
    }
    ASSERT_TRUE(scheduler.wire(WorldView(model.world)));
    ASSERT_TRUE(scheduler.stepTick().ok());
    const SignalBus& bus = scheduler.bus();
    const std::span<const double> rows = bus.readN(bus.find("sensor/labels/detections"));
    EXPECT_EQ(rows[1], 3.0);
    EXPECT_EQ(rows[kFields + 1], 11.0);
    EXPECT_EQ(rows[2 * kFields + 1], 11.0);
}

TEST(Detections, GroundTruthCamerasGiveNoDetections) {
    const Model model = scene();
    Scheduler scheduler{SchedulerOptions{}};
    for (auto& m : createSensorSubstitutes(model)) {
        scheduler.add(std::move(m));
    }
    ASSERT_TRUE(scheduler.wire(WorldView(model.world)));
    ASSERT_TRUE(scheduler.stepTick().ok());
    const SignalBus& bus = scheduler.bus();
    EXPECT_EQ(bus.read(bus.find("sensor/labels/count")), 0.0);
}

TEST(Detections, RefusesAMissingCamera) {
    const Model model = scene({{"camera", "rear_camera"}});
    Scheduler scheduler{SchedulerOptions{}};
    for (auto& m : createSensorSubstitutes(model)) {
        scheduler.add(std::move(m));
    }
    EXPECT_FALSE(scheduler.wire(WorldView(model.world)));
}

TEST(Detections, OnAFisheyeCameraTheBoxesComeFromItsPixelsOnly) {
    // A 200° fisheye: the occlusion renders and the truncation projected through a pinhole of
    // the same horizontal_fov (tan of more than 90°: a negative focal length). They are NaN; the
    // 2D boxes, pixels, classes and 3D boxes come from the instance image and the scene.
    Model model = scene();
    model.world.sensors.settings[0]["projection"] = 1.0;
    model.world.sensors.settings[0]["horizontal_fov"] = 3.5;
    std::string error;
    const char* gpu = std::getenv("LAGRANGE_TEST_GPU");
    const std::shared_ptr<CameraScene> cameras =
        createCameraScene(model, gpu != nullptr ? gpu : "", error);
    if (!cameras) {
        GTEST_SKIP() << "no Vulkan device: " << error;
    }
    Scheduler scheduler{SchedulerOptions{}};
    for (auto& m : createSensorModules(model, nullptr, cameras)) {
        scheduler.add(std::move(m));
    }
    auto wired = scheduler.wire(WorldView(model.world));
    ASSERT_TRUE(wired) << to_string(wired.error().front());
    ASSERT_TRUE(scheduler.stepTick().ok());
    const SignalBus& bus = scheduler.bus();
    ASSERT_GE(bus.read(bus.find("sensor/labels/count")), 1.0);
    const std::span<const double> crate = bus.readN(bus.find("sensor/labels/detections"));
    EXPECT_EQ(crate[1], 8.0);
    EXPECT_LT(crate[2], crate[4]);  // a box of its pixels
    EXPECT_GT(crate[6], 0.0);
    EXPECT_TRUE(std::isnan(crate[7])) << crate[7];
    EXPECT_TRUE(std::isnan(crate[8])) << crate[8];
    EXPECT_NEAR(crate[11], 3.0, 1e-6);
}

}  // namespace
}  // namespace lagrange::sensors
