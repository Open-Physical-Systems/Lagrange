// stereo_camera (sensors.md §9): the depth error has σ_z = z² σ_d / (f b) (Intel's D400
// tuning paper), the left band the right imager does not see is invalid, as are occlusion
// shadows, points nearer than min-Z = f b / max disparity (−inf) and beyond range (+inf); then a
// rendered run (llvmpipe without a GPU, skipped without any Vulkan device).
#include "CameraKinds.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/sensors/Kinds.hpp"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <memory>

#include <gtest/gtest.h>

namespace lagrange::sensors {
namespace {

using kinds::StereoConfig;

constexpr std::uint32_t kW = 200;
constexpr std::uint32_t kH = 200;

StereoConfig config() {
    StereoConfig c;
    c.fx = 400.0;
    c.cx = (kW - 1) / 2.0;
    c.cy = (kH - 1) / 2.0;
    c.baseline = 0.05;
    c.subpixel = 0.08;
    c.maxDisparity = 126.0;
    c.maxRange = 10.0;
    c.depthUnit = 0.0;
    return c;
}

TEST(StereoCamera, TheDepthErrorIsZSquaredSigmaDOverFB) {
    // D435 at 848 px and 87°: f ≈ 446.6 px, b = 50 mm, σ_d = 0.08 px: 3.6 mm at 1 m, 14 mm at 2 m.
    EXPECT_NEAR(kinds::stereoDepthSigma(1.0, 0.08, 446.6, 0.05), 0.08 / (446.6 * 0.05), 1e-12);
    EXPECT_NEAR(kinds::stereoDepthSigma(2.0, 0.08, 446.6, 0.05) /
                    kinds::stereoDepthSigma(1.0, 0.08, 446.6, 0.05),
                4.0, 1e-12);
    // A flat wall at z: the noise of the valid pixels has that σ.
    for (const double z : {1.0, 2.0, 4.0}) {
        const std::vector<float> wall(std::size_t{kW} * kH, static_cast<float>(z));
        std::vector<float> depth;
        kinds::stereoDepth(config(), kW, kH, wall, wall, true, 7, 0, depth);
        double sum = 0.0, square = 0.0;
        int n = 0;
        for (const float d : depth) {
            if (std::isfinite(d)) {
                sum += d - z;
                square += (d - z) * (d - z);
                ++n;
            }
        }
        ASSERT_GT(n, 10000);
        const double sigma = std::sqrt(square / n - (sum / n) * (sum / n));
        const double expected = z * z * 0.08 / (400.0 * 0.05);
        EXPECT_NEAR(sigma, expected, 0.03 * expected) << z;
        EXPECT_NEAR(sum / n, 0.0, 4.0 * expected / std::sqrt(n)) << z;
    }
}

TEST(StereoCamera, TheLeftBandIsInvalidAndItsWidthIsTheDisparity) {
    // At 2 m the disparity is f b / z = 10 px: the right imager does not see columns 0..9.
    const std::vector<float> wall(std::size_t{kW} * kH, 2.0F);
    std::vector<float> depth;
    kinds::stereoDepth(config(), kW, kH, wall, wall, false, 7, 0, depth);
    for (std::uint32_t u = 0; u < 20; ++u) {
        const float d = depth[100 * kW + u];
        if (u < 10) {
            EXPECT_TRUE(std::isnan(d)) << u;
        } else {
            EXPECT_EQ(d, 2.0F) << u;
        }
    }
}

TEST(StereoCamera, RangeLimitsAndOcclusionsFollowRep118) {
    StereoConfig c = config();
    c.depthUnit = 0.001;
    // min-Z = 400 × 0.05 / 126 = 0.159 m.
    std::vector<float> left(std::size_t{kW} * kH, 0.1F);
    std::vector<float> depth;
    kinds::stereoDepth(c, kW, kH, left, left, false, 7, 0, depth);
    EXPECT_EQ(depth[100 * kW + 150], -std::numeric_limits<float>::infinity());
    std::ranges::fill(left, 12.0F);
    kinds::stereoDepth(c, kW, kH, left, left, false, 7, 0, depth);
    EXPECT_EQ(depth[100 * kW + 150], std::numeric_limits<float>::infinity());
    // The right camera sees something nearer where the left pixel's point would be: hidden.
    std::ranges::fill(left, 2.0F);
    std::vector<float> right = left;
    right[100 * kW + 140] = 1.0F;  // the point of left column 150 is at right column 140
    kinds::stereoDepth(c, kW, kH, left, right, true, 7, 0, depth);
    EXPECT_TRUE(std::isnan(depth[100 * kW + 150]));
    EXPECT_TRUE(std::isfinite(depth[100 * kW + 151]));
    // The 1 mm quantum.
    const double q = depth[100 * kW + 151] / 0.001;
    EXPECT_NEAR(q, std::round(q), 1e-3);
}

// --- rendered ------------------------------------------------------------------------------

TEST(StereoCamera, RendersBothImagersAndTheWallAhead) {
    Model model;
    World& w = model.world;
    const BodyId post = w.addBody("post", kWorldBody, Pose{}, 0.0, {}, {});
    w.addCustomSensor("rs", "stereo_camera", post, {{0, 0, 1}}, 15.0,
                      {{"width", 160.0}, {"height", 120.0}}, {});
    // A 4 m wide wall 2 m ahead.
    model.scene.meshes.push_back(boxMesh(0.1, 4.0, 4.0));
    model.scene.materials.push_back({"grey", {0.6F, 0.6F, 0.6F, 1.0F}});
    ScenePrim wall;
    wall.name = "wall";
    wall.mesh = 0;
    wall.material = 0;
    wall.offset = {{2.05, 0, 1}};
    model.scene.prims.push_back(wall);
    std::string error;
    const char* gpu = std::getenv("LAGRANGE_TEST_GPU");
    const std::shared_ptr<CameraScene> scene =
        createCameraScene(model, gpu != nullptr ? gpu : "", error);
    if (!scene) {
        GTEST_SKIP() << "no Vulkan device: " << error;
    }
    Scheduler scheduler;
    for (auto& module : createSensorModules(w, nullptr, scene)) {
        scheduler.add(std::move(module));
    }
    auto wired = scheduler.wire(WorldView(w));
    ASSERT_TRUE(wired) << to_string(wired.error().front());
    ASSERT_TRUE(scheduler.stepTick().ok());
    const SignalBus& bus = scheduler.bus();
    const auto depth = bus.buffer(bus.readBuffer(bus.find("sensor/rs/depth")));
    const auto left = bus.buffer(bus.readBuffer(bus.find("sensor/rs/left")));
    const auto right = bus.buffer(bus.readBuffer(bus.find("sensor/rs/right")));
    ASSERT_TRUE(depth && left && right);
    EXPECT_EQ(depth->encoding, "r32f");
    ASSERT_EQ(depth->bytes.size(), 160U * 120U * 4U);
    EXPECT_EQ(left->bytes.size(), 160U * 120U * 4U);
    EXPECT_EQ(right->bytes.size(), 160U * 120U * 4U);
    const auto at = [&](std::uint32_t u, std::uint32_t v) {
        float d = 0.0F;
        std::memcpy(&d, depth->bytes.data() + (std::size_t{v} * 160 + u) * 4, 4);
        return d;
    };
    // f = 80 / tan(43.5°) = 84.2 px: σ_z at 2 m = 4 × 0.08 / (84.2 × 0.05) = 76 mm.
    const double f = 80.0 / std::tan(87.0 * 3.14159265358979323846 / 360.0);
    const double sigma = 4.0 * 0.08 / (f * 0.05);
    EXPECT_NEAR(at(80, 60), 2.0, 5.0 * sigma);
    // The left band: disparity f b / 2 = 2.1 px, so column 0 is invalid.
    EXPECT_TRUE(std::isnan(at(0, 60)));
}

TEST(StereoCamera, GroundTruthPublishesEmptyImagesThatKeepTheirSize) {
    World w;
    const BodyId post = w.addBody("post", kWorldBody, Pose{}, 0.0, {}, {});
    w.addCustomSensor("rs", "stereo_camera", post, Pose{}, 15.0, {}, {});
    Scheduler scheduler;
    for (auto& module : createSensorSubstitutes(w)) {
        scheduler.add(std::move(module));
    }
    ASSERT_TRUE(scheduler.wire(WorldView(w)));
    ASSERT_TRUE(scheduler.stepTick().ok());
    const SignalBus& bus = scheduler.bus();
    const auto depth = bus.buffer(bus.readBuffer(bus.find("sensor/rs/depth")));
    ASSERT_TRUE(depth);
    EXPECT_EQ(depth->width, 848U);
    EXPECT_EQ(depth->height, 480U);
    EXPECT_TRUE(depth->bytes.empty());
}

}  // namespace
}  // namespace lagrange::sensors
