// Fisheye cameras (sensors.md, "Fisheye cameras (as built)"): the Kannala–Brandt projection
// against its closed form and its inverse, the cube-face remap on synthetic renders, and a
// 240° fisheye of a known scene drawn on whatever Vulkan device there is (llvmpipe in CI).
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/sensors/Fisheye.hpp"
#include "lagrange/sensors/Sensors.hpp"
#include "lagrange/world/FisheyeLens.hpp"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <numbers>
#include <random>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::sensors {
namespace {

using Vec3 = std::array<double, 3>;
constexpr double kPi = std::numbers::pi;

double angleBetween(const Vec3& a, const Vec3& b) {
    const Vec3 c{a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    return std::atan2(std::hypot(c[0], c[1], c[2]), a[0] * b[0] + a[1] * b[1] + a[2] * b[2]);
}

TEST(FisheyeLens, TheIdealEquidistantLensPutsAnglesAtProportionalRadii) {
    FisheyeLens lens;
    lens.cutoff = kPi;
    // r = f θ: a ray 100° off the axis, to the right, lands 1.745 focal lengths right.
    const double theta = 100.0 * kPi / 180.0;
    const auto p = lens.project({std::sin(theta), 0.0, std::cos(theta)});
    ASSERT_TRUE(p);
    EXPECT_NEAR((*p)[0], theta, 1e-15);
    EXPECT_NEAR((*p)[1], 0.0, 1e-15);
    // The focal length that spans 200° across 1000 pixels.
    EXPECT_NEAR(lens.focal(1000.0, 2.0 * theta), 500.0 / theta, 1e-12);
    // Settings: horizontal_fov 200°, so the cutoff is 100° unless given.
    const FisheyeLens fromSettings = FisheyeLens::from(
        {{"projection", 1.0}, {"horizontal_fov", 2.0 * theta}, {"distortion_k2", 0.01}});
    EXPECT_TRUE(FisheyeLens::isFisheye({{"projection", 1.0}}));
    EXPECT_FALSE(FisheyeLens::isFisheye({{"distortion_model", 2.0}}));
    EXPECT_NEAR(fromSettings.cutoff, theta, 1e-15);
    EXPECT_EQ(fromSettings.k[1], 0.01);
    EXPECT_NEAR(FisheyeLens::from({{"horizontal_fov", 1.0}}).cutoff, kPi / 2.0, 1e-15);
}

TEST(FisheyeLens, ProjectAndUnprojectRoundTripToAMicroradian) {
    // The ideal lens, and k1..k4 of the order OpenCV's fisheye calibration gives real lenses.
    std::vector<FisheyeLens> lenses(3);
    lenses[0].cutoff = 110.0 * kPi / 180.0;
    lenses[1].k = {-0.013, 0.0042, -0.0011, 0.0001};
    lenses[1].cutoff = 100.0 * kPi / 180.0;
    lenses[2].k = {0.05, -0.01, 0.002, 0.0};
    lenses[2].cutoff = 150.0 * kPi / 180.0;
    std::mt19937_64 random(20261111);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    for (const FisheyeLens& lens : lenses) {
        ASSERT_TRUE(lens.monotonic());
        for (int n = 0; n < 20000; ++n) {
            const double theta = lens.cutoff * unit(random);
            const double phi = 2.0 * kPi * unit(random);
            const Vec3 d{std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi),
                         std::cos(theta)};
            const auto p = lens.project(d);
            ASSERT_TRUE(p);
            const auto back = lens.unproject((*p)[0], (*p)[1]);
            ASSERT_TRUE(back) << theta;
            EXPECT_LT(angleBetween(d, *back), 1e-6) << "θ = " << theta;
            // And the image point comes back too.
            const auto again = lens.project(*back);
            ASSERT_TRUE(again);
            EXPECT_NEAR((*again)[0], (*p)[0], 1e-9);
            EXPECT_NEAR((*again)[1], (*p)[1], 1e-9);
        }
        // Past the cutoff the lens sees nothing.
        const double past = lens.cutoff + 0.01;
        EXPECT_FALSE(lens.project({std::sin(past), 0.0, std::cos(past)}));
        EXPECT_FALSE(lens.unproject(lens.distorted(past), 0.0));
    }
}

TEST(FisheyeLens, ALensThatFoldsBackIsNotMonotonic) {
    FisheyeLens lens;
    lens.k = {-0.5, 0.0, 0.0, 0.0};  // θ_d = θ − 0.5 θ³ turns back at θ = 0.816
    lens.cutoff = kPi / 2.0;
    EXPECT_FALSE(lens.monotonic());
}

FisheyeConfig config(std::uint32_t size, double fov, double cutoff) {
    FisheyeConfig c;
    c.width = size;
    c.height = size;
    c.lens.cutoff = cutoff;
    c.fx = c.lens.focal(size, fov);
    c.fy = c.fx;
    c.cx = (size - 1.0) / 2.0;
    c.cy = (size - 1.0) / 2.0;
    return c;
}

TEST(FisheyeRemap, RendersOnlyTheFacesItsPixelsLookThrough) {
    EXPECT_EQ(FisheyeRemap(config(64, 1.0, 0.6)).faces().size(), 1U);                    // front
    EXPECT_EQ(FisheyeRemap(config(64, 200.0 * kPi / 180.0, 1.745)).faces().size(), 5U);  // no back
    EXPECT_EQ(FisheyeRemap(config(64, 300.0 * kPi / 180.0, 2.618)).faces().size(), 6U);
    // A face's render reaches no further than its pixels need: a 60° lens's front face is a
    // window about 60° wide, at the output's focal length.
    const FisheyeRemap narrow(config(64, kPi / 3.0, kPi / 6.0));
    const gpu::PinholeCamera& front = narrow.faces().front().camera;
    EXPECT_NEAR(front.fx, 64.0 / 2.0 / (kPi / 6.0), 1e-9);
    EXPECT_LE(front.width, static_cast<std::uint32_t>(2.0 * front.fx * std::tan(kPi / 6.0)) + 4);
}

TEST(FisheyeRemap, EachPixelReadsTheFaceItsRayCrossesAndItsRange) {
    // Synthetic renders: every face at depth 1 along its own axis, its index as instance id, its
    // colour a grey of its own. The remap must pick, for each pixel, the face its ray crosses,
    // and turn the face's depth into the range along the ray, 1 / cos(ray, face axis).
    const FisheyeConfig c = config(96, 220.0 * kPi / 180.0, 110.0 * kPi / 180.0);
    const FisheyeRemap remap(c);
    std::vector<gpu::RasterImages> faces(remap.faces().size());
    for (std::size_t f = 0; f < faces.size(); ++f) {
        const auto& cam = remap.faces()[f].camera;
        const std::size_t n = std::size_t{cam.width} * cam.height;
        faces[f].depth.assign(n, 1.0F);
        faces[f].instance.assign(n, static_cast<std::uint32_t>(f));
        faces[f].rgba.assign(n * 4, static_cast<std::uint8_t>(40 * f + 20));
    }
    gpu::RasterImages out;
    remap.apply(faces, out);
    ASSERT_EQ(out.depth.size(), 96U * 96U);
    std::size_t seen = 0;
    for (std::uint32_t v = 0; v < 96; ++v) {
        for (std::uint32_t u = 0; u < 96; ++u) {
            const std::size_t i = std::size_t{v} * 96 + u;
            const auto ray = c.lens.unproject((u - c.cx) / c.fx, (v - c.cy) / c.fy);
            if (!ray) {
                EXPECT_EQ(out.instance[i], gpu::kNoInstance);
                EXPECT_TRUE(std::isinf(out.depth[i]));
                EXPECT_EQ(out.rgba[i * 4], 0);
                continue;
            }
            ++seen;
            ASSERT_LT(out.instance[i], faces.size());
            // The face's axis in the optical frame: its rotation applied to z.
            const auto& q = remap.faces()[out.instance[i]].rotation;
            const Vec3 axis{2.0 * (q[1] * q[3] + q[0] * q[2]), 2.0 * (q[2] * q[3] - q[0] * q[1]),
                            1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])};
            const double cosine = (*ray)[0] * axis[0] + (*ray)[1] * axis[1] + (*ray)[2] * axis[2];
            // It is the face most nearly along the ray (a cube's faces are 90° apart).
            EXPECT_GE(cosine, 1.0 / std::sqrt(3.0) - 1e-6);
            EXPECT_NEAR(out.depth[i], 1.0 / cosine, 1e-5);
            EXPECT_EQ(out.rgba[i * 4], 40 * out.instance[i] + 20);
        }
    }
    // A 220° lens on a square image: the circle of 110° inside the frame, corners dark.
    EXPECT_GT(seen, 96U * 96U * 3 / 4);
    EXPECT_LT(seen, 96U * 96U);
}

// --- drawn on a device ----------------------------------------------------------------------

template <typename T>
T pixel(const Buffer& buffer, std::uint32_t u, std::uint32_t v) {
    T out{};
    std::memcpy(&out, buffer.bytes.data() + (std::size_t{v} * buffer.width + u) * sizeof(T),
                sizeof(T));
    return out;
}

TEST(FisheyeCamera, A240DegreeLensSeesBesideAndBehindItself) {
    // A camera 1 m up looking along +x; a 1 m cube ahead (face at 2.5 m), a 0.6 m cube straight
    // to the left (+y, 90° off the axis, face at 2.7 m) and one behind-left, 112° off the axis,
    // which no pinhole could see.
    constexpr std::uint32_t kSize = 256;
    const double fov = 240.0 * kPi / 180.0;
    Model model;
    World& w = model.world;
    const BodyId post = w.addBody("post", kWorldBody, Pose{}, 0.0, {}, {});
    w.addSensor("front_camera", SensorType::Camera, post, {{0, 0, 1}}, 30.0,
                {{"width", kSize},
                 {"height", kSize},
                 {"horizontal_fov", fov},
                 {"projection", 1.0},
                 {"near_clip", 0.05}});
    model.scene.meshes.push_back(boxMesh(1.0, 1.0, 1.0));
    model.scene.materials.push_back({"grey", {0.6F, 0.6F, 0.6F, 1.0F}});
    const auto add = [&](const char* name, Vec3 at, float scale) {
        ScenePrim prim;
        prim.name = name;
        prim.mesh = 0;
        prim.material = 0;
        prim.offset = {at};
        prim.scale = {scale, scale, scale};
        model.scene.prims.push_back(prim);
    };
    add("ahead", {3, 0, 1}, 1.0F);
    add("left", {0, 3, 1}, 0.6F);
    add("behind_left", {-1.2, 3, 1}, 0.6F);

    std::string error;
    const char* gpu = std::getenv("LAGRANGE_TEST_GPU");
    const std::shared_ptr<CameraScene> cameras =
        createCameraScene(model, gpu != nullptr ? gpu : "", error);
    if (!cameras) {
        GTEST_SKIP() << "no Vulkan device: " << error;
    }
    Scheduler scheduler{SchedulerOptions{}};
    for (auto& module : createSensorModules(model.world, nullptr, cameras)) {
        scheduler.add(std::move(module));
    }
    auto wired = scheduler.wire(WorldView(model.world));
    ASSERT_TRUE(wired) << to_string(wired.error().front());
    ASSERT_TRUE(scheduler.stepTick().ok());
    const SignalBus& bus = scheduler.bus();
    const auto depth = bus.buffer(bus.readBuffer(bus.find("sensor/front_camera/depth")));
    const auto instance = bus.buffer(bus.readBuffer(bus.find("sensor/front_camera/instance")));
    const auto image = bus.buffer(bus.readBuffer(bus.find("sensor/front_camera/image")));
    ASSERT_TRUE(depth && instance && image);
    ASSERT_EQ(depth->width, kSize);
    ASSERT_EQ(instance->bytes.size(), std::size_t{kSize} * kSize * 4);

    // Where a direction (sensor frame: x ahead, y left, z up) lands: r = f θ in the optical frame
    // (x right, y down, z ahead).
    const double f = kSize / 2.0 / (fov / 2.0);
    const double c = (kSize - 1.0) / 2.0;
    const auto at = [&](const Vec3& d) {
        const Vec3 o{-d[1], -d[2], d[0]};
        const double r = std::hypot(o[0], o[1]);
        const double theta = std::atan2(r, o[2]);
        return std::array<std::uint32_t, 2>{
            static_cast<std::uint32_t>(std::lround(c + f * theta * o[0] / r)),
            static_cast<std::uint32_t>(std::lround(c + f * theta * o[1] / r))};
    };
    const auto centre = std::array<std::uint32_t, 2>{kSize / 2, kSize / 2};
    EXPECT_EQ(pixel<std::uint32_t>(*instance, centre[0], centre[1]), 0U);
    EXPECT_NEAR(pixel<float>(*depth, centre[0], centre[1]), 2.5F, 0.02F);

    const auto left = at({0.0, 1.0, 0.0});
    EXPECT_EQ(pixel<std::uint32_t>(*instance, left[0], left[1]), 1U);
    EXPECT_NEAR(pixel<float>(*depth, left[0], left[1]), 2.7F, 0.03F);  // range along the ray

    const auto behind = at({-1.2, 3.0, 0.0});
    EXPECT_EQ(pixel<std::uint32_t>(*instance, behind[0], behind[1]), 2U);
    EXPECT_NEAR(pixel<float>(*depth, behind[0], behind[1]), std::hypot(1.08, 2.7), 0.03F);

    // Straight up is open sky; a corner of the square image is past the 120° cutoff: dark.
    const auto up = at({0.0, 0.0, 1.0});
    EXPECT_EQ(pixel<std::uint32_t>(*instance, up[0], up[1]), gpu::kNoInstance);
    EXPECT_EQ(pixel<std::uint32_t>(*instance, 0, 0), gpu::kNoInstance);
    EXPECT_TRUE(std::isinf(pixel<float>(*depth, 0, 0)));
    EXPECT_EQ(std::to_integer<int>(image->bytes[0]), 0);
}

TEST(FisheyeCamera, RefusesTheSensorModelPartsItDoesNotCombineYet) {
    Model model;
    World& w = model.world;
    const BodyId post = w.addBody("post", kWorldBody, Pose{}, 0.0, {}, {});
    w.addSensor("cam", SensorType::Camera, post, {}, 30.0,
                {{"width", 64},
                 {"height", 64},
                 {"horizontal_fov", 3.5},
                 {"projection", 1.0},
                 {"vignetting", 0.5}});
    Scheduler scheduler;
    for (auto& module : createSensorSubstitutes(model.world)) {
        scheduler.add(std::move(module));
    }
    auto wired = scheduler.wire(WorldView(model.world));
    ASSERT_FALSE(wired);
    EXPECT_NE(to_string(wired.error().front()).find("fisheye"), std::string::npos);
}

}  // namespace
}  // namespace lagrange::sensors
