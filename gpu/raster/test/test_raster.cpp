// The camera sensor's projection (docs/design/domains/sensors.md §7): checkerboards rendered at
// known poses are found with OpenCV; the corners lie where the pinhole model puts them, and a
// calibration from the images recovers the intrinsics. Reprojection error < 0.1 px.
#include "lagrange/gpu/Raster.hpp"
#include "lagrange/gpu/Realistic.hpp"
#include "lagrange/gpu/TreeShapes.hpp"
#include "lagrange/gpu/WaterOptics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <numbers>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::gpu {
namespace {

constexpr int kColumns = 10;            // squares
constexpr int kRows = 7;                //
constexpr double kSquareM = 0.03;       // m
constexpr std::uint32_t kWidth = 1280;  // px
constexpr std::uint32_t kHeight = 960;
constexpr double kFocalPx = 900.0;

struct Board {
    std::vector<float> positions[2];  // dark, light squares
    std::vector<float> normals[2];
    std::vector<std::uint32_t> indices[2];
};

/// Squares in the board's xy plane, corner (0, 0) at the origin.
Board board() {
    Board b;
    for (int row = 0; row < kRows; ++row) {
        for (int column = 0; column < kColumns; ++column) {
            const int k = (row + column) % 2;
            const auto base = static_cast<std::uint32_t>(b.positions[k].size() / 3);
            const auto x0 = static_cast<float>(column * kSquareM);
            const auto y0 = static_cast<float>(row * kSquareM);
            const auto s = static_cast<float>(kSquareM);
            b.positions[k].insert(b.positions[k].end(),
                                  {x0, y0, 0, x0 + s, y0, 0, x0 + s, y0 + s, 0, x0, y0 + s, 0});
            for (int v = 0; v < 4; ++v) {
                b.normals[k].insert(b.normals[k].end(),
                                    {0, 0, 1});  // lit: the shader ignores facing
            }
            b.indices[k].insert(b.indices[k].end(),
                                {base, base + 1, base + 2, base, base + 2, base + 3});
        }
    }
    return b;
}

std::array<double, 4> quaternion(double roll, double pitch, double yaw) {
    const double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
    const double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
    const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
    return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy};
}

std::array<double, 3> rotate(const std::array<double, 4>& q, const std::array<double, 3>& v) {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    return {
        (1 - 2 * (y * y + z * z)) * v[0] + 2 * (x * y - w * z) * v[1] + 2 * (x * z + w * y) * v[2],
        2 * (x * y + w * z) * v[0] + (1 - 2 * (x * x + z * z)) * v[1] + 2 * (y * z - w * x) * v[2],
        2 * (x * z - w * y) * v[0] + 2 * (y * z + w * x) * v[1] + (1 - 2 * (x * x + y * y)) * v[2]};
}

class Raster : public ::testing::Test {
protected:
    void SetUp() override {
        b_ = board();
        std::vector<MeshSource> meshes;
        for (int k = 0; k < 2; ++k) {
            meshes.push_back({b_.positions[k], b_.normals[k], b_.indices[k]});
        }
        std::string error;
        const char* gpu = std::getenv("LAGRANGE_TEST_GPU");
        scene_ = RasterScene::create(gpu != nullptr ? gpu : "", meshes, error);
        if (!scene_) {
            GTEST_SKIP() << "no Vulkan device: " << error;
        }
        camera_.width = kWidth;
        camera_.height = kHeight;
        camera_.fx = kFocalPx;
        camera_.fy = kFocalPx;
        camera_.cx = (kWidth - 1) / 2.0;
        camera_.cy = (kHeight - 1) / 2.0;
        // At the world origin, looking along +z: the optical frame is the world frame.
    }

    /// The board at `position` (m) and `rotation`, drawn by `camera`.
    RasterImages shoot(RasterCamera& camera, const std::array<double, 3>& position,
                       const std::array<double, 4>& rotation) {
        std::vector<RasterInstance> instances(2);
        instances[0] = {0, position, rotation, {1, 1, 1}, {0.02F, 0.02F, 0.02F, 1}, 7};
        instances[1] = {1, position, rotation, {1, 1, 1}, {0.95F, 0.95F, 0.95F, 1}, 8};
        RasterImages images;
        camera.render(instances, camera_, images);
        return images;
    }

    Board b_;
    std::shared_ptr<RasterScene> scene_;
    PinholeCamera camera_;
};

TEST_F(Raster, CheckerboardCornersLieWhereThePinholeModelPutsThem) {
    RasterCamera camera(scene_, kWidth, kHeight);
    const cv::Size pattern(kColumns - 1, kRows - 1);
    std::vector<std::vector<cv::Point3f>> objects;
    std::vector<std::vector<cv::Point2f>> found;
    double worst = 0.0;
    double sum = 0.0;
    std::size_t count = 0;
    const std::array<std::array<double, 6>, 8> poses = {{
        // x, y, z (m), roll, pitch, yaw (rad)
        {-0.13, -0.09, 0.55, 0.0, 0.0, 0.0},
        {-0.10, -0.06, 0.60, 0.35, 0.0, 0.1},
        {-0.16, -0.10, 0.65, -0.3, 0.25, 0.0},
        {-0.05, -0.12, 0.50, 0.2, -0.35, 0.3},
        {-0.18, -0.02, 0.70, -0.4, -0.2, -0.2},
        {-0.12, -0.08, 0.45, 0.1, 0.45, 0.05},
        {-0.08, -0.14, 0.62, 0.45, 0.3, -0.4},
        {-0.20, -0.05, 0.58, -0.25, -0.4, 0.5},
    }};
    for (const auto& p : poses) {
        const std::array<double, 3> position{p[0], p[1], p[2]};
        const std::array<double, 4> rotation = quaternion(p[3], p[4], p[5]);
        const RasterImages images = shoot(camera, position, rotation);
        cv::Mat rgba(kHeight, kWidth, CV_8UC4, const_cast<std::uint8_t*>(images.rgba.data()));
        cv::Mat gray;
        cv::cvtColor(rgba, gray, cv::COLOR_RGBA2GRAY);
        if (const char* dump = std::getenv("LAGRANGE_TEST_DUMP")) {
            (void)writePng(std::string(dump) + "/board_" + std::to_string(objects.size()) + ".png",
                           kWidth, kHeight, images.rgba);
        }
        std::vector<cv::Point2f> corners;
        ASSERT_TRUE(cv::findChessboardCorners(gray, pattern, corners, cv::CALIB_CB_ADAPTIVE_THRESH))
            << "pose " << p[0] << ", " << p[1];
        cv::cornerSubPix(
            gray, corners, cv::Size(7, 7), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 100, 1e-4));
        // The inner corners in the board frame and their pinhole projections; OpenCV may list
        // them in either direction, so compare with the closer ordering.
        std::vector<cv::Point3f> object;
        std::vector<cv::Point2f> expected;
        for (int row = 1; row < kRows; ++row) {
            for (int column = 1; column < kColumns; ++column) {
                const std::array<double, 3> local{column * kSquareM, row * kSquareM, 0.0};
                const std::array<double, 3> r = rotate(rotation, local);
                const double x = position[0] + r[0];
                const double y = position[1] + r[1];
                const double z = position[2] + r[2];
                object.emplace_back(static_cast<float>(local[0]), static_cast<float>(local[1]),
                                    0.0F);
                expected.emplace_back(static_cast<float>(kFocalPx * x / z + camera_.cx),
                                      static_cast<float>(kFocalPx * y / z + camera_.cy));
            }
        }
        const auto error = [&](bool reversed) {
            double total = 0.0;
            for (std::size_t i = 0; i < corners.size(); ++i) {
                const cv::Point2f& e = expected[reversed ? corners.size() - 1 - i : i];
                total += std::hypot(corners[i].x - e.x, corners[i].y - e.y);
            }
            return total;
        };
        const bool reversed = error(true) < error(false);
        if (reversed) {
            std::reverse(corners.begin(), corners.end());
        }
        for (std::size_t i = 0; i < corners.size(); ++i) {
            const double e = std::hypot(corners[i].x - expected[i].x, corners[i].y - expected[i].y);
            worst = std::max(worst, e);
            sum += e;
            ++count;
        }
        objects.push_back(object);
        found.push_back(corners);
    }
    const double mean = sum / static_cast<double>(count);
    RecordProperty("samples_per_pixel", std::to_string(scene_->samples()));
    RecordProperty("mean_corner_error_px", std::to_string(mean));
    RecordProperty("worst_corner_error_px", std::to_string(worst));
    EXPECT_LT(mean, 0.1);

    // Calibrate from the images alone: the intrinsics come back.
    cv::Mat k, distortion;
    std::vector<cv::Mat> rvecs, tvecs;
    const double rms = cv::calibrateCamera(
        objects, found, cv::Size(kWidth, kHeight), k, distortion, rvecs, tvecs,
        cv::CALIB_FIX_K1 | cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3 | cv::CALIB_ZERO_TANGENT_DIST);
    RecordProperty("calibration_rms_px", std::to_string(rms));
    EXPECT_LT(rms, 0.1);
    EXPECT_NEAR(k.at<double>(0, 0), kFocalPx, kFocalPx * 0.005);
    EXPECT_NEAR(k.at<double>(1, 1), kFocalPx, kFocalPx * 0.005);
    EXPECT_NEAR(k.at<double>(0, 2), camera_.cx, 1.0);
    EXPECT_NEAR(k.at<double>(1, 2), camera_.cy, 1.0);
}

TEST_F(Raster, DepthIsAlongTheOpticalAxisAndInstancesAreLabelled) {
    RasterCamera camera(scene_, kWidth, kHeight);
    // The board square-on at 0.6 m, centred on the optical axis.
    const RasterImages images = shoot(camera, {-0.15, -0.1, 0.6}, {1, 0, 0, 0});
    const std::size_t centre = std::size_t{kHeight / 2} * kWidth + kWidth / 2;
    EXPECT_NEAR(images.depth[centre], 0.6, 1e-5);
    // A pixel near the board's corner: still 0.6 m along the axis, not the ray length.
    const std::size_t off = std::size_t{350} * kWidth + 440;
    EXPECT_NEAR(images.depth[off], 0.6, 1e-5);
    EXPECT_TRUE(images.instance[off] == 7 || images.instance[off] == 8);
    // Nothing behind the board's edge: no depth, no instance, the sky colour.
    EXPECT_TRUE(std::isinf(images.depth[0]));
    EXPECT_EQ(images.instance[0], kNoInstance);
    EXPECT_GT(images.rgba[2], images.rgba[0]);  // bluish
}

TEST_F(Raster, LargeWorldCoordinatesRenderAsAtTheOrigin) {
    RasterCamera camera(scene_, kWidth, kHeight);
    const RasterImages near = shoot(camera, {-0.13, -0.09, 0.55}, quaternion(0.2, 0.1, 0.05));
    // Camera and board 500 km away (invariant 6: only relative coordinates reach the GPU).
    camera_.position = {500'000.0, -300'000.0, 1'000.0};
    const RasterImages far =
        shoot(camera, {500'000.0 - 0.13, -300'000.0 - 0.09, 1'000.55}, quaternion(0.2, 0.1, 0.05));
    std::size_t differing = 0;
    for (std::size_t i = 0; i < near.instance.size(); ++i) {
        differing += near.instance[i] != far.instance[i] ? 1 : 0;
    }
    EXPECT_LT(differing, near.instance.size() / 10'000);  // edge pixels at most
}

TEST_F(Raster, FramesInFlightGiveWhatRenderGives) {
    // Two cameras each begin a frame, then both finish (the scheduler's split steps): the images
    // are the ones render() draws, byte for byte.
    RasterCamera one(scene_, kWidth, kHeight);
    RasterCamera two(scene_, kWidth, kHeight);
    const std::array<double, 3> a{-0.13, -0.09, 0.55};
    const std::array<double, 3> b{-0.15, -0.1, 0.6};
    const RasterImages expectedA = shoot(one, a, quaternion(0.2, 0.1, 0.05));
    const RasterImages expectedB = shoot(two, b, {1, 0, 0, 0});
    std::vector<RasterInstance> instancesA(2);
    instancesA[0] = {0, a, quaternion(0.2, 0.1, 0.05), {1, 1, 1}, {0.02F, 0.02F, 0.02F, 1}, 7};
    instancesA[1] = {1, a, quaternion(0.2, 0.1, 0.05), {1, 1, 1}, {0.95F, 0.95F, 0.95F, 1}, 8};
    std::vector<RasterInstance> instancesB(2);
    instancesB[0] = {0, b, {1, 0, 0, 0}, {1, 1, 1}, {0.02F, 0.02F, 0.02F, 1}, 7};
    instancesB[1] = {1, b, {1, 0, 0, 0}, {1, 1, 1}, {0.95F, 0.95F, 0.95F, 1}, 8};
    one.begin(instancesA, camera_, nullptr);
    two.begin(instancesB, camera_, nullptr);
    EXPECT_TRUE(one.pending());
    EXPECT_TRUE(two.pending());
    RasterImages gotB;
    RasterImages gotA;
    two.finish(gotB);
    one.finish(gotA);
    EXPECT_FALSE(one.pending());
    EXPECT_EQ(gotA.rgba, expectedA.rgba);
    EXPECT_EQ(gotA.depth, expectedA.depth);
    EXPECT_EQ(gotA.instance, expectedA.instance);
    EXPECT_EQ(gotB.rgba, expectedB.rgba);
    EXPECT_EQ(gotB.depth, expectedB.depth);
    EXPECT_EQ(gotB.instance, expectedB.instance);
    // A frame begun and never finished is waited for when the camera goes.
    one.begin(instancesA, camera_, nullptr);
}

// The realistic look changes colour only: depth and instance ids are the same, sky and ground are
// drawn, and with no GPU the camera keeps its simple shading (sensors.md).
TEST(RasterLook, RealisticChangesColourOnly) {
    const std::vector<float> positions{-50, -50, 0, 50, -50, 0, 50, 50, 0, -50, 50, 0};
    const std::vector<float> normals{0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1};
    const std::vector<std::uint32_t> indices{0, 1, 2, 0, 2, 3};
    const std::vector<MeshSource> meshes{{positions, normals, indices}};
    PinholeCamera camera;
    camera.width = 320;
    camera.height = 240;
    camera.fx = camera.fy = 200.0;
    camera.cx = 159.5;
    camera.cy = 119.5;
    camera.position = {0.0, 0.0, 2.0};
    // Looking along +x: optical z = x, optical x = -y, optical y = -z.
    camera.orientation = {0.5, -0.5, 0.5, -0.5};
    RasterInstance floor;
    floor.id = 7;
    floor.pattern = 2.0F;  // asphalt
    floor.roughness = 0.9F;
    const std::vector<RasterInstance> instances{floor};
    const auto render = [&](RasterLook look, bool& realistic, RasterImages& out) {
        std::string error;
        auto scene = RasterScene::create("", meshes, error, look);
        ASSERT_TRUE(scene) << error;
        realistic = scene->realistic();
        RasterCamera renderer(scene, camera.width, camera.height);
        renderer.render(instances, camera, out, 1.0);
    };
    RasterImages simple;
    RasterImages realistic;
    bool isSimple = true;
    bool isRealistic = false;
    render({}, isSimple, simple);
    EXPECT_FALSE(isSimple);
    render({.realistic = true}, isRealistic, realistic);
    if (!isRealistic) {
        GTEST_SKIP() << "a software device keeps the simple shading";
    }
    EXPECT_EQ(simple.instance, realistic.instance);
    EXPECT_EQ(simple.depth, realistic.depth);
    EXPECT_NE(simple.rgba, realistic.rgba);
    const auto pixel = [&](std::uint32_t x, std::uint32_t y) {
        return &realistic.rgba[4 * ((std::size_t{y} * camera.width) + x)];
    };
    const std::uint8_t* sky = pixel(160, 10);
    const std::uint8_t* ground = pixel(160, 220);
    EXPECT_GT(sky[2], sky[0]);                          // blue overhead
    EXPECT_GT(sky[2] - sky[0], ground[2] - ground[0]);  // and bluer than the grey floor
}

// --- trees in the simple look (TreeShapes.hpp) ------------------------------------------------

// The clumps lie in the crown's ellipsoid, as the realistic view's leaves do; a sway moves the
// crown's centre by the sway and the foot not at all; cut cells take a bite out of a clump.
TEST(TreeShapes, ClumpsFillTheCrownSwayAndCut) {
    RealisticTree tree;
    tree.foot = {10.0, -4.0, 0.0};
    tree.height = 6.1;
    tree.crownRadius = 1.8;
    tree.crownBase = 2.5;
    tree.trunkDiameter = 0.5;
    tree.seed = 0x4d2a17;
    const double half = 0.5 * (tree.height - tree.crownBase);
    for (std::uint32_t k = 0; k < kTreeClumpCount; ++k) {
        const std::array<float, 4> c = treeClump(tree, k);
        const double e = (c[0] * c[0] + c[1] * c[1]) / (tree.crownRadius * tree.crownRadius) +
                         (c[2] * c[2]) / (half * half);
        EXPECT_LT(e, 1.0) << k;
        EXPECT_GT(c[3], 0.2F) << k;
    }
    std::vector<RasterInstance> still;
    appendTreeParts(tree, 0, {}, 100, 7, still);
    ASSERT_EQ(still.size(), 1U + kTreeClumpCount);
    EXPECT_EQ(still[0].mesh, 101U);  // the trunk first
    EXPECT_EQ(still[0].position, tree.foot);
    EXPECT_EQ(still[1].mesh, 100U);
    EXPECT_EQ(still[1].id, 7U);
    // Swayed: a clump at the crown centre's height moves by the whole sway.
    RealisticTree swayed = tree;
    swayed.sway = {0.3, 0.0, 0.0};
    std::vector<RasterInstance> moved;
    appendTreeParts(swayed, 0, {}, 100, 7, moved);
    EXPECT_EQ(moved[0].position, tree.foot);
    for (std::size_t k = 1; k < moved.size(); ++k) {
        const double z = moved[k].position[2];
        const double bend = std::pow(z / (tree.crownBase + half), 1.5);
        EXPECT_NEAR(moved[k].position[0] - still[k].position[0], 0.3 * bend, 1e-6) << k;
    }
    // Cut every cell of the first clump: it is gone, the others stay.
    const std::array<float, 4> first = treeClump(tree, 0);
    std::vector<RealisticTreeCut> cuts;
    for (int x = -10; x <= 10; ++x) {
        for (int y = -10; y <= 10; ++y) {
            for (int z = -10; z <= 10; ++z) {
                cuts.push_back({0,
                                {static_cast<std::int32_t>(std::floor(first[0] / 0.1)) + x,
                                 static_cast<std::int32_t>(std::floor(first[1] / 0.1)) + y,
                                 static_cast<std::int32_t>(std::floor(first[2] / 0.1)) + z}});
            }
        }
    }
    std::vector<RasterInstance> cut;
    appendTreeParts(tree, 0, cuts, 100, 7, cut);
    EXPECT_LT(cut.size(), still.size());
    // Another tree's cuts take nothing.
    std::vector<RasterInstance> other;
    appendTreeParts(tree, 1, cuts, 100, 7, other);
    EXPECT_EQ(other.size(), still.size());
}

// --- lamps (RFC 0021 §4) ----------------------------------------------------------------------

// The colour of a colour temperature: Planck's law through the CIE 1931 functions lands on CIE
// illuminant A's chromaticity at its 2856 K (CIE 15:2004 Table T.3 gives x = 0.44757,
// y = 0.40745), within the analytic fit's error.
TEST(RealisticLight, ColourTemperatureOfIlluminantA) {
    const std::array<float, 3> rgb = colorOfTemperature(2856.0);
    // Linear sRGB back to XYZ (IEC 61966-2-1).
    const double x = 0.4124 * rgb[0] + 0.3576 * rgb[1] + 0.1805 * rgb[2];
    const double y = 0.2126 * rgb[0] + 0.7152 * rgb[1] + 0.0722 * rgb[2];
    const double z = 0.0193 * rgb[0] + 0.1192 * rgb[1] + 0.9505 * rgb[2];
    EXPECT_NEAR(y, 1.0, 1e-4);  // luminance 1
    EXPECT_NEAR(x / (x + y + z), 0.44757, 0.003);
    EXPECT_NEAR(y / (x + y + z), 0.40745, 0.003);
    // Warm below the white point, cool above it.
    EXPECT_GT(rgb[0], rgb[2]);
    const std::array<float, 3> cool = colorOfTemperature(10000.0);
    EXPECT_GT(cool[2], cool[0]);
}

// The calibration: the drawn sun stands for the light_sensor's clear-sky beam (checked against
// iesDaylight in modules/sensors), and the adaptation runs from 1 by day to its limit at night.
TEST(RealisticLight, CalibrationAndAdaptation) {
    EXPECT_GT(realisticSunIlluminance(), 80e3);  // a clear afternoon: about 100 klx
    EXPECT_LT(realisticSunIlluminance(), 120e3);
    EXPECT_GT(realisticDaylight(), realisticSunIlluminance() * realisticSunDirection()[2]);
    EXPECT_EQ(realisticAdaptation(std::numeric_limits<double>::quiet_NaN()), 1.0);
    EXPECT_EQ(realisticAdaptation(realisticDaylight()), 1.0);
    EXPECT_NEAR(realisticAdaptation(realisticDaylight() / 10.0), 10.0, 1e-9);
    EXPECT_EQ(realisticAdaptation(0.8e-3), 1024.0);
}

// RFC 0021 §Validation, the renderer's calibration: a grey Lambertian plane under one spot at
// night, its linear pixel values (before tone mapping) against ρ·E/π at 5 points, within 2 %.
// The plane is drawn grey and black: their difference is the diffuse part alone (the specular
// part, the same on both, cancels).
TEST(RealisticLight, ALitPlaneReadsRhoEOverPi) {
    const std::vector<float> positions{-50, -50, 0, 50, -50, 0, 50, 50, 0, -50, 50, 0};
    const std::vector<float> normals{0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1};
    const std::vector<std::uint32_t> indices{0, 1, 2, 0, 2, 3};
    const std::vector<MeshSource> meshes{{positions, normals, indices}};
    std::string error;
    const char* gpu = std::getenv("LAGRANGE_TEST_GPU");
    auto scene = RasterScene::create(gpu != nullptr ? gpu : "", meshes, error, {.realistic = true});
    ASSERT_TRUE(scene) << error;
    if (!scene->realistic()) {
        GTEST_SKIP() << "a software device keeps the simple shading";
    }
    constexpr double kEyeHeight = 3.0;    // m, looking straight down
    constexpr double kLampHeight = 1.5;   // m, beam straight down
    constexpr double kIntensity = 1.2e5;  // cd: about 0.6 of full scale on the axis
    constexpr double kRho = 0.8;
    PinholeCamera camera;
    camera.width = 320;
    camera.height = 320;
    camera.fx = camera.fy = 200.0;
    camera.cx = camera.cy = 159.5;
    camera.position = {0.0, 0.0, kEyeHeight};
    camera.orientation = {0.0, 1.0, 0.0, 0.0};  // optical z down, x along world x, y along −y
    RealisticLamp lamp;
    lamp.position = {0.0, 0.0, kLampHeight};
    lamp.axis = {0.0, 0.0, -1.0};
    lamp.intensity = kIntensity;
    lamp.beamAngle = 1.2;
    lamp.fieldAngle = 2.0;
    const std::vector<RealisticLamp> lamps{lamp};
    RasterCamera renderer(scene, camera.width, camera.height);
    renderer.setLighting(lamps, 0.0, true);  // no sky: the lamp alone
    const auto shoot = [&](float albedo) {
        RasterInstance plane;
        plane.color = {albedo, albedo, albedo, 1.0F};
        plane.roughness = 0.45F;  // no surface grain (surfaceDetail)
        RasterImages out;
        const std::vector<RasterInstance> instances{plane};
        renderer.render(instances, camera, out, 0.0);
        return out;
    };
    const RasterImages grey = shoot(static_cast<float>(kRho));
    const RasterImages black = shoot(0.0F);
    const double k = realisticUnitsPerLux();
    // The taper of lights::taper (RFC 0021 §2).
    const auto taper = [&](double theta) {
        const double beam = lamp.beamAngle / 2.0;
        const double field = lamp.fieldAngle / 2.0;
        if (theta <= beam) {
            const double c = std::cos(0.5 * std::numbers::pi * theta / beam);
            return 0.5 + 0.5 * c * c;
        }
        const double c = std::cos(0.5 * std::numbers::pi * (theta - beam) / (field - beam));
        return 0.1 + 0.4 * c * c;
    };
    std::ofstream log;
    if (const char* dir = std::getenv("LAGRANGE_TEST_OUT")) {
        log.open(std::string(dir) + "/lit_plane.csv");
        log << "theta_rad,lux,expected,measured,ratio\n";
    }
    for (const double theta : {0.0, 0.2, 0.35, 0.5, 0.65}) {  // rad off the beam's axis
        const double x = kLampHeight * std::tan(theta);
        const double d = kLampHeight / std::cos(theta);
        const double lux = kIntensity * taper(theta) * std::cos(theta) / (d * d);
        // The summer haze over the eye's distance (realistic.slang's fogged, 0.0009 per m).
        const double eye = std::hypot(x, kEyeHeight);
        const double expected = kRho * lux * k / std::numbers::pi * std::exp(-eye * 0.0009);
        // A 3x3 patch about the point's pixel, to average the 8-bit steps.
        const double u = camera.cx + camera.fx * x / kEyeHeight;
        double sum = 0.0;
        for (int dv = -1; dv <= 1; ++dv) {
            for (int du = -1; du <= 1; ++du) {
                const std::size_t i =
                    4 * ((static_cast<std::size_t>(camera.cy) + dv) * camera.width +
                         static_cast<std::size_t>(std::lround(u)) + du);
                sum += (grey.rgba[i + 1] - black.rgba[i + 1]) / 255.0;  // green
            }
        }
        const double measured = sum / 9.0;
        EXPECT_NEAR(measured / expected, 1.0, 0.02) << theta << " rad, " << lux << " lx";
        if (log) {
            log << theta << ',' << lux << ',' << expected << ',' << measured << ','
                << measured / expected << '\n';
        }
    }
}

// The water's optics (water_optics, water.md "How the water looks"): Fresnel's reflectance of
// water is 0.020 at normal incidence ((n−1)²/(n+1)²), about 0.021 at 30°, 0.058 at 60°
// (Born and Wolf §1.5.2) and goes to 1 at grazing.
TEST(WaterOptics, FresnelOfWater) {
    using water_optics::fresnel;
    const double n = water_optics::kRefractiveIndex;
    EXPECT_NEAR(fresnel(1.0), (n - 1.0) * (n - 1.0) / ((n + 1.0) * (n + 1.0)), 1e-12);
    EXPECT_NEAR(fresnel(1.0), 0.0204, 2e-4);
    EXPECT_NEAR(fresnel(std::cos(60.0 * std::numbers::pi / 180.0)), 0.0596, 1e-3);
    EXPECT_GT(fresnel(std::cos(89.9 * std::numbers::pi / 180.0)), 0.98);
    EXPECT_DOUBLE_EQ(fresnel(0.0), 1.0);
    for (double c = 0.05; c < 1.0; c += 0.05) {
        EXPECT_GT(fresnel(c - 0.05), fresnel(c));  // more at more grazing incidence
    }
}

// The ripples' slope variance: Cox and Munk's clean minus slick at the wind a friction velocity
// makes at 12.5 m; none below the onset (u* ≈ 4.5 cm/s, Kahma and Donelan 1988), rising with u*,
// and at the wind of a 10 m/s sea, under Cox and Munk's whole clean-sea 0.054.
TEST(WaterOptics, RippleSlopeVarianceFollowsTheWind) {
    using namespace water_optics;
    EXPECT_DOUBLE_EQ(rippleSlopeVariance(0.0), 0.0);
    EXPECT_DOUBLE_EQ(rippleSlopeVariance(0.03), 0.0);  // under the onset: still, a mirror
    EXPECT_GT(rippleSlopeVariance(0.06), 0.0);
    // The onset: where clean and slick meet, U12.5 = 0.005/3.56e-3 = 1.40 m/s.
    double onset = 0.0;
    for (double u = 0.01; u < 0.2; u += 1e-4) {
        if (rippleSlopeVariance(u) > 0.0) {
            onset = u;
            break;
        }
    }
    EXPECT_NEAR(windAt(onset, kCoxMunkHeight), 0.005 / 3.56e-3, 0.01);
    EXPECT_GT(onset, 0.035);
    EXPECT_LT(onset, 0.10);
    for (double u = 0.06; u < 0.6; u += 0.02) {
        EXPECT_GT(rippleSlopeVariance(u + 0.02), rippleSlopeVariance(u));
    }
    // A 10 m/s wind at 10 m: u* by the log law and Charnock is about 0.35 m/s (Smith 1988's
    // drag coefficient 1.2e-3 gives 0.35), and the ripples hold under the whole clean sea's.
    const double u = frictionVelocityOf(10.0);
    EXPECT_NEAR(u, 0.35, 0.04);
    EXPECT_NEAR(windAt(u, 10.0), 10.0, 1e-6);
    const double clean = coxMunkClean(windAt(u, kCoxMunkHeight));
    EXPECT_NEAR(clean, 0.003 + 5.12e-3 * 10.2, 0.003);
    EXPECT_GT(rippleSlopeVariance(u), 0.3 * clean);
    EXPECT_LT(rippleSlopeVariance(u), clean);
}

// Cox and Munk's anisotropy: the short waves' upwind share rises as 2.38e-3·U, the crosswind as
// 1.08e-3·U (clean minus slick of each), so σ_u²/σ_c² = 2.2, and the two add to the total.
TEST(WaterOptics, RipplesAreSteeperAlongTheWindThanAcross) {
    using namespace water_optics;
    for (const double u : {0.08, 0.2, 0.5}) {
        const auto [up, cross] = rippleSlopeVariances(u);
        EXPECT_NEAR(up + cross, rippleSlopeVariance(u), 1e-15) << u;
        EXPECT_NEAR(up / cross, 2.38 / 1.08, 1e-9) << u;
    }
    const auto calm = rippleSlopeVariances(0.03);
    EXPECT_EQ(calm[0], 0.0);
    EXPECT_EQ(calm[1], 0.0);
}

// The ripple of least phase speed: 1.71 cm, 0.231 m/s (Lamb §267). The air grows it only above
// u* = √(100·ν·k·c) ≈ 9 cm/s, where Plant's growth passes viscous decay.
TEST(WaterOptics, RipplesGrowOnlyWhereTheAirOutrunsViscosity) {
    using namespace water_optics;
    EXPECT_NEAR(2.0 * std::numbers::pi / rippleWavenumber(), 0.0171, 2e-4);
    EXPECT_NEAR(ripplePhaseSpeed(), 0.231, 2e-3);
    const double threshold =
        std::sqrt(100.0 * kWaterViscosity * rippleWavenumber() * ripplePhaseSpeed());
    EXPECT_EQ(rippleGrowthRate(0.99 * threshold), 0.0);
    EXPECT_GT(rippleGrowthRate(1.01 * threshold), 0.0);
    EXPECT_GT(fetchRate(0.4), fetchRate(0.2));
}

// A jet's outwash on the grid: u* rising as (ρ/ρ_p)^0.8 to a peak at ρ_p and falling as
// (ρ_p/ρ)^1.1 (Tanner et al. 2015, as the aerodynamics domain's), outward. The ripples are
// calm where the air leaves the stagnation point (no fetch, too weak to grow them), grow over
// the annulus to near their equilibrium, and the stress's direction is radial.
TEST(WaterOptics, TheFetchLeavesACalmCentreUnderAJet) {
    using namespace water_optics;
    constexpr std::size_t n = 128;
    constexpr double cell = 0.03;
    constexpr double peakU = 0.5;    // m/s
    constexpr double peakAt = 0.45;  // m
    const double mid = 0.5 * static_cast<double>(n - 1);
    std::vector<float> friction(2 * n * n);
    const auto uStar = [&](double r) {
        return peakU * (r < peakAt ? std::pow(r / peakAt, 0.8) : std::pow(peakAt / r, 1.1));
    };
    for (std::size_t y = 0; y < n; ++y) {
        for (std::size_t x = 0; x < n; ++x) {
            const double dx = (static_cast<double>(x) - mid) * cell;
            const double dy = (static_cast<double>(y) - mid) * cell;
            const double r = std::hypot(dx, dy);
            friction[y * n + x] = static_cast<float>(uStar(r) * dx / r);
            friction[n * n + y * n + x] = static_cast<float>(uStar(r) * dy / r);
        }
    }
    std::vector<double> variance;
    std::vector<double> direction;
    fetchLimitedRipples(n, cell, friction, {0.0, 0.0}, variance, direction);
    const auto along = [&](double r) {  // on the +x axis, r from the centre
        const auto x = static_cast<std::size_t>(std::lround(mid + r / cell));
        const auto y = static_cast<std::size_t>(std::lround(mid));
        return std::pair{variance[y * n + x], direction[y * n + x]};
    };
    EXPECT_EQ(along(0.05).first, 0.0);  // calm: u* under the growth threshold
    const double inner = along(0.15).first / rippleSlopeVariance(uStar(0.15));
    const double ring = along(peakAt).first / rippleSlopeVariance(peakU);
    EXPECT_LT(inner, 0.6);  // the fetch lags
    EXPECT_GT(ring, 0.6);   // grown by the peak
    EXPECT_LT(ring, 1.0 + 1e-9);
    EXPECT_NEAR(along(peakAt).second, 1.0, 1e-3);  // runs outward
    // The roughest water is in the annulus, not under the jet.
    double best = 0.0;
    double bestAt = 0.0;
    for (double r = 0.0; r < 1.8; r += cell) {
        if (along(r).first > best) {
            best = along(r).first;
            bestAt = r;
        }
    }
    EXPECT_GT(bestAt, 0.6 * peakAt);
    EXPECT_LT(bestAt, 2.0 * peakAt);
    // Without air the water stays a mirror.
    fetchLimitedRipples(n, cell, {}, {0.0, 0.0}, variance, direction);
    EXPECT_EQ(*std::ranges::max_element(variance), 0.0);
}

// Whitecaps: Monahan and O'Muircheartaigh's 3.84e-6·U10^3.41 (1 % at about 10 m/s); in a
// rotor's wall jet none below the Kelvin–Helmholtz onset of its speed (6.7 m/s, where its spray
// starts), Monahan's above.
TEST(WaterOptics, WhitecapsOnlyWhereTheOutwashTearsTheSurface) {
    using namespace water_optics;
    EXPECT_NEAR(kelvinHelmholtzOnset(), 6.7, 0.1);
    EXPECT_NEAR(whitecapFraction(10.0), 3.84e-6 * std::pow(10.0, 3.41), 1e-12);
    EXPECT_NEAR(whitecapFraction(10.0), 0.01, 1e-3);
    EXPECT_EQ(whitecapFraction(0.0), 0.0);
    const double onset = kelvinHelmholtzOnset() * kWallJetFriction;  // u* there
    EXPECT_EQ(whitecapOf(0.99 * onset, 0.0), 0.0);
    EXPECT_GT(whitecapOf(1.01 * onset, 0.0), 0.0);
    EXPECT_NEAR(whitecapOf(0.6, 0.0), whitecapFraction(windAt(0.6, 10.0)), 1e-12);
    EXPECT_GT(whitecapOf(0.6, 0.0), whitecapOf(0.4, 0.0));
}

// At night with no lamp the picture is black, and a lamp lights a pool on the plane.
// The water's moving surface (RealisticWaterSurface, water.md): a dimple in the patch changes the
// water's colour where it lies and nowhere else; the same surface twice draws the same image.
// Seen at 45°, where its slopes turn what it reflects (from straight down a dimple over a pale
// floor just under it barely shows, as on real water).
TEST(RealisticWater, TheMovingSurfaceShadesTheWaterWhereItLies) {
    const std::vector<float> positions{-3, -3, 0, 3, -3, 0, 3, 3, 0, -3, 3, 0};
    const std::vector<float> normals{0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1};
    const std::vector<std::uint32_t> indices{0, 1, 2, 0, 2, 3};
    const std::vector<MeshSource> meshes{{positions, normals, indices}};
    std::string error;
    const char* gpu = std::getenv("LAGRANGE_TEST_GPU");
    auto scene = RasterScene::create(gpu != nullptr ? gpu : "", meshes, error, {.realistic = true});
    ASSERT_TRUE(scene) << error;
    if (!scene->realistic()) {
        GTEST_SKIP() << "a software device keeps the simple shading";
    }
    PinholeCamera camera;  // 1.5 m over the water, looking along +x 45° down
    camera.width = 240;
    camera.height = 240;
    camera.fx = camera.fy = 120.0;
    camera.cx = camera.cy = 119.5;
    camera.position = {-0.2, 0.0, 1.5};
    // Straight down (180° about x), then turned −45° about world y: q = qy(−45°)·qx(180°).
    const double half = -std::numbers::pi / 8.0;
    camera.orientation = {0.0, std::cos(half), 0.0, -std::sin(half)};
    RasterInstance water;
    water.water = true;
    water.box = {-3.0F, -3.0F, -1.0F, 3.0F, 3.0F, 0.0F};
    const std::vector<RasterInstance> instances{water};
    RasterCamera renderer(scene, camera.width, camera.height);
    // A 1.92 m patch in the +x half (x 0.3 to 2.22 m), a 30 mm dimple with rings in its middle.
    constexpr std::uint32_t kCells = 64;
    constexpr double kCell = 0.03;
    const std::array<double, 2> first{0.3 + kCell / 2.0, -0.96 + kCell / 2.0};
    std::vector<float> heights(kCells * kCells);
    for (std::uint32_t y = 0; y < kCells; ++y) {
        for (std::uint32_t x = 0; x < kCells; ++x) {
            const double dx = (x - 31.5) * kCell;
            const double dy = (y - 31.5) * kCell;
            const double r = std::hypot(dx, dy);
            heights[y * kCells + x] =
                static_cast<float>(-0.03 * std::exp(-r * r / 0.04) + 0.002 * std::cos(r / 0.02));
        }
    }
    const auto shoot = [&](bool moving) {
        RealisticWaterSurface surface;
        if (moving) {
            surface.cells = kCells;
            surface.cell = kCell;
            surface.first = first;
            surface.heights = heights;
        }
        renderer.setWaterSurface(surface);
        RasterImages out;
        renderer.render(instances, camera, out, 1.0);
        return out;
    };
    const RasterImages still = shoot(false);
    const RasterImages moved = shoot(true);
    const RasterImages again = shoot(true);
    EXPECT_EQ(moved.rgba, again.rgba);
    // Pixel of a world point on the water (z 0): into the camera's frame, Ry(45°) then Rx(180°).
    const auto difference = [&](double x0, double x1, double y0, double y1) {
        double sum = 0.0;
        int count = 0;
        const double c45 = std::sqrt(0.5);
        for (double x = x0; x <= x1; x += 0.05) {
            for (double y = y0; y <= y1; y += 0.05) {
                const double dx = x - camera.position[0];
                const double dy = y - camera.position[1];
                const double dz = -camera.position[2];
                const double cx = dx * c45 + dz * c45;
                const double cy = -dy;
                const double cz = -(-dx * c45 + dz * c45);
                const long pu = std::lround(camera.cx + camera.fx * cx / cz);
                const long pv = std::lround(camera.cy + camera.fy * cy / cz);
                if (cz <= 0.0 || pu < 0 || pv < 0 || pu >= static_cast<long>(camera.width) ||
                    pv >= static_cast<long>(camera.height)) {
                    continue;
                }
                const auto u = static_cast<std::size_t>(pu);
                const auto v = static_cast<std::size_t>(pv);
                for (std::size_t c = 0; c < 3; ++c) {
                    const std::size_t i = 4 * (v * camera.width + u) + c;
                    sum +=
                        std::abs(static_cast<int>(moved.rgba[i]) - static_cast<int>(still.rgba[i]));
                }
                ++count;
            }
        }
        EXPECT_GT(count, 20);
        return sum / (3.0 * std::max(count, 1));
    };
    // The dimple: its rings shift what the water reflects by a level or two, and its slopes
    // break up the sun's reflection (the image's largest change, far over 8-bit noise).
    EXPECT_GT(difference(0.96, 1.56, -0.3, 0.3), 0.3);  // 8-bit levels on average
    EXPECT_LT(difference(0.6, 1.6, 1.3, 1.9), 0.05);    // beside the patch: untouched
    int largest = 0;
    for (std::size_t i = 0; i < moved.rgba.size(); ++i) {
        largest = std::max(
            largest, std::abs(static_cast<int>(moved.rgba[i]) - static_cast<int>(still.rgba[i])));
    }
    EXPECT_GT(largest, 100);
}

TEST(RealisticLight, NightIsDarkButForTheLamp) {
    const std::vector<float> positions{-50, -50, 0, 50, -50, 0, 50, 50, 0, -50, 50, 0};
    const std::vector<float> normals{0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1};
    const std::vector<std::uint32_t> indices{0, 1, 2, 0, 2, 3};
    const std::vector<MeshSource> meshes{{positions, normals, indices}};
    std::string error;
    auto scene = RasterScene::create("", meshes, error, {.realistic = true});
    ASSERT_TRUE(scene) << error;
    if (!scene->realistic()) {
        GTEST_SKIP() << "a software device keeps the simple shading";
    }
    PinholeCamera camera;
    camera.width = 320;
    camera.height = 240;
    camera.fx = camera.fy = 200.0;
    camera.cx = 159.5;
    camera.cy = 119.5;
    camera.position = {0.0, 0.0, 2.0};
    camera.orientation = {0.5, -0.5, 0.5, -0.5};  // looking along +x
    RasterInstance floor;
    floor.roughness = 0.8F;
    const std::vector<RasterInstance> instances{floor};
    RasterCamera renderer(scene, camera.width, camera.height);
    const auto mean = [&](const RasterImages& image) {
        double sum = 0.0;
        for (std::size_t i = 0; i < image.rgba.size(); i += 4) {
            sum += image.rgba[i] + image.rgba[i + 1] + image.rgba[i + 2];
        }
        return sum / (3.0 * static_cast<double>(image.rgba.size() / 4));
    };
    constexpr double kMoonless = 0.8e-3;  // lx, RFC 0021 §6
    RasterImages day, night, lit;
    renderer.render(instances, camera, day, 1.0);
    renderer.setLighting({}, kMoonless);
    renderer.render(instances, camera, night, 1.0);
    RealisticLamp lamp;  // a 1500 cd headlamp 0.5 m up, tilted down onto the floor ahead
    lamp.position = {0.5, 0.0, 0.5};
    lamp.axis = {std::cos(0.3), 0.0, -std::sin(0.3)};
    lamp.intensity = 1500.0;
    lamp.beamAngle = 0.35;
    lamp.fieldAngle = 0.7;
    lamp.color = colorOfTemperature(5700.0);
    const std::vector<RealisticLamp> lamps{lamp};
    renderer.setLighting(lamps, kMoonless);
    renderer.render(instances, camera, lit, 1.0);
    EXPECT_GT(mean(day), 60.0);
    EXPECT_LT(mean(night), 2.0);
    EXPECT_GT(mean(lit), mean(night) + 2.0);
    if (const char* dir = std::getenv("LAGRANGE_TEST_OUT")) {
        for (const auto& [name, image] :
             {std::pair{"day", &day}, std::pair{"night", &night}, std::pair{"lit", &lit}}) {
            std::ofstream ppm(std::string(dir) + "/camera_" + name + ".ppm", std::ios::binary);
            ppm << "P6\n" << camera.width << ' ' << camera.height << "\n255\n";
            for (std::size_t i = 0; i < image->rgba.size(); i += 4) {
                ppm.write(reinterpret_cast<const char*>(&image->rgba[i]), 3);
            }
        }
    }
}

}  // namespace
}  // namespace lagrange::gpu
