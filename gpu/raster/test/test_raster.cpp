// The camera sensor's projection (docs/design/domains/sensors.md §7): checkerboards rendered at
// known poses are found with OpenCV; the corners lie where the pinhole model puts them, and a
// calibration from the images recovers the intrinsics. Reprojection error < 0.1 px.
#include "lagrange/gpu/Raster.hpp"

#include <cmath>
#include <cstdlib>
#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
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

}  // namespace
}  // namespace lagrange::gpu
