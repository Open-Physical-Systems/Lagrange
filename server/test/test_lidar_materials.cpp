// Lidar intensity from materials (docs/design/domains/sensors.md §2).
#include "lagrange/sensors/ShapeCaster.hpp"
#include "lagrange/server/Server.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <numbers>
#include <string>
#include <unistd.h>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

using sensors::lidarReturn;
using sensors::LidarReturn;

// Against the closed form: a Lambertian target at a known range and angle (sensors.md §7, 2 %).
TEST(LidarReturn, LambertianTargetByReflectivityAngleAndRange) {
    const SurfaceMaterial wall{0.5, 0.25, 0.0, 0.0, false};
    const double kAngle = 60.0 * std::numbers::pi / 180.0;
    EXPECT_NEAR(lidarReturn(wall, true, false, 1.0, 2.0, 30.0, 0.1).intensity, 0.5 / 4.0, 1e-12);
    EXPECT_NEAR(lidarReturn(wall, true, false, std::cos(kAngle), 2.0, 30.0, 0.1).intensity,
                0.5 * 0.5 / 4.0, 1e-12);
    EXPECT_NEAR(lidarReturn(wall, true, true, 1.0, 2.0, 30.0, 0.1).intensity, 0.25 / 4.0, 1e-12)
        << "at 1550 nm";
    EXPECT_EQ(lidarReturn(wall, true, false, 1.0, 0.1, 30.0, 0.1).intensity, 1.0) << "saturates";
}

TEST(LidarReturn, WaterMirrorsAndDropsOut) {
    SurfaceMaterial water;
    ASSERT_TRUE(surfaceMaterialPreset("water", water));
    const auto at = [&](double degrees, double range) {
        return lidarReturn(water, true, false, std::cos(degrees * std::numbers::pi / 180.0), range,
                           30.0, 0.1);
    };
    EXPECT_TRUE(at(0.0, 1.0).detected) << "straight down, close";
    EXPECT_FALSE(at(30.0, 1.0).detected) << "at a slant the mirror sends it away";
    EXPECT_FALSE(at(0.0, 25.0).detected) << "2 % is too faint from far";
    EXPECT_FALSE(surfaceMaterialPreset("banana", water));
}

TEST(LidarReturn, ARetroreflectorComesBackAtAnyAngle) {
    SurfaceMaterial sign;
    ASSERT_TRUE(surfaceMaterialPreset("retroreflector", sign));
    const LidarReturn slant = lidarReturn(sign, true, false, 0.3, 20.0, 30.0, 0.1);
    EXPECT_TRUE(slant.detected);
    EXPECT_NEAR(slant.intensity, 100.0 * 0.9 / 400.0, 1e-12);
    // No material: always a return, whatever its power.
    EXPECT_TRUE(lidarReturn(SurfaceMaterial{}, false, false, 0.01, 29.0, 30.0, 0.1).detected);
}

TEST(SurfaceNormal, PointsOutOfEachShape) {
    using sensors::surfaceNormal;
    const Pose at;
    EXPECT_EQ(surfaceNormal(ShapeType::Plane, {}, at, {3, 1, 0})[2], 1.0);
    EXPECT_NEAR(surfaceNormal(ShapeType::Sphere, {1, 0, 0}, at, {0, 1, 0})[1], 1.0, 1e-12);
    EXPECT_NEAR(surfaceNormal(ShapeType::Box, {1, 2, 3}, at, {1, 0.5, 0.5})[0], 1.0, 1e-12);
    EXPECT_NEAR(surfaceNormal(ShapeType::Box, {1, 2, 3}, at, {0.5, 0.5, -3})[2], -1.0, 1e-12);
    EXPECT_NEAR(surfaceNormal(ShapeType::Capsule, {0.5, 1, 0}, at, {0.5, 0, 0.3})[0], 1.0, 1e-12);
    EXPECT_NEAR(surfaceNormal(ShapeType::Capsule, {0.5, 1, 0}, at, {0, 0, 1.5})[2], 1.0, 1e-12);
    EXPECT_NEAR(surfaceNormal(ShapeType::Cylinder, {0.5, 1, 0}, at, {0, 0.5, 0})[1], 1.0, 1e-12);
    EXPECT_NEAR(surfaceNormal(ShapeType::Cylinder, {0.5, 1, 0}, at, {0.1, 0, 1})[2], 1.0, 1e-12);
    const Pose turned{{1, 0, 0}, {std::cos(0.7854), 0, 0, std::sin(0.7854)}};  // 90° about z
    EXPECT_NEAR(surfaceNormal(ShapeType::Box, {1, 1, 1}, turned, {2, 0, 0})[0], 1.0, 1e-3);
}

/// A wall 3 m ahead of the sample AMR's front lidar, with this <sensor_material>.
std::string wall(const std::string& material) {
    const std::filesystem::path path = std::filesystem::temp_directory_path() /
                                       ("lagrange_wall_" + std::to_string(::getpid()) + ".urdf");
    std::ofstream(path) << R"(<?xml version="1.0"?><robot name="scene"><link name="world"/>
<link name="wall"><collision><geometry><box size="0.2 10 4"/></geometry></collision></link>
<joint name="fix" type="fixed"><parent link="world"/><child link="wall"/>
<origin xyz="3 0 2"/></joint><gazebo reference="wall">)"
                        << material << "</gazebo></robot>";
    return path.string();
}

/// Range and intensity of the nearest return.
std::pair<double, double> ahead(const std::string& material) {
    ServerOptions o;
    o.robotPath = LAGRANGE_TEST_AMR_URDF;
    o.worldPath = wall(material);
    o.overlayPath.clear();
    o.demoDrive = false;
    o.ros = false;
    o.realTimeFactor = 0.0;
    Server server(o);
    EXPECT_TRUE(server.load().ok());
    EXPECT_TRUE(server.stepUntil(SimTime{300'000'000}).ok());
    const std::vector<double> ranges = server.read("sensor/front_lidar/ranges");
    const std::vector<double> intensities = server.read("sensor/front_lidar/intensities");
    EXPECT_EQ(ranges.size(), intensities.size());
    double best = std::numeric_limits<double>::infinity();
    double intensity = -1.0;
    for (std::size_t i = 0; i < ranges.size(); ++i) {
        if (ranges[i] < best) {
            best = ranges[i];
            intensity = intensities[i];
        }
    }
    return {best, intensity};
}

// End to end: the material in the URDF decides the intensity, and glass lets beams through.
TEST(LidarMaterials, AMaterialInTheUrdfSetsTheIntensity) {
    const auto [whiteRange, white] = ahead(R"(<sensor_material reflectivity_905="0.8"/>)");
    const auto [blackRange, black] = ahead(R"(<sensor_material preset="black_rubber"/>)");
    EXPECT_NEAR(whiteRange, blackRange, 0.02) << "the same wall";
    ASSERT_LT(whiteRange, 4.0);
    EXPECT_NEAR(white, 0.8 / (whiteRange * whiteRange), 0.02 * white) << "within 2 %";
    EXPECT_NEAR(black / white, 0.05 / 0.8, 0.02 * 0.05 / 0.8);
    const auto [mirrorRange, mirror] = ahead(R"(<sensor_material preset="mirror"/>)");
    EXPECT_NEAR(mirrorRange, whiteRange, 0.02) << "head on, a mirror returns";
    const auto [glassRange, glass] = ahead(R"(<sensor_material transmission="1"/>)");
    EXPECT_FALSE(std::isfinite(glassRange)) << "all beams pass: " << glass;
}

}  // namespace
}  // namespace lagrange::server
