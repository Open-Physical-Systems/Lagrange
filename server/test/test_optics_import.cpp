// Lidar scan patterns and fisheye lenses from a URDF's <gazebo> sensors (sensors.md, "Lidar scan
// patterns (as built)" and "Fisheye cameras (as built)"), and a rosette lidar run end to end.
#include "lagrange/server/Robot.hpp"
#include "lagrange/server/Server.hpp"
#include "lagrange/world/LidarLayout.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <unistd.h>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

std::string opticsRig() {
    const std::filesystem::path path = std::filesystem::temp_directory_path() /
                                       ("lagrange_optics_" + std::to_string(::getpid()) + ".urdf");
    std::ofstream(path) << R"(<?xml version="1.0"?><robot name="rig"><link name="world"/>
<link name="head"><inertial><mass value="5"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0"
iyz="0"/></inertial></link>
<joint name="fix" type="fixed"><parent link="world"/><child link="head"/><origin xyz="0 0 1"/>
</joint>
<link name="wall"><collision><origin xyz="5.5 0 0"/><geometry><box size="1 200 200"/></geometry>
</collision></link><joint name="wall_fix" type="fixed"><parent link="world"/><child link="wall"/>
</joint>
<gazebo reference="head">
<sensor name="rosette" type="lidar"><update_rate>10</update_rate><ray><pattern>rosette</pattern>
<scan><horizontal><samples>500</samples></horizontal></scan><range><min>0.1</min><max>60</max>
</range><rosette><fov>0.8</fov><point_rate>5000</point_rate><speed_1>700</speed_1>
<speed_2>-400</speed_2><prism_ratio>0.5</prism_ratio></rosette></ray></sensor>
<sensor name="flash" type="lidar"><update_rate>10</update_rate><ray><pattern>flash</pattern>
<scan><horizontal><samples>16</samples><min_angle>-0.5</min_angle><max_angle>0.5</max_angle>
</horizontal><vertical><samples>4</samples><min_angle>-0.2</min_angle><max_angle>0.2</max_angle>
</vertical></scan></ray></sensor>
<sensor name="vlp" type="ray"><update_rate>10</update_rate><ray><scan><horizontal>
<samples>8</samples></horizontal><vertical><samples>2</samples>
<angles>0.1 -0.3 0.0</angles></vertical></scan></ray></sensor>
<sensor name="fisheye" type="camera"><update_rate>10</update_rate><camera>
<horizontal_fov>3.4907</horizontal_fov><image><width>64</width><height>64</height></image>
<lens><type>equidistant</type><cutoff_angle>1.8</cutoff_angle><k1>0.01</k1><k4>0.0001</k4>
</lens></camera></sensor>
<sensor name="odd" type="camera"><camera><lens><type>stereographic</type></lens></camera></sensor>
</gazebo></robot>)";
    return path.string();
}

TEST(OpticsImport, PatternsAndLensesReachTheSettings) {
    const std::string path = opticsRig();
    const auto robot = loadUrdf(path);
    ASSERT_TRUE(robot) << robot.error();
    const World::Sensors& s = robot->world.sensors;
    const auto find = [&](const std::string& name) {
        for (std::size_t i = 0; i < s.names.size(); ++i) {
            if (s.names[i] == name) {
                return s.settings[i];
            }
        }
        ADD_FAILURE() << name;
        return LidarLayout::Settings{};
    };
    const LidarLayout rosette = LidarLayout::from(find("rosette"));
    EXPECT_EQ(rosette.pattern, LidarLayout::Pattern::Rosette);
    EXPECT_EQ(rosette.horizontal, 500U);
    EXPECT_DOUBLE_EQ(rosette.rosette.pointRate, 5000.0);
    EXPECT_DOUBLE_EQ(rosette.rosette.speed1, 700.0);
    EXPECT_DOUBLE_EQ(rosette.rosette.speed2, -400.0);
    EXPECT_NEAR(rosette.rosette.deflection1 + rosette.rosette.deflection2, 0.4, 1e-12);
    EXPECT_NEAR(rosette.rosette.deflection2 / rosette.rosette.deflection1, 0.5, 1e-12);

    const auto flashSettings = find("flash");
    const LidarLayout flash = LidarLayout::from(flashSettings);
    EXPECT_EQ(flash.pattern, LidarLayout::Pattern::Flash);
    EXPECT_EQ(flashSettings.at("motion_distortion"), 0.0);
    EXPECT_EQ(flash.beams(), 64U);

    const LidarLayout vlp = LidarLayout::from(find("vlp"));
    ASSERT_EQ(vlp.vertical, 3U);  // the table, not <samples>
    EXPECT_EQ(vlp.elevation(0), -0.3);
    EXPECT_EQ(vlp.elevation(1), 0.0);
    EXPECT_EQ(vlp.elevation(2), 0.1);

    const auto fisheye = find("fisheye");
    EXPECT_EQ(fisheye.at("projection"), 1.0);
    EXPECT_EQ(fisheye.at("distortion_model"), 2.0);
    EXPECT_EQ(fisheye.at("fisheye_cutoff"), 1.8);
    EXPECT_EQ(fisheye.at("distortion_k1"), 0.01);
    EXPECT_EQ(fisheye.at("distortion_k4"), 0.0001);
    EXPECT_FALSE(fisheye.contains("distortion_k2"));
    EXPECT_FALSE(find("odd").contains("projection"));
    bool noted = false;
    for (const std::string& note : robot->notes) {
        noted = noted || note.find("stereographic") != std::string::npos;
    }
    EXPECT_TRUE(noted);
    std::filesystem::remove(path);
}

TEST(OpticsImport, ARosetteLidarRunsAndSaysWhereItsBeamsWent) {
    const std::string path = opticsRig();
    ServerOptions o;
    o.robotPath = path;
    o.worldPath.clear();
    o.demoDrive = false;
    o.ros = false;
    o.realTimeFactor = 0.0;
    o.lidar = ServerOptions::Lidar::Cpu;
    o.cameras = false;  // no device needed for the fisheye
    Server server(o);
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.stepUntil(SimTime{350'000'000}).ok());
    const std::vector<double> ranges = server.read("sensor/rosette/ranges");
    const std::vector<double> d = server.read("sensor/rosette/directions");
    ASSERT_EQ(ranges.size(), 500U);
    ASSERT_EQ(d.size(), 1500U);
    for (std::size_t i = 0; i < 500; ++i) {
        // The wall 5 m ahead, or the ground 1 m below, whichever the beam meets first.
        const double ground = d[3 * i + 2] < 0.0 ? 1.0 / -d[3 * i + 2] : 1e9;
        EXPECT_NEAR(ranges[i], std::min(5.0 / d[3 * i], ground), 1e-6) << i;
    }
    const std::vector<double> flash = server.read("sensor/flash/ranges");
    ASSERT_EQ(flash.size(), 64U);
    EXPECT_TRUE(server.read("sensor/flash/directions").empty());
    std::filesystem::remove(path);
}

}  // namespace
}  // namespace lagrange::server
