// Lidar scan patterns and fisheye lenses from a URDF's <gazebo> sensors (sensors.md, "Lidar scan
// patterns (as built)" and "Fisheye cameras (as built)"), and a rosette lidar run end to end.
#include "lagrange/server/Robot.hpp"
#include "lagrange/server/Server.hpp"
#include "lagrange/stage/Stage.hpp"
#include "lagrange/world/FisheyeLens.hpp"
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

const LidarLayout::Settings& settingsOf(const Model& model, const std::string& name) {
    const World::Sensors& s = model.world.sensors;
    for (std::size_t i = 0; i < s.names.size(); ++i) {
        if (s.names[i] == name) {
            return s.settings[i];
        }
    }
    ADD_FAILURE() << "no sensor " << name;
    static const LidarLayout::Settings kNone;
    return kNone;
}

std::string usdOf(const std::string& urdf) {
    const auto robot = loadModel(urdf);
    EXPECT_TRUE(robot) << robot.error();
    const std::filesystem::path out = std::filesystem::temp_directory_path() /
                                      ("lagrange_optics_" + std::to_string(::getpid()) + ".usda");
    const auto written = stage::writeStage(*robot, out.string(), {});
    EXPECT_TRUE(written) << (written ? std::string() : written.error());
    return out.string();
}

// RFC 0015: URDF -> USD (as lagrange_import writes it) -> load gives the same beams, bit for bit,
// and the same lens (k and cutoff exactly; the angle through UsdGeomCamera's float focal length).
TEST(OpticsImport, PatternsAndLensesSurviveTheTripThroughUsd) {
    const std::string urdf = opticsRig();
    const auto robot = loadModel(urdf);
    ASSERT_TRUE(robot) << robot.error();
    const std::string usd = usdOf(urdf);
    const auto again = loadModel(usd);
    ASSERT_TRUE(again) << again.error();
    for (const std::string& note : again->notes) {
        EXPECT_EQ(note.find("rosette"), std::string::npos) << note;  // all five were written
    }
    for (const char* name : {"rosette", "flash", "vlp"}) {
        const auto& a = settingsOf(*robot, name);
        const auto& b = settingsOf(*again, name);
        const LidarLayout la = LidarLayout::from(a);
        const LidarLayout lb = LidarLayout::from(b);
        EXPECT_EQ(la.pattern, lb.pattern) << name;
        ASSERT_EQ(la.beams(), lb.beams()) << name;
        for (const std::uint64_t scan : {std::uint64_t{0}, std::uint64_t{7}}) {
            for (std::uint32_t i = 0; i < la.beams(); ++i) {
                ASSERT_EQ(la.direction(i, scan), lb.direction(i, scan))
                    << name << " beam " << i << " scan " << scan;
            }
        }
        EXPECT_EQ(a.contains("motion_distortion"), b.contains("motion_distortion")) << name;
    }
    const auto& a = settingsOf(*robot, "fisheye");
    const auto& b = settingsOf(*again, "fisheye");
    ASSERT_TRUE(FisheyeLens::isFisheye(b));
    const FisheyeLens la = FisheyeLens::from(a);
    const FisheyeLens lb = FisheyeLens::from(b);
    EXPECT_EQ(la.k, lb.k);
    EXPECT_EQ(la.cutoff, lb.cutoff);
    EXPECT_NEAR(b.at("horizontal_fov") / a.at("horizontal_fov"), 1.0, 1e-6);
    const double width = a.at("width");
    EXPECT_NEAR(lb.focal(width, b.at("horizontal_fov")) / la.focal(width, a.at("horizontal_fov")),
                1.0, 1e-6);
    EXPECT_FALSE(FisheyeLens::isFisheye(settingsOf(*again, "odd")));  // a pinhole, as from URDF

    // A CPU run of each gives the same ranges.
    const auto ranges = [](const std::string& path) {
        ServerOptions o;
        o.robotPath = path;
        o.worldPath.clear();
        o.demoDrive = false;
        o.ros = false;
        o.realTimeFactor = 0.0;
        o.lidar = ServerOptions::Lidar::Cpu;
        o.cameras = false;
        Server server(o);
        const Status loaded = server.load();
        EXPECT_TRUE(loaded.ok()) << loaded.message;
        EXPECT_TRUE(server.stepUntil(SimTime{350'000'000}).ok());
        std::vector<std::vector<double>> out;
        for (const char* name : {"rosette", "flash", "vlp"}) {
            out.push_back(server.read("sensor/" + std::string(name) + "/ranges"));
        }
        out.push_back(server.read("sensor/rosette/directions"));
        return out;
    };
    const auto fromUrdf = ranges(urdf);
    const auto fromUsd = ranges(usd);
    ASSERT_EQ(fromUrdf.size(), fromUsd.size());
    for (std::size_t i = 0; i < fromUrdf.size(); ++i) {
        ASSERT_FALSE(fromUrdf[i].empty()) << i;
        EXPECT_EQ(fromUrdf[i], fromUsd[i]) << i;
    }
    std::filesystem::remove(urdf);
    std::filesystem::remove(usd);
}

// RFC 0015 §4: the same sensors in SDFormat, with Lagrange's lagrange:-prefixed elements, give
// the URDF rig's beams and lens exactly. (SDFormat's horizontal angles default to 0, URDF's here
// to ±π: the vlp says them.)
TEST(OpticsImport, SdformatGivesTheUrdfRigsBeamsAndLens) {
    const std::string urdf = opticsRig();
    const std::filesystem::path sdf = std::filesystem::temp_directory_path() /
                                      ("lagrange_optics_" + std::to_string(::getpid()) + ".sdf");
    std::ofstream(sdf) << R"(<?xml version="1.0"?>
<sdf version="1.12" xmlns:lagrange="https://open-physical-systems.github.io/Lagrange">
<model name="rig"><static>true</static><link name="head"><pose>0 0 1 0 0 0</pose>
<sensor name="rosette" type="lidar"><update_rate>10</update_rate><lidar>
<lagrange:pattern>rosette</lagrange:pattern>
<scan><horizontal><samples>500</samples></horizontal></scan><range><min>0.1</min><max>60</max>
</range><lagrange:rosette><fov>0.8</fov><point_rate>5000</point_rate><speed_1>700</speed_1>
<speed_2>-400</speed_2><prism_ratio>0.5</prism_ratio></lagrange:rosette></lidar></sensor>
<sensor name="flash" type="lidar"><update_rate>10</update_rate><lidar>
<lagrange:pattern>flash</lagrange:pattern>
<scan><horizontal><samples>16</samples><min_angle>-0.5</min_angle><max_angle>0.5</max_angle>
</horizontal><vertical><samples>4</samples><min_angle>-0.2</min_angle><max_angle>0.2</max_angle>
</vertical></scan></lidar></sensor>
<sensor name="vlp" type="lidar"><update_rate>10</update_rate><lidar><scan><horizontal>
<samples>8</samples><min_angle>-3.14159265358979</min_angle><max_angle>3.14159265358979</max_angle>
</horizontal><vertical><samples>2</samples><lagrange:angles>0.1 -0.3 0.0</lagrange:angles></vertical></scan></lidar></sensor>
<sensor name="fisheye" type="camera"><update_rate>10</update_rate><camera>
<horizontal_fov>3.4907</horizontal_fov><image><width>64</width><height>64</height></image>
<lens><type>equidistant</type><cutoff_angle>1.8</cutoff_angle><lagrange:k1>0.01</lagrange:k1>
<lagrange:k4>0.0001</lagrange:k4></lens></camera></sensor>
</link></model></sdf>)";
    const auto fromUrdf = loadModel(urdf);
    const auto fromSdf = loadModel(sdf.string());
    ASSERT_TRUE(fromUrdf) << fromUrdf.error();
    ASSERT_TRUE(fromSdf) << fromSdf.error();
    for (const char* name : {"rosette", "flash", "vlp"}) {
        const auto& a = settingsOf(*fromUrdf, name);
        const auto& b = settingsOf(*fromSdf, name);
        const LidarLayout la = LidarLayout::from(a);
        const LidarLayout lb = LidarLayout::from(b);
        EXPECT_EQ(la.pattern, lb.pattern) << name;
        ASSERT_EQ(la.beams(), lb.beams()) << name;
        for (const std::uint64_t scan : {std::uint64_t{0}, std::uint64_t{7}}) {
            for (std::uint32_t i = 0; i < la.beams(); ++i) {
                ASSERT_EQ(la.direction(i, scan), lb.direction(i, scan)) << name << " " << i;
            }
        }
        EXPECT_EQ(a.contains("motion_distortion"), b.contains("motion_distortion")) << name;
    }
    const auto& a = settingsOf(*fromUrdf, "fisheye");
    const auto& b = settingsOf(*fromSdf, "fisheye");
    for (const char* key : {"projection", "distortion_model", "fisheye_cutoff", "distortion_k1",
                            "distortion_k4", "horizontal_fov"}) {
        EXPECT_EQ(a.at(key), b.at(key)) << key;
    }
    EXPECT_EQ(FisheyeLens::from(a).k, FisheyeLens::from(b).k);
    std::filesystem::remove(urdf);
    std::filesystem::remove(sdf);
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
