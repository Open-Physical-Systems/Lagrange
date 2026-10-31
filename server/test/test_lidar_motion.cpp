// Lidar motion distortion, against the closed form: a moving lidar and a wall (sensors.md).
#include "lagrange/server/Server.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <unistd.h>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

constexpr double kWallX = 12.0;  // m, the wall's face
constexpr double kPeriod = 0.1;  // s, 10 Hz
constexpr int kColumns = 101;    // over ±kFov
constexpr double kFov = 0.5;     // rad

/// A lidar on a joint of `type` (prismatic along x, or continuous about z), and a wall ahead.
std::string robotOn(const std::string& type, const std::string& axis, const std::string& extra) {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() /
        ("lagrange_motion_" + type + std::to_string(::getpid()) + ".urdf");
    std::ofstream(path) << R"(<?xml version="1.0"?><robot name="rig"><link name="world"/>
<link name="head"><inertial><mass value="5"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0"
iyz="0"/></inertial></link>
<joint name="move" type=")"
                        << type << R"("><parent link="world"/><child link="head"/>
<origin xyz="0 0 1"/><axis xyz=")"
                        << axis << R"("/><limit lower="-100" upper="100" effort="1000"
velocity="100"/></joint>
<link name="wall"><collision><origin xyz="12.5 0 0"/><geometry><box size="1 200 10"/></geometry>
</collision></link><joint name="wall_fix" type="fixed"><parent link="world"/><child link="wall"/>
</joint><gazebo reference="head"><sensor name="front" type="ray"><update_rate>10</update_rate><ray>
<scan><horizontal><samples>101</samples><min_angle>-0.5</min_angle><max_angle>0.5</max_angle>
</horizontal></scan><range><min>0.1</min><max>60</max></range>)"
                        << extra << R"(</ray></sensor></gazebo></robot>)";
    return path.string();
}

ServerOptions options(const std::string& robot, ServerOptions::Lidar lidar) {
    ServerOptions o;
    o.robotPath = robot;
    o.worldPath.clear();
    o.jointControl = true;
    o.demoDrive = false;
    o.ros = false;
    o.realTimeFactor = 0.0;
    o.lidar = lidar;
    return o;
}

double azimuth(int column) {
    return -kFov + 2.0 * kFov * column / (kColumns - 1);
}

double firedAt(int column) {
    return static_cast<double>(column + 1) / kColumns;  // of the scan
}

class Sweep : public testing::TestWithParam<ServerOptions::Lidar> {};

// Driving at the wall, each column sees it from a bit further back than the scan's end.
TEST_P(Sweep, DrivingStraightSkewsTheScan) {
    const auto o = options(robotOn("prismatic", "1 0 0", ""), GetParam());
    Server server(o);
    const Status loaded = server.load();
    if (!loaded.ok() && GetParam() == ServerOptions::Lidar::Gpu) {
        GTEST_SKIP() << loaded.message;
    }
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.commandJoint("move", "velocity", 2.0));
    ASSERT_TRUE(server.stepUntil(SimTime{1'900'000'000}).ok());
    const double before = server.read("joint/move/position").at(0);
    ASSERT_TRUE(server.stepUntil(SimTime{2'000'000'000}).ok());
    const double end = server.read("joint/move/position").at(0);
    const double moved = end - before;
    ASSERT_NEAR(moved, 2.0 * kPeriod, 0.01) << "at a steady speed";
    const std::vector<double> ranges = server.read("sensor/front/ranges");
    ASSERT_EQ(ranges.size(), static_cast<std::size_t>(kColumns));
    for (int c = 0; c < kColumns; ++c) {
        const double x = before + moved * firedAt(c);
        EXPECT_NEAR(ranges.at(c), (kWallX - x) / std::cos(azimuth(c)), 2e-3) << c;
    }
    EXPECT_GT(ranges.front() - ranges.back(), 0.15) << "same angle, either side: it skews";
}

// Spinning, each column looks where the sensor pointed at its moment.
TEST_P(Sweep, TurningBendsTheWall) {
    const auto o = options(robotOn("continuous", "0 0 1", ""), GetParam());
    Server server(o);
    const Status loaded = server.load();
    if (!loaded.ok() && GetParam() == ServerOptions::Lidar::Gpu) {
        GTEST_SKIP() << loaded.message;
    }
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.commandJoint("move", "velocity", 3.0));
    ASSERT_TRUE(server.stepUntil(SimTime{2'000'000'000}).ok());
    int checked = 0;
    for (std::int64_t scan = 21; scan < 45; ++scan) {
        const double before = server.read("joint/move/position").at(0);
        ASSERT_TRUE(server.stepUntil(SimTime{scan * 100'000'000}).ok());
        const double end = server.read("joint/move/position").at(0);
        const std::vector<double> ranges = server.read("sensor/front/ranges");
        for (int c = 0; c < kColumns; ++c) {
            const double heading = before + (end - before) * firedAt(c) + azimuth(c);
            if (std::cos(heading) > 0.6) {
                EXPECT_NEAR(ranges.at(c), kWallX / std::cos(heading), 3e-3) << scan << " " << c;
                ++checked;
            }
        }
    }
    EXPECT_GT(checked, 100);
}

INSTANTIATE_TEST_SUITE_P(Tiers, Sweep,
                         testing::Values(ServerOptions::Lidar::Cpu, ServerOptions::Lidar::Gpu));

// Off, the whole scan is taken at its end: a straight wall.
TEST(LidarMotion, CanBeTurnedOff) {
    const auto o =
        options(robotOn("prismatic", "1 0 0", "<motion_distortion>0</motion_distortion>"),
                ServerOptions::Lidar::Cpu);
    Server server(o);
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(server.commandJoint("move", "velocity", 2.0));
    ASSERT_TRUE(server.stepUntil(SimTime{2'000'000'000}).ok());
    const double end = server.read("joint/move/position").at(0);
    const std::vector<double> ranges = server.read("sensor/front/ranges");
    for (int c = 0; c < kColumns; ++c) {
        EXPECT_NEAR(ranges.at(c), (kWallX - end) / std::cos(azimuth(c)), 1e-6) << c;
    }
}

// A checkpoint carries where the last scan ended.
TEST(LidarMotion, ARestoredRunSweepsTheSame) {
    const auto o = options(robotOn("prismatic", "1 0 0", ""), ServerOptions::Lidar::Cpu);
    Server server(o);
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(server.commandJoint("move", "velocity", 2.0));
    ASSERT_TRUE(server.stepUntil(SimTime{1'000'000'000}).ok());
    const auto saved = server.saveState();
    ASSERT_TRUE(saved);
    ASSERT_TRUE(server.stepUntil(SimTime{1'100'000'000}).ok());
    const std::vector<double> first = server.read("sensor/front/ranges");
    ASSERT_TRUE(server.restoreState(*saved).ok());
    ASSERT_TRUE(server.stepUntil(SimTime{1'100'000'000}).ok());
    const std::vector<double> again = server.read("sensor/front/ranges");
    ASSERT_EQ(first.size(), again.size());
    for (std::size_t i = 0; i < first.size(); ++i) {
        EXPECT_EQ(first[i], again[i]) << i;
    }
}

// sensor/<n>/delayed/*: each scan arrives its latency late, unchanged, with its sampling time, at
// the bus stamp of its arrival; the sensor's own output stays on time. A run repeats and a restored
// run goes on as if it had not stopped, jitter included.
TEST(SensorDelay, TheDelayedSignalIsTheScanOfThen) {
    ServerOptions o = options(robotOn("prismatic", "1 0 0", ""), ServerOptions::Lidar::Cpu);
    o.sensorTiming.push_back({"front", 0.03, 0.0});
    Server server(o);
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(server.commandJoint("move", "velocity", 1.0));
    EXPECT_EQ(server.read("sensor/front/delayed/stamp").at(0), -1.0) << "nothing has arrived";

    std::map<std::int64_t, std::vector<double>> scans;  // by the time sampled
    int arrivals = 0;
    double lastStamp = -1.0;
    for (std::int64_t ms = 1; ms <= 1000; ++ms) {
        ASSERT_TRUE(server.stepUntil(SimTime{ms * 1'000'000}).ok());
        if (ms % 100 == 0) {
            scans[ms] = server.read("sensor/front/ranges");
        }
        const double stamp = server.read("sensor/front/delayed/stamp").at(0);
        if (stamp != lastStamp) {
            lastStamp = stamp;
            ++arrivals;
            const auto sampled = static_cast<std::int64_t>(std::llround(stamp * 1000.0));
            EXPECT_EQ(ms, sampled + 30) << "30 ms late";
            ASSERT_TRUE(scans.contains(sampled));
            EXPECT_EQ(server.read("sensor/front/delayed/ranges"), scans[sampled]);
        }
    }
    EXPECT_GE(arrivals, 9);
    EXPECT_NE(server.read("sensor/front/delayed/ranges"), server.read("sensor/front/ranges"))
        << "the robot moves, so a late scan differs";
}

TEST(SensorDelay, JitterStaysInItsBoundsAndRepeats) {
    const auto run = [] {
        ServerOptions o = options(robotOn("prismatic", "1 0 0", ""), ServerOptions::Lidar::Cpu);
        o.sensorTiming.push_back({"front", 0.04, 0.005});
        Server server(o);
        EXPECT_TRUE(server.load().ok());
        std::vector<std::int64_t> delays;
        double lastStamp = -1.0;
        std::vector<std::byte> saved;
        for (std::int64_t ms = 1; ms <= 2000; ++ms) {
            EXPECT_TRUE(server.stepUntil(SimTime{ms * 1'000'000}).ok());
            const double stamp = server.read("sensor/front/delayed/stamp").at(0);
            if (stamp != lastStamp) {
                lastStamp = stamp;
                delays.push_back(ms - std::llround(stamp * 1000.0));
            }
            if (ms == 1000) {
                saved = *server.saveState();
            }
        }
        EXPECT_TRUE(server.restoreState(saved).ok());
        lastStamp = server.read("sensor/front/delayed/stamp").at(0);
        std::vector<std::int64_t> again;
        for (std::int64_t ms = 1001; ms <= 2000; ++ms) {
            EXPECT_TRUE(server.stepUntil(SimTime{ms * 1'000'000}).ok());
            const double stamp = server.read("sensor/front/delayed/stamp").at(0);
            if (stamp != lastStamp) {
                lastStamp = stamp;
                again.push_back(ms - std::llround(stamp * 1000.0));
            }
        }
        return std::pair{delays, again};
    };
    const auto [delays, again] = run();
    ASSERT_GE(delays.size(), 15U);
    for (const std::int64_t late : delays) {
        EXPECT_GE(late, 25);  // 40 ms - 3 sigma
        EXPECT_LE(late, 56);  // 40 ms + 3 sigma, rounded up to the tick
    }
    EXPECT_GT(*std::max_element(delays.begin(), delays.end()),
              *std::min_element(delays.begin(), delays.end()))
        << "it jitters";
    EXPECT_EQ(run().first, delays) << "and repeats";
    // The last late values after the restore are those of the first time.
    ASSERT_FALSE(again.empty());
    EXPECT_TRUE(std::equal(again.rbegin(), again.rend(), delays.rbegin()));
}

// No sensor asks: no module, no signals.
TEST(SensorDelay, NoDelayNoSignals) {
    Server server(options(robotOn("prismatic", "1 0 0", ""), ServerOptions::Lidar::Cpu));
    ASSERT_TRUE(server.load().ok());
    for (const auto& signal : server.signals()) {
        EXPECT_EQ(signal.name.find("/delayed/"), std::string::npos) << signal.name;
    }
}

}  // namespace
}  // namespace lagrange::server
