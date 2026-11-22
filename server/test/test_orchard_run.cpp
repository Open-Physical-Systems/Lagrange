// The orchard sample (assets/worlds/orchard, ground.md "Trees"): 140 TreeAPI_1 trees in rows and
// a forest edge, loaded whole, run on the CPU faster than the wall clock. A hard real-time-factor
// threshold would be flaky on a loaded machine, so each run checks its wall time per simulated
// second against a bound a few times over what it takes here (docs/physics/ground.md "The
// orchard": about 0.04 s per simulated second for the quadcopter, 0.1 s for the sedan and its
// CPU lidar).
#include "lagrange/server/Server.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

/// Wall seconds per simulated second at most: RTF 1, which the runs here beat by 10x or more.
constexpr double kMostWallPerSimSecond = 1.0;

double runFor(Server& server, double seconds) {
    const auto begin = std::chrono::steady_clock::now();
    const std::int64_t end = server.simTime().ns + static_cast<std::int64_t>(seconds * 1e9);
    while (server.simTime().ns < end) {
        EXPECT_TRUE(server.stepUntil(SimTime{server.simTime().ns + 100'000'000}).ok());
    }
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
}

TEST(OrchardRun, TheQuadcopterHoversOverThePadFasterThanRealTime) {
    ServerOptions o;
    o.robotPath = LAGRANGE_TEST_QUAD_USDA;
    o.worldPath = LAGRANGE_TEST_ORCHARD;
    o.airDensityKgM3 = 1.225;
    o.ros = false;
    o.realTimeFactor = 0.0;
    o.lidar = ServerOptions::Lidar::Cpu;
    o.spawn.position = {-36.0, 0.0, 0.1};  // on the landing pad
    Server server(o);
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    // Every tree is in the run: the rows' and the forest edge's.
    for (const char* tree : {"row1_01", "row8_15", "edge_01"}) {
        EXPECT_EQ(server.read(std::string("tree/") + tree + "/sway").size(), 3U) << tree;
    }
    runFor(server, 1.0);  // warm up
    const double wall = runFor(server, 4.0);
    const std::vector<double> pose = server.read("body/base_link/pose");
    std::printf("orchard, quadcopter: %.3f s of wall time per simulated second (RTF %.1f)\n",
                wall / 4.0, 4.0 / wall);
    EXPECT_LT(wall / 4.0, kMostWallPerSimSecond);
    ASSERT_GE(pose.size(), 3U);
    EXPECT_GT(pose[2], 1.0) << "it took off over the pad";
}

TEST(OrchardRun, TheSedansCpuLidarAcrossTheRowsFasterThanRealTime) {
    ServerOptions o;
    o.robotPath = LAGRANGE_TEST_SEDAN_USDA;
    o.worldPath = LAGRANGE_TEST_ORCHARD;
    o.ros = false;
    o.realTimeFactor = 0.0;
    o.demoDrive = false;
    o.lidar = ServerOptions::Lidar::Cpu;
    o.spawn.position = {-34.0, -16.0, 0.5};  // on the headland, looking along the rows
    Server server(o);
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    runFor(server, 0.5);
    const double wall = runFor(server, 2.0);
    // Its 32 x 1024 beams see the crowns: some stop in leaves, short of the far trunks.
    const std::vector<double> ranges = server.read("sensor/roof_lidar/ranges");
    std::size_t returns = 0;
    for (const double r : ranges) {
        returns += std::isfinite(r) ? 1U : 0U;
    }
    std::printf(
        "orchard, sedan with a CPU lidar: %.3f s of wall time per simulated second "
        "(RTF %.1f), %zu of %zu beams returned\n",
        wall / 2.0, 2.0 / wall, returns, ranges.size());
    EXPECT_LT(wall / 2.0, kMostWallPerSimSecond);
    EXPECT_GT(returns, ranges.size() / 4);
}

}  // namespace
}  // namespace lagrange::server
