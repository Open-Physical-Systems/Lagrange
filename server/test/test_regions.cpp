#include "lagrange/server/RegionHost.hpp"
#include "lagrange/server/Server.hpp"

#include <array>
#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

ServerOptions::ExtraRobot amrAt(const std::string& name, double x, double y) {
    ServerOptions::ExtraRobot robot;
    robot.name = name;
    robot.path = LAGRANGE_TEST_AMR_URDF;
    robot.spawn.position = {x, y, 0.0};
    return robot;
}

RegionHostOptions hostOptions(std::vector<ServerOptions::ExtraRobot> robots) {
    RegionHostOptions options;
    options.base.realTimeFactor = 0.0;
    options.base.ros = false;
    options.robots = std::move(robots);
    options.regions.radius = 500.0;
    return options;
}

// Roadmap §8 acceptance (active regions): robots 10 km apart. Each gets its own region: its own
// run, physics scene and floating origin near it, so the one 10 km out simulates exactly as the
// same robot alone at the origin.
TEST(Regions, RobotsTenKilometresApartRunInTheirOwnRegions) {
    RegionHost host(hostOptions({amrAt("west", 0.0, 0.0), amrAt("east", 10'000.0, 0.0)}));
    ASSERT_TRUE(host.load().ok());
    ASSERT_EQ(host.regionCount(), 2U);
    EXPECT_EQ(host.region(0).world().region.worldEnu, (std::array<double, 3>{0.0, 0.0, 0.0}));
    EXPECT_EQ(host.region(1).world().region.worldEnu, (std::array<double, 3>{10'000.0, 0.0, 0.0}));
    ASSERT_TRUE(host.stepUntil(SimTime{10'000'000'000}).ok());

    ServerOptions alone;
    alone.robotPath = LAGRANGE_TEST_AMR_URDF;
    alone.realTimeFactor = 0.0;
    alone.ros = false;
    Server reference(alone);
    ASSERT_TRUE(reference.load().ok());
    ASSERT_TRUE(reference.stepUntil(SimTime{10'000'000'000}).ok());
    const std::vector<double> expected = reference.read("body/base_link/pose");
    const std::array<double, 3> west = host.robotPosition(0);
    const std::array<double, 3> east = host.robotPosition(1);
    EXPECT_GT(std::hypot(west[0], west[1]), 0.5) << "it drove";
    for (std::size_t i = 0; i < 3; ++i) {
        EXPECT_NEAR(west.at(i), expected[i], 1e-9);
        EXPECT_NEAR(east.at(i) - (i == 0 ? 10'000.0 : 0.0), expected[i], 1e-9);
    }
}

// Robots whose bubbles overlap share one run: the second is in the first's world, passive,
// with its names prefixed.
TEST(Regions, RobotsCloseTogetherShareARun) {
    RegionHost host(hostOptions({amrAt("lead", 0.0, 0.0), amrAt("mate", 30.0, 0.0)}));
    ASSERT_TRUE(host.load().ok());
    ASSERT_EQ(host.regionCount(), 1U);
    const World& world = host.region(0).world();
    bool mate = false;
    for (const std::string& name : world.bodies.names) {
        mate = mate || name == "mate_base_link";
    }
    EXPECT_TRUE(mate);
    ASSERT_TRUE(host.stepUntil(SimTime{3'000'000'000}).ok());
    const std::array<double, 3> parked = host.robotPosition(1);
    EXPECT_NEAR(parked[0], 30.0, 0.05) << "the passive robot stays where it was put";
    EXPECT_NEAR(parked[1], 0.0, 0.05);
}

}  // namespace
}  // namespace lagrange::server
