#include "lagrange/server/Server.hpp"

#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

ServerOptions emptyRun() {
    ServerOptions options;
    options.robotPath.clear();  // --robot none
    options.realTimeFactor = 0.0;
    options.ros = false;
    return options;
}

Pose at(double x, double y) {
    Pose pose;
    pose.position = {x, y, 0.0};
    return pose;
}

bool hasBody(const Server& server, const std::string& name) {
    for (const std::string& body : server.world().bodies.names) {
        if (body == name) {
            return true;
        }
    }
    return false;
}

// A run starts with no robot (the ground only) and goes on; a robot spawned into it becomes the
// robot (unprefixed names, the demo drive moves it) and time does not start again.
TEST(Spawn, RobotSpawnedIntoAnEmptyRunIsTheRobot) {
    Server server(emptyRun());
    ASSERT_TRUE(server.load().ok());
    EXPECT_TRUE(server.world().bodies.names.empty());
    ASSERT_TRUE(server.stepUntil(SimTime{500'000'000}).ok());

    const auto name = server.spawnRobot(LAGRANGE_TEST_AMR_URDF, "", at(1.0, 2.0));
    ASSERT_TRUE(name) << name.error().message;
    EXPECT_EQ(*name, "/amr_diffdrive");
    EXPECT_EQ(server.now().ns, 500'000'000) << "time goes on from where it was";
    ASSERT_TRUE(hasBody(server, "base_link"));
    const std::vector<double> start = server.read("body/base_link/pose");
    ASSERT_EQ(start.size(), 7U);
    EXPECT_NEAR(start[0], 1.0, 1e-6);
    EXPECT_NEAR(start[1], 2.0, 1e-6);

    ASSERT_TRUE(server.stepUntil(SimTime{4'000'000'000}).ok());
    const std::vector<double> driven = server.read("body/base_link/pose");
    EXPECT_GT(std::hypot(driven[0] - 1.0, driven[1] - 2.0), 0.2) << "the demo drive moves it";
}

// Spawning a second robot and deleting it again leaves the first where it was: its pose and its
// wheels' angles are carried over the rebuild; the second is passive, its names prefixed.
TEST(Spawn, OthersKeepTheirPosesAcrossSpawnAndDelete) {
    ServerOptions options = emptyRun();
    options.robotPath = LAGRANGE_TEST_AMR_URDF;
    Server server(options);
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(server.stepUntil(SimTime{3'000'000'000}).ok());
    const std::vector<double> before = server.read("body/base_link/pose");
    const std::vector<double> wheel = server.read("joint/left_wheel_joint/position");
    ASSERT_EQ(wheel.size(), 1U);
    EXPECT_GT(std::abs(wheel[0]), 0.1) << "the wheel turned";

    const auto mate = server.spawnRobot(LAGRANGE_TEST_AMR_URDF, "mate", at(0.0, 5.0));
    ASSERT_TRUE(mate) << mate.error().message;
    EXPECT_EQ(*mate, "/mate");
    EXPECT_EQ(server.now().ns, 3'000'000'000);
    EXPECT_TRUE(hasBody(server, "mate_base_link"));
    const std::vector<double> after = server.read("body/base_link/pose");
    for (std::size_t i = 0; i < 7; ++i) {
        EXPECT_NEAR(after[i], before[i], 1e-6) << i;
    }
    EXPECT_NEAR(server.read("joint/left_wheel_joint/position")[0], wheel[0], 1e-9);

    EXPECT_FALSE(server.spawnRobot(LAGRANGE_TEST_AMR_URDF, "mate", at(0.0, -5.0)))
        << "names are unique";
    EXPECT_FALSE(server.spawnRobot(LAGRANGE_TEST_AMR_URDF, "2mate", at(0.0, -5.0)));
    EXPECT_FALSE(server.spawnRobot("/no/such/robot.urdf", "", at(0.0, -5.0)));
    EXPECT_TRUE(hasBody(server, "mate_base_link")) << "a refused spawn leaves the run";

    ASSERT_TRUE(server.stepUntil(SimTime{4'000'000'000}).ok());
    const std::vector<double> parked = server.read("body/mate_base_link/pose");
    EXPECT_NEAR(parked[0], 0.0, 0.05) << "the spawned robot is passive";
    EXPECT_NEAR(parked[1], 5.0, 0.05);

    const std::vector<double> moving = server.read("body/base_link/pose");
    ASSERT_TRUE(server.removeRobot("/mate").ok());
    EXPECT_FALSE(hasBody(server, "mate_base_link"));
    EXPECT_NEAR(server.read("body/base_link/pose")[0], moving[0], 1e-6);
    EXPECT_FALSE(server.removeRobot("mate").ok()) << "already gone";

    // The robot loaded at start can go too, leaving the ground.
    ASSERT_TRUE(server.removeRobot("amr_diffdrive").ok());
    EXPECT_TRUE(server.world().bodies.names.empty());
    ASSERT_TRUE(server.stepUntil(SimTime{4'500'000'000}).ok());
}

// A person spawned into an empty run is not the robot (RFC 0009 §8): the robot spawned next is,
// unprefixed, and is the first body; the person stands where asked and can be deleted.
TEST(Spawn, APersonIsNeverTheRobot) {
    Server server(emptyRun());
    ASSERT_TRUE(server.load().ok());
    const auto person = server.spawnRobot(LAGRANGE_TEST_ADULT_USDA, "ana", at(3.0, 0.0));
    ASSERT_TRUE(person) << person.error().message;
    EXPECT_EQ(*person, "/ana");
    ASSERT_TRUE(hasBody(server, "human_ana_pelvis"));
    ASSERT_TRUE(server.stepUntil(SimTime{500'000'000}).ok());
    const std::vector<double> pelvis = server.read("body/human_ana_pelvis/pose");
    ASSERT_EQ(pelvis.size(), 7U);
    EXPECT_NEAR(pelvis[0], 3.0, 1e-9);
    EXPECT_NEAR(pelvis[2], 0.53 * 1.75, 1e-9) << "standing, not falling";

    const auto robot = server.spawnRobot(LAGRANGE_TEST_AMR_URDF, "", at(0.0, 0.0));
    ASSERT_TRUE(robot) << robot.error().message;
    EXPECT_EQ(server.world().bodies.names.front(), "base_link") << "the robot comes first";
    EXPECT_NEAR(server.read("body/human_ana_pelvis/pose").at(0), 3.0, 1e-9);

    ASSERT_TRUE(server.removeRobot("/ana").ok());
    EXPECT_FALSE(hasBody(server, "human_ana_pelvis"));
    EXPECT_TRUE(hasBody(server, "base_link"));
}

}  // namespace
}  // namespace lagrange::server
