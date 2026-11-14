#include "lagrange/server/Server.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
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

// QA 2026-11-12: a pose that is not finite was taken (Python spawn(at=[nan, 0])) and broke the
// physics; a joint the run does not have, or a NaN, was "commanded" (True).
TEST(Spawn, RefusesAPoseThatIsNotFiniteAndCommandsToNoJoint) {
    ServerOptions options = emptyRun();
    options.robotPath = LAGRANGE_TEST_AMR_URDF;
    options.jointControl = true;
    Server server(options);
    ASSERT_TRUE(server.load().ok());
    EXPECT_FALSE(server.spawnRobot(LAGRANGE_TEST_AMR_URDF, "mate", at(std::nan(""), 0.0)));
    Pose tilted = at(0.0, 5.0);
    tilted.orientation[1] = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(server.spawnRobot(LAGRANGE_TEST_AMR_URDF, "mate", tilted));
    EXPECT_FALSE(hasBody(server, "mate_base_link"));

    EXPECT_FALSE(server.commandJoint("no_such_joint", "velocity", 1.0));
    EXPECT_FALSE(server.commandJoint("left_wheel_joint", "velocity", std::nan("")));
    EXPECT_TRUE(server.commandJoint("left_wheel_joint", "velocity", 1.0));
    ASSERT_TRUE(server.stepUntil(SimTime{500'000'000}).ok());
    for (const double x : server.read("body/base_link/pose")) {
        EXPECT_TRUE(std::isfinite(x));
    }
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

// QA 2026-11-12 #8: a spawn rebuilds the run, but a module whose signals are the same in the
// new run (the first robot's wheel odometry, its demo drive) goes on where it was instead of
// starting again: the odometry does not jump back to (0, 0).
TEST(Spawn, TheFirstRobotsSensorsKeepTheirStateAcrossASpawn) {
    ServerOptions options = emptyRun();
    options.robotPath =
        (std::filesystem::path(LAGRANGE_TEST_AMR_URDF).parent_path() / "amr_diffdrive_safety.urdf")
            .string();
    options.lidar = ServerOptions::Lidar::Cpu;
    Server server(options);
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.stepUntil(SimTime{3'000'000'000}).ok());
    const std::vector<double> before = server.read("sensor/wheel_odom/pose");
    ASSERT_EQ(before.size(), 7U);
    const double travelled = std::hypot(before[0], before[1]);
    ASSERT_GT(travelled, 0.2) << "the demo drive moved it";

    const auto mate = server.spawnRobot(LAGRANGE_TEST_AMR_URDF, "mate", at(0.0, 5.0));
    ASSERT_TRUE(mate) << mate.error().message;
    ASSERT_TRUE(server.stepUntil(SimTime{3'100'000'000}).ok());
    const std::vector<double> after = server.read("sensor/wheel_odom/pose");
    ASSERT_EQ(after.size(), 7U);
    EXPECT_NEAR(std::hypot(after[0] - before[0], after[1] - before[1]), 0.0, 0.1)
        << "odometry went on from (" << before[0] << ", " << before[1] << "), not from zero";

    // A spawn refused once the new run is built (here a sensor with a parameter out of range)
    // puts the run back as it was, its modules' state too.
    const std::filesystem::path bad =
        std::filesystem::temp_directory_path() / "lagrange_test_refused_spawn.urdf";
    std::ofstream(bad) << R"(<robot name="bad"><link name="base_link">
      <inertial><mass value="1"/><inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0"
        iyz="0"/></inertial>
      <collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision></link>
      <gazebo reference="base_link"><sensor name="bad_odom" type="wheel_odometry">
        <update_rate>50</update_rate><wheel_odometry><separation_error>-1</separation_error>
        </wheel_odometry></sensor></gazebo></robot>)";
    const auto refused = server.spawnRobot(bad.string(), "bad", at(0.0, -5.0));
    std::filesystem::remove(bad);
    ASSERT_FALSE(refused);
    ASSERT_TRUE(server.stepUntil(SimTime{3'200'000'000}).ok());
    const std::vector<double> later = server.read("sensor/wheel_odom/pose");
    ASSERT_EQ(later.size(), 7U);
    EXPECT_NEAR(std::hypot(later[0] - after[0], later[1] - after[1]), 0.0, 0.1)
        << "a refused spawn did not reset the odometry";
}

}  // namespace
}  // namespace lagrange::server
