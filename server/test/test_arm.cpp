// The sample arm (roadmap v0.3 arm integration): arm6 with its motors on the 48 V bus, driven by
// joint_trajectory_controller at 1 kHz through the drives' position servos, moves to a pose and
// holds it against gravity.
#include "lagrange/server/Server.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <future>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace lagrange::server {
namespace {

using namespace std::chrono_literals;

const std::vector<std::string> kJoints{"shoulder_pan_joint", "shoulder_lift_joint",
                                       "elbow_joint",        "wrist_1_joint",
                                       "wrist_2_joint",      "wrist_3_joint"};

TEST(Arm6, TheTrajectoryControllerMovesTheArmAndHoldsItAgainstGravity) {
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    const std::filesystem::path dir = std::filesystem::path(::testing::TempDir()) / "lagrange_arm";
    std::filesystem::create_directories(dir);
    ServerOptions options;
    options.endpoint = proto::Endpoint::unixSocket((dir / "arm.sock").string());
    options.robotPath = LAGRANGE_TEST_ARM6_URDF;
    options.worldPath = LAGRANGE_TEST_WORKCELL_URDF;
    options.overlayPath = LAGRANGE_TEST_ARM6_OVERLAY;
    options.ros2ControlParams = LAGRANGE_TEST_ARM6_CONTROLLERS;
    options.ros = true;
    options.realTimeFactor = 1.0;
    Server server(options);
    const Status started = server.start();
    ASSERT_TRUE(started.ok()) << started.message;
    EXPECT_NE(server.wiringDot().find("electrical/averaged_network"), std::string::npos);

    auto spawned = std::async(std::launch::async, [] {
        const std::string command = std::string(
                                        "ros2 run controller_manager spawner "
                                        "joint_state_broadcaster arm_controller "
                                        "gripper_controller --controller-manager-timeout 30 "
                                        "--param-file ") +
                                    LAGRANGE_TEST_ARM6_CONTROLLERS + " > /dev/null 2>&1";
        return std::system(command.c_str());
    });
    const auto deadline = std::chrono::steady_clock::now() + 90s;
    while (spawned.wait_for(0s) != std::future_status::ready &&
           std::chrono::steady_clock::now() < deadline) {
        server.runOnce(5);
    }
    ASSERT_EQ(spawned.get(), 0) << "the spawner failed";

    const std::vector<double> target{0.4, -0.5, 1.0, 0.3, 0.6, -0.4};
    auto node = std::make_shared<rclcpp::Node>("arm6_test");
    auto publisher = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/arm_controller/joint_trajectory", rclcpp::SystemDefaultsQoS());
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = kJoints;
    trajectory.points.resize(1);
    trajectory.points[0].positions = target;
    trajectory.points[0].time_from_start = rclcpp::Duration(3s);
    // Publish until the controller has it (its subscription may still be connecting).
    const auto subscribed = std::chrono::steady_clock::now() + 20s;
    while (publisher->get_subscription_count() == 0 &&
           std::chrono::steady_clock::now() < subscribed) {
        server.runOnce(5);
    }
    publisher->publish(trajectory);
    const std::int64_t sent = server.simTime().ns;
    while (server.simTime().ns < sent + 5'000'000'000) {  // 3 s to move, 2 s to hold
        server.runOnce(5);
        ASSERT_TRUE(server.running());
    }
    for (std::size_t j = 0; j < kJoints.size(); ++j) {
        EXPECT_NEAR(server.read("joint/" + kJoints[j] + "/position").at(0), target[j], 0.02)
            << kJoints[j];
    }
    // In this pose the elbow holds the forearm's weight (about 8.5 N·m): its motor carries
    // current. Lowering the arm regenerates into the bus capacitor, which the 48 V supply does
    // not take back: the bus floats above the supply (below the brake resistor's 56 V) and the
    // supply never charges.
    EXPECT_GT(std::abs(server.read("motor/elbow_motor/current_q").at(0)), 0.5);
    const double bus = server.read("bus/dc_48v/voltage").at(0);
    EXPECT_GE(bus, 47.5);
    EXPECT_LE(bus, 56.0);
    EXPECT_GE(server.read("battery/supply/current").at(0), 0.0);
}

}  // namespace
}  // namespace lagrange::server
