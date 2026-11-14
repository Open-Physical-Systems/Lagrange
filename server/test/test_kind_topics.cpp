// The built-in kinds on ROS with no --ros-topics map (sensors.md §9 "ROS topics of the built-in
// kinds"), end to end: the sample AMR with its safety sensors, on ROS_DOMAIN_ID 77.
#include "lagrange/server/Server.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/bool.hpp>

namespace lagrange::server {
namespace {

using namespace std::chrono_literals;

TEST(KindTopics, TheSafetyAmrsKindsPublishWithNoMap) {
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    const std::filesystem::path dir =
        std::filesystem::path(::testing::TempDir()) / "lagrange_kind_topics";
    std::filesystem::create_directories(dir);
    ServerOptions options;
    options.endpoint = proto::Endpoint::unixSocket((dir / "s.sock").string());
    options.robotPath = LAGRANGE_ASSETS "/robots/amr_diffdrive/amr_diffdrive_safety.urdf";
    options.worldPath.clear();
    options.overlayPath.clear();
    options.ros = true;
    options.realTimeFactor = 0.0;
    options.lidar = ServerOptions::Lidar::Cpu;
    Server server(options);
    const Status started = server.start();
    ASSERT_TRUE(started.ok()) << started.message;

    auto node = std::make_shared<rclcpp::Node>("kind_topics_test");
    std::optional<sensor_msgs::msg::Range> cliff;
    std::optional<sensor_msgs::msg::LaserScan> safety;
    std::optional<nav_msgs::msg::Odometry> odom;
    std::optional<sensor_msgs::msg::JointState> torque;
    std::optional<std_msgs::msg::Bool> bumper, protective;
    auto a = node->create_subscription<sensor_msgs::msg::Range>(
        "/cliff_front_left/range", 10, [&](const sensor_msgs::msg::Range& m) { cliff = m; });
    auto b = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "/safety/scan", 10, [&](const sensor_msgs::msg::LaserScan& m) { safety = m; });
    auto c = node->create_subscription<nav_msgs::msg::Odometry>(
        "/wheel_odom/odom", 10, [&](const nav_msgs::msg::Odometry& m) { odom = m; });
    auto d = node->create_subscription<sensor_msgs::msg::JointState>(
        "/left_wheel_torque/joint_state", 10,
        [&](const sensor_msgs::msg::JointState& m) { torque = m; });
    auto e = node->create_subscription<std_msgs::msg::Bool>(
        "/front_bumper/pressed", 10, [&](const std_msgs::msg::Bool& m) { bumper = m; });
    auto f = node->create_subscription<std_msgs::msg::Bool>(
        "/safety/protective", 10, [&](const std_msgs::msg::Bool& m) { protective = m; });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    const auto deadline = std::chrono::steady_clock::now() + 60s;
    // Until the robot has settled on the ground: the cliff sensor then sees the floor.
    while (!(cliff && std::isfinite(cliff->range) && safety && odom && torque && bumper &&
             protective) &&
           std::chrono::steady_clock::now() < deadline) {
        server.runOnce(5);
        executor.spin_some(10ms);
    }

    // Every kind on the robot shows in the graph, under /<sensor>/<quantity>.
    const std::map<std::string, std::vector<std::string>> graph = node->get_topic_names_and_types();
    for (const char* topic :
         {"/front_bumper/pressed", "/rear_bumper/pressed", "/front_bumper/force",
          "/cliff_front_left/range", "/cliff_rear_right/cliff", "/wheel_odom/odom", "/safety/scan",
          "/safety/protective", "/safety/warning", "/safety/min_distance",
          "/left_wheel_torque/joint_state", "/right_wheel_torque/saturated"}) {
        EXPECT_TRUE(graph.contains(topic)) << topic;
    }
    ASSERT_TRUE(cliff) << "no Range";
    EXPECT_EQ(cliff->header.frame_id, "cliff_front_left");
    EXPECT_GT(cliff->range, 0.03F) << "the floor, 5.5 cm down";
    EXPECT_LT(cliff->range, 0.08F);
    ASSERT_TRUE(safety) << "no LaserScan";
    EXPECT_NEAR(safety->angle_min, -3.316F / 2, 1e-5);
    EXPECT_GT(safety->ranges.size(), 1000U) << "0.17° steps over 190°";
    ASSERT_TRUE(odom) << "no Odometry";
    EXPECT_EQ(odom->header.frame_id, "odom");
    EXPECT_EQ(odom->child_frame_id, "base_link");
    ASSERT_TRUE(torque) << "no JointState";
    ASSERT_EQ(torque->name.size(), 1U);
    EXPECT_EQ(torque->name[0], "left_wheel_joint");
    ASSERT_TRUE(bumper && protective);
    EXPECT_FALSE(bumper->data) << "standing alone";
}

}  // namespace
}  // namespace lagrange::server
