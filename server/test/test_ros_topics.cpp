// --ros-topics (RFC 0008 §4): a plugin's signal on a ROS topic, and a topic into a signal.
#include "lagrange/server/Server.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <string>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace lagrange::server {
namespace {

using namespace std::chrono_literals;

std::filesystem::path directory() {
    const std::filesystem::path dir =
        std::filesystem::path(::testing::TempDir()) / "lagrange_ros_topics";
    std::filesystem::create_directories(dir);
    return dir;
}

ServerOptions tofBox(const std::string& topics) {
    const std::filesystem::path dir = directory();
    const std::filesystem::path map = dir / "topics.yaml";
    std::ofstream(map) << topics;
    ServerOptions options;
    options.endpoint = proto::Endpoint::unixSocket((dir / "s.sock").string());
    options.robotPath = LAGRANGE_TEST_DATA "/tof_box.urdf";
    options.worldPath.clear();
    options.overlayPath.clear();
    options.plugins = {"lagrange_tof"};
    options.rosTopicsPath = map.string();
    options.ros = true;
    options.realTimeFactor = 0.0;
    return options;
}

TEST(RosTopics, APluginsSignalIsPublishedAndATopicBecomesASignal) {
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    Server server(tofBox(R"(publish:
  - {signal: sensor/front_tof/range, topic: /front_tof/range, type: sensor_msgs/msg/Range,
     frame: front_tof, params: {min_range: 0.02, max_range: 4.0}}
subscribe:
  - {topic: /acme/valve, type: std_msgs/msg/Float64, signal: acme/valve/command}
)"));
    const Status started = server.start();
    ASSERT_TRUE(started.ok()) << started.message;

    auto node = std::make_shared<rclcpp::Node>("ros_topics_test");
    std::optional<sensor_msgs::msg::Range> received;
    auto subscription = node->create_subscription<sensor_msgs::msg::Range>(
        "/front_tof/range", 10,
        [&received](const sensor_msgs::msg::Range& message) { received = message; });
    auto valve = node->create_publisher<std_msgs::msg::Float64>("/acme/valve", 10);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std_msgs::msg::Float64 command;
    command.data = 2.5;
    const auto deadline = std::chrono::steady_clock::now() + 60s;
    while ((!received || server.read("acme/valve/command").at(0) != 2.5) &&
           std::chrono::steady_clock::now() < deadline) {
        server.runOnce(5);
        if (valve->get_subscription_count() > 0) {
            valve->publish(command);
        }
        executor.spin_some(10ms);
    }
    ASSERT_TRUE(received) << "no Range message";
    EXPECT_EQ(received->header.frame_id, "front_tof");
    EXPECT_EQ(received->max_range, 4.0F);
    EXPECT_GT(received->range, 0.1F) << "the ground, ahead and below";
    EXPECT_LT(received->range, 1.0F);
    EXPECT_EQ(server.read("acme/valve/command").at(0), 2.5) << "the topic wrote the signal";
}

// RFC 0009 §6: a person's goal over ROS: x, y, active, through --ros-topics.
TEST(RosTopics, APersonTakesAGoalFromATopic) {
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    const std::filesystem::path dir = directory();
    const std::filesystem::path map = dir / "goal.yaml";
    std::ofstream(map)
        << "subscribe:\n  - {topic: /ana/goal, type: std_msgs/msg/Float64MultiArray, "
           "signal: human/ana/goal, size: 3}\n";
    const std::filesystem::path world = dir / "ana.usda";
    std::ofstream(world)
        << "#usda 1.0\n(\n    metersPerUnit = 1\n    upAxis = \"Z\"\n)\n"
           "def Xform \"ana\" (\n    prepend apiSchemas = [\"HumanAPI_1\"]\n)\n{\n}\n";
    ServerOptions options;
    options.endpoint = proto::Endpoint::unixSocket((dir / "g.sock").string());
    options.worldPath = world.string();
    options.rosTopicsPath = map.string();
    options.ros = true;
    options.realTimeFactor = 0.0;
    Server server(options);
    const Status started = server.start();
    ASSERT_TRUE(started.ok()) << started.message;
    auto node = std::make_shared<rclcpp::Node>("ros_goal_test");
    auto goal = node->create_publisher<std_msgs::msg::Float64MultiArray>("/ana/goal", 10);
    std_msgs::msg::Float64MultiArray command;
    command.data = {0.0, 3.0, 1.0};
    const auto deadline = std::chrono::steady_clock::now() + 60s;
    while (server.read("human/ana/pose").at(1) < 2.5 &&
           std::chrono::steady_clock::now() < deadline) {
        server.runOnce(5);
        if (goal->get_subscription_count() > 0) {
            goal->publish(command);
        }
    }
    EXPECT_GT(server.read("human/ana/pose").at(1), 2.5) << "the person walked to the goal";
}

TEST(RosTopics, ASignalThatDoesNotFitItsMessageStopsTheStart) {
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    Server unknown(tofBox(
        "publish:\n  - {signal: no/such/signal, topic: /unknown, type: std_msgs/msg/Float64}\n"));
    const Status missing = unknown.start();
    ASSERT_FALSE(missing.ok());
    EXPECT_NE(missing.message.find("no signal no/such/signal"), std::string::npos)
        << missing.message;

    Server wrong(
        tofBox("publish:\n  - {signal: body/base_link/pose, topic: /unfit, "
               "type: sensor_msgs/msg/Range}\n"));
    const Status unfit = wrong.start();
    ASSERT_FALSE(unfit.ok());
    EXPECT_NE(unfit.message.find("has 7 values"), std::string::npos) << unfit.message;
}

}  // namespace
}  // namespace lagrange::server
