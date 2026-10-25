// The Autoware interface against a stand-in for both sides (docs/guide/autoware.md): a node that
// publishes what Autoware's control and the simulator publish, and checks what comes out.
#include "lagrange/autoware/AutowareInterface.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <memory>
#include <optional>

#include <gtest/gtest.h>

namespace lagrange::autoware {
namespace {

using namespace std::chrono_literals;
using autoware_vehicle_msgs::msg::GearCommand;

TEST(Geometry, BicycleAngleOfAnAckermannPair) {
    // A 2.7 m wheelbase, 1.6 m track, turning left round a 10 m radius at the rear axle's
    // centre: the inner (left) wheel atan(L/(R−t/2)), the outer atan(L/(R+t/2)).
    const double inner = std::atan(2.7 / (10.0 - 0.8));
    const double outer = std::atan(2.7 / (10.0 + 0.8));
    EXPECT_NEAR(bicycleAngle(inner, outer), std::atan(2.7 / 10.0), 1e-12);
    EXPECT_NEAR(bicycleAngle(-inner, -outer), -std::atan(2.7 / 10.0), 1e-12);
    EXPECT_DOUBLE_EQ(bicycleAngle(0.0, 0.0), 0.0);
}

TEST(Geometry, SpeedForGear) {
    EXPECT_DOUBLE_EQ(speedForGear(3.0, GearCommand::DRIVE), 3.0);
    EXPECT_DOUBLE_EQ(speedForGear(-3.0, GearCommand::REVERSE), -3.0);
    EXPECT_DOUBLE_EQ(speedForGear(3.0, GearCommand::REVERSE), -3.0);
    EXPECT_DOUBLE_EQ(speedForGear(3.0, GearCommand::PARK), 0.0);
    EXPECT_DOUBLE_EQ(speedForGear(3.0, GearCommand::NEUTRAL), 0.0);
    EXPECT_DOUBLE_EQ(speedForGear(-1.0, GearCommand::NONE), -1.0);
}

class StandIn : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        rclcpp::NodeOptions options;
        options.parameter_overrides({rclcpp::Parameter(
            "steer_joints", std::vector<std::string>{"front_left_steer", "front_right_steer"})});
        interface_ = std::make_shared<AutowareInterface>(options);
        standIn_ = std::make_shared<rclcpp::Node>("stand_in");
        executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(interface_);
        executor_->add_node(standIn_);
    }
    void TearDown() override {
        executor_->remove_node(interface_);
        executor_->remove_node(standIn_);
        executor_.reset();
        interface_.reset();
        standIn_.reset();
        rclcpp::shutdown();
    }
    /// Spins until `done` or 10 s.
    bool spinUntil(const std::function<bool()>& done) {
        const auto end = std::chrono::steady_clock::now() + 10s;
        while (!done() && std::chrono::steady_clock::now() < end) {
            executor_->spin_some(50ms);
        }
        return done();
    }

    std::shared_ptr<AutowareInterface> interface_;
    std::shared_ptr<rclcpp::Node> standIn_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;  // after init
};

TEST_F(StandIn, ControlBecomesAnAckermannCommand) {
    std::optional<ackermann_msgs::msg::AckermannDriveStamped> got;
    auto sub = standIn_->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "/ackermann_cmd", 10,
        [&](const ackermann_msgs::msg::AckermannDriveStamped& m) { got = m; });
    auto gear = standIn_->create_publisher<GearCommand>("/control/command/gear_cmd", 1);
    auto control = standIn_->create_publisher<autoware_control_msgs::msg::Control>(
        "/control/command/control_cmd", 1);
    autoware_control_msgs::msg::Control c;
    c.lateral.steering_tire_angle = 0.2F;
    c.longitudinal.velocity = 4.0F;
    c.longitudinal.acceleration = 0.5F;
    ASSERT_TRUE(spinUntil([&] {
        control->publish(c);
        return got.has_value();
    }));
    EXPECT_FLOAT_EQ(got->drive.steering_angle, 0.2F);
    EXPECT_FLOAT_EQ(got->drive.speed, 4.0F);
    EXPECT_FLOAT_EQ(got->drive.acceleration, 0.5F);
    // In reverse the same velocity backs up (once the gear report says the gear is in).
    std::optional<std::uint8_t> geared;
    auto report = standIn_->create_subscription<autoware_vehicle_msgs::msg::GearReport>(
        "/vehicle/status/gear_status", 1,
        [&](const autoware_vehicle_msgs::msg::GearReport& m) { geared = m.report; });
    GearCommand g;
    g.command = GearCommand::REVERSE;
    ASSERT_TRUE(spinUntil([&] {
        gear->publish(g);
        return geared == GearCommand::REVERSE;
    }));
    got.reset();
    ASSERT_TRUE(spinUntil([&] {
        control->publish(c);
        return got.has_value();
    }));
    EXPECT_FLOAT_EQ(got->drive.speed, -4.0F);
}

TEST_F(StandIn, ManualModeStopsForwardingTheCommands) {
    using autoware_vehicle_msgs::srv::ControlModeCommand;
    int forwarded = 0;
    auto sub = standIn_->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "/ackermann_cmd", 10,
        [&](const ackermann_msgs::msg::AckermannDriveStamped&) { ++forwarded; });
    auto control = standIn_->create_publisher<autoware_control_msgs::msg::Control>(
        "/control/command/control_cmd", 1);
    auto client = standIn_->create_client<ControlModeCommand>("/control/control_mode_request");
    ASSERT_TRUE(spinUntil([&] { return client->service_is_ready(); }));
    auto request = std::make_shared<ControlModeCommand::Request>();
    request->mode = ControlModeCommand::Request::MANUAL;
    auto answer = client->async_send_request(request);
    ASSERT_TRUE(spinUntil(
        [&] { return answer.wait_for(std::chrono::seconds(0)) == std::future_status::ready; }));
    EXPECT_TRUE(answer.get()->success);
    ASSERT_TRUE(spinUntil([&] { return sub->get_publisher_count() > 0; }));
    for (int i = 0; i < 20; ++i) {
        control->publish(autoware_control_msgs::msg::Control{});
        executor_->spin_some(20ms);
    }
    EXPECT_EQ(forwarded, 0);
    // Back in AUTONOMOUS the same commands go through (so the silence above was the mode's).
    request->mode = ControlModeCommand::Request::AUTONOMOUS;
    auto again = client->async_send_request(request);
    ASSERT_TRUE(spinUntil(
        [&] { return again.wait_for(std::chrono::seconds(0)) == std::future_status::ready; }));
    ASSERT_TRUE(spinUntil([&] {
        control->publish(autoware_control_msgs::msg::Control{});
        return forwarded > 0;
    }));
}

TEST_F(StandIn, TheSimulatorsStateBecomesAutowaresReports) {
    std::optional<autoware_vehicle_msgs::msg::VelocityReport> velocity;
    std::optional<autoware_vehicle_msgs::msg::SteeringReport> steering;
    std::optional<autoware_vehicle_msgs::msg::ControlModeReport> mode;
    std::optional<nav_msgs::msg::Odometry> state;
    auto a = standIn_->create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
        "/vehicle/status/velocity_status", 1,
        [&](const autoware_vehicle_msgs::msg::VelocityReport& m) { velocity = m; });
    auto b = standIn_->create_subscription<autoware_vehicle_msgs::msg::SteeringReport>(
        "/vehicle/status/steering_status", 1,
        [&](const autoware_vehicle_msgs::msg::SteeringReport& m) { steering = m; });
    auto c = standIn_->create_subscription<autoware_vehicle_msgs::msg::ControlModeReport>(
        "/vehicle/status/control_mode", 1,
        [&](const autoware_vehicle_msgs::msg::ControlModeReport& m) { mode = m; });
    auto d = standIn_->create_subscription<nav_msgs::msg::Odometry>(
        "/localization/kinematic_state", 1, [&](const nav_msgs::msg::Odometry& m) { state = m; });
    auto odom = standIn_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    auto joints = standIn_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    nav_msgs::msg::Odometry o;
    o.header.frame_id = "odom";
    o.child_frame_id = "base_link";
    o.pose.pose.position.x = 12.0;
    o.twist.twist.linear.x = 5.0;
    o.twist.twist.angular.z = 0.3;
    sensor_msgs::msg::JointState j;
    j.name = {"front_left_spin", "front_left_steer", "front_right_steer"};
    j.position = {1.0, 0.25, 0.21};
    ASSERT_TRUE(spinUntil([&] {
        joints->publish(j);
        odom->publish(o);
        return velocity && steering && mode && state;
    }));
    EXPECT_FLOAT_EQ(velocity->longitudinal_velocity, 5.0F);
    EXPECT_FLOAT_EQ(velocity->heading_rate, 0.3F);
    EXPECT_NEAR(steering->steering_tire_angle, bicycleAngle(0.25, 0.21), 1e-6);
    EXPECT_GT(steering->steering_tire_angle, 0.21F);
    EXPECT_LT(steering->steering_tire_angle, 0.25F);
    EXPECT_EQ(mode->mode, autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS);
    EXPECT_EQ(state->header.frame_id, "map");
    EXPECT_DOUBLE_EQ(state->pose.pose.position.x, 12.0);
}

}  // namespace
}  // namespace lagrange::autoware
