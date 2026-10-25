#pragma once

// Autoware's vehicle interface for a Lagrange road vehicle (docs/guide/autoware.md): the node
// Autoware talks to in place of a real car's drive-by-wire gateway.
//
//   Autoware                                   this node                 lagrange_server
//   /control/command/control_cmd      ──▶  steering angle, speed   ──▶  /ackermann_cmd
//   /control/command/gear_cmd         ──▶  forward, reverse, stop
//   /control/command/turn_indicators_cmd, hazard_lights_cmd ──▶ echoed as their reports
//   /vehicle/status/velocity_status   ◀──  /odom (body frame twist)
//   /vehicle/status/steering_status   ◀──  /joint_states (the steer joints, as one bicycle angle)
//   /vehicle/status/gear_status, control_mode (as /control/control_mode_request set it)
//   /localization/kinematic_state     ◀──  /odom (ground truth, in the map frame; optional)

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_vehicle_msgs/srv/control_mode_command.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace lagrange::autoware {

/// The bicycle model's road-wheel angle of an Ackermann pair: cot δ = (cot δ_l + cot δ_r) / 2
/// (rad, positive left).
[[nodiscard]] double bicycleAngle(double left, double right);

/// The speed sent for an Autoware velocity under a gear: zero in park and neutral, backwards in
/// the reverse gears, forwards in the drive and low gears (m/s).
[[nodiscard]] double speedForGear(double velocity, std::uint8_t gear);

/// Parameters: steer_joints (the left and right steer joints' names, required),
/// publish_kinematic_state (true: /localization/kinematic_state from the simulator's ground
/// truth, standing in for Autoware's localization), map_frame ("map"), base_frame ("base_link").
///
/// It starts in AUTONOMOUS (a simulated car has no driver). /control/control_mode_request
/// switches it: in MANUAL the control commands are not forwarded (the server's car controller
/// then stops the car after 0.5 s); the steer-only and velocity-only modes forward both.
class AutowareInterface final : public rclcpp::Node {
public:
    explicit AutowareInterface(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void onControl(const autoware_control_msgs::msg::Control& control);
    void onOdometry(const nav_msgs::msg::Odometry& odometry);
    void onJointStates(const sensor_msgs::msg::JointState& states);

    std::vector<std::string> steerJoints_;
    bool publishKinematicState_{true};
    std::string mapFrame_, baseFrame_;
    std::uint8_t gear_{autoware_vehicle_msgs::msg::GearCommand::DRIVE};
    std::uint8_t mode_{autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS};
    std::optional<double> left_, right_;  // steer joint positions (rad)

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gearReport_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr controlMode_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr turnReport_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::HazardLightsReport>::SharedPtr hazardReport_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr kinematicState_;
    rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr control_;
    rclcpp::Subscription<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr gearCommand_;
    rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr turn_;
    rclcpp::Subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr hazard_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_;
    rclcpp::Service<autoware_vehicle_msgs::srv::ControlModeCommand>::SharedPtr modeRequest_;
};

}  // namespace lagrange::autoware
