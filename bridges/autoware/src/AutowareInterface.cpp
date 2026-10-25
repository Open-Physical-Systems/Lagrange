#include "lagrange/autoware/AutowareInterface.hpp"

#include <cmath>
#include <stdexcept>

namespace lagrange::autoware {
namespace {

using autoware_vehicle_msgs::msg::GearCommand;

constexpr double kStraight = 1e-6;  // rad: below this a wheel counts as straight ahead

}  // namespace

double bicycleAngle(double left, double right) {
    // Straight wheels have an infinite cotangent; average the curvatures (tan) instead when
    // either is straight, which is the same limit.
    if (std::abs(left) < kStraight || std::abs(right) < kStraight) {
        return std::atan((std::tan(left) + std::tan(right)) / 2.0);
    }
    const double cot = (1.0 / std::tan(left) + 1.0 / std::tan(right)) / 2.0;
    return std::atan(1.0 / cot);
}

double speedForGear(double velocity, std::uint8_t gear) {
    if (gear == GearCommand::PARK || gear == GearCommand::NEUTRAL) {
        return 0.0;
    }
    if (gear == GearCommand::REVERSE || gear == GearCommand::REVERSE_2) {
        return -std::abs(velocity);
    }
    if (gear == GearCommand::NONE) {
        return velocity;  // no gear asked for: the velocity's own sign
    }
    return std::abs(velocity);  // the drive and low gears
}

AutowareInterface::AutowareInterface(const rclcpp::NodeOptions& options)
    : rclcpp::Node("lagrange_autoware_interface", options) {
    steerJoints_ =
        declare_parameter<std::vector<std::string>>("steer_joints", std::vector<std::string>{});
    if (steerJoints_.size() != 2) {
        throw std::invalid_argument(
            "lagrange_autoware_interface: steer_joints must name the left and right steer joints");
    }
    publishKinematicState_ = declare_parameter<bool>("publish_kinematic_state", true);
    mapFrame_ = declare_parameter<std::string>("map_frame", "map");
    baseFrame_ = declare_parameter<std::string>("base_frame", "base_link");

    const rclcpp::QoS reports(1);
    ackermann_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/ackermann_cmd", 10);
    velocity_ = create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
        "/vehicle/status/velocity_status", reports);
    steering_ = create_publisher<autoware_vehicle_msgs::msg::SteeringReport>(
        "/vehicle/status/steering_status", reports);
    gearReport_ = create_publisher<autoware_vehicle_msgs::msg::GearReport>(
        "/vehicle/status/gear_status", reports);
    controlMode_ = create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>(
        "/vehicle/status/control_mode", reports);
    turnReport_ = create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>(
        "/vehicle/status/turn_indicators_status", reports);
    hazardReport_ = create_publisher<autoware_vehicle_msgs::msg::HazardLightsReport>(
        "/vehicle/status/hazard_lights_status", reports);
    if (publishKinematicState_) {
        kinematicState_ =
            create_publisher<nav_msgs::msg::Odometry>("/localization/kinematic_state", reports);
    }

    control_ = create_subscription<autoware_control_msgs::msg::Control>(
        "/control/command/control_cmd", 1,
        [this](const autoware_control_msgs::msg::Control& c) { onControl(c); });
    gearCommand_ = create_subscription<GearCommand>(
        "/control/command/gear_cmd", 1, [this](const GearCommand& g) {
            gear_ = g.command;
            autoware_vehicle_msgs::msg::GearReport report;
            report.stamp = now();
            report.report = gear_;
            gearReport_->publish(report);
        });
    turn_ = create_subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>(
        "/control/command/turn_indicators_cmd", 1,
        [this](const autoware_vehicle_msgs::msg::TurnIndicatorsCommand& c) {
            if (c.command == c.NO_COMMAND) {
                return;
            }
            autoware_vehicle_msgs::msg::TurnIndicatorsReport report;
            report.stamp = now();
            report.report = c.command;  // the same codes
            turnReport_->publish(report);
        });
    hazard_ = create_subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>(
        "/control/command/hazard_lights_cmd", 1,
        [this](const autoware_vehicle_msgs::msg::HazardLightsCommand& c) {
            if (c.command == c.NO_COMMAND) {
                return;
            }
            autoware_vehicle_msgs::msg::HazardLightsReport report;
            report.stamp = now();
            report.report = c.command;
            hazardReport_->publish(report);
        });
    modeRequest_ = create_service<autoware_vehicle_msgs::srv::ControlModeCommand>(
        "/control/control_mode_request",
        [this](
            const std::shared_ptr<autoware_vehicle_msgs::srv::ControlModeCommand::Request>& in,
            const std::shared_ptr<autoware_vehicle_msgs::srv::ControlModeCommand::Response>& out) {
            // The request's codes are the report's for the modes it can ask for.
            out->success = in->mode != in->NO_COMMAND;
            if (out->success) {
                mode_ = in->mode;
            }
        });
    odometry_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, [this](const nav_msgs::msg::Odometry& o) { onOdometry(o); });
    joints_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, [this](const sensor_msgs::msg::JointState& j) { onJointStates(j); });
}

void AutowareInterface::onControl(const autoware_control_msgs::msg::Control& control) {
    if (mode_ == autoware_vehicle_msgs::msg::ControlModeReport::MANUAL) {
        return;
    }
    ackermann_msgs::msg::AckermannDriveStamped drive;
    drive.header.stamp = control.stamp;
    drive.header.frame_id = baseFrame_;
    drive.drive.steering_angle = control.lateral.steering_tire_angle;
    drive.drive.steering_angle_velocity = control.lateral.steering_tire_rotation_rate;
    drive.drive.speed = static_cast<float>(speedForGear(control.longitudinal.velocity, gear_));
    drive.drive.acceleration = control.longitudinal.acceleration;
    drive.drive.jerk = control.longitudinal.jerk;
    ackermann_->publish(drive);
}

void AutowareInterface::onOdometry(const nav_msgs::msg::Odometry& odometry) {
    // The simulator's /odom twist is in the body frame (child_frame_id), as REP 105 has it.
    autoware_vehicle_msgs::msg::VelocityReport velocity;
    velocity.header.stamp = odometry.header.stamp;
    velocity.header.frame_id = baseFrame_;
    velocity.longitudinal_velocity = static_cast<float>(odometry.twist.twist.linear.x);
    velocity.lateral_velocity = static_cast<float>(odometry.twist.twist.linear.y);
    velocity.heading_rate = static_cast<float>(odometry.twist.twist.angular.z);
    velocity_->publish(velocity);

    autoware_vehicle_msgs::msg::ControlModeReport mode;
    mode.stamp = odometry.header.stamp;
    mode.mode = mode_;
    controlMode_->publish(mode);

    autoware_vehicle_msgs::msg::GearReport gear;
    gear.stamp = odometry.header.stamp;
    gear.report = gear_;
    gearReport_->publish(gear);

    if (left_ && right_) {
        autoware_vehicle_msgs::msg::SteeringReport steering;
        steering.stamp = odometry.header.stamp;
        steering.steering_tire_angle = static_cast<float>(bicycleAngle(*left_, *right_));
        steering_->publish(steering);
    }
    if (kinematicState_) {
        nav_msgs::msg::Odometry state = odometry;
        state.header.frame_id = mapFrame_;
        state.child_frame_id = baseFrame_;
        kinematicState_->publish(state);
    }
}

void AutowareInterface::onJointStates(const sensor_msgs::msg::JointState& states) {
    for (std::size_t i = 0; i < states.name.size() && i < states.position.size(); ++i) {
        if (states.name[i] == steerJoints_[0]) {
            left_ = states.position[i];
        } else if (states.name[i] == steerJoints_[1]) {
            right_ = states.position[i];
        }
    }
}

}  // namespace lagrange::autoware
