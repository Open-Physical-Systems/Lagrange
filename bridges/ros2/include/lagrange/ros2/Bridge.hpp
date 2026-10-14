#pragma once

#include "lagrange/signals/SignalBus.hpp"
#include "lagrange/time/SimTime.hpp"
#include "lagrange/world/World.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace lagrange::ros2 {

struct BridgeSettings {
    std::string nodeName{"lagrange_ros2_bridge"};
    std::string odomFrame{"odom"};
    std::string baseLink;          // the robot's root body; empty: no /odom and no odom TF
    std::string robotDescription;  // URDF text for /robot_description; empty: not published
    double stateRateHz{50.0};      // /joint_states, /odom and the odom TF
    bool stampedCmdVel{true};      // /cmd_vel as TwistStamped (Nav2's default) or Twist
};

/// A /cmd_vel message: forward speed and turn rate.
struct CmdVel {
    double linearMps{0.0};
    double angularRadps{0.0};
};

/// What the viewer's ROS 2 workspace shows (lagrange.viewer.v1.RosGraph).
struct GraphSnapshot {
    struct Topic {
        std::string name;
        std::string type;
        std::uint32_t publishers{0};
        std::uint32_t subscribers{0};
        double rateHz{0.0};        // measured for the bridge's own topics, 0 for others
        double bandwidthBps{0.0};  // bits per second, the bridge's own topics
        bool fromBridge{false};
    };
    struct Node {
        std::string name;  // "/robot_state_publisher"
        std::vector<std::string> publishes;
        std::vector<std::string> subscribes;
        bool external{true};
    };
    std::vector<Topic> topics;
    std::vector<Node> nodes;
    std::string rmw;
    std::string distro;
    std::uint32_t domainId{0};
};

/// The ROS 2 bridge (roadmap 0.2-F1, F2): an rclcpp node in the server process.
///
/// It reads the signal bus after the scheduler's ticks and publishes, stamped with simulation
/// time: /clock; /joint_states for every hinge and slide joint; /odom and the odom → base TF
/// from the base body's pose and twist (ground truth); /tf_static for sensor frames; /scan
/// (sensor_msgs/LaserScan) and /imu (sensor_msgs/Imu) when their sensors sample, named /scan
/// and /imu when the robot has one of each, /<sensor>/scan and /<sensor>/imu otherwise; and
/// /robot_description, latched. It subscribes /cmd_vel: geometry_msgs/TwistStamped, which Nav2
/// and ros2_control publish by default, or geometry_msgs/Twist (BridgeSettings::stampedCmdVel).
///
/// ROS callbacks run on the bridge's own executor thread; publish() runs on the caller's
/// thread between ticks and never blocks on the network (rclcpp publishing is asynchronous).
/// The design's per-consumer tick ring (architecture.md §10) replaces the direct bus reads when
/// the server gets its I/O threads.
class Bridge {
public:
    /// rclcpp must be initialised (rclcpp::init) before a bridge is made.
    explicit Bridge(BridgeSettings settings);
    ~Bridge();
    Bridge(const Bridge&) = delete;
    Bridge& operator=(const Bridge&) = delete;
    Bridge(Bridge&&) = delete;
    Bridge& operator=(Bridge&&) = delete;

    /// Resolves the signals to publish for this world on this bus. Call again after the
    /// scheduler is rebuilt (reset).
    void attach(const World& world, const SignalBus& bus);
    /// Publishes what changed up to `now`.
    void publish(const SignalBus& bus, SimTime now);
    /// The newest /cmd_vel if it arrived less than `maxAge` ago (wall clock).
    [[nodiscard]] std::optional<CmdVel> latestCommand(std::chrono::milliseconds maxAge) const;
    /// The ROS graph as this node sees it. Cold path: about once a second.
    [[nodiscard]] GraphSnapshot graph();
    [[nodiscard]] const std::string& nodeName() const { return settings_.nodeName; }

private:
    struct Impl;
    BridgeSettings settings_;
    std::unique_ptr<Impl> impl_;
};

}  // namespace lagrange::ros2
