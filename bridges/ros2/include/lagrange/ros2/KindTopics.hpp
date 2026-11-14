#pragma once

#include "lagrange/ros2/SignalTopics.hpp"
#include "lagrange/signals/SignalBus.hpp"
#include "lagrange/time/SimTime.hpp"
#include "lagrange/world/World.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace rclcpp {
class Node;
}

namespace lagrange::ros2 {

/// One topic a custom sensor publishes with no --ros-topics map (docs/design/domains/sensors.md
/// §9, "ROS topics of the built-in kinds"): `/<sensor>/<quantity>` on the standard message a real
/// driver of the device publishes, or for a quantity no standard message carries, std_msgs.
struct KindTopic {
    std::uint32_t sensor{0};  // index in World::Sensors
    std::string topic;        // "/altimeter/range"
    std::string type;         // "sensor_msgs/msg/Range"
    std::string frame;        // header.frame_id; "" for the unstamped std_msgs
    std::string childFrame;   // nav_msgs/Odometry child_frame_id
    /// The signals it carries. A new sample of the first sends it, stamped when it was sampled.
    std::vector<std::string> signals;
    /// Message fields the signals do not carry: Range radiation_type, field_of_view, min_range,
    /// max_range; LaserScan angle_min, angle_max, range_min, range_max, scan_time; BatteryState
    /// capacity (Ah); Odometry pose_covariance_0..5 and twist_covariance_0..5 (the diagonals).
    std::map<std::string, double, std::less<>> params;
    std::string text;  // JointState: the joint's name; BatteryState: the location
    /// Float64MultiArray: the dimensions' sizes, outermost first (rows, then columns); empty for
    /// one dimension of the signal's length.
    std::vector<std::uint32_t> layout;
};

/// The default topics of custom sensor `sensor` on `bus`, from the signals it writes. Typed
/// messages for what a kind's device publishes in ROS (Range, LaserScan, Odometry, BatteryState,
/// JointState, WrenchStamped, vision_msgs detections, ...); every other number the sensor writes
/// goes by its signal type and unit: Bool, Int64, Float64 (Temperature for K, FluidPressure for
/// Pa, RelativeHumidity and Illuminance by name), Float64MultiArray, Vector3Stamped, PoseStamped,
/// TwistStamped or WrenchStamped. Images, events and `delayed/` copies are left to the bridge.
[[nodiscard]] std::vector<KindTopic> kindTopics(const World& world, std::uint32_t sensor,
                                                const SignalBus& bus);

/// `topics` without those a --ros-topics map replaces: one on a topic the map publishes, or one
/// that carries a signal the map publishes. All of them when the map says `kind_topics: false`.
[[nodiscard]] std::vector<KindTopic> withoutMapped(std::vector<KindTopic> topics,
                                                   const TopicMaps& maps);

/// Publishes the default topics of custom sensors (KindTopic) on the bridge's node. Lazy: a
/// message is built only when someone subscribes, or when `Hooks::forced` says so.
class KindTopics {
public:
    struct Hooks {
        /// The topic rides on a radio link or a sensor delay: build it even with no subscriber,
        /// so that the random draws of the link and the delay do not change with who listens.
        std::function<bool(const std::string& topic)> forced;
        /// Sends (now or later) a message of about `bytes` on `topic`.
        std::function<void(const std::string& topic, std::function<void()> send, std::size_t bytes)>
            deliver;
        /// The region now minus the first region (m), added to positions in the world frame.
        std::array<double, 3> offset{};
    };

    explicit KindTopics(rclcpp::Node& node);
    ~KindTopics();
    KindTopics(const KindTopics&) = delete;
    KindTopics& operator=(const KindTopics&) = delete;
    KindTopics(KindTopics&&) = delete;
    KindTopics& operator=(KindTopics&&) = delete;

    /// Publishes `topics` from now on, finding their signals on `bus`. Publishers are kept across
    /// calls by topic (a new one would drop its subscribers). Returns what could not be made.
    std::vector<std::string> attach(std::vector<KindTopic> topics, const SignalBus& bus);
    /// Sends every topic whose first signal has a new sample.
    void publish(const SignalBus& bus, const Hooks& hooks);
    /// The topics attached, for tests and the graph.
    [[nodiscard]] const std::vector<KindTopic>& topics() const;

    struct Impl;

private:
    std::unique_ptr<Impl> impl_;
};

}  // namespace lagrange::ros2
