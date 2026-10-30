#pragma once

#include "lagrange/module/Expected.hpp"
#include "lagrange/module/IModule.hpp"
#include "lagrange/signals/SignalBus.hpp"
#include "lagrange/time/SimTime.hpp"

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

/// One signal ↔ topic pair of `lagrange_server --ros-topics FILE.yaml` (RFC 0008 §4).
struct TopicMap {
    std::string signal;  // "sensor/front_tof/range"
    std::string topic;   // "/front_tof/range"
    std::string type;    // "sensor_msgs/msg/Range"
    std::string frame;   // the header's frame_id, for stamped messages
    /// Message fields that are not in the signal: Range min_range, max_range, field_of_view,
    /// radiation_type; LaserScan angle_min, angle_max, range_min, range_max.
    std::map<std::string, double, std::less<>> params;
    std::string unit{"1"};  // subscribe: the signal's unit, as the unit grammar writes it
    std::uint32_t size{1};  // subscribe Float64MultiArray: how many values
};

struct TopicMaps {
    std::vector<TopicMap> publish;    // signal → topic
    std::vector<TopicMap> subscribe;  // topic → signal
};

/// Reads a topic map file:
///
///     publish:
///       - {signal: sensor/front_tof/range, topic: /front_tof/range,
///          type: sensor_msgs/msg/Range, frame: front_tof, params: {max_range: 4.0}}
///     subscribe:
///       - {topic: /acme/valve, type: std_msgs/msg/Float64, signal: acme/valve/command}
///
/// Fails, naming the entry, for a missing key or a message type this bridge does not map.
[[nodiscard]] Expected<TopicMaps, std::string> readTopicMaps(const std::string& path);

/// The message types each direction maps.
[[nodiscard]] const std::vector<std::string>& publishTypes();
[[nodiscard]] const std::vector<std::string>& subscribeTypes();

/// Publishes mapped signals on their topics, and writes subscribed topics into signals through
/// a module of its own (domain "ros2"). Made by the bridge on its node; publish() runs on the
/// simulation thread, the subscriptions on the bridge's executor.
class SignalTopics {
public:
    SignalTopics(rclcpp::Node& node, TopicMaps maps);
    ~SignalTopics();
    SignalTopics(const SignalTopics&) = delete;
    SignalTopics& operator=(const SignalTopics&) = delete;
    SignalTopics(SignalTopics&&) = delete;
    SignalTopics& operator=(SignalTopics&&) = delete;

    /// Finds the published signals on a newly built bus. Returns what does not fit: an unknown
    /// signal, or one whose type, size or unit the message cannot carry.
    [[nodiscard]] std::vector<std::string> attach(const SignalBus& bus);
    /// Publishes every mapped signal committed since its last message, stamped `now`.
    void publish(const SignalBus& bus, SimTime now);
    /// The module that writes the subscribed topics' latest values (nullptr when nothing is
    /// subscribed). Make one for every run; the values outlive rebuilds.
    [[nodiscard]] std::unique_ptr<IModule> inputModule() const;

    struct Impl;

private:
    std::unique_ptr<Impl> impl_;
};

}  // namespace lagrange::ros2
