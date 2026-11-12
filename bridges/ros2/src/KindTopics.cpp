// The default ROS topics of custom sensors, built-in kinds first (docs/design/domains/sensors.md
// §9, "ROS topics of the built-in kinds"): what a real driver of the device publishes, with no
// --ros-topics map.
#include "lagrange/ros2/KindTopics.hpp"
#include "lagrange/signals/Signal.hpp"
#include "lagrange/signals/Unit.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <set>
#include <span>
#include <sstream>
#include <string_view>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/illuminance.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int64.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

namespace lagrange::ros2 {

namespace {

constexpr double kZeroCelsiusK = 273.15;        // K at 0 °C (Temperature, BatteryState: °C)
constexpr double kCoulombsPerAmpHour = 3600.0;  // C per Ah (BatteryState charge: Ah)
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr std::uint8_t kUltrasound = sensor_msgs::msg::Range::ULTRASOUND;
constexpr std::uint8_t kInfrared = sensor_msgs::msg::Range::INFRARED;  // light: laser, LED, IR
constexpr std::size_t kDetectionFields = 16;  // a detections row (sensors.md §9 "detections")
/// Odometry's covariance diagonals (x, y, z, roll, pitch, yaw) when the sensor gives none:
/// illustrative for a planar robot; 1e6 marks what wheels do not measure (z, roll, pitch), as
/// robot_localization reads it. Set them with `pose_covariance_diagonal` and
/// `twist_covariance_diagonal` (six numbers, ros2_control diff_drive_controller's names).
constexpr std::array<double, 6> kPoseCovariance{1e-3, 1e-3, 1e6, 1e6, 1e6, 1e-2};
constexpr std::array<double, 6> kTwistCovariance{1e-3, 1e-3, 1e6, 1e6, 1e6, 1e-2};

// The kinds' own defaults, for the message fields their signals do not carry; keep in step with
// modules/sensors/src/kinds/ (the constant named after each).
constexpr double kRangefinderMin = 0.1;   // m, Rangefinder.cpp kMinRange
constexpr double kRangefinderMax = 12.0;  // m, Rangefinder.cpp kMaxRange
constexpr double kCliffMin = 0.005;       // m, Cliff.cpp kDefaultMinRange
constexpr double kCliffMax = 0.15;        // m, Cliff.cpp kDefaultMaxRange
constexpr double kProximityFov = 0.44;    // rad, Proximity.cpp kDefaultFovRad
constexpr double kProximityMax = 0.3;     // m, Proximity.cpp kDefaultMaxRangeM
constexpr double kSonarMin = 0.1;         // m, Sonar.cpp kMinRange
constexpr double kSonarMax = 120.0;       // m, Sonar.cpp kMaxRange
constexpr double kSonarFov = 130.0 * std::numbers::pi / 180.0;       // rad, Sonar.cpp kFovH
constexpr double kSonarBeamWidth = 25.0 * std::numbers::pi / 180.0;  // rad, kSingleBeamWidth
constexpr double kScannerFov = 275.0 * std::numbers::pi / 180.0;     // rad, SafetyScanner.cpp
constexpr double kScannerMax = 40.0;  // m, SafetyScanner.cpp kNans3Range

builtin_interfaces::msg::Time stampOf(SimTime t) {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<std::int32_t>(t.ns / 1'000'000'000);
    stamp.nanosec = static_cast<std::uint32_t>(t.ns % 1'000'000'000);
    return stamp;
}

/// The ROS frame of a signal's frame (SignalSpec::frame) for sensor `name` on body `body`.
std::string rosFrame(const std::string& frame, const std::string& name, const std::string& body) {
    if (frame.empty() || frame == "sensor") {
        return name;
    }
    if (frame == "optical") {
        return name + "_optical";
    }
    if (frame == "body") {
        return body;
    }
    if (frame == "world_local" || frame == "world") {
        return "world";
    }
    for (const std::string_view prefix : {"sensor:", "body:"}) {
        if (frame.starts_with(prefix)) {
            return frame.substr(prefix.size());
        }
    }
    return frame;  // "odom"
}

/// Six numbers of a text parameter ("0.001 0.001 1e6 1e6 1e6 0.01"), else `fallback`.
std::array<double, 6> diagonal(const World& world, std::uint32_t sensor, std::string_view key,
                               const std::array<double, 6>& fallback) {
    const auto& text = world.sensors.text[sensor];
    const auto it = text.find(key);
    if (it == text.end()) {
        return fallback;
    }
    std::istringstream in(it->second);
    std::array<double, 6> out{};
    for (double& value : out) {
        if (!(in >> value)) {
            return fallback;
        }
    }
    return out;
}

/// The joint a joint_torque sensor reads: its `joint`, else the joint that moves its body.
std::string jointOf(const World& world, std::uint32_t sensor) {
    const auto& text = world.sensors.text[sensor];
    if (const auto it = text.find("joint"); it != text.end()) {
        return it->second;
    }
    for (std::size_t j = 0; j < world.joints.names.size(); ++j) {
        if (world.joints.child[j].index == world.sensors.body[sensor].index) {
            return world.joints.names[j];
        }
    }
    return {};
}

}  // namespace

std::vector<KindTopic> kindTopics(const World& world, std::uint32_t sensor, const SignalBus& bus) {
    const std::string& name = world.sensors.names[sensor];
    const std::string& kind = world.sensors.kind[sensor];
    const std::string body = world.bodies.names[world.sensors.body[sensor].index];
    const auto& settings = world.sensors.settings[sensor];
    const auto get = [&settings](std::string_view key, double fallback) {
        const auto it = settings.find(key);
        return it == settings.end() ? fallback : it->second;
    };
    const std::string prefix = "sensor/" + name + "/";

    // What the sensor writes, by quantity ("range", "fault/active").
    std::map<std::string, const SignalSpec*, std::less<>> written;
    std::set<std::string, std::less<>> images;
    for (std::size_t k = 0; k < bus.signalCount(); ++k) {
        const SignalSpec& spec = bus.specAt(k);
        if (!spec.name.starts_with(prefix)) {
            continue;
        }
        std::string quantity = spec.name.substr(prefix.size());
        if (quantity.empty() || quantity.starts_with("delayed/")) {
            continue;  // the bridge delays the messages itself (sensor timing)
        }
        if (spec.type == SignalType::Image) {
            images.insert(quantity);
        }
        written.emplace(std::move(quantity), &spec);
    }
    const auto has = [&written](std::string_view quantity) { return written.contains(quantity); };
    const auto length = [&written](std::string_view quantity) -> std::uint32_t {
        const auto it = written.find(quantity);
        return it == written.end() ? 0U
                                   : static_cast<std::uint32_t>(
                                         componentCount(it->second->type, it->second->arrayLength));
    };

    std::vector<KindTopic> out;
    std::set<std::string, std::less<>> used;  // quantities a typed message carries
    const auto typed = [&](const std::string& quantity, const std::string& type,
                           std::vector<std::string> carried) -> KindTopic& {
        KindTopic topic;
        topic.sensor = sensor;
        topic.topic = "/" + name + "/" + quantity;
        topic.type = type;
        topic.frame = name;
        for (const std::string& q : carried) {
            topic.signals.push_back(prefix + q);
            used.insert(q);
        }
        out.push_back(std::move(topic));
        return out.back();
    };
    const auto range = [&](const std::string& quantity, const std::string& signal,
                           std::uint8_t radiation, double fov, double minRange, double maxRange) {
        KindTopic& topic = typed(quantity, "sensor_msgs/msg/Range", {signal});
        topic.params = {{"radiation_type", radiation},
                        {"field_of_view", fov},
                        {"min_range", minRange},
                        {"max_range", maxRange}};
    };

    // Typed messages: what the kind's real device publishes in ROS.
    if (kind == "rangefinder" && has("range")) {
        range("range", "range", kInfrared, 0.0, get("range/min", kRangefinderMin),
              get("range/max", kRangefinderMax));
    } else if (kind == "cliff" && has("range")) {
        const double fov = get("rays", 1.0) > 1.0 ? 2.0 * get("beam_angle", 0.0) : 0.0;
        range("range", "range", kInfrared, fov, get("min_range", kCliffMin),
              get("max_range", kCliffMax));
    } else if (kind == "proximity" && has("min_distance")) {
        range("range", "min_distance", kInfrared, get("fov", kProximityFov), get("min_range", 0.0),
              get("max_range", kProximityMax));
        // The array keeps its rows and columns.
        used.erase("min_distance");
    } else if (kind == "sonar" && has("ranges")) {
        const double minRange = get("range/min", kSonarMin);
        const double maxRange = get("range/max", kSonarMax);
        if (length("ranges") == 1) {  // a single-beam echo sounder or altimeter
            range("range", "ranges", kUltrasound, get("beam_width", kSonarBeamWidth), minRange,
                  maxRange);
        } else {
            // Beam 0 is the left one (+y): the angles run from +fov/2 down to −fov/2.
            const double fov = get("horizontal_fov", kSonarFov);
            std::vector<std::string> carried{"ranges"};
            if (has("intensities")) {
                carried.emplace_back("intensities");
            }
            KindTopic& topic = typed("scan", "sensor_msgs/msg/LaserScan", carried);
            topic.params = {{"angle_min", fov / 2.0},
                            {"angle_max", -fov / 2.0},
                            {"range_min", minRange},
                            {"range_max", maxRange}};
        }
    } else if (kind == "safety_scanner" && has("ranges")) {
        const double fov = get("fov", kScannerFov);
        KindTopic& topic = typed("scan", "sensor_msgs/msg/LaserScan", {"ranges"});
        topic.params = {{"angle_min", -fov / 2.0},
                        {"angle_max", fov / 2.0},
                        {"range_min", get("min_range", 0.0)},
                        {"range_max", get("max_range", kScannerMax)},
                        {"scan_time", 1.0 / std::max(world.sensors.rateHz[sensor], 1e-3)}};
    } else if (kind == "wheel_odometry" && has("pose") && has("twist")) {
        KindTopic& topic = typed("odom", "nav_msgs/msg/Odometry", {"pose", "twist"});
        topic.frame = "odom";
        topic.childFrame = body;
        const std::array<double, 6> pose =
            diagonal(world, sensor, "pose_covariance_diagonal", kPoseCovariance);
        const std::array<double, 6> twist =
            diagonal(world, sensor, "twist_covariance_diagonal", kTwistCovariance);
        for (std::size_t k = 0; k < 6; ++k) {
            topic.params["pose_covariance_" + std::to_string(k)] = pose.at(k);
            topic.params["twist_covariance_" + std::to_string(k)] = twist.at(k);
        }
        used.insert("yaw");  // in the pose
    } else if (kind == "battery_monitor" && has("voltage")) {
        std::vector<std::string> carried{"voltage"};
        for (const char* q : {"current", "soc", "temperature", "remaining_capacity", "valid"}) {
            carried.emplace_back(has(q) ? q : "");
        }
        KindTopic& topic = typed("battery_state", "sensor_msgs/msg/BatteryState", {});
        for (const std::string& q : carried) {
            topic.signals.push_back(q.empty() ? std::string{} : prefix + q);
            used.insert(q);
        }
        topic.params["capacity"] = get("capacity", kNaN);  // Ah
        const auto& text = world.sensors.text[sensor];
        const auto battery = text.find("battery");
        topic.text = battery == text.end() ? std::string{} : battery->second;
    } else if (kind == "joint_torque" && (has("torque") || has("force"))) {
        KindTopic& topic = typed("joint_state", "sensor_msgs/msg/JointState",
                                 {has("torque") ? "torque" : "force"});
        topic.text = jointOf(world, sensor);
    } else if (kind == "force_torque" && has("wrench")) {
        typed("wrench", "geometry_msgs/msg/WrenchStamped", {"wrench"}).frame =
            rosFrame(written.at("wrench")->frame, name, body);
        used.insert("force");
        used.insert("torque");
    } else if (kind == "optical_flow" && length("flow") == 2) {
        // About the optical frame's x and y, PX4's sign (SensorOpticalFlow.pixel_flow); z is 0.
        typed("flow", "geometry_msgs/msg/Vector3Stamped", {"flow"}).frame = name + "_optical";
    } else if (kind == "detections" && has("detections") &&
               length("detections") % kDetectionFields == 0) {
        const auto& text = world.sensors.text[sensor];
        const auto camera = text.find("camera");
        const std::string frame = (camera == text.end() ? name : camera->second) + "_optical";
        std::vector<std::string> carried{"detections"};
        if (has("count")) {
            carried.emplace_back("count");
        }
        typed("detections", "vision_msgs/msg/Detection2DArray", carried).frame = frame;
        typed("detections_3d", "vision_msgs/msg/Detection3DArray", carried).frame = frame;
        used.erase("count");  // also on its own
    }

    // Every other number by its type and unit.
    std::set<std::string, std::less<>> topicsTaken;
    for (const KindTopic& topic : out) {
        topicsTaken.insert(topic.topic);
    }
    for (const auto& [quantity, spec] : written) {
        if (used.contains(quantity) || spec->type == SignalType::Image || isHandle(spec->type)) {
            continue;
        }
        // An image's calibration and a stereo pair's offset belong to its image (camera_info).
        const auto slash = quantity.rfind('/');
        if (slash != std::string::npos && images.contains(quantity.substr(0, slash))) {
            continue;
        }
        if (quantity == "events" && spec->type == SignalType::ScalarArray &&
            spec->arrayLength % 4 == 0) {
            continue;  // the bridge's /<name>/events point cloud
        }
        KindTopic topic;
        topic.sensor = sensor;
        topic.topic = "/" + name + "/" + quantity;
        if (topicsTaken.contains(topic.topic)) {
            continue;
        }
        topic.signals = {spec->name};
        topic.frame = rosFrame(spec->frame, name, body);
        const std::string leaf = slash == std::string::npos ? quantity : quantity.substr(slash + 1);
        const bool velocity = spec->unit.dim == units::m_per_s.dim;
        switch (spec->type) {
            case SignalType::Bool:
                topic.type = "std_msgs/msg/Bool";
                topic.frame.clear();
                break;
            case SignalType::Int:
                topic.type = "std_msgs/msg/Int64";
                topic.frame.clear();
                break;
            case SignalType::Vec3:
                topic.type = "geometry_msgs/msg/Vector3Stamped";
                if (topic.frame == "world" && spec->unit.dim == units::m.dim) {
                    topic.params["position"] = 1.0;  // gets the floating origin's offset
                }
                break;
            case SignalType::Quat:
                topic.type = "geometry_msgs/msg/QuaternionStamped";
                break;
            case SignalType::Pose:
                topic.type = "geometry_msgs/msg/PoseStamped";
                break;
            case SignalType::Wrench:
                topic.type =
                    velocity ? "geometry_msgs/msg/TwistStamped" : "geometry_msgs/msg/WrenchStamped";
                break;
            case SignalType::ScalarArray: {
                topic.type = "std_msgs/msg/Float64MultiArray";
                topic.frame.clear();
                // Rows of fields (sensors.md §9): outermost first.
                const std::uint32_t n = spec->arrayLength;
                const auto rows = [&topic, n](std::uint32_t columns) {
                    if (columns > 0 && n % columns == 0) {
                        topic.layout = {n / columns, columns};
                    }
                };
                if (kind == "adsb_receiver" && quantity == "aircraft") {
                    rows(10);
                } else if (kind == "v2x" && quantity == "bsm") {
                    rows(10);
                } else if (kind == "v2x" && quantity == "spat") {
                    rows(7);
                } else if (kind == "proximity" && quantity == "distances") {
                    rows(static_cast<std::uint32_t>(std::llround(get("columns", 1.0))));
                } else if (kind == "sonar" && quantity == "image") {
                    rows(static_cast<std::uint32_t>(std::llround(get("image_bins", 0.0))));
                }
                break;
            }
            default:  // Scalar
                if (spec->unit.dim == units::K.dim) {
                    topic.type = "sensor_msgs/msg/Temperature";
                } else if (spec->unit.dim == units::Pa.dim) {
                    topic.type = "sensor_msgs/msg/FluidPressure";
                } else if (leaf == "relative_humidity") {
                    topic.type = "sensor_msgs/msg/RelativeHumidity";
                } else if (leaf == "illuminance") {
                    topic.type = "sensor_msgs/msg/Illuminance";
                } else {
                    topic.type = "std_msgs/msg/Float64";
                    topic.frame.clear();
                }
                break;
        }
        out.push_back(std::move(topic));
    }
    return out;
}

std::vector<KindTopic> withoutMapped(std::vector<KindTopic> topics, const TopicMaps& maps) {
    if (!maps.kindTopics) {
        return {};
    }
    std::set<std::string, std::less<>> mappedTopics;
    std::set<std::string, std::less<>> mappedSignals;
    for (const TopicMap& map : maps.publish) {
        mappedTopics.insert(map.topic);
        mappedSignals.insert(map.signal);
    }
    std::erase_if(topics, [&](const KindTopic& topic) {
        return mappedTopics.contains(topic.topic) ||
               std::ranges::any_of(topic.signals, [&](const std::string& signal) {
                   return mappedSignals.contains(signal);
               });
    });
    return topics;
}

struct KindTopics::Impl {
    using Values = std::vector<std::span<const double>>;
    /// Builds the message from its signals' values and sends it through the hooks.
    using Send = std::function<void(const KindTopic&, const Values&, SimTime, const Hooks&)>;
    struct Out {
        KindTopic topic;
        std::vector<SignalId> ids;  // per KindTopic::signals; invalid for an absent one
        SimTime sent{-1};
        rclcpp::PublisherBase::SharedPtr publisher;
        Send send;
    };
    rclcpp::Node& node;
    std::vector<KindTopic> topics;
    std::vector<Out> outs;
    std::map<std::string, rclcpp::PublisherBase::SharedPtr, std::less<>> publishers;  // by topic

    explicit Impl(rclcpp::Node& n) : node(n) {}

    template <class Message, class Fill>
    void make(Out& out, Fill fill) {
        using Publisher = rclcpp::Publisher<Message>;
        std::shared_ptr<Publisher> publisher;
        if (const auto it = publishers.find(out.topic.topic); it != publishers.end()) {
            publisher = std::dynamic_pointer_cast<Publisher>(it->second);
        }
        if (!publisher) {
            publisher = node.create_publisher<Message>(out.topic.topic, 10);
            publishers[out.topic.topic] = publisher;
        }
        out.publisher = publisher;
        out.send = [publisher, fill](const KindTopic& topic, const Values& v, SimTime stamp,
                                     const Hooks& hooks) {
            auto message = std::make_shared<Message>();
            const std::size_t bytes = fill(*message, topic, v, stamp, hooks);
            hooks.deliver(
                topic.topic, [publisher, message]() { publisher->publish(*message); }, bytes);
        };
    }

    void make(Out& out);
};

namespace {

double param(const KindTopic& topic, const char* key, double fallback) {
    const auto it = topic.params.find(key);
    return it == topic.params.end() ? fallback : it->second;
}

/// The value of signal `k`, or `fallback` when the topic has no such signal.
double at(const KindTopics::Impl::Values& v, std::size_t k, double fallback = kNaN) {
    return k < v.size() && !v[k].empty() ? v[k][0] : fallback;
}

/// Rows filled of a detections array (signals detections, count): `count`, else all rows.
std::size_t detectionCount(const KindTopics::Impl::Values& v) {
    const std::size_t rows = v[0].size() / kDetectionFields;
    const double counted = at(v, 1, kNaN);
    return std::isfinite(counted)
               ? std::min(rows, static_cast<std::size_t>(std::llround(std::max(counted, 0.0))))
               : rows;
}

template <class Header>
void stampHeader(Header& header, const KindTopic& topic, SimTime stamp) {
    header.stamp = stampOf(stamp);
    header.frame_id = topic.frame;
}

/// Positions in the world frame: the region's offset added (the floating origin, as /odom).
std::array<double, 3> worldPosition(const KindTopic& topic, std::span<const double> v,
                                    const KindTopics::Hooks& hooks) {
    std::array<double, 3> p{v[0], v[1], v[2]};
    if (topic.frame == "world") {
        for (std::size_t i = 0; i < 3; ++i) {
            p.at(i) += hooks.offset.at(i);
        }
    }
    return p;
}

}  // namespace

void KindTopics::Impl::make(Out& out) {
    using namespace std_msgs::msg;
    using namespace sensor_msgs::msg;
    using namespace geometry_msgs::msg;
    using V = const Values&;
    using H = const Hooks&;
    const std::string& t = out.topic.type;
    if (t == "std_msgs/msg/Bool") {
        make<Bool>(out, [](Bool& m, const KindTopic&, V v, SimTime, H) {
            m.data = at(v, 0, 0.0) > 0.5;
            return std::size_t{1};
        });
    } else if (t == "std_msgs/msg/Int64") {
        make<Int64>(out, [](Int64& m, const KindTopic&, V v, SimTime, H) {
            const double x = at(v, 0, 0.0);
            m.data = std::isfinite(x) ? std::llround(x) : 0;
            return std::size_t{8};
        });
    } else if (t == "std_msgs/msg/Float64") {
        make<Float64>(out, [](Float64& m, const KindTopic&, V v, SimTime, H) {
            m.data = at(v, 0);
            return std::size_t{8};
        });
    } else if (t == "std_msgs/msg/Float64MultiArray") {
        make<Float64MultiArray>(out, [](Float64MultiArray& m, const KindTopic& topic, V v, SimTime,
                                        H) {
            m.data.assign(v[0].begin(), v[0].end());
            const std::vector<std::uint32_t> sizes =
                topic.layout.empty()
                    ? std::vector<std::uint32_t>{static_cast<std::uint32_t>(v[0].size())}
                    : topic.layout;
            static constexpr std::array<const char*, 2> kLabels{"rows", "columns"};
            auto stride = static_cast<std::uint32_t>(v[0].size());
            for (std::size_t d = 0; d < sizes.size(); ++d) {
                MultiArrayDimension dim;
                dim.label = sizes.size() == 1 ? "values" : kLabels.at(std::min<std::size_t>(d, 1));
                dim.size = sizes[d];
                dim.stride = stride;
                stride = sizes[d] == 0 ? 0 : stride / sizes[d];
                m.layout.dim.push_back(dim);
            }
            return m.data.size() * 8 + 32;
        });
    } else if (t == "sensor_msgs/msg/Temperature") {
        make<Temperature>(out, [](Temperature& m, const KindTopic& topic, V v, SimTime s, H) {
            stampHeader(m.header, topic, s);
            m.temperature = at(v, 0) - kZeroCelsiusK;
            return std::size_t{40};
        });
    } else if (t == "sensor_msgs/msg/FluidPressure") {
        make<FluidPressure>(out, [](FluidPressure& m, const KindTopic& topic, V v, SimTime s, H) {
            stampHeader(m.header, topic, s);
            m.fluid_pressure = at(v, 0);
            return std::size_t{40};
        });
    } else if (t == "sensor_msgs/msg/RelativeHumidity") {
        make<RelativeHumidity>(out,
                               [](RelativeHumidity& m, const KindTopic& topic, V v, SimTime s, H) {
                                   stampHeader(m.header, topic, s);
                                   m.relative_humidity = at(v, 0);  // 0 to 1, as the signal
                                   return std::size_t{40};
                               });
    } else if (t == "sensor_msgs/msg/Illuminance") {
        make<Illuminance>(out, [](Illuminance& m, const KindTopic& topic, V v, SimTime s, H) {
            stampHeader(m.header, topic, s);
            m.illuminance = at(v, 0);
            return std::size_t{40};
        });
    } else if (t == "sensor_msgs/msg/Range") {
        make<Range>(out, [](Range& m, const KindTopic& topic, V v, SimTime s, H) {
            stampHeader(m.header, topic, s);
            m.radiation_type = static_cast<std::uint8_t>(param(topic, "radiation_type", 1));
            m.field_of_view = static_cast<float>(param(topic, "field_of_view", 0.0));
            m.min_range = static_cast<float>(param(topic, "min_range", 0.0));
            m.max_range = static_cast<float>(param(topic, "max_range", kInf));
            m.range = static_cast<float>(at(v, 0, kInf));  // +inf: nothing in range (REP 117)
            return std::size_t{48};
        });
    } else if (t == "sensor_msgs/msg/LaserScan") {
        make<LaserScan>(out, [](LaserScan& m, const KindTopic& topic, V v, SimTime s, H) {
            stampHeader(m.header, topic, s);
            const std::span<const double> ranges = v[0];
            m.angle_min = static_cast<float>(param(topic, "angle_min", 0.0));
            m.angle_max = static_cast<float>(param(topic, "angle_max", 0.0));
            m.angle_increment = ranges.size() > 1 ? (m.angle_max - m.angle_min) /
                                                        static_cast<float>(ranges.size() - 1)
                                                  : 0.0F;
            m.scan_time = static_cast<float>(param(topic, "scan_time", 0.0));
            m.range_min = static_cast<float>(param(topic, "range_min", 0.0));
            m.range_max = static_cast<float>(param(topic, "range_max", kInf));
            m.ranges.assign(ranges.begin(), ranges.end());
            if (v.size() > 1 && v[1].size() == ranges.size()) {
                m.intensities.assign(v[1].begin(), v[1].end());
            }
            return (m.ranges.size() + m.intensities.size()) * 4 + 64;
        });
    } else if (t == "geometry_msgs/msg/Vector3Stamped") {
        make<Vector3Stamped>(
            out, [](Vector3Stamped& m, const KindTopic& topic, V v, SimTime s, H hooks) {
                stampHeader(m.header, topic, s);
                if (v[0].size() == 2) {  // a flow's x and y
                    m.vector.x = v[0][0];
                    m.vector.y = v[0][1];
                    return std::size_t{56};
                }
                // A position in the world gets the region's offset; a direction or a force does
                // not.
                const bool position = topic.frame == "world" && topic.params.contains("position");
                const std::array<double, 3> p =
                    position ? worldPosition(topic, v[0], hooks)
                             : std::array<double, 3>{v[0][0], v[0][1], v[0][2]};
                m.vector.x = p[0];
                m.vector.y = p[1];
                m.vector.z = p[2];
                return std::size_t{56};
            });
    } else if (t == "geometry_msgs/msg/QuaternionStamped") {
        make<QuaternionStamped>(
            out, [](QuaternionStamped& m, const KindTopic& topic, V v, SimTime s, H) {
                stampHeader(m.header, topic, s);
                m.quaternion.w = v[0][0];
                m.quaternion.x = v[0][1];
                m.quaternion.y = v[0][2];
                m.quaternion.z = v[0][3];
                return std::size_t{64};
            });
    } else if (t == "geometry_msgs/msg/PoseStamped") {
        make<PoseStamped>(out, [](PoseStamped& m, const KindTopic& topic, V v, SimTime s, H hooks) {
            stampHeader(m.header, topic, s);
            const std::array<double, 3> p = worldPosition(topic, v[0], hooks);
            m.pose.position.x = p[0];
            m.pose.position.y = p[1];
            m.pose.position.z = p[2];
            m.pose.orientation.w = v[0][3];
            m.pose.orientation.x = v[0][4];
            m.pose.orientation.y = v[0][5];
            m.pose.orientation.z = v[0][6];
            return std::size_t{88};
        });
    } else if (t == "geometry_msgs/msg/TwistStamped") {
        make<TwistStamped>(out, [](TwistStamped& m, const KindTopic& topic, V v, SimTime s, H) {
            stampHeader(m.header, topic, s);
            m.twist.linear.x = v[0][0];
            m.twist.linear.y = v[0][1];
            m.twist.linear.z = v[0][2];
            m.twist.angular.x = v[0][3];
            m.twist.angular.y = v[0][4];
            m.twist.angular.z = v[0][5];
            return std::size_t{80};
        });
    } else if (t == "geometry_msgs/msg/WrenchStamped") {
        make<WrenchStamped>(out, [](WrenchStamped& m, const KindTopic& topic, V v, SimTime s, H) {
            stampHeader(m.header, topic, s);
            m.wrench.force.x = v[0][0];
            m.wrench.force.y = v[0][1];
            m.wrench.force.z = v[0][2];
            m.wrench.torque.x = v[0][3];
            m.wrench.torque.y = v[0][4];
            m.wrench.torque.z = v[0][5];
            return std::size_t{80};
        });
    } else if (t == "nav_msgs/msg/Odometry") {
        make<nav_msgs::msg::Odometry>(
            out, [](nav_msgs::msg::Odometry& m, const KindTopic& topic, V v, SimTime s, H) {
                stampHeader(m.header, topic, s);
                m.child_frame_id = topic.childFrame;
                const std::span<const double> pose = v[0];
                m.pose.pose.position.x = pose[0];  // the odometry frame: no floating origin
                m.pose.pose.position.y = pose[1];
                m.pose.pose.position.z = pose[2];
                m.pose.pose.orientation.w = pose[3];
                m.pose.pose.orientation.x = pose[4];
                m.pose.pose.orientation.y = pose[5];
                m.pose.pose.orientation.z = pose[6];
                const std::span<const double> twist = v[1];  // the body frame, as Odometry has it
                m.twist.twist.linear.x = twist[0];
                m.twist.twist.linear.y = twist[1];
                m.twist.twist.linear.z = twist[2];
                m.twist.twist.angular.x = twist[3];
                m.twist.twist.angular.y = twist[4];
                m.twist.twist.angular.z = twist[5];
                for (std::size_t k = 0; k < 6; ++k) {
                    m.pose.covariance.at(7 * k) =
                        param(topic, ("pose_covariance_" + std::to_string(k)).c_str(), 0.0);
                    m.twist.covariance.at(7 * k) =
                        param(topic, ("twist_covariance_" + std::to_string(k)).c_str(), 0.0);
                }
                return std::size_t{720};
            });
    } else if (t == "sensor_msgs/msg/BatteryState") {
        make<BatteryState>(out, [](BatteryState& m, const KindTopic& topic, V v, SimTime s, H) {
            stampHeader(m.header, topic, s);
            // Signals: voltage, current, soc, temperature, remaining_capacity, valid.
            const double current = at(v, 1);  // A, positive discharging (Lagrange's sign)
            const double soc = at(v, 2);
            const double charge = at(v, 4) / kCoulombsPerAmpHour;  // Ah
            const bool valid = at(v, 5, 1.0) > 0.5;
            m.voltage = static_cast<float>(at(v, 0));
            m.current = static_cast<float>(-current);  // ROS: negative when discharging
            m.percentage = static_cast<float>(soc);
            m.temperature = static_cast<float>(at(v, 3) - kZeroCelsiusK);
            m.charge = static_cast<float>(charge);
            const double capacity = param(topic, "capacity", kNaN);
            m.design_capacity = static_cast<float>(capacity);
            m.capacity = static_cast<float>(
                std::isfinite(capacity) ? capacity : (soc > 0.0 ? charge / soc : kNaN));
            m.present = valid;
            if (!valid) {
                m.power_supply_status = BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
            } else if (soc >= 1.0) {
                m.power_supply_status = BatteryState::POWER_SUPPLY_STATUS_FULL;
            } else if (current > 0.0) {
                m.power_supply_status = BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
            } else if (current < 0.0) {
                m.power_supply_status = BatteryState::POWER_SUPPLY_STATUS_CHARGING;
            } else {
                m.power_supply_status = BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
            }
            m.power_supply_health = BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
            m.power_supply_technology = BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
            m.location = topic.text;
            return std::size_t{96};
        });
    } else if (t == "sensor_msgs/msg/JointState") {
        make<JointState>(out, [](JointState& m, const KindTopic& topic, V v, SimTime s, H) {
            stampHeader(m.header, topic, s);
            m.name = {topic.text};
            m.effort = {at(v, 0)};  // N·m, or N on a slide
            return std::size_t{64};
        });
    } else if (t == "vision_msgs/msg/Detection2DArray") {
        make<vision_msgs::msg::Detection2DArray>(
            out,
            [](vision_msgs::msg::Detection2DArray& m, const KindTopic& topic, V v, SimTime s, H) {
                stampHeader(m.header, topic, s);
                const std::span<const double> rows = v[0];
                const std::size_t count = detectionCount(v);
                for (std::size_t r = 0; r < count; ++r) {
                    const std::span<const double> row =
                        rows.subspan(r * kDetectionFields, kDetectionFields);
                    if (!std::isfinite(row[0])) {
                        break;  // NaN rows after the last
                    }
                    vision_msgs::msg::Detection2D d;
                    d.header = m.header;
                    d.id = std::to_string(std::llround(row[0]));
                    vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
                    hypothesis.hypothesis.class_id = std::to_string(std::llround(row[1]));
                    hypothesis.hypothesis.score = 1.0;  // ground truth
                    d.results.push_back(hypothesis);
                    // Inclusive pixel columns and rows: the centre between them, the size across.
                    d.bbox.center.position.x = (row[2] + row[4]) / 2.0;
                    d.bbox.center.position.y = (row[3] + row[5]) / 2.0;
                    d.bbox.center.theta = 0.0;
                    d.bbox.size_x = row[4] - row[2] + 1.0;
                    d.bbox.size_y = row[5] - row[3] + 1.0;
                    m.detections.push_back(std::move(d));
                }
                return m.detections.size() * 120 + 32;
            });
    } else if (t == "vision_msgs/msg/Detection3DArray") {
        make<vision_msgs::msg::Detection3DArray>(
            out,
            [](vision_msgs::msg::Detection3DArray& m, const KindTopic& topic, V v, SimTime s, H) {
                stampHeader(m.header, topic, s);
                const std::span<const double> rows = v[0];
                const std::size_t count = detectionCount(v);
                for (std::size_t r = 0; r < count; ++r) {
                    const std::span<const double> row =
                        rows.subspan(r * kDetectionFields, kDetectionFields);
                    if (!std::isfinite(row[0])) {
                        break;
                    }
                    vision_msgs::msg::Detection3D d;
                    d.header = m.header;
                    d.id = std::to_string(std::llround(row[0]));
                    vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
                    hypothesis.hypothesis.class_id = std::to_string(std::llround(row[1]));
                    hypothesis.hypothesis.score = 1.0;
                    // KITTI's box in the optical frame: turned by rotation_y about the optical y,
                    // its x along the object, y down (the object's height), z across (its width).
                    const double yaw = std::isfinite(row[15]) ? row[15] : 0.0;
                    d.bbox.center.position.x = row[9];
                    d.bbox.center.position.y = row[10];
                    d.bbox.center.position.z = row[11];
                    d.bbox.center.orientation.w = std::cos(yaw / 2.0);
                    d.bbox.center.orientation.y = std::sin(yaw / 2.0);
                    d.bbox.size.x = row[12];
                    d.bbox.size.y = row[14];
                    d.bbox.size.z = row[13];
                    hypothesis.pose.pose = d.bbox.center;
                    d.results.push_back(hypothesis);
                    m.detections.push_back(std::move(d));
                }
                return m.detections.size() * 400 + 32;
            });
    }
}

KindTopics::KindTopics(rclcpp::Node& node) : impl_(std::make_unique<Impl>(node)) {}

KindTopics::~KindTopics() = default;

std::vector<std::string> KindTopics::attach(std::vector<KindTopic> topics, const SignalBus& bus) {
    std::vector<std::string> problems;
    impl_->outs.clear();
    impl_->topics.clear();
    for (KindTopic& topic : topics) {
        Impl::Out out;
        out.topic = topic;
        bool complete = true;
        for (const std::string& signal : topic.signals) {
            const SignalId id = signal.empty() ? SignalId{} : bus.find(signal);
            out.ids.push_back(id.valid() && !id.isInput() ? id : SignalId{});
            complete = complete && (signal.empty() || out.ids.back().valid());
        }
        if (!complete || out.ids.empty() || !out.ids.front().valid()) {
            problems.push_back(topic.topic + ": a signal it carries is not in this run");
            continue;
        }
        try {
            impl_->make(out);
        } catch (const std::exception& e) {
            problems.push_back(topic.topic + ": " + e.what());
            continue;
        }
        if (!out.send) {
            problems.push_back(topic.topic + ": " + topic.type + " is not published");
            continue;
        }
        impl_->topics.push_back(std::move(topic));
        impl_->outs.push_back(std::move(out));
    }
    return problems;
}

void KindTopics::publish(const SignalBus& bus, const Hooks& hooks) {
    Impl::Values values;
    for (Impl::Out& out : impl_->outs) {
        const SimTime stamp = bus.stamp(out.ids.front());
        if (stamp == out.sent) {
            continue;
        }
        out.sent = stamp;
        if (out.publisher->get_subscription_count() +
                    out.publisher->get_intra_process_subscription_count() ==
                0 &&
            !(hooks.forced && hooks.forced(out.topic.topic))) {
            continue;  // nobody listens: not worth building
        }
        values.clear();
        for (const SignalId id : out.ids) {
            values.push_back(id.valid() ? bus.readN(id) : std::span<const double>());
        }
        out.send(out.topic, values, stamp, hooks);
    }
}

const std::vector<KindTopic>& KindTopics::topics() const {
    return impl_->topics;
}

}  // namespace lagrange::ros2
