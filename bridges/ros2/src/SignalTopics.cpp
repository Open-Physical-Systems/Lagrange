// Signals on ROS 2 topics and back (RFC 0008 §4): lagrange_server --ros-topics FILE.yaml.
#include "lagrange/ros2/SignalTopics.hpp"
#include "lagrange/signals/Signal.hpp"
#include "lagrange/signals/Unit.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <mutex>
#include <span>

#include <yaml-cpp/yaml.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/illuminance.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int64.hpp>

namespace lagrange::ros2 {

namespace {

constexpr double kZeroCelsiusK = 273.15;  // K at 0 °C (sensor_msgs/Temperature is in °C)

builtin_interfaces::msg::Time stampOf(SimTime t) {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<std::int32_t>(t.ns / 1'000'000'000);
    stamp.nanosec = static_cast<std::uint32_t>(t.ns % 1'000'000'000);
    return stamp;
}

/// What a published message needs of its signal: a number of values (0: any, 3: a multiple of
/// three for point clouds when `triples`), and a unit (empty: any).
struct Fit {
    std::size_t components{1};
    bool any{false};
    bool triples{false};
    const char* unit{""};
};

Fit fitOf(const std::string& type) {
    if (type == "std_msgs/msg/Float64MultiArray" || type == "sensor_msgs/msg/LaserScan") {
        return {0, true, false, type == "sensor_msgs/msg/LaserScan" ? "m" : ""};
    }
    if (type == "sensor_msgs/msg/PointCloud2") {
        return {0, true, true, "m"};
    }
    if (type == "geometry_msgs/msg/PoseStamped") {
        return {7, false, false, "m"};
    }
    if (type == "geometry_msgs/msg/TwistStamped") {
        return {6, false, false, "m/s"};
    }
    if (type == "geometry_msgs/msg/WrenchStamped") {
        return {6, false, false, "N"};
    }
    if (type == "geometry_msgs/msg/Vector3Stamped") {
        return {3, false, false, ""};
    }
    if (type == "sensor_msgs/msg/Range") {
        return {1, false, false, "m"};
    }
    if (type == "sensor_msgs/msg/Temperature") {
        return {1, false, false, "K"};
    }
    if (type == "sensor_msgs/msg/FluidPressure") {
        return {1, false, false, "Pa"};
    }
    return {};  // the scalars
}

double param(const TopicMap& map, const char* key, double fallback) {
    const auto it = map.params.find(key);
    return it == map.params.end() ? fallback : it->second;
}

}  // namespace

const std::vector<std::string>& publishTypes() {
    static const std::vector<std::string> kTypes{"std_msgs/msg/Float64",
                                                 "std_msgs/msg/Float64MultiArray",
                                                 "std_msgs/msg/Bool",
                                                 "std_msgs/msg/Int64",
                                                 "sensor_msgs/msg/Range",
                                                 "sensor_msgs/msg/Temperature",
                                                 "sensor_msgs/msg/FluidPressure",
                                                 "sensor_msgs/msg/Illuminance",
                                                 "sensor_msgs/msg/RelativeHumidity",
                                                 "sensor_msgs/msg/LaserScan",
                                                 "sensor_msgs/msg/PointCloud2",
                                                 "geometry_msgs/msg/PoseStamped",
                                                 "geometry_msgs/msg/TwistStamped",
                                                 "geometry_msgs/msg/WrenchStamped",
                                                 "geometry_msgs/msg/Vector3Stamped"};
    return kTypes;
}

const std::vector<std::string>& subscribeTypes() {
    static const std::vector<std::string> kTypes{"std_msgs/msg/Float64",
                                                 "std_msgs/msg/Float64MultiArray",
                                                 "std_msgs/msg/Bool", "std_msgs/msg/Int64"};
    return kTypes;
}

Expected<TopicMaps, std::string> readTopicMaps(const std::string& path) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        return Unexpected{path + ": " + e.what()};
    }
    TopicMaps maps;
    for (const char* direction : {"publish", "subscribe"}) {
        const YAML::Node list = root[direction];
        if (!list) {
            continue;
        }
        if (!list.IsSequence()) {
            return Unexpected{path + ": " + direction + " is a list"};
        }
        const bool publishing = std::string_view(direction) == "publish";
        const std::vector<std::string>& types = publishing ? publishTypes() : subscribeTypes();
        for (std::size_t i = 0; i < list.size(); ++i) {
            const YAML::Node entry = list[i];
            const std::string at = path + ": " + direction + "[" + std::to_string(i) + "]";
            try {
                TopicMap map;
                for (const char* key : {"signal", "topic", "type"}) {
                    if (!entry[key] || !entry[key].IsScalar()) {
                        return Unexpected{at + " needs " + key};
                    }
                }
                map.signal = entry["signal"].as<std::string>();
                map.topic = entry["topic"].as<std::string>();
                map.type = entry["type"].as<std::string>();
                if (std::ranges::find(types, map.type) == types.end()) {
                    std::string choices;
                    for (const std::string& type : types) {
                        choices += (choices.empty() ? "" : ", ") + type;
                    }
                    return Unexpected{at + ": " + map.type + " is not mapped " +
                                      (publishing ? "from" : "to") + " a signal; one of " +
                                      choices};
                }
                if (entry["frame"]) {
                    map.frame = entry["frame"].as<std::string>();
                }
                if (entry["unit"]) {
                    map.unit = entry["unit"].as<std::string>();
                }
                if (entry["size"]) {
                    map.size = entry["size"].as<std::uint32_t>();
                }
                if (const YAML::Node params = entry["params"]; params && params.IsMap()) {
                    for (const auto& item : params) {
                        map.params[item.first.as<std::string>()] = item.second.as<double>();
                    }
                }
                (publishing ? maps.publish : maps.subscribe).push_back(std::move(map));
            } catch (const YAML::Exception& e) {
                return Unexpected{at + ": " + e.what()};
            }
        }
    }
    return maps;
}

struct SignalTopics::Impl {
    struct Out {
        TopicMap map;
        SignalId id;
        SimTime sent{-1};
        std::function<void(std::span<const double>, SimTime)> send;
    };
    struct In {
        TopicMap map;
        rclcpp::SubscriptionBase::SharedPtr subscription;
    };
    rclcpp::Node& node;
    std::vector<Out> outs;
    std::vector<std::string> problems;  // from making the publishers and subscriptions
    std::vector<In> ins;
    /// The subscribed topics' latest values, by signal; written by the executor thread.
    struct Latest {
        std::mutex mutex;
        std::vector<std::vector<double>> values;
    };
    std::shared_ptr<Latest> latest = std::make_shared<Latest>();

    explicit Impl(rclcpp::Node& n) : node(n) {}

    template <class Message, class Build>
    std::function<void(std::span<const double>, SimTime)> publisher(const TopicMap& map,
                                                                    Build build) {
        auto pub = node.create_publisher<Message>(map.topic, 10);
        return [pub, build, map](std::span<const double> v, SimTime now) {
            Message message;
            build(message, v, now, map);
            pub->publish(std::move(message));
        };
    }

    std::function<void(std::span<const double>, SimTime)> sender(const TopicMap& m) {
        using namespace std_msgs::msg;
        using namespace sensor_msgs::msg;
        using namespace geometry_msgs::msg;
        const auto header = [](auto& h, SimTime now, const TopicMap& map) {
            h.stamp = stampOf(now);
            h.frame_id = map.frame;
        };
        const std::string& t = m.type;
        if (t == "std_msgs/msg/Float64") {
            return publisher<Float64>(
                m, [](Float64& out, auto v, SimTime, const TopicMap&) { out.data = v[0]; });
        }
        if (t == "std_msgs/msg/Float64MultiArray") {
            return publisher<Float64MultiArray>(
                m, [](Float64MultiArray& out, auto v, SimTime, const TopicMap&) {
                    out.data.assign(v.begin(), v.end());
                });
        }
        if (t == "std_msgs/msg/Bool") {
            return publisher<Bool>(
                m, [](Bool& out, auto v, SimTime, const TopicMap&) { out.data = v[0] > 0.5; });
        }
        if (t == "std_msgs/msg/Int64") {
            return publisher<Int64>(m, [](Int64& out, auto v, SimTime, const TopicMap&) {
                out.data = std::llround(v[0]);
            });
        }
        if (t == "sensor_msgs/msg/Range") {
            return publisher<Range>(
                m, [header](Range& out, auto v, SimTime now, const TopicMap& map) {
                    header(out.header, now, map);
                    out.radiation_type = static_cast<std::uint8_t>(param(map, "radiation_type", 1));
                    out.field_of_view = static_cast<float>(param(map, "field_of_view", 0.0));
                    out.min_range = static_cast<float>(param(map, "min_range", 0.0));
                    out.max_range = static_cast<float>(
                        param(map, "max_range", std::numeric_limits<double>::infinity()));
                    out.range = static_cast<float>(v[0]);  // +inf: nothing in range (REP 117)
                });
        }
        if (t == "sensor_msgs/msg/Temperature") {
            return publisher<Temperature>(
                m, [header](Temperature& out, auto v, SimTime now, const TopicMap& map) {
                    header(out.header, now, map);
                    out.temperature = v[0] - kZeroCelsiusK;
                });
        }
        if (t == "sensor_msgs/msg/FluidPressure") {
            return publisher<FluidPressure>(
                m, [header](FluidPressure& out, auto v, SimTime now, const TopicMap& map) {
                    header(out.header, now, map);
                    out.fluid_pressure = v[0];
                });
        }
        if (t == "sensor_msgs/msg/Illuminance") {
            return publisher<Illuminance>(
                m, [header](Illuminance& out, auto v, SimTime now, const TopicMap& map) {
                    header(out.header, now, map);
                    out.illuminance = v[0];
                });
        }
        if (t == "sensor_msgs/msg/RelativeHumidity") {
            return publisher<RelativeHumidity>(
                m, [header](RelativeHumidity& out, auto v, SimTime now, const TopicMap& map) {
                    header(out.header, now, map);
                    out.relative_humidity = v[0];
                });
        }
        if (t == "sensor_msgs/msg/LaserScan") {
            return publisher<LaserScan>(
                m, [header](LaserScan& out, auto v, SimTime now, const TopicMap& map) {
                    header(out.header, now, map);
                    out.angle_min = static_cast<float>(param(map, "angle_min", 0.0));
                    out.angle_max = static_cast<float>(param(map, "angle_max", 0.0));
                    out.angle_increment = v.size() > 1 ? (out.angle_max - out.angle_min) /
                                                             static_cast<float>(v.size() - 1)
                                                       : 0.0F;
                    out.range_min = static_cast<float>(param(map, "range_min", 0.0));
                    out.range_max = static_cast<float>(
                        param(map, "range_max", std::numeric_limits<double>::infinity()));
                    out.ranges.assign(v.begin(), v.end());
                });
        }
        if (t == "sensor_msgs/msg/PointCloud2") {
            return publisher<PointCloud2>(
                m, [header](PointCloud2& out, auto v, SimTime now, const TopicMap& map) {
                    header(out.header, now, map);
                    const std::array<const char*, 3> names{"x", "y", "z"};
                    for (std::uint32_t i = 0; i < names.size(); ++i) {
                        PointField field;
                        field.name = names.at(i);
                        field.offset = 4 * i;
                        field.datatype = PointField::FLOAT32;
                        field.count = 1;
                        out.fields.push_back(field);
                    }
                    const auto points = static_cast<std::uint32_t>(v.size() / 3);
                    out.height = 1;
                    out.width = points;
                    out.point_step = 12;
                    out.row_step = 12 * points;
                    out.is_dense = true;
                    out.is_bigendian = false;
                    out.data.assign(static_cast<std::size_t>(out.row_step), 0);
                    for (std::size_t i = 0; i < static_cast<std::size_t>(points) * 3; ++i) {
                        const auto value = static_cast<float>(v[i]);
                        std::memcpy(out.data.data() + 4 * i, &value, sizeof value);
                    }
                });
        }
        if (t == "geometry_msgs/msg/PoseStamped") {
            return publisher<PoseStamped>(
                m, [header](PoseStamped& out, auto v, SimTime now, const TopicMap& map) {
                    header(out.header, now, map);
                    out.pose.position.x = v[0];
                    out.pose.position.y = v[1];
                    out.pose.position.z = v[2];
                    out.pose.orientation.w = v[3];
                    out.pose.orientation.x = v[4];
                    out.pose.orientation.y = v[5];
                    out.pose.orientation.z = v[6];
                });
        }
        if (t == "geometry_msgs/msg/TwistStamped") {
            return publisher<TwistStamped>(
                m, [header](TwistStamped& out, auto v, SimTime now, const TopicMap& map) {
                    header(out.header, now, map);
                    out.twist.linear.x = v[0];
                    out.twist.linear.y = v[1];
                    out.twist.linear.z = v[2];
                    out.twist.angular.x = v[3];
                    out.twist.angular.y = v[4];
                    out.twist.angular.z = v[5];
                });
        }
        if (t == "geometry_msgs/msg/WrenchStamped") {
            return publisher<WrenchStamped>(
                m, [header](WrenchStamped& out, auto v, SimTime now, const TopicMap& map) {
                    header(out.header, now, map);
                    out.wrench.force.x = v[0];
                    out.wrench.force.y = v[1];
                    out.wrench.force.z = v[2];
                    out.wrench.torque.x = v[3];
                    out.wrench.torque.y = v[4];
                    out.wrench.torque.z = v[5];
                });
        }
        return publisher<Vector3Stamped>(
            m, [header](Vector3Stamped& out, auto v, SimTime now, const TopicMap& map) {
                header(out.header, now, map);
                out.vector.x = v[0];
                out.vector.y = v[1];
                out.vector.z = v[2];
            });
    }

    template <class Message, class Read>
    void subscribe(std::size_t index, const TopicMap& map, Read read) {
        In in;
        in.map = map;
        in.subscription = node.create_subscription<Message>(
            map.topic, 10, [state = latest, index, read, size = map.size](const Message& message) {
                std::vector<double> values = read(message);
                values.resize(size, 0.0);
                const std::lock_guard<std::mutex> lock(state->mutex);
                state->values.at(index) = std::move(values);
            });
        ins.push_back(std::move(in));
    }
};

namespace {

/// Writes the subscribed topics' latest values into their signals (RFC 0008 §4). Its input
/// comes from outside the simulation, so it is not deterministic.
class TopicInputs final : public IModule {
public:
    TopicInputs(std::vector<TopicMap> maps, std::shared_ptr<SignalTopics::Impl::Latest> latest)
        : maps_(std::move(maps)), latest_(std::move(latest)) {}

    [[nodiscard]] ModuleInfo info() const override {
        ModuleInfo info{"ros2", "topic_inputs"};
        info.deterministic = false;
        return info;
    }
    [[nodiscard]] Duration preferredStep() const override { return Duration{10'000'000}; }

    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& declarer) override {
        ids_.clear();
        for (const TopicMap& map : maps_) {
            const UnitParse unit = parseUnit(map.unit);
            if (!unit.unit) {
                return Status::error(StatusCode::InvalidConfig,
                                     map.topic + ": unit " + map.unit + ": " + unit.error);
            }
            SignalSpec spec;
            spec.name = map.signal;
            spec.type = map.size > 1 ? SignalType::ScalarArray : SignalType::Scalar;
            spec.arrayLength = map.size;
            spec.unit = *unit.unit;
            ids_.push_back(declarer.output(spec));
        }
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        return step({}, bus);
    }
    Status step(const StepContext&, SignalBus& bus) override {
        const std::lock_guard<std::mutex> lock(latest_->mutex);
        for (std::size_t i = 0; i < ids_.size(); ++i) {
            const std::vector<double>& values = latest_->values.at(i);
            if (values.empty()) {
                continue;  // nothing received yet: the signal stays 0
            }
            bus.writeN(ids_[i], values);
        }
        return {};
    }
    // Its values come from outside; a rebuilt run keeps receiving them.
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    std::vector<TopicMap> maps_;
    std::shared_ptr<SignalTopics::Impl::Latest> latest_;
    std::vector<SignalId> ids_;
};

}  // namespace

SignalTopics::SignalTopics(rclcpp::Node& node, TopicMaps maps)
    : impl_(std::make_unique<Impl>(node)) {
    // rclcpp throws for a topic that already exists with another type: reported by attach().
    for (TopicMap& map : maps.publish) {
        Impl::Out out;
        try {
            out.send = impl_->sender(map);
        } catch (const std::exception& e) {
            impl_->problems.push_back(map.topic + ": " + e.what());
            continue;
        }
        out.map = std::move(map);
        impl_->outs.push_back(std::move(out));
    }
    impl_->latest->values.resize(maps.subscribe.size());
    for (std::size_t i = 0; i < maps.subscribe.size(); ++i) {
        const TopicMap& map = maps.subscribe[i];
        using namespace std_msgs::msg;
        try {
            if (map.type == "std_msgs/msg/Float64") {
                impl_->subscribe<Float64>(
                    i, map, [](const Float64& m) { return std::vector<double>{m.data}; });
            } else if (map.type == "std_msgs/msg/Bool") {
                impl_->subscribe<Bool>(
                    i, map, [](const Bool& m) { return std::vector<double>{m.data ? 1.0 : 0.0}; });
            } else if (map.type == "std_msgs/msg/Int64") {
                impl_->subscribe<Int64>(i, map, [](const Int64& m) {
                    return std::vector<double>{static_cast<double>(m.data)};
                });
            } else {
                impl_->subscribe<Float64MultiArray>(i, map, [](const Float64MultiArray& m) {
                    return std::vector<double>(m.data.begin(), m.data.end());
                });
            }
        } catch (const std::exception& e) {
            impl_->problems.push_back(map.topic + ": " + e.what());
        }
    }
}

SignalTopics::~SignalTopics() = default;

std::vector<std::string> SignalTopics::attach(const SignalBus& bus) {
    std::vector<std::string> problems = impl_->problems;
    for (Impl::Out& out : impl_->outs) {
        out.id = bus.find(out.map.signal);
        out.sent = SimTime{-1};
        if (!out.id.valid() || out.id.isInput()) {
            problems.push_back(out.map.topic + ": no signal " + out.map.signal + " in this run");
            continue;
        }
        const SignalSpec& spec = bus.specAt(out.id.index);
        const std::size_t components = componentCount(spec.type, spec.arrayLength);
        const Fit fit = fitOf(out.map.type);
        const bool sized = fit.any ? (components > 0 && (!fit.triples || components % 3 == 0))
                                   : components == fit.components;
        if (!sized) {
            problems.push_back(
                out.map.topic + ": " + out.map.signal + " has " + std::to_string(components) +
                " values; " + out.map.type +
                (fit.any ? (fit.triples ? " takes x, y, z triples" : " takes at least one")
                         : " takes " + std::to_string(fit.components)));
            out.id = SignalId{};
            continue;
        }
        if (fit.unit[0] != '\0') {
            const UnitParse unit = parseUnit(fit.unit);
            if (unit.unit && unit.unit->dim != spec.unit.dim) {
                problems.push_back(out.map.topic + ": " + out.map.signal + " is not in the unit " +
                                   out.map.type + " carries (" + fit.unit + ")");
                out.id = SignalId{};
            }
        }
    }
    return problems;
}

void SignalTopics::publish(const SignalBus& bus, SimTime now) {
    for (Impl::Out& out : impl_->outs) {
        if (!out.id.valid()) {
            continue;
        }
        const SimTime stamp = bus.stamp(out.id);
        if (stamp == out.sent) {
            continue;
        }
        out.sent = stamp;
        out.send(bus.readN(out.id), now);
    }
}

std::unique_ptr<IModule> SignalTopics::inputModule() const {
    if (impl_->ins.empty()) {
        return nullptr;
    }
    std::vector<TopicMap> maps;
    for (const Impl::In& in : impl_->ins) {
        maps.push_back(in.map);
    }
    return std::make_unique<TopicInputs>(std::move(maps), impl_->latest);
}

}  // namespace lagrange::ros2
