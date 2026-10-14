#include "lagrange/ros2/Bridge.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <map>
#include <mutex>
#include <set>
#include <thread>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace lagrange::ros2 {

namespace {

using Clock = std::chrono::steady_clock;

builtin_interfaces::msg::Time stampOf(SimTime t) {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<std::int32_t>(t.ns / 1'000'000'000);
    stamp.nanosec = static_cast<std::uint32_t>(t.ns % 1'000'000'000);
    return stamp;
}

std::span<const double> values(const SignalBus& bus, SignalId id) {
    return id.valid() ? bus.readN(id) : std::span<const double>();
}

/// Messages and an estimate of their size, per topic, for the graph's rate and bandwidth.
struct TopicCounter {
    std::uint64_t messages{0};
    std::uint64_t bytes{0};
    std::uint64_t windowMessages{0};
    std::uint64_t windowBytes{0};
    void add(std::size_t size) {
        ++messages;
        bytes += size;
    }
};

}  // namespace

struct Bridge::Impl {
    rclcpp::Node::SharedPtr node;
    rclcpp::executors::SingleThreadedExecutor executor;
    std::thread spinner;

    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStates;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tfStatic;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr description;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVel;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmdVelStamped;

    void take(const geometry_msgs::msg::Twist& twist) {
        const std::lock_guard<std::mutex> lock(commandMutex);
        command = CmdVel{twist.linear.x, twist.angular.z};
        commandAt = Clock::now();
    }

    struct Joint {
        std::string name;
        SignalId position, velocity, torque;
    };
    struct Lidar {
        std::string frame;
        std::string topic;
        SignalId ranges;
        double minAngle{0.0}, maxAngle{0.0}, minRange{0.0}, maxRange{0.0}, rateHz{10.0};
        SimTime sent{-1};
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher;
    };
    struct Imu {
        std::string frame;
        std::string topic;
        SignalId force, rate, orientation;
        double accelNoise{0.0}, gyroNoise{0.0};
        SimTime sent{-1};
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher;
    };
    std::vector<Joint> joints;
    std::vector<Lidar> lidars;
    std::vector<Imu> imus;
    SignalId basePose, baseTwist;
    SimTime lastClock{-1};
    SimTime lastState{-1};

    mutable std::mutex commandMutex;
    std::optional<CmdVel> command;
    Clock::time_point commandAt;

    std::map<std::string, TopicCounter> counters;  // by topic, the bridge's own
    Clock::time_point windowStart{Clock::now()};
};

Bridge::Bridge(BridgeSettings settings)
    : settings_(std::move(settings)), impl_(std::make_unique<Impl>()) {
    Impl& s = *impl_;
    s.node = std::make_shared<rclcpp::Node>(settings_.nodeName);
    s.clock = s.node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    s.jointStates = s.node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    if (!settings_.baseLink.empty()) {
        s.odom = s.node->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    }
    s.tf = s.node->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));
    s.tfStatic = s.node->create_publisher<tf2_msgs::msg::TFMessage>(
        "/tf_static", rclcpp::QoS(1).transient_local().reliable());
    if (!settings_.robotDescription.empty()) {
        s.description = s.node->create_publisher<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(1).transient_local().reliable());
        std_msgs::msg::String text;
        text.data = settings_.robotDescription;
        s.description->publish(text);
    }
    if (settings_.stampedCmdVel) {
        s.cmdVelStamped = s.node->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel", 10,
            [&s](const geometry_msgs::msg::TwistStamped& twist) { s.take(twist.twist); });
    } else {
        s.cmdVel = s.node->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, [&s](const geometry_msgs::msg::Twist& twist) { s.take(twist); });
    }
    s.executor.add_node(s.node);
    s.spinner = std::thread([&s] { s.executor.spin(); });
}

Bridge::~Bridge() {
    impl_->executor.cancel();
    if (impl_->spinner.joinable()) {
        impl_->spinner.join();
    }
}

void Bridge::attach(const World& world, const SignalBus& bus) {
    Impl& s = *impl_;
    s.joints.clear();
    for (const std::string& name : world.joints.names) {
        const SignalId position = bus.find("joint/" + name + "/position");
        if (position.valid()) {
            s.joints.push_back({name, position, bus.find("joint/" + name + "/velocity"),
                                bus.find("joint/" + name + "/torque")});
        }
    }
    s.basePose = bus.find("body/" + settings_.baseLink + "/pose");
    s.baseTwist = bus.find("body/" + settings_.baseLink + "/twist");

    // Sensor frames are children of their links, at their mounts: published once, latched.
    tf2_msgs::msg::TFMessage mounts;
    const World::Sensors& sensors = world.sensors;
    std::size_t lidarCount = 0;
    std::size_t imuCount = 0;
    for (const SensorType type : sensors.type) {
        (type == SensorType::Lidar ? lidarCount : imuCount) += 1;
    }
    std::vector<Impl::Lidar> lidars;
    std::vector<Impl::Imu> imus;
    for (std::size_t i = 0; i < sensors.names.size(); ++i) {
        const std::string& name = sensors.names[i];
        geometry_msgs::msg::TransformStamped mount;
        mount.header.frame_id = world.bodies.names[sensors.body[i].index];
        mount.child_frame_id = name;
        const Pose& pose = sensors.poseInBody[i];
        mount.transform.translation.x = pose.position[0];
        mount.transform.translation.y = pose.position[1];
        mount.transform.translation.z = pose.position[2];
        mount.transform.rotation.w = pose.orientation[0];
        mount.transform.rotation.x = pose.orientation[1];
        mount.transform.rotation.y = pose.orientation[2];
        mount.transform.rotation.z = pose.orientation[3];
        mounts.transforms.push_back(mount);
        const auto& settings = sensors.settings[i];
        const auto get = [&settings](const char* key, double fallback) {
            const auto it = settings.find(key);
            return it == settings.end() ? fallback : it->second;
        };
        if (sensors.type[i] == SensorType::Lidar) {
            Impl::Lidar lidar;
            lidar.frame = name;
            lidar.topic = lidarCount == 1 ? "/scan" : "/" + name + "/scan";
            lidar.ranges = bus.find("sensor/" + name + "/ranges");
            lidar.minAngle = get("min_angle", -3.14159265358979);
            lidar.maxAngle = get("max_angle", 3.14159265358979);
            lidar.minRange = get("min_range", 0.1);
            lidar.maxRange = get("max_range", 30.0);
            lidar.rateHz = sensors.rateHz[i];
            // Reuse the publisher across attach() calls: a new one would drop subscribers.
            for (const Impl::Lidar& old : s.lidars) {
                if (old.topic == lidar.topic) {
                    lidar.publisher = old.publisher;
                }
            }
            if (!lidar.publisher) {
                lidar.publisher = s.node->create_publisher<sensor_msgs::msg::LaserScan>(
                    lidar.topic, rclcpp::SensorDataQoS());
            }
            lidars.push_back(std::move(lidar));
        } else {
            Impl::Imu imu;
            imu.frame = name;
            imu.topic = imuCount == 1 ? "/imu" : "/" + name + "/imu";
            imu.force = bus.find("sensor/" + name + "/specific_force");
            imu.rate = bus.find("sensor/" + name + "/angular_rate");
            imu.orientation = bus.find("sensor/" + name + "/orientation");
            imu.accelNoise = get("accel_noise", 0.0);
            imu.gyroNoise = get("gyro_noise", 0.0);
            for (const Impl::Imu& old : s.imus) {
                if (old.topic == imu.topic) {
                    imu.publisher = old.publisher;
                }
            }
            if (!imu.publisher) {
                imu.publisher = s.node->create_publisher<sensor_msgs::msg::Imu>(
                    imu.topic, rclcpp::SensorDataQoS());
            }
            imus.push_back(std::move(imu));
        }
    }
    s.lidars = std::move(lidars);
    s.imus = std::move(imus);
    s.lastClock = SimTime{-1};
    s.lastState = SimTime{-1};
    if (!mounts.transforms.empty()) {
        s.tfStatic->publish(mounts);
        s.counters["/tf_static"].add(mounts.transforms.size() * 80);
    }
}

void Bridge::publish(const SignalBus& bus, SimTime now) {
    Impl& s = *impl_;
    if (now != s.lastClock) {
        rosgraph_msgs::msg::Clock clock;
        clock.clock = stampOf(now);
        s.clock->publish(clock);
        s.counters["/clock"].add(8);
        s.lastClock = now;
    }

    const auto period = static_cast<std::int64_t>(1e9 / settings_.stateRateHz);
    if (s.lastState.ns < 0 || now.ns - s.lastState.ns >= period) {
        s.lastState = now;
        sensor_msgs::msg::JointState joints;
        joints.header.stamp = stampOf(now);
        for (const Impl::Joint& joint : s.joints) {
            joints.name.push_back(joint.name);
            joints.position.push_back(bus.read(joint.position));
            joints.velocity.push_back(joint.velocity.valid() ? bus.read(joint.velocity) : 0.0);
            joints.effort.push_back(joint.torque.valid() ? bus.read(joint.torque) : 0.0);
        }
        s.jointStates->publish(joints);
        s.counters["/joint_states"].add(joints.name.size() * 40 + 16);

        const std::span<const double> pose = values(bus, s.basePose);
        const std::span<const double> twist = values(bus, s.baseTwist);
        if (s.odom && pose.size() == 7 && twist.size() == 6) {
            nav_msgs::msg::Odometry odom;
            odom.header.stamp = stampOf(now);
            odom.header.frame_id = settings_.odomFrame;
            odom.child_frame_id = settings_.baseLink;
            odom.pose.pose.position.x = pose[0];
            odom.pose.pose.position.y = pose[1];
            odom.pose.pose.position.z = pose[2];
            odom.pose.pose.orientation.w = pose[3];
            odom.pose.pose.orientation.x = pose[4];
            odom.pose.pose.orientation.y = pose[5];
            odom.pose.pose.orientation.z = pose[6];
            // ROS odometry gives the twist in the child frame; the bus gives it in the world.
            const std::array<double, 4> inverse = conjugate({pose[3], pose[4], pose[5], pose[6]});
            const std::array<double, 3> linear = rotate(inverse, {twist[0], twist[1], twist[2]});
            const std::array<double, 3> angular = rotate(inverse, {twist[3], twist[4], twist[5]});
            odom.twist.twist.linear.x = linear[0];
            odom.twist.twist.linear.y = linear[1];
            odom.twist.twist.linear.z = linear[2];
            odom.twist.twist.angular.x = angular[0];
            odom.twist.twist.angular.y = angular[1];
            odom.twist.twist.angular.z = angular[2];
            s.odom->publish(odom);
            s.counters["/odom"].add(720);

            tf2_msgs::msg::TFMessage tf;
            geometry_msgs::msg::TransformStamped base;
            base.header = odom.header;
            base.child_frame_id = settings_.baseLink;
            base.transform.translation.x = pose[0];
            base.transform.translation.y = pose[1];
            base.transform.translation.z = pose[2];
            base.transform.rotation = odom.pose.pose.orientation;
            tf.transforms.push_back(base);
            s.tf->publish(tf);
            s.counters["/tf"].add(80);
        }
    }

    for (Impl::Lidar& lidar : s.lidars) {
        if (!lidar.ranges.valid() || bus.stamp(lidar.ranges) == lidar.sent) {
            continue;
        }
        lidar.sent = bus.stamp(lidar.ranges);
        const std::span<const double> ranges = bus.readN(lidar.ranges);
        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = stampOf(lidar.sent);
        scan.header.frame_id = lidar.frame;
        scan.angle_min = static_cast<float>(lidar.minAngle);
        scan.angle_max = static_cast<float>(lidar.maxAngle);
        scan.angle_increment = ranges.size() > 1
                                   ? static_cast<float>((lidar.maxAngle - lidar.minAngle) /
                                                        static_cast<double>(ranges.size() - 1))
                                   : 0.0F;
        scan.scan_time = static_cast<float>(1.0 / lidar.rateHz);
        scan.range_min = static_cast<float>(lidar.minRange);
        scan.range_max = static_cast<float>(lidar.maxRange);
        scan.ranges.reserve(ranges.size());
        for (const double r : ranges) {
            scan.ranges.push_back(static_cast<float>(r));
        }
        lidar.publisher->publish(scan);
        s.counters[lidar.topic].add(ranges.size() * 4 + 64);
    }

    for (Impl::Imu& imu : s.imus) {
        if (!imu.force.valid() || bus.stamp(imu.force) == imu.sent) {
            continue;
        }
        imu.sent = bus.stamp(imu.force);
        const std::span<const double> force = bus.readN(imu.force);
        const std::span<const double> rate = bus.readN(imu.rate);
        const std::span<const double> q = bus.readN(imu.orientation);
        sensor_msgs::msg::Imu message;
        message.header.stamp = stampOf(imu.sent);
        message.header.frame_id = imu.frame;
        message.orientation.w = q[0];
        message.orientation.x = q[1];
        message.orientation.y = q[2];
        message.orientation.z = q[3];
        message.angular_velocity.x = rate[0];
        message.angular_velocity.y = rate[1];
        message.angular_velocity.z = rate[2];
        message.linear_acceleration.x = force[0];
        message.linear_acceleration.y = force[1];
        message.linear_acceleration.z = force[2];
        // Diagonal covariances from the configured white noise.
        for (std::size_t axis = 0; axis < 3; ++axis) {
            message.angular_velocity_covariance.at(axis * 4) = imu.gyroNoise * imu.gyroNoise;
            message.linear_acceleration_covariance.at(axis * 4) = imu.accelNoise * imu.accelNoise;
        }
        imu.publisher->publish(message);
        s.counters[imu.topic].add(330);
    }
}

std::optional<CmdVel> Bridge::latestCommand(std::chrono::milliseconds maxAge) const {
    const std::lock_guard<std::mutex> lock(impl_->commandMutex);
    if (!impl_->command || Clock::now() - impl_->commandAt > maxAge) {
        return std::nullopt;
    }
    return impl_->command;
}

GraphSnapshot Bridge::graph() {
    Impl& s = *impl_;
    GraphSnapshot out;
    const auto now = Clock::now();
    const double seconds =
        std::max(std::chrono::duration<double>(now - s.windowStart).count(), 1e-3);
    s.windowStart = now;
    for (const auto& [name, types] : s.node->get_topic_names_and_types()) {
        GraphSnapshot::Topic topic;
        topic.name = name;
        topic.type = types.empty() ? "" : types.front();
        topic.publishers = static_cast<std::uint32_t>(s.node->count_publishers(name));
        topic.subscribers = static_cast<std::uint32_t>(s.node->count_subscribers(name));
        const auto counter = s.counters.find(name);
        if (counter != s.counters.end()) {
            TopicCounter& c = counter->second;
            topic.fromBridge = true;
            topic.rateHz = static_cast<double>(c.messages - c.windowMessages) / seconds;
            topic.bandwidthBps = 8.0 * static_cast<double>(c.bytes - c.windowBytes) / seconds;
            c.windowMessages = c.messages;
            c.windowBytes = c.bytes;
        }
        out.topics.push_back(std::move(topic));
    }
    const auto graph = s.node->get_node_graph_interface();
    constexpr std::size_t kMaxNodes = 64;
    for (const auto& [name, space] : graph->get_node_names_and_namespaces()) {
        if (out.nodes.size() >= kMaxNodes) {
            break;
        }
        if (name.starts_with('_')) {
            continue;  // hidden by ROS convention (the ros2cli daemon, for one)
        }
        GraphSnapshot::Node node;
        node.name = (space == "/" ? "/" : space + "/") + name;
        node.external = name != settings_.nodeName;
        try {
            for (const auto& [topic, types] :
                 graph->get_publisher_names_and_types_by_node(name, space)) {
                node.publishes.push_back(topic);
            }
            for (const auto& [topic, types] :
                 graph->get_subscriber_names_and_types_by_node(name, space)) {
                node.subscribes.push_back(topic);
            }
        } catch (const std::exception&) {
            // A node that left between the two queries: list it without topics.
        }
        out.nodes.push_back(std::move(node));
    }
    out.rmw = rmw_get_implementation_identifier();
    if (const char* distro = std::getenv("ROS_DISTRO")) {
        out.distro = distro;
    }
    if (const char* domain = std::getenv("ROS_DOMAIN_ID")) {
        out.domainId = static_cast<std::uint32_t>(std::strtoul(domain, nullptr, 10));
    }
    return out;
}

}  // namespace lagrange::ros2
