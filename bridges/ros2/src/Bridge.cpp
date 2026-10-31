#include "lagrange/ros2/Bridge.hpp"
#include "lagrange/world/LidarLayout.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <future>
#include <limits>
#include <map>
#include <mutex>
#include <numbers>
#include <optional>
#include <random>
#include <regex>
#include <set>
#include <simulation_interfaces/msg/simulator_features.hpp>
#include <simulation_interfaces/srv/delete_entity.hpp>
#include <simulation_interfaces/srv/get_available_worlds.hpp>
#include <simulation_interfaces/srv/get_current_world.hpp>
#include <simulation_interfaces/srv/get_entities.hpp>
#include <simulation_interfaces/srv/get_entities_states.hpp>
#include <simulation_interfaces/srv/get_entity_bounds.hpp>
#include <simulation_interfaces/srv/get_entity_info.hpp>
#include <simulation_interfaces/srv/get_entity_state.hpp>
#include <simulation_interfaces/srv/get_named_pose_bounds.hpp>
#include <simulation_interfaces/srv/get_named_poses.hpp>
#include <simulation_interfaces/srv/get_simulation_state.hpp>
#include <simulation_interfaces/srv/get_simulator_features.hpp>
#include <simulation_interfaces/srv/get_spawnables.hpp>
#include <simulation_interfaces/srv/load_world.hpp>
#include <simulation_interfaces/srv/reset_simulation.hpp>
#include <simulation_interfaces/srv/set_entity_info.hpp>
#include <simulation_interfaces/srv/set_entity_state.hpp>
#include <simulation_interfaces/srv/set_simulation_state.hpp>
#include <simulation_interfaces/srv/spawn_entities.hpp>
#include <simulation_interfaces/srv/spawn_entity.hpp>
#include <simulation_interfaces/srv/step_simulation.hpp>
#include <simulation_interfaces/srv/unload_world.hpp>
#include <thread>
#include <variant>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
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
    // The floating origin (huge-worlds.md §1.2): poses on the bus are region-local, and the
    // region moves when the robot drives far. ROS sees them in the region of the first attach,
    // so /odom, its TF, ground truth and entity states stay continuous across a rebase.
    std::optional<std::array<double, 3>> firstRegion;
    std::array<std::atomic<double>, 3> regionOffset{};  // region now minus firstRegion, m

    [[nodiscard]] std::array<double, 3> offset() const {
        return {regionOffset[0].load(), regionOffset[1].load(), regionOffset[2].load()};
    }
    rclcpp::executors::SingleThreadedExecutor executor;
    std::thread spinner;

    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStates;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr groundTruth;  // /ground_truth/odom
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tfStatic;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr description;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointCommands;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr siteParameters;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVel;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmdVelStamped;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermannCmd;

    void take(const geometry_msgs::msg::Twist& twist) {
        take(CmdVel{twist.linear.x, twist.angular.z, twist.linear.y, twist.linear.z, {}});
    }

    void take(const CmdVel& next) {
        const std::lock_guard<std::mutex> lock(commandMutex);
        auto delay = std::chrono::nanoseconds(0);
        if (commandGate.mapped) {
            // On a radio link: lost, or late by its latency (wall clock, the command's clock).
            if (!commandGate.up || uniform(commandRng) < commandGate.loss) {
                ++commandGate.dropped;
                return;
            }
            delay = std::chrono::nanoseconds(std::llround(commandGate.latency * 1e9));
        }
        if (delay.count() > 0) {
            // The previous command stays in force until this one arrives.
            lateCommand = next;
            lateAt = Clock::now() + std::chrono::duration_cast<Clock::duration>(delay);
            return;
        }
        command = next;
        commandAt = Clock::now();
    }

    /// A topic on a radio link: the link's signals.
    struct Gate {
        SignalId loss, latency, up;
        std::uint64_t dropped{0};
    };
    std::map<std::string, Gate> gates;  // by topic
    /// A sensor's topic with sensor:latency and sensor:jitter (s): when its messages go out.
    struct SensorDelay {
        double latency{0.0}, jitter{0.0};
        SimTime last{0};  // a sensor's output stays in order, whatever the jitter
    };
    std::map<std::string, SensorDelay> sensorDelays;  // by topic
    std::mt19937_64 timingRng{0x74696d65U};
    std::normal_distribution<double> timingNoise;
    struct Pending {
        SimTime at;
        std::function<void()> send;
    };
    std::deque<Pending> pending;  // late messages, in the order they were sent
    std::mt19937_64 rng{0x6c616772U};
    std::uniform_real_distribution<double> uniform{0.0, 1.0};
    const SignalBus* gateBus{nullptr};  // during publish()
    SimTime gateNow{0};
    /// /cmd_vel's link, as of the last publish(), for the executor thread (under commandMutex).
    struct CommandGate {
        bool mapped{false};
        bool up{true};
        double loss{0.0}, latency{0.0};
        std::uint64_t dropped{0};
    };
    CommandGate commandGate;
    std::mt19937_64 commandRng{0x636d6476U};

    /// Publishes `message` on `topic`, through its radio link if it rides on one.
    template <class Publisher, class Message>
    void deliver(const std::string& topic, const Publisher& publisher, Message&& message) {
        const auto it = gates.find(topic);
        const auto timed = sensorDelays.find(topic);
        const bool gated = it != gates.end();
        if ((!gated && timed == sensorDelays.end()) || gateBus == nullptr) {
            publisher->publish(std::forward<Message>(message));
            return;
        }
        std::int64_t delay = 0;
        if (gated) {
            Gate& gate = it->second;
            if (gateBus->read(gate.up) < 0.5 || uniform(rng) < gateBus->read(gate.loss)) {
                ++gate.dropped;
                return;
            }
            delay = static_cast<std::int64_t>(std::llround(gateBus->read(gate.latency) * 1e9));
        }
        if (timed != sensorDelays.end()) {
            SensorDelay& sensor = timed->second;
            // Gaussian, cut at three sigma; never before it was sampled, nor before the last one.
            const double late =
                sensor.latency + sensor.jitter * std::clamp(timingNoise(timingRng), -3.0, 3.0);
            delay += static_cast<std::int64_t>(std::llround(std::max(late, 0.0) * 1e9));
            delay = std::max(delay, sensor.last.ns - gateNow.ns);
            sensor.last = SimTime{gateNow.ns + delay};
        }
        if (delay <= 0 && pending.empty()) {
            publisher->publish(std::forward<Message>(message));
            return;
        }
        auto held = std::make_shared<std::decay_t<Message>>(std::forward<Message>(message));
        pending.push_back({SimTime{gateNow.ns + delay},
                           [publisher, held]() { publisher->publish(std::move(*held)); }});
    }

    /// Sends the late messages that are due by `now`, oldest first.
    void flush(SimTime now) {
        std::stable_sort(pending.begin(), pending.end(),
                         [](const Pending& a, const Pending& b) { return a.at < b.at; });
        while (!pending.empty() && pending.front().at <= now) {
            pending.front().send();
            pending.pop_front();
        }
    }

    struct Joint {
        std::string name;
        SignalId position, velocity, torque;
    };
    struct Lidar {
        std::string frame;
        std::string topic;
        SignalId ranges, intensities;
        LidarLayout layout;
        double minRange{0.0}, maxRange{0.0}, rateHz{10.0};
        double sweep{0.0};  // s from the first column's firing to the scan's end + one column
        SimTime sent{-1};
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher;         // 2D
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPublisher;  // 3D
    };
    /// A camera: colour (rgb8) and depth (32FC1, m; +inf where nothing is in range, REP 118)
    /// in its optical frame, each with its CameraInfo.
    struct Camera {
        std::string frame;  // "<sensor>_optical"
        std::string ns;     // "/<sensor>"
        SignalId image, depth;
        sensor_msgs::msg::CameraInfo info;
        SimTime sent{-1};
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color, depthImage;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr colorInfo, depthInfo;
    };
    struct Imu {
        std::string frame;
        std::string topic;
        SignalId force, rate, orientation;
        double accelNoise{0.0}, gyroNoise{0.0};
        SimTime sent{-1};
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher;
    };
    // GNSS, magnetometer and barometer: /<sensor>/fix, /<sensor>/field, /<sensor>/pressure and
    // /<sensor>/temperature.
    struct Geo {
        SensorType type{SensorType::Gnss};
        std::string name;
        SignalId first, second, third, fourth;  // lat lon alt fix | field | pressure temperature
        double noise{0.0}, verticalNoise{0.0};
        SimTime sent{-1};
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix;
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr field;
        rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure;
        rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature;
    };
    std::vector<Joint> joints;
    std::vector<Lidar> lidars;
    std::vector<Camera> cameras;
    std::vector<Imu> imus;
    std::vector<Geo> geos;
    /// A radar's detections as sensor_msgs/PointCloud2 (x, y, z, velocity, rcs, snr: float32),
    /// in the radar's frame, one point per detection (domains/sensors.md).
    struct Radar {
        std::string frame;
        std::string topic;
        SignalId detections, count;
        SimTime sent{-1};
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
    };
    std::vector<Radar> radars;
    SignalId basePose, baseTwist;
    SimTime lastClock{-1};
    SimTime lastState{-1};

    mutable std::mutex commandMutex;
    std::optional<CmdVel> command;
    Clock::time_point commandAt;
    std::optional<CmdVel> lateCommand;  // on its way over a radio link
    Clock::time_point lateAt;

    std::map<std::string, TopicCounter> counters;  // by topic, the bridge's own

    /// When a scan's first column fired: the stamp of a swept scan, as LaserScan has it.
    static SimTime firstBeam(const Lidar& lidar) {
        const double before =
            lidar.sweep * (lidar.layout.horizontal - 1.0) / lidar.layout.horizontal;
        return SimTime{std::max<std::int64_t>(lidar.sent.ns - std::llround(before * 1e9), 0)};
    }

    /// A 3D lidar's scan as sensor_msgs/PointCloud2: x, y, z, intensity (float32), ring (uint16)
    /// and time (float32, s after the stamp, when that column fired), in the sensor frame,
    /// organised as rings x columns with NaN where no return.
    /// In Lyrical, PointCloud2::data is a rosidl::Buffer with the CPU backend; a CUDA backend
    /// can replace it without touching this code: docs/plan/spikes/0.2-F7-rosidl-buffer.md.
    void publishCloud(const Lidar& lidar, std::span<const double> ranges,
                      std::span<const double> intensities) {
        using sensor_msgs::msg::PointField;
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp = stampOf(firstBeam(lidar));
        cloud.header.frame_id = lidar.frame;
        cloud.height = lidar.layout.vertical;
        cloud.width = lidar.layout.horizontal;
        const auto field = [&cloud](const char* name, std::uint32_t offset, std::uint8_t type) {
            PointField f;
            f.name = name;
            f.offset = offset;
            f.datatype = type;
            f.count = 1;
            cloud.fields.push_back(f);
        };
        field("x", 0, PointField::FLOAT32);
        field("y", 4, PointField::FLOAT32);
        field("z", 8, PointField::FLOAT32);
        field("intensity", 12, PointField::FLOAT32);
        field("ring", 16, PointField::UINT16);
        field("time", 20, PointField::FLOAT32);
        constexpr std::uint32_t kStride = 24;
        cloud.point_step = kStride;
        cloud.row_step = kStride * cloud.width;
        cloud.is_dense = false;
        cloud.is_bigendian = false;
        const std::uint32_t beams = std::min<std::uint32_t>(
            lidar.layout.beams(), static_cast<std::uint32_t>(ranges.size()));
        cloud.data.assign(static_cast<std::size_t>(cloud.row_step) * cloud.height, 0);
        const float nan = std::numeric_limits<float>::quiet_NaN();
        for (std::uint32_t i = 0; i < beams; ++i) {
            const std::array<double, 3> d = lidar.layout.direction(i);
            const bool hit = std::isfinite(ranges[i]);
            const std::array<float, 4> values{
                hit ? static_cast<float>(ranges[i] * d[0]) : nan,
                hit ? static_cast<float>(ranges[i] * d[1]) : nan,
                hit ? static_cast<float>(ranges[i] * d[2]) : nan,
                hit && i < intensities.size() ? static_cast<float>(intensities[i]) : 0.0F};
            const auto ring = static_cast<std::uint16_t>(i / lidar.layout.horizontal);
            std::uint8_t* point = cloud.data.data() + static_cast<std::size_t>(i) * kStride;
            std::memcpy(point, values.data(), sizeof(values));
            std::memcpy(point + 16, &ring, sizeof(ring));
            const auto fired = static_cast<float>(lidar.sweep * (i % lidar.layout.horizontal) /
                                                  lidar.layout.horizontal);
            std::memcpy(point + 20, &fired, sizeof(fired));
        }
        counters[lidar.topic].add(cloud.data.size() + 128);
        deliver(lidar.topic, lidar.cloudPublisher, std::move(cloud));
    }

    /// The publishers and calibration of a camera, reusing publishers of an earlier attach()
    /// (a new one would drop subscribers).
    Camera camera(const std::string& name, const std::map<std::string, double, std::less<>>& set) {
        const auto get = [&set](const char* key, double fallback) {
            const auto it = set.find(key);
            return it == set.end() ? fallback : it->second;
        };
        Camera out;
        out.frame = name + "_optical";
        out.ns = "/" + name;
        // The pinhole of the camera module: square pixels, principal point at the centre.
        const double width = get("width", 640.0);
        const double height = get("height", 480.0);
        const double f = width / 2.0 / std::tan(get("horizontal_fov", 1.047) / 2.0);
        out.info.header.frame_id = out.frame;
        out.info.width = static_cast<std::uint32_t>(width);
        out.info.height = static_cast<std::uint32_t>(height);
        // The lens of the sensor model (CameraModel.hpp), in the same OpenCV order.
        const double lens = get("distortion_model", 0.0);
        out.info.distortion_model = lens == 2.0 ? "equidistant" : "plumb_bob";
        out.info.d.assign(lens == 2.0 ? 4 : 5, 0.0);
        if (lens > 0.0) {
            for (std::size_t k = 0; k < out.info.d.size(); ++k) {
                out.info.d[k] = get(("distortion_k" + std::to_string(k + 1)).c_str(), 0.0);
            }
        }
        out.info.k = {f, 0.0, (width - 1.0) / 2.0, 0.0, f, (height - 1.0) / 2.0, 0.0, 0.0, 1.0};
        out.info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        out.info.p = {
            f,   0.0, (width - 1.0) / 2.0, 0.0, 0.0, f, (height - 1.0) / 2.0, 0.0, 0.0, 0.0,
            1.0, 0.0};
        for (const Camera& old : cameras) {
            if (old.ns == out.ns) {
                out.color = old.color;
                out.depthImage = old.depthImage;
                out.colorInfo = old.colorInfo;
                out.depthInfo = old.depthInfo;
            }
        }
        if (!out.color) {
            const rclcpp::SensorDataQoS qos;
            out.color = node->create_publisher<sensor_msgs::msg::Image>(out.ns + "/image_raw", qos);
            out.colorInfo =
                node->create_publisher<sensor_msgs::msg::CameraInfo>(out.ns + "/camera_info", qos);
            out.depthImage =
                node->create_publisher<sensor_msgs::msg::Image>(out.ns + "/depth/image_raw", qos);
            out.depthInfo = node->create_publisher<sensor_msgs::msg::CameraInfo>(
                out.ns + "/depth/camera_info", qos);
        }
        return out;
    }

    /// Both images of a camera's latest frame; nothing for an empty one (the ground-truth
    /// substitute publishes only the calibration).
    void publishCamera(Camera& camera, const SignalBus& bus) {
        camera.info.header.stamp = stampOf(camera.sent);
        deliver(camera.ns + "/camera_info", camera.colorInfo, camera.info);
        deliver(camera.ns + "/depth/camera_info", camera.depthInfo, camera.info);
        counters[camera.ns + "/camera_info"].add(350);
        counters[camera.ns + "/depth/camera_info"].add(350);
        const std::shared_ptr<const Buffer> rgba = bus.buffer(bus.readBuffer(camera.image));
        if (rgba && !rgba->bytes.empty()) {
            sensor_msgs::msg::Image image;
            image.header = camera.info.header;
            image.width = rgba->width;
            image.height = rgba->height;
            image.encoding = "rgb8";
            image.step = rgba->width * 3;
            const std::size_t pixels = std::size_t{rgba->width} * rgba->height;
            image.data.assign(pixels * 3, 0);
            const auto* in = reinterpret_cast<const std::uint8_t*>(rgba->bytes.data());
            for (std::size_t p = 0; p < pixels; ++p) {
                std::memcpy(image.data.data() + p * 3, in + p * 4, 3);  // drop alpha
            }
            counters[camera.ns + "/image_raw"].add(image.data.size() + 64);
            deliver(camera.ns + "/image_raw", camera.color, std::move(image));
        }
        const std::shared_ptr<const Buffer> depth = bus.buffer(bus.readBuffer(camera.depth));
        if (depth && !depth->bytes.empty()) {
            sensor_msgs::msg::Image image;
            image.header = camera.info.header;
            image.width = depth->width;
            image.height = depth->height;
            image.encoding = "32FC1";
            image.step = depth->width * 4;
            const auto* in = reinterpret_cast<const std::uint8_t*>(depth->bytes.data());
            image.data.assign(in, in + depth->bytes.size());
            counters[camera.ns + "/depth/image_raw"].add(image.data.size() + 64);
            deliver(camera.ns + "/depth/image_raw", camera.depthImage, std::move(image));
        }
    }

    // simulation_interfaces: requests queued by the service callbacks for handleRequests().
    SimulationHandler* handler{nullptr};
    std::vector<rclcpp::ServiceBase::SharedPtr> services;
    std::mutex requestMutex;
    std::deque<std::function<void()>> requests;

    /// Runs `work` on the handleRequests() thread and returns its result, or nullopt if that
    /// thread did not get to it in time. `work` owns everything it touches, so a late run is
    /// harmless.
    template <class Work>
    auto onMain(Work work) -> std::optional<decltype(work())> {
        using Result = decltype(work());
        auto task = std::make_shared<std::packaged_task<Result()>>(std::move(work));
        std::future<Result> result = task->get_future();
        {
            const std::lock_guard<std::mutex> lock(requestMutex);
            requests.emplace_back([task] { (*task)(); });
        }
        if (result.wait_for(std::chrono::seconds(30)) != std::future_status::ready) {
            return std::nullopt;
        }
        return result.get();
    }
    Clock::time_point windowStart{Clock::now()};
};

Bridge::Bridge(BridgeSettings settings)
    : settings_(std::move(settings)), impl_(std::make_unique<Impl>()) {
    Impl& s = *impl_;
    s.node = std::make_shared<rclcpp::Node>(settings_.nodeName);
    s.clock = s.node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    if (!settings_.controllersOwnState) {
        s.jointStates = s.node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    }
    s.tf = s.node->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));
    s.tfStatic = s.node->create_publisher<tf2_msgs::msg::TFMessage>(
        "/tf_static", rclcpp::QoS(1).transient_local().reliable());
    if (settings_.controllersOwnState) {
        // diff_drive_controller (or another) takes the commands.
    } else if (settings_.stampedCmdVel) {
        s.cmdVelStamped = s.node->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel", 10,
            [&s](const geometry_msgs::msg::TwistStamped& twist) { s.take(twist.twist); });
    } else {
        s.cmdVel = s.node->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, [&s](const geometry_msgs::msg::Twist& twist) { s.take(twist); });
    }
    setRobot(settings_.baseLink, settings_.robotDescription, settings_.ackermannCmd);
    if (settings_.jointCommands) {
        s.jointCommands = s.node->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_commands", 10,
            [send = settings_.jointCommands](const sensor_msgs::msg::JointState& state) {
                for (std::size_t i = 0; i < state.name.size(); ++i) {
                    const auto given = [i](const std::vector<double>& field) {
                        return i < field.size() && std::isfinite(field[i]);
                    };
                    JointCommand command;
                    command.joint = state.name[i];
                    if (given(state.position)) {
                        command.value = state.position[i];
                    } else if (given(state.velocity)) {
                        command.mode = JointCommand::Mode::Velocity;
                        command.value = state.velocity[i];
                    } else if (given(state.effort)) {
                        command.mode = JointCommand::Mode::Effort;
                        command.value = state.effort[i];
                    } else {
                        continue;
                    }
                    send(command);
                }
            });
    }
    if (!settings_.signalTopics.publish.empty() || !settings_.signalTopics.subscribe.empty()) {
        signalTopics_ = std::make_unique<SignalTopics>(*s.node, settings_.signalTopics);
    }
    s.executor.add_node(s.node);
    s.spinner = std::thread([&s] { s.executor.spin(); });
}

void Bridge::setRobot(const std::string& baseLink, const std::string& robotDescription,
                      bool ackermannCmd) {
    Impl& s = *impl_;
    settings_.baseLink = baseLink;
    settings_.robotDescription = robotDescription;
    settings_.ackermannCmd = ackermannCmd;
    // Topics come when a robot needs them and stay: subscribers keep their connections.
    if (!baseLink.empty() && !s.groundTruth) {
        s.groundTruth = s.node->create_publisher<nav_msgs::msg::Odometry>("/ground_truth/odom", 10);
    }
    if (!baseLink.empty() && !settings_.controllersOwnState && !s.odom) {
        s.odom = s.node->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    }
    if (!robotDescription.empty()) {
        if (!s.description) {
            s.description = s.node->create_publisher<std_msgs::msg::String>(
                "/robot_description", rclcpp::QoS(1).transient_local().reliable());
        }
        std_msgs::msg::String text;
        text.data = robotDescription;
        s.description->publish(text);
    }
    if (ackermannCmd && !settings_.controllersOwnState && !s.ackermannCmd) {
        s.ackermannCmd = s.node->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/ackermann_cmd", 10, [&s](const ackermann_msgs::msg::AckermannDriveStamped& drive) {
                s.take(CmdVel{drive.drive.speed, 0.0, 0.0, 0.0, drive.drive.steering_angle});
            });
    }
}

Bridge::~Bridge() {
    impl_->executor.cancel();
    if (impl_->spinner.joinable()) {
        impl_->spinner.join();
    }
}

std::unique_ptr<IModule> Bridge::topicInputs() const {
    return signalTopics_ ? signalTopics_->inputModule() : nullptr;
}

void Bridge::attach(const World& world, const SignalBus& bus) {
    Impl& s = *impl_;
    topicProblems_ = signalTopics_ ? signalTopics_->attach(bus) : std::vector<std::string>{};
    if (!s.firstRegion) {
        s.firstRegion = world.region.worldEnu;
    }
    for (std::size_t i = 0; i < 3; ++i) {
        s.regionOffset.at(i).store(world.region.worldEnu.at(i) - s.firstRegion->at(i));
    }
    s.joints.clear();
    for (const std::string& name : world.joints.names) {
        const SignalId position = bus.find("joint/" + name + "/position");
        if (position.valid()) {
            s.joints.push_back({name, position, bus.find("joint/" + name + "/velocity"),
                                bus.find("joint/" + name + "/torque")});
        }
    }
    // Topics on radio links: the comms domain's link signals (none without it).
    s.gates.clear();
    s.pending.clear();
    s.sensorDelays.clear();
    s.timingRng.seed(0x74696d65U);
    s.timingNoise.reset();
    const auto timed = [&s](const auto& set, const std::string& topic) {
        const auto value = [&set](const char* key) {
            const auto it = set.find(key);
            return it == set.end() ? 0.0 : it->second;
        };
        if (value("latency") > 0.0 || value("jitter") > 0.0) {
            s.sensorDelays[topic] = {value("latency"), value("jitter")};
        }
    };
    s.gateBus = nullptr;
    {
        const std::lock_guard<std::mutex> lock(s.commandMutex);
        s.commandGate = {};
    }
    for (const auto& [topic, link] : settings_.topicLinks) {
        const std::string base = "link/" + link + "/";
        const Impl::Gate gate{bus.find(base + "packet_error_rate"), bus.find(base + "latency"),
                              bus.find(base + "up")};
        if (!gate.loss.valid() || !gate.latency.valid() || !gate.up.valid()) {
            RCLCPP_WARN(s.node->get_logger(),
                        "%s rides on link %s, which this run does not have (no RadioLinkAPI_1 "
                        "prim named so, or comms off): it is sent as is",
                        topic.c_str(), link.c_str());
            continue;
        }
        s.gates.emplace(topic, gate);
    }
    s.basePose = bus.find("body/" + settings_.baseLink + "/pose");
    s.baseTwist = bus.find("body/" + settings_.baseLink + "/twist");

    // Sensor frames are children of their links, at their mounts: published once, latched.
    tf2_msgs::msg::TFMessage mounts;
    const World::Sensors& sensors = world.sensors;
    // /scan, /points and /imu when the robot has one of the kind, /<sensor>/... otherwise.
    std::size_t scanners = 0;
    std::size_t clouds = 0;
    std::size_t imuCount = 0;
    for (std::size_t i = 0; i < sensors.type.size(); ++i) {
        if (sensors.type[i] == SensorType::Imu) {
            ++imuCount;
        } else if (sensors.type[i] == SensorType::Lidar) {
            (LidarLayout::from(sensors.settings[i]).is3d() ? clouds : scanners) += 1;
        }
    }
    std::vector<Impl::Lidar> lidars;
    std::vector<Impl::Camera> cameras;
    std::vector<Impl::Imu> imus;
    std::vector<Impl::Geo> geos;
    std::vector<Impl::Radar> radars;
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
            lidar.layout = LidarLayout::from(settings);
            if (lidar.layout.is3d()) {
                lidar.topic = clouds == 1 ? "/points" : "/" + name + "/points";
            } else {
                lidar.topic = scanners == 1 ? "/scan" : "/" + name + "/scan";
            }
            timed(settings, lidar.topic);
            lidar.ranges = bus.find("sensor/" + name + "/ranges");
            lidar.intensities = bus.find("sensor/" + name + "/intensities");
            lidar.minRange = get("min_range", 0.1);
            lidar.maxRange = get("max_range", 30.0);
            lidar.rateHz = sensors.rateHz[i];
            lidar.sweep = get("motion_distortion", 1.0) != 0.0 ? 1.0 / lidar.rateHz : 0.0;
            // Reuse the publisher across attach() calls: a new one would drop subscribers.
            for (const Impl::Lidar& old : s.lidars) {
                if (old.topic == lidar.topic) {
                    lidar.publisher = old.publisher;
                    lidar.cloudPublisher = old.cloudPublisher;
                }
            }
            if (lidar.layout.is3d() && !lidar.cloudPublisher) {
                lidar.cloudPublisher = s.node->create_publisher<sensor_msgs::msg::PointCloud2>(
                    lidar.topic, rclcpp::SensorDataQoS());
            } else if (!lidar.layout.is3d() && !lidar.publisher) {
                lidar.publisher = s.node->create_publisher<sensor_msgs::msg::LaserScan>(
                    lidar.topic, rclcpp::SensorDataQoS());
            }
            lidars.push_back(std::move(lidar));
        } else if (sensors.type[i] == SensorType::Camera) {
            for (const char* topic :
                 {"/camera_info", "/depth/camera_info", "/image_raw", "/depth/image_raw"}) {
                timed(settings, "/" + name + topic);
            }
            cameras.push_back(s.camera(name, settings));
            geometry_msgs::msg::TransformStamped optical;
            optical.header.frame_id = name;
            optical.child_frame_id = cameras.back().frame;
            optical.transform.rotation.w = kSensorFromOptical[0];
            optical.transform.rotation.x = kSensorFromOptical[1];
            optical.transform.rotation.y = kSensorFromOptical[2];
            optical.transform.rotation.z = kSensorFromOptical[3];
            mounts.transforms.push_back(optical);
            cameras.back().image = bus.find("sensor/" + name + "/image");
            cameras.back().depth = bus.find("sensor/" + name + "/depth");
        } else if (sensors.type[i] == SensorType::Gnss ||
                   sensors.type[i] == SensorType::Magnetometer ||
                   sensors.type[i] == SensorType::Barometer) {
            Impl::Geo geo;
            geo.type = sensors.type[i];
            geo.name = name;
            for (const char* topic : {"/fix", "/field", "/pressure", "/temperature"}) {
                timed(settings, "/" + name + topic);
            }
            const std::string base = "sensor/" + name + "/";
            for (const Impl::Geo& old : s.geos) {
                if (old.name == name) {  // keep subscribers across attach() calls
                    geo.fix = old.fix;
                    geo.field = old.field;
                    geo.pressure = old.pressure;
                    geo.temperature = old.temperature;
                }
            }
            const auto qos = rclcpp::SensorDataQoS();
            if (geo.type == SensorType::Gnss) {
                geo.first = bus.find(base + "latitude");
                geo.second = bus.find(base + "longitude");
                geo.third = bus.find(base + "altitude");
                geo.fourth = bus.find(base + "fix");
                geo.noise = get("horizontal_noise", 0.0);
                geo.verticalNoise = get("vertical_noise", 0.0);
                if (!geo.fix) {
                    geo.fix = s.node->create_publisher<sensor_msgs::msg::NavSatFix>(
                        "/" + name + "/fix", qos);
                }
            } else if (geo.type == SensorType::Magnetometer) {
                geo.first = bus.find(base + "field");
                geo.noise = get("noise", 0.0);
                if (!geo.field) {
                    geo.field = s.node->create_publisher<sensor_msgs::msg::MagneticField>(
                        "/" + name + "/field", qos);
                }
            } else {
                geo.first = bus.find(base + "pressure");
                geo.second = bus.find(base + "temperature");
                geo.noise = get("noise", 0.0);
                if (!geo.pressure) {
                    geo.pressure = s.node->create_publisher<sensor_msgs::msg::FluidPressure>(
                        "/" + name + "/pressure", qos);
                    geo.temperature = s.node->create_publisher<sensor_msgs::msg::Temperature>(
                        "/" + name + "/temperature", qos);
                }
            }
            geos.push_back(std::move(geo));
        } else if (sensors.type[i] == SensorType::Radar) {
            Impl::Radar radar;
            radar.frame = name;
            radar.topic = "/" + name + "/detections";
            timed(settings, radar.topic);
            radar.detections = bus.find("sensor/" + name + "/detections");
            radar.count = bus.find("sensor/" + name + "/count");
            for (const Impl::Radar& old : s.radars) {
                if (old.topic == radar.topic) {
                    radar.publisher = old.publisher;
                }
            }
            if (!radar.publisher) {
                radar.publisher = s.node->create_publisher<sensor_msgs::msg::PointCloud2>(
                    radar.topic, rclcpp::SensorDataQoS());
            }
            radars.push_back(std::move(radar));
        } else if (sensors.type[i] == SensorType::Imu) {
            Impl::Imu imu;
            imu.frame = name;
            imu.topic = imuCount == 1 ? "/imu" : "/" + name + "/imu";
            timed(settings, imu.topic);
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
    s.cameras = std::move(cameras);
    s.imus = std::move(imus);
    s.geos = std::move(geos);
    s.radars = std::move(radars);
    s.lastClock = SimTime{-1};
    s.lastState = SimTime{-1};
    if (!mounts.transforms.empty()) {
        s.tfStatic->publish(mounts);
        s.counters["/tf_static"].add(mounts.transforms.size() * 80);
    }
}

void Bridge::publish(const SignalBus& bus, SimTime now) {
    Impl& s = *impl_;
    s.gateBus = &bus;
    s.gateNow = now;
    s.flush(now);
    if (const auto it = s.gates.find("/cmd_vel"); it != s.gates.end()) {
        const std::lock_guard<std::mutex> lock(s.commandMutex);
        s.commandGate.mapped = true;
        s.commandGate.up = bus.read(it->second.up) > 0.5;
        s.commandGate.loss = bus.read(it->second.loss);
        s.commandGate.latency = bus.read(it->second.latency);
    }
    if (signalTopics_) {
        signalTopics_->publish(bus, now);
    }
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
        if (s.jointStates) {
            s.deliver("/joint_states", s.jointStates, joints);
            s.counters["/joint_states"].add(joints.name.size() * 40 + 16);
        }

        const std::span<const double> pose = values(bus, s.basePose);
        const std::span<const double> twist = values(bus, s.baseTwist);
        if (s.groundTruth && pose.size() == 7 && twist.size() == 6) {
            nav_msgs::msg::Odometry odom;
            odom.header.stamp = stampOf(now);
            odom.header.frame_id = settings_.odomFrame;
            odom.child_frame_id = settings_.baseLink;
            const std::array<double, 3> offset = s.offset();
            odom.pose.pose.position.x = pose[0] + offset[0];
            odom.pose.pose.position.y = pose[1] + offset[1];
            odom.pose.pose.position.z = pose[2] + offset[2];
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
            // The true pose, always, in the world frame (for tests and evaluation, as a
            // simulator's ground-truth plugin publishes it).
            nav_msgs::msg::Odometry truth = odom;
            truth.header.frame_id = "world";
            s.deliver("/ground_truth/odom", s.groundTruth, truth);
            s.counters["/ground_truth/odom"].add(720);
            // With ros2_control, its controllers publish /odom and the odom TF.
            if (s.odom) {
                s.deliver("/odom", s.odom, odom);
                s.counters["/odom"].add(720);

                tf2_msgs::msg::TFMessage tf;
                geometry_msgs::msg::TransformStamped base;
                base.header = odom.header;
                base.child_frame_id = settings_.baseLink;
                base.transform.translation.x = odom.pose.pose.position.x;
                base.transform.translation.y = odom.pose.pose.position.y;
                base.transform.translation.z = odom.pose.pose.position.z;
                base.transform.rotation = odom.pose.pose.orientation;
                tf.transforms.push_back(base);
                s.deliver("/tf", s.tf, tf);
                s.counters["/tf"].add(80);
            }
        }
    }

    for (Impl::Lidar& lidar : s.lidars) {
        if (!lidar.ranges.valid() || bus.stamp(lidar.ranges) == lidar.sent) {
            continue;
        }
        lidar.sent = bus.stamp(lidar.ranges);
        const std::span<const double> ranges = bus.readN(lidar.ranges);
        if (lidar.cloudPublisher) {
            s.publishCloud(lidar, ranges, values(bus, lidar.intensities));
            continue;
        }
        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = stampOf(Impl::firstBeam(lidar));
        scan.header.frame_id = lidar.frame;
        scan.time_increment = static_cast<float>(lidar.sweep / lidar.layout.horizontal);
        scan.angle_min = static_cast<float>(lidar.layout.minAngle);
        scan.angle_max = static_cast<float>(lidar.layout.maxAngle);
        scan.angle_increment =
            ranges.size() > 1 ? static_cast<float>((lidar.layout.maxAngle - lidar.layout.minAngle) /
                                                   static_cast<double>(ranges.size() - 1))
                              : 0.0F;
        scan.scan_time = static_cast<float>(1.0 / lidar.rateHz);
        scan.range_min = static_cast<float>(lidar.minRange);
        scan.range_max = static_cast<float>(lidar.maxRange);
        scan.ranges.reserve(ranges.size());
        for (const double r : ranges) {
            scan.ranges.push_back(static_cast<float>(r));
        }
        if (lidar.intensities.valid()) {
            const std::span<const double> intensities = bus.readN(lidar.intensities);
            scan.intensities.assign(intensities.begin(), intensities.end());
        }
        s.deliver(lidar.topic, lidar.publisher, scan);
        s.counters[lidar.topic].add(ranges.size() * 4 + 64);
    }

    for (Impl::Camera& camera : s.cameras) {
        if (!camera.image.valid() || bus.stamp(camera.image) == camera.sent) {
            continue;
        }
        camera.sent = bus.stamp(camera.image);
        s.publishCamera(camera, bus);
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
        s.deliver(imu.topic, imu.publisher, message);
        s.counters[imu.topic].add(330);
    }

    for (Impl::Radar& radar : s.radars) {
        if (!radar.detections.valid() || bus.stamp(radar.detections) == radar.sent) {
            continue;
        }
        radar.sent = bus.stamp(radar.detections);
        using sensor_msgs::msg::PointField;
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp = stampOf(radar.sent);
        cloud.header.frame_id = radar.frame;
        constexpr std::array<const char*, 6> kFields{"x", "y", "z", "velocity", "rcs", "snr"};
        for (std::uint32_t f = 0; f < kFields.size(); ++f) {
            PointField field;
            field.name = kFields[f];
            field.offset = 4 * f;
            field.datatype = PointField::FLOAT32;
            field.count = 1;
            cloud.fields.push_back(field);
        }
        const std::span<const double> d = bus.readN(radar.detections);
        const auto n = static_cast<std::uint32_t>(bus.read(radar.count));
        cloud.height = 1;
        cloud.width = n;
        cloud.point_step = 4 * kFields.size();
        cloud.row_step = cloud.point_step * n;
        cloud.is_dense = true;
        cloud.is_bigendian = false;
        cloud.data.assign(static_cast<std::size_t>(cloud.row_step), 0);
        for (std::uint32_t i = 0; i < n; ++i) {
            // range, azimuth, elevation, radial velocity, rcs, snr → x, y, z, ...
            const double r = d[6 * i];
            const double az = d[6 * i + 1];
            const double el = d[6 * i + 2];
            const std::array<float, 6> values{static_cast<float>(r * std::cos(el) * std::cos(az)),
                                              static_cast<float>(r * std::cos(el) * std::sin(az)),
                                              static_cast<float>(r * std::sin(el)),
                                              static_cast<float>(d[6 * i + 3]),
                                              static_cast<float>(d[6 * i + 4]),
                                              static_cast<float>(d[6 * i + 5])};
            std::memcpy(cloud.data.data() + static_cast<std::size_t>(i) * cloud.point_step,
                        values.data(), sizeof(values));
        }
        s.deliver(radar.topic, radar.publisher, std::move(cloud));
        s.counters[radar.topic].add(24 * n + 64);
    }

    for (Impl::Geo& geo : s.geos) {
        // Not running without the site's geo origin: no signals, nothing to publish.
        if (!geo.first.valid() || bus.stamp(geo.first) == geo.sent) {
            continue;
        }
        geo.sent = bus.stamp(geo.first);
        const auto stamp = stampOf(geo.sent);
        if (geo.type == SensorType::Gnss) {
            constexpr double kDegrees = 180.0 / std::numbers::pi;
            sensor_msgs::msg::NavSatFix fix;
            fix.header.stamp = stamp;
            fix.header.frame_id = geo.name;
            fix.latitude = bus.read(geo.first) * kDegrees;
            fix.longitude = bus.read(geo.second) * kDegrees;
            fix.altitude = bus.read(geo.third);  // above the WGS84 ellipsoid, as the message says
            fix.status.status = bus.read(geo.fourth) >= 2.0
                                    ? sensor_msgs::msg::NavSatStatus::STATUS_FIX
                                    : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
            fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
            fix.position_covariance.at(0) = geo.noise * geo.noise;
            fix.position_covariance.at(4) = geo.noise * geo.noise;
            fix.position_covariance.at(8) = geo.verticalNoise * geo.verticalNoise;
            fix.position_covariance_type =
                sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
            s.deliver("/" + geo.name + "/fix", geo.fix, fix);
            s.counters["/" + geo.name + "/fix"].add(120);
        } else if (geo.type == SensorType::Magnetometer) {
            const std::span<const double> b = bus.readN(geo.first);
            sensor_msgs::msg::MagneticField field;
            field.header.stamp = stamp;
            field.header.frame_id = geo.name;
            field.magnetic_field.x = b[0];
            field.magnetic_field.y = b[1];
            field.magnetic_field.z = b[2];
            for (std::size_t axis = 0; axis < 3; ++axis) {
                field.magnetic_field_covariance.at(axis * 4) = geo.noise * geo.noise;
            }
            s.deliver("/" + geo.name + "/field", geo.field, field);
            s.counters["/" + geo.name + "/field"].add(100);
        } else {
            sensor_msgs::msg::FluidPressure pressure;
            pressure.header.stamp = stamp;
            pressure.header.frame_id = geo.name;
            pressure.fluid_pressure = bus.read(geo.first);
            pressure.variance = geo.noise * geo.noise;
            s.deliver("/" + geo.name + "/pressure", geo.pressure, pressure);
            sensor_msgs::msg::Temperature temperature;
            temperature.header = pressure.header;
            constexpr double kCelsiusZero = 273.15;  // K
            temperature.temperature = bus.read(geo.second) - kCelsiusZero;
            s.deliver("/" + geo.name + "/temperature", geo.temperature, temperature);
            s.counters["/" + geo.name + "/pressure"].add(40);
            s.counters["/" + geo.name + "/temperature"].add(40);
        }
    }
}

std::map<std::string, std::uint64_t> Bridge::droppedOnLinks() const {
    std::map<std::string, std::uint64_t> out;
    for (const auto& [topic, gate] : impl_->gates) {
        out[topic] = gate.dropped;
    }
    const std::lock_guard<std::mutex> lock(impl_->commandMutex);
    if (impl_->commandGate.mapped) {
        out["/cmd_vel"] = impl_->commandGate.dropped;
    }
    return out;
}

std::optional<CmdVel> Bridge::latestCommand(std::chrono::milliseconds maxAge) const {
    const std::lock_guard<std::mutex> lock(impl_->commandMutex);
    const Clock::time_point now = Clock::now();
    if (impl_->lateCommand && now >= impl_->lateAt) {
        impl_->command = impl_->lateCommand;
        impl_->commandAt = impl_->lateAt;
        impl_->lateCommand.reset();
    }
    if (!impl_->command || now - impl_->commandAt > maxAge) {
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

namespace {

namespace si = simulation_interfaces;

void setResult(si::msg::Result& out, const SimOutcome& outcome) {
    out.result = static_cast<std::uint8_t>(outcome.result);
    out.error_message = outcome.message;
}

void timedOut(si::msg::Result& out) {
    out.result = si::msg::Result::RESULT_OPERATION_FAILED;
    out.error_message = "the simulator did not answer within 30 s";
}

/// Entities that pass `filters`, or an outcome saying why the filters cannot be applied.
std::variant<std::vector<EntityDescription>, SimOutcome> filtered(
    std::vector<EntityDescription> all, const si::msg::EntityFilters& filters) {
    if (!filters.tags.tags.empty()) {
        return SimOutcome{SimResult::Unsupported, "entities have no tags in this simulator"};
    }
    if (filters.bounds.type != si::msg::Bounds::TYPE_EMPTY) {
        return SimOutcome{SimResult::Unsupported, "filtering by bounds is not supported"};
    }
    std::optional<std::regex> pattern;
    if (!filters.filter.empty()) {
        try {
            pattern.emplace(filters.filter, std::regex::extended);
        } catch (const std::regex_error& e) {
            return SimOutcome{SimResult::Failed,
                              "invalid filter '" + filters.filter + "': " + e.what()};
        }
    }
    std::vector<EntityDescription> out;
    for (EntityDescription& entity : all) {
        const bool nameOk = !pattern || std::regex_search(entity.name, *pattern);
        const bool categoryOk =
            filters.categories.empty() ||
            std::any_of(filters.categories.begin(), filters.categories.end(),
                        [&](const si::msg::EntityCategory& c) {
                            return c.category == static_cast<std::uint8_t>(entity.category);
                        });
        if (nameOk && categoryOk) {
            out.push_back(std::move(entity));
        }
    }
    return out;
}

/// A service this simulator does not support yet: it answers RESULT_FEATURE_UNSUPPORTED rather
/// than leaving callers waiting for a server that never appears.
template <class Srv>
rclcpp::ServiceBase::SharedPtr unsupported(rclcpp::Node& node, const std::string& name,
                                           const std::string& why) {
    return node.create_service<Srv>(name,
                                    [why](const std::shared_ptr<typename Srv::Request>&,
                                          const std::shared_ptr<typename Srv::Response>& response) {
                                        setResult(response->result, {SimResult::Unsupported, why});
                                    });
}

/// From a SpawnEntity request, or one of SpawnEntities' (the same fields).
template <typename Message>
SpawnRequest spawnRequest(const Message& in) {
    SpawnRequest out;
    out.name = in.name;
    out.allowRenaming = in.allow_renaming;
    out.uri = in.entity_resource.uri;
    out.resource = in.entity_resource.resource_string;
    out.entityNamespace = in.entity_namespace;
    out.frame = in.initial_pose.header.frame_id;
    const geometry_msgs::msg::Pose& p = in.initial_pose.pose;
    out.pose.position = {p.position.x, p.position.y, p.position.z};
    out.pose.orientation = {p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z};
    return out;
}

si::msg::EntityState stateMessage(const EntityMotion& motion, SimTime now,
                                  const std::array<double, 3>& offset) {
    si::msg::EntityState state;
    state.header.stamp = stampOf(now);
    state.header.frame_id = "world";
    state.pose.position.x = motion.pose.position[0] + offset[0];
    state.pose.position.y = motion.pose.position[1] + offset[1];
    state.pose.position.z = motion.pose.position[2] + offset[2];
    state.pose.orientation.w = motion.pose.orientation[0];
    state.pose.orientation.x = motion.pose.orientation[1];
    state.pose.orientation.y = motion.pose.orientation[2];
    state.pose.orientation.z = motion.pose.orientation[3];
    state.twist.linear.x = motion.twist[0];
    state.twist.linear.y = motion.twist[1];
    state.twist.linear.z = motion.twist[2];
    state.twist.angular.x = motion.twist[3];
    state.twist.angular.y = motion.twist[4];
    state.twist.angular.z = motion.twist[5];
    state.acceleration.linear.x = motion.acceleration[0];
    state.acceleration.linear.y = motion.acceleration[1];
    state.acceleration.linear.z = motion.acceleration[2];
    state.acceleration.angular.x = motion.acceleration[3];
    state.acceleration.angular.y = motion.acceleration[4];
    state.acceleration.angular.z = motion.acceleration[5];
    return state;
}

}  // namespace

void Bridge::serveSimulation(SimulationHandler& handler) {
    Impl& s = *impl_;
    s.handler = &handler;
    SimulationHandler* h = &handler;
    rclcpp::Node& node = *s.node;
    using si::msg::SimulatorFeatures;

    // The site, as parameters: ros2 param set /lagrange_ros2_bridge gravity 1.62 (lagrange set).
    // NaN: as the run started.
    for (const char* name : {"gravity", "wind_speed", "wind_from", "temperature", "air_density",
                             "pressure", "relative_humidity", "rain_rate", "visibility"}) {
        node.declare_parameter(name, std::numeric_limits<double>::quiet_NaN());
    }
    s.siteParameters = node.add_on_set_parameters_callback(
        [&s, h](const std::vector<rclcpp::Parameter>& parameters) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const rclcpp::Parameter& parameter : parameters) {
                if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
                    std::isnan(parameter.as_double())) {
                    continue;
                }
                const std::string name = parameter.get_name();
                const double value = parameter.as_double();
                const auto outcome = s.onMain([h, name, value] { return h->setSite(name, value); });
                if (!outcome || outcome->result != SimResult::Ok) {
                    result.successful = false;
                    result.reason = outcome ? outcome->message : "the simulator did not answer";
                    break;
                }
            }
            return result;
        });

    s.services.push_back(node.create_service<si::srv::GetSimulatorFeatures>(
        "get_simulator_features",
        [](const std::shared_ptr<si::srv::GetSimulatorFeatures::Request>&,
           const std::shared_ptr<si::srv::GetSimulatorFeatures::Response>& response) {
            response->features.features = {
                SimulatorFeatures::ENTITY_CATEGORIES,
                SimulatorFeatures::ENTITY_STATE_GETTING,
                SimulatorFeatures::ENTITY_INFO_GETTING,
                SimulatorFeatures::SIMULATION_RESET,
                SimulatorFeatures::SIMULATION_STATE_GETTING,
                SimulatorFeatures::SIMULATION_STATE_SETTING,
                SimulatorFeatures::SIMULATION_STATE_PAUSE,
                SimulatorFeatures::STEP_SIMULATION_SINGLE,
                SimulatorFeatures::STEP_SIMULATION_MULTIPLE,
                SimulatorFeatures::SPAWNING,
                SimulatorFeatures::SPAWNING_RESOURCE_STRING,
                SimulatorFeatures::SPAWNING_ENTITIES,
                SimulatorFeatures::DELETING,
                SimulatorFeatures::SIMULATION_RESET_SPAWNED,
            };
            response->features.spawn_formats = {"urdf", "sdf", "mjcf", "usd"};
            response->features.custom_info =
                "Lagrange: entities are the robots and the fixed scenery objects; a step is one "
                "base tick of the scheduler; reset restarts time and state together. A robot "
                "spawned into a run without one is the robot ROS drives; later ones are passive. "
                "Spawning or deleting rebuilds the run: time and poses go on, velocities start "
                "from rest.";
        }));

    s.services.push_back(node.create_service<si::srv::GetSimulationState>(
        "get_simulation_state",
        [&s, h](const std::shared_ptr<si::srv::GetSimulationState::Request>&,
                const std::shared_ptr<si::srv::GetSimulationState::Response>& response) {
            const auto state = s.onMain([h] { return h->simulationState(); });
            if (!state) {
                timedOut(response->result);
                return;
            }
            response->state.state = static_cast<std::uint8_t>(*state);
            response->result.result = si::msg::Result::RESULT_OK;
        }));

    s.services.push_back(node.create_service<si::srv::SetSimulationState>(
        "set_simulation_state",
        [&s, h](const std::shared_ptr<si::srv::SetSimulationState::Request>& request,
                const std::shared_ptr<si::srv::SetSimulationState::Response>& response) {
            const std::uint8_t target = request->state.state;
            if (target > static_cast<std::uint8_t>(SimState::Quitting)) {
                setResult(response->result,
                          {SimResult::Unsupported, "worlds cannot be loaded or unloaded"});
                return;
            }
            const auto outcome = s.onMain(
                [h, target] { return h->setSimulationState(static_cast<SimState>(target)); });
            outcome ? setResult(response->result, *outcome) : timedOut(response->result);
        }));

    s.services.push_back(node.create_service<si::srv::StepSimulation>(
        "step_simulation",
        [&s, h](const std::shared_ptr<si::srv::StepSimulation::Request>& request,
                const std::shared_ptr<si::srv::StepSimulation::Response>& response) {
            const std::uint64_t steps = request->steps;
            const auto outcome = s.onMain([h, steps] { return h->step(steps); });
            outcome ? setResult(response->result, *outcome) : timedOut(response->result);
        }));

    s.services.push_back(node.create_service<si::srv::ResetSimulation>(
        "reset_simulation",
        [&s, h](const std::shared_ptr<si::srv::ResetSimulation::Request>& request,
                const std::shared_ptr<si::srv::ResetSimulation::Response>& response) {
            using Reset = si::srv::ResetSimulation::Request;
            const std::uint8_t scope = request->scope;
            const std::uint8_t timeAndState = Reset::SCOPE_TIME | Reset::SCOPE_STATE;
            if (scope == Reset::SCOPE_SPAWNED) {
                const auto outcome = s.onMain([h] { return h->removeSpawned(); });
                outcome ? setResult(response->result, *outcome) : timedOut(response->result);
                return;
            }
            if (scope != Reset::SCOPE_DEFAULT && (scope & timeAndState) != timeAndState) {
                setResult(response->result,
                          {SimResult::Unsupported, "time and state can only be reset together"});
                return;
            }
            const auto outcome = s.onMain([h] { return h->reset(); });
            outcome ? setResult(response->result, *outcome) : timedOut(response->result);
        }));

    s.services.push_back(node.create_service<si::srv::GetEntities>(
        "get_entities", [&s, h](const std::shared_ptr<si::srv::GetEntities::Request>& request,
                                const std::shared_ptr<si::srv::GetEntities::Response>& response) {
            const auto all = s.onMain([h] { return h->entities(); });
            if (!all) {
                timedOut(response->result);
                return;
            }
            auto matched = filtered(*all, request->filters);
            if (const auto* failed = std::get_if<SimOutcome>(&matched)) {
                setResult(response->result, *failed);
                return;
            }
            for (const EntityDescription& entity : std::get<0>(matched)) {
                response->entities.push_back(entity.name);
            }
            response->result.result = si::msg::Result::RESULT_OK;
        }));

    s.services.push_back(node.create_service<si::srv::GetEntitiesStates>(
        "get_entities_states",
        [&s, h, this](const std::shared_ptr<si::srv::GetEntitiesStates::Request>& request,
                      const std::shared_ptr<si::srv::GetEntitiesStates::Response>& response) {
            si::msg::EntityFilters filters = request->filters;
            using States = std::pair<std::vector<std::string>, std::vector<si::msg::EntityState>>;
            const auto result = s.onMain([h, this, filters]() -> std::variant<States, SimOutcome> {
                auto matched = filtered(h->entities(), filters);
                if (auto* failed = std::get_if<SimOutcome>(&matched)) {
                    return *failed;
                }
                States states;
                for (const EntityDescription& entity : std::get<0>(matched)) {
                    if (const auto motion = h->entityMotion(entity.name)) {
                        states.first.push_back(entity.name);
                        states.second.push_back(
                            stateMessage(*motion, impl_->lastClock, impl_->offset()));
                    }
                }
                return states;
            });
            if (!result) {
                timedOut(response->result);
            } else if (const auto* failed = std::get_if<SimOutcome>(&*result)) {
                setResult(response->result, *failed);
            } else {
                const States& states = std::get<States>(*result);
                response->entities = states.first;
                response->states = states.second;
                response->result.result = si::msg::Result::RESULT_OK;
            }
        }));

    s.services.push_back(node.create_service<si::srv::GetEntityState>(
        "get_entity_state",
        [&s, h, this](const std::shared_ptr<si::srv::GetEntityState::Request>& request,
                      const std::shared_ptr<si::srv::GetEntityState::Response>& response) {
            const std::string name = request->entity;
            const auto motion = s.onMain([h, this, name]() -> std::optional<si::msg::EntityState> {
                if (const auto found = h->entityMotion(name)) {
                    return stateMessage(*found, impl_->lastClock, impl_->offset());
                }
                return std::nullopt;
            });
            if (!motion) {
                timedOut(response->result);
            } else if (!*motion) {
                setResult(response->result,
                          {SimResult::NotFound, "no entity named '" + name + "'"});
            } else {
                response->state = **motion;
                response->result.result = si::msg::Result::RESULT_OK;
            }
        }));

    s.services.push_back(node.create_service<si::srv::GetEntityInfo>(
        "get_entity_info",
        [&s, h](const std::shared_ptr<si::srv::GetEntityInfo::Request>& request,
                const std::shared_ptr<si::srv::GetEntityInfo::Response>& response) {
            const std::string name = request->entity;
            const auto all = s.onMain([h] { return h->entities(); });
            if (!all) {
                timedOut(response->result);
                return;
            }
            for (const EntityDescription& entity : *all) {
                if (entity.name == name) {
                    response->info.category.category = static_cast<std::uint8_t>(entity.category);
                    response->info.description = entity.description;
                    response->result.result = si::msg::Result::RESULT_OK;
                    return;
                }
            }
            setResult(response->result, {SimResult::NotFound, "no entity named '" + name + "'"});
        }));
    s.services.push_back(node.create_service<si::srv::SpawnEntity>(
        "spawn_entity", [&s, h](const std::shared_ptr<si::srv::SpawnEntity::Request>& request,
                                const std::shared_ptr<si::srv::SpawnEntity::Response>& response) {
            const SpawnRequest asked = spawnRequest(*request);
            const auto spawned = s.onMain([h, asked] { return h->spawn(asked); });
            if (!spawned) {
                timedOut(response->result);
                return;
            }
            setResult(response->result, spawned->outcome);
            response->entity_name = spawned->entityName;
        }));
    s.services.push_back(node.create_service<si::srv::SpawnEntities>(
        "spawn_entities",
        [&s, h](const std::shared_ptr<si::srv::SpawnEntities::Request>& request,
                const std::shared_ptr<si::srv::SpawnEntities::Response>& response) {
            bool failed = false;
            for (const auto& one : request->spawn_requests) {
                const SpawnRequest asked = spawnRequest(one);
                const auto spawned = s.onMain([h, asked] { return h->spawn(asked); });
                si::msg::SpawnResult& result = response->results.emplace_back();
                if (!spawned) {
                    timedOut(result.result);
                } else {
                    setResult(result.result, spawned->outcome);
                    result.entity_name = spawned->entityName;
                }
                failed = failed || result.result.result != si::msg::Result::RESULT_OK;
            }
            response->result.result = failed
                                          ? si::srv::SpawnEntities::Response::ENTITIES_SPAWN_FAILED
                                          : si::msg::Result::RESULT_OK;
        }));
    s.services.push_back(node.create_service<si::srv::DeleteEntity>(
        "delete_entity", [&s, h](const std::shared_ptr<si::srv::DeleteEntity::Request>& request,
                                 const std::shared_ptr<si::srv::DeleteEntity::Response>& response) {
            const std::string name = request->entity;
            const auto outcome = s.onMain([h, name] { return h->remove(name); });
            outcome ? setResult(response->result, *outcome) : timedOut(response->result);
        }));
    const std::string noWorlds = "this server runs the world it was started with";
    const std::string noEditing = "entity state and info are read-only in this version";
    s.services.push_back(unsupported<si::srv::GetSpawnables>(
        node, "get_spawnables", "spawn any URDF, SDFormat, MJCF or USD file by its path"));
    s.services.push_back(unsupported<si::srv::SetEntityState>(node, "set_entity_state", noEditing));
    s.services.push_back(unsupported<si::srv::SetEntityInfo>(node, "set_entity_info", noEditing));
    s.services.push_back(unsupported<si::srv::GetEntityBounds>(
        node, "get_entity_bounds", "entity bounds are not reported yet"));
    s.services.push_back(
        unsupported<si::srv::GetNamedPoses>(node, "get_named_poses", "no named poses are defined"));
    s.services.push_back(unsupported<si::srv::GetNamedPoseBounds>(node, "get_named_pose_bounds",
                                                                  "no named poses are defined"));
    s.services.push_back(unsupported<si::srv::LoadWorld>(node, "load_world", noWorlds));
    s.services.push_back(unsupported<si::srv::UnloadWorld>(node, "unload_world", noWorlds));
    s.services.push_back(
        unsupported<si::srv::GetCurrentWorld>(node, "get_current_world", noWorlds));
    s.services.push_back(
        unsupported<si::srv::GetAvailableWorlds>(node, "get_available_worlds", noWorlds));
}

void Bridge::handleRequests() {
    Impl& s = *impl_;
    std::deque<std::function<void()>> pending;
    {
        const std::lock_guard<std::mutex> lock(s.requestMutex);
        pending.swap(s.requests);
    }
    for (const std::function<void()>& request : pending) {
        request();
    }
}

}  // namespace lagrange::ros2
