// The custom sensors' default topics (KindTopics.hpp, sensors.md §9 "ROS topics of the built-in
// kinds") over a real DDS domain (ROS_DOMAIN_ID 77, localhost): a fake module writes the signals a
// kind would, the bridge publishes them with no --ros-topics map, and a test node listens.
#include "lagrange/ros2/Bridge.hpp"
#include "lagrange/ros2/KindTopics.hpp"
#include "lagrange/scheduler/Scheduler.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

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

using namespace literals;
using namespace std::chrono_literals;

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

/// A signal a fake kind writes, the same value every step.
struct Fake {
    std::string name;
    SignalType type{SignalType::Scalar};
    Unit unit{units::one};
    std::vector<double> values;
    std::string frame;
};

/// Writes the fake signals every 10 ms.
class FakeKinds final : public IModule {
public:
    explicit FakeKinds(std::vector<Fake> fakes) : fakes_(std::move(fakes)) {}
    [[nodiscard]] ModuleInfo info() const override { return {"sensors", "fake_kinds"}; }
    [[nodiscard]] Duration preferredStep() const override { return 10_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        ids_.clear();
        for (const Fake& fake : fakes_) {
            SignalSpec spec;
            spec.name = fake.name;
            spec.type = fake.type;
            spec.unit = fake.unit;
            spec.frame = fake.frame;
            spec.arrayLength = static_cast<std::uint32_t>(fake.values.size());
            ids_.push_back(d.output(spec));
        }
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        return step(StepContext{}, bus);
    }
    Status step(const StepContext&, SignalBus& bus) override {
        for (std::size_t i = 0; i < fakes_.size(); ++i) {
            bus.writeN(ids_[i], fakes_[i].values);
        }
        return {};
    }

private:
    std::vector<Fake> fakes_;
    std::vector<SignalId> ids_;
};

/// A world of custom sensors on one base body (and a wheel with a joint), the fake signals on a
/// scheduler, and the bridge.
class KindTopicsTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
    }

    void SetUp() override {
        base_ = world_.addBody("base_link", kWorldBody, Pose{}, 1.0, {}, {});
        wheel_ = world_.addBody("left_wheel_link", base_, Pose{}, 1.0, {}, {});
        world_.addJoint("left_wheel", JointType::Continuous, wheel_, {0, 1, 0}, true);
        listener_ = std::make_shared<rclcpp::Node>("kind_topics_listener");
        executor_.add_node(listener_);
    }

    void start(std::vector<Fake> fakes, TopicMaps maps = {}) {
        scheduler_.add(std::make_unique<FakeKinds>(std::move(fakes)));
        ASSERT_TRUE(scheduler_.wire(WorldView(world_)));
        BridgeSettings settings;
        settings.signalTopics = std::move(maps);
        bridge_ = std::make_unique<Bridge>(settings);
        bridge_->attach(world_, scheduler_.bus());
        ASSERT_TRUE(bridge_->topicProblems().empty());
    }

    /// Steps 10 ms at a time and publishes, spinning the listener, until `done` or 10 s.
    bool run(const std::function<bool()>& done) {
        const auto deadline = std::chrono::steady_clock::now() + 10s;
        while (std::chrono::steady_clock::now() < deadline) {
            EXPECT_TRUE(scheduler_.stepUntil(scheduler_.now() + 10_ms).ok());
            bridge_->publish(scheduler_.bus(), scheduler_.now());
            executor_.spin_some();
            if (done()) {
                return true;
            }
            std::this_thread::sleep_for(5ms);
        }
        return false;
    }

    template <class Message>
    std::shared_ptr<rclcpp::Subscription<Message>> listen(const std::string& topic,
                                                          std::optional<Message>& into) {
        return listener_->create_subscription<Message>(
            topic, 10, [&into](const Message& message) { into = message; });
    }

    World world_;
    BodyId base_{}, wheel_{};
    Scheduler scheduler_;
    std::unique_ptr<Bridge> bridge_;
    rclcpp::Node::SharedPtr listener_;
    rclcpp::executors::SingleThreadedExecutor executor_;
};

double stampSeconds(const builtin_interfaces::msg::Time& t) {
    return t.sec + (t.nanosec * 1e-9);
}

const KindTopic* find(const std::vector<KindTopic>& topics, const std::string& name) {
    for (const KindTopic& topic : topics) {
        if (topic.topic == name) {
            return &topic;
        }
    }
    return nullptr;
}

// The plan alone: which topics a sensor gets, of what type, and which a user map takes over.
TEST_F(KindTopicsTest, PlansTypedTopicsThenEveryOtherNumberAndGivesWayToAMap) {
    world_.addCustomSensor("safety", "safety_scanner", base_, Pose{}, 25.0,
                           {{"fov", std::numbers::pi}, {"max_range", 30.0}}, {});
    scheduler_.add(std::make_unique<FakeKinds>(std::vector<Fake>{
        {"sensor/safety/ranges", SignalType::ScalarArray, units::m, {1.0, 2.0, kInf}},
        {"sensor/safety/protective", SignalType::Bool, units::one, {1.0}},
        {"sensor/safety/min_distance", SignalType::Scalar, units::m, {1.0}},
        {"sensor/safety/fault/dropped", SignalType::Int, units::one, {3.0}},
        {"sensor/safety/delayed/ranges", SignalType::ScalarArray, units::m, {1.0, 2.0, kInf}}}));
    ASSERT_TRUE(scheduler_.wire(WorldView(world_)));
    const std::vector<KindTopic> topics = kindTopics(world_, 0, scheduler_.bus());
    ASSERT_EQ(topics.size(), 4U) << "delayed/ copies are the bridge's own business";
    const KindTopic* scan = find(topics, "/safety/scan");
    ASSERT_NE(scan, nullptr);
    EXPECT_EQ(scan->type, "sensor_msgs/msg/LaserScan");
    EXPECT_EQ(scan->frame, "safety");
    EXPECT_DOUBLE_EQ(scan->params.at("angle_min"), -std::numbers::pi / 2.0);
    EXPECT_DOUBLE_EQ(scan->params.at("range_max"), 30.0);
    EXPECT_DOUBLE_EQ(scan->params.at("scan_time"), 0.04);
    EXPECT_EQ(find(topics, "/safety/ranges"), nullptr) << "the scan carries them";
    EXPECT_EQ(find(topics, "/safety/protective")->type, "std_msgs/msg/Bool");
    EXPECT_EQ(find(topics, "/safety/min_distance")->type, "std_msgs/msg/Float64");
    EXPECT_EQ(find(topics, "/safety/fault/dropped")->type, "std_msgs/msg/Int64");

    TopicMaps byTopic;
    byTopic.publish.push_back({"sensor/safety/min_distance", "/safety/scan"});
    EXPECT_EQ(find(withoutMapped(topics, byTopic), "/safety/scan"), nullptr);
    TopicMaps bySignal;
    bySignal.publish.push_back({"sensor/safety/ranges", "/my/scan"});
    const std::vector<KindTopic> left = withoutMapped(topics, bySignal);
    EXPECT_EQ(left.size(), 3U);
    EXPECT_EQ(find(left, "/safety/scan"), nullptr);
    TopicMaps off;
    off.kindTopics = false;
    EXPECT_TRUE(withoutMapped(topics, off).empty());
}

// Lazy: nothing is built for a topic nobody subscribes to, unless it rides on a delay or a link.
TEST_F(KindTopicsTest, BuildsNothingNobodyListensTo) {
    world_.addCustomSensor("alt", "rangefinder", base_, Pose{}, 100.0, {}, {});
    scheduler_.add(std::make_unique<FakeKinds>(
        std::vector<Fake>{{"sensor/alt/range", SignalType::Scalar, units::m, {2.0}}}));
    ASSERT_TRUE(scheduler_.wire(WorldView(world_)));
    auto node = std::make_shared<rclcpp::Node>("kind_topics_lazy");
    KindTopics topics(*node);
    EXPECT_TRUE(topics.attach(kindTopics(world_, 0, scheduler_.bus()), scheduler_.bus()).empty());
    std::size_t sent = 0;
    KindTopics::Hooks hooks;
    hooks.deliver = [&sent](const std::string&, std::function<void()> send, std::size_t) {
        ++sent;
        send();
    };
    bool forced = false;
    hooks.forced = [&forced](const std::string&) { return forced; };
    for (int i = 0; i < 5; ++i) {
        ASSERT_TRUE(scheduler_.stepUntil(scheduler_.now() + 10_ms).ok());
        topics.publish(scheduler_.bus(), hooks);
    }
    EXPECT_EQ(sent, 0U);
    forced = true;
    for (int i = 0; i < 5; ++i) {
        ASSERT_TRUE(scheduler_.stepUntil(scheduler_.now() + 10_ms).ok());
        topics.publish(scheduler_.bus(), hooks);
        topics.publish(scheduler_.bus(), hooks);  // once per sample
    }
    EXPECT_EQ(sent, 5U);
}

// sensor_msgs/Range: a rangefinder (light), a cliff sensor's cone, a proximity array's nearest
// and a single-beam sonar (ultrasound), with their kinds' limits.
TEST_F(KindTopicsTest, RangesCarryTheirKindsLimits) {
    world_.addCustomSensor("alt", "rangefinder", base_, Pose{}, 100.0, {{"range/max", 8.0}}, {});
    world_.addCustomSensor("cliff_fl", "cliff", base_, Pose{}, 50.0,
                           {{"rays", 5.0}, {"beam_angle", 0.1}}, {});
    world_.addCustomSensor("skin", "proximity", base_, Pose{}, 50.0,
                           {{"rows", 2.0}, {"columns", 2.0}, {"fov", 1.2}}, {});
    world_.addCustomSensor("ping", "sonar", base_, Pose{}, 10.0, {{"beams", 1.0}}, {});
    start({{"sensor/alt/range", SignalType::Scalar, units::m, {2.5}},
           {"sensor/alt/valid", SignalType::Bool, units::one, {1.0}},
           {"sensor/cliff_fl/range", SignalType::Scalar, units::m, {kInf}},
           {"sensor/cliff_fl/cliff", SignalType::Bool, units::one, {1.0}},
           {"sensor/skin/distances", SignalType::ScalarArray, units::m, {0.2, kInf, 0.1, 0.25}},
           {"sensor/skin/min_distance", SignalType::Scalar, units::m, {0.1}},
           {"sensor/ping/ranges", SignalType::ScalarArray, units::m, {4.0}}});
    std::optional<sensor_msgs::msg::Range> alt, cliff, skin, ping;
    std::optional<std_msgs::msg::Bool> valid, isCliff;
    std::optional<std_msgs::msg::Float64MultiArray> distances;
    auto a = listen("/alt/range", alt);
    auto b = listen("/cliff_fl/range", cliff);
    auto c = listen("/skin/range", skin);
    auto d = listen("/ping/range", ping);
    auto e = listen("/alt/valid", valid);
    auto f = listen("/cliff_fl/cliff", isCliff);
    auto g = listen("/skin/distances", distances);
    ASSERT_TRUE(run([&] { return alt && cliff && skin && ping && valid && isCliff && distances; }));

    EXPECT_EQ(alt->header.frame_id, "alt");
    EXPECT_EQ(alt->radiation_type, sensor_msgs::msg::Range::INFRARED);
    EXPECT_FLOAT_EQ(alt->range, 2.5F);
    EXPECT_FLOAT_EQ(alt->min_range, 0.1F) << "the kind's default";
    EXPECT_FLOAT_EQ(alt->max_range, 8.0F);
    EXPECT_GT(stampSeconds(alt->header.stamp), 0.0);
    EXPECT_TRUE(valid->data);
    EXPECT_TRUE(std::isinf(cliff->range)) << "REP 117: nothing in range";
    EXPECT_FLOAT_EQ(cliff->field_of_view, 0.2F);
    EXPECT_FLOAT_EQ(cliff->max_range, 0.15F);
    EXPECT_TRUE(isCliff->data);
    EXPECT_FLOAT_EQ(skin->range, 0.1F);
    EXPECT_FLOAT_EQ(skin->field_of_view, 1.2F);
    ASSERT_EQ(distances->layout.dim.size(), 2U) << "rows and columns";
    EXPECT_EQ(distances->layout.dim[0].size, 2U);
    EXPECT_EQ(distances->layout.dim[0].stride, 4U);
    EXPECT_EQ(distances->layout.dim[1].stride, 2U);
    EXPECT_EQ(ping->radiation_type, sensor_msgs::msg::Range::ULTRASOUND);
    EXPECT_FLOAT_EQ(ping->range, 4.0F);
    EXPECT_NEAR(ping->field_of_view, 25.0 * std::numbers::pi / 180.0, 1e-6);
}

// sensor_msgs/LaserScan: a safety scanner from −fov/2, a multibeam sonar from its left beam.
TEST_F(KindTopicsTest, ScansRunOverTheirFieldOfView) {
    world_.addCustomSensor("safety", "safety_scanner", base_, Pose{}, 25.0,
                           {{"fov", std::numbers::pi}, {"max_range", 30.0}}, {});
    world_.addCustomSensor("fls", "sonar", base_, Pose{}, 10.0,
                           {{"beams", 3.0}, {"horizontal_fov", 1.0}}, {});
    start({{"sensor/safety/ranges", SignalType::ScalarArray, units::m, {1.0, 2.0, kInf}},
           {"sensor/fls/ranges", SignalType::ScalarArray, units::m, {5.0, kInf, 6.0}},
           {"sensor/fls/intensities", SignalType::ScalarArray, units::one, {-10.0, kNaN, -20.0}}});
    std::optional<sensor_msgs::msg::LaserScan> safety, fls;
    auto a = listen("/safety/scan", safety);
    auto b = listen("/fls/scan", fls);
    ASSERT_TRUE(run([&] { return safety && fls; }));
    EXPECT_EQ(safety->header.frame_id, "safety");
    EXPECT_FLOAT_EQ(safety->angle_min, static_cast<float>(-std::numbers::pi / 2));
    EXPECT_FLOAT_EQ(safety->angle_increment, static_cast<float>(std::numbers::pi / 2));
    EXPECT_FLOAT_EQ(safety->range_max, 30.0F);
    EXPECT_FLOAT_EQ(safety->scan_time, 0.04F);
    ASSERT_EQ(safety->ranges.size(), 3U);
    EXPECT_TRUE(std::isinf(safety->ranges[2]));
    EXPECT_FLOAT_EQ(fls->angle_min, 0.5F) << "beam 0 is the left one";
    EXPECT_FLOAT_EQ(fls->angle_increment, -0.5F);
    ASSERT_EQ(fls->intensities.size(), 3U);
    EXPECT_FLOAT_EQ(fls->intensities[2], -20.0F);
}

// nav_msgs/Odometry from wheel odometry: odom → its body, the twist in the body frame, the
// covariance diagonals by default and from the sensor's text.
TEST_F(KindTopicsTest, WheelOdometryIsOdometryWithItsCovariance) {
    world_.addCustomSensor("wheel_odom", "wheel_odometry", base_, Pose{}, 50.0, {},
                           {{"twist_covariance_diagonal", "0.01 0.02 0.03 0.04 0.05 0.06"}});
    const double s = std::sin(0.25);
    const double c = std::cos(0.25);
    start({{"sensor/wheel_odom/pose",
            SignalType::Pose,
            units::m,
            {1.0, 2.0, 0.0, c, 0.0, 0.0, s},
            "odom"},
           {"sensor/wheel_odom/yaw", SignalType::Scalar, units::rad, {0.5}},
           {"sensor/wheel_odom/twist",
            SignalType::Wrench,
            units::m_per_s,
            {0.4, 0.0, 0.0, 0.0, 0.0, 0.3},
            "body"}});
    std::optional<nav_msgs::msg::Odometry> odom;
    std::optional<std_msgs::msg::Float64> yaw;
    auto a = listen("/wheel_odom/odom", odom);
    auto b = listen("/wheel_odom/yaw", yaw);
    ASSERT_TRUE(run([&] { return odom.has_value(); }));
    EXPECT_FALSE(yaw.has_value()) << "the odometry carries it";
    EXPECT_EQ(odom->header.frame_id, "odom");
    EXPECT_EQ(odom->child_frame_id, "base_link");
    EXPECT_DOUBLE_EQ(odom->pose.pose.position.y, 2.0);
    EXPECT_DOUBLE_EQ(odom->pose.pose.orientation.z, s);
    EXPECT_DOUBLE_EQ(odom->twist.twist.linear.x, 0.4);
    EXPECT_DOUBLE_EQ(odom->twist.twist.angular.z, 0.3);
    EXPECT_DOUBLE_EQ(odom->pose.covariance[0], 1e-3);
    EXPECT_DOUBLE_EQ(odom->pose.covariance[14], 1e6) << "z is not measured";
    EXPECT_DOUBLE_EQ(odom->pose.covariance[35], 1e-2);
    EXPECT_DOUBLE_EQ(odom->twist.covariance[7], 0.02);
    EXPECT_DOUBLE_EQ(odom->twist.covariance[35], 0.06);
    EXPECT_DOUBLE_EQ(odom->pose.covariance[1], 0.0);
}

// sensor_msgs/BatteryState: ROS's signs and units (negative discharging, Ah, °C, 0 to 1).
TEST_F(KindTopicsTest, ABatteryMonitorIsABatteryState) {
    world_.addCustomSensor("bms", "battery_monitor", base_, Pose{}, 10.0, {{"capacity", 5.0}},
                           {{"battery", "pack"}});
    start({{"sensor/bms/voltage", SignalType::Scalar, units::V, {24.5}},
           {"sensor/bms/current", SignalType::Scalar, units::A, {3.0}},
           {"sensor/bms/soc", SignalType::Scalar, units::one, {0.6}},
           {"sensor/bms/temperature", SignalType::Scalar, units::K, {303.15}},
           {"sensor/bms/remaining_capacity", SignalType::Scalar, units::C, {10800.0}},
           {"sensor/bms/valid", SignalType::Bool, units::one, {1.0}}});
    std::optional<sensor_msgs::msg::BatteryState> state;
    std::optional<sensor_msgs::msg::Temperature> temperature;
    auto a = listen("/bms/battery_state", state);
    auto b = listen("/bms/temperature", temperature);
    ASSERT_TRUE(run([&] { return state.has_value(); }));
    EXPECT_FALSE(temperature.has_value()) << "the battery state carries it";
    EXPECT_FLOAT_EQ(state->voltage, 24.5F);
    EXPECT_FLOAT_EQ(state->current, -3.0F) << "ROS: negative when discharging";
    EXPECT_FLOAT_EQ(state->percentage, 0.6F);
    EXPECT_FLOAT_EQ(state->charge, 3.0F) << "Ah";
    EXPECT_FLOAT_EQ(state->capacity, 5.0F);
    EXPECT_NEAR(state->temperature, 30.0F, 1e-4);
    EXPECT_EQ(state->power_supply_status,
              sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING);
    EXPECT_TRUE(state->present);
    EXPECT_EQ(state->location, "pack");
}

// Joint torque as sensor_msgs/JointState (the joint that moves its body), a force-torque sensor as
// geometry_msgs/WrenchStamped and the flow of an optical flow sensor in its optical frame.
TEST_F(KindTopicsTest, TorquesWrenchesAndFlow) {
    world_.addCustomSensor("left_torque", "joint_torque", wheel_, Pose{}, 100.0, {}, {});
    world_.addCustomSensor("wrist", "force_torque", base_, Pose{}, 100.0, {}, {});
    world_.addCustomSensor("flow", "optical_flow", base_, Pose{}, 100.0, {}, {});
    start({{"sensor/left_torque/torque", SignalType::Scalar, units::N_m, {-1.5}},
           {"sensor/left_torque/saturated", SignalType::Bool, units::one, {0.0}},
           {"sensor/wrist/wrench",
            SignalType::Wrench,
            units::N,
            {1.0, 2.0, 3.0, 0.1, 0.2, 0.3},
            "sensor:wrist"},
           {"sensor/wrist/force", SignalType::Vec3, units::N, {1.0, 2.0, 3.0}, "sensor:wrist"},
           {"sensor/flow/flow", SignalType::ScalarArray, units::rad, {0.01, -0.02}, "optical"},
           {"sensor/flow/quality", SignalType::Scalar, units::one, {200.0}}});
    std::optional<sensor_msgs::msg::JointState> torque;
    std::optional<geometry_msgs::msg::WrenchStamped> wrench;
    std::optional<geometry_msgs::msg::Vector3Stamped> flow, force;
    std::optional<std_msgs::msg::Float64> quality;
    auto a = listen("/left_torque/joint_state", torque);
    auto b = listen("/wrist/wrench", wrench);
    auto c = listen("/flow/flow", flow);
    auto d = listen("/flow/quality", quality);
    auto e = listen("/wrist/force", force);
    ASSERT_TRUE(run([&] { return torque && wrench && flow && quality; }));
    EXPECT_FALSE(force.has_value()) << "the wrench carries it";
    ASSERT_EQ(torque->name.size(), 1U);
    EXPECT_EQ(torque->name[0], "left_wheel");
    EXPECT_DOUBLE_EQ(torque->effort.at(0), -1.5);
    EXPECT_TRUE(torque->position.empty());
    EXPECT_EQ(wrench->header.frame_id, "wrist");
    EXPECT_DOUBLE_EQ(wrench->wrench.force.z, 3.0);
    EXPECT_DOUBLE_EQ(wrench->wrench.torque.x, 0.1);
    EXPECT_EQ(flow->header.frame_id, "flow_optical");
    EXPECT_DOUBLE_EQ(flow->vector.y, -0.02);
    EXPECT_DOUBLE_EQ(flow->vector.z, 0.0);
    EXPECT_DOUBLE_EQ(quality->data, 200.0);
}

// vision_msgs: the detections' rows as 2D boxes in pixels and 3D boxes in the camera's optical
// frame, KITTI's rotation_y about the optical y; NaN rows and rows past `count` are left out.
TEST_F(KindTopicsTest, DetectionsAreVisionMsgs) {
    world_.addCustomSensor("labels", "detections", base_, Pose{}, 10.0, {},
                           {{"camera", "front_camera"}});
    std::vector<double> rows(3 * 16, kNaN);
    const std::array<double, 16> box{42,  8,   10,  20,  29,  59,  300, 0.1,
                                     0.0, 0.5, 0.2, 4.0, 1.2, 0.8, 1.0, -std::numbers::pi / 2};
    std::copy(box.begin(), box.end(), rows.begin());
    std::copy(box.begin(), box.end(), rows.begin() + 16);
    rows[16] = 43;
    start({{"sensor/labels/detections", SignalType::ScalarArray, units::one, rows},
           {"sensor/labels/count", SignalType::Int, units::one, {1.0}}});
    std::optional<vision_msgs::msg::Detection2DArray> flat;
    std::optional<vision_msgs::msg::Detection3DArray> boxes;
    std::optional<std_msgs::msg::Int64> count;
    auto a = listen("/labels/detections", flat);
    auto b = listen("/labels/detections_3d", boxes);
    auto c = listen("/labels/count", count);
    ASSERT_TRUE(run([&] { return flat && boxes && count; }));
    EXPECT_EQ(count->data, 1);
    ASSERT_EQ(flat->detections.size(), 1U) << "count says one";
    const vision_msgs::msg::Detection2D& d = flat->detections[0];
    EXPECT_EQ(flat->header.frame_id, "front_camera_optical");
    EXPECT_EQ(d.id, "42");
    ASSERT_EQ(d.results.size(), 1U);
    EXPECT_EQ(d.results[0].hypothesis.class_id, "8");
    EXPECT_DOUBLE_EQ(d.bbox.center.position.x, 19.5);
    EXPECT_DOUBLE_EQ(d.bbox.center.position.y, 39.5);
    EXPECT_DOUBLE_EQ(d.bbox.size_x, 20.0);
    EXPECT_DOUBLE_EQ(d.bbox.size_y, 40.0);
    ASSERT_EQ(boxes->detections.size(), 1U);
    const vision_msgs::msg::BoundingBox3D& b3 = boxes->detections[0].bbox;
    EXPECT_DOUBLE_EQ(b3.center.position.z, 4.0);
    EXPECT_DOUBLE_EQ(b3.size.x, 1.2) << "along the object";
    EXPECT_DOUBLE_EQ(b3.size.y, 1.0) << "its height, along the optical y";
    EXPECT_DOUBLE_EQ(b3.size.z, 0.8) << "its width";
    // Turned −90° about the optical y: the object's x along the optical z (facing away).
    EXPECT_NEAR(b3.center.orientation.w, std::cos(-std::numbers::pi / 4), 1e-12);
    EXPECT_NEAR(b3.center.orientation.y, std::sin(-std::numbers::pi / 4), 1e-12);
}

// Every other number by its unit and name: Temperature in °C, FluidPressure, RelativeHumidity,
// Illuminance, a vector in the sensor frame, a position in the world, rows of a receiver.
TEST_F(KindTopicsTest, OtherNumbersGoByTheirUnitAndName) {
    world_.addCustomSensor("hyt", "humidity_sensor", base_, Pose{}, 10.0, {}, {});
    world_.addCustomSensor("pitot", "airspeed", base_, Pose{}, 10.0, {}, {});
    world_.addCustomSensor("lux", "light_sensor", base_, Pose{}, 10.0, {}, {});
    world_.addCustomSensor("wind", "anemometer", base_, Pose{}, 10.0, {}, {});
    world_.addCustomSensor("rfid", "tag_reader", base_, Pose{}, 10.0, {}, {});
    world_.addCustomSensor("adsb", "adsb_receiver", base_, Pose{}, 1.0, {}, {});
    std::vector<double> aircraft(20, 1.0);
    start({{"sensor/hyt/relative_humidity", SignalType::Scalar, units::one, {0.45}},
           {"sensor/hyt/temperature", SignalType::Scalar, units::K, {293.15}},
           {"sensor/pitot/differential_pressure", SignalType::Scalar, units::Pa, {61.25}},
           {"sensor/pitot/airspeed", SignalType::Scalar, units::m_per_s, {10.0}},
           {"sensor/lux/illuminance", SignalType::Scalar, units::one, {500.0}},
           {"sensor/wind/wind_velocity", SignalType::Vec3, units::m_per_s, {3, 4, 0}, "sensor"},
           {"sensor/rfid/tag_position", SignalType::Vec3, units::m, {5, 6, 0}, "world_local"},
           {"sensor/adsb/aircraft", SignalType::ScalarArray, units::one, aircraft}});
    std::optional<sensor_msgs::msg::RelativeHumidity> humidity;
    std::optional<sensor_msgs::msg::Temperature> temperature;
    std::optional<sensor_msgs::msg::FluidPressure> pressure;
    std::optional<std_msgs::msg::Float64> airspeed;
    std::optional<sensor_msgs::msg::Illuminance> lux;
    std::optional<geometry_msgs::msg::Vector3Stamped> wind, tag;
    std::optional<std_msgs::msg::Float64MultiArray> planes;
    auto a = listen("/hyt/relative_humidity", humidity);
    auto b = listen("/hyt/temperature", temperature);
    auto c = listen("/pitot/differential_pressure", pressure);
    auto d = listen("/pitot/airspeed", airspeed);
    auto e = listen("/lux/illuminance", lux);
    auto f = listen("/wind/wind_velocity", wind);
    auto g = listen("/rfid/tag_position", tag);
    auto h = listen("/adsb/aircraft", planes);
    ASSERT_TRUE(run([&] {
        return humidity && temperature && pressure && airspeed && lux && wind && tag && planes;
    }));
    EXPECT_DOUBLE_EQ(humidity->relative_humidity, 0.45) << "0 to 1, as ROS has it";
    EXPECT_EQ(humidity->header.frame_id, "hyt");
    EXPECT_NEAR(temperature->temperature, 20.0, 1e-9) << "°C";
    EXPECT_DOUBLE_EQ(pressure->fluid_pressure, 61.25);
    EXPECT_DOUBLE_EQ(airspeed->data, 10.0);
    EXPECT_DOUBLE_EQ(lux->illuminance, 500.0);
    EXPECT_EQ(wind->header.frame_id, "wind");
    EXPECT_DOUBLE_EQ(wind->vector.y, 4.0);
    EXPECT_EQ(tag->header.frame_id, "world");
    EXPECT_DOUBLE_EQ(tag->vector.x, 5.0);
    ASSERT_EQ(planes->layout.dim.size(), 2U);
    EXPECT_EQ(planes->layout.dim[0].size, 2U) << "two aircraft";
    EXPECT_EQ(planes->layout.dim[1].size, 10U) << "ten fields each";
}

// A --ros-topics entry wins: on the same topic it replaces the default, and a signal it
// publishes elsewhere takes that signal's default topic away; the others stay.
TEST_F(KindTopicsTest, AMapReplacesTheDefaultsItCovers) {
    world_.addCustomSensor("alt", "rangefinder", base_, Pose{}, 100.0, {}, {});
    world_.addCustomSensor("hyt", "humidity_sensor", base_, Pose{}, 10.0, {}, {});
    TopicMaps maps;
    TopicMap mine;
    mine.signal = "sensor/alt/range";
    mine.topic = "/alt/range";
    mine.type = "std_msgs/msg/Float64";
    maps.publish.push_back(mine);
    TopicMap elsewhere;
    elsewhere.signal = "sensor/hyt/temperature";
    elsewhere.topic = "/air/temperature";
    elsewhere.type = "sensor_msgs/msg/Temperature";
    maps.publish.push_back(elsewhere);
    start({{"sensor/alt/range", SignalType::Scalar, units::m, {2.5}},
           {"sensor/hyt/relative_humidity", SignalType::Scalar, units::one, {0.45}},
           {"sensor/hyt/temperature", SignalType::Scalar, units::K, {293.15}}},
          maps);
    std::optional<std_msgs::msg::Float64> range;
    std::optional<sensor_msgs::msg::Temperature> air, defaulted;
    std::optional<sensor_msgs::msg::RelativeHumidity> humidity;
    auto a = listen("/alt/range", range);
    auto b = listen("/air/temperature", air);
    auto c = listen("/hyt/temperature", defaulted);
    auto d = listen("/hyt/relative_humidity", humidity);
    ASSERT_TRUE(run([&] { return range && air && humidity; }));
    EXPECT_DOUBLE_EQ(range->data, 2.5) << "the map's Float64, not the default Range";
    EXPECT_FALSE(defaulted.has_value());
    EXPECT_EQ(listener_->count_publishers("/hyt/temperature"), 0U)
        << "no default publisher for a signal the map publishes";
    EXPECT_EQ(listener_->count_publishers("/alt/range"), 1U) << "the map's alone";
}

}  // namespace
}  // namespace lagrange::ros2
