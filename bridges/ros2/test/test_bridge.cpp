// The bridge over a real DDS domain (ROS_DOMAIN_ID 73, localhost): a fake module writes the
// signals a robot would, the bridge publishes them, and a test node listens.
#include "lagrange/ros2/Bridge.hpp"
#include "lagrange/scheduler/Scheduler.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <numbers>
#include <optional>

#include <gtest/gtest.h>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace lagrange::ros2 {
namespace {

using namespace literals;
using namespace std::chrono_literals;

/// Writes the signals of a robot moving along +x at 0.5 m/s, a wheel and a 4-beam lidar.
class FakeRobot final : public IModule {
public:
    [[nodiscard]] ModuleInfo info() const override { return {"rigid_body", "fake"}; }
    [[nodiscard]] Duration preferredStep() const override { return 1_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        const auto out = [&d](const char* name, SignalType type, Unit unit, std::uint32_t n = 1) {
            SignalSpec spec;
            spec.name = name;
            spec.type = type;
            spec.unit = unit;
            spec.arrayLength = n;
            return d.output(spec);
        };
        pose_ = out("body/base_link/pose", SignalType::Pose, units::m);
        twist_ = out("body/base_link/twist", SignalType::Wrench, units::m_per_s);
        wheel_ = out("joint/wheel/position", SignalType::Scalar, units::rad);
        ranges_ = out("sensor/front_lidar/ranges", SignalType::ScalarArray, units::m, 4);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        return step(StepContext{}, bus);
    }
    Status step(const StepContext& c, SignalBus& bus) override {
        const double t = (c.t + c.dt).seconds();
        const std::array<double, 7> pose{0.5 * t, 0.0, 0.1, 1.0, 0.0, 0.0, 0.0};
        bus.writeN(pose_, pose);
        const std::array<double, 6> twist{0.5, 0.0, 0.0, 0.0, 0.0, 0.0};
        bus.writeN(twist_, twist);
        bus.write(wheel_, 5.0 * t);
        const std::array<double, 4> ranges{1.0, 2.0, std::numeric_limits<double>::infinity(), 4.0};
        bus.writeN(ranges_, ranges);
        return {};
    }

private:
    SignalId pose_, twist_, wheel_, ranges_;
};

class BridgeTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
    }

    void SetUp() override {
        const BodyId base = world_.addBody("base_link", kWorldBody, Pose{}, 1.0, {}, {});
        const BodyId wheel = world_.addBody("wheel_link", base, Pose{}, 1.0, {}, {});
        world_.addJoint("wheel", JointType::Continuous, wheel, {0, 1, 0}, true);
        world_.addSensor("front_lidar", SensorType::Lidar, base, Pose{{0.2, 0.0, 0.1}}, 10.0,
                         {{"min_angle", -std::numbers::pi / 2},
                          {"max_angle", std::numbers::pi},
                          {"min_range", 0.1},
                          {"max_range", 10.0}});
        scheduler_.add(std::make_unique<FakeRobot>());
        ASSERT_TRUE(scheduler_.wire(WorldView(world_)));
        BridgeSettings settings;
        settings.baseLink = "base_link";
        settings.robotDescription = "<robot name=\"test\"/>";
        bridge_ = std::make_unique<Bridge>(settings);
        bridge_->attach(world_, scheduler_.bus());
        listener_ = std::make_shared<rclcpp::Node>("bridge_test_listener");
        executor_.add_node(listener_);
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

    World world_;
    Scheduler scheduler_;
    std::unique_ptr<Bridge> bridge_;
    rclcpp::Node::SharedPtr listener_;
    rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(BridgeTest, PublishesClockOdometryJointStatesAndScans) {
    std::optional<rosgraph_msgs::msg::Clock> clock;
    std::optional<nav_msgs::msg::Odometry> odom;
    std::optional<sensor_msgs::msg::JointState> joints;
    std::optional<sensor_msgs::msg::LaserScan> scan;
    auto a = listener_->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", rclcpp::ClockQoS(), [&](const rosgraph_msgs::msg::Clock& m) { clock = m; });
    auto b = listener_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, [&](const nav_msgs::msg::Odometry& m) { odom = m; });
    auto c = listener_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, [&](const sensor_msgs::msg::JointState& m) { joints = m; });
    auto d = listener_->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(), [&](const sensor_msgs::msg::LaserScan& m) { scan = m; });
    ASSERT_TRUE(run([&] { return clock && odom && joints && scan; }));

    EXPECT_GT(clock->clock.sec * 1'000'000'000LL + clock->clock.nanosec, 0);
    EXPECT_EQ(odom->header.frame_id, "odom");
    EXPECT_EQ(odom->child_frame_id, "base_link");
    const double t = odom->header.stamp.sec + odom->header.stamp.nanosec * 1e-9;
    EXPECT_NEAR(odom->pose.pose.position.x, 0.5 * t, 1e-9);
    EXPECT_NEAR(odom->twist.twist.linear.x, 0.5, 1e-12);
    ASSERT_EQ(joints->name.size(), 1U);
    EXPECT_EQ(joints->name[0], "wheel");
    EXPECT_EQ(scan->header.frame_id, "front_lidar");
    ASSERT_EQ(scan->ranges.size(), 4U);
    EXPECT_FLOAT_EQ(scan->ranges[1], 2.0F);
    EXPECT_TRUE(std::isinf(scan->ranges[2]));
    EXPECT_FLOAT_EQ(scan->angle_increment, static_cast<float>(std::numbers::pi / 2));
}

TEST_F(BridgeTest, TakesVelocityCommandsAndExpiresThem) {
    auto publisher = listener_->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
    geometry_msgs::msg::TwistStamped twist;
    twist.twist.linear.x = 0.3;
    twist.twist.angular.z = -0.2;
    ASSERT_TRUE(run([&] {
        publisher->publish(twist);
        return bridge_->latestCommand(500ms).has_value();
    }));
    const CmdVel command = *bridge_->latestCommand(500ms);
    EXPECT_DOUBLE_EQ(command.linearMps, 0.3);
    EXPECT_DOUBLE_EQ(command.angularRadps, -0.2);
    std::this_thread::sleep_for(60ms);
    EXPECT_FALSE(bridge_->latestCommand(50ms).has_value()) << "a stale command must not count";
}

TEST_F(BridgeTest, GraphListsTheBridgesTopicsWithRates) {
    ASSERT_TRUE(run([&] { return scheduler_.now() >= SimTime{500'000'000}; }));
    const GraphSnapshot graph = bridge_->graph();
    bool clock = false;
    for (const GraphSnapshot::Topic& topic : graph.topics) {
        if (topic.name == "/clock") {
            clock = true;
            EXPECT_TRUE(topic.fromBridge);
            EXPECT_EQ(topic.type, "rosgraph_msgs/msg/Clock");
            EXPECT_GT(topic.rateHz, 0.0);
        }
    }
    EXPECT_TRUE(clock);
    bool bridgeNode = false;
    for (const GraphSnapshot::Node& node : graph.nodes) {
        bridgeNode = bridgeNode || (node.name == "/lagrange_ros2_bridge" && !node.external);
    }
    EXPECT_TRUE(bridgeNode);
    EXPECT_FALSE(graph.rmw.empty());
}

}  // namespace
}  // namespace lagrange::ros2
