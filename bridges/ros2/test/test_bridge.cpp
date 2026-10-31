// The bridge over a real DDS domain (ROS_DOMAIN_ID 73, localhost): a fake module writes the
// signals a robot would, the bridge publishes them, and a test node listens.
#include "lagrange/ros2/Bridge.hpp"
#include "lagrange/scheduler/Scheduler.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <future>
#include <memory>
#include <numbers>
#include <optional>
#include <simulation_interfaces/srv/delete_entity.hpp>
#include <simulation_interfaces/srv/get_entities.hpp>
#include <simulation_interfaces/srv/get_entity_state.hpp>
#include <simulation_interfaces/srv/get_simulation_state.hpp>
#include <simulation_interfaces/srv/reset_simulation.hpp>
#include <simulation_interfaces/srv/set_simulation_state.hpp>
#include <simulation_interfaces/srv/spawn_entities.hpp>
#include <simulation_interfaces/srv/spawn_entity.hpp>
#include <simulation_interfaces/srv/step_simulation.hpp>

#include <gtest/gtest.h>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

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

/// The same robot with a two-ring lidar: two columns at 0 and 90 degrees, rings at 0 and +30.
class CloudBridgeTest : public BridgeTest {
protected:
    void SetUp() override {
        const BodyId base = world_.addBody("base_link", kWorldBody, Pose{}, 1.0, {}, {});
        const BodyId wheel = world_.addBody("wheel_link", base, Pose{}, 1.0, {}, {});
        world_.addJoint("wheel", JointType::Continuous, wheel, {0, 1, 0}, true);
        world_.addSensor("front_lidar", SensorType::Lidar, base, Pose{}, 10.0,
                         {{"samples", 2.0},
                          {"min_angle", 0.0},
                          {"max_angle", std::numbers::pi / 2},
                          {"vertical_samples", 2.0},
                          {"vertical_min_angle", 0.0},
                          {"vertical_max_angle", std::numbers::pi / 6},
                          {"min_range", 0.1},
                          {"max_range", 10.0}});
        scheduler_.add(std::make_unique<FakeRobot>());
        ASSERT_TRUE(scheduler_.wire(WorldView(world_)));
        BridgeSettings settings;
        settings.baseLink = "base_link";
        bridge_ = std::make_unique<Bridge>(settings);
        bridge_->attach(world_, scheduler_.bus());
        listener_ = std::make_shared<rclcpp::Node>("bridge_cloud_listener");
        executor_.add_node(listener_);
    }
};

TEST_F(CloudBridgeTest, PublishesA3dLidarAsAnOrganisedPointCloud) {
    std::optional<sensor_msgs::msg::PointCloud2> cloud;
    auto sub = listener_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/points", rclcpp::SensorDataQoS(),
        [&](const sensor_msgs::msg::PointCloud2& m) { cloud = m; });
    ASSERT_TRUE(run([&] { return cloud.has_value(); }));
    EXPECT_EQ(cloud->header.frame_id, "front_lidar");
    EXPECT_EQ(cloud->height, 2U);
    EXPECT_EQ(cloud->width, 2U);
    ASSERT_EQ(cloud->fields.size(), 6U);
    EXPECT_EQ(cloud->fields[4].name, "ring");
    const auto point = [&](std::size_t i) {
        std::array<float, 3> xyz{};
        std::memcpy(xyz.data(), cloud->data.data() + i * cloud->point_step, sizeof(xyz));
        return xyz;
    };
    // The fake ranges are 1, 2, +inf, 4: ring 0 at 0 and 90 deg, then ring 1 (+30 deg).
    EXPECT_NEAR(point(0)[0], 1.0F, 1e-6F);
    EXPECT_NEAR(point(1)[1], 2.0F, 1e-6F);
    EXPECT_TRUE(std::isnan(point(2)[0])) << "no return is NaN in an organised cloud";
    EXPECT_NEAR(point(3)[1], 4.0F * std::cos(std::numbers::pi_v<float> / 6), 1e-5F);
    EXPECT_NEAR(point(3)[2], 2.0F, 1e-5F);  // 4 sin 30
    std::uint16_t ring = 0;
    std::memcpy(&ring, cloud->data.data() + 3 * cloud->point_step + 16, sizeof(ring));
    EXPECT_EQ(ring, 1U);
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

TEST_F(BridgeTest, TakesAckermannCommandsForRoadVehicles) {
    // A road vehicle's bridge: the same robot, with /ackermann_cmd taken as well.
    BridgeSettings settings;
    settings.baseLink = "base_link";
    settings.ackermannCmd = true;
    bridge_ = std::make_unique<Bridge>(settings);
    bridge_->attach(world_, scheduler_.bus());
    auto publisher = listener_->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        "/ackermann_cmd", 10);
    ackermann_msgs::msg::AckermannDriveStamped drive;
    drive.drive.steering_angle = 0.15F;
    drive.drive.speed = 4.0F;
    ASSERT_TRUE(run([&] {
        publisher->publish(drive);
        return bridge_->latestCommand(500ms).has_value();
    }));
    const CmdVel command = *bridge_->latestCommand(500ms);
    EXPECT_DOUBLE_EQ(command.linearMps, 4.0);
    ASSERT_TRUE(command.steeringAngle.has_value());
    EXPECT_FLOAT_EQ(static_cast<float>(*command.steeringAngle), 0.15F);
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

/// A simulator that records what it is asked, for the simulation_interfaces plumbing.
class FakeSimulation final : public SimulationHandler {
public:
    SimState state{SimState::Playing};
    std::uint64_t stepped{0};
    int resets{0};
    std::vector<SpawnRequest> spawns;
    std::vector<std::string> removed;

    SimState simulationState() override { return state; }
    SpawnOutcome spawn(const SpawnRequest& request) override {
        spawns.push_back(request);
        return {{}, "/" + request.name};
    }
    SimOutcome remove(const std::string& name) override {
        removed.push_back(name);
        return name == "/robot" ? SimOutcome{} : SimOutcome{SimResult::NotFound, "no " + name};
    }
    SimOutcome setSimulationState(SimState target) override {
        if (target == state) {
            return {SimResult::AlreadyInTargetState, ""};
        }
        state = target;
        return {};
    }
    SimOutcome step(std::uint64_t ticks) override {
        if (state == SimState::Playing) {
            return {SimResult::Failed, "pause first"};
        }
        stepped += ticks;
        return {};
    }
    SimOutcome reset() override {
        ++resets;
        return {};
    }
    std::vector<EntityDescription> entities() override {
        return {{"/robot", EntityCategory::Robot, "the robot"},
                {"/scenery/pallet", EntityCategory::StaticObject, "a pallet"},
                {"/scenery/rack", EntityCategory::StaticObject, "a rack"}};
    }
    std::optional<EntityMotion> entityMotion(const std::string& name) override {
        if (name != "/robot") {
            return std::nullopt;
        }
        EntityMotion motion;
        motion.pose.position = {1.0, 2.0, 0.1};
        motion.twist = {0.5, 0.0, 0.0, 0.0, 0.0, 0.25};
        return motion;
    }
};

class SimulationServicesTest : public BridgeTest {
protected:
    void SetUp() override {
        BridgeTest::SetUp();
        bridge_->serveSimulation(fake_);
    }

    /// Calls `service` and serves the bridge's queue on this thread until the answer comes.
    template <class Srv>
    typename Srv::Response::SharedPtr call(const std::string& service,
                                           typename Srv::Request request) {
        auto client = listener_->create_client<Srv>(service);
        EXPECT_TRUE(client->wait_for_service(5s)) << service;
        auto future =
            client->async_send_request(std::make_shared<typename Srv::Request>(std::move(request)));
        const auto deadline = std::chrono::steady_clock::now() + 10s;
        while (future.wait_for(5ms) != std::future_status::ready &&
               std::chrono::steady_clock::now() < deadline) {
            bridge_->handleRequests();
            executor_.spin_some();
        }
        EXPECT_EQ(future.wait_for(0s), std::future_status::ready) << service;
        return future.get();
    }

    FakeSimulation fake_;
};

namespace si = simulation_interfaces;

TEST_F(SimulationServicesTest, PausesStepsAndResets) {
    si::srv::StepSimulation::Request step;
    step.steps = 5;
    EXPECT_EQ(call<si::srv::StepSimulation>("/step_simulation", step)->result.result,
              si::msg::Result::RESULT_OPERATION_FAILED)
        << "stepping a playing simulation must fail";

    si::srv::SetSimulationState::Request pause;
    pause.state.state = si::msg::SimulationState::STATE_PAUSED;
    EXPECT_EQ(call<si::srv::SetSimulationState>("/set_simulation_state", pause)->result.result,
              si::msg::Result::RESULT_OK);
    EXPECT_EQ(call<si::srv::SetSimulationState>("/set_simulation_state", pause)->result.result,
              si::srv::SetSimulationState::Response::ALREADY_IN_TARGET_STATE);
    EXPECT_EQ(call<si::srv::GetSimulationState>("/get_simulation_state",
                                                si::srv::GetSimulationState::Request())
                  ->state.state,
              si::msg::SimulationState::STATE_PAUSED);

    EXPECT_EQ(call<si::srv::StepSimulation>("/step_simulation", step)->result.result,
              si::msg::Result::RESULT_OK);
    EXPECT_EQ(fake_.stepped, 5U);

    si::srv::ResetSimulation::Request timeOnly;
    timeOnly.scope = si::srv::ResetSimulation::Request::SCOPE_TIME;
    EXPECT_EQ(call<si::srv::ResetSimulation>("/reset_simulation", timeOnly)->result.result,
              si::msg::Result::RESULT_FEATURE_UNSUPPORTED);
    EXPECT_EQ(
        call<si::srv::ResetSimulation>("/reset_simulation", si::srv::ResetSimulation::Request())
            ->result.result,
        si::msg::Result::RESULT_OK);
    EXPECT_EQ(fake_.resets, 1);

    si::srv::SpawnEntity::Request spawn;
    spawn.name = "mate";
    spawn.entity_resource.uri = "/robots/amr.urdf";
    spawn.initial_pose.header.frame_id = "world";
    spawn.initial_pose.pose.position.x = 2.0;
    spawn.initial_pose.pose.orientation.w = 1.0;
    const auto spawned = call<si::srv::SpawnEntity>("/spawn_entity", spawn);
    EXPECT_EQ(spawned->result.result, si::msg::Result::RESULT_OK);
    EXPECT_EQ(spawned->entity_name, "/mate");
    ASSERT_EQ(fake_.spawns.size(), 1U);
    EXPECT_EQ(fake_.spawns[0].uri, "/robots/amr.urdf");
    EXPECT_EQ(fake_.spawns[0].frame, "world");
    EXPECT_EQ(fake_.spawns[0].pose.position[0], 2.0);
    EXPECT_EQ(fake_.spawns[0].pose.orientation[0], 1.0);

    si::srv::SpawnEntities::Request many;
    many.spawn_requests.resize(2);
    many.spawn_requests[0].name = "a";
    many.spawn_requests[1].name = "b";
    const auto both = call<si::srv::SpawnEntities>("/spawn_entities", many);
    EXPECT_EQ(both->result.result, si::msg::Result::RESULT_OK);
    ASSERT_EQ(both->results.size(), 2U);
    EXPECT_EQ(both->results[1].entity_name, "/b");

    si::srv::DeleteEntity::Request gone;
    gone.entity = "/robot";
    EXPECT_EQ(call<si::srv::DeleteEntity>("/delete_entity", gone)->result.result,
              si::msg::Result::RESULT_OK);
    gone.entity = "/nobody";
    EXPECT_EQ(call<si::srv::DeleteEntity>("/delete_entity", gone)->result.result,
              si::msg::Result::RESULT_NOT_FOUND);
}

TEST_F(SimulationServicesTest, ListsAndFiltersEntitiesAndReportsTheirState) {
    EXPECT_EQ(call<si::srv::GetEntities>("/get_entities", si::srv::GetEntities::Request())
                  ->entities.size(),
              3U);

    si::srv::GetEntities::Request byName;
    byName.filters.filter = "pallet|robot";
    EXPECT_EQ(call<si::srv::GetEntities>("/get_entities", byName)->entities,
              (std::vector<std::string>{"/robot", "/scenery/pallet"}));

    si::srv::GetEntities::Request byCategory;
    si::msg::EntityCategory statics;
    statics.category = si::msg::EntityCategory::CATEGORY_STATIC_OBJECT;
    byCategory.filters.categories.push_back(statics);
    EXPECT_EQ(call<si::srv::GetEntities>("/get_entities", byCategory)->entities,
              (std::vector<std::string>{"/scenery/pallet", "/scenery/rack"}));

    si::srv::GetEntities::Request byTag;
    byTag.filters.tags.tags = {"shelf"};
    EXPECT_EQ(call<si::srv::GetEntities>("/get_entities", byTag)->result.result,
              si::msg::Result::RESULT_FEATURE_UNSUPPORTED);

    si::srv::GetEntityState::Request robot;
    robot.entity = "/robot";
    const auto state = call<si::srv::GetEntityState>("/get_entity_state", robot);
    EXPECT_EQ(state->result.result, si::msg::Result::RESULT_OK);
    EXPECT_EQ(state->state.header.frame_id, "world");
    EXPECT_DOUBLE_EQ(state->state.pose.position.y, 2.0);
    EXPECT_DOUBLE_EQ(state->state.twist.angular.z, 0.25);

    si::srv::GetEntityState::Request missing;
    missing.entity = "/nope";
    EXPECT_EQ(call<si::srv::GetEntityState>("/get_entity_state", missing)->result.result,
              si::msg::Result::RESULT_NOT_FOUND);
}

}  // namespace
}  // namespace lagrange::ros2
