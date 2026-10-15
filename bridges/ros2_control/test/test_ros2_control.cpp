// ros2_control in the loop: a forward_command_controller loaded by the standard spawner commands
// a simulated wheel's velocity through LagrangeSystem, and the servo makes the wheel follow.
#include "lagrange/ros2_control/ControllerHost.hpp"
#include "lagrange/scheduler/Scheduler.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <future>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace lagrange::ros2_control {
namespace {

using namespace literals;
using namespace std::chrono_literals;

constexpr double kWheelInertia = 0.05;  // kg·m²

/// A free wheel: integrates joint/wheel/torque into velocity and position.
class Wheel final : public IModule {
public:
    [[nodiscard]] ModuleInfo info() const override { return {"rigid_body", "wheel"}; }
    [[nodiscard]] Duration preferredStep() const override { return 1_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        const auto make = [](const char* name, Unit unit) {
            SignalSpec s;
            s.name = name;
            s.unit = unit;
            return s;
        };
        SignalSpec torque = make("joint/wheel/torque", units::N_m);
        torque.hold = Hold::WindowAverage;
        torque_ = d.input(torque);
        position_ = d.output(make("joint/wheel/position", units::rad));
        velocity_ = d.output(make("joint/wheel/velocity", units::rad_per_s));
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        bus.write(position_, 0.0);
        bus.write(velocity_, 0.0);
        return {};
    }
    Status step(const StepContext& c, SignalBus& bus) override {
        velocity += bus.read(torque_) / kWheelInertia * c.dt.seconds();
        position += velocity * c.dt.seconds();
        bus.write(position_, position);
        bus.write(velocity_, velocity);
        return {};
    }
    double velocity{0.0}, position{0.0};

private:
    SignalId torque_, position_, velocity_;
};

constexpr char kUrdf[] = R"(<?xml version="1.0"?>
<robot name="wheel_rig">
  <link name="base"/>
  <link name="rim">
    <inertial><mass value="1"/><inertia ixx="0.05" iyy="0.05" izz="0.05" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>
  <joint name="wheel" type="continuous">
    <parent link="base"/><child link="rim"/><axis xyz="0 1 0"/>
    <limit effort="5.0" velocity="20.0"/>
  </joint>
  <ros2_control name="Rig" type="system">
    <hardware><plugin>lagrange_ros2_control/LagrangeSystem</plugin></hardware>
    <joint name="wheel">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <param name="velocity_p">2.0</param>
    </joint>
  </ros2_control>
</robot>)";

TEST(Ros2Control, AControllerDrivesTheWheelThroughTheServo) {
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    auto joints = controlJoints(kUrdf);
    ASSERT_TRUE(joints) << joints.error();
    ASSERT_EQ(joints->size(), 1U);
    EXPECT_DOUBLE_EQ((*joints)[0].gains.velocityP, 2.0) << "gains come from the URDF";
    EXPECT_DOUBLE_EQ((*joints)[0].gains.velocityI, ServoGains{}.velocityI);

    auto host = ControllerHost::start(kUrdf, LAGRANGE_TEST_PARAMS);
    ASSERT_TRUE(host) << host.error();
    EXPECT_EQ((*host)->updateRateHz(), 100U);

    World world;
    const BodyId rim =
        world.addBody("rim", kWorldBody, Pose{}, 1.0, {}, {0.05, 0.05, 0.05, 0, 0, 0});
    world.addJoint("wheel", JointType::Continuous, rim, {0, 1, 0}, true);
    world.joints.effortLimit.back() = 5.0;
    Scheduler scheduler;
    auto wheel = std::make_unique<Wheel>();
    Wheel* sim = wheel.get();
    scheduler.add(std::move(wheel));
    scheduler.add(createControlModule(**host, *joints));
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_TRUE(wired) << to_string(wired.error().front());

    // The standard spawner, as a launch file would run it, while the simulation steps: the
    // switch to active completes in the controller manager's update, which the scheduler calls.
    auto spawned = std::async(std::launch::async, [] {
        const std::string command = std::string(
                                        "ros2 run controller_manager spawner "
                                        "wheel_velocity --param-file ") +
                                    LAGRANGE_TEST_PARAMS +
                                    " --controller-manager-timeout 20 > /dev/null 2>&1";
        return std::system(command.c_str());
    });
    const auto deadline = std::chrono::steady_clock::now() + 60s;
    while (spawned.wait_for(0s) != std::future_status::ready &&
           std::chrono::steady_clock::now() < deadline) {
        ASSERT_TRUE(scheduler.stepUntil(scheduler.now() + 10_ms).ok());
        std::this_thread::sleep_for(2ms);
    }
    ASSERT_EQ(spawned.get(), 0) << "the spawner failed";

    auto node = std::make_shared<rclcpp::Node>("ros2_control_test");
    auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/wheel_velocity/commands", rclcpp::SystemDefaultsQoS());
    std_msgs::msg::Float64MultiArray command;
    command.data = {3.0};  // rad/s
    for (int i = 0; i < 300 && std::abs(sim->velocity - 3.0) > 0.01; ++i) {
        publisher->publish(command);
        ASSERT_TRUE(scheduler.stepUntil(scheduler.now() + 10_ms).ok());
        std::this_thread::sleep_for(2ms);
    }
    EXPECT_NEAR(sim->velocity, 3.0, 0.01) << "the servo follows the controller's command";
    // Saturation: 5 N·m on 0.05 kg·m² can reach 100 rad/s², so 3 rad/s takes at least 30 ms.
    EXPECT_GT(scheduler.now().ns, 30'000'000);
    host->reset();
}

}  // namespace
}  // namespace lagrange::ros2_control
