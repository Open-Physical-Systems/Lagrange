// GPIO interfaces of LagrangeSystem (RFC 0020 §5): a <gpio> of the <ros2_control> block bound to
// an actuator's signals, driven by gpio_controllers' GpioCommandController loaded by the
// standard spawner: its command reaches act/<gpio>/command, and act/<gpio>/current and held come
// back as state interfaces on its gpio_states topic.
#include "lagrange/ros2_control/ControllerHost.hpp"
#include "lagrange/scheduler/Scheduler.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <future>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include <control_msgs/msg/dynamic_interface_group_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace lagrange::ros2_control {
namespace {

using namespace literals;
using namespace std::chrono_literals;

/// An actuator as the actuators domain declares one: command in, current (A) and held (Bool) out.
class Coil final : public IModule {
public:
    [[nodiscard]] ModuleInfo info() const override { return {"actuators", "coil"}; }
    [[nodiscard]] Duration preferredStep() const override { return 1_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec command;
        command.name = "act/plunger/command";
        command.unit = units::one;
        command.optional = true;
        command.defaultValue = 0.0;
        command_ = d.input(command);
        SignalSpec current;
        current.name = "act/plunger/current";
        current.unit = units::A;
        current_ = d.output(current);
        SignalSpec held;
        held.name = "act/plunger/held";
        held.unit = units::one;
        held.type = SignalType::Bool;
        held_ = d.output(held);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        bus.write(current_, 0.0);
        bus.write(held_, 0.0);
        return {};
    }
    Status step(const StepContext&, SignalBus& bus) override {
        const double value = bus.read(command_);
        if (std::isfinite(value)) {
            seen = value;
        }
        bus.write(current_, 0.5 * seen);
        bus.write(held_, seen > 0.5 ? 1.0 : 0.0);
        return {};
    }
    double seen{0.0};

private:
    SignalId command_, current_, held_;
};

constexpr char kUrdf[] = R"(<?xml version="1.0"?>
<robot name="gpio_rig">
  <link name="base"/>
  <ros2_control name="Rig" type="system">
    <hardware><plugin>lagrange_ros2_control/LagrangeSystem</plugin></hardware>
    <gpio name="plunger">
      <command_interface name="command"/>
      <state_interface name="current"/>
      <state_interface name="held"/>
    </gpio>
  </ros2_control>
</robot>)";

TEST(Gpio, InterfacesAreBoundToActuatorSignals) {
    auto gpios = controlGpios(kUrdf);
    ASSERT_TRUE(gpios) << gpios.error();
    ASSERT_EQ(gpios->size(), 1U);
    EXPECT_EQ((*gpios)[0].commands.at(0).signal, "act/plunger/command");
    EXPECT_EQ((*gpios)[0].states.at(1).signal, "act/plunger/held");
    // RFC 0020 §4.1's suction names, and a signal of one's own with its unit.
    auto suction = controlGpios(R"(<robot name="r"><ros2_control name="R" type="system">
      <hardware><plugin>lagrange_ros2_control/LagrangeSystem</plugin></hardware>
      <gpio name="cup"><command_interface name="vacuum"/><state_interface name="holding"/>
        <state_interface name="bar"><param name="signal">act/cup/pressure</param></state_interface>
      </gpio></ros2_control></robot>)");
    ASSERT_TRUE(suction) << suction.error();
    EXPECT_EQ((*suction)[0].commands.at(0).signal, "act/cup/command");
    EXPECT_EQ((*suction)[0].states.at(0).signal, "act/cup/held");
    EXPECT_EQ((*suction)[0].states.at(1).signal, "act/cup/pressure");
}

TEST(Gpio, TheGpioCommandControllerDrivesTheActuator) {
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    auto gpios = controlGpios(kUrdf);
    ASSERT_TRUE(gpios) << gpios.error();
    auto host = ControllerHost::start(kUrdf, LAGRANGE_TEST_GPIO_PARAMS);
    ASSERT_TRUE(host) << host.error();
    World world;
    {
        // A signal Lagrange does not know, without its unit: the start stops and names it. (On
        // the same host: a controller manager started and stopped without a cycle races its own
        // shutdown.)
        auto unknown = controlGpios(R"(<robot name="r"><ros2_control name="R" type="system">
          <hardware><plugin>lagrange_ros2_control/LagrangeSystem</plugin></hardware>
          <gpio name="lamp"><command_interface name="brightness"/></gpio></ros2_control></robot>)");
        ASSERT_TRUE(unknown);
        Scheduler refused;
        refused.add(createControlModule(**host, {}, *unknown));
        auto wired = refused.wire(WorldView(world));
        ASSERT_FALSE(wired);
        EXPECT_NE(to_string(wired.error().front()).find("act/lamp/brightness"), std::string::npos);
    }
    Scheduler scheduler;
    auto coil = std::make_unique<Coil>();
    Coil* sim = coil.get();
    scheduler.add(createControlModule(**host, {}, *gpios));
    scheduler.add(std::move(coil));
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_TRUE(wired) << to_string(wired.error().front());

    auto spawned = std::async(std::launch::async, [] {
        const std::string command =
            std::string("ros2 run controller_manager spawner plunger_gpio --param-file ") +
            LAGRANGE_TEST_GPIO_PARAMS + " --controller-manager-timeout 20 > /dev/null 2>&1";
        return std::system(command.c_str());
    });
    const auto deadline = std::chrono::steady_clock::now() + 60s;
    while (spawned.wait_for(0s) != std::future_status::ready &&
           std::chrono::steady_clock::now() < deadline) {
        ASSERT_TRUE(scheduler.stepUntil(scheduler.now() + 10_ms).ok());
        std::this_thread::sleep_for(2ms);
    }
    ASSERT_EQ(spawned.get(), 0) << "the spawner failed";

    auto node = std::make_shared<rclcpp::Node>("gpio_test");
    double current = std::nan("");
    double held = std::nan("");
    auto states = node->create_subscription<control_msgs::msg::DynamicInterfaceGroupValues>(
        "/plunger_gpio/gpio_states", rclcpp::SystemDefaultsQoS(),
        [&](const control_msgs::msg::DynamicInterfaceGroupValues& m) {
            for (std::size_t g = 0; g < m.interface_groups.size(); ++g) {
                const auto& values = m.interface_values[g];
                for (std::size_t k = 0; k < values.interface_names.size(); ++k) {
                    if (values.interface_names[k] == "current") {
                        current = values.values[k];
                    } else if (values.interface_names[k] == "held") {
                        held = values.values[k];
                    }
                }
            }
        });
    auto publisher = node->create_publisher<control_msgs::msg::DynamicInterfaceGroupValues>(
        "/plunger_gpio/commands", rclcpp::SystemDefaultsQoS());
    control_msgs::msg::DynamicInterfaceGroupValues command;
    command.interface_groups = {"plunger"};
    command.interface_values.resize(1);
    command.interface_values[0].interface_names = {"command"};
    command.interface_values[0].values = {1.0};
    for (int i = 0; i < 300 && !(current == 0.5 && held == 1.0); ++i) {
        publisher->publish(command);
        ASSERT_TRUE(scheduler.stepUntil(scheduler.now() + 10_ms).ok());
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(2ms);
    }
    EXPECT_EQ(sim->seen, 1.0) << "the controller's command reached act/plunger/command";
    EXPECT_EQ(current, 0.5) << "act/plunger/current came back as a state interface";
    EXPECT_EQ(held, 1.0) << "act/plunger/held (Bool) came back";
    host->reset();
}

}  // namespace
}  // namespace lagrange::ros2_control
