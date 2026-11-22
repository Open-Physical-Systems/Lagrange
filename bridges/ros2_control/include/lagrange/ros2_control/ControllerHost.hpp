#pragma once

#include "lagrange/module/Expected.hpp"
#include "lagrange/module/IModule.hpp"
#include "lagrange/world/World.hpp"

#include <memory>
#include <string>
#include <vector>

namespace lagrange::ros2_control {

/// A ros2_control controller manager inside the simulator (roadmap 0.2-F3, docs/design/
/// robot-classes.md §2.2), as gz_ros2_control and mujoco_ros2_control run one: node
/// /controller_manager on its own executor thread, so `ros2 control` and the controller
/// spawners talk to it as to any robot. The hardware is lagrange_ros2_control/LagrangeSystem,
/// named by the URDF's <ros2_control> blocks.
///
/// It outlives the scheduler: a reset rebuilds the modules, not the controllers.
class ControllerHost {
public:
    /// rclcpp must be initialised. `paramsFile` holds the controller manager's and the
    /// controllers' parameters (update_rate, controller types); use_sim_time is forced on.
    static Expected<std::unique_ptr<ControllerHost>, std::string> start(
        const std::string& urdf, const std::string& paramsFile);
    ~ControllerHost();
    ControllerHost(const ControllerHost&) = delete;
    ControllerHost& operator=(const ControllerHost&) = delete;
    ControllerHost(ControllerHost&&) = delete;
    ControllerHost& operator=(ControllerHost&&) = delete;

    /// The controller manager's update rate (its update_rate parameter).
    [[nodiscard]] unsigned int updateRateHz() const;
    /// One read/update/write cycle at simulation time `nowNs`, `periodNs` after the last.
    void cycle(std::int64_t nowNs, std::int64_t periodNs);

    struct Impl;

private:
    explicit ControllerHost(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

/// Joint servo gains (docs/design/robot-classes.md §2.2), from <param>s of the joint in the
/// <ros2_control> block: velocity_p (N·m per rad/s), velocity_i (N·m per rad), position_p
/// (N·m per rad), position_d (N·m per rad/s, on the error between the command's rate and the
/// joint's speed: velocity feedforward); N and m for prismatic joints.
struct ServoGains {
    double velocityP{5.0};
    double velocityI{20.0};
    double positionP{100.0};
    double positionD{10.0};
    /// Position integral (N·m per rad·s): holds a joint against a steady load such as gravity.
    double positionI{0.0};
};

struct ControlJoint {
    std::string name;
    ServoGains gains;
};

/// The joints of the URDF's <ros2_control> blocks that use LagrangeSystem, with their gains.
[[nodiscard]] Expected<std::vector<ControlJoint>, std::string> controlJoints(
    const std::string& urdf);

/// A <gpio> of the URDF's <ros2_control> blocks that use LagrangeSystem (RFC 0020 §5): each of
/// its interfaces is bound to a signal, `act/<gpio>/<interface>` unless the interface's
/// <param name="signal"> names another; its unit and type come from the actuator signal of that
/// name (Bool for held, sealed, overload) or from <param name="unit"> ("Pa", "1"...) and
/// <param name="type"> (bool or scalar). Aliases of RFC 0020 §4.1: `vacuum` is `command`,
/// `holding` is `held`, `tank` is `tank_mass`.
struct ControlGpio {
    struct Interface {
        std::string name;    // as the controllers see it: "<gpio>/<name>"
        std::string signal;  // the bus signal it reads or writes
        std::string unit;    // empty: by the signal's name
        std::string type;    // empty: by the signal's name
    };
    std::string name;
    std::vector<Interface> commands;
    std::vector<Interface> states;
};

/// The GPIOs of the URDF's <ros2_control> blocks that use LagrangeSystem.
[[nodiscard]] Expected<std::vector<ControlGpio>, std::string> controlGpios(const std::string& urdf);

/// The scheduler module that steps `host` (domain "control", name "ros2_control", 1 ms): it
/// reads joint/j/position and velocity, runs the controller manager every 1/update_rate, and
/// writes joint/j/torque from the claimed command: effort directly, velocity and position
/// through the servo, clamped to the joint's effort limit. Each GPIO's command interfaces are
/// written to their signals (NaN until a controller sets one) and its state interfaces read from
/// theirs, every controller cycle.
[[nodiscard]] std::unique_ptr<IModule> createControlModule(ControllerHost& host,
                                                           std::vector<ControlJoint> joints,
                                                           std::vector<ControlGpio> gpios = {});

}  // namespace lagrange::ros2_control
