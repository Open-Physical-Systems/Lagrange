#pragma once

#include "lagrange/module/IModule.hpp"

#include <functional>
#include <optional>
#include <string>

namespace lagrange::server {

/// A velocity command for a ground robot: forward speed, turn rate and, for a robot that moves
/// sideways (a swerve base), the speed to the left.
struct VelocityCommand {
    double linearMps{0.0};
    double angularRadps{0.0};
    double lateralMps{0.0};
};

/// Writes the base velocity command: base/command/linear_velocity (m/s) and
/// base/command/angular_velocity (rad/s), and with Settings::lateral also
/// base/command/lateral_velocity (m/s, to the left).
///
/// The command comes from `external` (the ROS 2 bridge's /cmd_vel), which returns nothing
/// while no fresh command exists. Until the first external command, the module plays a demo:
/// a figure-eight (a left loop, then a right one), or with Settings::lateral a square driven
/// without turning (forward, left, back, right) and then a full turn on the spot. After it, a
/// missing command means stop.
/// With an external source the module is not deterministic (its input comes from outside).
class DriveCommand final : public IModule {
public:
    struct Settings {
        double speedMps{0.6};     // demo forward speed
        double turnRadiusM{1.2};  // demo loop radius
        double rampSeconds{2.0};  // demo time to full speed
        bool demo{true};          // play the figure-eight until an external command arrives
        bool lateral{false};      // the robot moves sideways: write the lateral velocity
    };

    DriveCommand(Settings settings, std::function<std::optional<VelocityCommand>()> external);

    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

    /// Where the current command comes from, for the console: "demo", "ros", "stopped".
    [[nodiscard]] const char* source() const { return source_; }

private:
    Settings settings_;
    std::function<std::optional<VelocityCommand>()> external_;
    bool externalSeen_{false};
    const char* source_{"demo"};
    SignalId linear_, angular_, lateral_;
};

/// Turns the base velocity command into wheel torque commands (joint/<j>/torque_command) for a
/// two-wheel differential drive,
/// with a PI loop on each wheel's speed. It stands in for ros2_control's
/// diff_drive_controller until the ros2_control bridge exists (roadmap 0.2-F3).
class DiffDriveController final : public IModule {
public:
    struct Settings {
        std::string leftJoint;
        std::string rightJoint;
        double wheelRadiusM{0.10};
        double trackWidthM{0.46};  // distance between the wheel contact points
        double gainNmPerRadPerS{2.0};
        double integralGainNmPerRad{8.0};
        double torqueLimitNm{10.0};  // the wheel's effort limit
    };

    explicit DiffDriveController(Settings settings);

    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

private:
    Settings settings_;
    SignalId linear_, angular_, leftVelocity_, rightVelocity_, leftTorque_, rightTorque_;
    double leftIntegral_{0.0};
    double rightIntegral_{0.0};
};

}  // namespace lagrange::server
