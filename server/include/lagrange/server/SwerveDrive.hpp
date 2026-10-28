#pragma once

#include "lagrange/module/IModule.hpp"
#include "lagrange/world/Model.hpp"

#include <optional>
#include <string>
#include <vector>

namespace lagrange::server {

/// Turns the base velocity command (base/command/linear_velocity, lateral_velocity,
/// angular_velocity) into steer and wheel torque commands (joint/<j>/torque_command) for a
/// swerve base: wheels that each steer about a vertical axis.
///
/// Per module at (x, y) from the base's centre the wheel's ground velocity is
/// (vx - wz·y, vy + wz·x); the module steers to its direction and the wheel turns at its speed.
/// A module never turns more than 90°: past that it steers the other way and reverses the
/// wheel, and its speed is scaled by the cosine of the steering error until it points right.
/// With no command a module keeps its angle. Steering is a PD loop on the steer angle, the wheel
/// a PI loop on its speed, both clamped to the joints' effort limits.
class SwerveController final : public IModule {
public:
    struct Module {
        std::string steerJoint;
        std::string wheelJoint;
        double xM{0.0};  // steer axis from the base's centre, base frame
        double yM{0.0};
        double steerTorqueLimitNm{20.0};
        double wheelTorqueLimitNm{10.0};
    };
    struct Settings {
        std::vector<Module> modules;
        double wheelRadiusM{0.10};
        double steerGainNmPerRad{40.0};
        double steerDampingNmPerRadPerS{1.5};
        double wheelGainNmPerRadPerS{0.5};
        double wheelIntegralGainNmPerRad{4.0};
    };

    explicit SwerveController(Settings settings);

    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

private:
    struct Signals {
        SignalId steerAngle, steerRate, wheelSpeed, steerTorque, wheelTorque;
    };
    Settings settings_;
    SignalId linear_, lateral_, angular_;
    std::vector<Signals> signals_;
    std::vector<double> wheelIntegral_;
};

/// The swerve modules of a robot: actuated joints that steer about its vertical axis, each
/// carrying an actuated wheel joint; at least three of them, placed about their centre (where
/// the base turns), with the wheel radius from the wheels' cylinder or sphere colliders and the
/// torque limits from the joints' effort limits. Nothing for any other robot.
[[nodiscard]] std::optional<SwerveController::Settings> findSwerveModules(const Model& robot);

}  // namespace lagrange::server
