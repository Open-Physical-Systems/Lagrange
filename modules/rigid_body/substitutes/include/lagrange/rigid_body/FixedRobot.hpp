#pragma once

#include "lagrange/module/IModule.hpp"

namespace lagrange::rigid_body {

/// The rigid_body domain's "fixed" substitute (constant kind, CLAUDE.md §11.2): the robot
/// clamped at its rest pose, as on a bench. It declares the MuJoCo backend's signals, so every
/// consumer wires unchanged:
///
/// - joint/j/position, velocity (and for actuated joints acceleration) are 0; effective_inertia
///   is the child body's moment about the joint axis; constraint_torque holds the motor:
///   it is minus the commanded torque plus load.
/// - body/b/pose is the rest pose, twist 0, and acceleration the proper acceleration of a body
///   at rest: gravity reversed, in the body frame (an IMU reads 1 g up).
///
/// Inputs (torques, external wrenches, friction scale) are optional and only used for the
/// constraint torque. Costs nothing per step beyond copying a few values.
class FixedRobot final : public IModule {
public:
    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter&) const override { return {}; }  // no state
    Status restoreState(StateReader&) override { return {}; }

private:
    struct Joint {
        SignalId position, velocity, torque, loadTorque, acceleration, effectiveInertia,
            constraintTorque;
        double inertia{0.0};
    };
    struct Body {
        SignalId pose, twist, acceleration;
        std::array<double, 7> restPose{};
        std::array<double, 6> properAcceleration{};
    };
    std::vector<Joint> joints_;
    std::vector<Body> bodies_;
};

}  // namespace lagrange::rigid_body
