#pragma once

#include "lagrange/module/IModule.hpp"

#include <cstdint>
#include <vector>

// MuJoCo's types, so this header does not include MuJoCo.
struct mjModel_;
struct mjData_;

namespace lagrange::rigid_body {

/// The rigid-body domain on MuJoCo 3.12 (docs/design/domains/rigid_body.md, spike 0.2-C1).
///
/// init() builds a MuJoCo model from the world tables (bodies, joints, colliders): no MJCF file
/// and no USD are involved, so the stage stays the only source of the robot (invariant 5).
/// Settings from the validation spike: elliptic friction cone, implicitfast integrator.
///
/// Signals per actuated joint `j` (prismatic joints use N, m, m/s, m/s², kg):
///   in  joint/j/torque             N·m, WindowAverage
///   in  joint/j/load_torque        N·m, WindowAverage, optional
///   out joint/j/position, velocity, acceleration, effective_inertia, constraint_torque
/// per dynamic body `b`:
///   in  body/b/external_wrench     Wrench N, world_local, at the centre of mass, optional
///   out body/b/pose                Pose m, world_local
///   out body/b/twist               Wrench m/s then rad/s, world_local, at the body origin
///   out body/b/acceleration        Wrench m/s² then rad/s², body frame, proper acceleration
/// and   env/friction_scale         1, optional, default 1 (wet surfaces, from weather).
///
/// Outputs describe the state at the end of each step. Accelerations and constraint torques are
/// those of the step just taken.
class MujocoBackend final : public IModule {
public:
    MujocoBackend();
    ~MujocoBackend() override;
    MujocoBackend(const MujocoBackend&) = delete;
    MujocoBackend& operator=(const MujocoBackend&) = delete;
    MujocoBackend(MujocoBackend&&) = delete;
    MujocoBackend& operator=(MujocoBackend&&) = delete;

    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;

    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    void shutdown() override;

    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

    /// The compiled model, for tests and tools; null before init().
    [[nodiscard]] const mjModel_* model() const { return model_; }

private:
    struct JointSignals {
        std::uint32_t joint{0};
        SignalId torque, loadTorque;
        SignalId position, velocity, acceleration, effectiveInertia, constraintTorque;
        int qposAddress{0};
        int dofAddress{0};
    };
    struct BodySignals {
        std::uint32_t body{0};
        SignalId externalWrench, pose, twist, acceleration;
        int mujocoBody{0};
    };

    void writeOutputs(SignalBus& bus) const;

    std::vector<JointSignals> joints_;
    std::vector<BodySignals> bodies_;
    SignalId frictionScale_;
    double appliedFrictionScale_{1.0};
    std::vector<double> baseFriction_;  // per geom, sliding coefficient at scale 1
    mjModel_* model_{nullptr};
    mjData_* data_{nullptr};
};

}  // namespace lagrange::rigid_body
