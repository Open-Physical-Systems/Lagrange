#pragma once

#include "lagrange/module/IModule.hpp"

#include <memory>

namespace lagrange::rigid_body {

/// The rigid-body domain on Jolt Physics 5.6, the second backend (rigid_body.md; spike 0.2-C6,
/// docs/plan/spikes/0.2-C6-jolt.md). The same signals as MujocoBackend, so either runs a profile:
/// joint/j/position and velocity for hinges and slides; for actuated ones joint/j/torque and
/// load_torque in (optional, WindowAverage) and acceleration, effective_inertia and
/// constraint_torque out; for every moving body body/b/pose, twist and acceleration (proper,
/// body frame, as an IMU feels it) out and external_wrench in; env/friction_scale in.
///
/// Where Jolt differs from MuJoCo:
/// - Bodies joined by fixed joints are one Jolt body (masses and inertias combined).
/// - Accelerations are the change of velocity over the step (Jolt has no qacc).
/// - constraint_torque is the joint limit's reaction only; MuJoCo's includes contacts.
/// - Rotor inertia (armature) is applied as joint torques computed from the joints' inertia
///   matrix, one step late (Impl::updateArmatureTorques); MuJoCo has it in its joint space.
/// - Friction is per Jolt body (the largest of its colliders); a contact takes the larger of
///   the two bodies', as MuJoCo does.
/// - No spherical joints yet.
/// Deterministic: Jolt built cross-platform deterministic, stepped on one thread.
class JoltBackend final : public IModule {
public:
    JoltBackend();
    ~JoltBackend() override;
    JoltBackend(const JoltBackend&) = delete;
    JoltBackend& operator=(const JoltBackend&) = delete;
    JoltBackend(JoltBackend&&) = delete;
    JoltBackend& operator=(JoltBackend&&) = delete;

    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    void shutdown() override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;
    void shiftOrigin(const std::array<double, 3>& shift) override;

    struct Impl;

private:
    std::unique_ptr<Impl> impl_;
};

}  // namespace lagrange::rigid_body
