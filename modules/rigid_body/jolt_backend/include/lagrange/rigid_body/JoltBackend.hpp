#pragma once

#include "lagrange/module/IModule.hpp"

#include <memory>

namespace lagrange::rigid_body {

/// The rigid-body domain on Jolt Physics 5.6 (spike 0.2-C6; docs/plan/spikes/0.2-C6-jolt.md).
///
/// Same signals as MujocoBackend for what the spike covers: joint/j/position and velocity for
/// hinges and slides, joint/j/torque in for actuated ones (WindowAverage), and for every moving
/// body body/b/pose and body/b/twist out and body/b/external_wrench in (optional). Bodies joined
/// by fixed joints are one Jolt body (masses and inertias combined); static scenery and the
/// ground plane are static bodies. Deterministic: Jolt built cross-platform deterministic,
/// stepped on one thread. Not covered yet (the spike report lists them): joint acceleration,
/// effective inertia and constraint torque, load torque, body acceleration, spherical joints,
/// joint limits' softness, env/friction_scale.
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

    struct Impl;

private:
    std::unique_ptr<Impl> impl_;
};

}  // namespace lagrange::rigid_body
