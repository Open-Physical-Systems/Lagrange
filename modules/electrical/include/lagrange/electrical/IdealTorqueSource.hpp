#pragma once

#include "lagrange/module/IModule.hpp"

#include <string>
#include <vector>

namespace lagrange::electrical {

/// The electrical domain's `ideal` substitute (CLAUDE.md §11.5, domains/electrical.md §5): the
/// commanded torque reaches the joint exactly, as if motors, drives and the battery were
/// perfect.
///
/// For every joint/<j>/torque_command declared before it (by a controller: the built-in diff
/// drive, ros2_control, a script), it writes joint/<j>/torque with the same unit (N·m, or N for
/// a slide). Add it after the controllers. Stateless and deterministic.
class IdealTorqueSource final : public IModule {
public:
    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

    /// The joints it drives, after declare.
    [[nodiscard]] const std::vector<std::string>& joints() const { return joints_; }

private:
    struct Joint {
        SignalId torqueCommand;
        SignalId torqueOut;
    };
    std::vector<Joint> pairs_;
    std::vector<std::string> joints_;
};

}  // namespace lagrange::electrical
