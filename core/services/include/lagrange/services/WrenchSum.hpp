#pragma once

// Core services (roadmap 0.2-A12): modules every run may need, owned by no domain.

#include "lagrange/module/IModule.hpp"

#include <string>
#include <vector>

namespace lagrange::services {

/// Sums the wrenches several modules put on one body (docs/design/domains/aerodynamics.md §3).
///
/// A signal has one writer, so aerodynamics, rotors and hydrodynamics cannot all write
/// body/<b>/external_wrench, which the rigid-body backend reads. Each writes
/// body/<b>/wrench/<source> instead (Wrench: N then N·m, frame "world_local"), and this module
/// writes their sum to body/<b>/external_wrench for every body that has at least one.
///
/// It finds the contributions among the outputs declared before it (SignalDeclarer::
/// declaredOutputs), so it must be added to the scheduler after the modules it sums. A
/// contribution in another frame or unit is refused at declare. It steps at 1 ms by default;
/// its inputs average over its step (a flow: the impulse is conserved across rates).
class WrenchSum final : public IModule {
public:
    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

    /// The bodies with contributions, with the sources summed for each, after declare.
    struct Body {
        std::string name;
        std::vector<std::string> sources;
        std::vector<SignalId> inputs;
        SignalId output;
    };
    [[nodiscard]] const std::vector<Body>& bodies() const { return bodies_; }

private:
    std::vector<Body> bodies_;
};

}  // namespace lagrange::services
