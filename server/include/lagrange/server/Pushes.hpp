#pragma once

#include "lagrange/module/IModule.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace lagrange::server {

/// One push on a body: a constant wrench (N then N·m, world_local, at the body's centre of mass)
/// from `start` for `duration` (s, simulated time).
struct Push {
    std::string body;
    double start{0.0};
    double duration{0.0};
    std::array<double, 6> wrench{};
};

/// The pushes waiting for the module, set by the caller (Server::setPushes) between steps.
class PushSchedule {
public:
    void replace(std::vector<Push> pushes);
    /// Copies the pushes into `out` if they changed since version `seen`.
    void refresh(std::uint64_t& seen, std::vector<Push>& out) const;

private:
    mutable std::mutex mutex_;
    std::vector<Push> pushes_;
    std::atomic<std::uint64_t> version_{1};
};

/// External push disturbances (domain randomisation for learning, docs/guide/rl.md): writes
/// body/<b>/wrench/push for each body it was given, which WrenchSum adds to what else pushes it.
/// Over each step it writes the mean of the pushes that act in it, weighted by the time they act,
/// so the impulse a push gives is its wrench times its duration whatever the step (a flow).
/// Its schedule is an input from outside, like a joint command, not part of the run's state.
class Pushes final : public IModule {
public:
    Pushes(std::shared_ptr<PushSchedule> schedule, std::vector<std::string> bodies);

    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

private:
    struct Body {
        std::string name;  // as World::Bodies::names
        SignalId wrench;
    };
    std::shared_ptr<PushSchedule> schedule_;
    std::vector<std::string> names_;
    std::vector<Body> bodies_;
    std::uint64_t seen_{0};
    std::vector<Push> pushes_;
};

}  // namespace lagrange::server
