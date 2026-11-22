#pragma once

// The actuators domain's holding brakes (RFC 0020 "Also worth adding"): every joint with
// BrakeAPI_1, stepped at 1 ms. Its command act/<b>/release (0 to 1: the share of ratedVoltage
// on the coil) powers the coil; Brake.hpp gives its current and whether the springs hold the
// disc; while applied it writes joint/<j>/friction_limit (the engines' optional Coulomb limit,
// RFC 0020 §4.4, made stiff in MuJoCo because it has a writer): the holding torque at rest, the
// dynamic torque while the joint slips. A power-off brake: with no command it holds.
//
// Out: act/<b>/current (A), voltage (V), power (W), engaged (Bool), torque (N·m or N, the
// friction limit written).
//
// Substitutes: `ideal` switches at once on the command (no coil delay); `constant` holds always;
// `zero` never holds (as if the joint had no brake).

#include "lagrange/actuators/Brake.hpp"
#include "lagrange/actuators/Grippers.hpp"
#include "lagrange/module/IModule.hpp"

#include <memory>
#include <string>
#include <vector>

namespace lagrange::actuators {

/// One brake of the stage.
struct BrakeSpec {
    std::string name;   // the joint's signal instance name: act/<name>/...
    std::string joint;  // the joint's name in World::Joints
    PrimId prim{};
    bool slide{false};  // a prismatic joint: N, m/s
    BrakeParams params;
};

/// Every brake of the world; an error names what is wrong.
[[nodiscard]] std::vector<BrakeSpec> brakesOf(const WorldView& world, std::string& error);

class Brakes final : public IModule {
public:
    enum class Mode : std::uint8_t { Full, Ideal, Hold, Off };

    /// `commands`: given outside the bus (Server::commandActuator), keyed by the brake's name.
    explicit Brakes(Mode mode = Mode::Full, std::shared_ptr<Commands> commands = {})
        : mode_(mode), commands_(std::move(commands)) {}

    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

private:
    struct In {
        SignalId velocity, release;
    };
    struct Out {
        SignalId friction, current, voltage, power, engaged, torque;
    };
    struct State {
        BrakeState brake;
        double command{0.0};
        double busCommand{0.0};
    };

    Mode mode_;
    std::shared_ptr<Commands> commands_;
    std::uint64_t seenGeneration_{0};
    std::vector<BrakeSpec> specs_;
    std::vector<In> in_;
    std::vector<Out> out_;
    std::vector<State> state_;
};

}  // namespace lagrange::actuators
