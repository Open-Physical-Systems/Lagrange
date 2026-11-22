#pragma once

// The actuators domain's solenoid module (RFC 0020 §4.6): every prismatic joint with
// SolenoidAPI_1, stepped at 100 µs. Its command act/<s>/command (0 to 1: off, on, or a PWM duty
// as the coil sees it on average) puts command × ratedVoltage across the coil; the coil's flux,
// its current and the plunger's pull follow Solenoid.hpp; the pull and the return spring drive
// the joint through joint/<j>/torque_command (N on a slide), which the electrical domain (or its
// substitute) passes on, as for any joint without a motor. The joint's limits are the stops.
//
// Out: act/<s>/current (A), voltage (V), force (N, the magnetic pull alone), position (m, the
// plunger's), power (W, drawn from the supply). The supply is not a bus of the electrical
// network yet (as for MagnetGripperAPI_1): TODO(RFC 0020) a solenoid as a load on its bus.
//
// Substitutes: `ideal` puts the steady current V/R through the coil at once (no inductance's
// delay, no dip); `zero` and `constant` leave it unpowered. The spring acts in every one: it is
// the device's, not its electronics'.

#include "lagrange/actuators/Grippers.hpp"
#include "lagrange/actuators/Solenoid.hpp"
#include "lagrange/module/IModule.hpp"

#include <memory>
#include <string>
#include <vector>

namespace lagrange::actuators {

/// One solenoid of the stage.
struct SolenoidSpec {
    std::string name;   // the joint's signal instance name: act/<name>/...
    std::string joint;  // the joint's name in World::Joints
    PrimId prim{};
    SolenoidParams params;
};

/// Every solenoid of the world; an error names what is wrong.
[[nodiscard]] std::vector<SolenoidSpec> solenoidsOf(const WorldView& world, std::string& error);

class Solenoids final : public IModule {
public:
    enum class Mode : std::uint8_t { Full, Ideal, Off };

    /// `commands`: given outside the bus (Server::commandActuator), keyed by the solenoid's name.
    explicit Solenoids(Mode mode = Mode::Full, std::shared_ptr<Commands> commands = {})
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
        SignalId position, command;
    };
    struct Out {
        SignalId force, current, voltage, pull, plunger, power;
    };
    struct State {
        double flux{0.0};     // Wb
        double command{0.0};  // 0..1
        double busCommand{0.0};
    };

    Mode mode_;
    std::shared_ptr<Commands> commands_;
    std::uint64_t seenGeneration_{0};
    std::vector<SolenoidSpec> specs_;
    std::vector<In> in_;
    std::vector<Out> out_;
    std::vector<State> state_;
};

}  // namespace lagrange::actuators
