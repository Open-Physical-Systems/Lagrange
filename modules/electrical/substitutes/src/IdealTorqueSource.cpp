#include "lagrange/electrical/IdealTorqueSource.hpp"

#include <string_view>

namespace lagrange::electrical {

using namespace literals;

namespace {

constexpr std::string_view kPrefix = "joint/";
constexpr std::string_view kCommand = "/torque_command";

}  // namespace

ModuleInfo IdealTorqueSource::info() const {
    return {"electrical", "ideal_torque_source", 0, 1, true, true, CapabilityTier::Tier0};
}

Duration IdealTorqueSource::preferredStep() const {
    return 1_ms;
}

Status IdealTorqueSource::declare(const WorldView&, const ModuleConfig&, SignalDeclarer& declarer) {
    pairs_.clear();
    joints_.clear();
    for (const SignalSpec& command : declarer.declaredOutputs(kPrefix)) {
        const std::string_view name = command.name;
        if (!name.ends_with(kCommand)) {
            continue;
        }
        const std::string joint(
            name.substr(kPrefix.size(), name.size() - kPrefix.size() - kCommand.size()));
        SignalSpec in = command;
        SignalSpec out = command;
        out.name = std::string(kPrefix) + joint + "/torque";
        pairs_.push_back({declarer.input(in), declarer.output(out)});
        joints_.push_back(joint);
    }
    return {};
}

Status IdealTorqueSource::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    for (const Joint& j : pairs_) {
        bus.write(j.torqueOut, 0.0);
    }
    return {};
}

// The body of CLAUDE.md §11.5.
Status IdealTorqueSource::step(const StepContext&, SignalBus& bus) {
    for (const Joint& j : pairs_) {
        bus.write(j.torqueOut, bus.read(j.torqueCommand));
    }
    return {};
}

Status IdealTorqueSource::saveState(StateWriter&) const {
    return {};
}

Status IdealTorqueSource::restoreState(StateReader&) {
    return {};
}

}  // namespace lagrange::electrical
