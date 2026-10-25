#pragma once

#include "lagrange/module/ModuleConfig.hpp"
#include "lagrange/module/StateIO.hpp"
#include "lagrange/module/Status.hpp"
#include "lagrange/signals/Signal.hpp"
#include "lagrange/signals/SignalBus.hpp"
#include "lagrange/time/SimTime.hpp"
#include "lagrange/world/World.hpp"

#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace lagrange {

/// What a module is. Normative in RFC 0001 and docs/design/architecture.md §5.
struct ModuleInfo {
    std::string_view domain;  // "electrical": the key in the profile's `domains`
    std::string_view name;    // "averaged_network", or for a substitute "ideal_torque_source"
    std::uint32_t versionMajor{0};
    std::uint32_t versionMinor{0};
    bool isSubstitute{false};
    bool deterministic{true};  // false for GPU paths that cannot guarantee bit-exact output
    CapabilityTier minimumTier{CapabilityTier::Tier0};
};

/// Passed to every step; new fields are appended so the step signature never changes.
struct StepContext {
    SimTime t;                   // time at the start of this step
    Duration dt;                 // this module's step
    std::uint64_t stepIndex{0};  // 0, 1, 2 ... for this module
    std::uint64_t seed{0};       // per (run, module) seed for lagrange::random::Stream
};

/// Handed to IModule::declare. Each call returns the id the module uses in step().
class SignalDeclarer {
public:
    SignalId input(SignalSpec spec);
    SignalId output(SignalSpec spec);

    [[nodiscard]] std::span<const SignalSpec> inputs() const { return inputs_; }
    [[nodiscard]] std::span<const SignalSpec> outputs() const { return outputs_; }
    /// Outputs that modules added before this one have declared, whose names start with
    /// `prefix`. For collectors such as WrenchSum, which sum what others write: add a
    /// collector after the modules it collects from. Copies: declaring more signals moves the
    /// bus's table.
    [[nodiscard]] std::vector<SignalSpec> declaredOutputs(std::string_view prefix) const;

private:
    friend class Scheduler;
    SignalDeclarer(SignalBus& bus, std::uint32_t module) : bus_(&bus), module_(module) {}

    SignalBus* bus_;
    std::uint32_t module_;
    std::vector<SignalSpec> inputs_;
    std::vector<SignalSpec> outputs_;
    std::vector<std::uint32_t> inputSlots_;
    std::vector<std::uint32_t> outputSignals_;
    std::vector<std::string> problems_;  // invalid declarations, reported by wire()
};

/// A simulation module: one domain, a backend of it, or a substitute (RFC 0001).
///
/// Lifecycle: declare → (scheduler wires and checks) → init → step × N → shutdown.
/// Modules communicate only through the bus (invariant 1) and never throw.
class IModule {
public:
    IModule() = default;
    IModule(const IModule&) = delete;
    IModule& operator=(const IModule&) = delete;
    IModule(IModule&&) = delete;
    IModule& operator=(IModule&&) = delete;
    virtual ~IModule() = default;

    [[nodiscard]] virtual ModuleInfo info() const = 0;
    /// 100 µs electrical, 1 ms physics, ...; the profile's `dt` overrides it.
    [[nodiscard]] virtual Duration preferredStep() const = 0;

    /// 1. Inspect the world and config; declare every input and output. No large allocations.
    virtual Status declare(const WorldView& world, const ModuleConfig& config,
                           SignalDeclarer& declarer) = 0;
    /// 2. Units are checked and ids valid. Allocate state and write initial output values.
    virtual Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) = 0;
    /// 3. Hot path: no allocation, no USD, no blocking I/O, no string lookups.
    virtual Status step(const StepContext& context, SignalBus& bus) = 0;
    virtual void shutdown() {}

    /// Complete dynamic state as bytes; required when info().deterministic is true.
    virtual Status saveState(StateWriter& /*writer*/) const {
        return Status::error(StatusCode::Unsupported, "saveState not implemented");
    }
    virtual Status restoreState(StateReader& /*reader*/) {
        return Status::error(StatusCode::Unsupported, "restoreState not implemented");
    }
};

}  // namespace lagrange
