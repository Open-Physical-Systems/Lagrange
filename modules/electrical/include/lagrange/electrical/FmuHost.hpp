#pragma once

#include "lagrange/module/IModule.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace lagrange::electrical {

/// What an FMU stands in for: its file, and how the names in its map file are filled in.
struct FmuBinding {
    std::string fmuPath;  // the .fmu; its map is next to it, "<fmu>.map.yaml"
    std::string name;     // the component, e.g. "left_drive_motor": the module's and instance's
    /// Values of the map's placeholders: "joint", "bus", "prim" for a motor.
    std::map<std::string, std::string> placeholders;
    /// The component's schema attributes (doubles), for the map's parameters.
    std::map<std::string, double, std::less<>> attributes;
};

/// One FMI 3.0 co-simulation FMU on the bus (docs/design/formats.md §6), through FMI Library 3.
///
/// The map file names the FMU's inputs and outputs as signals (with {placeholders}) and its
/// parameters as schema attributes. Each mapped variable must be a Float64 of the right
/// causality whose unit (from the BaseUnit exponents in modelDescription.xml, SI only) has the
/// dimension of the map's unit; anything else fails at declare, naming the variable. An input
/// with `optional: <value>` reads that value while its signal is not written. The module steps
/// at the map's `step`, one fmi3DoStep per step.
///
/// It is deterministic (and saves its state) only when the FMU declares canGetAndSetFMUState
/// and canSerializeFMUState. The FMU is unpacked into a temporary directory, removed with the
/// module.
class FmuComponent final : public IModule {
public:
    explicit FmuComponent(FmuBinding binding);
    ~FmuComponent() override;
    FmuComponent(const FmuComponent&) = delete;
    FmuComponent& operator=(const FmuComponent&) = delete;
    FmuComponent(FmuComponent&&) = delete;
    FmuComponent& operator=(FmuComponent&&) = delete;

    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    void shutdown() override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

    /// The signals it reads and writes, after declare.
    [[nodiscard]] std::vector<std::string> inputNames() const;
    [[nodiscard]] std::vector<std::string> outputNames() const;

private:
    struct Impl;
    /// Reads the map and loads the FMU once (info() needs its capabilities before declare).
    Impl& load() const;

    FmuBinding binding_;
    std::string moduleName_;  // "fmu_<name>"
    mutable std::unique_ptr<Impl> impl_;
};

/// The map file of an FMU: "<fmu>.map.yaml".
[[nodiscard]] std::string fmuMapPath(const std::string& fmuPath);

}  // namespace lagrange::electrical
