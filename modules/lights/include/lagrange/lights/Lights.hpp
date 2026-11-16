#pragma once

// The lights domain (RFC 0021 §3): every LampAPI_1 prim is a lamp, stepped every 10 ms. Per lamp
// <n> it reads lamp/<n>/command (0–1, held between writes; its initialCommand until the first)
// and lamp/<n>/blink (above 0.5: blink), and for a lamp that is a load load/<n>/powered and its
// bus's voltage; it writes lamp/<n>/intensity (cd, on the axis), lamp/<n>/on, lamp/<n>/level
// (the output as a share of full), load/<n>/power_demand (W, to the electrical network) and
// lamp/<n>/loss_power (W, to a thermal node named <n>).
//
//   c      = clamp(command, 0, 1) · blink(t) · powered
//   I_out += (c · I_v · g(V) − I_out) · (1 − e^(−dt/riseTime))
//   g(V)   = (V/V₀)^n below V₀ (the bus voltage at the start), else 1
//   P_elec = I_v · Ω_eff · c / η_v;  P_heat = heatFraction · P_elec
//
// Substitutes: `zero` (every lamp dark, no power, no heat) and `constant` (every lamp at its
// initialCommand, commands and blink ignored).

#include "lagrange/lights/Lamp.hpp"
#include "lagrange/module/IModule.hpp"

#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

namespace lagrange::lights {

enum class Mode : std::uint8_t { Model, Zero, Constant };

class Lights final : public IModule {
public:
    explicit Lights(Mode mode = Mode::Model) : mode_(mode) {}

    /// The substitute a profile's domains.lights.substitute names: zero or constant.
    [[nodiscard]] static std::optional<Mode> substitute(std::string_view token);

    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

    [[nodiscard]] std::span<const LampSpec> lamps() const { return specs_; }

private:
    struct Io {
        SignalId command{}, blink{}, powered{}, voltage{};
        SignalId intensity{}, on{}, level{}, demand{}, loss{};
        bool hasVoltage{false};
    };
    void write(std::size_t k, double output, double command, SignalBus& bus) const;

    Mode mode_;
    std::vector<LampSpec> specs_;
    std::vector<double> solidAngle_;  // sr, per lamp
    std::vector<Io> io_;
    // Dynamic state: the light output (cd) and the nominal voltage seen first (V; 0 until then).
    std::vector<double> output_;
    std::vector<double> nominal_;
};

}  // namespace lagrange::lights
