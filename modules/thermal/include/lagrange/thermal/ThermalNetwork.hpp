#pragma once

#include "lagrange/module/IModule.hpp"
#include "lagrange/thermal/Lumped.hpp"

#include <string>
#include <vector>

namespace lagrange::thermal {

/// The ambient air temperature, K, from the module config ("ambient_temperature": the profile's
/// thermal.ambient_temperature or the server's --ambient-temperature). A site parameter: there is
/// no default, and the thermal modules refuse to start without it (CLAUDE.md §10).
inline constexpr const char* kAmbientKey = "ambient_temperature";

/// The thermal domain at the `lumped` fidelity (docs/design/domains/thermal.md): one node per
/// ThermalAPI_1 prim, ThermalLinkAPI_1 conductances between them (LumpedNetwork).
///
/// Heat sources: every `<class>/<node>/loss_power` signal declared before it on the node's prim
/// (motor/<m>/loss_power, battery/<b>/loss_power from the electrical network), averaged over the
/// thermal step. Surroundings: env/air_temperature and env/sky_temperature (optional; the
/// ambient temperature until a weather module writes them) and env/solar_irradiance (optional,
/// 0). Outputs per node: thermal/<p>/temperature, /derating (read back by the electrical
/// network), /overtemp and /heat_flow_out. Not yet: airspeed (forced convection, with the
/// aerodynamics domain) and the sun's direction. Default step 100 ms.
class ThermalNetwork final : public IModule {
public:
    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

    /// Node names, as in the signal names, after declare.
    [[nodiscard]] const std::vector<std::string>& nodes() const { return names_; }
    /// The heat-source signals each node reads, after declare.
    [[nodiscard]] const std::vector<std::vector<std::string>>& sources() const {
        return sourceNames_;
    }

private:
    struct Limits {
        double deratingStart{0.0}, maxTemperature{0.0};  // K; equal: none
        bool hasMax{false};
    };
    struct Outputs {
        SignalId temperature, derating, overtemp, heatOut;
    };
    struct Source {
        std::size_t node{0};
        SignalId signal;
    };

    LumpedNetwork network_;
    std::vector<std::string> names_;
    std::vector<std::vector<std::string>> sourceNames_;
    std::vector<double> initial_;
    std::vector<Limits> limits_;
    std::vector<Outputs> outputs_;
    std::vector<Source> sources_;
    SignalId air_, sky_, irradiance_;
    std::vector<double> temperatures_, heat_, heatOut_;
};

/// The thermal domain's `ambient` substitute (constant): every ThermalAPI_1 node at the
/// ambient temperature, derating 1, never over temperature.
class AmbientTemperature final : public IModule {
public:
    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

private:
    double ambient_{0.0};
    std::vector<SignalId> temperature_, derating_, overtemp_;
};

}  // namespace lagrange::thermal
