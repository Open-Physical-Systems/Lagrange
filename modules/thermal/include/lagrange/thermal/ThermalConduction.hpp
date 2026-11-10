#pragma once

#include "lagrange/module/IModule.hpp"
#include "lagrange/thermal/Conduction.hpp"

#include <string>
#include <vector>

namespace lagrange::thermal {

/// Profile keys of the conduction fidelity (thermal.cell, thermal.max_cells,
/// thermal.surface_coefficient).
inline constexpr const char* kCellKey = "cell";                               // m, default 0.002
inline constexpr const char* kMaxCellsKey = "max_cells";                      // per part, 200000
inline constexpr const char* kSurfaceCoefficientKey = "surface_coefficient";  // W/(m²·K)

/// The thermal domain at the `conduction` fidelity (RFC 0012, docs/physics/heat-conduction.md):
/// every link with ThermalMaterialAPI_1 cut into cells (ConductionModel), heated by its
/// HeatSourceAPI_1 boxes and the loss_power signals on its prim, cooled by convection (natural,
/// and forced by body/<b>/airspeed when it is on the bus) and radiation. It runs beside the
/// lumped network, which keeps its ThermalAPI_1 nodes.
///
/// Signals per link L: thermal/L/temperature (volume mean), /max_temperature,
/// /min_temperature, /heat_flow_out, /overtemp; per source S: thermal/S/temperature (mean over
/// its box), /heat ((1 − η)·P), /radiant_power (η·P). Inputs: env/air_temperature,
/// env/sky_temperature, env/solar_irradiance (optional), body/<b>/pose (feedback, moving
/// bodies), body/<b>/airspeed (optional, feedback). Default step 100 ms.
class ThermalConduction final : public IModule {
public:
    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

    /// The cells, for the viewer (after declare) and their temperatures (after a step).
    [[nodiscard]] const ConductionModel& model() const { return model_; }
    [[nodiscard]] const std::vector<double>& temperatures() const { return temperatures_; }
    /// What the build noticed (coarser cells, touching parts with no contact conductance).
    [[nodiscard]] const std::vector<std::string>& notes() const { return notes_; }
    /// The body of each part, by index in model().parts().
    [[nodiscard]] const std::vector<BodyId>& bodies() const { return bodies_; }

private:
    struct SourceIo {
        std::size_t part{0};
        double power{0.0}, efficiency{0.0};
        SignalId powerSignal;  // invalid: the constant power
        SignalId temperature, heat, radiant;
    };
    struct PartIo {
        SignalId pose, airspeed;  // invalid when the body does not move / no airspeed
        std::vector<SignalId> losses;
        SignalId temperature, maxTemperature, minTemperature, heatOut, overtemp;
        std::optional<double> maxAllowed;
    };

    ConductionModel model_;
    std::vector<double> temperatures_;
    std::vector<std::string> notes_;
    std::vector<BodyId> bodies_;
    std::vector<Pose> rest_;
    std::vector<PartIo> parts_;
    std::vector<SourceIo> sources_;
    std::vector<PartInput> inputs_;
    std::vector<double> sourceHeat_;
    std::optional<double> fixedCoefficient_;
    SignalId air_, sky_, irradiance_;
};

}  // namespace lagrange::thermal
