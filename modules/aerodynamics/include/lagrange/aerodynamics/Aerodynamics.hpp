#pragma once

#include "lagrange/module/Expected.hpp"
#include "lagrange/module/IModule.hpp"

#include <array>
#include <string>
#include <vector>

namespace lagrange::aerodynamics {

/// The site's wind until the weather domain gives it (v0.5): mean speed at a reference height,
/// the direction it blows from, the terrain's roughness length for the logarithmic profile, and
/// whether Dryden turbulence is added. All site parameters, without defaults.
struct Wind {
    double speed{0.0};            // m/s at referenceHeight
    double fromDegrees{0.0};      // meteorological: 0 = from the north, 90 = from the east
    double referenceHeight{0.0};  // m above the ground
    double roughnessLength{0.0};  // m (z0)
    bool dryden{false};
};

/// Mean wind speed at `height` (m) from the logarithmic profile U(h) = U_ref·ln(h/z0)/ln(h_ref/z0)
/// (neutral surface layer); 0 below the roughness length.
[[nodiscard]] double meanWindAt(const Wind& wind, double height);

/// Dryden turbulence for one point moving through the air (MIL-F-8785C, §3.7.3, low altitude;
/// the same filters as MIL-HDBK-1797 once its scale lengths are substituted; TODO(verify) the
/// handbook's equation numbers against the document). Longitudinal: σu·√(2Lu/(πV))/(1+(Lu/V)s);
/// lateral and vertical: σ·√(L/(πV))·(1+√3(L/V)s)/(1+(L/V)s)². Below 1000 ft: Lw = h,
/// Lu = Lv = h/(0.177+0.000823h)^1.2, σw = 0.1·W20, σu = σv = σw/(0.177+0.000823h)^0.4 (h in
/// feet, W20 the mean wind at 20 ft). Each filter is integrated exactly over the step for white
/// noise held over it.
class Dryden {
public:
    /// Advances by `dt` (s) at height `height` (m), airspeed `airspeed` (m/s) and W20 (m/s), with
    /// five standard normal samples; returns the gust (u along the mean wind, v to its left, w
    /// up), m/s.
    std::array<double, 3> step(double dt, double height, double airspeed, double w20,
                               const std::array<double, 5>& normal);
    void reset() { state_ = {}; }
    [[nodiscard]] const std::array<double, 5>& state() const { return state_; }
    void setState(const std::array<double, 5>& s) { state_ = s; }

private:
    std::array<double, 5> state_{};  // u: 1, v: 2, w: 2
};

/// The aerodynamics domain at the `drag` fidelity (docs/design/domains/aerodynamics.md): for
/// every link with AeroAPI_1, the drag F = −½·ρ·|v_rel|·(Cd ∘ A ∘ v_rel) in the body frame at the
/// centre of pressure and angular damping, against the air: the mean wind (Wind, logarithmic
/// profile) plus Dryden turbulence at the link. Writes body/<b>/wrench/aero (for WrenchSum),
/// body/<b>/airspeed and env/wind/gust (the turbulence at the first link). The air density is
/// the config's "air_density", as rotor_aero. Not yet: lift tables, wind fields from VDB.
class AeroDrag final : public IModule {
public:
    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

    /// Reads the Wind of a module config ("wind_speed", "wind_from", "wind_reference_height",
    /// "roughness_length", "turbulence"); no wind (speed 0) when "wind_speed" is absent.
    [[nodiscard]] static Expected<Wind, std::string> windOf(const ModuleConfig& config);

private:
    struct Link {
        std::string name;
        std::array<double, 3> area{}, cd{}, centre{}, damping{};
        SignalId pose, twist, wrench, airspeed;
        Dryden turbulence;
    };

    double rho_{0.0};
    Wind wind_;
    std::array<double, 3> groundPoint_{};
    std::vector<Link> links_;
    SignalId gust_;
};

}  // namespace lagrange::aerodynamics
