#pragma once

// What the rotors' downwash does on water (docs/physics/downwash.md §5a; water.md "The surface
// under a rotor"): the load it puts on each body of water's surface, sampled on the grid of the
// patch the water domain moves, for the aerodynamics domain to hand over through the bus.

#include "lagrange/aerodynamics/Downwash.hpp"
#include "lagrange/module/IModule.hpp"

#include <array>
#include <functional>
#include <span>
#include <string>
#include <vector>

namespace lagrange::aerodynamics {

/// The patch grid both domains share (the server gives both the same config).
struct WaterGrid {
    int cells{128};     // a side
    double cell{0.03};  // m
    double step{0.04};  // s, how often the load is sampled
};

/// The patch corner (m, world x and y) a grid of `grid` puts round `centre`: at whole multiples
/// of the cell, so the two domains' grids agree.
[[nodiscard]] std::array<double, 2> patchCorner(const WaterGrid& grid,
                                                const std::array<double, 2>& centre);

// --- Spray (downwash.md §5b) ---------------------------------------------------------------

/// The air speed over still water above which its surface is unstable (Kelvin–Helmholtz with
/// gravity and surface tension; Lamb, Hydrodynamics, 6th ed., 1932, §268; Chandrasekhar,
/// Hydrodynamic and Hydromagnetic Stability, 1961, §101):
/// U² = 2·(ρ_a + ρ_w)/(ρ_a·ρ_w) · √(g·σ·(ρ_w − ρ_a)); about 6.6 m/s for air over water at 20 °C.
/// Below it no wave grows from the shear and no spray is torn off; the model takes it as the
/// outwash's onset of spray, compared with the wall jet's peak speed u_m (the air's speed against
/// still water across a thin layer). TODO(verify): a measured onset of rotor-wash spray.
[[nodiscard]] double kelvinHelmholtzOnset(double rhoAir, double rhoWater, double tension,
                                          double gravity);
/// Water torn off the surface, kg/(m²·s), where the outwash's peak u_m passes the onset:
/// C·ρ_a·u*·(1 − (U_KH/u_m)²), u* its friction velocity; 0 below. Behavioural (as the dust's
/// lifting), C = kSprayEntrainment chosen so that a drone 0.5 m over water stands in spray of
/// about a gram per cubic metre, a heavy fog's water; uncalibrated, TODO(verify) against a
/// measured rotor-wash spray rate.
inline constexpr double kSprayEntrainment = 0.01;
[[nodiscard]] double sprayEmission(double peakOutwash, double friction, double rhoAir,
                                   double onset);
/// The drops' diameters (m), the torn-off water shared equally between them: spume drops from a
/// shear-torn surface are tens to hundreds of micrometres (TODO(verify) a measured rotor-wash
/// spray size distribution).
inline constexpr std::array<double, 3> kSprayDiameters{50e-6, 150e-6, 450e-6};
/// Air's dynamic viscosity at 20 °C (TODO(verify): 1.81e-5 Pa·s, the usual table value).
inline constexpr double kAirViscosity = 1.81e-5;
/// A drop's velocity relaxation time (s) at slip speed `slip` (m/s): Stokes's ρ_w·D²/(18·μ) over
/// Schiller and Naumann's drag correction 1 + 0.15·Re^0.687 (Re = ρ_a·slip·D/μ < 800; Clift,
/// Grace and Weber, Bubbles, Drops and Particles, 1978, table 5.2).
[[nodiscard]] double dropRelaxation(double diameter, double slip, double rhoAir,
                                    double rhoWater = 1000.0, double viscosity = kAirViscosity);
/// Its terminal fall speed in still air (m/s), g·τ solved with the slip-dependent τ.
[[nodiscard]] double dropTerminalSpeed(double diameter, double rhoAir, double gravity,
                                       double rhoWater = 1000.0, double viscosity = kAirViscosity);
/// Light extinction of drops much larger than the wavelength (1/m), Q_ext = 2 (van de Hulst,
/// Light Scattering by Small Particles, 1957, §8.42): Σ 3·W_i/(ρ_w·D_i) over the sizes' water
/// contents W_i (kg/m³); the same at visible and near-infrared wavelengths.
[[nodiscard]] double sprayExtinction(std::span<const double> water, double rhoWater = 1000.0);
/// Turbulent dispersion of the drops (behavioural): a Langevin velocity of standard deviation
/// kSprayTurbulence·|u_air| and time scale kSprayLagrangianTime, the wall jet's turbulence
/// intensity order (TODO(verify) its measured value). The drops are launched up at that speed.
inline constexpr double kSprayTurbulence = 0.25;
inline constexpr double kSprayLagrangianTime = 0.05;  // s
/// How far round a point the drops count for its water content (m): a sphere round each point.
inline constexpr double kSprayReach = 0.25;
/// How far round the first multirotor its spray counts for the sensors' extinction (m): what a
/// camera or lidar on it looks through near it, taken as uniform (as the weather's extinction).
inline constexpr double kSprayViewReach = 2.5;
inline constexpr std::size_t kSprayMaxDrops = 2048;
inline constexpr std::size_t kSprayDropsPerStep = 96;

/// Per body of water w, writes water/<w>/patch (m, 2: the lower corner of the patch, following
/// the first wake that meets this water: moved when the wake's centre leaves the patch's middle
/// half; NaN until a wake comes), water/<w>/air_pressure (Pa, per cell: Downwash::surfacePressure)
/// and water/<w>/air_stress (Pa, x for every cell then y: ρ·|u*|·u*, Downwash::surfaceFriction),
/// every grid.step seconds.
///
/// With spray on (downwash.md §5b), where the outwash passes the Kelvin–Helmholtz onset it tears
/// drops off (sprayEmission, the sizes kSprayDiameters), carried as parcels by the air (`air`:
/// wind and every wake) with their drag (dropRelaxation), gravity and turbulent dispersion, until
/// they fall back under the surface. It writes water/<w>/outwash_peak (m/s, the largest u_m on
/// it) and water/<w>/spray_rate (kg/s torn off), rotor/<j>/spray (kg/m³ round each hub),
/// env/spray/downwash (kg/m³ within kSprayReach of the first multirotor), env/spray/extinction
/// (1/m, of the spray within kSprayViewReach of it) and env/spray/rate (m/s of water, W·|v|/ρ_w
/// within kSprayReach, the drops' flux onto it as a rain rate), and
/// aero/spray_drops (m, x, y, z per drop, NaN past the last) and aero/spray_mass (kg of water per
/// drop, 0 past the last) for the viewer.
class WaterAir {
public:
    /// `rotors`: the rotor joints' names, in the order step() gets their hubs.
    Status declare(const std::vector<Downwash::WaterSurface>& water, const WaterGrid& grid,
                   bool spray, const std::vector<std::string>& rotors, double gravity,
                   SignalDeclarer& d);
    void init(SignalBus& bus);
    using AirAt = std::function<Downwash::Vec3(const Downwash::Vec3&)>;
    /// At sim time `t` (s): resamples and moves the spray when the grid's step has passed.
    /// `hubs` the rotors' hubs (m, world), `air` the air's velocity at a point.
    void step(const StepContext& context, const Downwash& downwash, double rhoAir,
              std::span<const Downwash::Vec3> hubs, const AirAt& air, SignalBus& bus);
    [[nodiscard]] bool empty() const { return pools_.empty(); }
    /// The spray's water content round `x` (kg/m³), and the drops' mean velocity there.
    [[nodiscard]] double sprayAt(const Downwash::Vec3& x, Downwash::Vec3* velocity = nullptr,
                                 double reach = kSprayReach) const;
    /// The same per drop size (kg/m³ each).
    [[nodiscard]] std::array<double, 3> sprayBySize(const Downwash::Vec3& x,
                                                    double reach = kSprayReach) const;
    [[nodiscard]] std::size_t drops() const { return drops_.size(); }
    struct Drop {
        Downwash::Vec3 at{}, velocity{}, gust{};
        double mass{0.0};  // kg of water the parcel carries
        int size{0};       // into kSprayDiameters
        int water{0};      // the body of water it fell from
    };
    [[nodiscard]] const std::vector<Drop>& dropList() const { return drops_; }
    void saveState(StateWriter& writer) const;
    bool restoreState(StateReader& reader);

private:
    struct Pool {
        Downwash::WaterSurface water;
        std::array<double, 2> corner{};
        bool placed{false};
        bool loaded{false};  // the last sample had a load on it
        SignalId patchOut, pressureOut, stressOut, peakOut, rateOut;
        std::vector<double> pressure, stress;
        std::vector<double> torn;  // kg per coarse cell, waiting to be launched
    };
    void moveDrops(const StepContext& context, double dt, double rhoAir, const AirAt& air);
    std::vector<Pool> pools_;
    WaterGrid grid_;
    double next_{0.0};
    double last_{0.0};
    bool spray_{false};
    double gravity_{9.80665};
    std::vector<Drop> drops_;
    std::vector<SignalId> rotorSpray_;
    SignalId sprayOut_, extinctionOut_, rateOut_, dropsOut_, massOut_;
    std::vector<double> dropValues_, massValues_;
};

}  // namespace lagrange::aerodynamics
