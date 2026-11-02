#pragma once

#include "lagrange/aerodynamics/Downwash.hpp"
#include "lagrange/fields/Field.hpp"
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

/// A rotor of the stage, for the downwash (rotor_aero.md §2): its RotorAPI_1 joint and link (as
/// in signal names), radius, and group, its robot (the root body above it), numbered in the
/// order robots' first rotors come.
struct RotorSource {
    std::string joint, link;
    double radius{0.0};  // m
    std::size_t group{0};
};
[[nodiscard]] std::vector<RotorSource> rotorSources(const WorldView& world);
/// Every body's rotor group (Downwash::kNoGroup when its robot has no rotors), as rotorSources.
[[nodiscard]] std::vector<std::size_t> rotorGroupOfBodies(const WorldView& world);
/// The fixed boxes of the scenery (static Box colliders: buildings, walls), for the downwash.
[[nodiscard]] std::vector<Downwash::Box> downwashObstacles(const World& world);

/// A wall beside a multirotor (rotor_aero.md §2.7): it starves the inflow on its side (Herz and
/// McCrink, VFS Forum 81, 2025), so air goes down faster through the rotors next to it and
/// slower through those away from it. Rotors at a fixed speed then make less thrust on the
/// wall's side, and the drone rolls towards the wall (Tanabe et al., J. Robot. Mechatron. 30(3),
/// 2018). Rotor i's extra downward air, as a share of its induced velocity, is
/// A·exp(−(g/D)/λ)·s_i: g the gap between the nearest rotor tip and the wall, D the rotor
/// diameter, s_i from +1 (the hub farthest towards the wall) to −1 (farthest away). A and λ are
/// fitted to `lagrange foam-hover` runs of the sample quadcopter 2 m up beside a 12 m building at
/// tip gaps of 0.25, 0.5, 1 and 2 D (rotor_aero.md §7): the near rotors' less the far rotors'
/// inflow, against open air, was 3.4% of v_i at the wall, fading over 3.5 ± 2.7 D. The fading is
/// poorly resolved by those runs (mesh noise about ±0.05 m/s, 0.8% of v_i); Tanabe et al. find
/// the effect gone beyond 1.5 D.
inline constexpr double kWallInflowGain = 0.0173;  // A: 0.0346 / 2 (near +A, far −A)
inline constexpr double kWallInflowLength = 3.5;   // λ, rotor diameters
inline constexpr double kWallReach = 10.0;         // diameters: beyond, nothing
[[nodiscard]] double wallInflow(double gapOverDiameter);

/// Dust lifted by the outwash, kg/m³: `most`·(1 − (u*_t/u*)²) above the threshold, 0 below
/// (behavioural, a range check). u* = kWallJetFriction·u_peak, u*_t = A·√((ρ_p − ρ)/ρ·g·d) with
/// A = kBagnoldA (Bagnold, The Physics of Blown Sand and Desert Dunes, 1941).
inline constexpr double kBagnoldA = 0.1;
inline constexpr double kGrainDensity = 2650.0;  // kg/m³, quartz
/// u*/u_m of a wall jet: √(c_f/2) with c_f ≈ 0.005 (TODO(verify): the radial wall jet's skin
/// friction, e.g. Poreh et al. 1967).
inline constexpr double kWallJetFriction = 0.05;
[[nodiscard]] double dustThreshold(double diameter, double rho, double gravity);
[[nodiscard]] double liftedDust(double peakOutwash, double diameter, double most, double rho,
                                double gravity);

/// The aerodynamics domain at the `drag` fidelity (docs/design/domains/aerodynamics.md): for
/// every link with AeroAPI_1, the drag F = −½·ρ·|v_rel|·(Cd ∘ A ∘ v_rel) in the body frame at the
/// centre of pressure and angular damping, against the air: the mean wind (Wind, logarithmic
/// profile) plus Dryden turbulence at the link. Writes body/<b>/wrench/aero (for WrenchSum),
/// body/<b>/airspeed, env/wind/gust (the turbulence at the first link) and env/wind/velocity
/// (the air's velocity there, mean wind and gust, world frame: what the rotors fly in, at the
/// first link's height). The air density is
/// the config's "air_density", as rotor_aero. Not yet: lift tables.
///
/// Downwash (config "downwash", default true; rotor_aero.md §2): for every RotorAPI_1 joint it
/// reads rotor/<j>/thrust, rotor/<j>/induced_velocity, rotor/<j>/thrust_axis and its link's pose
/// and twist, builds the rotors' wakes (Downwash, one group per robot) and adds them to the air
/// at every link of another robot, and writes rotor/<j>/air_velocity (m/s, world: the wind and
/// gust plus other robots' wakes at the hub) for the rotors. With the site's loose ground dust
/// (config "ground_dust_diameter", m, and "ground_dust_concentration", kg/m³: the most the
/// outwash keeps in the air there; site parameters, no defaults) it writes env/dust/downwash
/// (kg/m³) once the outwash's friction velocity passes Bagnold's threshold for those grains.
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
        std::size_t group{Downwash::kNoGroup};  // its robot, when that robot has rotors
    };
    struct RotorLink {
        double radius{0.0};
        std::size_t group{0};
        SignalId thrust, induced, axis, pose, twist, air;
    };

    double rho_{0.0};
    Wind wind_;
    std::array<double, 3> groundPoint_{};
    std::vector<Link> links_;
    SignalId gust_;
    SignalId windField_;  // the weather's wind grid, when it runs
    fields::FieldSignal windFieldSignal_;
    SignalId windVelocity_;
    bool downwashOn_{true};
    std::vector<RotorLink> rotors_;
    std::size_t groupCount_{0};
    Downwash downwash_;
    std::array<double, 3> groundNormal_{0.0, 0.0, 1.0};
    bool hasGround_{false};
    double gravity_{9.80665};
    double dustDiameter_{0.0}, dustMost_{0.0};
    SignalId dustOut_;
};

}  // namespace lagrange::aerodynamics
