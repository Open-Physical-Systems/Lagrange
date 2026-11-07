#pragma once

#include "lagrange/aerodynamics/Downwash.hpp"
#include "lagrange/aerodynamics/LiveAir.hpp"
#include "lagrange/fields/Field.hpp"
#include "lagrange/module/Expected.hpp"
#include "lagrange/module/IModule.hpp"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace lagrange::fields {
class Field;
}

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

/// The fountain under a low multirotor (rotor_aero.md §2.8): near the ground its rotor jets meet
/// under it and turn up, pushing its frame up, against the download it feels higher up. The
/// change of the force on the frame from what it is 2 m up, as a share of the multirotor's
/// thrust, at a height h of the rotor plane over the surface its wake meets, in motor-to-motor
/// diagonals l: F·(exp(−(h/l)/λ) − exp(−R/λ)) below R = kFountainReach diagonals, none beyond,
/// and held at its value at kFountainLowest below that (the lowest case run; no extrapolation).
/// F and λ are a least-squares fit of the logarithm to `lagrange foam-hover` runs of the sample
/// quadcopter at 0.5, 0.75, 1, 1.5 and 2 m (rotor_aero.md §7): 12.7, 7.4, 2.7 and 1.0 % of its
/// thrust at h/l = 1, 1.5, 2 and 3, the fit within 0.23 N (1.2 % of the weight) of each. Our own
/// CFD only, of one frame, no measurement; 2 m stands in for "high up", and the download there
/// (9 % of the weight) is not in the model, the rotor data being of isolated rotors.
inline constexpr double kFountainGain = 0.4634;    // F, share of thrust
inline constexpr double kFountainLength = 0.7623;  // λ, diagonals
inline constexpr double kFountainReach = 4.0;      // R, diagonals: 2 m for the sample
inline constexpr double kFountainLowest = 1.0;     // diagonals: 0.5 m for the sample
[[nodiscard]] double fountainLift(double heightOverDiagonal);

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

/// Rain and snow on a body (aerodynamics.md §3.2). The falling water is a cloud of drops with
/// a mass concentration W = ρ_w·R/v̄ (R the rate as a depth of water per second, v̄ the drops'
/// fall speed), moving with the wind and down at v̄. Each drop that meets the body stops on it
/// (an inelastic impact, as Haines and Luers, J. Aircraft 20(2), 1983, for rain on aircraft), so
/// the body takes the momentum flux F = W·|u|·A_⊥(û)·u, u the drops' velocity relative to the
/// body and A_⊥ its area across u, here the box Σ A_k·|û_k| of the link's reference areas.
/// Rain's v̄ is the mass-weighted fall speed of the Marshall and Palmer drop sizes
/// (J. Meteor. 5, 1948: N(D) = N0·exp(−Λ·D), N0 = 8000 m⁻³·mm⁻¹, Λ = 4.1·R^−0.21 mm⁻¹, R in
/// mm/h) at the speed of Atlas, Srivastava and Sekhon (Rev. Geophys. 11(1), 1973: v(D) = 9.65 −
/// 10.3·exp(−0.6·D) m/s, D in mm, their fit to Gunn and Kinzer 1949): in closed form v̄ = 9.65 −
/// 10.3·(Λ/(Λ + 0.6))⁴, held at no less than kLeastFallSpeed for drizzle. Snow falls at
/// kSnowFallSpeed. A drone feels little of it: 25 mm/h on the sample X500's 0.09 m² top is 4 mN.
inline constexpr double kWaterDensity = 1000.0;  // kg/m³
/// m/s: unrimed snow aggregates fall at about 1 m/s, nearly whatever their size (Locatelli and
/// Hobbs, J. Geophys. Res. 79(15), 1974; TODO(verify) their fitted coefficients).
inline constexpr double kSnowFallSpeed = 1.0;
inline constexpr double kLeastFallSpeed = 0.5;  // m/s, the slowest drizzle drop counted
/// Rain's mass-weighted fall speed, m/s, at a rate in m/s (of water depth).
[[nodiscard]] double rainFallSpeed(double rainRate);
/// Water in the air, kg/m³, of rain falling at `rainRate` and snow at `snowRate` (m/s of water
/// depth), as {rain, snow}.
[[nodiscard]] std::array<double, 2> fallingWater(double rainRate, double snowRate);
/// The force on a box of reference areas `area` (m², along the body's x, y, z) from water of
/// concentration `water` (kg/m³) moving at `relative` (m/s, the drops' velocity less the body's,
/// in the body frame), N in the body frame.
[[nodiscard]] std::array<double, 3> impactForce(double water, const std::array<double, 3>& relative,
                                                const std::array<double, 3>& area);

/// The aerodynamics domain at the `drag` fidelity (docs/design/domains/aerodynamics.md): for
/// every link with AeroAPI_1, the drag F = −½·ρ·|v_rel|·(Cd ∘ A ∘ v_rel) in the body frame at the
/// centre of pressure and angular damping, against the air: the mean wind (Wind, logarithmic
/// profile) plus Dryden turbulence at the link. Writes body/<b>/wrench/aero (for WrenchSum),
/// body/<b>/airspeed, env/wind/gust (the turbulence at the first link) and env/wind/velocity
/// (the air's velocity there, mean wind and gust, world frame: what the rotors fly in, at the
/// first link's height). The air density is
/// the config's "air_density", as rotor_aero; with config "weather_air" (the weather runs) it is
/// the weather's env/air_density, from the site's temperature, pressure and humidity, and the
/// weather's env/rain_rate and env/snow_rate push every link (impactForce). Not yet: lift tables.
///
/// Downwash (config "downwash", default true; rotor_aero.md §2): for every RotorAPI_1 joint it
/// reads rotor/<j>/thrust, rotor/<j>/induced_velocity, rotor/<j>/thrust_axis and its link's pose
/// and twist, builds the rotors' wakes (Downwash, one group per robot) and adds them to the air
/// at every link of another robot, and writes rotor/<j>/air_velocity (m/s, world: the wind and
/// gust plus other robots' wakes at the hub) for the rotors. With the site's loose ground dust
/// (config "ground_dust_diameter", m, and "ground_dust_concentration", kg/m³: the most the
/// outwash keeps in the air there; site parameters, no defaults) it writes env/dust/downwash
/// (kg/m³) once the outwash's friction velocity passes Bagnold's threshold for those grains.
///
/// Config "downwash_model": "fitted" (the default, above) or "lbm" (rotor_aero.md §8): the first
/// multirotor's air is computed live in a lattice Boltzmann box that follows it (LiveAir, on a
/// hardware GPU; config "downwash_lbm_cell", m, default 0.04; "downwash_lbm_cpu": the CPU
/// reference). Its rotors then fly in the box's air less their own (walls, ground, their other
/// rotors), links of other robots in the box feel it, and the box goes out as aero/air_box
/// (Field3D, every second cell). Outside the box, and for every other robot, the fitted model
/// stays. Without a hardware GPU the fitted model runs instead.
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
    SignalId densityIn_, rainIn_, snowIn_;  // the weather's, when it runs
    bool downwashOn_{true};
    std::vector<RotorLink> rotors_;
    std::size_t groupCount_{0};
    Downwash downwash_;
    std::array<double, 3> groundNormal_{0.0, 0.0, 1.0};
    bool hasGround_{false};
    double gravity_{9.80665};
    double dustDiameter_{0.0}, dustMost_{0.0};
    SignalId dustOut_;
    // The live air (downwash_model: lbm).
    std::unique_ptr<LiveAir> live_;
    std::size_t liveGroup_{Downwash::kNoGroup};  // the multirotor in the box
    Downwash others_;                            // every other multirotor's fitted wake
    SignalId airBox_;
    std::shared_ptr<const fields::Field> published_;
    bool lbmWanted_{false};
    double lbmCell_{0.04};
    bool lbmCpu_{false};
};

}  // namespace lagrange::aerodynamics
