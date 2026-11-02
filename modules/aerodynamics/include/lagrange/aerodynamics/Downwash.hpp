#pragma once

#include <array>
#include <cstddef>
#include <limits>
#include <span>
#include <vector>

namespace lagrange::aerodynamics {

struct Wind;

/// Rotor downwash and ground outwash (docs/design/domains/rotor_aero.md §2): the air the rotors
/// push, as a velocity field the aerodynamics domain adds to the wind.
///
/// Three regions, all from momentum-theory quantities the rotor domain already computes (the
/// induced velocity v_i at the disc and the thrust direction):
///
/// 1. Near wake of each rotor, the vortex-cylinder (actuator-disc) solution on the axis
///    (Leishman, Principles of Helicopter Aerodynamics, 2nd ed., 2006, §2.7):
///    w(s) = v_i·(1 + s/√(s² + R²)), v_i at the disc and 2·v_i far below, with the slipstream's
///    radius from continuity, r_w = R·√(v_i/w): R/√2 far below.
/// 2. The far wake of a multirotor, a turbulent round jet measured below six drones by
///    Bauersfeld, Muller, Ziegler, Coletti and Scaramuzza (2024), "Robotics meets Fluid Dynamics:
///    a characterization of the induced airflow below a quadrotor as a turbulent jet",
///    arXiv:2403.13321, eqs. 3, 5, 6, 9: U_C = U_H·B̃/(s/l − s̃0), B̃ = 10.11, s̃0 = −5.817,
///    r½ = S·(s − s̃0·l), S = 0.07668, profile U/U_C = (1 + (√2 − 1)ξ²)^−2, ξ = r/r½, with U_H the
///    momentum-theory hover induced velocity and l the motor-to-motor (diagonal) distance. The
///    rotor jets merge between 1.5 l and 2.5 l below the drone (their Fig. 5): the two regions
///    blend linearly there.
/// 3. Outwash where the wake meets the ground, fitted to the model-rotor measurements of Tanner,
///    Overmeyer, Jenkins, Yao and Bartram (2015), "Experimental investigation of rotorcraft
///    outwash in ground effect", AHS Forum 71 (NTRS 20160006428): the mean outwash peaks at
///    1.7–1.8 R from the axis, at about 1.9 V_h for a rotor 1 R above the ground, 0.02–0.08 R
///    above it, in a layer about 0.15 R deep. Scaled by the arriving jet (its speed and radius),
///    so a rotor higher up, or a multirotor whose jets have merged, gives a weaker, wider ring.
///    Outside the peak the peak velocity falls as (ρ_p/ρ)^1.1, the radial wall jet's decay
///    (TODO(verify) against Poreh, Tsuei and Cermak, J. Appl. Mech. 34, 1967).
///
/// Everything is per group: the rotors of one robot. A body is not pushed by its own group's
/// wake (the rotor data already hold the isolated rotor, and the frame's download is left out).
/// Wake skew in wind or forward flight follows the air relative to the group: the wake column
/// leans along −n·v_i + (air − vehicle), Leishman's skew angle tan χ = μ/λ_i.
///
/// Not modelled: inflow above the disc, the wake's swirl, rotor-rotor interference within one
/// drone, unsteadiness (the mean flow only), obstacles other than the ground plane.
class Downwash {
public:
    using Vec3 = std::array<double, 3>;
    static constexpr std::size_t kNoGroup = std::numeric_limits<std::size_t>::max();

    struct Rotor {
        Vec3 hub{};            // m, world
        Vec3 thrustAxis{};     // unit, world: where the thrust points (the wake goes the other way)
        double radius{0.0};    // m
        double induced{0.0};   // m/s, momentum-theory induced velocity at the disc
        double thrust{0.0};    // N; no wake at zero or reverse thrust
        std::size_t group{0};  // the robot it belongs to
    };

    /// The ground plane (a point on it and its unit normal); none by default.
    void setGround(const Vec3& point, const Vec3& normal);
    void clearGround() { hasGround_ = false; }

    /// Takes this step's rotors and, per group, the air's velocity relative to the vehicle
    /// (m/s, world; for the skew), indexed by group (shorter: zero).
    void update(std::span<const Rotor> rotors, std::span<const Vec3> relativeAir = {});

    /// The rotors' air velocity at `x` (m/s, world), leaving out group `exclude`.
    [[nodiscard]] Vec3 velocityAt(const Vec3& x, std::size_t exclude = kNoGroup) const;

    /// The largest outwash speed at the ground over all groups (m/s, at the wall jet's peak
    /// height and radius); 0 without a ground or an active group.
    [[nodiscard]] double peakOutwash() const;

    /// The groups after update(), for tests and drawing.
    struct Group {
        std::size_t id{0};
        Vec3 centre{};           // mean hub position
        Vec3 down{};             // unit: the way the induced flow goes (−thrust)
        Vec3 wakeAxis{};         // unit: the way the wake column goes (skewed)
        double hoverInduced{0};  // U_H, m/s: rms of the rotors' v_i
        double length{0};        // l, m: motor-to-motor diagonal (2 R for a single rotor)
        std::vector<Rotor> rotors;
    };
    [[nodiscard]] const std::vector<Group>& groups() const { return groups_; }

private:
    [[nodiscard]] double axialSpeed(const Group& g, const Vec3& x, double& s) const;
    bool hasGround_{false};
    Vec3 groundPoint_{}, groundNormal_{0.0, 0.0, 1.0};
    std::vector<Group> groups_;
};

// --- The pieces, for tests and the docs ---------------------------------------------------

/// Near wake on the axis, s ≥ 0 below the disc: v_i·(1 + s/√(s² + R²)).
[[nodiscard]] double nearWakeSpeed(double induced, double radius, double s);
/// The near wake's radius from continuity, R·√(v_i/w(s)).
[[nodiscard]] double nearWakeRadius(double induced, double radius, double s);

/// Bauersfeld et al. (2024) far-jet constants (eqs. 6 and 9).
inline constexpr double kJetDecay = 10.11;     // B·d / l, the fitted decay product
inline constexpr double kJetOrigin = -5.817;   // s̃0, virtual origin in units of l
inline constexpr double kJetSpread = 0.07668;  // S, half-width growth per unit distance
inline constexpr double kJetMergeStart = 1.5;  // l: rotor jets begin to merge (their Fig. 5)
inline constexpr double kJetMergeEnd = 2.5;    // l: fully merged, where their fit starts
/// Centreline speed and half-width of the merged jet at distance s (m) below the drone.
[[nodiscard]] double farJetSpeed(double hoverInduced, double length, double s);
[[nodiscard]] double farJetHalfWidth(double length, double s);
/// The jet's radial profile, U/U_C at ξ = r/r½ (Bauersfeld et al. eq. 3, after Pope, Turbulent
/// Flows, 2000).
[[nodiscard]] double farJetProfile(double xi);

/// Tanner et al. (2015) outwash fit, for a single rotor 1 R above the ground (the calibration):
/// peak 1.9 V_h at 1.7 R, peak height 0.05 R, layer 0.15 R deep. The near wake arrives there at
/// w = 1.707 V_h with radius 0.765 R, so the peak is 1.113 times the arriving speed at 2.22 times
/// the arriving radius.
inline constexpr double kOutwashPeakSpeed = 1.9;    // ×V_h, at h = 1 R (1.7–2.1 in their data)
inline constexpr double kOutwashPeakRadius = 1.7;   // ×R (1.7–1.8 in their data)
inline constexpr double kOutwashPeakHeight = 0.05;  // ×R (0.02–0.08)
inline constexpr double kOutwashLayer = 0.15;       // ×R: where it has halved, above the peak
inline constexpr double kOutwashRise = 0.8;         // inboard of the peak, u ∝ (ρ/ρ_p)^0.8
inline constexpr double kOutwashDecay = 1.1;        // outboard, u ∝ (ρ_p/ρ)^1.1
/// Wall-jet thickening outboard of the peak, dδ/dρ (TODO(verify): the plane wall jet's 0.073,
/// Launder and Rodi, Annu. Rev. Fluid Mech. 15, 1983, taken for the radial one).
inline constexpr double kOutwashGrowth = 0.073;
/// Outwash speed at radius ρ from the impingement point and height z above the ground, for a jet
/// arriving at speed `arriving` with top-hat radius `jetRadius`.
[[nodiscard]] double outwashSpeed(double arriving, double jetRadius, double rho, double z);

/// The air at `x`: the logarithmic mean wind at its height above `groundZ`, plus `gust`, plus
/// the rotors' wakes except group `exclude`. The one place the aerodynamics domain and the
/// server's airflow view ask what the air does.
[[nodiscard]] std::array<double, 3> windAt(const Wind& wind, double groundZ,
                                           const std::array<double, 3>& gust,
                                           const Downwash& downwash, const std::array<double, 3>& x,
                                           std::size_t exclude = Downwash::kNoGroup);

}  // namespace lagrange::aerodynamics
