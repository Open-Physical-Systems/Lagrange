#pragma once

// The gripper models of RFC 0020 §4.1–§4.2, free of the bus so that tests check them against
// their closed forms: a suction cup's pressure and pull, and an electromagnet's coil and pull.
// SI units throughout; pressures are absolute.

#include "lagrange/services/SceneQuery.hpp"
#include "lagrange/world/World.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <span>

namespace lagrange::actuators {

using Vec3 = std::array<double, 3>;

/// Specific gas constant of dry air, J/(kg·K) (R / M with M = 28.9647 g/mol; ISO 2533's value,
/// as the barometer's standard atmosphere uses it).
inline constexpr double kAirGasConstant = 287.05287;
/// Standard sea-level pressure (ISO 2533), Pa: the atmosphere when neither the weather nor the
/// site gives one.
inline constexpr double kStandardPressure = 101'325.0;
/// The temperature of the air in the cup when the site gives none, K: 20 °C, the reference
/// temperature of ISO 6358's flow rating (ISO 8778's reference atmosphere; TODO(verify) the
/// clause, the standards are sold).
inline constexpr double kReferenceTemperature = 293.15;
/// ISO 6358's reference density ρ0 of its flow rating, kg/m³: the ideal gas at 100 kPa and
/// 293.15 K (ISO 8778 rounds it to 1.185; TODO(verify)).
inline constexpr double kIso6358ReferenceDensity = 100'000.0 / (kAirGasConstant * 293.15);
/// Critical pressure ratio b of an ideal orifice in air, (2 / (γ + 1))^(γ / (γ − 1)) with
/// γ = 1.4: below it the flow is choked (ISO 6358 rates a real one by its own b).
inline constexpr double kCriticalPressureRatio = 0.528282;
/// Vacuum permeability μ0, H/m (CODATA 2018: 1.25663706212e-6).
inline constexpr double kMu0 = 1.25663706212e-6;

/// SuctionGripperAPI_1's attributes.
struct SuctionParams {
    double diameter{0.0};          // m, effective
    double volume{0.0};            // m³, cup and hose
    double flow{0.0};              // kg/s, the generator's suction at atmospheric pressure
    double ultimatePressure{0.0};  // Pa absolute, where the generator's flow falls to zero
    double sealGap{0.002};         // m
    double maxTilt{0.26};          // rad
    double leakConductance{0.0};   // m³/(s·Pa), ISO 6358 C of the open cup; 0: vents at once
    Vec3 axis{0.0, 0.0, 1.0};      // in the prim's frame, unit
};

/// The cup's effective area π d² / 4, m².
[[nodiscard]] double suctionArea(const SuctionParams& p);

/// ISO 6358's mass flow through a component of sonic conductance `conductance` (m³/(s·Pa)) from
/// `upstream` to `downstream` (Pa) at upstream temperature `temperature` (K): choked below the
/// critical ratio b, an elliptic fall to zero above it. 0 when downstream is not lower.
[[nodiscard]] double iso6358MassFlow(double conductance, double upstream, double downstream,
                                     double temperature);

/// What the cup sees in one step.
struct SuctionInputs {
    bool on{false};                             // the generator runs
    bool sealed{false};                         // the lip is on one surface all round
    double atmosphere{kStandardPressure};       // Pa
    double temperature{kReferenceTemperature};  // K
    /// Volume flow per pressure difference through a porous surface the cup is sealed on,
    /// m³/(s·Pa): its air permeability times the cup's area. 0: airtight.
    double porousConductance{0.0};
};

/// The cup's pressure after `dt` from `pressure` (Pa). The air in the cup at constant temperature,
/// V / (R T) · dp/dt = ṁ_in − ṁ_gen (§4.1). The generator's line, ṁ_gen = q0 (p − p_min) /
/// (p_atm − p_min), and a porous surface's Darcy leak, ρ_atm κ A (p_atm − p), are linear in p and
/// integrated exactly over the step (a sealed cup on airtight ground follows the closed form
/// p_min + (p_atm − p_min) e^(−t/τ) to rounding). Switched off, the cup vents back through the
/// generator along the same line. Open (not sealed), air comes in through leakConductance by
/// ISO 6358, or at once when that is 0.
[[nodiscard]] double stepSuctionPressure(const SuctionParams& params, double pressure, double dt,
                                         const SuctionInputs& in);

/// Time constant τ = V (p_atm − p_min) / (R T q0) of evacuating a sealed, airtight cup, s.
[[nodiscard]] double evacuationTimeConstant(const SuctionParams& params, double atmosphere,
                                            double temperature);

/// The pull of a sealed cup, (p_atm − p) A, N (0 when not below the atmosphere).
[[nodiscard]] double suctionForce(const SuctionParams& params, double pressure, double atmosphere);

/// MagnetGripperAPI_1's attributes.
struct MagnetParams {
    bool electropermanent{false};
    double turns{0.0};
    double poleArea{0.0};          // m², each of two poles
    double ironPathLength{0.0};    // m, core and part together
    double ironPermeability{1.0};  // the core's relative permeability
    double saturation{0.0};        // T; 0: none
    double coilResistance{0.0};    // ohm
    double coilInductance{0.0};    // H
    double ratedVoltage{0.0};      // V
    Vec3 axis{0.0, 0.0, 1.0};
};

/// The flux density in the gap, T: B = μ0 N i / (2 g + l_iron / μr_eff), with half the iron path in
/// the core (ironPermeability) and half in the part (`partPermeability`), clamped at saturation.
/// 0 for a part that is not ferromagnetic (relative permeability at most 1).
[[nodiscard]] double magnetFlux(const MagnetParams& params, double current, double gap,
                                double partPermeability);

/// The pull of both poles across the gap, F = B² A_pole / μ0 (Maxwell stress, B²/(2 μ0) per pole
/// face), N.
[[nodiscard]] double magnetForce(const MagnetParams& params, double current, double gap,
                                 double partPermeability);

/// The coil current after `dt` from `current` with `voltage` across it: L di/dt = v − R i, exact
/// over the step (at once when L is 0).
[[nodiscard]] double stepCoilCurrent(const MagnetParams& params, double current, double voltage,
                                     double dt);

/// Ray casts that skip one body's own shapes: the first hit along `dir` from `origin` within
/// `reach` on a shape whose body is not `self`. Shape i's body is `shapeBody[i]`. A ray that
/// starts inside another shape (past the gripper's own, into a part pressed against it) hits it
/// where it starts, with no normal.
[[nodiscard]] services::RayHit castPast(const services::SceneQuery& scene, const Vec3& origin,
                                        const Vec3& dir, double reach, BodyId self,
                                        std::span<const BodyId> shapeBody);

/// Where a cup's lip meets what is in front of it (§4.1): `kRimRays` rays from the lip's rim,
/// along the axis. It seals when every ray finds a surface within sealGap of the lip, all on one
/// body, each facing the cup within maxTilt.
inline constexpr int kRimRays = 8;
struct Seal {
    bool sealed{false};
    BodyId body{kWorldBody};  // what it is sealed on
    std::uint32_t shape{services::RayHit::kNone};
};
/// `centre` and `axis` (unit) in the scene's frame.
[[nodiscard]] Seal findSeal(const services::SceneQuery& scene, const SuctionParams& params,
                            const Vec3& centre, const Vec3& axis, BodyId self,
                            std::span<const BodyId> shapeBody);

/// How far a magnet's face is from the surface in front of it (one ray from the face's centre,
/// along the axis), m; the shape hit, kNone when nothing is within kMagnetReach.
inline constexpr double kMagnetReach = 0.1;  // m: beyond it the pull is left out
struct Gap {
    double gap{0.0};
    std::uint32_t shape{services::RayHit::kNone};
};
[[nodiscard]] Gap findGap(const services::SceneQuery& scene, const Vec3& centre, const Vec3& axis,
                          BodyId self, std::span<const BodyId> shapeBody);

}  // namespace lagrange::actuators
