#pragma once

// The fire engineering correlations of RFC 0022 §2, §3 and §5, as plain functions in SI units.
// The published forms take the heat release rate in kW; these convert, with the constants named.
// Each is checked against its closed form in test_correlations.cpp, and the sources are in
// docs/physics/fire.md.

#include <array>
#include <span>

namespace lagrange::fire {

/// t² growth (RFC 0022 §2): Q = α·t² (W), t the time since ignition (s), α in W/s².
[[nodiscard]] double tSquaredHrr(double alpha, double sinceIgnition);

/// The equivalent diameter of a burning area: D = √(4A/π) (m).
[[nodiscard]] double equivalentDiameter(double area);

/// A pool fire (Babrauskas 1983, as RFC 0022 §2 gives it): Q = ṁ''∞·(1 − e^(−kβ·D))·ΔH_c·A (W),
/// with ṁ''∞ in kg/(m²·s), kβ in 1/m, ΔH_c in J/kg, A in m² and D its equivalent diameter.
[[nodiscard]] double poolHrr(double burningRate, double extinction, double heatOfCombustion,
                             double area);

/// Heskestad's mean flame height (SFPE Handbook 5th ed. ch. 13): L = 0.235·Q^(2/5) − 1.02·D (m,
/// Q in kW), never below 0.
[[nodiscard]] double flameHeight(double hrr, double diameter);

/// Heskestad's virtual origin: z0 = 0.083·Q^(2/5) − 1.02·D (m, Q in kW); may be negative.
[[nodiscard]] double virtualOrigin(double hrr, double diameter);

/// Heskestad's plume centreline velocity above the flame:
/// u0 = 3.4·(g/(c_p·T∞·ρ∞))^(1/3)·Q_c^(1/3)·(z − z0)^(−1/3) (m/s), Q_c = (1 − χ_r)·Q in W, z the
/// height above the fuel's surface (m), c_p the air's specific heat (J/(kg·K)), T∞ (K), ρ∞
/// (kg/m³), g (m/s²). 0 at or below the virtual origin.
[[nodiscard]] double plumeVelocity(double hrr, double radiativeFraction, double height,
                                   double virtualOriginHeight, double gravity, double airCp,
                                   double airTemperature, double airDensity);

/// The AGA flame tilt in wind, as Mudan and Croce give it in the SFPE Handbook: u* = u_w /
/// (g·ṁ''·D/ρ_a)^(1/3); cos θ = 1 for u* ≤ 1, u*^(−1/2) above. Returns cos θ. ṁ'' is the mass
/// burning flux (kg/(m²·s)); with no burning or no air it is 1 (upright).
[[nodiscard]] double tiltCosine(double windSpeed, double massFlux, double diameter, double gravity,
                                double airDensity);

/// The point-source model (RFC 0022 §3): q'' = χ_r·Q·cos β/(4π·R²) (W/m²). `cosBeta` is the
/// cosine between the target's normal and the line to the source (clamped at 0).
[[nodiscard]] double pointSourceFlux(double radiativeFraction, double hrr, double distance,
                                     double cosBeta = 1.0);

/// What a lumped node absorbs from an incident flux q'' (W): ε·q''·A_rad/4, A_rad/4 being the
/// mean projected area of a convex body of surface A_rad (Cauchy).
[[nodiscard]] double absorbedByNode(double emissivity, double flux, double radiativeArea);

/// Evans' sprinkler suppression time constant (NISTIR 5254, 1993, eq. 3): τ = 3.0·w''^(−1.85) s,
/// w'' the application rate in mm/s of water, which is kg/(m²·s) at 1000 kg/m³. Fitted to wood
/// cribs from 0.026 to 0.133 mm/s.
[[nodiscard]] double evansTau(double waterRate);
inline constexpr double kEvansMinRate = 0.026;  // kg/(m²·s): the fitted data's range (NISTIR 5254)
inline constexpr double kEvansMaxRate = 0.133;

/// Heat a kilogram of water takes up from `waterTemperature` (K) to steam at 100 °C (J/kg):
/// c_w·(373.15 − T_w) + h_fg; only h_fg when the temperature is not known (≤ 0).
[[nodiscard]] double waterHeatPerKg(double waterTemperature);
inline constexpr double kLatentHeatWater = 2.256e6;  // J/kg at 100 °C (RFC 0022 §5.1, NIST WebBook)

/// The burn-out factor (RFC 0022 §2, open question 5): 1 while more than `decayFraction` of the
/// fuel remains, then the remaining share of that fraction, linearly to 0.
[[nodiscard]] double burnOutFactor(double remaining, double fuelMass, double decayFraction);

/// A measured curve `t0 Q0 t1 Q1 ...` at `sinceIgnition` (s), linear between points, its last
/// value after its end, 0 before its start.
[[nodiscard]] double curveHrr(std::span<const double> curve, double sinceIgnition);

}  // namespace lagrange::fire
