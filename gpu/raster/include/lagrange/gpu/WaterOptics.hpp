#pragma once

// The optics of a water surface for the realistic look (docs/physics/water.md "How the water
// looks"): how rough the air makes it below what the moving surface's grid holds, and how much
// of the light it reflects. The shader (gpu/kernels/camera/realistic.slang, fsWater) computes
// the same Fresnel; the slope variance is computed here, on the host, per cell.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <span>
#include <vector>

namespace lagrange::gpu::water_optics {

/// Refractive index of water in the visible (20 °C, 589 nm: 1.333; Hale and Querry, Appl. Opt.
/// 12(3), 1973, give 1.331 to 1.338 over 400-700 nm).
inline constexpr double kRefractiveIndex = 1.333;
/// von Kármán's constant.
inline constexpr double kKarman = 0.40;
/// Charnock's constant for the roughness length of the sea, z0 = α·u*²/g (Smith, J. Geophys.
/// Res. 93(C12), 1988: 0.011), plus the smooth-flow term 0.11·ν/u*.
inline constexpr double kCharnock = 0.011;
inline constexpr double kAirViscosity = 1.5e-5;  // m²/s, at 20 °C
inline constexpr double kGravity = 9.80665;      // m/s²
/// Cox and Munk's anemometer height (J. Opt. Soc. Am. 44(11), 1954: 41 ft).
inline constexpr double kCoxMunkHeight = 12.5;  // m

/// Unpolarized Fresnel reflectance of a smooth interface from the side of index 1 into index
/// `n`, at an incidence whose cosine is `cosIncidence` (Born and Wolf, Principles of Optics,
/// §1.5.2): the mean of the s and p reflectances. For water, 0.020 at normal incidence and 1 at
/// grazing.
[[nodiscard]] inline double fresnel(double cosIncidence, double n = kRefractiveIndex) {
    const double ci = std::clamp(cosIncidence, 0.0, 1.0);
    const double st2 = (1.0 - ci * ci) / (n * n);  // sin² of the refracted angle
    if (st2 >= 1.0) {
        return 1.0;
    }
    const double ct = std::sqrt(1.0 - st2);
    const double rs = (ci - n * ct) / (ci + n * ct);
    const double rp = (n * ci - ct) / (n * ci + ct);
    return 0.5 * (rs * rs + rp * rp);
}

/// The roughness length (m) of the water under a friction velocity u* (m/s): Charnock's
/// gravity-wave term and the aerodynamically smooth one (Smith 1988).
[[nodiscard]] inline double roughnessLength(double frictionVelocity) {
    const double u = std::max(frictionVelocity, 1e-4);
    return kCharnock * u * u / kGravity + 0.11 * kAirViscosity / u;
}

/// The wind (m/s) at `height` over the water in a neutral surface layer of friction velocity u*:
/// the log law U(z) = (u*/κ)·ln(z/z0).
[[nodiscard]] inline double windAt(double frictionVelocity, double height) {
    if (frictionVelocity <= 0.0) {
        return 0.0;
    }
    return frictionVelocity / kKarman * std::log(height / roughnessLength(frictionVelocity));
}

/// The friction velocity (m/s) of a wind `speed` (m/s) at `height` over open water, the log law
/// inverted by fixed-point iteration (it converges in a few steps: z0 changes slowly with u*).
[[nodiscard]] inline double frictionVelocityOf(double speed, double height = 10.0) {
    if (speed <= 0.0) {
        return 0.0;
    }
    double u = 0.04 * speed;
    for (int i = 0; i < 30; ++i) {
        u = kKarman * speed / std::log(height / roughnessLength(u));
    }
    return u;
}

/// Cox and Munk's (1954) total mean square slope of a clean sea, 0.003 + 5.12e-3·U, and of a
/// sea under an oil slick, 0.008 + 1.56e-3·U (U, m/s, at 12.5 m; Cox and Munk, J. Mar. Res.
/// 13(2), 1954). A slick damps the short capillary-gravity waves and leaves the longer ones, so
/// the difference, 3.56e-3·U − 0.005, is taken as the slope variance of the short waves alone.
[[nodiscard]] inline double coxMunkClean(double windAt12m5) {
    return 0.003 + 5.12e-3 * std::max(windAt12m5, 0.0);
}
[[nodiscard]] inline double coxMunkSlick(double windAt12m5) {
    return 0.008 + 1.56e-3 * std::max(windAt12m5, 0.0);
}

/// The mean square slope (both directions summed) of the capillary ripples that a friction
/// velocity u* (m/s) raises on clean water: Cox and Munk's clean minus slick at the wind that
/// u* makes at 12.5 m in the log law. It is 0 below about u* = 0.045 m/s (a wind of 1.4 m/s at
/// 12.5 m), the onset of ripples, where Kahma and Donelan (J. Fluid Mech. 192, 1988) put it
/// on clean water (u* ≈ 5 cm/s in theory, measured up to twice that).
///
/// Assumed: that the ripples are in equilibrium with the local u*, wherever it comes from (a
/// wind over open water or a rotor's outwash), so that a sea's slope statistics apply on a
/// patch of metres. Short ripples (centimetres) do come to equilibrium within tens of
/// centimetres of fetch and a second; longer waves do not, which is why only the short waves'
/// share is taken.
[[nodiscard]] inline double rippleSlopeVariance(double frictionVelocity) {
    const double u = windAt(frictionVelocity, kCoxMunkHeight);
    return std::max(coxMunkClean(u) - coxMunkSlick(u), 0.0);
}

/// Cox and Munk's (1954) upwind and crosswind mean square slopes rise with the wind as
/// 3.16e-3·U and 1.92e-3·U on a clean sea and 0.78e-3·U and 0.84e-3·U under a slick (U at
/// 12.5 m; J. Mar. Res. 13(2), 1954): the short waves' share of each rises as 2.38e-3·U upwind
/// and 1.08e-3·U across. The ripples' variance (rippleSlopeVariance) is split between the two
/// directions in that ratio, 2.2 to 1: short wind waves are steeper along the wind than across.
inline constexpr double kShortUpwindRate = 3.16e-3 - 0.78e-3;
inline constexpr double kShortCrosswindRate = 1.92e-3 - 0.84e-3;
/// The ripples' upwind (along the stress) and crosswind mean square slopes for a friction
/// velocity u*: rippleSlopeVariance split in Cox and Munk's ratio. Their sum is the total.
[[nodiscard]] inline std::array<double, 2> rippleSlopeVariances(double frictionVelocity) {
    const double total = rippleSlopeVariance(frictionVelocity);
    const double sum = kShortUpwindRate + kShortCrosswindRate;
    return {total * kShortUpwindRate / sum, total * kShortCrosswindRate / sum};
}

// --- Fetch: how far ripples need to grow --------------------------------------------------

/// σ/ρ of clean water at 20 °C, m³/s² (0.0728 N/m over 1000 kg/m³).
inline constexpr double kTensionOverDensity = 7.275e-5;
/// Water's kinematic viscosity at 20 °C, m²/s.
inline constexpr double kWaterViscosity = 1.0e-6;
/// The ripple that grows first and stands for the short waves here: the one of least phase
/// speed, k_m = √(g·ρ/σ) (367 rad/m, 1.71 cm), whose group speed equals its phase speed,
/// c_m = (4·g·σ/ρ)^¼ = 0.231 m/s (Lamb, Hydrodynamics, 1932, §267).
[[nodiscard]] inline double rippleWavenumber() {
    return std::sqrt(kGravity / kTensionOverDensity);
}
[[nodiscard]] inline double ripplePhaseSpeed() {
    return std::pow(4.0 * kGravity * kTensionOverDensity, 0.25);
}
/// Plant's (J. Geophys. Res. 87(C3), 1982) growth rate of wind-wave energy, β = 0.04·(u*/c)²·ω
/// (1/s, measured 0.02 to 0.06; TODO(verify) the coefficient for capillary ripples), less the
/// viscous decay of their energy, 4·ν·k² (Lamb §348), at the ripple of least phase speed: the
/// net rate, 0 where the air is too weak to grow it.
[[nodiscard]] inline double rippleGrowthRate(double frictionVelocity) {
    const double k = rippleWavenumber();
    const double c = ripplePhaseSpeed();
    const double beta = 0.04 * (frictionVelocity / c) * (frictionVelocity / c) * c * k;
    return std::max(beta - 4.0 * kWaterViscosity * k * k, 0.0);
}
/// The ripples' slope variance a fetch takes to come into equilibrium with the air (1/m): their
/// growth rate over the speed their energy travels at, γ = rippleGrowthRate / c_m. Along the air's
/// path over the water the variance relaxes to rippleSlopeVariance's at that rate,
/// dS/ds = γ·(S_eq − S): it starts from the still water where the air does (a jet's stagnation
/// point, or a slick's edge) and lags where u* changes faster than γ⁻¹. Behavioural: a
/// relaxation, not the linear-then-exponential growth of a wave spectrum (Phillips, Miles).
[[nodiscard]] inline double fetchRate(double frictionVelocity) {
    return rippleGrowthRate(frictionVelocity) / ripplePhaseSpeed();
}

/// The ripples on a grid of `cells`² square cells of `cell` m, the air's friction velocity on
/// each in `friction` (m/s, x for every cell then y, as RealisticWaterSurface::friction), plus a
/// uniform wind's u* `wind` (m/s, x and y) everywhere: per cell, in `variance` (cells², the
/// total of the two directions) the slope variance the fetch has grown along the air's path
/// (fetchRate), and in `direction` (2·cells², x then y, unit) the way the stress runs. The air
/// enters the grid at its rim with the wind's ripples, which are in equilibrium (an open sea's
/// fetch). Their stresses add as vectors, ρ·|u*|·u*. Solved by upwind sweeps (each cell from the
/// point a cell upstream, bilinear), in the four orders of the grid, until it settles.
inline void fetchLimitedRipples(std::size_t cells, double cell, std::span<const float> friction,
                                const std::array<double, 2>& wind, std::vector<double>& variance,
                                std::vector<double>& direction) {
    const std::size_t n = cells;
    variance.assign(n * n, 0.0);
    direction.assign(2 * n * n, 0.0);
    if (n < 2 || !(cell > 0.0)) {
        return;
    }
    const double windU = std::hypot(wind[0], wind[1]);
    const double background = rippleSlopeVariance(windU);
    std::vector<double> rate(n * n, 0.0);
    std::vector<double> target(n * n, 0.0);
    const bool rubbed = friction.size() == 2 * n * n;
    for (std::size_t i = 0; i < n * n; ++i) {
        double sx = windU * wind[0];
        double sy = windU * wind[1];
        if (rubbed && std::isfinite(friction[i]) && std::isfinite(friction[n * n + i])) {
            const double ux = friction[i];
            const double uy = friction[n * n + i];
            const double u = std::hypot(ux, uy);
            sx += u * ux;
            sy += u * uy;
        }
        const double magnitude = std::hypot(sx, sy);
        const double u = std::sqrt(magnitude);
        target[i] = rippleSlopeVariance(u);
        rate[i] = fetchRate(u);
        direction[i] = magnitude > 0.0 ? sx / magnitude : (windU > 0.0 ? wind[0] / windU : 1.0);
        direction[n * n + i] =
            magnitude > 0.0 ? sy / magnitude : (windU > 0.0 ? wind[1] / windU : 0.0);
        variance[i] = background;
    }
    const auto at = [&](double fx, double fy) {
        if (fx < 0.0 || fy < 0.0 || fx > static_cast<double>(n - 1) ||
            fy > static_cast<double>(n - 1)) {
            return background;  // from beyond the grid: the wind's
        }
        const auto x0 = std::min(static_cast<std::size_t>(fx), n - 2);
        const auto y0 = std::min(static_cast<std::size_t>(fy), n - 2);
        const double tx = fx - static_cast<double>(x0);
        const double ty = fy - static_cast<double>(y0);
        const std::size_t i = y0 * n + x0;
        return (1.0 - ty) * ((1.0 - tx) * variance[i] + tx * variance[i + 1]) +
               ty * ((1.0 - tx) * variance[i + n] + tx * variance[i + n + 1]);
    };
    // Enough rounds of four sweeps for the farthest cell's upstream path to have reached it
    // along any monotone run, and for a stagnation point's outflow to settle.
    constexpr int kRounds = 3;
    for (int round = 0; round < kRounds; ++round) {
        for (int order = 0; order < 4; ++order) {
            const bool flipX = (order & 1) != 0;
            const bool flipY = (order & 2) != 0;
            for (std::size_t yy = 0; yy < n; ++yy) {
                const std::size_t y = flipY ? n - 1 - yy : yy;
                for (std::size_t xx = 0; xx < n; ++xx) {
                    const std::size_t x = flipX ? n - 1 - xx : xx;
                    const std::size_t i = y * n + x;
                    const double up = at(static_cast<double>(x) - direction[i],
                                         static_cast<double>(y) - direction[n * n + i]);
                    const double keep = std::exp(-rate[i] * cell);
                    variance[i] = target[i] + (up - target[i]) * keep;
                }
            }
        }
    }
}

// --- Whitecaps ------------------------------------------------------------------------------

/// The air speed over still water above which its surface is unstable (Kelvin–Helmholtz with
/// gravity and surface tension; Lamb 1932, §268): U² = 2·(ρ_a + ρ_w)/(ρ_a·ρ_w)·√(g·σ·(ρ_w − ρ_a));
/// 6.7 m/s for air (1.2 kg/m³) over water (998 kg/m³, 0.0728 N/m). The same as
/// aerodynamics::kelvinHelmholtzOnset, where the spray starts; the viewer does not link it.
[[nodiscard]] inline double kelvinHelmholtzOnset(double rhoAir = 1.2, double rhoWater = 998.0,
                                                 double tension = 0.0728) {
    return std::sqrt(2.0 * (rhoAir + rhoWater) / (rhoAir * rhoWater) *
                     std::sqrt(kGravity * tension * (rhoWater - rhoAir)));
}
/// u*/u_m of the rotors' wall jet over the water: aerodynamics::kWallJetFriction (√(c_f/2),
/// c_f ≈ 0.005), with which the server's water/friction_velocity was made from the outwash's
/// peak speed u_m; so u_m = u*/kWallJetFriction here.
inline constexpr double kWallJetFriction = 0.05;
/// Monahan and O'Muircheartaigh's (J. Phys. Oceanogr. 10(12), 1980) whitecap coverage of the
/// sea, W = 3.84e-6·U10^3.41 (U10, m/s, the wind at 10 m), as a fraction of the area (capped at
/// 1). Fitted to winds of 1 to 20 m/s over the sea; for a rotor's outwash the wind at 10 m is the
/// log law's of the local u* (windAt), which carries it beyond its data. TODO(verify) its use for
/// a wall jet of a metre's fetch.
[[nodiscard]] inline double whitecapFraction(double windAt10m) {
    const double u = std::max(windAt10m, 0.0);
    return std::min(3.84e-6 * std::pow(u, 3.41), 1.0);
}
/// The whitecaps of a cell the air rubs with the outwash's friction velocity `outwash` (m/s,
/// |u*| of the rotors' wall jet) and the wind's `wind` (m/s): the wind's own coverage, and where
/// the outwash's speed u_m = u*/kWallJetFriction passes `onset` (Kelvin–Helmholtz, where its
/// spray is torn off), Monahan's for the stress of both (their u* adding as stresses). None in
/// a wall jet below the onset, where its surface does not break.
[[nodiscard]] inline double whitecapOf(double outwash, double wind,
                                       double onset = kelvinHelmholtzOnset()) {
    const double own = whitecapFraction(windAt(wind, 10.0));
    if (outwash / kWallJetFriction < onset) {
        return own;
    }
    const double both = std::sqrt(outwash * outwash + wind * wind);
    return std::max(own, whitecapFraction(windAt(both, 10.0)));
}

}  // namespace lagrange::gpu::water_optics
