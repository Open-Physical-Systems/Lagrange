#pragma once

// The optics of a water surface for the realistic look (docs/physics/water.md "How the water
// looks"): how rough the air makes it below what the moving surface's grid holds, and how much
// of the light it reflects. The shader (gpu/kernels/camera/realistic.slang, fsWater) computes
// the same Fresnel; the slope variance is computed here, on the host, per cell.

#include <algorithm>
#include <cmath>

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

}  // namespace lagrange::gpu::water_optics
