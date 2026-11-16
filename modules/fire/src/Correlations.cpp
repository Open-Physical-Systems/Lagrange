#include "lagrange/fire/Correlations.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace lagrange::fire {

namespace {

constexpr double kWattsPerKw = 1e3;  // the correlations take Q in kW
// Heskestad (SFPE Handbook 5th ed. ch. 13), Q in kW and lengths in m.
constexpr double kFlameHeightCoeff = 0.235;    // m/kW^(2/5)
constexpr double kVirtualOriginCoeff = 0.083;  // m/kW^(2/5)
constexpr double kDiameterCoeff = 1.02;        // 1
constexpr double kPlumeVelocityCoeff = 3.4;    // 1, SI form with Q_c in W
// Evans, NISTIR 5254 (1993) eq. 3: τ = 3.0·w''^(−1.85), w'' in mm/s, τ in s.
constexpr double kEvansCoeff = 3.0;
constexpr double kEvansExponent = -1.85;
// Liquid water's specific heat, J/(kg·K), over 20 to 100 °C. TODO(verify): the mean of the NIST
// Chemistry WebBook's saturated-liquid c_p over that range (4.18 to 4.22 kJ/(kg·K) there).
constexpr double kWaterCp = 4186.0;
constexpr double kBoilingPoint = 373.15;  // K, at 1 atm

}  // namespace

double tSquaredHrr(double alpha, double sinceIgnition) {
    const double t = std::max(sinceIgnition, 0.0);
    return alpha * t * t;
}

double equivalentDiameter(double area) {
    return std::sqrt(4.0 * std::max(area, 0.0) / std::numbers::pi);
}

double poolHrr(double burningRate, double extinction, double heatOfCombustion, double area) {
    const double d = equivalentDiameter(area);
    return burningRate * (1.0 - std::exp(-extinction * d)) * heatOfCombustion * area;
}

double flameHeight(double hrr, double diameter) {
    const double q = std::max(hrr, 0.0) / kWattsPerKw;
    return std::max(kFlameHeightCoeff * std::pow(q, 0.4) - kDiameterCoeff * diameter, 0.0);
}

double virtualOrigin(double hrr, double diameter) {
    const double q = std::max(hrr, 0.0) / kWattsPerKw;
    return kVirtualOriginCoeff * std::pow(q, 0.4) - kDiameterCoeff * diameter;
}

double plumeVelocity(double hrr, double radiativeFraction, double height,
                     double virtualOriginHeight, double gravity, double airCp,
                     double airTemperature, double airDensity) {
    const double above = height - virtualOriginHeight;
    const double convective = (1.0 - radiativeFraction) * hrr;
    if (!(above > 0.0) || !(convective > 0.0) || !(airCp * airTemperature * airDensity > 0.0)) {
        return 0.0;
    }
    return kPlumeVelocityCoeff * std::cbrt(gravity / (airCp * airTemperature * airDensity)) *
           std::cbrt(convective) / std::cbrt(above);
}

double tiltCosine(double windSpeed, double massFlux, double diameter, double gravity,
                  double airDensity) {
    const double scale = gravity * massFlux * diameter / airDensity;
    if (!(scale > 0.0) || !(airDensity > 0.0)) {
        return 1.0;
    }
    const double uStar = std::abs(windSpeed) / std::cbrt(scale);
    return uStar <= 1.0 ? 1.0 : 1.0 / std::sqrt(uStar);
}

double pointSourceFlux(double radiativeFraction, double hrr, double distance, double cosBeta) {
    if (!(distance > 0.0)) {
        return 0.0;
    }
    return radiativeFraction * std::max(hrr, 0.0) * std::max(cosBeta, 0.0) /
           (4.0 * std::numbers::pi * distance * distance);
}

double absorbedByNode(double emissivity, double flux, double radiativeArea) {
    return emissivity * flux * radiativeArea / 4.0;
}

double evansTau(double waterRate) {
    return kEvansCoeff * std::pow(waterRate, kEvansExponent);
}

double waterHeatPerKg(double waterTemperature) {
    if (!(waterTemperature > 0.0)) {
        return kLatentHeatWater;
    }
    return kWaterCp * std::max(kBoilingPoint - waterTemperature, 0.0) + kLatentHeatWater;
}

double burnOutFactor(double remaining, double fuelMass, double decayFraction) {
    const double tail = decayFraction * fuelMass;
    if (remaining <= 0.0) {
        return 0.0;
    }
    if (!(tail > 0.0) || remaining >= tail) {
        return 1.0;
    }
    return remaining / tail;
}

double curveHrr(std::span<const double> curve, double sinceIgnition) {
    const std::size_t n = curve.size() / 2;
    if (n == 0 || sinceIgnition < curve[0]) {
        return 0.0;
    }
    for (std::size_t i = 1; i < n; ++i) {
        const double t0 = curve[2 * (i - 1)];
        const double t1 = curve[2 * i];
        if (sinceIgnition <= t1) {
            const double q0 = curve[2 * (i - 1) + 1];
            const double q1 = curve[2 * i + 1];
            return t1 > t0 ? q0 + (q1 - q0) * (sinceIgnition - t0) / (t1 - t0) : q1;
        }
    }
    return curve[2 * n - 1];
}

}  // namespace lagrange::fire
