#include "lagrange/sensors/ThermalCamera.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace lagrange::sensors {
namespace {

constexpr double kPi4Over15 = std::numbers::pi * std::numbers::pi * std::numbers::pi *
                              std::numbers::pi / 15.0;  // ∫_0^∞ t³/(eᵗ−1) dt
constexpr double kSmallX = 0.1;                         // below, the integral's Taylor series
constexpr int kMostTerms = 100000;

/// ∫_x^∞ t³/(eᵗ−1) dt.
double tail(double x) {
    if (!(x < std::numeric_limits<double>::infinity())) {
        return 0.0;
    }
    if (x < kSmallX) {
        // t³/(eᵗ−1) = t² − t³/2 + t⁴/12 − t⁶/720 + ...
        const double x3 = x * x * x;
        return kPi4Over15 - (x3 / 3.0 - x3 * x / 8.0 + x3 * x * x / 60.0 - x3 * x3 * x / 5040.0);
    }
    double sum = 0.0;
    for (int n = 1; n <= kMostTerms; ++n) {
        const double nn = n;
        const double term =
            std::exp(-nn * x) * (x * x * x / nn + 3.0 * x * x / (nn * nn) +
                                 6.0 * x / (nn * nn * nn) + 6.0 / (nn * nn * nn * nn));
        sum += term;
        if (term < 1e-17 * sum) {
            break;
        }
    }
    return sum;
}

/// hc/(λkT), +inf at λ = 0.
double reduced(double wavelength, double temperature) {
    if (wavelength <= 0.0) {
        return std::numeric_limits<double>::infinity();
    }
    return kPlanck * kLightSpeed / (wavelength * kBoltzmann * temperature);
}

}  // namespace

double bandRadiance(double temperature, double shortest, double longest) {
    if (temperature <= 0.0 || longest <= shortest) {
        return 0.0;
    }
    const double kt = kBoltzmann * temperature;
    const double scale =
        2.0 * kt * kt * kt * kt / (kPlanck * kPlanck * kPlanck * kLightSpeed * kLightSpeed);
    return scale * (tail(reduced(longest, temperature)) - tail(reduced(shortest, temperature)));
}

double blackBodyFraction(double wavelengthTimesTemperature) {
    if (wavelengthTimesTemperature <= 0.0) {
        return 0.0;
    }
    return tail(kPlanck * kLightSpeed / (kBoltzmann * wavelengthTimesTemperature)) / kPi4Over15;
}

double clearSkyTemperature(double airTemperature) {
    constexpr double kSwinbank = 0.0552;  // K^-0.5 (Swinbank 1963)
    return kSwinbank * std::pow(airTemperature, 1.5);
}

ThermalBand::ThermalBand(double shortest, double longest) : shortest_(shortest), longest_(longest) {
    const auto count = static_cast<std::size_t>(std::lround((kHighest - kLowest) / kStep)) + 1;
    table_.resize(count);
    for (std::size_t k = 0; k < count; ++k) {
        table_[k] = bandRadiance(kLowest + static_cast<double>(k) * kStep, shortest, longest);
    }
}

double ThermalBand::radiance(double temperature) const {
    return bandRadiance(temperature, shortest_, longest_);
}

double ThermalBand::temperatureOf(double radiance) const {
    if (radiance <= table_.front()) {
        return kLowest;
    }
    if (radiance >= table_.back()) {
        return kHighest;
    }
    const auto it = std::ranges::upper_bound(table_, radiance);
    const auto k = static_cast<std::size_t>(it - table_.begin());  // table_[k-1] <= L < table_[k]
    const double lo = table_[k - 1];
    const double hi = table_[k];
    double t = kLowest + (static_cast<double>(k - 1) + (radiance - lo) / (hi - lo)) * kStep;
    // Two Newton steps on the exact radiance take the interpolation's error (about 1 mK) away.
    for (int i = 0; i < 2; ++i) {
        const double h = 1e-3;
        const double f = bandRadiance(t, shortest_, longest_) - radiance;
        const double df =
            (bandRadiance(t + h, shortest_, longest_) - bandRadiance(t - h, shortest_, longest_)) /
            (2.0 * h);
        if (df > 0.0) {
            t -= f / df;
        }
    }
    return t;
}

double ThermalBand::apparent(double surface, double emissivity, double surroundings) const {
    const double e = std::clamp(emissivity, 0.0, 1.0);
    return temperatureOf(e * radiance(surface) + (1.0 - e) * radiance(surroundings));
}

std::array<std::uint8_t, 4> paletteColour(ThermalPalette palette, double x) {
    x = std::clamp(std::isfinite(x) ? x : 0.0, 0.0, 1.0);
    const auto byte = [](double v) {
        return static_cast<std::uint8_t>(std::lround(std::clamp(v, 0.0, 1.0) * 255.0));
    };
    switch (palette) {
        case ThermalPalette::WhiteHot:
            return {byte(x), byte(x), byte(x), 255};
        case ThermalPalette::BlackHot:
            return {byte(1.0 - x), byte(1.0 - x), byte(1.0 - x), 255};
        case ThermalPalette::Ironbow: {
            // Stops, by eye: black, indigo, purple-red, orange, yellow, white.
            constexpr std::array<std::array<double, 4>, 6> kStops{{{0.0, 0.0, 0.0, 0.0},
                                                                   {0.2, 0.15, 0.0, 0.45},
                                                                   {0.45, 0.65, 0.0, 0.55},
                                                                   {0.7, 0.95, 0.4, 0.0},
                                                                   {0.88, 1.0, 0.8, 0.1},
                                                                   {1.0, 1.0, 1.0, 0.95}}};
            std::size_t k = 1;
            while (k + 1 < kStops.size() && x > kStops[k][0]) {
                ++k;
            }
            const auto& a = kStops[k - 1];
            const auto& b = kStops[k];
            const double f = (x - a[0]) / (b[0] - a[0]);
            return {byte(a[1] + f * (b[1] - a[1])), byte(a[2] + f * (b[2] - a[2])),
                    byte(a[3] + f * (b[3] - a[3])), 255};
        }
    }
    return {0, 0, 0, 255};
}

}  // namespace lagrange::sensors
