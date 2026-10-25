// What the weather does to sensors (WeatherEffects.hpp).
#include "lagrange/sensors/WeatherEffects.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace lagrange::sensors {

namespace {

constexpr double kNoReturn = std::numeric_limits<double>::infinity();
// Marshall–Palmer (as lagrange_weather; Marshall and Palmer 1948): N0 = 8000 m⁻³·mm⁻¹,
// Λ = 4.1·R^−0.21 mm⁻¹ with R in mm/h.
constexpr double kMarshallPalmerN0 = 8.0e6;    // m⁻⁴
constexpr double kMpsPerMmph = 1e-3 / 3600.0;  // m/s per mm/h
constexpr int kPeakSamples = 400;              // ranges searched for the fog return's peak
constexpr double kPeakSearchOverlaps = 10.0;   // out to this many overlap distances

double standardNormalCdf(double x) {
    return 0.5 * std::erfc(-x / std::numbers::sqrt2);
}

}  // namespace

double lidarDetectionRange(double maxRange, double extinction, double contamination) {
    const double reach = maxRange * std::clamp(1.0 - contamination, 0.0, 1.0);
    if (extinction <= 0.0 || reach <= 0.0) {
        return reach;
    }
    // r·exp(σr) = R·T, by Newton from the clear-air range (the function is convex and rising,
    // so the iterates fall monotonically onto the root).
    double r = reach;
    for (int i = 0; i < 60; ++i) {
        const double e = std::exp(extinction * r);
        const double next = r - (r * e - reach) / (e * (1.0 + extinction * r));
        if (std::abs(next - r) < 1e-12 * reach) {
            return next;
        }
        r = next;
    }
    return r;
}

double fogReturn(const LidarClutter& c, double maxRange, double fogExtinction, double contamination,
                 double range) {
    if (fogExtinction <= 0.0 || range <= 0.0) {
        return 0.0;
    }
    const double window = std::clamp(1.0 - contamination, 0.0, 1.0);
    // The receiver's overlap with the beam: a smooth step that reaches 1 about c.overlap.
    const double x = range / c.overlap;
    const double overlap = 1.0 - std::exp(-x * x * x * x);
    const double backscatter = fogExtinction / c.lidarRatio * c.pulseLength;  // per sr
    return c.gain * overlap * backscatter * std::exp(-2.0 * fogExtinction * range) * window *
           window * std::numbers::pi * maxRange * maxRange / (c.reflectivity * range * range);
}

Clutter fogClutter(const LidarClutter& c, double maxRange, double fogExtinction,
                   double contamination, double minRange) {
    Clutter out;
    if (fogExtinction <= 0.0) {
        return out;
    }
    const double far = std::min(maxRange, kPeakSearchOverlaps * c.overlap);
    double peak = 0.0;
    for (int i = 0; i <= kPeakSamples; ++i) {
        const double r = minRange + (far - minRange) * i / kPeakSamples;
        const double g = fogReturn(c, maxRange, fogExtinction, contamination, r);
        if (g > peak) {
            peak = g;
            out.range = r;
        }
    }
    out.probability = peak > 0.0 ? standardNormalCdf(std::log(peak) / c.speckle) : 0.0;
    return out;
}

double rainClutterProbability(const LidarClutter& c, double rainRateMps, double minRange) {
    if (rainRateMps <= 0.0 || c.rainRange <= minRange) {
        return 0.0;
    }
    const double slope = 4.1 * std::pow(rainRateMps / kMpsPerMmph, -0.21) * 1000.0;  // 1/m
    const double drops = kMarshallPalmerN0 / slope * std::exp(-slope * c.minDropDiameter);
    const double volume =
        std::numbers::pi * c.beamDiameter * c.beamDiameter / 4.0 * (c.rainRange - minRange);
    return 1.0 - std::exp(-drops * volume);
}

double weatherRange(double range, double minRange, double maxRange, const SensorWeather& w,
                    const LidarClutter& c, double u1, double u2, double u3, double fieldDepth) {
    const double reach = maxRange * std::clamp(1.0 - w.contamination, 0.0, 1.0);
    double out =
        std::isfinite(range) && range * std::exp(w.extinction * range + fieldDepth) <= reach
            ? range
            : kNoReturn;
    const Clutter fog = fogClutter(c, maxRange, w.fogExtinction, w.contamination, minRange);
    if (u1 < fog.probability) {
        // Somewhere about the peak of the fog's return.
        out = std::min(out, std::max(minRange, fog.range * (0.7 + 0.6 * u3)));
    }
    if (u2 < rainClutterProbability(c, w.rainRate, minRange)) {
        out = std::min(out, minRange + u3 * (c.rainRange - minRange));
    }
    return out;
}

void applyCameraWeather(std::span<std::uint8_t> rgba, std::span<const float> depth,
                        double extinction, double contamination,
                        std::span<const float> fieldDepth) {
    const double haze = std::clamp(contamination, 0.0, 1.0) / 2.0;
    if (extinction <= 0.0 && haze <= 0.0 && fieldDepth.empty()) {
        return;
    }
    const std::size_t pixels = std::min(rgba.size() / 4, depth.size());
    for (std::size_t p = 0; p < pixels; ++p) {
        const double d = depth[p];
        const double extra = p < fieldDepth.size() ? fieldDepth[p] : 0.0;
        const double transmission = std::isfinite(d) ? std::exp(-extinction * d - extra)
                                                     : (extinction > 0.0 ? 0.0 : std::exp(-extra));
        for (std::size_t k = 0; k < 3; ++k) {
            const double seen =
                rgba[4 * p + k] * transmission + kAirLight[k] * (1.0 - transmission);
            const double hazed = seen * (1.0 - haze) + kAirLight[k] * haze;
            rgba[4 * p + k] = static_cast<std::uint8_t>(std::clamp(std::lround(hazed), 0L, 255L));
        }
    }
}

}  // namespace lagrange::sensors
