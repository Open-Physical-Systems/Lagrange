#pragma once
// Private to lagrange_sensors: the measuring chain the power kinds share (sensors.md §9): a fixed
// gain error and offset, white noise per conversion, an average of N conversions, a converter's
// step (LSB) and its range. Quantisation rounds to the nearest step; outside the range it
// saturates. A missing input (NaN) reads NaN.
#include "KindModule.hpp"
#include "lagrange/random/Philox.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace lagrange::sensors::kinds {
// Its own inline namespace: the kinds' private headers may name helpers alike (ODR).
inline namespace converter {

inline constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
inline constexpr double kInf = std::numeric_limits<double>::infinity();

struct Converter {
    double gainError{0.0};     // fraction: reads (1 + gainError) × the truth
    double offset{0.0};        // in the measured unit
    double noise{0.0};         // 1σ of one conversion, in the measured unit
    std::uint32_t samples{1};  // conversions averaged into one reading
    double lsb{0.0};           // the step of a reading; 0: none
    double low{-kInf};         // the range: readings saturate here
    double high{kInf};

    /// To the nearest step, inside the range.
    [[nodiscard]] double quantise(double v) const {
        if (lsb > 0.0) {
            v = std::round(v / lsb) * lsb;
        }
        return std::clamp(v, low, high) + 0.0;  // + 0.0: never -0
    }

    /// One reading of `truth`: exact under ground truth; each conversion is quantised, their
    /// mean is quantised again (the result register).
    [[nodiscard]] double measure(double truth, random::Stream& rng, bool exact) const {
        if (exact || std::isnan(truth)) {
            return truth;
        }
        const double biased = truth * (1.0 + gainError) + offset;
        double sum = 0.0;
        const std::uint32_t n = std::max<std::uint32_t>(samples, 1);
        for (std::uint32_t i = 0; i < n; ++i) {
            sum += quantise(biased + (noise > 0.0 ? noise * rng.normal() : 0.0));
        }
        return quantise(sum / static_cast<double>(n));
    }
};

/// A converter from the sensor's parameters under `prefix` ("voltage/"): `gain_error`, `offset`,
/// `noise`, `samples`, `resolution` (the step itself) or `bits` with `full_scale` (step =
/// full_scale / 2^bits; range [0, full_scale], or ±full_scale with step full_scale / 2^(bits-1)
/// when `bipolar`), over the kind's defaults.
inline Converter converterFrom(const World& world, std::uint32_t sensor, const std::string& prefix,
                               Converter c, bool bipolar) {
    c.gainError = kindSetting(world, sensor, prefix + "gain_error", c.gainError);
    c.offset = kindSetting(world, sensor, prefix + "offset", c.offset);
    c.noise = std::max(0.0, kindSetting(world, sensor, prefix + "noise", c.noise));
    c.samples = static_cast<std::uint32_t>(
        std::clamp(kindSetting(world, sensor, prefix + "samples", c.samples), 1.0, 4096.0));
    const double fullScale = kindSetting(world, sensor, prefix + "full_scale", 0.0);
    const double bits = kindSetting(world, sensor, prefix + "bits", 0.0);
    if (fullScale > 0.0) {
        c.high = fullScale;
        c.low = bipolar ? -fullScale : 0.0;
        if (bits >= 1.0) {
            c.lsb = fullScale / std::exp2(std::round(bits) - (bipolar ? 1.0 : 0.0));
        }
    }
    c.lsb = std::max(0.0, kindSetting(world, sensor, prefix + "resolution", c.lsb));
    return c;
}

}  // namespace converter
}  // namespace lagrange::sensors::kinds
