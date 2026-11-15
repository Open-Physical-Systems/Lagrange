#include "lagrange/viewer/RadarScale.hpp"

#include <algorithm>
#include <cmath>

namespace lagrange::viewer {

namespace {

constexpr std::array<float, 3> rgb(std::uint32_t hex) {
    return {static_cast<float>((hex >> 16U) & 0xFFU) / 255.0F,
            static_cast<float>((hex >> 8U) & 0xFFU) / 255.0F,
            static_cast<float>(hex & 0xFFU) / 255.0F};
}

// design-tokens.json colormap.error's ends, and t3 for its middle (RadarScale.hpp).
constexpr std::array<float, 3> kApproaching = rgb(0x2F6FED);
constexpr std::array<float, 3> kAtRest = rgb(0x8190A6);
constexpr std::array<float, 3> kReceding = rgb(0xE8772E);

constexpr float kLowestDbsm = -20.0F;  // dBsm at the small end of the size scale
constexpr float kDbsmSpan = 40.0F;     // dB from the small end to the large end

}  // namespace

std::array<float, 4> radarVelocityColour(float velocity, float span) {
    if (!std::isfinite(velocity) || !(span > 0.0F)) {
        return {kAtRest[0], kAtRest[1], kAtRest[2], 1.0F};
    }
    const float t = std::clamp(velocity / span, -1.0F, 1.0F);
    const std::array<float, 3>& end = t < 0.0F ? kApproaching : kReceding;
    const float blend = std::abs(t);
    return {kAtRest[0] + (end[0] - kAtRest[0]) * blend, kAtRest[1] + (end[1] - kAtRest[1]) * blend,
            kAtRest[2] + (end[2] - kAtRest[2]) * blend, 1.0F};
}

int radarVelocityBin(float velocity, float span, int bins) {
    if (bins <= 1) {
        return 0;
    }
    if (!std::isfinite(velocity) || !(span > 0.0F)) {
        return bins / 2;
    }
    const float t = (std::clamp(velocity / span, -1.0F, 1.0F) + 1.0F) * 0.5F;  // 0..1
    return std::clamp(static_cast<int>(t * static_cast<float>(bins)), 0, bins - 1);
}

float radarBinVelocity(int bin, int bins, float span) {
    if (bins <= 1) {
        return 0.0F;
    }
    const float t = (static_cast<float>(bin) + 0.5F) / static_cast<float>(bins);
    return (t * 2.0F - 1.0F) * span;
}

float radarVelocitySpan(float fastest) {
    if (!std::isfinite(fastest) || fastest <= kRadarMinimumSpan) {
        return kRadarMinimumSpan;
    }
    const float decade = std::pow(10.0F, std::floor(std::log10(fastest)));
    for (const float step : {1.0F, 2.0F, 5.0F, 10.0F}) {
        if (fastest <= step * decade * (1.0F + 1e-6F)) {
            return step * decade;
        }
    }
    return 10.0F * decade;
}

float radarRcsFraction(float rcs) {
    if (!std::isfinite(rcs) || rcs <= 0.0F) {
        return 0.0F;
    }
    const float dbsm = 10.0F * std::log10(rcs);
    return std::clamp((dbsm - kLowestDbsm) / kDbsmSpan, 0.0F, 1.0F);
}

}  // namespace lagrange::viewer
