// The thermal false-colour map: the design tokens' colormap.thermal ("inferno"), as linear RGB.
#include "lagrange/viewer/RenderOptions.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace lagrange::viewer {
namespace {

float srgb(float linear) {
    return std::pow(linear, 1.0F / 2.2F) * 255.0F;
}

TEST(ThermalColour, EndsAndMiddleAreTheTokensStops) {
    const auto at = [](float t, int r, int g, int b) {
        const std::array<float, 3> c = thermalColor(t);
        EXPECT_NEAR(srgb(c[0]), static_cast<float>(r), 0.5F) << t;
        EXPECT_NEAR(srgb(c[1]), static_cast<float>(g), 0.5F) << t;
        EXPECT_NEAR(srgb(c[2]), static_cast<float>(b), 0.5F) << t;
    };
    at(0.0F, 0x00, 0x00, 0x04);
    at(0.6F, 0xDD, 0x51, 0x3A);  // the fourth of six stops
    at(1.0F, 0xFC, 0xFF, 0xA4);
    at(-1.0F, 0x00, 0x00, 0x04);  // clamped
    at(2.0F, 0xFC, 0xFF, 0xA4);
}

TEST(ThermalColour, BrightnessRisesWithTemperature) {
    float previous = -1.0F;
    for (int k = 0; k <= 20; ++k) {
        const std::array<float, 3> c = thermalColor(static_cast<float>(k) / 20.0F);
        const float luminance = 0.2126F * c[0] + 0.7152F * c[1] + 0.0722F * c[2];
        EXPECT_GT(luminance, previous) << k;
        previous = luminance;
    }
}

}  // namespace
}  // namespace lagrange::viewer
