// The thermal false-colour map: the design tokens' colormap.thermal ("inferno"), as linear RGB.
#include "lagrange/viewer/RenderOptions.hpp"

#include <cmath>
#include <vector>

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

// RFC 0012: a link's cells as the faces of its outside, each coloured by its own cell.
TEST(HeatFaces, OnlyTheOutsideFacesOfTheSolidCellsAreDrawn) {
    // 3 × 2 × 1 cells, the middle one of the first row outside the solid.
    const std::vector<float> values{300.0F, NAN, 320.0F, 310.0F, 310.0F, 310.0F};
    std::vector<float> out;
    heatFaces(3, 2, 1, {0.0, 0.0, 0.0}, 0.01, values, 300.0F, 320.0F, out);
    // Five cells; faces: each has top and bottom (10), and the sides that touch no solid
    // neighbour: count by hand: (0,0): -x,+x,-y = 3; (2,0): -x,+x,-y = 3; (0,1): -x,+y = 2;
    // (1,1): -y,+y = 2; (2,1): +x,+y = 2. Total 10 + 12 = 22 faces, 6 vertices each.
    ASSERT_EQ(out.size(), 22U * 6U * 6U);
    // The hottest cell's top face is the map's top colour, unshaded.
    const std::array<float, 3> top = thermalColor(1.0F);
    bool seen = false;
    for (std::size_t v = 0; v < out.size(); v += 6) {
        if (out[v] > 0.019F && out[v + 2] > 0.004F && out[v + 3] == top[0] &&
            out[v + 4] == top[1]) {
            seen = true;
        }
    }
    EXPECT_TRUE(seen);
    // A wrong size draws nothing.
    std::vector<float> none;
    heatFaces(2, 2, 2, {0.0, 0.0, 0.0}, 0.01, values, 300.0F, 320.0F, none);
    EXPECT_TRUE(none.empty());
}

TEST(WeatherLook, FallingWeatherBringsItsSeasonOnlyOverSummer) {
    RenderOptions o;
    EXPECT_EQ(effectiveSeason(o), Season::Summer);
    o.rainMmPerH = 25.0F;
    EXPECT_EQ(effectiveSeason(o), Season::Rainy);
    o.snowMmPerH = 1.0F;  // sleet: snow wins, the ground whitens
    EXPECT_EQ(effectiveSeason(o), Season::Winter);
    o = RenderOptions{};
    o.snowCover = 1.0F;  // snow lying after it stopped
    EXPECT_EQ(effectiveSeason(o), Season::Winter);
    o.season = Season::Fall;  // a season picked by hand stays
    EXPECT_EQ(effectiveSeason(o), Season::Fall);
}

}  // namespace
}  // namespace lagrange::viewer
