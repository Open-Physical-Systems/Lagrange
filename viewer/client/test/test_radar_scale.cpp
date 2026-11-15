#include "lagrange/viewer/RadarScale.hpp"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

namespace lagrange::viewer {
namespace {

constexpr float kTolerance = 1e-6F;

void expectColour(const std::array<float, 4>& colour, std::uint32_t hex) {
    EXPECT_NEAR(colour[0], static_cast<float>((hex >> 16U) & 0xFFU) / 255.0F, kTolerance);
    EXPECT_NEAR(colour[1], static_cast<float>((hex >> 8U) & 0xFFU) / 255.0F, kTolerance);
    EXPECT_NEAR(colour[2], static_cast<float>(hex & 0xFFU) / 255.0F, kTolerance);
    EXPECT_FLOAT_EQ(colour[3], 1.0F);
}

TEST(RadarScale, VelocityColourRunsFromApproachingThroughRestToReceding) {
    expectColour(radarVelocityColour(-10.0F, 10.0F), 0x2F6FED);  // approaching: blue
    expectColour(radarVelocityColour(0.0F, 10.0F), 0x8190A6);    // at rest: t3
    expectColour(radarVelocityColour(10.0F, 10.0F), 0xE8772E);   // receding: orange
    // Beyond the span: clamped to the ends.
    expectColour(radarVelocityColour(-80.0F, 10.0F), 0x2F6FED);
    expectColour(radarVelocityColour(80.0F, 10.0F), 0xE8772E);
    // Halfway: halfway between rest and the end, channel by channel.
    const std::array<float, 4> half = radarVelocityColour(5.0F, 10.0F);
    EXPECT_NEAR(half[0], (0x81 + 0xE8) / 2.0F / 255.0F, kTolerance);
    EXPECT_NEAR(half[2], (0xA6 + 0x2E) / 2.0F / 255.0F, kTolerance);
    // Nothing to scale by, or no velocity: the middle.
    expectColour(radarVelocityColour(std::numeric_limits<float>::quiet_NaN(), 10.0F), 0x8190A6);
    expectColour(radarVelocityColour(3.0F, 0.0F), 0x8190A6);
}

TEST(RadarScale, VelocityColourIsMonotonicTowardEachEnd) {
    // Receding gets steadily more red and less blue; approaching steadily more blue.
    float red = 0.0F;
    float blue = 1.0F;
    for (float v = 0.0F; v <= 10.0F; v += 0.5F) {
        const std::array<float, 4> c = radarVelocityColour(v, 10.0F);
        EXPECT_GE(c[0], red - kTolerance);
        EXPECT_LE(c[2], blue + kTolerance);
        red = c[0];
        blue = c[2];
    }
    blue = 0.0F;
    for (float v = 0.0F; v >= -10.0F; v -= 0.5F) {
        const std::array<float, 4> c = radarVelocityColour(v, 10.0F);
        EXPECT_GE(c[2], blue - kTolerance);
        blue = c[2];
    }
}

TEST(RadarScale, BinsCoverTheSpanAndHoldZeroInTheMiddle) {
    constexpr int kBins = 11;
    EXPECT_EQ(radarVelocityBin(-10.0F, 10.0F, kBins), 0);
    EXPECT_EQ(radarVelocityBin(-1e9F, 10.0F, kBins), 0);
    EXPECT_EQ(radarVelocityBin(10.0F, 10.0F, kBins), kBins - 1);
    EXPECT_EQ(radarVelocityBin(1e9F, 10.0F, kBins), kBins - 1);
    EXPECT_EQ(radarVelocityBin(0.0F, 10.0F, kBins), kBins / 2);
    EXPECT_EQ(radarVelocityBin(std::numeric_limits<float>::quiet_NaN(), 10.0F, kBins), kBins / 2);
    EXPECT_FLOAT_EQ(radarBinVelocity(kBins / 2, kBins, 10.0F), 0.0F);
    // Every band's centre falls in that band, and the centres climb.
    float last = -1e9F;
    for (int b = 0; b < kBins; ++b) {
        const float v = radarBinVelocity(b, kBins, 10.0F);
        EXPECT_EQ(radarVelocityBin(v, 10.0F, kBins), b);
        EXPECT_GT(v, last);
        last = v;
    }
}

TEST(RadarScale, SpanRoundsUpToOneTwoOrFive) {
    EXPECT_FLOAT_EQ(radarVelocitySpan(0.0F), kRadarMinimumSpan);
    EXPECT_FLOAT_EQ(radarVelocitySpan(0.3F), kRadarMinimumSpan);
    EXPECT_FLOAT_EQ(radarVelocitySpan(2.0F), 2.0F);
    EXPECT_FLOAT_EQ(radarVelocitySpan(2.1F), 5.0F);
    EXPECT_FLOAT_EQ(radarVelocitySpan(7.0F), 10.0F);
    EXPECT_FLOAT_EQ(radarVelocitySpan(10.0F), 10.0F);
    EXPECT_FLOAT_EQ(radarVelocitySpan(13.9F), 20.0F);
    EXPECT_FLOAT_EQ(radarVelocitySpan(48.7F), 50.0F);
    EXPECT_FLOAT_EQ(radarVelocitySpan(51.0F), 100.0F);
    EXPECT_FLOAT_EQ(radarVelocitySpan(std::numeric_limits<float>::infinity()), kRadarMinimumSpan);
}

TEST(RadarScale, RcsSizeIsLinearInDecibels) {
    EXPECT_FLOAT_EQ(radarRcsFraction(0.01F), 0.0F);   // −20 dBsm
    EXPECT_FLOAT_EQ(radarRcsFraction(1.0F), 0.5F);    // 0 dBsm
    EXPECT_FLOAT_EQ(radarRcsFraction(100.0F), 1.0F);  // +20 dBsm
    EXPECT_NEAR(radarRcsFraction(10.0F), 0.75F, 1e-6F);
    EXPECT_FLOAT_EQ(radarRcsFraction(1e-6F), 0.0F);  // clamped
    EXPECT_FLOAT_EQ(radarRcsFraction(1e6F), 1.0F);
    EXPECT_FLOAT_EQ(radarRcsFraction(0.0F), 0.0F);
    EXPECT_FLOAT_EQ(radarRcsFraction(-1.0F), 0.0F);
    EXPECT_FLOAT_EQ(radarRcsFraction(std::numeric_limits<float>::quiet_NaN()), 0.0F);
}

}  // namespace
}  // namespace lagrange::viewer
