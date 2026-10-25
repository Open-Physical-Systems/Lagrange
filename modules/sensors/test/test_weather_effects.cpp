// What the weather does to the lidar and the camera (WeatherEffects.hpp, weather.md §7): the
// lidar equation's detection range, clutter's trend with fog and rain, and the camera's veil
// (Koschmieder's contrast).
#include "lagrange/sensors/WeatherEffects.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::sensors {
namespace {

constexpr double kMmph = 1e-3 / 3600.0;  // m/s per mm/h

TEST(LidarWeather, DetectionRangeSolvesTheLidarEquation) {
    EXPECT_EQ(lidarDetectionRange(30.0, 0.0, 0.0), 30.0);  // clear air, clean window
    EXPECT_EQ(lidarDetectionRange(30.0, 0.0, 0.5), 15.0);  // half the light out: half the range
    for (const double sigma : {0.003, 0.02, 0.1}) {
        const double r = lidarDetectionRange(30.0, sigma, 0.2);
        // Received power ∝ T²·exp(−2σr)/r² meets the threshold R⁻²: r·exp(σr) = R·T.
        EXPECT_NEAR(r * std::exp(sigma * r), 30.0 * 0.8, 1e-9) << sigma;
        EXPECT_LT(r, 24.0);
    }
    // Fog of 50 m visibility (σ = 3.912/50): a 100 m lidar sees about 20 m.
    EXPECT_NEAR(lidarDetectionRange(100.0, 3.912 / 50.0, 0.0), 20.35, 0.01);
}

TEST(LidarWeather, FogClutterAppearsOnlyInDenseFog) {
    const LidarClutter c;
    const auto probability = [&](double visibility) {
        return fogClutter(c, 30.0, 3.912 / visibility, 0.0, 0.1).probability;
    };
    EXPECT_LT(probability(1000.0), 1e-3);
    EXPECT_LT(probability(200.0), 0.05);
    EXPECT_NEAR(probability(60.0), 0.5, 0.2);  // the gain's design point
    EXPECT_GT(probability(30.0), 0.75);
    // Near the sensor, where the receiver begins to see the beam.
    const Clutter dense = fogClutter(c, 30.0, 3.912 / 30.0, 0.0, 0.1);
    EXPECT_GT(dense.range, 0.5);
    EXPECT_LT(dense.range, 4.0);
}

TEST(LidarWeather, RainClutterGrowsWithTheRainRate) {
    const LidarClutter c;
    const double light = rainClutterProbability(c, 2.0 * kMmph, 0.1);
    const double heavy = rainClutterProbability(c, 25.0 * kMmph, 0.1);
    EXPECT_GT(heavy, light);
    // Order of magnitude (weather.md §7): about 1 % of beams in 2 mm/h, 5 % in 25 mm/h.
    EXPECT_NEAR(light, 0.014, 0.005);
    EXPECT_NEAR(heavy, 0.05, 0.015);
    EXPECT_EQ(rainClutterProbability(c, 0.0, 0.1), 0.0);
}

TEST(LidarWeather, ABeamInWeather) {
    const LidarClutter c;
    const double none = std::numeric_limits<double>::infinity();
    // Clear: unchanged.
    EXPECT_EQ(weatherRange(12.0, 0.1, 30.0, {}, c, 0.0, 0.0, 0.5), 12.0);
    EXPECT_EQ(weatherRange(none, 0.1, 30.0, {}, c, 0.0, 0.0, 0.5), none);
    // A target beyond the range the fog leaves: no return.
    const SensorWeather fog{3.912 / 40.0, 3.912 / 40.0, 0.0, 0.0};
    EXPECT_EQ(weatherRange(25.0, 0.1, 30.0, fog, c, 0.99, 0.99, 0.5), none);
    // Heavy rain, a beam whose draw falls in: a drop's return within 2 m.
    const SensorWeather rain{0.003, 0.0, 25.0 * kMmph, 0.0};
    const double r = weatherRange(10.0, 0.1, 30.0, rain, c, 0.99, 0.0, 0.5);
    EXPECT_GE(r, 0.1);
    EXPECT_LE(r, 2.0);
}

TEST(CameraWeather, TheVeilIsKoschmieders) {
    // A black target at the visibility shows 2 % of the contrast it has against the air light.
    const double sigma = 3.912 / 100.0;
    std::vector<std::uint8_t> rgba{0, 0, 0, 255, 0, 0, 0, 255};
    const std::vector<float> depth{100.0F, std::numeric_limits<float>::infinity()};
    applyCameraWeather(rgba, depth, sigma, 0.0);
    for (std::size_t k = 0; k < 3; ++k) {
        EXPECT_NEAR(rgba[k], 0.98 * kAirLight[k], 1.0) << k;
        EXPECT_EQ(rgba[4 + k], kAirLight[k]);  // the sky: all air light
    }
    EXPECT_EQ(rgba[3], 255);  // alpha untouched
}

}  // namespace
}  // namespace lagrange::sensors
