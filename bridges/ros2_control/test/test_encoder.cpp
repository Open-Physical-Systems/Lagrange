// Roadmap 0.2-D2: the encoder model behind <param name="encoder_resolution">.
#include "lagrange/ros2_control/Encoder.hpp"

#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

namespace lagrange::ros2_control {
namespace {

constexpr double kTwoPi = 2.0 * std::numbers::pi;

TEST(Encoder, CountsWholeStepsRoundedDownLikeACounter) {
    const Encoder encoder(16384);
    EXPECT_DOUBLE_EQ(encoder.step(), kTwoPi / 16384.0);
    EXPECT_EQ(encoder.count(0.0), 0);
    EXPECT_EQ(encoder.count(kTwoPi), 16384);
    EXPECT_EQ(encoder.count(0.999 * encoder.step()), 0);
    EXPECT_EQ(encoder.count(-0.001 * encoder.step()), -1);  // a counter has no "minus zero"
    // The reported angle is never ahead of the true one, and at most one count behind.
    for (double angle = -20.0; angle < 20.0; angle += 0.0137) {
        const double reported = encoder.position(angle);
        EXPECT_LE(reported, angle);
        EXPECT_GT(reported, angle - encoder.step() - 1e-12);
    }
}

TEST(Encoder, VelocityIsTheCountChangeOverThePeriod) {
    Encoder encoder(1024);
    const double period = 0.01;  // s, a 100 Hz controller
    const double omega = 3.7;    // rad/s
    EXPECT_EQ(encoder.velocity(0.0, period), 0.0) << "nothing to difference on the first read";
    double sum = 0.0;
    const int steps = 1000;
    for (int k = 1; k <= steps; ++k) {
        const double v = encoder.velocity(omega * period * k, period);
        // Each reading is a whole number of counts per period.
        const double counts = v * period / encoder.step();
        EXPECT_NEAR(counts, std::round(counts), 1e-9);
        // ...within one count per period of the truth.
        EXPECT_LE(std::abs(v - omega), encoder.step() / period + 1e-9);
        sum += v;
    }
    // Over 10 s the counts add up: the mean is exact to one count over the whole run.
    EXPECT_NEAR(sum / steps, omega, encoder.step() / (period * steps) + 1e-9);
}

TEST(Encoder, AZeroPeriodGivesZeroVelocity) {
    Encoder encoder(4096);
    (void)encoder.velocity(0.0, 0.01);
    EXPECT_EQ(encoder.velocity(1.0, 0.0), 0.0);
}

}  // namespace
TEST(Encoder, InterferenceAddsOrDropsCountsPerEdge) {
    Encoder encoder(1024);
    const double step = encoder.step();
    (void)encoder.velocity(0.5 * step, 0.001);  // the first reading: no edges yet
    // Ten edges at a probability of 1, every draw 0: each edge gains a count, and they stay.
    encoder.glitch(10.5 * step, 1.0, [] { return 0.0; });
    EXPECT_EQ(encoder.glitches(), 10);
    EXPECT_NEAR(encoder.position(10.5 * step), 20.0 * step, 1e-12);
    EXPECT_NEAR(encoder.velocity(10.5 * step, 0.001), 20.0 * step / 0.001, 1e-6);
    // No interference: no change, however far it turns.
    encoder.glitch(100.5 * step, 0.0, [] { return 0.0; });
    EXPECT_EQ(encoder.glitches(), 10);
}

}  // namespace lagrange::ros2_control
