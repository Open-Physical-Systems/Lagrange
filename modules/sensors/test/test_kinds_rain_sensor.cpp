// Kind "rain_sensor" (sensors.md §9): the weather's rain rate, the rain fallen (∫R dt) on the
// gauge's 0.2 mm steps, and nothing without the weather.
#include "kinds_test_util.hpp"

#include <cmath>

namespace lagrange::sensors {
namespace {

using namespace test;

constexpr double kMmPerHour = 1e-3 / 3600.0;  // m/s

std::vector<Feed> rain(double rate) {
    return {constant("env/rain_rate", SignalType::Scalar, units::m_per_s, {rate})};
}

TEST(RainSensor, GroundTruthIntegratesTheRate) {
    KindRun r;
    rig(r.world, "gauge", "rain_sensor", Pose{}, 10.0);
    ASSERT_TRUE(start(r, rain(36.0 * kMmPerHour), true));  // 36 mm/h: 0.01 mm/s
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{10'050'000'000}).ok());
    EXPECT_NEAR(r.read("sensor/gauge/rain_rate"), 36.0 * kMmPerHour, 1e-15);
    // 100 samples of 0.1 s each, up to t = 10 s.
    EXPECT_NEAR(r.read("sensor/gauge/accumulation"), 0.01e-3 * 10.0, 1e-12);
    EXPECT_EQ(r.read("sensor/gauge/raining"), 1.0);
}

TEST(RainSensor, TheGaugeCountsInSteps) {
    KindRun r;
    rig(r.world, "gauge", "rain_sensor", Pose{}, 10.0, {{"noise/gain", 0.0}});
    ASSERT_TRUE(start(r, rain(36.0 * kMmPerHour)));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{30'050'000'000}).ok());
    // 0.303 mm fallen shows as one 0.2 mm step.
    EXPECT_NEAR(r.read("sensor/gauge/accumulation"), 0.2e-3, 1e-12);
}

TEST(RainSensor, GainErrorWithinTheRG15sTenPercent) {
    KindRun r;
    rig(r.world, "gauge", "rain_sensor", Pose{}, 10.0);
    ASSERT_TRUE(start(r, rain(10.0 * kMmPerHour)));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{1'000'000'000}).ok());
    EXPECT_NEAR(r.read("sensor/gauge/rain_rate"), 10.0 * kMmPerHour, 0.2 * 10.0 * kMmPerHour);
}

TEST(RainSensor, DryAndWithoutWeather) {
    KindRun dry;
    rig(dry.world, "gauge", "rain_sensor", Pose{}, 10.0);
    ASSERT_TRUE(start(dry, rain(0.0)));
    ASSERT_TRUE(dry.scheduler->stepUntil(SimTime{500'000'000}).ok());
    EXPECT_EQ(dry.read("sensor/gauge/raining"), 0.0);
    EXPECT_EQ(dry.read("sensor/gauge/valid"), 1.0);
    KindRun none;
    rig(none.world, "gauge", "rain_sensor", Pose{}, 10.0);
    ASSERT_TRUE(start(none, {}));
    ASSERT_TRUE(none.scheduler->stepUntil(SimTime{500'000'000}).ok());
    EXPECT_EQ(none.read("sensor/gauge/valid"), 0.0);
    EXPECT_TRUE(std::isnan(none.read("sensor/gauge/rain_rate")));
}

}  // namespace
}  // namespace lagrange::sensors
