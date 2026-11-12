// Kind "humidity_sensor" (sensors.md §9): the weather's humidity and temperature through the
// SHT4x's first-order response (τ63 4 s and 2 s), and NaN without the weather.
#include "kinds_test_util.hpp"
#include "lagrange/sensors/EnvironmentSensors.hpp"

#include <cmath>

namespace lagrange::sensors {
namespace {

using namespace test;

TEST(HumiditySensor, FirstOrderLagIsExact) {
    // y(t) = u·(1 − e^(−t/τ)) for a step from 0, whatever the step size.
    double y = 0.0;
    for (int k = 0; k < 40; ++k) {
        y = firstOrderStep(y, 1.0, 0.1, 4.0);
    }
    EXPECT_NEAR(y, 1.0 - std::exp(-1.0), 1e-12);
    EXPECT_EQ(firstOrderStep(0.3, 0.7, 0.1, 0.0), 0.7);
}

std::vector<Feed> weather() {
    return {{makeSpec("env/relative_humidity", SignalType::Scalar, units::one),
             [](double t) { return std::vector<double>{t < 1.0 ? 0.5 : 0.8}; }},
            {makeSpec("env/air_temperature", SignalType::Scalar, units::K),
             [](double t) { return std::vector<double>{t < 1.0 ? 290.0 : 300.0}; }}};
}

TEST(HumiditySensor, AStepThroughItsResponseTime) {
    KindRun r;
    rig(r.world, "sht", "humidity_sensor", Pose{}, 10.0,
        {{"noise/humidity", 0.0}, {"noise/temperature", 0.0}});
    ASSERT_TRUE(start(r, weather()));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{900'000'000}).ok());
    EXPECT_NEAR(r.read("sensor/sht/relative_humidity"), 0.5, 1e-12);
    // One time constant of each after the step; sampled every 0.1 s, the step seen at most one
    // sample late.
    for (const double after : {2.0, 4.0}) {
        const auto at = static_cast<std::int64_t>(std::llround((1.0 + after) * 1e9));
        ASSERT_TRUE(r.scheduler->stepUntil(SimTime{at}).ok());
        const double rh = r.read("sensor/sht/relative_humidity");
        const double t = r.read("sensor/sht/temperature");
        EXPECT_NEAR(rh, 0.5 + 0.3 * (1.0 - std::exp(-after / 4.0)), 0.3 * 0.1 / 4.0 + 1e-4);
        EXPECT_NEAR(t, 290.0 + 10.0 * (1.0 - std::exp(-after / 2.0)), 10.0 * 0.1 / 2.0 + 0.01);
    }
}

TEST(HumiditySensor, GroundTruthIsTheAir) {
    KindRun r;
    rig(r.world, "sht", "humidity_sensor", Pose{}, 10.0);
    ASSERT_TRUE(start(r, weather(), true));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{1'500'000'000}).ok());
    EXPECT_EQ(r.read("sensor/sht/relative_humidity"), 0.8);
    EXPECT_EQ(r.read("sensor/sht/temperature"), 300.0);
}

TEST(HumiditySensor, WithoutTheWeatherOnlyTheSiteTemperature) {
    KindRun r;
    rig(r.world, "sht", "humidity_sensor", Pose{}, 10.0);
    ModuleConfigData c;
    c.values["ambient_temperature"] = 293.15;
    ASSERT_TRUE(start(r, {}, false, c));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{500'000'000}).ok());
    EXPECT_EQ(r.read("sensor/sht/valid"), 0.0);
    EXPECT_TRUE(std::isnan(r.read("sensor/sht/relative_humidity")));
    EXPECT_NEAR(r.read("sensor/sht/temperature"), 293.15, 0.2);
}

}  // namespace
}  // namespace lagrange::sensors
