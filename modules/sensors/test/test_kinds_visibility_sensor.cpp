// Kind "visibility_sensor" (sensors.md §9): the meteorological optical range by Koschmieder's law
// with WMO's 5 % threshold, against the weather's visibility at 2 %, and its averaging lag.
#include "kinds_test_util.hpp"
#include "lagrange/sensors/EnvironmentSensors.hpp"
#include "lagrange/sensors/WeatherEffects.hpp"
#include "smoke_box.hpp"

#include <cmath>

namespace lagrange::sensors {
namespace {

using namespace test;

TEST(VisibilitySensor, Koschmieder) {
    EXPECT_NEAR(meteorologicalOpticalRange(3e-3), std::log(20.0) / 3e-3, 1e-9);
    EXPECT_NEAR(meteorologicalOpticalRange(1.0), 2.9957, 1e-4);
    EXPECT_TRUE(std::isinf(meteorologicalOpticalRange(0.0)));
    // Fog the weather makes for a 2 % visibility V (σ = 3.912/V) has an MOR of ln 20/ln 50 · V.
    const double v = 200.0;
    EXPECT_NEAR(meteorologicalOpticalRange(-std::log(0.02) / v),
                std::log(20.0) / std::log(50.0) * v, 1e-9);
}

std::vector<Feed> fog(std::function<double(double)> sigma) {
    return {{makeSpec("env/extinction/visible", SignalType::Scalar, units::per_m),
             [sigma = std::move(sigma)](double t) { return std::vector<double>{sigma(t)}; }}};
}

TEST(VisibilitySensor, GroundTruthMor) {
    KindRun r;
    rig(r.world, "pwd", "visibility_sensor", Pose{}, 1.0);
    ASSERT_TRUE(start(r, fog([](double) { return 0.01; }), true));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{1'500'000'000}).ok());
    EXPECT_NEAR(r.read("sensor/pwd/visibility"), std::log(20.0) / 0.01, 1e-9);
    EXPECT_EQ(r.read("sensor/pwd/valid"), 1.0);
}

TEST(VisibilitySensor, ClearAirReadsItsLimit) {
    KindRun r;
    rig(r.world, "pwd", "visibility_sensor", Pose{}, 1.0);
    ASSERT_TRUE(start(r, fog([](double) { return 0.0; })));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{1'500'000'000}).ok());
    EXPECT_EQ(r.read("sensor/pwd/visibility"), 20000.0);
}

TEST(VisibilitySensor, FogRollsInThroughItsAveraging) {
    // Extinction steps from 1e-3 to 1e-2 /m at t = 10 s; τ = 60 s.
    KindRun r;
    rig(r.world, "pwd", "visibility_sensor", Pose{}, 1.0, {{"noise/fraction", 0.0}});
    ASSERT_TRUE(start(r, fog([](double t) { return t < 10.0 ? 1e-3 : 1e-2; })));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{70'500'000'000}).ok());
    const double low = 1e-3 + 9e-3 * (1.0 - std::exp(-59.0 / 60.0));
    const double high = 1e-3 + 9e-3 * (1.0 - std::exp(-61.0 / 60.0));
    const double sigma = r.read("sensor/pwd/extinction");
    EXPECT_GE(sigma, low);
    EXPECT_LE(sigma, high);
    EXPECT_NEAR(r.read("sensor/pwd/visibility"), std::log(20.0) / sigma, 1e-6);
}

TEST(VisibilitySensor, NoWeather) {
    KindRun r;
    rig(r.world, "pwd", "visibility_sensor", Pose{}, 1.0);
    ASSERT_TRUE(start(r, {}));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{1'500'000'000}).ok());
    EXPECT_EQ(r.read("sensor/pwd/valid"), 0.0);
    EXPECT_TRUE(std::isnan(r.read("sensor/pwd/visibility")));
}

// In the fires' smoke (RFC 0022 §4, §Validation "a uniform smoke box: visibility_sensor reads
// 3/σ"): σ at the sensor, added to the weather's; with no weather, smoke alone. The sign
// visibilities are C/σ, C = 3 (reflecting) and 8 (emitting), 1 %.
TEST(VisibilitySensor, ReadsTheSmokeAtItsPlace) {
    for (const bool weather : {false, true}) {
        KindRun r;
        rig(r.world, "pwd", "visibility_sensor", Pose{{0.0, 0.0, 1.0}}, 1.0);
        const double smoke = 0.4;   // 1/m
        const double haze = 0.002;  // 1/m from the weather
        std::vector<Feed> feeds = bodyFeeds(Pose{}, {});
        if (weather) {
            feeds.push_back(fog([haze](double) { return haze; }).front());
        }
        r.scheduler = std::make_unique<Scheduler>(SchedulerOptions{.deterministic = true});
        r.scheduler->add(std::make_unique<Source>(std::move(feeds)));
        r.scheduler->add(std::make_unique<SmokeBox>(
            fields::DenseBox{{-2.0, -2.0, -1.0}, 0.25, {17, 17, 17}}, smoke));
        auto module = createBuiltinKind(r.world, 0, true, nullptr, nullptr);
        ASSERT_TRUE(module);
        r.scheduler->add(std::move(module), smokeConfig());
        auto wired = r.scheduler->wire(WorldView(r.world));
        ASSERT_TRUE(wired) << to_string(wired.error().front());
        ASSERT_TRUE(r.scheduler->stepUntil(SimTime{1'500'000'000}).ok());
        const double sigma = smoke + (weather ? haze : 0.0);
        EXPECT_NEAR(r.read("sensor/pwd/extinction"), sigma, 1e-6 * sigma);
        EXPECT_EQ(r.read("sensor/pwd/valid"), 1.0);
        EXPECT_NEAR(r.read("sensor/pwd/visibility"), std::log(20.0) / sigma, 0.01 / sigma);
        EXPECT_NEAR(r.read("sensor/pwd/sign_visibility"), 3.0 / sigma, 0.01 * 3.0 / sigma);
        EXPECT_NEAR(r.read("sensor/pwd/light_visibility"), 8.0 / sigma, 0.01 * 8.0 / sigma);
    }
}

// Dense smoke under its 10 m floor: the MOR is held there, the signs' visibility is not (a real
// meter, no noise, no lag): 3/σ and 8/σ at σ = 1/m.
TEST(VisibilitySensor, SignsInDenseSmokeAreBelowItsRange) {
    KindRun r;
    rig(r.world, "pwd", "visibility_sensor", Pose{{0.0, 0.0, 1.0}}, 1.0,
        {{"noise/fraction", 0.0}, {"time_constant", 0.0}});
    r.scheduler = std::make_unique<Scheduler>(SchedulerOptions{.deterministic = true});
    r.scheduler->add(std::make_unique<Source>(bodyFeeds(Pose{}, {})));
    r.scheduler->add(
        std::make_unique<SmokeBox>(fields::DenseBox{{-2.0, -2.0, -1.0}, 0.25, {17, 17, 17}}, 1.0));
    r.scheduler->add(createBuiltinKind(r.world, 0, false, nullptr, nullptr), smokeConfig());
    ASSERT_TRUE(r.scheduler->wire(WorldView(r.world)));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{1'500'000'000}).ok());
    EXPECT_EQ(r.read("sensor/pwd/visibility"), 10.0);
    EXPECT_NEAR(r.read("sensor/pwd/sign_visibility"), 3.0, 0.01 * 3.0);
    EXPECT_NEAR(r.read("sensor/pwd/light_visibility"), 8.0, 0.01 * 8.0);
}

}  // namespace
}  // namespace lagrange::sensors
