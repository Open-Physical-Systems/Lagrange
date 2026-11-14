// particle_sensor (sensors.md §9): the PM classes of monodisperse dust, the first-order lag in
// closed form, the SPS30's precision as noise, saturation, and NaN without the weather domain.
#include "more_kinds_util.hpp"

#include <cmath>

namespace lagrange::sensors::test {
namespace {

constexpr double kUg = 1e-9;  // kg/m³ per µg/m³

/// A sensor at 1 Hz (the SPS30's interval) in air whose dust is `dust(t)` kg/m³ of `diameter`.
KindRun pmRun(std::map<std::string, double, std::less<>> settings, std::unique_ptr<Writer> w,
              bool groundTruth = false) {
    KindRun r;
    const BodyId robot = movingBody(r.world, "robot");
    r.world.addCustomSensor("pm", "particle_sensor", robot, Pose{}, 1.0, std::move(settings), {});
    r.wire(std::move(w), groundTruth);
    return r;
}

std::unique_ptr<Writer> air(std::function<double(double)> dust, double diameter,
                            double lifted = 0.0) {
    auto w = std::make_unique<Writer>();
    w->add("env/dust/concentration", SignalType::Scalar, units::kg_per_m3,
           [dust](double t) { return std::vector<double>{dust(t)}; });
    w->add("env/dust/median_diameter", SignalType::Scalar, units::m,
           [diameter](double) { return std::vector<double>{diameter}; });
    w->add("env/dust/downwash", SignalType::Scalar, units::kg_per_m3,
           [lifted](double) { return std::vector<double>{lifted}; });
    return w;
}

TEST(ParticleSensor, GrainsCountInTheClassesTheyFit) {
    // 2 µm grains are PM2.5, PM4 and PM10, not PM1; 100 µm lifted grains are none of them.
    KindRun r = pmRun({}, air([](double) { return 200.0 * kUg; }, 2e-6, 1e-6), true);
    ASSERT_TRUE(r.runTo(3.0));
    EXPECT_EQ(r.read("sensor/pm/pm1"), 0.0);
    EXPECT_EQ(r.read("sensor/pm/pm2_5"), 200.0 * kUg);
    EXPECT_EQ(r.read("sensor/pm/pm4"), 200.0 * kUg);
    EXPECT_EQ(r.read("sensor/pm/pm10"), 200.0 * kUg);
    EXPECT_EQ(r.read("sensor/pm/valid"), 1.0);
}

TEST(ParticleSensor, LiftedDustOfItsSize) {
    KindRun r = pmRun({{"downwash_diameter", 5e-6}},
                      air([](double) { return 0.0; }, 2e-6, 300.0 * kUg), true);
    ASSERT_TRUE(r.runTo(3.0));
    EXPECT_EQ(r.read("sensor/pm/pm2_5"), 0.0);
    EXPECT_EQ(r.read("sensor/pm/pm10"), 300.0 * kUg);
}

TEST(ParticleSensor, FollowsAStepWithItsTimeConstant) {
    // 500 µg/m³ from t = 10 s; τ = 8 s: 1 − e^(−t/τ) of it at whole seconds after.
    KindRun r = pmRun({{"noise_scale", 0.0}},
                      air([](double t) { return t < 10.0 ? 0.0 : 500.0 * kUg; }, 2e-6));
    for (int k = 12; k <= 40; k += 4) {
        ASSERT_TRUE(r.runTo(k + 0.5));
        const double after = r.read("sensor/pm/pm2_5");
        // The step reaches the sensor at its first sample at or after 10 s.
        const double expected = 500.0 * kUg * -std::expm1(-(k - 10 + 1) / 8.0);
        EXPECT_NEAR(after, expected, 1e-6 * 500.0 * kUg) << k;
    }
}

TEST(ParticleSensor, ThePrecisionOfTheDatasheet) {
    // 50 µg/m³: PM2.5 ±(5 µg/m³ + 5 %) = ±7.5 µg/m³ (SPS30 datasheet v2.0), PM10 ±25 µg/m³ (1σ).
    KindRun r = pmRun({{"time_constant", 0.0}}, air([](double) { return 50.0 * kUg; }, 2e-6));
    double s25 = 0.0, q25 = 0.0, s10 = 0.0, q10 = 0.0;
    constexpr int kSamples = 3000;
    for (int k = 1; k <= kSamples; ++k) {
        ASSERT_TRUE(r.runTo(k + 0.5));
        const double a = r.read("sensor/pm/pm2_5") / kUg - 50.0;
        const double b = r.read("sensor/pm/pm10") / kUg - 50.0;
        s25 += a;
        q25 += a * a;
        s10 += b;
        q10 += b * b;
    }
    EXPECT_NEAR(std::sqrt(q25 / kSamples - (s25 / kSamples) * (s25 / kSamples)), 7.5, 0.45);
    EXPECT_NEAR(s25 / kSamples, 0.0, 1.0);
    // PM10's spread is cut at 0 (no negative readings) at 2σ below: a little under 25.
    EXPECT_GT(std::sqrt(q10 / kSamples), 18.0);
    EXPECT_LT(std::sqrt(q10 / kSamples), 26.5);
}

TEST(ParticleSensor, SaturatesAtItsRange) {
    KindRun r = pmRun({{"noise_scale", 0.0}, {"time_constant", 0.0}},
                      air([](double) { return 5000.0 * kUg; }, 2e-6));
    ASSERT_TRUE(r.runTo(2.5));
    EXPECT_EQ(r.read("sensor/pm/pm2_5"), 1000.0 * kUg);
}

TEST(ParticleSensor, NothingWithoutTheWeather) {
    KindRun r = pmRun({}, nullptr);
    ASSERT_TRUE(r.runTo(2.5));
    EXPECT_TRUE(std::isnan(r.read("sensor/pm/pm10")));
    EXPECT_EQ(r.read("sensor/pm/valid"), 0.0);
}

}  // namespace
}  // namespace lagrange::sensors::test
