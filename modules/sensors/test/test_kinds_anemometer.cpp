// Kind "anemometer" (sensors.md §9): the air relative to the sensor in its frame, the
// meteorological direction, the cup's starting threshold and distance-constant lag.
#include "kinds_test_util.hpp"
#include "lagrange/sensors/EnvironmentSensors.hpp"

#include <cmath>
#include <numbers>

namespace lagrange::sensors {
namespace {

using namespace test;

constexpr double kPi = std::numbers::pi;

std::vector<Feed> feeds(const Pose& pose, const std::array<double, 6>& twist,
                        std::function<std::vector<double>(double)> wind) {
    std::vector<Feed> f = bodyFeeds(pose, twist);
    f.push_back({makeSpec("env/wind/velocity", SignalType::Vec3, units::m_per_s, "world_local"),
                 std::move(wind)});
    return f;
}

TEST(Anemometer, MeteorologicalDirection) {
    EXPECT_NEAR(windFromDirection(-1.0, 0.0), 0.0, 1e-12);        // from +x
    EXPECT_NEAR(windFromDirection(0.0, 1.0), kPi / 2.0, 1e-12);   // from −y (clockwise)
    EXPECT_NEAR(windFromDirection(1.0, 0.0), kPi, 1e-12);         // from −x
    EXPECT_NEAR(windFromDirection(0.0, -1.0), 1.5 * kPi, 1e-12);  // from +y
}

TEST(Anemometer, GroundTruthIsTheWindInTheSensorFrame) {
    KindRun r;
    rig(r.world, "wind", "anemometer", Pose{}, 50.0, {{"axes", 3.0}});
    ASSERT_TRUE(
        start(r, feeds(Pose{}, {}, [](double) { return std::vector<double>{3, 4, 1}; }), true));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{100'000'000}).ok());
    const auto v = r.readN("sensor/wind/wind_velocity");
    EXPECT_NEAR(v[0], 3.0, 1e-12);
    EXPECT_NEAR(v[1], 4.0, 1e-12);
    EXPECT_NEAR(v[2], 1.0, 1e-12);
    EXPECT_NEAR(r.read("sensor/wind/wind_speed"), std::sqrt(26.0), 1e-12);
    EXPECT_NEAR(r.read("sensor/wind/wind_direction"), std::atan2(4.0, -3.0), 1e-12);
    EXPECT_EQ(r.read("sensor/wind/valid"), 1.0);
}

TEST(Anemometer, ItsOwnMotionAndHeadingCount) {
    // Turned 90° left (x along world +y) and moving at 10 m/s along world +y in still air: the air
    // comes at it from straight ahead, at 10 m/s.
    KindRun r;
    rig(r.world, "wind", "anemometer", Pose{}, 50.0);
    const double h = std::sqrt(0.5);
    const Pose yawed{{0, 0, 0}, {h, 0, 0, h}};
    ASSERT_TRUE(start(
        r, feeds(yawed, {0, 10, 0, 0, 0, 0}, [](double) { return std::vector<double>{0, 0, 0}; }),
        true));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{100'000'000}).ok());
    const auto v = r.readN("sensor/wind/wind_velocity");
    EXPECT_NEAR(v[0], -10.0, 1e-9);
    EXPECT_NEAR(v[1], 0.0, 1e-9);
    EXPECT_NEAR(r.read("sensor/wind/wind_speed"), 10.0, 1e-9);
    EXPECT_NEAR(r.read("sensor/wind/wind_direction"), 0.0, 1e-9);
}

TEST(Anemometer, NoAerodynamicsDomainIsNotValid) {
    KindRun r;
    rig(r.world, "wind", "anemometer", Pose{}, 50.0);
    ASSERT_TRUE(start(r, bodyFeeds(Pose{}, {})));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{100'000'000}).ok());
    EXPECT_EQ(r.read("sensor/wind/valid"), 0.0);
    EXPECT_TRUE(std::isnan(r.read("sensor/wind/wind_speed")));
}

TEST(Anemometer, SonicNoiseStaysWithinItsAccuracy) {
    // WindSonic: 2 % RMSE of 12 m/s (1σ), ±2° (2σ); every sample within 4σ of the truth, on its
    // grid.
    KindRun r;
    rig(r.world, "wind", "anemometer", Pose{}, 100.0);
    ASSERT_TRUE(start(r, feeds(Pose{}, {}, [](double) { return std::vector<double>{-12, 0, 0}; })));
    double sum = 0.0;
    int n = 0;
    for (int k = 1; k <= 200; ++k) {
        ASSERT_TRUE(r.scheduler->stepUntil(SimTime{k * 10'000'000LL}).ok());
        const double s = r.read("sensor/wind/wind_speed");
        double d = r.read("sensor/wind/wind_direction");
        d = d > kPi ? d - 2.0 * kPi : d;
        EXPECT_NEAR(s, 12.0, 4.0 * 0.24);
        EXPECT_NEAR(d, 0.0, 4.0 * 1.0 * kPi / 180.0 + kPi / 360.0);
        EXPECT_NEAR(std::remainder(s, 0.01), 0.0, 1e-9);
        sum += s;
        ++n;
    }
    EXPECT_NEAR(sum / n, 12.0, 0.05);
}

TEST(Anemometer, CupStartsAtItsThresholdAndLagsByItsDistanceConstant) {
    // R. M. Young 05103: threshold 1.0 m/s, distance constant 2.7 m, so τ = 2.7 m / 5 m/s.
    KindRun r;
    rig(r.world, "wind", "anemometer", Pose{}, 100.0, {{"resolution/speed", 0.0}},
        {{"type", "cup"}});
    ASSERT_TRUE(start(r, feeds(Pose{}, {}, [](double t) {
                          return std::vector<double>{t < 1.0 ? -0.5 : -5.0, 0, 0};
                      })));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{990'000'000}).ok());
    EXPECT_NEAR(r.read("sensor/wind/wind_speed"), 0.0, 1e-12);  // under the threshold
    // The test's source writes the wind for the end of its step, so the sample that ends at 1 s
    // already sees the gust: y(t) = 5·(1 − e^(−(t − 0.99 s)/τ)), exact for a held input.
    const double tau = 2.7 / 5.0;
    for (const double after : {0.2, tau, 1.5}) {
        const auto at = static_cast<std::int64_t>(std::llround((1.0 + after) * 1e9));
        ASSERT_TRUE(r.scheduler->stepUntil(SimTime{at}).ok());
        EXPECT_NEAR(r.read("sensor/wind/wind_speed"), 5.0 * (1.0 - std::exp(-(after + 0.01) / tau)),
                    1e-9)
            << after;
    }
}

}  // namespace
}  // namespace lagrange::sensors
