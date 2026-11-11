// Kind "sun_sensor" (sensors.md §9): the sun's direction in the sensor frame, its field of view,
// shade, and the weather's sun when the site gives no place and date.
#include "kinds_test_util.hpp"
#include "lagrange/sensors/EnvironmentSensors.hpp"

#include <cmath>
#include <numbers>

namespace lagrange::sensors {
namespace {

using namespace test;

/// Pitched up by `degrees` (+x towards +z).
Pose pitchedUp(double degrees) {
    const double a = -degrees * std::numbers::pi / 360.0;
    return {{0, 0, 0}, {std::cos(a), 0.0, std::sin(a), 0.0}};
}

std::vector<Feed> overhead() {
    std::vector<Feed> f = bodyFeeds(Pose{}, {});
    f.push_back(
        constant("env/sun_direction", SignalType::Vec3, units::one, {0, 0, 1}, "world_local"));
    return f;
}

TEST(SunSensor, TheWeathersSunInTheSensorFrame) {
    KindRun r;
    rig(r.world, "sun", "sun_sensor", pitchedUp(60.0), 10.0);
    ASSERT_TRUE(start(r, overhead(), true));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{150'000'000}).ok());
    const auto v = r.readN("sensor/sun/sun_direction");
    EXPECT_NEAR(v[0], std::cos(30.0 * std::numbers::pi / 180.0), 1e-12);
    EXPECT_NEAR(v[1], 0.0, 1e-12);
    EXPECT_NEAR(v[2], 0.5, 1e-12);
    EXPECT_EQ(r.read("sensor/sun/sun_visible"), 1.0);
}

TEST(SunSensor, OutsideItsFieldOfView) {
    KindRun r;
    rig(r.world, "sun", "sun_sensor", pitchedUp(20.0), 10.0);  // 70° off: outside ±60°
    ASSERT_TRUE(start(r, overhead(), true));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{150'000'000}).ok());
    EXPECT_EQ(r.read("sensor/sun/sun_visible"), 0.0);
    EXPECT_EQ(r.read("sensor/sun/valid"), 1.0);
}

TEST(SunSensor, InTheShade) {
    KindRun r;
    rig(r.world, "sun", "sun_sensor", pitchedUp(90.0), 10.0);
    Pose roof;
    roof.position = {0.0, 0.0, 3.0};
    r.world.addCollider(kWorldBody, ShapeType::Box, {1.0, 1.0, 0.1}, roof, 1.0);
    ASSERT_TRUE(start(r, overhead(), true));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{150'000'000}).ok());
    EXPECT_EQ(r.read("sensor/sun/sun_visible"), 0.0);
}

TEST(SunSensor, NoiseStaysWithinItsAccuracy) {
    // nanoSSOC-A60: < 0.5° (3σ).
    KindRun r;
    rig(r.world, "sun", "sun_sensor", pitchedUp(90.0), 100.0);
    ASSERT_TRUE(start(r, overhead()));
    for (int k = 1; k <= 100; ++k) {
        ASSERT_TRUE(r.scheduler->stepUntil(SimTime{k * 10'000'000LL}).ok());
        const auto v = r.readN("sensor/sun/sun_direction");
        EXPECT_NEAR(v[0] * v[0] + v[1] * v[1] + v[2] * v[2], 1.0, 1e-12);
        EXPECT_LT(std::acos(std::min(1.0, v[0])), 0.75 * std::numbers::pi / 180.0);
    }
}

TEST(SunSensor, NoSunAtAll) {
    KindRun r;
    rig(r.world, "sun", "sun_sensor", pitchedUp(90.0), 10.0);
    ASSERT_TRUE(start(r, bodyFeeds(Pose{}, {})));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{150'000'000}).ok());
    EXPECT_EQ(r.read("sensor/sun/valid"), 0.0);
    EXPECT_EQ(r.read("sensor/sun/sun_visible"), 0.0);
}

}  // namespace
}  // namespace lagrange::sensors
