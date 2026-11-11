// Kind "light_sensor" (sensors.md §9) and the sun behind it: the NOAA solar position against the
// NREL SPA's published example, Kasten and Young's air mass, the IES daylight model, the sensor's
// cosine response, shadow and overcast.
#include "kinds_test_util.hpp"
#include "lagrange/sensors/EnvironmentSensors.hpp"

#include <cmath>
#include <numbers>

namespace lagrange::sensors {
namespace {

using namespace test;

constexpr double kDeg = std::numbers::pi / 180.0;

TEST(Sun, JulianDayOfMeeusExample7a) {
    // Meeus (1998) example 7.a: 1957 October 4.81 is JD 2436116.31.
    EXPECT_NEAR(julianDay(1957, 10, 4, 0.81 * 24.0, 0.0, 0.0), 2436116.31, 1e-6);
    // J2000.0: 2000 January 1, 12h is JD 2451545.0.
    EXPECT_NEAR(*parseUtc("2000-01-01T12:00:00Z"), 2451545.0, 1e-9);
    EXPECT_NEAR(*parseUtc("2000-01-01T12:00:30.5"), 2451545.0 + 30.5 / 86400.0, 1e-9);
    EXPECT_FALSE(parseUtc("2000-13-01T12:00:00Z"));
    EXPECT_FALSE(parseUtc("yesterday"));
}

TEST(Sun, NoaaAgainstTheSpaExample) {
    // Reda and Andreas, "Solar Position Algorithm for Solar Radiation Applications" (NREL/TP-560-
    // 34302, 2004/2008), example A.5: 2003-10-17 12:30:30 at UTC−7, 39.742476° N, 105.1786° W:
    // topocentric zenith 50.11162°, azimuth 194.34024°. NOAA's algorithm is good to about 0.01°;
    // parallax (0.002°) and the SPA's 820 hPa refraction (0.004° less) are within that.
    const double jd = julianDay(2003, 10, 17, 19.0, 30.0, 30.0);
    const SolarPosition sun = solarPosition(jd, 39.742476 * kDeg, -105.1786 * kDeg);
    EXPECT_NEAR(90.0 - sun.elevation / kDeg, 50.11162, 0.02);
    EXPECT_NEAR(sun.azimuth / kDeg, 194.34024, 0.02);
    const auto v = sun.enu();
    EXPECT_NEAR(v[0] * v[0] + v[1] * v[1] + v[2] * v[2], 1.0, 1e-12);
    EXPECT_LT(v[0], 0.0);  // south-south-west: west of south
    EXPECT_LT(v[1], 0.0);
}

TEST(Sun, KastenYoungAirMass) {
    EXPECT_NEAR(airMass(0.0), 1.0, 1e-3);
    EXPECT_NEAR(airMass(60.0 * kDeg), 2.0, 0.01);    // the plane-parallel sec z, high up
    EXPECT_NEAR(airMass(90.0 * kDeg), 37.92, 0.05);  // Kasten and Young: 37.92 at the horizon
}

TEST(Sun, IesClearAndOvercastDaylight) {
    const double up = 90.0 * kDeg;
    const Daylight clear = iesDaylight(up, 80, false);
    const double extraterrestrial =
        127.5e3 * (1.0 + 0.033 * std::cos(2.0 * std::numbers::pi * 80.0 / 365.0));
    EXPECT_NEAR(clear.directNormal, extraterrestrial * std::exp(-0.21 * airMass(0.0)), 1e-6);
    EXPECT_NEAR(clear.diffuseHorizontal, 16.3e3, 1e-6);
    const Daylight overcast = iesDaylight(30.0 * kDeg, 80, true);
    EXPECT_EQ(overcast.directNormal, 0.0);
    EXPECT_NEAR(overcast.diffuseHorizontal, (0.3 + 21.0 * 0.5) * 1e3, 1e-6);
    EXPECT_EQ(iesDaylight(-1.0 * kDeg, 80, false).diffuseHorizontal, 0.0);
    // A level surface: direct·sin α + diffuse. Facing down: none of the sky.
    const Daylight low = iesDaylight(30.0 * kDeg, 80, false);
    const std::array<double, 3> sun{0.0, std::cos(30.0 * kDeg), 0.5};
    EXPECT_NEAR(surfaceIlluminance(low, sun, {0, 0, 1}, false, 1.0, 0.0),
                low.directNormal * 0.5 + low.diffuseHorizontal, 1e-6);
    EXPECT_NEAR(surfaceIlluminance(low, sun, {0, 0, -1}, false, 1.0, 0.0), 0.0, 1e-9);
}

/// A sensor facing up (+x turned to +z) on the equator at 0° E near the March equinox's noon.
Pose facingUp() {
    return {{0, 0, 0}, {std::sqrt(0.5), 0.0, -std::sqrt(0.5), 0.0}};
}

void equator(KindRun& r, std::map<std::string, double, std::less<>> settings = {}) {
    rig(r.world, "lux", "light_sensor", facingUp(), 10.0, std::move(settings),
        {{"start_utc", "2026-03-20T12:00:00Z"}});
    r.world.geoOrigin = coords::Geodetic{0.0, 0.0, 0.0};
}

double expectedLux(bool overcast, bool blocked) {
    const double jd = *parseUtc("2026-03-20T12:00:00Z") + 0.1 / 86400.0;
    const SolarPosition sun = solarPosition(jd, 0.0, 0.0);
    const Daylight light = iesDaylight(sun.elevation, 79, overcast);
    return surfaceIlluminance(light, sun.enu(), {0, 0, 1}, blocked, 1.0, 0.0);
}

TEST(LightSensor, NoonSunOnTheEquator) {
    KindRun r;
    equator(r);
    ASSERT_TRUE(start(r, bodyFeeds(Pose{}, {}), true));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{150'000'000}).ok());
    const double lux = r.read("sensor/lux/illuminance");
    EXPECT_NEAR(lux, expectedLux(false, false), 2.0);
    EXPECT_GT(lux, 90e3);  // a clear tropical noon: about 100 klx
    EXPECT_LT(lux, 130e3);
    EXPECT_EQ(r.read("sensor/lux/valid"), 1.0);
}

TEST(LightSensor, ARoofShadesTheBeamButNotTheSky) {
    KindRun r;
    equator(r);
    Pose roof;
    roof.position = {0.0, 0.0, 5.0};
    r.world.addCollider(kWorldBody, ShapeType::Box, {5.0, 5.0, 0.1}, roof, 1.0);
    ASSERT_TRUE(start(r, bodyFeeds(Pose{}, {}), true));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{150'000'000}).ok());
    EXPECT_NEAR(r.read("sensor/lux/illuminance"), expectedLux(false, true), 2.0);
}

TEST(LightSensor, RainMeansAnOvercastSky) {
    KindRun r;
    equator(r);
    std::vector<Feed> f = bodyFeeds(Pose{}, {});
    f.push_back(constant("env/rain_rate", SignalType::Scalar, units::m_per_s, {1e-6}));
    ASSERT_TRUE(start(r, std::move(f), true));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{150'000'000}).ok());
    EXPECT_NEAR(r.read("sensor/lux/illuminance"), expectedLux(true, false), 2.0);
}

TEST(LightSensor, NoSunWithoutAPlaceAndADate) {
    KindRun r;
    rig(r.world, "lux", "light_sensor", facingUp(), 10.0);
    ASSERT_TRUE(start(r, bodyFeeds(Pose{}, {})));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{150'000'000}).ok());
    EXPECT_EQ(r.read("sensor/lux/valid"), 0.0);
    EXPECT_TRUE(std::isnan(r.read("sensor/lux/illuminance")));
}

TEST(LightSensor, CountsOfTheBH1750SaturateAt65535) {
    KindRun r;
    equator(r);
    ASSERT_TRUE(start(r, bodyFeeds(Pose{}, {})));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{150'000'000}).ok());
    EXPECT_EQ(r.read("sensor/lux/illuminance"), 65535.0);  // full sun overloads it
}

}  // namespace
}  // namespace lagrange::sensors
