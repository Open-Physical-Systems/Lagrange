// Kind "light_sensor" (sensors.md §9) and the sun behind it: the NOAA solar position against the
// NREL SPA's published example, Kasten and Young's air mass, the IES daylight model, the sensor's
// cosine response, shadow and overcast; and the run's lamps (RFC 0021 §5) against the
// inverse-square law, with a box between lamp and sensor.
#include "kinds_test_util.hpp"
#include "lagrange/gpu/Realistic.hpp"
#include "lagrange/lights/Lamp.hpp"
#include "lagrange/sensors/EnvironmentSensors.hpp"
#include "lagrange/world/PoseMath.hpp"

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

TEST(LightSensor, CountsOfTheBH1750SaturateAt65535Counts) {
    KindRun r;
    equator(r);
    ASSERT_TRUE(start(r, bodyFeeds(Pose{}, {})));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{150'000'000}).ok());
    // full sun overloads it: 65535 counts / 1.2 (BH1750FVI datasheet Rev.D, p. 7)
    EXPECT_NEAR(r.read("sensor/lux/illuminance"), 65535.0 / 1.2, 1e-9);
}

// --- lamps (RFC 0021 §5) ----------------------------------------------------------------------

constexpr double kLampIntensity = 1200.0;  // cd on the axis
constexpr double kBeam = 0.6;              // rad, full angle to 50 %
constexpr double kField = 1.0;             // rad, full angle to 10 %

/// A spot lamp fixed in the world at `pose` (its beam along +X), named "spot".
void addLamp(World& w, const Pose& pose) {
    SchemaInstance s;
    s.schema = "LampAPI_1";
    s.prim = w.addPrim("/scene/spot", "spot");
    w.prims.body.back() = kWorldBody;
    w.prims.poseInBody.back() = pose;
    s.reals = {{"lamp:luminousIntensity", kLampIntensity},
               {"lamp:beamAngle", kBeam},
               {"lamp:fieldAngle", kField}};
    s.tokens["lamp:kind"] = "spot";
    w.schemaInstances.push_back(s);
}

/// The lamp 2 m above the sensor's body origin, beam straight down (+X turned onto −Z).
Pose lampAbove(double height) {
    return {{0.0, 0.0, height},
            {std::cos(std::numbers::pi / 4.0), 0.0, std::sin(std::numbers::pi / 4.0), 0.0}};
}

/// The lamp's spec as lights::lampsOf reads it, for the closed form.
lights::LampSpec lampSpec(const Pose& pose) {
    lights::LampSpec spec;
    spec.name = "spot";
    spec.intensity = kLampIntensity;
    spec.beamAngle = kBeam;
    spec.fieldAngle = kField;
    spec.poseInBody = pose;
    return spec;
}

/// I·f(θ)·cos(incidence)/d² written out from the geometry, not through lights::illuminance.
double closedForm(const Pose& lamp, const std::array<double, 3>& at,
                  const std::array<double, 3>& normal) {
    const std::array<double, 3> to{at[0] - lamp.position[0], at[1] - lamp.position[1],
                                   at[2] - lamp.position[2]};
    const double d = std::hypot(to[0], to[1], to[2]);
    const std::array<double, 3> axis = rotate(lamp.orientation, {1.0, 0.0, 0.0});
    const double theta = std::acos((to[0] * axis[0] + to[1] * axis[1] + to[2] * axis[2]) / d);
    const double incidence = -(to[0] * normal[0] + to[1] * normal[1] + to[2] * normal[2]) / d;
    return kLampIntensity * lights::taper(lampSpec(lamp), theta) * incidence / (d * d);
}

/// A run with the sensor facing up on the free body at `at`, the lamp at `lamp`, its output fed.
bool lampRun(KindRun& r, const Pose& lamp, const std::array<double, 3>& at, double intensity) {
    rig(r.world, "lux", "light_sensor", facingUp(), 10.0);
    addLamp(r.world, lamp);
    std::vector<Feed> f = bodyFeeds(Pose{at, {1.0, 0.0, 0.0, 0.0}}, {});
    f.push_back(constant("lamp/spot/intensity", SignalType::Scalar, units::cd, {intensity}));
    return start(r, std::move(f), true);
}

TEST(LightSensor, ALampFollowsTheInverseSquareLaw) {
    // RFC 0021 §Validation: on the axis and at 3 angles, at 3 distances, within 0.1 %. No sun
    // (no place, date or weather): the lamp alone.
    for (const double height : {0.5, 2.0, 7.0}) {
        for (const double angle : {0.0, 0.15, 0.3, 0.45}) {  // rad off the axis
            const Pose lamp = lampAbove(height);
            const std::array<double, 3> at{height * std::tan(angle), 0.0, 0.0};
            KindRun r;
            ASSERT_TRUE(lampRun(r, lamp, at, kLampIntensity));
            ASSERT_TRUE(r.scheduler->stepUntil(SimTime{150'000'000}).ok());
            const double expected = closedForm(lamp, at, {0.0, 0.0, 1.0});
            ASSERT_GT(expected, 0.0);
            EXPECT_NEAR(r.read("sensor/lux/illuminance") / expected, 1.0, 1e-3)
                << height << " m, " << angle << " rad";
            EXPECT_EQ(r.read("sensor/lux/valid"), 1.0);
        }
    }
    // On the axis the taper is 1: E = I/d².
    KindRun r;
    ASSERT_TRUE(lampRun(r, lampAbove(2.0), {0.0, 0.0, 0.0}, kLampIntensity));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{150'000'000}).ok());
    EXPECT_NEAR(r.read("sensor/lux/illuminance"), kLampIntensity / 4.0,
                kLampIntensity / 4.0 * 1e-3);
}

TEST(LightSensor, ALampOutsideItsFieldOrDarkGivesNothing) {
    KindRun outside;  // 0.6 rad off the axis: beyond half the field angle (0.5 rad)
    ASSERT_TRUE(lampRun(outside, lampAbove(2.0), {2.0 * std::tan(0.6), 0.0, 0.0}, kLampIntensity));
    ASSERT_TRUE(outside.scheduler->stepUntil(SimTime{150'000'000}).ok());
    EXPECT_EQ(outside.read("sensor/lux/illuminance"), 0.0);
    KindRun dark;  // the lights domain says it is off
    ASSERT_TRUE(lampRun(dark, lampAbove(2.0), {0.0, 0.0, 0.0}, 0.0));
    ASSERT_TRUE(dark.scheduler->stepUntil(SimTime{150'000'000}).ok());
    EXPECT_EQ(dark.read("sensor/lux/illuminance"), 0.0);
    EXPECT_EQ(dark.read("sensor/lux/valid"), 1.0);
}

TEST(LightSensor, ABoxBetweenLampAndSensorBlocksIt) {
    // RFC 0021 §Validation: 0 lx from an occluded lamp. A 0.4 m box halfway down the beam.
    KindRun r;
    rig(r.world, "lux", "light_sensor", facingUp(), 10.0);
    addLamp(r.world, lampAbove(2.0));
    Pose box;
    box.position = {0.0, 0.0, 1.0};
    r.world.addCollider(kWorldBody, ShapeType::Box, {0.2, 0.2, 0.2}, box, 1.0);
    std::vector<Feed> f = bodyFeeds(Pose{}, {});
    f.push_back(constant("lamp/spot/intensity", SignalType::Scalar, units::cd, {kLampIntensity}));
    ASSERT_TRUE(start(r, std::move(f), true));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{150'000'000}).ok());
    EXPECT_EQ(r.read("sensor/lux/illuminance"), 0.0);
    // The same box off to the side shades nothing.
    KindRun beside;
    rig(beside.world, "lux", "light_sensor", facingUp(), 10.0);
    addLamp(beside.world, lampAbove(2.0));
    box.position = {1.0, 0.0, 1.0};
    beside.world.addCollider(kWorldBody, ShapeType::Box, {0.2, 0.2, 0.2}, box, 1.0);
    std::vector<Feed> g = bodyFeeds(Pose{}, {});
    g.push_back(constant("lamp/spot/intensity", SignalType::Scalar, units::cd, {kLampIntensity}));
    ASSERT_TRUE(start(beside, std::move(g), true));
    ASSERT_TRUE(beside.scheduler->stepUntil(SimTime{150'000'000}).ok());
    EXPECT_NEAR(beside.read("sensor/lux/illuminance"), kLampIntensity / 4.0, 0.3);
}

TEST(LightSensor, LampsAddToTheSun) {
    // Daylight and a lamp: their sum.
    KindRun r;
    equator(r, {{"max_illuminance", 1e9}});
    addLamp(r.world, lampAbove(2.0));
    std::vector<Feed> f = bodyFeeds(Pose{}, {});
    f.push_back(constant("lamp/spot/intensity", SignalType::Scalar, units::cd, {kLampIntensity}));
    ASSERT_TRUE(start(r, std::move(f), true));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{150'000'000}).ok());
    EXPECT_NEAR(r.read("sensor/lux/illuminance"), expectedLux(false, false) + kLampIntensity / 4.0,
                2.0);
}

TEST(LightSensor, TheRenderersSunIsThisDaylight) {
    // RFC 0021 §4: the realistic renderer's units per lux are fixed by its drawn sun standing for
    // this sensor's clear-sky beam at that sun's elevation on its day without a date.
    const std::array<double, 3> sun = gpu::realisticSunDirection();
    const Daylight light = iesDaylight(std::asin(sun[2]), 80, false);
    EXPECT_NEAR(gpu::realisticSunIlluminance() / light.directNormal, 1.0, 1e-12);
    EXPECT_NEAR(
        gpu::realisticDaylight() / surfaceIlluminance(light, sun, {0.0, 0.0, 1.0}, false, 1.0, 0.0),
        1.0, 1e-12);
}

}  // namespace
}  // namespace lagrange::sensors
