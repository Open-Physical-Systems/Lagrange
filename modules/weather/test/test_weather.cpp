// The weather domain against closed forms (domains/weather.md §7): the atmosphere, Koschmieder
// and Beer–Lambert, the Kim model, Marshall–Palmer, dust; and the module's signals.
#include "lagrange/fields/Field.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/weather/Weather.hpp"

#include <cmath>
#include <numbers>
#include <string>

#include <gtest/gtest.h>

namespace lagrange::weather {
namespace {

using namespace literals;

constexpr double kMmph = 1e-3 / 3600.0;  // m/s per mm/h

TEST(Atmosphere, DryAirAtSeaLevelIsTheStandardAtmosphere) {
    // ICAO standard atmosphere at sea level: 288.15 K, 101325 Pa, 1.2250 kg/m³ (dry).
    EXPECT_NEAR(moistAirDensity(288.15, 101325.0, 0.0), 1.2250, 5e-4);
    // Humid air is lighter: water vapour's molar mass is below dry air's.
    EXPECT_LT(moistAirDensity(303.15, 101325.0, 1.0), moistAirDensity(303.15, 101325.0, 0.0));
    // Saturation vapour pressure: 611.2 Pa at 0 °C, 2338 Pa at 20 °C, 7384 Pa at 40 °C (WMO
    // reference values over water).
    EXPECT_NEAR(saturationVapourPressure(273.15), 610.94, 1e-9);
    EXPECT_NEAR(saturationVapourPressure(293.15), 2338.0, 0.004 * 2338.0);
    EXPECT_NEAR(saturationVapourPressure(313.15), 7384.0, 0.004 * 7384.0);
}

TEST(Extinction, KoschmiederAndBeerLambert) {
    // At the visibility the contrast of a black target has fallen to 2 %.
    for (const double v : {50.0, 200.0, 1000.0}) {
        EXPECT_NEAR(std::exp(-fogExtinctionVisible(v) * v), 0.02, 1e-4) << v;
    }
    EXPECT_EQ(fogExtinctionVisible(0.0), 0.0);  // no fog
}

TEST(Extinction, KimModelFlattensInDenseFog) {
    EXPECT_EQ(kimExponent(100e3), 1.6);
    EXPECT_EQ(kimExponent(10e3), 1.3);
    EXPECT_NEAR(kimExponent(3e3), 0.16 * 3.0 + 0.34, 1e-12);
    EXPECT_NEAR(kimExponent(800.0), 0.3, 1e-12);
    EXPECT_EQ(kimExponent(200.0), 0.0);
    // Dense fog: the same extinction at 905 and 1550 nm as in the visible.
    EXPECT_NEAR(fogExtinction(200.0, kLidar1550), fogExtinctionVisible(200.0), 1e-12);
    // Haze: longer wavelengths get through better.
    EXPECT_LT(fogExtinction(3e3, kLidar1550), fogExtinction(3e3, kLidar905));
}

/// Rain extinction is the integral of twice each drop's cross-section over the distribution.
TEST(Extinction, RainIsTheMarshallPalmerIntegral) {
    for (const double mmph : {1.0, 10.0, 50.0}) {
        const double slope = marshallPalmerSlope(mmph * kMmph);
        EXPECT_NEAR(slope, 4100.0 * std::pow(mmph, -0.21), 1e-9 * slope);
        double sum = 0.0;
        const double step = 1e-6;  // m
        for (double d = 0.5 * step; d < 0.02; d += step) {
            sum += 2.0 * std::numbers::pi * d * d / 4.0 * kMarshallPalmerN0 * std::exp(-slope * d) *
                   step;
        }
        EXPECT_NEAR(rainExtinction(mmph * kMmph), sum, 1e-6 * sum) << mmph;
    }
    // 25 mm/h, heavy rain: about 2.8e-3 1/m, a visibility of about 1.4 km.
    EXPECT_NEAR(rainExtinction(25.0 * kMmph), 2.8e-3, 0.1e-3);
}

/// The rain rate carried by the distribution, with the fall speed of drops (Atlas, Srivastava and
/// Sekhon 1973: v = 9.65 − 10.3·exp(−0.6·D), D in mm, v in m/s): Marshall and Palmer's fit
/// carries 5 to 18 % more than it was built from at 100 to 1 mm/h. This checks the units (an
/// error there is orders of magnitude), not the fit, which is known to be loose.
TEST(Extinction, MarshallPalmerCarriesItsRainRate) {
    for (const double mmph : {1.0, 10.0, 100.0}) {
        const double slope = marshallPalmerSlope(mmph * kMmph);
        double rate = 0.0;
        const double step = 1e-6;
        for (double d = 0.5 * step; d < 0.008; d += step) {
            const double fall = std::max(0.0, 9.65 - 10.3 * std::exp(-600.0 * d));
            rate += kMarshallPalmerN0 * std::exp(-slope * d) * std::numbers::pi / 6.0 * d * d * d *
                    fall * step;
        }
        EXPECT_NEAR(rate / (mmph * kMmph), 1.0, 0.2) << mmph;
    }
}

TEST(Extinction, Dust) {
    // 1 mg/m³ of 10 µm quartz: 3·1e-6 / (2650·1e-5) = 1.13e-4 1/m.
    EXPECT_NEAR(dustExtinction(1e-6, 1e-5), 3e-6 / (2650.0 * 1e-5), 1e-15);
    EXPECT_EQ(dustExtinction(0.0, 1e-5), 0.0);
}

World site() {
    World w;
    const BodyId base = w.addBody("base", kWorldBody, Pose{}, 1.0, {}, {0.1, 0.1, 0.1, 0, 0, 0});
    w.addSensor("lidar", SensorType::Lidar, base, Pose{}, 10.0, {});
    return w;
}

struct Fixture {
    std::unique_ptr<Scheduler> scheduler;
    [[nodiscard]] double read(const std::string& name) const {
        return scheduler->bus().read(scheduler->bus().find(name));
    }
};

Fixture run(const World& world, ModuleConfigData config, bool clear = false) {
    Fixture r;
    r.scheduler = std::make_unique<Scheduler>(SchedulerOptions{.deterministic = true});
    r.scheduler->add(std::make_unique<Weather>(clear), ModuleConfig(std::move(config)));
    auto wired = r.scheduler->wire(WorldView(world));
    EXPECT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    return r;
}

ModuleConfigData siteConfig() {
    ModuleConfigData c;
    c.values["temperature"] = 283.15;
    c.values["pressure"] = 100000.0;
    c.values["relative_humidity"] = 0.95;
    return c;
}

TEST(Weather, PublishesTheAtmosphereAndItsExtinction) {
    const World world = site();
    ModuleConfigData c = siteConfig();
    c.values["visibility"] = 200.0;
    c.values["rain_rate"] = 10.0 * kMmph;
    Fixture r = run(world, c);
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{1'000'000'000}).ok());
    EXPECT_EQ(r.read("env/air_temperature"), 283.15);
    EXPECT_NEAR(r.read("env/air_density"), moistAirDensity(283.15, 100000.0, 0.95), 1e-12);
    const double expected = fogExtinctionVisible(200.0) + rainExtinction(10.0 * kMmph);
    EXPECT_NEAR(r.read("env/extinction/visible"), expected, 1e-12);
    EXPECT_NEAR(r.read("env/extinction/nir_905"), expected, 1e-12);  // dense fog: q = 0
    // A minute of rain: the ground wets (friction falls), the lidar's window clouds.
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{60'000'000'000}).ok());
    EXPECT_LT(r.read("env/friction_scale"), 0.95);
    EXPECT_GT(r.read("env/friction_scale"), kWetFrictionScale);
    EXPECT_GT(r.read("sensor/lidar/contamination"), 0.0);
}

TEST(Weather, ClearHasNoFogRainOrDust) {
    const World world = site();
    ModuleConfigData c = siteConfig();
    c.values["visibility"] = 200.0;
    c.values["rain_rate"] = 10.0 * kMmph;
    Fixture r = run(world, c, true);
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{10'000'000'000}).ok());
    EXPECT_EQ(r.read("env/extinction/visible"), 0.0);
    EXPECT_EQ(r.read("env/rain_rate"), 0.0);
    EXPECT_EQ(r.read("env/friction_scale"), 1.0);
    EXPECT_EQ(r.read("sensor/lidar/contamination"), 0.0);
    EXPECT_EQ(r.read("env/air_temperature"), 283.15);
}

TEST(Weather, PublishesItsFogGrid) {
    const std::string path = testing::TempDir() + "/fog_grid.nvdb";
    ASSERT_TRUE(fields::writeScalarField(path, "fog", {{0.0, 0.0, 0.0}, 0.5, {9, 9, 5}},
                                         [](const fields::Vec3& p) { return 0.01 + 0.01 * p[0]; }));
    const World world = site();
    ModuleConfigData c = siteConfig();
    c.values["fog_field"] = path;
    Fixture r = run(world, c);
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{100'000'000}).ok());
    const SignalBus& bus = r.scheduler->bus();
    fields::FieldSignal signal;
    const auto field = signal.read(bus, bus.find("env/extinction/field"));
    ASSERT_TRUE(field);
    EXPECT_NEAR(field->sampleScalar({2.0, 1.0, 1.0}), 0.03, 1e-6);
    EXPECT_FALSE(signal.read(bus, bus.find("env/wind/velocity_field")));  // no wind grid
}

TEST(Weather, RefusesWithoutTheSite) {
    const World world = site();
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<Weather>());
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_FALSE(wired);
    EXPECT_NE(to_string(wired.error().front()).find("no defaults"), std::string::npos);
}

TEST(Weather, TheEffectsListSwitchesEffects) {
    const World world = site();
    ModuleConfigData c = siteConfig();
    c.values["visibility"] = 200.0;
    c.values["rain_rate"] = 10.0 * kMmph;
    c.values["effects"] = std::vector<std::string>{"rain"};
    Fixture r = run(world, c);
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{100'000'000}).ok());
    EXPECT_NEAR(r.read("env/extinction/visible"), rainExtinction(10.0 * kMmph), 1e-12);
}

}  // namespace
}  // namespace lagrange::weather
