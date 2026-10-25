// The EMI domain (domains/emi.md §7): closed-form fields, the behavioural victim curves, and the
// module end to end, including a mains field aliased by a magnetometer's sample rate.
#include "lagrange/emi/Emi.hpp"
#include "lagrange/scheduler/Scheduler.hpp"

#include <cmath>
#include <complex>
#include <numbers>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::emi {
namespace {

constexpr double kPi = std::numbers::pi;

TEST(EmiFields, StraightConductorByTheRightHandRule) {
    // 400 A along +x, 10 m below the point: B = μ0·I/(2π·d) = 8 µT, along x̂ × ẑ = −ŷ.
    const Vec3 b = lineField({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 400.0, {3.0, 0.0, 10.0});
    const double expected = kMu0 * 400.0 / (2.0 * kPi * 10.0);
    EXPECT_NEAR(b[1], -expected, 1e-6 * expected);
    EXPECT_NEAR(b[0], 0.0, 1e-18);
    EXPECT_NEAR(b[2], 0.0, 1e-18);
}

TEST(EmiFields, DipoleOnItsAxisAndAtItsEquator) {
    const Vec3 m{0.0, 0.0, 2.0};  // A·m²
    const double r = 0.5;
    const double k = kMu0 / (4.0 * kPi * r * r * r);
    const Vec3 axis = dipoleField({0.0, 0.0, 0.0}, m, {0.0, 0.0, r});
    EXPECT_NEAR(axis[2], 2.0 * k * 2.0, 1e-6 * 4.0 * k);  // 2m along the axis
    const Vec3 equator = dipoleField({0.0, 0.0, 0.0}, m, {r, 0.0, 0.0});
    EXPECT_NEAR(equator[2], -k * 2.0, 1e-6 * 2.0 * k);  // −m at the equator
    EXPECT_NEAR(equator[0], 0.0, 1e-18);
}

TEST(EmiFields, FarFieldMatchesFriis) {
    // E = √(30·P)/d, and an isotropic antenna in it receives P·(λ/(4πd))².
    const double eirp = 0.1;  // W
    const double d = 25.0;
    const double f = 2.437e9;
    const double e = farField(eirp, d);
    EXPECT_NEAR(e, std::sqrt(30.0 * eirp) / d, 1e-15);
    const double lambda = 299792458.0 / f;
    const double friis = eirp * std::pow(lambda / (4.0 * kPi * d), 2);
    EXPECT_NEAR(receivedPower(e, f), friis, 1e-9 * friis);
}

TEST(EmiVictims, BehaviouralCurves) {
    EXPECT_DOUBLE_EQ(glitchProbability(5.0, 5.0), 0.5);
    EXPECT_EQ(glitchProbability(0.0, 5.0), 0.0);
    EXPECT_NEAR(glitchProbability(10.0, 5.0), 16.0 / 17.0, 1e-15);
    EXPECT_DOUBLE_EQ(bitErrorRate(2.0, 1.0, 1e-5), 4e-5);
    EXPECT_EQ(bitErrorRate(1e6, 1.0, 1e-5), 0.5);
    EXPECT_DOUBLE_EQ(bandOverlap(2.437e9, 20e6, 2.437e9, 20e6), 1.0);
    EXPECT_DOUBLE_EQ(bandOverlap(2.437e9, 20e6, 2.447e9, 20e6), 0.5);
    EXPECT_EQ(bandOverlap(5.2e9, 20e6, 2.437e9, 20e6), 0.0);
}

/// A world with sources and victims fixed in it.
struct Scene {
    World world;
    void add(const char* name, const char* schema, Pose pose,
             std::map<std::string, double, std::less<>> reals,
             std::map<std::string, std::string, std::less<>> tokens) {
        SchemaInstance s;
        s.schema = schema;
        s.prim = world.addPrim(std::string("/site/") + name, name);
        world.prims.poseInBody[s.prim.index] = pose;
        s.reals = std::move(reals);
        s.tokens = std::move(tokens);
        world.schemaInstances.push_back(std::move(s));
    }
};

Pose at(double x, double y, double z) {
    Pose p;
    p.position = {x, y, z};
    return p;
}

Scene powerLine() {
    Scene s;
    // Mains: a site parameter, stated per scene (60 Hz: western Japan, the Americas).
    s.add("line", "EmiSourceAPI_1", at(0.0, 0.0, 10.0),
          {{"emi:frequency", 60.0}, {"emi:current", 400.0}}, {{"emi:sourceType", "power_line"}});
    s.add("mag", "EmiSusceptibilityAPI_1", at(0.0, 0.0, 0.0), {},
          {{"emi:victimType", "magnetometer"}});
    return s;
}

std::unique_ptr<Scheduler> run(const World& world, ModuleConfigData config = {},
                               bool zero = false) {
    auto scheduler = std::make_unique<Scheduler>(SchedulerOptions{.deterministic = true});
    scheduler->add(std::make_unique<Emi>(zero), ModuleConfig(std::move(config)));
    auto wired = scheduler->wire(WorldView(world));
    EXPECT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    return wired ? std::move(scheduler) : nullptr;
}

TEST(Emi, AMainsFieldAliasesAtTheMagnetometersRate) {
    const Scene scene = powerLine();
    auto scheduler = run(scene.world);
    ASSERT_TRUE(scheduler);
    const SignalBus& bus = scheduler->bus();
    const SignalId b = bus.find("emi/mag/b_field");
    // Sampled at 100 Hz for 1 s: a 60 Hz field shows as 40 Hz (|60 − 100|).
    constexpr int kSamples = 100;
    std::vector<double> y;
    double peak = 0.0;
    for (int i = 1; i <= kSamples; ++i) {
        ASSERT_TRUE(scheduler->stepUntil(SimTime{i * 10'000'000LL + 1}).ok());
        const std::span<const double> v = bus.readN(b);
        y.push_back(v[1]);
        peak = std::max(peak, std::abs(v[1]));
        EXPECT_EQ(v[0], 0.0);
    }
    const double rms = kMu0 * 400.0 / (2.0 * kPi * 10.0);
    EXPECT_LE(peak, std::numbers::sqrt2 * rms * (1.0 + 1e-9));
    EXPECT_GT(peak, 0.9 * std::numbers::sqrt2 * rms);
    int best = 0;
    double bestPower = 0.0;
    for (int k = 1; k < kSamples / 2; ++k) {
        std::complex<double> sum;
        for (int n = 0; n < kSamples; ++n) {
            sum += y[n] * std::polar(1.0, -2.0 * kPi * k * n / kSamples);
        }
        if (std::norm(sum) > bestPower) {
            bestPower = std::norm(sum);
            best = k;
        }
    }
    EXPECT_EQ(best, 40);
}

TEST(Emi, APowerLineNeedsTheSitesFrequency) {
    Scene scene;
    scene.add("line", "EmiSourceAPI_1", at(0.0, 0.0, 10.0), {{"emi:current", 400.0}},
              {{"emi:sourceType", "power_line"}});
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<Emi>());
    auto wired = scheduler.wire(WorldView(scene.world));
    ASSERT_FALSE(wired);
    EXPECT_NE(to_string(wired.error().front()).find("never a default"), std::string::npos);
}

TEST(Emi, CouplingAndTheZeroSubstitute) {
    const Scene scene = powerLine();
    ModuleConfigData c;
    c.values["coupling"] = std::vector<std::string>{"imu"};
    for (bool zero : {false, true}) {
        auto scheduler = run(scene.world, zero ? ModuleConfigData{} : c, zero);
        ASSERT_TRUE(scheduler);
        ASSERT_TRUE(scheduler->stepUntil(SimTime{4'000'000}).ok());
        const SignalBus& bus = scheduler->bus();
        for (const double v : bus.readN(bus.find("emi/mag/b_field"))) {
            EXPECT_EQ(v, 0.0);
        }
    }
}

TEST(Emi, TransmitterNoiseInterferenceAndShielding) {
    Scene scene;
    scene.add("tx", "EmiSourceAPI_1", at(10.0, 0.0, 0.0),
              {{"emi:frequency", 2.437e9}, {"emi:radiatedPower", 0.1}, {"emi:bandwidth", 20e6}},
              {{"emi:sourceType", "transmitter"}});
    scene.add("imu", "EmiSusceptibilityAPI_1", at(0.0, 0.0, 0.0),
              {{"emi:accelNoisePerField", 0.01},
               {"emi:gyroNoisePerField", 0.002},
               {"emi:shieldingEffectiveness", 2.0}},
              {{"emi:victimType", "imu_adc"}});
    scene.add("radio", "EmiSusceptibilityAPI_1", at(0.0, 0.0, 0.0), {},
              {{"emi:victimType", "radio"}});
    SchemaInstance band;
    band.schema = "RadioLinkAPI_1";
    band.prim = scene.world.schemaInstances.back().prim;
    band.reals = {{"comms:frequency", 2.447e9}, {"comms:bandwidth", 20e6}};
    scene.world.schemaInstances.push_back(band);
    auto scheduler = run(scene.world);
    ASSERT_TRUE(scheduler);
    ASSERT_TRUE(scheduler->stepUntil(SimTime{2'000'000}).ok());
    const SignalBus& bus = scheduler->bus();
    const double e = farField(0.1, 10.0);
    EXPECT_NEAR(bus.read(bus.find("emi/imu/e_field_rms")), e / 2.0, 1e-12);
    EXPECT_NEAR(bus.read(bus.find("emi/imu/accel_noise")), 0.01 * e / 2.0, 1e-12);
    EXPECT_NEAR(bus.read(bus.find("emi/imu/gyro_noise")), 0.002 * e / 2.0, 1e-12);
    // Half the transmitter's band falls in the receiver's.
    const double p = receivedPower(e, 2.437e9) * 0.5;
    EXPECT_NEAR(bus.read(bus.find("emi/radio/interference_power")), p, 1e-9 * p);
}

TEST(Emi, AHarnessLoopPicksUpTheMainsField) {
    Scene scene = powerLine();
    scene.world.schemaInstances.back().reals["emi:couplingLoopArea"] = 0.01;  // 10 cm × 10 cm
    auto scheduler = run(scene.world);
    ASSERT_TRUE(scheduler);
    ASSERT_TRUE(scheduler->stepUntil(SimTime{2'000'000}).ok());
    const SignalBus& bus = scheduler->bus();
    // V = 2π·f·B·A, RMS.
    const double b = kMu0 * 400.0 / (2.0 * kPi * 10.0);
    EXPECT_NEAR(bus.read(bus.find("emi/mag/induced_voltage")), 2.0 * kPi * 60.0 * b * 0.01, 1e-12);
}

}  // namespace
}  // namespace lagrange::emi
