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

namespace lagrange::emi {
namespace {

// RFC 0013: a straight segment by Biot–Savart (Griffiths Example 5.5).
TEST(EmiWires, ALongSegmentIsTheInfiniteWire) {
    const Vec3 b = segmentField({-100.0, 0.0, 0.0}, {100.0, 0.0, 0.0}, 20.0, {0.0, 0.0, 0.05});
    const Vec3 line = lineField({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 20.0, {0.0, 0.0, 0.05});
    EXPECT_NEAR(b[1], line[1], 1e-6 * std::abs(line[1]));
    EXPECT_NEAR(b[1], -kMu0 * 20.0 / (2.0 * kPi * 0.05), 1e-9);  // 80 µT, along x̂ × ẑ = −ŷ
}

TEST(EmiWires, OnItsPerpendicularBisector) {
    // B = μ0·I/(4π·d) · L/√(L²/4 + d²) for a segment of length L.
    constexpr double kL = 0.2;
    constexpr double kD = 0.03;
    const Vec3 b = segmentField({-kL / 2, 0.0, 0.0}, {kL / 2, 0.0, 0.0}, 10.0, {0.0, kD, 0.0});
    const double expected = kMu0 * 10.0 / (4.0 * kPi * kD) * kL / std::sqrt(kL * kL / 4 + kD * kD);
    EXPECT_NEAR(b[2], expected, 1e-12 * expected);  // x̂ × ŷ = ẑ
    // And off-centre, the two angles differ: the sum of two half segments.
    const Vec3 left = segmentField({-kL / 2, 0.0, 0.0}, {0.0, 0.0, 0.0}, 10.0, {0.0, kD, 0.0});
    const Vec3 right = segmentField({0.0, 0.0, 0.0}, {kL / 2, 0.0, 0.0}, 10.0, {0.0, kD, 0.0});
    EXPECT_NEAR(left[2] + right[2], b[2], 1e-15);
}

TEST(EmiWires, APairFallsOffAsTheSquareOfTheDistance) {
    // Supply and return 5 mm apart: exactly μ0·I/(2π)·(1/(r − s) − 1/r) in their plane, which
    // far away is μ0·I·s/(2π·r²), falling as 1/r² (PX4's compass compensation guide says the
    // same of a cable).
    constexpr double kS = 0.005;
    constexpr double kI = 20.0;
    std::vector<double> fields;
    for (const double r : {0.1, 0.2, 0.4}) {
        const Vec3 go = segmentField({-50.0, 0.0, 0.0}, {50.0, 0.0, 0.0}, kI, {0.0, r, 0.0});
        const Vec3 back = segmentField({-50.0, kS, 0.0}, {50.0, kS, 0.0}, -kI, {0.0, r, 0.0});
        const double pair = std::abs(go[2] + back[2]);
        const double exact = kMu0 * kI / (2.0 * kPi) * (1.0 / (r - kS) - 1.0 / r);
        EXPECT_NEAR(pair, exact, 1e-10) << r;  // the 100 m wires' ends: picotesla
        EXPECT_NEAR(pair, kMu0 * kI * kS / (2.0 * kPi * r * r), 0.06 * exact) << r;
        fields.push_back(pair);
    }
    EXPECT_NEAR(fields[1] / fields[2], 4.0, 0.06);  // twice as far, a quarter
}

TEST(EmiWires, AWireFollowsItsCurrentSignalLinearly) {
    World world;
    SchemaInstance wire;
    wire.schema = "EmiSourceAPI_1";
    wire.prim = world.addPrim("/bot/lead", "lead");
    wire.tokens = {{"emi:sourceType", "wire"}, {"emi:currentSignal", "battery/main/current"}};
    wire.realArrays = {{"emi:path", {0.0, -0.1, 0.0, 0.0, 0.1, 0.0}},
                       {"emi:returnOffset", {0.0, 0.0, 0.004}}};
    world.schemaInstances.push_back(wire);
    SchemaInstance mag;
    mag.schema = "EmiSusceptibilityAPI_1";
    mag.prim = world.addPrim("/bot/mag", "mag");
    world.prims.poseInBody[mag.prim.index] = at(0.05, 0.0, 0.0);
    mag.tokens = {{"emi:victimType", "magnetometer"}};
    world.schemaInstances.push_back(mag);
    const auto fieldAt = [&](double amps) {
        auto scheduler = std::make_unique<Scheduler>(SchedulerOptions{.deterministic = true});
        struct Current final : IModule {
            explicit Current(double a) : amps(a) {}
            double amps;
            SignalId out;
            [[nodiscard]] ModuleInfo info() const override {
                return {"test", "battery", 1, 0, false, true, CapabilityTier::Tier0};
            }
            [[nodiscard]] Duration preferredStep() const override { return Duration{1'000'000}; }
            Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
                SignalSpec s;
                s.name = "battery/main/current";
                s.unit = units::A;
                out = d.output(s);
                return {};
            }
            Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
                bus.write(out, amps);
                return {};
            }
            Status step(const StepContext&, SignalBus& bus) override {
                bus.write(out, amps);
                return {};
            }
            Status saveState(StateWriter&) const override { return {}; }
            Status restoreState(StateReader&) override { return {}; }
        };
        scheduler->add(std::make_unique<Current>(amps));
        scheduler->add(std::make_unique<Emi>());
        auto wired = scheduler->wire(WorldView(world));
        EXPECT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
        EXPECT_TRUE(scheduler->stepUntil(SimTime{5'000'000}).ok());
        const std::span<const double> b =
            scheduler->bus().readN(scheduler->bus().find("emi/mag/b_field"));
        return Vec3{b[0], b[1], b[2]};
    };
    const Vec3 ten = fieldAt(10.0);
    const Vec3 twenty = fieldAt(20.0);
    const Vec3 back = fieldAt(-10.0);
    // The pair, by hand: the lead along +y at the origin and its return 4 mm above, 5 cm away.
    const Vec3 go = segmentField({0.0, -0.1, 0.0}, {0.0, 0.1, 0.0}, 10.0, {0.05, 0.0, 0.0});
    const Vec3 ret = segmentField({0.0, -0.1, 0.004}, {0.0, 0.1, 0.004}, -10.0, {0.05, 0.0, 0.0});
    for (int k = 0; k < 3; ++k) {
        EXPECT_NEAR(ten[k], go[k] + ret[k], 1e-15) << k;
        EXPECT_NEAR(twenty[k], 2.0 * ten[k], 1e-15) << k;  // linear in the current
        EXPECT_NEAR(back[k], -ten[k], 1e-15) << k;         // and its sign
    }
    EXPECT_GT(std::hypot(ten[0], ten[1], ten[2]), 1e-6);  // over a microtesla at 10 A
}

TEST(EmiWires, AWireNeedsAPathAndACurrent) {
    World world;
    SchemaInstance wire;
    wire.schema = "EmiSourceAPI_1";
    wire.prim = world.addPrim("/bot/lead", "lead");
    wire.tokens = {{"emi:sourceType", "wire"}};
    wire.realArrays = {{"emi:path", {0.0, 0.0, 0.0}}};
    world.schemaInstances.push_back(wire);
    const auto sources = sourcesOf(WorldView(world));
    ASSERT_FALSE(sources);
    EXPECT_NE(sources.error().find("emi:path"), std::string::npos);
}

}  // namespace
}  // namespace lagrange::emi
