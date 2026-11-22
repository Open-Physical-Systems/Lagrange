// RFC 0021's lamp model against closed forms: the beam's taper at its defined edges, the
// luminous flux of a sharp cone, the inverse-square law on the axis and off it, blinking at the
// 10 ms step, the rise time, dimming with the bus voltage, a brownout, the substitutes, and the
// same run twice giving the same state.
#include "lagrange/lights/Lights.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <cmath>
#include <functional>
#include <memory>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::lights {
namespace {

using namespace literals;
constexpr double kPi = std::numbers::pi;

LampSpec spot(double intensity, double beam, double field = 0.0) {
    LampSpec lamp;
    lamp.name = "spot";
    lamp.intensity = intensity;
    lamp.beamAngle = beam;
    lamp.fieldAngle = field > 0.0 ? field : 2.0 * beam;
    return lamp;
}

TEST(Lamp, TheTaperPassesThroughTheBeamAndFieldEdges) {
    const LampSpec lamp = spot(1000.0, 0.6, 1.0);
    EXPECT_DOUBLE_EQ(taper(lamp, 0.0), 1.0);
    EXPECT_NEAR(taper(lamp, 0.3), 0.5, 1e-12);  // half the beam angle: 50 %
    EXPECT_NEAR(taper(lamp, 0.5), 0.1, 1e-12);  // half the field angle: 10 %
    EXPECT_EQ(taper(lamp, 0.5001), 0.0);        // dark beyond
    for (double theta = 0.0; theta < 0.5; theta += 0.01) {
        EXPECT_GE(taper(lamp, theta), taper(lamp, theta + 0.01));  // falls monotonically
    }
    LampSpec point = lamp;
    point.kind = LampKind::Point;
    EXPECT_EQ(taper(point, 2.0), 1.0);
}

TEST(Lamp, TheFluxOfASharpConeIsIntensityTimesItsSolidAngle) {
    // RFC 0021 §Validation: Φ = I·2π(1 − cos α), within 0.5 % (the integration).
    for (const double alpha : {0.05, 0.3, 1.0, kPi / 2.0}) {
        const double numeric = integrateOverCone([](double) { return 1.0; }, alpha, 64);
        EXPECT_NEAR(numeric / coneSolidAngle(alpha), 1.0, 0.005) << alpha;
    }
    // The tapered beam's flux lies between its beam cone's and its field cone's, and matches a
    // brute-force midpoint sum.
    const LampSpec lamp = spot(1000.0, 0.6, 1.0);
    const double omega = effectiveSolidAngle(lamp);
    EXPECT_GT(omega, coneSolidAngle(0.3));
    EXPECT_LT(omega, coneSolidAngle(0.5));
    double sum = 0.0;
    constexpr int kSteps = 200000;
    for (int i = 0; i < kSteps; ++i) {
        const double theta = (i + 0.5) * 0.5 / kSteps;
        sum += taper(lamp, theta) * 2.0 * kPi * std::sin(theta) * 0.5 / kSteps;
    }
    EXPECT_NEAR(omega / sum, 1.0, 1e-6);
    LampSpec point = lamp;
    point.kind = LampKind::Point;
    EXPECT_DOUBLE_EQ(effectiveSolidAngle(point), 4.0 * kPi);
}

TEST(Lamp, IlluminanceFollowsTheInverseSquareLaw) {
    // RFC 0021 §Validation: E = I·f(θ)·cos(incidence)/d², on the axis and at 3 angles, at 3
    // distances, within 0.1 %.
    const LampSpec lamp = spot(1200.0, 0.6, 1.0);
    // The lamp 2 m up, pointing down (+X of its frame turned onto −Z).
    const Pose down{{0.0, 0.0, 2.0}, {std::cos(kPi / 4.0), 0.0, std::sin(kPi / 4.0), 0.0}};
    ASSERT_NEAR(rotate(down.orientation, {1.0, 0.0, 0.0})[2], -1.0, 1e-12);
    for (const double d : {0.5, 2.0, 7.0}) {
        // On the axis, a surface facing the lamp.
        const Pose lamp1{{0.0, 0.0, d}, down.orientation};
        const double onAxis = illuminance(lamp, 1200.0, lamp1, {0, 0, 0}, {0, 0, 1}, 0.0);
        EXPECT_NEAR(onAxis / (1200.0 / (d * d)), 1.0, 1e-3) << d;
        for (const double theta : {0.15, 0.3, 0.45}) {  // 0.3: the beam edge, half the light
            const std::array<double, 3> at{d * std::tan(theta), 0.0, 0.0};
            const double range = d / std::cos(theta);
            // The floor sees the lamp at incidence θ.
            const double expected = 1200.0 * taper(lamp, theta) * std::cos(theta) / (range * range);
            EXPECT_NEAR(illuminance(lamp, 1200.0, lamp1, at, {0, 0, 1}, 0.0) / expected, 1.0, 1e-3)
                << d << " " << theta;
        }
        const std::array<double, 3> edge{d * std::tan(0.3), 0.0, 0.0};
        const double range = d / std::cos(0.3);
        EXPECT_NEAR(illuminance(lamp, 1200.0, lamp1, edge, {0, 0, 1}, 0.0),
                    0.5 * 1200.0 * std::cos(0.3) / (range * range), 1e-9);
    }
    // A surface facing away is dark; a point outside the field is dark.
    EXPECT_EQ(illuminance(lamp, 1200.0, down, {0, 0, 0}, {0, 0, -1}, 0.0), 0.0);
    EXPECT_EQ(illuminance(lamp, 1200.0, down, {3.0, 0, 0}, {0, 0, 1}, 0.0), 0.0);
}

TEST(Lamp, ABeaconSweepsAboutItsZAxis) {
    LampSpec beacon = spot(100.0, 0.4);
    beacon.kind = LampKind::Beacon;
    beacon.rotationSpeed = kPi;  // half a turn a second
    const auto axis = beamAxis(beacon, 0.5);
    EXPECT_NEAR(axis[0], 0.0, 1e-12);
    EXPECT_NEAR(axis[1], 1.0, 1e-12);
    // A wall 3 m along +Y is lit at t = 0.5 s, not at t = 0.
    const Pose at{};
    EXPECT_NEAR(illuminance(beacon, 100.0, at, {0, 3, 0}, {0, -1, 0}, 0.5), 100.0 / 9.0, 1e-9);
    EXPECT_EQ(illuminance(beacon, 100.0, at, {0, 3, 0}, {0, -1, 0}, 0.0), 0.0);
}

/// Writes a signal as a function of time, or records one each step.
class Probe final : public IModule {
public:
    Probe(std::string name, std::string signal, Unit unit, std::function<double(double)> f)
        : name_(std::move(name)), signal_(std::move(signal)), unit_(unit), f_(std::move(f)) {}
    [[nodiscard]] ModuleInfo info() const override {
        return {"test", name_, 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return 10_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec spec;
        spec.name = signal_;
        spec.unit = unit_;
        id_ = d.output(spec);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        bus.write(id_, f_(0.0));
        return {};
    }
    Status step(const StepContext& c, SignalBus& bus) override {
        bus.write(id_, f_((c.t + c.dt).seconds()));
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    std::string name_, signal_;
    Unit unit_;
    std::function<double(double)> f_;
    SignalId id_;
};

/// A world with one lamp on a body; `values` are its LampAPI_1 reals.
World lampWorld(std::map<std::string, double, std::less<>> values, std::string kind = "spot",
                bool load = false) {
    World w;
    const BodyId body = w.addBody("base", kWorldBody, {}, 1.0, {}, {0.1, 0.1, 0.1, 0, 0, 0});
    SchemaInstance s;
    s.schema = "LampAPI_1";
    s.prim = w.addPrim("/bot/base/beacon", "beacon");
    w.prims.body.back() = body;
    s.reals = std::move(values);
    s.tokens["lamp:kind"] = std::move(kind);
    w.schemaInstances.push_back(s);
    if (load) {
        SchemaInstance bus;
        bus.schema = "ElectricalBusAPI_1";
        bus.prim = w.addPrim("/bot/power/dc_12v", "dc_12v");
        w.schemaInstances.push_back(bus);
        SchemaInstance l;
        l.schema = "ElectricalLoadAPI_1";
        l.prim = s.prim;
        l.tokens["elec:load:model"] = "signal";
        l.relationships["elec:powerPort"] = {bus.prim};
        w.schemaInstances.push_back(l);
    }
    return w;
}

struct LampRun {
    Scheduler scheduler{SchedulerOptions{.deterministic = true}};
    double read(const std::string& name) {
        const SignalId id = scheduler.bus().find(name);
        EXPECT_TRUE(id.valid()) << name;
        return scheduler.bus().read(id);
    }
};

TEST(Lights, AnIndicatorBlinksAt1_5HzWithinOneStep) {
    // UN R48: 90 ± 30 flashes a minute; 1.5 Hz is the middle. 50 % duty, 10 ms steps.
    const World w = lampWorld({{"lamp:luminousIntensity", 50.0},
                               {"lamp:beamAngle", 1.5},
                               {"lamp:blinkFrequency", 1.5},
                               {"lamp:initialCommand", 1.0}});
    LampRun run;
    run.scheduler.add(std::make_unique<Lights>());
    run.scheduler.add(std::make_unique<Probe>("blink", "lamp/beacon/blink", units::one,
                                              [](double) { return 1.0; }));
    ASSERT_TRUE(run.scheduler.wire(WorldView(w)));
    std::vector<double> rises;
    std::vector<double> falls;
    double was = 1.0;
    for (int k = 1; k <= 400; ++k) {  // 4 s
        ASSERT_TRUE(run.scheduler.stepUntil(SimTime{k * 10'000'000LL}).ok());
        const double on = run.read("lamp/beacon/on");
        const double t = (k - 1) * 0.01;  // the start of the step that wrote it
        if (on > 0.5 && was < 0.5) {
            rises.push_back(t);
        }
        if (on < 0.5 && was > 0.5) {
            falls.push_back(t);
        }
        was = on;
    }
    ASSERT_GE(rises.size(), 5U);
    ASSERT_GE(falls.size(), 5U);
    const double period = 1.0 / 1.5;
    for (std::size_t i = 0; i < rises.size(); ++i) {
        // The n-th rise at n periods, within one step.
        EXPECT_NEAR(rises[i], std::round(rises[i] / period) * period, 0.0100001) << i;
    }
    for (std::size_t i = 0; i < falls.size(); ++i) {
        const double half = falls[i] - 0.5 * period;
        EXPECT_NEAR(half, std::round(half / period) * period, 0.0100001) << i;
    }
    EXPECT_NEAR((rises.back() - rises.front()) / static_cast<double>(rises.size() - 1), period,
                0.0100001);
}

TEST(Lights, ALampRisesWithItsTimeConstant) {
    const World w = lampWorld(
        {{"lamp:luminousIntensity", 800.0}, {"lamp:beamAngle", 0.5}, {"lamp:riseTime", 0.1}});
    LampRun run;
    run.scheduler.add(std::make_unique<Lights>());
    run.scheduler.add(std::make_unique<Probe>("command", "lamp/beacon/command", units::one,
                                              [](double t) { return t > 0.0 ? 1.0 : 0.0; }));
    ASSERT_TRUE(run.scheduler.wire(WorldView(w)));
    EXPECT_EQ(run.read("lamp/beacon/intensity"), 0.0);
    for (int k = 1; k <= 30; ++k) {
        ASSERT_TRUE(run.scheduler.stepUntil(SimTime{k * 10'000'000LL}).ok());
    }
    // The probe writes before the lamp reads (writers first), so the lamp follows the command
    // from its first step: after 30 steps of 10 ms the output is 1 − e^(−0.3/0.1) of full,
    // exactly in discrete time.
    const double expected = 800.0 * (1.0 - std::pow(std::exp(-0.01 / 0.1), 30));
    EXPECT_NEAR(run.read("lamp/beacon/intensity"), expected, 1e-9 * 800.0);
    EXPECT_NEAR(run.read("lamp/beacon/level"), expected / 800.0, 1e-12);
}

TEST(Lights, PowerAndHeatFollowTheFluxAndEfficacy) {
    const World w = lampWorld({{"lamp:luminousIntensity", 1000.0},
                               {"lamp:beamAngle", 0.6},
                               {"lamp:luminousEfficacy", 100.0},
                               {"lamp:heatFraction", 0.8},
                               {"lamp:initialCommand", 0.5}},
                              "spot", true);
    LampRun run;
    auto lights = std::make_unique<Lights>();
    const Lights* model = lights.get();
    run.scheduler.add(std::move(lights));
    ASSERT_TRUE(run.scheduler.wire(WorldView(w)));
    ASSERT_TRUE(run.scheduler.stepUntil(SimTime{100'000'000}).ok());
    const double flux = 1000.0 * effectiveSolidAngle(model->lamps()[0]);  // lm at full
    EXPECT_NEAR(run.read("load/beacon/power_demand"), 0.5 * flux / 100.0, 1e-9);
    EXPECT_NEAR(run.read("lamp/beacon/loss_power"), 0.8 * 0.5 * flux / 100.0, 1e-9);
    EXPECT_EQ(model->lamps()[0].bus, "dc_12v");
}

TEST(Lights, ALampDimsWithItsBusVoltageAndGoesOutInABrownout) {
    const World w = lampWorld({{"lamp:luminousIntensity", 100.0},
                               {"lamp:beamAngle", 0.6},
                               {"lamp:luminousEfficacy", 15.0},
                               {"lamp:voltageExponent", 3.4},
                               {"lamp:initialCommand", 1.0}},
                              "spot", true);
    LampRun run;
    run.scheduler.add(std::make_unique<Lights>());
    // 12 V at the start, 10.8 V from 0.5 s, the load unpowered from 1 s.
    run.scheduler.add(std::make_unique<Probe>("bus", "bus/dc_12v/voltage", units::V,
                                              [](double t) { return t < 0.5 ? 12.0 : 10.8; }));
    run.scheduler.add(std::make_unique<Probe>("powered", "load/beacon/powered", units::one,
                                              [](double t) { return t < 1.0 ? 1.0 : 0.0; }));
    ASSERT_TRUE(run.scheduler.wire(WorldView(w)));
    ASSERT_TRUE(run.scheduler.stepUntil(SimTime{400'000'000}).ok());
    EXPECT_NEAR(run.read("lamp/beacon/intensity"), 100.0, 1e-12);
    ASSERT_TRUE(run.scheduler.stepUntil(SimTime{800'000'000}).ok());
    EXPECT_NEAR(run.read("lamp/beacon/intensity"), 100.0 * std::pow(0.9, 3.4), 1e-9);
    ASSERT_TRUE(run.scheduler.stepUntil(SimTime{1'200'000'000}).ok());
    EXPECT_EQ(run.read("lamp/beacon/intensity"), 0.0);
    EXPECT_EQ(run.read("lamp/beacon/on"), 0.0);
    EXPECT_EQ(run.read("load/beacon/power_demand"), 0.0);
}

TEST(Lights, SubstitutesAreDarkOrConstant) {
    const World w = lampWorld({{"lamp:luminousIntensity", 100.0},
                               {"lamp:beamAngle", 0.6},
                               {"lamp:luminousEfficacy", 50.0},
                               {"lamp:initialCommand", 0.25}},
                              "spot", true);
    for (const Mode mode : {Mode::Zero, Mode::Constant}) {
        LampRun run;
        run.scheduler.add(std::make_unique<Lights>(mode));
        ASSERT_TRUE(run.scheduler.wire(WorldView(w)));
        ASSERT_TRUE(run.scheduler.stepUntil(SimTime{100'000'000}).ok());
        const double expected = mode == Mode::Zero ? 0.0 : 25.0;
        EXPECT_NEAR(run.read("lamp/beacon/intensity"), expected, 1e-12);
        if (mode == Mode::Zero) {
            EXPECT_EQ(run.read("load/beacon/power_demand"), 0.0);
            EXPECT_EQ(run.read("lamp/beacon/loss_power"), 0.0);
        }
    }
    EXPECT_EQ(Lights::substitute("constant"), Mode::Constant);
    EXPECT_EQ(Lights::substitute("zero"), Mode::Zero);
    EXPECT_FALSE(Lights::substitute("replay"));
}

TEST(Lights, MistakesAreRefusedByName) {
    std::string error;
    const World noIntensity = lampWorld({{"lamp:beamAngle", 0.5}});
    EXPECT_TRUE(lampsOf(WorldView(noIntensity), error).empty());
    EXPECT_NE(error.find("lamp beacon: lamp:luminousIntensity"), std::string::npos) << error;
    error.clear();
    const World noBeam = lampWorld({{"lamp:luminousIntensity", 5.0}});
    EXPECT_TRUE(lampsOf(WorldView(noBeam), error).empty());
    EXPECT_NE(error.find("lamp:beamAngle"), std::string::npos) << error;
    error.clear();
    const World loadNoEfficacy = lampWorld({{"lamp:luminousIntensity", 5.0}}, "point", true);
    EXPECT_TRUE(lampsOf(WorldView(loadNoEfficacy), error).empty());
    EXPECT_NE(error.find("lamp:luminousEfficacy"), std::string::npos) << error;
}

TEST(Lights, TheSameRunTwiceHasTheSameState) {
    const World w = lampWorld({{"lamp:luminousIntensity", 300.0},
                               {"lamp:beamAngle", 0.6},
                               {"lamp:riseTime", 0.05},
                               {"lamp:blinkFrequency", 2.0},
                               {"lamp:rotationSpeed", 3.0}},
                              "beacon");
    std::vector<std::uint64_t> hashes;
    for (int trial = 0; trial < 2; ++trial) {
        LampRun run;
        run.scheduler.add(std::make_unique<Lights>());
        run.scheduler.add(
            std::make_unique<Probe>("command", "lamp/beacon/command", units::one,
                                    [](double t) { return 0.5 + 0.5 * std::sin(t); }));
        run.scheduler.add(std::make_unique<Probe>("blink", "lamp/beacon/blink", units::one,
                                                  [](double t) { return t > 1.0 ? 1.0 : 0.0; }));
        ASSERT_TRUE(run.scheduler.wire(WorldView(w)));
        ASSERT_TRUE(run.scheduler.stepUntil(SimTime{3'000'000'000}).ok());
        auto hash = run.scheduler.stateHash();
        ASSERT_TRUE(hash);
        hashes.push_back(*hash);
    }
    EXPECT_EQ(hashes[0], hashes[1]);
}

}  // namespace
}  // namespace lagrange::lights
