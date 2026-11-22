// The fire domain on the bus (RFC 0022 §Validation, the checks without sensors): radiation into
// thermal nodes at five positions with one behind a wall, a node heated to its steady state with
// the thermal network, spread to a second item, the water schedule, the substitutes, and the same
// run twice.
#include "lagrange/fire/Fires.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/thermal/ThermalNetwork.hpp"

#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::fire {
namespace {

using namespace literals;

constexpr double kStefanBoltzmann = 5.670374419e-8;  // W/(m²·K⁴), CODATA 2018

/// Reads a scalar signal every 100 ms.
class Probe final : public IModule {
public:
    Probe(std::string signal, Unit unit, SignalType type = SignalType::Scalar)
        : signal_(std::move(signal)), unit_(unit), type_(type) {}
    std::vector<double> seen;
    [[nodiscard]] ModuleInfo info() const override {
        return {"test", "reader", 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return 100_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec spec;
        spec.name = signal_;
        spec.unit = unit_;
        spec.type = type_;
        id_ = d.input(spec);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus&) override { return {}; }
    Status step(const StepContext&, SignalBus& bus) override {
        seen.push_back(bus.read(id_));
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    std::string signal_;
    Unit unit_;
    SignalType type_;
    SignalId id_;
};

/// Axis-aligned boxes, exact: the fixed scenery for the occlusion rays.
class Boxes final : public services::SceneQuery {
public:
    struct Box {
        Vec3 lo, hi;
    };
    explicit Boxes(std::vector<Box> boxes) : boxes_(std::move(boxes)) {}
    [[nodiscard]] services::RayHit ray(const std::array<double, 3>& o,
                                       const std::array<double, 3>& d,
                                       double maxDistance) const override {
        services::RayHit best;
        for (std::uint32_t i = 0; i < boxes_.size(); ++i) {
            double enter = -1e300;
            double leave = 1e300;
            for (int a = 0; a < 3; ++a) {
                if (std::abs(d[a]) < 1e-15) {
                    if (o[a] < boxes_[i].lo[a] || o[a] > boxes_[i].hi[a]) {
                        enter = 1e300;
                    }
                    continue;
                }
                double t0 = (boxes_[i].lo[a] - o[a]) / d[a];
                double t1 = (boxes_[i].hi[a] - o[a]) / d[a];
                if (t0 > t1) {
                    std::swap(t0, t1);
                }
                enter = std::max(enter, t0);
                leave = std::min(leave, t1);
            }
            if (enter > leave || leave < 0.0) {
                continue;
            }
            const double at = enter >= 0.0 ? enter : leave;  // inside: where it leaves
            if (at <= maxDistance && at < best.distance) {
                best.distance = at;
                best.shape = i;
            }
        }
        return best;
    }
    [[nodiscard]] std::vector<std::uint32_t> overlapSphere(const std::array<double, 3>&,
                                                           double) const override {
        return {};
    }

private:
    std::vector<Box> boxes_;
};

/// A fuel item's values, as an example for the laws (not a fuel's data).
void putFire(SchemaInstance& s, double ignitionTime = 0.0) {
    s.schema = "FireAPI_1";
    s.reals = {{"fire:alpha", 46.889},           {"fire:hrrPerArea", 5e5},
               {"fire:fuelMass", 500.0},         {"fire:heatOfCombustion", 1.2e7},
               {"fire:radiativeFraction", 0.3},  {"fire:sootYield", 0.015},
               {"fire:criticalWaterRate", 0.03}, {"fire:ignitionTime", ignitionTime}};
}

PrimId primAt(World& w, const std::string& name, const Vec3& at) {
    const PrimId p = w.addPrim("/w/" + name, name);
    w.prims.poseInBody.at(p.index).position = at;
    return p;
}

/// A 1 m × 1 m × 0.6 m pallet at the origin on fire, and thermal nodes at `nodes`.
World palletAnd(const std::vector<Vec3>& nodes, double ignitionTime = 0.0) {
    World w;
    w.gravityLocal = {0.0, 0.0, -9.81};
    const PrimId pallet = primAt(w, "pallet", {0.0, 0.0, 0.3});
    w.addCollider(kWorldBody, ShapeType::Box, {0.5, 0.5, 0.3}, Pose{{0.0, 0.0, 0.3}, {1, 0, 0, 0}},
                  0.5);
    w.schemaInstances.push_back({});
    putFire(w.schemaInstances.back(), ignitionTime);
    w.schemaInstances.back().prim = pallet;
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        const PrimId p = primAt(w, "node" + std::to_string(i), nodes[i]);
        SchemaInstance s;
        s.schema = "ThermalAPI_1";
        s.prim = p;
        s.reals = {{"thermal:heatCapacity", 2000.0},
                   {"thermal:convectionArea", 0.5},
                   {"thermal:emissivity", 0.8},
                   {"thermal:radiativeArea", 1.0}};
        w.schemaInstances.push_back(s);
    }
    return w;
}

ModuleConfig constant() {
    ModuleConfigData data;
    data.enabled = false;
    return ModuleConfig(data);
}

// Point source at five positions, one behind a wall: each node takes ε·χ_r·Q/(4πR²)·A/4 from the
// source at half the flame's height above the pallet, 0.1 %; the hidden one takes 0. The pallet's
// own box is in the scenery and does not hide it.
TEST(Fires, RadiationIntoNodesAtFivePositionsOneHidden) {
    const std::vector<Vec3> at{
        {3.0, 0.0, 1.0}, {0.0, 5.0, 0.5}, {-4.0, -4.0, 2.0}, {0.0, -8.0, 0.2}, {-6.0, 0.0, 1.0}};
    const World world = palletAnd(at);
    // Box 0: the pallet (World collider 0); box 1: a wall between the fire and node 4.
    auto scenery = std::make_shared<Boxes>(std::vector<Boxes::Box>{
        {{-0.5, -0.5, 0.0}, {0.5, 0.5, 0.6}}, {{-3.2, -2.0, 0.0}, {-3.0, 2.0, 4.0}}});
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(
        std::make_unique<Fires>(Fires::Mode::Constant, scenery, std::vector<std::uint32_t>{0, 1}));
    std::vector<Probe*> probes;
    for (std::size_t i = 0; i < at.size(); ++i) {
        auto p = std::make_unique<Probe>("fire_radiation/node" + std::to_string(i) + "/loss_power",
                                         units::W);
        probes.push_back(p.get());
        scheduler.add(std::move(p));
    }
    auto hrr = std::make_unique<Probe>("fire/pallet/hrr", units::W);
    Probe* q = hrr.get();
    scheduler.add(std::move(hrr));
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_TRUE(wired) << to_string(wired.error().front());
    ASSERT_TRUE(scheduler.stepUntil(SimTime{1'000'000'000}).ok());
    const double full = 5e5;  // W: 500 kW/m² over 1 m²
    EXPECT_EQ(q->seen.back(), full);
    const double diameter = std::sqrt(4.0 / std::numbers::pi);
    const double flame = 0.235 * std::pow(full / 1e3, 0.4) - 1.02 * diameter;
    const Vec3 source{0.0, 0.0, 0.6 + flame / 2.0};
    for (std::size_t i = 0; i < at.size(); ++i) {
        const double r2 = std::pow(at[i][0] - source[0], 2) + std::pow(at[i][1] - source[1], 2) +
                          std::pow(at[i][2] - source[2], 2);
        const double expected = 0.8 * 0.3 * full / (4.0 * std::numbers::pi * r2) * 1.0 / 4.0;
        if (i == 4) {
            EXPECT_EQ(probes[i]->seen.back(), 0.0) << "behind the wall";
        } else {
            EXPECT_NEAR(probes[i]->seen.back(), expected, 1e-3 * expected) << i;
        }
    }
}

// A node 4 m from a steady 500 kW fire reaches the temperature of its energy balance: what it
// absorbs equals its convection plus its radiation to the sky at the ambient, 1 %.
TEST(Fires, HeatsANodeToItsSteadyState) {
    const Vec3 node{4.0, 0.0, 1.0};
    const World world = palletAnd({node});
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<Fires>(Fires::Mode::Constant), constant());
    ModuleConfigData heat;
    constexpr double kAmbient = 293.15;
    heat.values.emplace(thermal::kAmbientKey, kAmbient);
    scheduler.add(std::make_unique<thermal::ThermalNetwork>(), ModuleConfig(heat));
    auto temperature = std::make_unique<Probe>("thermal/node0/temperature", units::K);
    auto absorbed = std::make_unique<Probe>("fire_radiation/node0/loss_power", units::W);
    Probe* t = temperature.get();
    Probe* p = absorbed.get();
    scheduler.add(std::move(temperature));
    scheduler.add(std::move(absorbed));
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_TRUE(wired) << to_string(wired.error().front());
    ASSERT_TRUE(scheduler.stepUntil(SimTime{3600LL * 1'000'000'000LL}).ok());
    const double power = p->seen.back();
    ASSERT_GT(power, 0.0);
    // Solve P = h·A_c·(T − T∞) + ε·σ·A_r·(T⁴ − T∞⁴) by bisection.
    const auto balance = [&](double temp) {
        return 10.0 * 0.5 * (temp - kAmbient) +
               0.8 * kStefanBoltzmann * 1.0 * (std::pow(temp, 4) - std::pow(kAmbient, 4)) - power;
    };
    double lo = kAmbient;
    double hi = kAmbient + 2000.0;
    for (int i = 0; i < 200; ++i) {
        const double mid = 0.5 * (lo + hi);
        (balance(mid) > 0.0 ? hi : lo) = mid;
    }
    EXPECT_NEAR(t->seen.back() - kAmbient, lo - kAmbient, 0.01 * (lo - kAmbient));
}

// Spread (RFC 0022 §7): a second item with an ignition temperature and a thermal node ignites
// within a step of its node reaching that temperature, heated by the first fire.
TEST(Fires, SpreadsToAnItemWhoseNodeReachesItsIgnitionTemperature) {
    World world = palletAnd({{2.5, 0.0, 0.3}});
    for (SchemaInstance& s : world.schemaInstances) {
        if (s.schema == "FireAPI_1") {
            s.reals["fire:alpha"] = 1000.0;  // to full involvement in 22 s
        }
    }
    SchemaInstance second;
    putFire(second, -1.0);
    second.prim = world.schemaInstances.back().prim;  // node0's prim
    second.reals["fire:ignitionTemperature"] = 310.0;
    second.reals["fire:area"] = 1.0;
    world.schemaInstances.push_back(second);
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<Fires>());
    ModuleConfigData heat;
    heat.values.emplace(thermal::kAmbientKey, 293.15);
    scheduler.add(std::make_unique<thermal::ThermalNetwork>(), ModuleConfig(heat));
    auto temperature = std::make_unique<Probe>("thermal/node0/temperature", units::K);
    auto state = std::make_unique<Probe>("fire/node0/state", units::one, SignalType::Int);
    Probe* t = temperature.get();
    Probe* s = state.get();
    scheduler.add(std::move(temperature));
    scheduler.add(std::move(state));
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_TRUE(wired) << to_string(wired.error().front());
    ASSERT_TRUE(scheduler.stepUntil(SimTime{1800LL * 1'000'000'000LL}).ok());
    std::size_t hot = t->seen.size();
    std::size_t lit = s->seen.size();
    for (std::size_t i = 0; i < t->seen.size(); ++i) {
        if (hot == t->seen.size() && t->seen[i] >= 310.0) {
            hot = i;
        }
        if (lit == s->seen.size() && s->seen[i] >= 1.0) {
            lit = i;
        }
    }
    ASSERT_LT(hot, t->seen.size()) << "the node never reached 310 K";
    ASSERT_LT(lit, s->seen.size()) << "the item never ignited";
    EXPECT_GE(lit, hot);
    EXPECT_LE(lit, hot + 2);  // the node's temperature reaches the fire a step late (feedback)
}

// Water from the run's options: from 30 s on, 0.1 kg/(m²·s) stops the growth and decays it.
TEST(Fires, TheWaterScheduleDecaysTheFire) {
    const World world = palletAnd({});
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    ModuleConfigData data;
    data.values["water_schedule"] = std::vector<std::string>{"pallet=0.1@30"};
    scheduler.add(std::make_unique<Fires>(), ModuleConfig(data));
    auto hrr = std::make_unique<Probe>("fire/pallet/hrr", units::W);
    auto rate = std::make_unique<Probe>("fire/pallet/water_rate",
                                        Unit{{.length = -2, .mass = 1, .time = -1}});
    Probe* q = hrr.get();
    Probe* w = rate.get();
    scheduler.add(std::move(hrr));
    scheduler.add(std::move(rate));
    ASSERT_TRUE(scheduler.wire(WorldView(world)));
    ASSERT_TRUE(scheduler.stepUntil(SimTime{130LL * 1'000'000'000LL}).ok());
    // Steps end at 0.1, 0.2, ...: index 299 ends at 30.0 s, the last step without water.
    EXPECT_NEAR(q->seen[299], 46.889 * 30.0 * 30.0, 1e-6 * q->seen[299]);
    EXPECT_EQ(w->seen[299], 0.0);
    EXPECT_EQ(w->seen[300], 0.1);
    const double tau = 3.0 * std::pow(0.1, -1.85);
    EXPECT_NEAR(q->seen[1299], q->seen[299] * std::exp(-100.0 / tau), 1e-3 * q->seen[1299]);
    ModuleConfigData wrong;
    wrong.values["water_schedule"] = std::vector<std::string>{"crate=0.1@30"};
    Scheduler refused;
    refused.add(std::make_unique<Fires>(), ModuleConfig(wrong));
    EXPECT_FALSE(refused.wire(WorldView(world)));
}

// Zero: nothing burns. A fire missing a required value is refused with its name.
TEST(Fires, ZeroSubstituteAndRequiredValues) {
    World world = palletAnd({{3.0, 0.0, 1.0}});
    {
        Scheduler scheduler;
        scheduler.add(std::make_unique<Fires>(Fires::Mode::Zero));
        auto hrr = std::make_unique<Probe>("fire/pallet/hrr", units::W);
        Probe* q = hrr.get();
        scheduler.add(std::move(hrr));
        ASSERT_TRUE(scheduler.wire(WorldView(world)));
        ASSERT_TRUE(scheduler.stepUntil(SimTime{10'000'000'000}).ok());
        EXPECT_EQ(q->seen.back(), 0.0);
    }
    for (SchemaInstance& s : world.schemaInstances) {
        if (s.schema == "FireAPI_1") {
            s.reals.erase("fire:radiativeFraction");
        }
    }
    Scheduler scheduler;
    scheduler.add(std::make_unique<Fires>());
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_FALSE(wired);
    EXPECT_NE(to_string(wired.error().front()).find("fire pallet: fire:radiativeFraction"),
              std::string::npos)
        << to_string(wired.error().front());
}

// The same run twice gives the same numbers (invariant 7).
TEST(Fires, TheSameRunTwiceIsIdentical) {
    const World world = palletAnd({{3.0, 0.0, 1.0}});
    const auto once = [&] {
        Scheduler scheduler(SchedulerOptions{.deterministic = true});
        ModuleConfigData data;
        data.values["water_schedule"] = std::vector<std::string>{"pallet=0.05@40"};
        scheduler.add(std::make_unique<Fires>(), ModuleConfig(data));
        auto heat = std::make_unique<Probe>("fire_radiation/node0/loss_power", units::W);
        Probe* p = heat.get();
        scheduler.add(std::move(heat));
        EXPECT_TRUE(scheduler.wire(WorldView(world)));
        EXPECT_TRUE(scheduler.stepUntil(SimTime{90'000'000'000}).ok());
        return p->seen;
    };
    EXPECT_EQ(once(), once());
}

// ------------------------------------------------------------------ nozzles (RFC 0022 §8)

/// Stands in for the actuators domain: one nozzle's jet, landing at a fixed point from `from` s.
class Jet final : public IModule {
public:
    Jet(double flow, Vec3 impact, double footprint, double from)
        : flow_(flow), impact_(impact), footprint_(footprint), from_(from) {}
    [[nodiscard]] ModuleInfo info() const override {
        return {"actuators", "jet", 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return 10_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        const auto out = [&d](const char* name, SignalType type, Unit unit) {
            SignalSpec s;
            s.name = name;
            s.type = type;
            s.unit = unit;
            s.frame = type == SignalType::Vec3 ? "world_local" : "";
            return d.output(s);
        };
        flowId_ = out("act/hose/flow", SignalType::Scalar, Unit{{.mass = 1, .time = -1}});
        impactId_ = out("act/hose/impact", SignalType::Vec3, units::m);
        footprintId_ = out("act/hose/footprint", SignalType::Scalar, units::m);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        write(bus, false);
        return {};
    }
    Status step(const StepContext& c, SignalBus& bus) override {
        write(bus, c.t.seconds() + 1e-9 >= from_);
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    void write(SignalBus& bus, bool on) const {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        bus.write(flowId_, on ? flow_ : 0.0);
        bus.writeN(impactId_, on ? impact_ : Vec3{nan, nan, nan});
        bus.write(footprintId_, on ? footprint_ : 0.0);
    }
    double flow_;
    Vec3 impact_;
    double footprint_, from_;
    SignalId flowId_, impactId_, footprintId_;
};

/// The pallet with a nozzle prim of `agent` (its jet comes from Jet).
World palletAndHose(const char* agent = "water") {
    World w = palletAnd({});
    SchemaInstance s;
    s.schema = "NozzleAPI_1";
    s.prim = primAt(w, "hose", {-4.0, 0.0, 0.5});
    s.reals = {{"act:nozzle:diameter", 0.008},
               {"act:nozzle:dischargeCoefficient", 0.95},
               {"act:nozzle:pressure", 5e4},
               {"act:nozzle:foamExpansion", 8.0}};
    s.tokens["act:nozzle:agent"] = agent;
    w.schemaInstances.push_back(s);
    return w;
}

struct HoseRun {
    std::vector<double> hrr, rate, state, foam;
};
HoseRun hoseRun(const World& world, double flow, const Vec3& impact, double footprint, double from,
                double until) {
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<Fires>());
    scheduler.add(std::make_unique<Jet>(flow, impact, footprint, from));
    auto hrr = std::make_unique<Probe>("fire/pallet/hrr", units::W);
    auto rate = std::make_unique<Probe>("fire/pallet/water_rate",
                                        Unit{{.length = -2, .mass = 1, .time = -1}});
    auto state = std::make_unique<Probe>("fire/pallet/state", units::one, SignalType::Int);
    auto foam = std::make_unique<Probe>("fire/pallet/foam_coverage", units::one);
    Probe* q = hrr.get();
    Probe* w = rate.get();
    Probe* st = state.get();
    Probe* blanket = foam.get();
    scheduler.add(std::move(hrr));
    scheduler.add(std::move(rate));
    scheduler.add(std::move(state));
    scheduler.add(std::move(foam));
    const auto wired = scheduler.wire(WorldView(world));
    EXPECT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    EXPECT_TRUE(scheduler.stepUntil(SimTime{static_cast<std::int64_t>(until * 1e9)}).ok());
    return {q->seen, w->seen, st->seen, blanket->seen};
}

// The water a jet gives a fire is its flow times the share of its landing disc on the burning
// area, over that area: all of it in the middle, the lens's share on the edge, none on the floor
// beside it or on anything above it.
TEST(Fires, AJetWetsTheFireByTheShareOfItsDiscOnIt) {
    const World world = palletAndHose();
    const double area = 1.0;  // the pallet's 1 m × 1 m
    const double radius = std::sqrt(area / std::numbers::pi);
    const double flow = 0.3;
    const auto rateAt = [&](const Vec3& impact, double footprint) {
        return hoseRun(world, flow, impact, footprint, 1.0, 3.0).rate.back();
    };
    EXPECT_NEAR(rateAt({0.1, 0.0, 0.6}, 0.2), flow / area, 1e-12);
    // On the side of the fuel, halfway up: still the fuel.
    EXPECT_NEAR(rateAt({-0.5, 0.0, 0.3}, 0.0), flow / area, 1e-12);
    const double share = discOverlapArea(radius, 0.2, radius) / (std::numbers::pi * 0.04);
    EXPECT_NEAR(rateAt({radius, 0.0, 0.6}, 0.2), flow * share / area, 1e-12);
    std::printf("on the edge: %.4f of the spray lands (%.5f kg/(m2 s))\n", share,
                flow * share / area);
    EXPECT_EQ(rateAt({2.0, 0.0, 0.0}, 0.2), 0.0);  // the floor beside it
    EXPECT_EQ(rateAt({0.0, 0.0, 2.5}, 0.2), 0.0);  // a ceiling over it
}

// A jet above the critical rate decays the fire as Evans gives it, Q(t_a)·e^(−(t − t_a)/τ) with
// τ = 3.0·w''^(−1.85), and puts it out when Q is down to 1 % of Q(t_a): at τ·ln 100 after the
// water arrives, to the step.
TEST(Fires, AJetAboveTheCriticalRatePutsTheFireOutInEvansTime) {
    const World world = palletAndHose();
    const double flow = 0.3;  // kg/s over 1 m²: 0.3 mm/s, ten times the critical rate
    const double tau = 3.0 * std::pow(flow, -1.85);
    const double arrives = 30.0;
    // The fire reads the jet's previous step: on from 29.95 s, it is there at the 30.0 s step.
    const HoseRun r = hoseRun(world, flow, {0.0, 0.0, 0.6}, 0.1, arrives - 0.05, 200.0);
    // Index i ends at 0.1·(i + 1) s; the step from 30.0 is the first with water.
    const double before = r.hrr[299];
    EXPECT_NEAR(before, 46.889 * 30.0 * 30.0, 1e-6 * before);
    EXPECT_NEAR(r.hrr[349], before * std::exp(-5.0 / tau), 1e-6 * before);
    const double out = tau * std::log(100.0);
    std::size_t first = 0;
    while (first < r.state.size() && r.state[first] != 4.0) {
        ++first;
    }
    ASSERT_LT(first, r.state.size());
    const double when = 0.1 * static_cast<double>(first + 1) - arrives;
    std::printf("w'' %.2f mm/s: tau %.2f s, out after %.1f s (tau ln 100 = %.2f s)\n", flow, tau,
                when, out);
    EXPECT_NEAR(when, out, 0.1 + 1e-9);
    EXPECT_EQ(r.hrr.back(), 0.0);
    // Below the critical rate it still grows.
    const HoseRun weak = hoseRun(world, 0.01, {0.0, 0.0, 0.6}, 0.1, arrives - 0.05, 40.0);
    EXPECT_GT(weak.hrr.back(), weak.hrr[299]);
}

// A foam nozzle lays a blanket instead: no water, the coverage grows.
TEST(Fires, AFoamJetSmothers) {
    World world = palletAndHose("foam");
    for (SchemaInstance& s : world.schemaInstances) {
        if (s.schema == "FireAPI_1") {
            s.reals["fire:foamBlanketThickness"] = 0.05;
        }
    }
    const HoseRun r = hoseRun(world, 0.3, {0.0, 0.0, 0.6}, 0.1, 1.0, 5.0);
    EXPECT_EQ(r.rate.back(), 0.0);
    EXPECT_GT(r.foam.back(), 0.0);
}

}  // namespace
}  // namespace lagrange::fire
