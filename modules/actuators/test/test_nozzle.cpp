// The nozzle and tank of RFC 0022 §8 against their closed forms: the exit velocity
// C_d √(2Δp/ρ) and flow ρ A v0; the jet's landing point against the drag-free projectile's range
// at five elevations and from a height, and its limit cases (45° reaches farthest on level ground,
// complementary angles land together, straight up comes back down beside the nozzle); the
// reaction ṁ v0 on the nozzle's body; the tank's mass falling as m0 − ṁ t and its weight m g;
// the pump's Δp Q / η; the jet passing through its own robot; determinism; refusals.
#include "lagrange/actuators/Nozzles.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::actuators {
namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kG = 9.80665;

/// Axis-aligned boxes, cast against exactly (shape i is box i).
class Boxes final : public services::SceneQuery {
public:
    struct Box {
        Vec3 centre, half;
    };
    std::vector<Box> boxes;

    [[nodiscard]] services::RayHit ray(const Vec3& o, const Vec3& d, double reach) const override {
        services::RayHit best;
        for (std::uint32_t i = 0; i < boxes.size(); ++i) {
            const Box& b = boxes[i];
            double t0 = -std::numeric_limits<double>::infinity();
            double t1 = std::numeric_limits<double>::infinity();
            bool miss = false;
            for (int a = 0; a < 3; ++a) {
                const double lo = b.centre[a] - b.half[a];
                const double hi = b.centre[a] + b.half[a];
                if (std::abs(d[a]) < 1e-15) {
                    miss = miss || o[a] < lo || o[a] > hi;
                    continue;
                }
                double ta = (lo - o[a]) / d[a];
                double tb = (hi - o[a]) / d[a];
                if (ta > tb) {
                    std::swap(ta, tb);
                }
                t0 = std::max(t0, ta);
                t1 = std::min(t1, tb);
            }
            if (miss || t0 > t1 || t1 < 0.0) {
                continue;
            }
            const double t = t0 < 0.0 ? t1 : t0;
            if (t > reach || t >= best.distance) {
                continue;
            }
            best.distance = t;
            best.shape = i;
        }
        return best;
    }
    [[nodiscard]] std::vector<std::uint32_t> overlapSphere(const Vec3& c, double r) const override {
        std::vector<std::uint32_t> out;
        for (std::uint32_t i = 0; i < boxes.size(); ++i) {
            bool in = true;
            for (int a = 0; a < 3; ++a) {
                in = in && std::abs(c[a] - boxes[i].centre[a]) <= boxes[i].half[a] + r;
            }
            if (in) {
                out.push_back(i);
            }
        }
        return out;
    }
};

/// Level ground: its top at z = 0.
void addGround(Boxes& s) {
    s.boxes.push_back({{0.0, 0.0, -0.5}, {500.0, 500.0, 0.5}});
}

TEST(Nozzle, ExitVelocityAndFlowFollowBernoulli) {
    // 0.5 bar behind a 8 mm nozzle with C_d 0.95: v0 = 0.95 √(2·50 000/998.2) = 9.5085 m/s,
    // ṁ = 998.2 · π 0.004² · v0 = 0.47709 kg/s.
    NozzleParams p;
    p.diameter = 0.008;
    p.dischargeCoefficient = 0.95;
    const double v0 = exitVelocity(0.95, 50'000.0, kWaterDensity);
    EXPECT_NEAR(v0, 0.95 * std::sqrt(2.0 * 50'000.0 / 998.2), 1e-12);
    EXPECT_NEAR(v0, 9.5085, 1e-4);
    EXPECT_NEAR(nozzleArea(p), kPi * 0.004 * 0.004, 1e-15);
    const double flow = nozzleMassFlow(kWaterDensity, nozzleArea(p), v0, 1.0);
    EXPECT_NEAR(flow, 0.47709, 1e-5);
    EXPECT_NEAR(nozzleMassFlow(kWaterDensity, nozzleArea(p), v0, 0.25), 0.25 * flow, 1e-15);
    EXPECT_EQ(nozzleMassFlow(kWaterDensity, nozzleArea(p), v0, 2.0), flow);  // fully open
    EXPECT_EQ(exitVelocity(0.95, 0.0, kWaterDensity), 0.0);
    // The pump: Δp Q / η = 50 000 Pa · 4.7795e-4 m³/s / 0.5 = 47.80 W.
    EXPECT_NEAR(pumpPower(50'000.0, flow / kWaterDensity, 0.5), 50'000.0 * flow / 998.2 / 0.5,
                1e-9);
    EXPECT_EQ(pumpPower(50'000.0, 1e-3, 0.0), 0.0);
    // The landing disc widens with the spray cone: 4 mm + 5 m · tan 0.05.
    EXPECT_NEAR(footprintRadius(0.008, 0.05, 5.0), 0.004 + 5.0 * std::tan(0.05), 1e-15);
}

TEST(Jet, LandsWhereTheDragFreeProjectileDoes) {
    Boxes scene;
    addGround(scene);
    const double v = 9.5;
    const double h = 0.5;
    double worst = 0.0;
    for (const double degrees : {0.0, 15.0, 30.0, 45.0, 60.0}) {
        const double th = degrees * kPi / 180.0;
        const JetPath jet = traceJet({0.0, 0.0, h}, {v * std::cos(th), 0.0, v * std::sin(th)},
                                     {0.0, 0.0, -kG}, &scene, {});
        ASSERT_TRUE(jet.landed) << degrees;
        const double expected = ballisticRange(v, th, h, kG);
        const double error = std::abs(jet.impact[0] - expected) / expected;
        worst = std::max(worst, error);
        EXPECT_NEAR(jet.impact[2], 0.0, 1e-9);
        // Its time of flight: x = v cos θ t.
        EXPECT_NEAR(jet.time, expected / (v * std::cos(th)), 1e-3 * jet.time);
        std::printf("%4.0f deg: lands at %.4f m, drag-free closed form %.4f m (%.1e)\n", degrees,
                    jet.impact[0], expected, error);
    }
    EXPECT_LT(worst, 0.005);  // RFC 0022 §Validation: 0.5 %
}

TEST(Jet, LimitCasesOfTheProjectile) {
    Boxes scene;
    addGround(scene);
    const double v = 12.0;
    const auto range = [&](double degrees) {
        const double th = degrees * kPi / 180.0;
        const JetPath jet = traceJet({0.0, 0.0, 1e-9}, {v * std::cos(th), 0.0, v * std::sin(th)},
                                     {0.0, 0.0, -kG}, &scene, {}, JetLimits{.segment = 0.01});
        return jet.impact[0];
    };
    // Level ground: R = v² sin 2θ / g, longest at 45°, the same at complementary angles.
    EXPECT_NEAR(range(45.0), v * v / kG, 0.002 * v * v / kG);
    EXPECT_GT(range(45.0), range(40.0));
    EXPECT_GT(range(45.0), range(50.0));
    EXPECT_NEAR(range(30.0), range(60.0), 0.002 * range(30.0));
    // Straight up: it rises v²/(2g) and comes down where it left.
    const JetPath up = traceJet({0.0, 0.0, 1e-9}, {0.0, 0.0, v}, {0.0, 0.0, -kG}, &scene, {});
    ASSERT_TRUE(up.landed);
    double top = 0.0;
    for (const Vec3& p : up.points) {
        top = std::max(top, p[2]);
    }
    EXPECT_NEAR(top, v * v / (2.0 * kG), 0.01);
    EXPECT_NEAR(up.impact[0], 0.0, 1e-12);
    EXPECT_NEAR(up.time, 2.0 * v / kG, 1e-3);
    // Nothing to meet: it runs to its limit and does not land.
    const JetPath free = traceJet({0.0, 0.0, 0.0}, {v, 0.0, 0.0}, {0.0, 0.0, -kG}, nullptr, {});
    EXPECT_FALSE(free.landed);
}

TEST(Jet, PassesThroughWhatItIsToldTo) {
    Boxes scene;
    addGround(scene);
    scene.boxes.push_back({{1.0, 0.0, 0.5}, {0.1, 1.0, 0.5}});  // a wall 1 m away
    const Vec3 v{8.0, 0.0, 2.0};
    const JetPath stopped = traceJet({0.0, 0.0, 0.5}, v, {0.0, 0.0, -kG}, &scene, {});
    ASSERT_TRUE(stopped.landed);
    EXPECT_EQ(stopped.shape, 1U);
    EXPECT_NEAR(stopped.impact[0], 0.9, 1e-9);
    const JetPath through = traceJet({0.0, 0.0, 0.5}, v, {0.0, 0.0, -kG}, &scene,
                                     [](std::uint32_t s) { return s == 1; });
    ASSERT_TRUE(through.landed);
    EXPECT_EQ(through.shape, 0U);
    EXPECT_GT(through.impact[0], 1.1);
}

// ---------------------------------------------------------------------------- the module

/// A cart on level ground with a nozzle 0.5 m up pointing forward and up at `elevation`, and
/// optionally a tank. The cart has a collider round the nozzle, which the jet must pass through.
struct Rig {
    World world;
    Boxes scene;
    std::shared_ptr<Commands> commands = std::make_shared<Commands>();
    BodyId cart{};

    Rig() {
        addGround(scene);
        cart = world.addBody("cart", kWorldBody, Pose{}, 20.0, {0.0, 0.0, 0.0},
                             {1.0, 1.0, 1.0, 0.0, 0.0, 0.0});
        world.addJoint("cart_free", JointType::Free, cart, {0.0, 0.0, 1.0}, false);
        world.addCollider(kWorldBody, ShapeType::Box, {500.0, 500.0, 0.5},
                          Pose{{0.0, 0.0, -0.5}, {1, 0, 0, 0}}, 0.5);
        world.addCollider(cart, ShapeType::Box, {0.3, 0.3, 0.3},
                          Pose{{0.0, 0.0, 0.4}, {1, 0, 0, 0}}, 0.5);
        scene.boxes.push_back({{0.0, 0.0, 0.4}, {0.3, 0.3, 0.3}});
    }
    PrimId onCart(const char* name, const Vec3& at) {
        const PrimId prim = world.addPrim(std::string("/cart/") + name, name);
        world.prims.body[prim.index] = cart;
        world.prims.poseInBody[prim.index].position = at;
        return prim;
    }
    PrimId addTank(double capacity, double initial) {
        SchemaInstance s;
        s.schema = "TankAPI_1";
        s.prim = onCart("tank", {-0.2, 0.0, 0.3});
        s.reals = {{"act:tank:capacity", capacity}, {"act:tank:initial", initial}};
        const PrimId prim = s.prim;
        world.schemaInstances.push_back(std::move(s));
        return prim;
    }
    void addNozzle(double elevation, std::optional<PrimId> tank, double efficiency = 0.0) {
        SchemaInstance s;
        s.schema = "NozzleAPI_1";
        s.prim = onCart("nozzle", {0.0, 0.0, 0.5});
        s.reals = {{"act:nozzle:diameter", 0.008},
                   {"act:nozzle:dischargeCoefficient", 0.95},
                   {"act:nozzle:pressure", 50'000.0},
                   {"act:nozzle:sprayHalfAngle", 0.05},
                   {"act:nozzle:pumpEfficiency", efficiency}};
        s.realArrays["act:nozzle:axis"] = {std::cos(elevation), 0.0, std::sin(elevation)};
        if (tank) {
            s.relationships["act:nozzle:tank"] = {*tank};
        }
        world.schemaInstances.push_back(std::move(s));
    }
};

/// Writes the cart's pose at the origin, and nothing else.
class CartPose final : public IModule {
public:
    [[nodiscard]] ModuleInfo info() const override {
        return {"rigid_body", "stub", 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return Duration{1'000'000}; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec s;
        s.name = "body/cart/pose";
        s.type = SignalType::Pose;
        s.unit = units::m;
        s.frame = "world_local";
        out_ = d.output(s);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        bus.writeN(out_, std::array<double, 7>{0, 0, 0, 1, 0, 0, 0});
        return {};
    }
    Status step(const StepContext&, SignalBus& bus) override {
        bus.writeN(out_, std::array<double, 7>{0, 0, 0, 1, 0, 0, 0});
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    SignalId out_;
};

std::unique_ptr<Scheduler> run(Rig& rig, Nozzles::Mode mode = Nozzles::Mode::Full) {
    auto scheduler = std::make_unique<Scheduler>(SchedulerOptions{.deterministic = true});
    scheduler->add(std::make_unique<CartPose>());
    const Boxes* scene = &rig.scene;
    scheduler->add(
        std::make_unique<Nozzles>(mode, [scene](const SignalBus&) { return scene; }, rig.commands));
    auto wired = scheduler->wire(WorldView(rig.world));
    EXPECT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    return wired ? std::move(scheduler) : nullptr;
}

double value(const Scheduler& s, const char* name) {
    return s.bus().read(s.bus().find(name));
}
std::vector<double> values(const Scheduler& s, const char* name) {
    const auto v = s.bus().readN(s.bus().find(name));
    return {v.begin(), v.end()};
}

TEST(Nozzles, TheJetPushesBackWithMassFlowTimesVelocity) {
    Rig rig;
    const double th = 20.0 * kPi / 180.0;
    rig.addNozzle(th, std::nullopt, 0.6);
    auto s = run(rig);
    ASSERT_TRUE(s);
    ASSERT_TRUE(s->stepUntil(SimTime{100'000'000}).ok());
    EXPECT_EQ(value(*s, "act/nozzle/flow"), 0.0);  // shut
    EXPECT_TRUE(std::isnan(values(*s, "act/nozzle/impact")[0]));
    rig.commands->set("nozzle", 1.0);
    ASSERT_TRUE(s->stepUntil(SimTime{200'000'000}).ok());
    const double v0 = exitVelocity(0.95, 50'000.0, kWaterDensity);
    const double flow = kWaterDensity * kPi * 0.004 * 0.004 * v0;
    EXPECT_NEAR(value(*s, "act/nozzle/flow"), flow, 1e-12);
    EXPECT_NEAR(value(*s, "act/nozzle/velocity"), v0, 1e-12);
    const double f = flow * v0;  // ρ Q v0 = ρ A v0²
    EXPECT_NEAR(value(*s, "act/nozzle/reaction"), f, 1e-12);
    const std::vector<double> w = values(*s, "body/cart/wrench/nozzle");
    ASSERT_EQ(w.size(), 6U);
    EXPECT_NEAR(w[0], -f * std::cos(th), 1e-9 * f);
    EXPECT_NEAR(w[1], 0.0, 1e-12);
    EXPECT_NEAR(w[2], -f * std::sin(th), 1e-9 * f);
    // At the exit, 0.5 m over the centre of mass: τ = r × F, about y.
    EXPECT_NEAR(w[4], 0.5 * (-f * std::cos(th)), 1e-9 * f);
    std::printf("reaction %.4f N (rho A v0^2 = %.4f N) at %.4f kg/s\n", w[0] / -std::cos(th), f,
                flow);
    // Through its own cart, onto the ground where the closed form puts it; the disc it wets.
    const std::vector<double> impact = values(*s, "act/nozzle/impact");
    const double expected = ballisticRange(v0, th, 0.5, kG);
    EXPECT_NEAR(impact[0], expected, 0.005 * expected);
    EXPECT_EQ(value(*s, "act/nozzle/landed"), 1.0);
    EXPECT_GT(value(*s, "act/nozzle/footprint"), 0.004 + expected * std::tan(0.05));
    // The path starts at the exit and ends at the impact.
    const std::vector<double> path = values(*s, "act/nozzle/path");
    ASSERT_EQ(path.size(), 3U * kJetPathPoints);
    EXPECT_NEAR(path[2], 0.5, 1e-12);
    EXPECT_NEAR(path[3 * (kJetPathPoints - 1)], impact[0], 1e-12);
    // The pump: Δp Q / η.
    EXPECT_NEAR(value(*s, "act/nozzle/pump_power"), 50'000.0 * flow / kWaterDensity / 0.6, 1e-9);
}

TEST(Nozzles, TheTankEmptiesAtTheFlowAndWeighsWhatItHolds) {
    Rig rig;
    const PrimId tank = rig.addTank(0.002, 0.5);  // 2 L, half full: 0.9982 kg
    rig.addNozzle(0.3, tank);
    auto s = run(rig);
    ASSERT_TRUE(s);
    const double m0 = 0.001 * kWaterDensity;
    ASSERT_TRUE(s->stepUntil(SimTime{10'000'000}).ok());
    EXPECT_NEAR(value(*s, "act/nozzle/tank_mass"), m0, 1e-12);
    std::vector<double> w = values(*s, "body/cart/wrench/tank");
    EXPECT_NEAR(w[2], -m0 * kG, 1e-9);
    // Its weight at the tank, 0.2 m behind the centre of mass: τ_y = z·F_x − x·F_z = −0.2 m g.
    EXPECT_NEAR(w[4], -0.2 * m0 * kG, 1e-9);
    rig.commands->set("nozzle", 1.0);
    const double v0 = exitVelocity(0.95, 50'000.0, kWaterDensity);
    const double flow = kWaterDensity * kPi * 0.004 * 0.004 * v0;
    ASSERT_TRUE(s->stepUntil(SimTime{1'010'000'000}).ok());  // open for 1 s from 10 ms
    EXPECT_NEAR(value(*s, "act/nozzle/tank_mass"), m0 - flow * 1.0, 1e-9);
    w = values(*s, "body/cart/wrench/tank");
    EXPECT_NEAR(w[2], -(m0 - flow) * kG, 1e-9);
    std::printf("tank: %.5f kg after 1 s at %.5f kg/s from %.5f kg\n",
                value(*s, "act/nozzle/tank_mass"), flow, m0);
    // Empty at m0 / ṁ = 2.09 s; then nothing comes out and nothing pushes back.
    ASSERT_TRUE(s->stepUntil(SimTime{3'000'000'000}).ok());
    EXPECT_EQ(value(*s, "act/nozzle/tank_mass"), 0.0);
    EXPECT_EQ(value(*s, "act/nozzle/flow"), 0.0);
    EXPECT_EQ(value(*s, "act/nozzle/reaction"), 0.0);
    EXPECT_EQ(values(*s, "body/cart/wrench/tank")[2], 0.0);
    EXPECT_EQ(values(*s, "body/cart/wrench/nozzle")[0], 0.0);
}

TEST(Nozzles, AShutSubstituteNeverSprays) {
    Rig rig;
    const PrimId tank = rig.addTank(0.002, 1.0);
    rig.addNozzle(0.3, tank);
    auto s = run(rig, Nozzles::Mode::Shut);
    ASSERT_TRUE(s);
    rig.commands->set("nozzle", 1.0);
    ASSERT_TRUE(s->stepUntil(SimTime{500'000'000}).ok());
    EXPECT_EQ(value(*s, "act/nozzle/flow"), 0.0);
    EXPECT_NEAR(value(*s, "act/nozzle/tank_mass"), 0.002 * kWaterDensity, 1e-12);
}

TEST(Nozzles, SameRunSameBits) {
    const auto once = [] {
        Rig rig;
        const PrimId tank = rig.addTank(0.002, 1.0);
        rig.addNozzle(0.4, tank);
        auto s = run(rig);
        std::vector<double> out;
        for (int k = 1; k <= 40; ++k) {
            if (k == 5) {
                rig.commands->set("nozzle", 0.7);
            }
            EXPECT_TRUE(s->stepUntil(SimTime{k * 50'000'000LL}).ok());
            out.push_back(value(*s, "act/nozzle/tank_mass"));
            const std::vector<double> impact = values(*s, "act/nozzle/impact");
            out.insert(out.end(), impact.begin(), impact.end());
        }
        return out;
    };
    const std::vector<double> a = once();
    const std::vector<double> b = once();
    ASSERT_EQ(a.size(), b.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        EXPECT_EQ(std::memcmp(&a[i], &b[i], sizeof(double)), 0) << i;
    }
}

TEST(Nozzles, RefusesMissingValuesAndATankThatIsNot) {
    {
        World world;
        SchemaInstance s;
        s.schema = "NozzleAPI_1";
        s.prim = world.addPrim("/rig/nozzle", "nozzle");
        s.reals = {{"act:nozzle:diameter", 0.008}, {"act:nozzle:pressure", 1e5}};
        world.schemaInstances.push_back(s);
        std::string error;
        EXPECT_TRUE(nozzlesOf(WorldView(world), error).nozzles.empty());
        EXPECT_NE(error.find("dischargeCoefficient"), std::string::npos) << error;
    }
    {
        World world;
        SchemaInstance s;
        s.schema = "NozzleAPI_1";
        s.prim = world.addPrim("/rig/nozzle", "nozzle");
        s.reals = {{"act:nozzle:diameter", 0.008},
                   {"act:nozzle:dischargeCoefficient", 0.9},
                   {"act:nozzle:pressure", 1e5}};
        s.relationships["act:nozzle:tank"] = {world.addPrim("/rig/box", "box")};
        world.schemaInstances.push_back(s);
        std::string error;
        EXPECT_TRUE(nozzlesOf(WorldView(world), error).nozzles.empty());
        EXPECT_NE(error.find("no TankAPI_1"), std::string::npos) << error;
    }
    {
        World world;
        SchemaInstance s;
        s.schema = "NozzleAPI_1";
        s.prim = world.addPrim("/rig/nozzle", "nozzle");
        s.reals = {{"act:nozzle:diameter", 0.008},
                   {"act:nozzle:dischargeCoefficient", 0.9},
                   {"act:nozzle:pressure", 1e5}};
        s.tokens["act:nozzle:agent"] = "foam";
        world.schemaInstances.push_back(s);
        std::string error;
        EXPECT_TRUE(nozzlesOf(WorldView(world), error).nozzles.empty());
        EXPECT_NE(error.find("foamExpansion"), std::string::npos) << error;
    }
}

}  // namespace
}  // namespace lagrange::actuators
