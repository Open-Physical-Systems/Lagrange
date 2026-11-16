// The grippers of RFC 0020 §4.1–§4.2 against their closed forms: a suction cup's evacuation
// (exponential approach to the generator's ultimate vacuum), its pull Δp·A, the steady pressure on
// a porous part and through an open cup's ISO 6358 leak, when its lip seals; an electromagnet's
// pull B²A/μ0 with B = μ0 N i / (2g + l/μr) at five gaps, its coil's R–L rise; the substitutes;
// and the equal and opposite forces. The scene is boxes, cast against exactly.
#include "lagrange/actuators/Grippers.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::actuators {
namespace {

constexpr double kPi = std::numbers::pi;

// A cup of 40 mm on a hose, a generator of 0.1 g/s down to 20 kPa absolute: illustrative numbers
// of the size a datasheet gives, not one product.
SuctionParams cup() {
    SuctionParams p;
    p.diameter = 0.04;
    p.volume = 2.0e-5;
    p.flow = 1.0e-4;
    p.ultimatePressure = 20'000.0;
    return p;
}

TEST(Suction, EvacuationFollowsTheClosedForm) {
    // V/(R T) dp/dt = −q0 (p − p_min)/(p_atm − p_min): p = p_min + (p_atm − p_min) e^(−t/τ).
    const SuctionParams p = cup();
    const double pAtm = kStandardPressure;
    const double tau = p.volume * (pAtm - p.ultimatePressure) / (287.05287 * 293.15 * p.flow);
    EXPECT_NEAR(evacuationTimeConstant(p, pAtm, 293.15), tau, 1e-12 * tau);
    SuctionInputs in;
    in.on = true;
    in.sealed = true;
    double pressure = pAtm;
    double worst = 0.0;
    constexpr double kDt = 0.001;
    for (int k = 1; k <= 5000; ++k) {
        pressure = stepSuctionPressure(p, pressure, kDt, in);
        const double t = k * kDt;
        const double expected =
            p.ultimatePressure + (pAtm - p.ultimatePressure) * std::exp(-t / tau);
        worst = std::max(worst, std::abs(pressure - expected) / (pAtm - expected + 1.0));
    }
    std::printf("evacuation: tau %.4f s; worst error %.2e of the vacuum drawn\n", tau, worst);
    EXPECT_LT(worst, 0.01);
}

TEST(Suction, HoldsWithTheDefinitionOfPressure) {
    const SuctionParams p = cup();
    EXPECT_NEAR(suctionArea(p), kPi * 0.02 * 0.02, 1e-15);
    // F = Δp·A: 81 325 Pa over 1 256.6 mm² is 102.2 N.
    const double f = suctionForce(p, p.ultimatePressure, kStandardPressure);
    EXPECT_NEAR(f, (kStandardPressure - 20'000.0) * kPi * 0.0004, 1e-9);
    EXPECT_NEAR(f, 102.196, 0.001);
    EXPECT_EQ(suctionForce(p, kStandardPressure + 10.0, kStandardPressure), 0.0);
}

TEST(Suction, Iso6358FlowIsChokedBelowTheCriticalRatio) {
    const double c = 1.0e-8;  // m³/(s·Pa)
    const double choked = c * 1.0e5 * kIso6358ReferenceDensity * std::sqrt(293.15 / 300.0);
    EXPECT_NEAR(iso6358MassFlow(c, 1.0e5, 0.2e5, 300.0), choked, 1e-15);
    EXPECT_NEAR(iso6358MassFlow(c, 1.0e5, 0.5e5, 300.0), choked, 1e-15);
    EXPECT_NEAR(iso6358MassFlow(c, 1.0e5, kCriticalPressureRatio * 1.0e5, 300.0), choked, 1e-15);
    // Halfway between b and 1: √(1 − 0.5²) of choked.
    const double half = 0.5 * (1.0 + kCriticalPressureRatio) * 1.0e5;
    EXPECT_NEAR(iso6358MassFlow(c, 1.0e5, half, 300.0), choked * std::sqrt(0.75), 1e-12);
    EXPECT_EQ(iso6358MassFlow(c, 1.0e5, 1.0e5, 300.0), 0.0);
    EXPECT_EQ(iso6358MassFlow(c, 1.0e5, 1.1e5, 300.0), 0.0);
    // b itself: the critical ratio of an ideal nozzle in air, γ = 1.4.
    EXPECT_NEAR(kCriticalPressureRatio, std::pow(2.0 / 2.4, 1.4 / 0.4), 1e-6);
}

TEST(Suction, APorousPartLeaksToTheDarcySteadyState) {
    SuctionParams p = cup();
    const double kappa = 2.0e-6;  // m/(Pa·s), air permeance of the part
    SuctionInputs in;
    in.on = true;
    in.sealed = true;
    in.porousConductance = kappa * suctionArea(p);
    double pressure = kStandardPressure;
    for (int k = 0; k < 20000; ++k) {
        pressure = stepSuctionPressure(p, pressure, 0.001, in);
    }
    // q0 (p − p_min)/(p_atm − p_min) = ρ_atm κ A (p_atm − p).
    const double k = p.flow / (kStandardPressure - p.ultimatePressure);
    const double kl = kStandardPressure / (287.05287 * 293.15) * kappa * suctionArea(p);
    const double expected = (k * p.ultimatePressure + kl * kStandardPressure) / (k + kl);
    std::printf("porous: %.0f Pa against %.0f Pa (airtight %.0f Pa)\n", pressure, expected,
                p.ultimatePressure);
    EXPECT_NEAR(pressure, expected, 1e-6 * expected);
    EXPECT_GT(pressure, p.ultimatePressure + 1000.0);
}

TEST(Suction, AnOpenCupSettlesWhereTheGeneratorMeetsItsLeak) {
    SuctionParams p = cup();
    p.leakConductance = 2.0e-10;  // m³/(s·Pa)
    SuctionInputs in;
    in.on = true;
    in.sealed = false;
    double pressure = kStandardPressure;
    for (int k = 0; k < 20000; ++k) {
        pressure = stepSuctionPressure(p, pressure, 0.001, in);
    }
    // Bisection of q0 (p − p_min)/(p_atm − p_min) = ṁ_ISO6358(p_atm → p).
    double lo = p.ultimatePressure;
    double hi = kStandardPressure;
    for (int i = 0; i < 200; ++i) {
        const double mid = 0.5 * (lo + hi);
        const double gen =
            p.flow * (mid - p.ultimatePressure) / (kStandardPressure - p.ultimatePressure);
        (gen > iso6358MassFlow(p.leakConductance, kStandardPressure, mid, 293.15) ? hi : lo) = mid;
    }
    std::printf("open cup: %.0f Pa against %.0f Pa\n", pressure, lo);
    EXPECT_NEAR(pressure, lo, 0.01 * (kStandardPressure - lo));
    // Without a rated leak an open cup is at atmospheric pressure.
    p.leakConductance = 0.0;
    EXPECT_EQ(stepSuctionPressure(p, 30'000.0, 0.001, in), kStandardPressure);
}

// An electromagnet: 600 turns, 1 A, two 20 mm × 20 mm poles, a 60 mm iron path in a core of
// μr 2000 and a mild-steel part (illustrative values).
MagnetParams magnet() {
    MagnetParams m;
    m.turns = 600.0;
    m.poleArea = 4.0e-4;
    m.ironPathLength = 0.06;
    m.ironPermeability = 2000.0;
    m.coilResistance = 24.0;
    m.coilInductance = 0.12;
    m.ratedVoltage = 24.0;
    return m;
}

TEST(Magnet, PullAtFiveGapsIsTheMaxwellStress) {
    const MagnetParams m = magnet();
    const double mu = 1000.0;
    for (const double gap : {0.0002, 0.0005, 0.001, 0.002, 0.004}) {
        const double reluctanceLength = 2.0 * gap + 0.03 / 2000.0 + 0.03 / mu;
        const double b = kMu0 * 600.0 * 1.0 / reluctanceLength;
        const double perPole = b * b * 4.0e-4 / (2.0 * kMu0);
        const double f = magnetForce(m, 1.0, gap, mu);
        std::printf("magnet at %.1f mm: B %.4f T, F %.3f N (closed form %.3f N)\n", gap * 1e3, b, f,
                    2.0 * perPole);
        EXPECT_NEAR(f, 2.0 * perPole, 0.01 * f);
    }
    // A part that is not ferromagnetic is not pulled; saturation caps the flux.
    EXPECT_EQ(magnetForce(m, 1.0, 0.001, 1.0), 0.0);
    MagnetParams thin = m;
    thin.saturation = 0.3;
    EXPECT_NEAR(magnetFlux(thin, 1.0, 0.0001, mu), 0.3, 1e-12);
    EXPECT_NEAR(magnetForce(thin, 1.0, 0.0001, mu), 0.09 * 4.0e-4 / kMu0, 1e-9);
}

TEST(Magnet, TheCoilRisesWithItsTimeConstant) {
    const MagnetParams m = magnet();
    const double tau = 0.12 / 24.0;
    double i = 0.0;
    const int steps = static_cast<int>(std::round(tau / 1e-4));
    for (int k = 0; k < steps; ++k) {
        i = stepCoilCurrent(m, i, 24.0, 1e-4);
    }
    EXPECT_NEAR(i, 1.0 - std::exp(-1.0), 1e-9);
}

/// Boxes, each on a body: an exact scene for the rays (outward normals, a ray starting inside
/// one reports where it leaves it, ties to the lower index).
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
            int a0 = -1;
            int a1 = -1;
            bool miss = false;
            for (int a = 0; a < 3; ++a) {
                const double lo = b.centre[a] - b.half[a];
                const double hi = b.centre[a] + b.half[a];
                if (std::abs(d[a]) < 1e-15) {
                    if (o[a] < lo || o[a] > hi) {
                        miss = true;
                    }
                    continue;
                }
                double ta = (lo - o[a]) / d[a];
                double tb = (hi - o[a]) / d[a];
                if (ta > tb) {
                    std::swap(ta, tb);
                }
                if (ta > t0) {
                    t0 = ta;
                    a0 = a;
                }
                if (tb < t1) {
                    t1 = tb;
                    a1 = a;
                }
            }
            if (miss || t0 > t1 || t1 < 0.0) {
                continue;
            }
            const bool inside = t0 < 0.0;
            const double t = inside ? t1 : t0;
            const int axis = inside ? a1 : a0;
            if (t > reach || t >= best.distance) {
                continue;
            }
            best.distance = t;
            best.shape = i;
            best.normal = {};
            best.normal.at(static_cast<std::size_t>(axis)) = d[axis] > 0.0 ? -1.0 : 1.0;
            if (inside) {
                best.normal.at(static_cast<std::size_t>(axis)) *= -1.0;
            }
        }
        return best;
    }
    [[nodiscard]] std::vector<std::uint32_t> overlapSphere(const Vec3&, double) const override {
        return {};
    }
};

Vec3 down() {
    return {0.0, 0.0, -1.0};
}

TEST(Seal, OnAFlatPartAllRound) {
    Boxes scene;
    scene.boxes.push_back({{0.0, 0.0, -0.03}, {0.05, 0.05, 0.03}});  // top at z = 0
    const std::vector<BodyId> bodies{BodyId{1}};
    const SuctionParams p = cup();
    // The lip 1 mm above the top: sealed on body 1.
    Seal s = findSeal(scene, p, {0.0, 0.0, 0.001}, down(), BodyId{0}, bodies);
    EXPECT_TRUE(s.sealed);
    EXPECT_EQ(s.body.index, 1U);
    // Pressed 1 mm in: still sealed (the rays start behind the lip).
    EXPECT_TRUE(findSeal(scene, p, {0.0, 0.0, -0.001}, down(), BodyId{0}, bodies).sealed);
    // 3 mm above: beyond sealGap.
    EXPECT_FALSE(findSeal(scene, p, {0.0, 0.0, 0.003}, down(), BodyId{0}, bodies).sealed);
    // Over the edge: part of the rim on nothing.
    EXPECT_FALSE(findSeal(scene, p, {0.045, 0.0, 0.001}, down(), BodyId{0}, bodies).sealed);
}

TEST(Seal, NeverTiltedBeyondMaxTiltNorAcrossTwoParts) {
    Boxes scene;
    scene.boxes.push_back({{0.0, 0.0, -0.03}, {0.5, 0.5, 0.03}});
    const std::vector<BodyId> one{BodyId{1}};
    const SuctionParams p = cup();  // maxTilt 0.26 rad
    // A lip whose rim stays within sealGap of the surface needs a small tilt; lift the far side
    // by giving the cup a generous gap and test the tilt rule on its own.
    SuctionParams loose = p;
    loose.sealGap = 0.05;
    const auto tilted = [](double angle) { return Vec3{std::sin(angle), 0.0, -std::cos(angle)}; };
    EXPECT_TRUE(findSeal(scene, loose, {0.0, 0.0, 0.002}, tilted(0.2), BodyId{0}, one).sealed);
    EXPECT_FALSE(findSeal(scene, loose, {0.0, 0.0, 0.002}, tilted(0.3), BodyId{0}, one).sealed);
    // Two parts side by side, the cup over their seam.
    Boxes two;
    two.boxes.push_back({{-0.05, 0.0, -0.03}, {0.05, 0.05, 0.03}});
    two.boxes.push_back({{0.05, 0.0, -0.03}, {0.05, 0.05, 0.03}});
    const std::vector<BodyId> both{BodyId{1}, BodyId{2}};
    EXPECT_FALSE(findSeal(two, p, {0.0, 0.0, 0.001}, down(), BodyId{0}, both).sealed);
    // The cup's own shape in front of the lip is skipped.
    Boxes own;
    own.boxes.push_back({{0.0, 0.0, 0.01}, {0.02, 0.02, 0.012}});  // the cup's body, to z = -0.002
    own.boxes.push_back({{0.0, 0.0, -0.03}, {0.05, 0.05, 0.03}});
    const std::vector<BodyId> mine{BodyId{0}, BodyId{1}};
    EXPECT_TRUE(findSeal(own, p, {0.0, 0.0, 0.001}, down(), BodyId{0}, mine).sealed);
    // The cup's bellows down to its lip, on a part touching it, and pressed 1 mm into it.
    for (const double top : {0.0, 0.001}) {
        Boxes pressed;
        pressed.boxes.push_back({{0.0, 0.0, 0.0075}, {0.02, 0.02, 0.0075}});  // lip at z = 0
        pressed.boxes.push_back({{0.0, 0.0, top - 0.03}, {0.05, 0.05, 0.03}});
        const Seal seal = findSeal(pressed, p, {0.0, 0.0, 0.0}, down(), BodyId{0}, mine);
        EXPECT_TRUE(seal.sealed) << top;
        EXPECT_EQ(seal.body.index, 1U);
    }
}

// The module in a scheduler: a gripper on the world, a part on a free body whose pose a stub
// writes, and the boxes as the scene.
struct Rig {
    World world;
    Boxes scene;
    std::shared_ptr<Commands> commands = std::make_shared<Commands>();
    BodyId part{};

    Rig() {
        part = world.addBody("part", kWorldBody, Pose{{0.0, 0.0, -0.03}, {1, 0, 0, 0}}, 0.2,
                             {0.0, 0.0, 0.0}, {1e-4, 1e-4, 1e-4, 0.0, 0.0, 0.0});
        world.addJoint("part_free", JointType::Free, part, {0.0, 0.0, 1.0}, false);
        world.addCollider(part, ShapeType::Box, {0.05, 0.05, 0.03}, Pose{}, 0.5);
        scene.boxes.push_back({{0.0, 0.0, -0.03}, {0.05, 0.05, 0.03}});
    }
    PrimId tool(const char* name, const Vec3& at, const Vec3& axis) {
        const PrimId prim = world.addPrim(std::string("/rig/") + name, name);
        world.prims.poseInBody[prim.index].position = at;
        (void)axis;
        return prim;
    }
    void addCup(const SuctionParams& p, const Vec3& at) {
        SchemaInstance s;
        s.schema = "SuctionGripperAPI_1";
        s.prim = tool("cup", at, down());
        s.reals = {{"act:suction:diameter", p.diameter},
                   {"act:suction:volume", p.volume},
                   {"act:suction:flow", p.flow},
                   {"act:suction:ultimatePressure", p.ultimatePressure}};
        if (p.leakConductance > 0.0) {
            s.reals["act:suction:leakConductance"] = p.leakConductance;
        }
        s.realArrays["act:suction:axis"] = {0.0, 0.0, -1.0};
        world.schemaInstances.push_back(std::move(s));
    }
    void addMagnet(const MagnetParams& m, const Vec3& at, bool permanent = false) {
        SchemaInstance s;
        s.schema = "MagnetGripperAPI_1";
        s.prim = tool("magnet", at, down());
        s.reals = {{"act:magnet:turns", m.turns},
                   {"act:magnet:poleArea", m.poleArea},
                   {"act:magnet:ironPathLength", m.ironPathLength},
                   {"act:magnet:ironPermeability", m.ironPermeability},
                   {"act:magnet:coilResistance", m.coilResistance},
                   {"act:magnet:coilInductance", m.coilInductance},
                   {"act:magnet:ratedVoltage", m.ratedVoltage}};
        s.tokens["act:magnet:kind"] = permanent ? "electropermanent" : "electro";
        s.realArrays["act:magnet:axis"] = {0.0, 0.0, -1.0};
        world.schemaInstances.push_back(std::move(s));
    }
    void material(double airPermeability, double mu) {
        SchemaInstance s;
        s.schema = "SensorMaterialAPI_1";
        s.prim = world.bodies.prim[part.index];
        s.reals = {{"sensmat:airPermeability", airPermeability},
                   {"sensmat:relativePermeability", mu}};
        world.schemaInstances.push_back(std::move(s));
    }
};

/// Writes the part's pose where it rests, and nothing else.
class PartPose final : public IModule {
public:
    explicit PartPose(Pose pose) : pose_(pose) {}
    [[nodiscard]] ModuleInfo info() const override {
        return {"rigid_body", "stub", 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return Duration{1'000'000}; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec s;
        s.name = "body/part/pose";
        s.type = SignalType::Pose;
        s.unit = units::m;
        s.frame = "world_local";
        out_ = d.output(s);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        write(bus);
        return {};
    }
    Status step(const StepContext&, SignalBus& bus) override {
        write(bus);
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    void write(SignalBus& bus) const {
        const std::array<double, 7> p{
            pose_.position[0],    pose_.position[1],    pose_.position[2],   pose_.orientation[0],
            pose_.orientation[1], pose_.orientation[2], pose_.orientation[3]};
        bus.writeN(out_, p);
    }
    Pose pose_;
    SignalId out_;
};

std::unique_ptr<Scheduler> run(Rig& rig, Grippers::Mode mode = Grippers::Mode::Full) {
    auto scheduler = std::make_unique<Scheduler>(SchedulerOptions{.deterministic = true});
    scheduler->add(std::make_unique<PartPose>(Pose{{0.0, 0.0, -0.03}, {1, 0, 0, 0}}));
    const Boxes* scene = &rig.scene;
    scheduler->add(std::make_unique<Grippers>(
        mode, [scene](const SignalBus&) { return scene; }, rig.commands));
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

TEST(Grippers, ASealedCupPullsItsPartWithDeltaPTimesA) {
    Rig rig;
    rig.addCup(cup(), {0.0, 0.0, 0.001});
    auto s = run(rig);
    ASSERT_TRUE(s);
    ASSERT_TRUE(s->stepUntil(SimTime{100'000'000}).ok());
    EXPECT_EQ(value(*s, "act/cup/sealed"), 1.0);
    EXPECT_EQ(value(*s, "act/cup/force"), 0.0);  // not commanded: at atmospheric pressure
    EXPECT_EQ(value(*s, "act/cup/held"), 0.0);
    rig.commands->set("cup", 1.0);
    // On the way down, the pull follows the pressure step by step.
    ASSERT_TRUE(s->stepUntil(SimTime{150'000'000}).ok());
    const double midway = value(*s, "act/cup/pressure");
    EXPECT_LT(midway, 90'000.0);
    EXPECT_NEAR(value(*s, "act/cup/force"), (kStandardPressure - midway) * suctionArea(cup()),
                1e-9);
    ASSERT_TRUE(s->stepUntil(SimTime{3'000'000'000}).ok());  // over 20 τ
    const double dpA = (kStandardPressure - 20'000.0) * suctionArea(cup());
    const double force = value(*s, "act/cup/force");
    std::printf("held: %.4f N against Δp·A %.4f N\n", force, dpA);
    EXPECT_NEAR(force, dpA, 1e-6 * dpA);
    EXPECT_EQ(value(*s, "act/cup/holding"), static_cast<double>(rig.part.index));
    EXPECT_EQ(value(*s, "act/cup/held"), 1.0);
    // The pull on the part is up, toward the cup, at the lip: a moment about its centre of mass
    // only across the lip's offset (none here, the lip is over it).
    const std::vector<double> w = values(*s, "body/part/wrench/suction");
    ASSERT_EQ(w.size(), 6U);
    EXPECT_NEAR(w[2], dpA, 1e-6 * dpA);
    EXPECT_NEAR(std::abs(w[0]) + std::abs(w[1]) + std::abs(w[3]) + std::abs(w[4]) + std::abs(w[5]),
                0.0, 1e-9);
    // Released: the cup vents through its generator and lets go.
    rig.commands->set("cup", 0.0);
    ASSERT_TRUE(s->stepUntil(SimTime{6'000'000'000}).ok());
    EXPECT_LT(value(*s, "act/cup/force"), 1e-6 * dpA);
}

TEST(Grippers, APorousPartHoldsLess) {
    Rig rig;
    rig.addCup(cup(), {0.0, 0.0, 0.001});
    rig.material(2.0e-6, 1.0);
    auto s = run(rig);
    ASSERT_TRUE(s);
    rig.commands->set("cup", 1.0);
    ASSERT_TRUE(s->stepUntil(SimTime{5'000'000'000}).ok());
    const SuctionParams p = cup();
    const double k = p.flow / (kStandardPressure - p.ultimatePressure);
    const double kl = kStandardPressure / (287.05287 * 293.15) * 2.0e-6 * suctionArea(p);
    const double expected = (k * p.ultimatePressure + kl * kStandardPressure) / (k + kl);
    EXPECT_NEAR(value(*s, "act/cup/pressure"), expected, 1e-6 * expected);
    EXPECT_NEAR(value(*s, "act/cup/force"), (kStandardPressure - expected) * suctionArea(p), 1e-6);
}

TEST(Grippers, SubstitutesHoldAtOnceOrNever) {
    Rig rig;
    rig.addCup(cup(), {0.0, 0.0, 0.001});
    {
        auto s = run(rig, Grippers::Mode::Ideal);
        ASSERT_TRUE(s);
        rig.commands->set("cup", 1.0);
        ASSERT_TRUE(s->stepUntil(SimTime{2'000'000}).ok());
        EXPECT_NEAR(value(*s, "act/cup/force"), (kStandardPressure - 20'000.0) * suctionArea(cup()),
                    1e-9);
        EXPECT_EQ(value(*s, "act/cup/pressure"), 20'000.0);
    }
    {
        auto s = run(rig, Grippers::Mode::Never);
        ASSERT_TRUE(s);
        rig.commands->set("cup", 1.0);
        ASSERT_TRUE(s->stepUntil(SimTime{1'000'000'000}).ok());
        EXPECT_EQ(value(*s, "act/cup/force"), 0.0);
        EXPECT_EQ(value(*s, "act/cup/held"), 0.0);
        EXPECT_FALSE(s->bus().find("body/part/wrench/suction").valid());
    }
}

TEST(Grippers, AMagnetPullsAcrossTheGapItMeasures) {
    for (const double gap : {0.0005, 0.001, 0.002, 0.004, 0.008}) {
        Rig rig;
        MagnetParams m = magnet();
        rig.addMagnet(m, {0.0, 0.0, gap});
        rig.material(0.0, 1000.0);
        auto s = run(rig);
        ASSERT_TRUE(s);
        rig.commands->set("magnet", 1.0);
        ASSERT_TRUE(s->stepUntil(SimTime{200'000'000}).ok());  // 40 τ of the coil
        const double i = value(*s, "act/magnet/current");
        EXPECT_NEAR(i, 1.0, 1e-9);
        EXPECT_NEAR(value(*s, "act/magnet/gap"), gap, 1e-9);
        const double b = kMu0 * 600.0 * i / (2.0 * gap + 0.03 / 2000.0 + 0.03 / 1000.0);
        const double expected = b * b * 4.0e-4 / kMu0;
        const double f = value(*s, "act/magnet/force");
        EXPECT_NEAR(f, expected, 0.01 * expected) << gap;
        const std::vector<double> w = values(*s, "body/part/wrench/magnet");
        EXPECT_NEAR(w[2], f, 1e-9 * f);
        if (gap != 0.002) {  // held: within 2 mm
            EXPECT_EQ(value(*s, "act/magnet/held"), gap < 0.002 ? 1.0 : 0.0);
        }
        EXPECT_NEAR(value(*s, "act/magnet/power"), 24.0, 1e-6);
    }
}

TEST(Grippers, AnElectropermanentMagnetHoldsUnpowered) {
    Rig rig;
    rig.addMagnet(magnet(), {0.0, 0.0, 0.001}, true);
    rig.material(0.0, 1000.0);
    auto s = run(rig);
    ASSERT_TRUE(s);
    rig.commands->set("magnet", 1.0);
    ASSERT_TRUE(s->stepUntil(SimTime{10'000'000}).ok());
    EXPECT_EQ(value(*s, "act/magnet/current"), 0.0);
    EXPECT_NEAR(value(*s, "act/magnet/force"), magnetForce(magnet(), 1.0, 0.001, 1000.0), 1e-9);
    rig.commands->set("magnet", 0.0);
    ASSERT_TRUE(s->stepUntil(SimTime{20'000'000}).ok());
    EXPECT_EQ(value(*s, "act/magnet/force"), 0.0);
    // Aluminium (μr 1) is not pulled.
    Rig alu;
    alu.addMagnet(magnet(), {0.0, 0.0, 0.001});
    auto a = run(alu);
    ASSERT_TRUE(a);
    alu.commands->set("magnet", 1.0);
    ASSERT_TRUE(a->stepUntil(SimTime{200'000'000}).ok());
    EXPECT_EQ(value(*a, "act/magnet/force"), 0.0);
}

TEST(Grippers, SameRunSameBits) {
    const auto once = [] {
        Rig rig;
        SuctionParams p = cup();
        p.leakConductance = 2.0e-10;
        rig.addCup(p, {0.0, 0.0, 0.001});
        auto s = run(rig);
        std::vector<double> out;
        for (int k = 1; k <= 50; ++k) {
            if (k == 5) {
                rig.commands->set("cup", 1.0);
            }
            EXPECT_TRUE(s->stepUntil(SimTime{k * 10'000'000LL}).ok());
            out.push_back(value(*s, "act/cup/pressure"));
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

TEST(Grippers, RefusesAMissingDatasheetValue) {
    World world;
    SchemaInstance s;
    s.schema = "SuctionGripperAPI_1";
    s.prim = world.addPrim("/rig/cup", "cup");
    s.reals = {{"act:suction:diameter", 0.04}};
    world.schemaInstances.push_back(s);
    std::string error;
    EXPECT_TRUE(grippersOf(WorldView(world), error).empty());
    EXPECT_NE(error.find("act:suction:volume"), std::string::npos) << error;
}

}  // namespace
}  // namespace lagrange::actuators
