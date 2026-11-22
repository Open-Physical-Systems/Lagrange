// The rotor_aero domain against the closed forms of aerodynamics.md §2: the coefficient model,
// the Cheeseman–Bennett ground effect, the first-order inflow lag, and the wrench a rotor puts on
// its link.
#include "lagrange/rotor_aero/RotorAero.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::rotor_aero {
namespace {

using namespace literals;

constexpr double kRho = 1.225;     // kg/m³, ISA sea level: the test's site
constexpr double kRadius = 0.127;  // m, a 10-inch propeller
constexpr double kCt = 0.11;
constexpr double kCp = 0.045;

TEST(RotorAero, StaticThrustAndTorqueFollowTheCoefficients) {
    const RotorParams p{kRadius, kCt, kCp, 1.0};
    const double omega = 600.0;  // rad/s
    const double n = omega / (2.0 * std::numbers::pi);
    const double d = 2.0 * kRadius;
    const Load load = staticLoad(p, omega, kRho);
    EXPECT_NEAR(load.thrust, kCt * kRho * n * n * std::pow(d, 4), 1e-12);
    EXPECT_NEAR(load.torque, -kCp * kRho * n * n * std::pow(d, 5) / (2.0 * std::numbers::pi),
                1e-12);
    // Power = torque × speed = CP·ρ·n³·D⁵.
    EXPECT_NEAR(-load.torque * omega, kCp * kRho * n * n * n * std::pow(d, 5), 1e-9);
    // A cw rotor makes thrust turning the other way; the drag torque always opposes rotation.
    const Load cw = staticLoad({kRadius, kCt, kCp, -1.0}, -omega, kRho);
    EXPECT_NEAR(cw.thrust, load.thrust, 1e-12);
    EXPECT_NEAR(cw.torque, -load.torque, 1e-12);
    EXPECT_LT(staticLoad(p, -omega, kRho).thrust, 0.0);  // reversed, it pushes the other way
    EXPECT_EQ(staticLoad(p, 0.0, kRho).thrust, 0.0);
}

TEST(RotorAero, GroundEffectFollowsCheesemanBennett) {
    for (const double zOverR : {0.5, 0.75, 1.0, 2.0, 3.0}) {
        const double expected = 1.0 / (1.0 - std::pow(1.0 / (4.0 * zOverR), 2));
        EXPECT_NEAR(groundEffect(kRadius, zOverR * kRadius), expected, 1e-12) << zOverR;
    }
    EXPECT_NEAR(groundEffect(kRadius, 3.0 * kRadius), 1.007, 1e-3);  // fades out with height
    // Below half a radius the factor holds (the formula diverges at a quarter radius).
    EXPECT_EQ(groundEffect(kRadius, 0.1 * kRadius), groundEffect(kRadius, 0.5 * kRadius));
}

/// Writes a rotor joint's speed and its link's pose, as physics would.
class Plant final : public IModule {
public:
    Plant(std::string joint, std::string link) : joint_(std::move(joint)), link_(std::move(link)) {}
    double omega{0.0};
    Pose pose{{0.0, 0.0, 10.0}, {1.0, 0.0, 0.0, 0.0}};
    std::array<double, 6> twist{};  // world frame: linear, angular
    [[nodiscard]] ModuleInfo info() const override {
        return {"test", "plant", 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return 1_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec v;
        v.name = "joint/" + joint_ + "/velocity";
        v.unit = units::rad_per_s;
        velocity_ = d.output(v);
        SignalSpec p;
        p.name = "body/" + link_ + "/pose";
        p.type = SignalType::Pose;
        p.unit = units::m;
        p.frame = "world_local";
        pose_ = d.output(p);
        SignalSpec t = p;
        t.name = "body/" + link_ + "/twist";
        t.type = SignalType::Wrench;
        t.unit = units::m_per_s;
        twist_ = d.output(t);
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
        bus.write(velocity_, omega);
        bus.writeN(pose_,
                   std::array<double, 7>{pose.position[0], pose.position[1], pose.position[2],
                                         pose.orientation[0], pose.orientation[1],
                                         pose.orientation[2], pose.orientation[3]});
        bus.writeN(twist_, twist);
    }
    std::string joint_, link_;
    SignalId velocity_, pose_, twist_;
};

/// A frame with one propeller on a continuous joint (axis +z), its centre of mass 2 cm off the
/// axis, over a ground plane at z = 0.
World propellerRig(const std::string& direction = "ccw", const std::string& propeller = {},
                   const std::string& mount = "puller") {
    World w;
    w.addCollider(kWorldBody, ShapeType::Plane, {0, 0, 0}, Pose{}, 1.0);
    const BodyId frame =
        w.addBody("frame", kWorldBody, Pose{}, 1.0, {}, {0.01, 0.01, 0.01, 0, 0, 0});
    const BodyId prop = w.addBody("prop", frame, {{0.2, 0.0, 0.05}}, 0.02, {0.02, 0.0, 0.0},
                                  {1e-5, 1e-5, 2e-5, 0, 0, 0});
    w.addJoint("prop_joint", JointType::Continuous, prop, {0, 0, 1}, true);
    w.schemaInstances.push_back({});
    SchemaInstance& rotor = w.schemaInstances.back();
    rotor.schema = "RotorAPI_1";
    rotor.prim = w.joints.prim.back();
    rotor.reals = {{"aero:rotor:radius", kRadius},
                   {"aero:rotor:thrustCoeff", kCt},
                   {"aero:rotor:powerCoeff", kCp}};
    rotor.integers = {{"aero:rotor:bladeCount", 2}};
    rotor.tokens = {{"aero:rotor:direction", direction}, {"aero:rotor:mount", mount}};
    if (!propeller.empty()) {
        rotor.assetPaths = {{"aero:rotor:propeller", propeller}};
    }
    return w;
}

struct Rig {
    std::unique_ptr<Scheduler> scheduler;
    Plant* plant{nullptr};
    RotorAero* rotor{nullptr};
    [[nodiscard]] double read(const std::string& name) const {
        return scheduler->bus().read(scheduler->bus().find(name));
    }
    [[nodiscard]] std::vector<double> wrench() const {
        const auto v = scheduler->bus().readN(scheduler->bus().find("body/prop/wrench/rotor"));
        return {v.begin(), v.end()};
    }
};

Rig makeRig(const World& /*world*/, ModuleConfigData config, bool ideal = false) {
    Rig rig;
    rig.scheduler = std::make_unique<Scheduler>(SchedulerOptions{.deterministic = true});
    auto plant = std::make_unique<Plant>("prop_joint", "prop");
    rig.plant = plant.get();
    rig.scheduler->add(std::move(plant));
    auto rotor = std::make_unique<RotorAero>(ideal);
    rig.rotor = rotor.get();
    rig.scheduler->add(std::move(rotor), ModuleConfig(std::move(config)));
    return rig;
}

ModuleConfigData withDensity() {
    ModuleConfigData config;
    config.values[kAirDensityKey] = kRho;
    return config;
}

TEST(RotorAero, ThePropellerPushesItsLinkAlongTheTiltedAxis) {
    const World world = propellerRig();
    Rig rig = makeRig(world, withDensity());
    auto wired = rig.scheduler->wire(WorldView(world));
    ASSERT_TRUE(wired) << to_string(wired.error().front());
    EXPECT_EQ(rig.rotor->rotors(), std::vector<std::string>{"prop_joint"});
    rig.plant->omega = 600.0;
    // Tilted 30° about x, high above the ground (no ground effect to speak of).
    const double half = 15.0 * std::numbers::pi / 180.0;
    rig.plant->pose = {{0.0, 0.0, 100.0}, {std::cos(half), std::sin(half), 0.0, 0.0}};
    ASSERT_TRUE(rig.scheduler->stepUntil(SimTime{10'000'000}).ok());
    const double thrust =
        staticLoad({kRadius, kCt, kCp, 1.0}, 600.0, kRho).thrust * groundEffect(kRadius, 100.0);
    EXPECT_NEAR(rig.read("rotor/prop_joint/thrust"), thrust, 1e-9);
    const std::vector<double> w = rig.wrench();
    const std::array<double, 3> axis = rotate(rig.plant->pose.orientation, {0.0, 0.0, 1.0});
    for (std::size_t i = 0; i < 3; ++i) {
        EXPECT_NEAR(w[i], thrust * axis[i], 1e-9) << i;
    }
    // Lever arm: the force acts at the link's origin, 2 cm from its centre of mass; and the
    // air's shaft torque, against the rotation, about the joint axis, on the propeller alone
    // (not a joint torque, whose reaction would reach the frame).
    const double shaft = rig.read("rotor/prop_joint/aero_torque");
    EXPECT_LT(shaft, 0.0);
    EXPECT_EQ(rig.read("joint/prop_joint/load_torque"), 0.0);
    const std::array<double, 3> com = rotate(rig.plant->pose.orientation, {0.02, 0.0, 0.0});
    const std::array<double, 3> f{w[0], w[1], w[2]};
    EXPECT_NEAR(w[3], -(com[1] * f[2] - com[2] * f[1]) + shaft * axis[0], 1e-9);
    EXPECT_NEAR(w[4], -(com[2] * f[0] - com[0] * f[2]) + shaft * axis[1], 1e-9);
    EXPECT_NEAR(w[5], -(com[0] * f[1] - com[1] * f[0]) + shaft * axis[2], 1e-9);
    // Momentum theory: v = sqrt(T / (2ρA)).
    EXPECT_NEAR(rig.read("rotor/prop_joint/induced_velocity"),
                std::sqrt(thrust / (2.0 * kRho * std::numbers::pi * kRadius * kRadius)), 1e-9);
}

TEST(RotorAero, NearTheGroundThrustRisesAsCheesemanBennettSays) {
    const World world = propellerRig();
    Rig rig = makeRig(world, withDensity());
    ASSERT_TRUE(rig.scheduler->wire(WorldView(world)));
    rig.plant->omega = 600.0;
    rig.plant->pose = {{0.0, 0.0, kRadius}, {1.0, 0.0, 0.0, 0.0}};  // z/R = 1
    ASSERT_TRUE(rig.scheduler->stepUntil(SimTime{5'000'000}).ok());
    const double free = staticLoad({kRadius, kCt, kCp, 1.0}, 600.0, kRho).thrust;
    EXPECT_NEAR(rig.read("rotor/prop_joint/thrust") / free, 1.0 / (1.0 - 1.0 / 16.0), 1e-9);
}

TEST(RotorAero, OverWaterTheGroundEffectIsTakenFromItsSurface) {
    // A pond whose surface is 1 m over the ground plane: Cheeseman–Bennett with z measured to
    // the water, R above it; beside the pond, to the ground.
    World world = propellerRig();
    world.schemaInstances.push_back({});
    SchemaInstance& water = world.schemaInstances.back();
    water.schema = "WaterAPI_1";
    water.prim = world.addPrim("/pond", "pond");
    world.prims.poseInBody.at(water.prim.index).position = {0.0, 0.0, 1.0};
    water.reals = {{"water:sizeX", 4.0}, {"water:sizeY", 4.0}, {"water:depth", 1.0}};
    const double free = staticLoad({kRadius, kCt, kCp, 1.0}, 600.0, kRho).thrust;
    for (const double zOverR : {0.5, 1.0, 2.0}) {
        Rig rig = makeRig(world, withDensity());
        ASSERT_TRUE(rig.scheduler->wire(WorldView(world)));
        rig.plant->omega = 600.0;
        rig.plant->pose = {{0.0, 0.0, 1.0 + zOverR * kRadius}, {1.0, 0.0, 0.0, 0.0}};
        ASSERT_TRUE(rig.scheduler->stepUntil(SimTime{5'000'000}).ok());
        const double ratio = 1.0 / (4.0 * zOverR);
        EXPECT_NEAR(rig.read("rotor/prop_joint/thrust") / free, 1.0 / (1.0 - ratio * ratio), 1e-9)
            << zOverR;
    }
    Rig beside = makeRig(world, withDensity());
    ASSERT_TRUE(beside.scheduler->wire(WorldView(world)));
    beside.plant->omega = 600.0;
    beside.plant->pose = {{3.0, 0.0, kRadius}, {1.0, 0.0, 0.0, 0.0}};
    ASSERT_TRUE(beside.scheduler->stepUntil(SimTime{5'000'000}).ok());
    EXPECT_NEAR(beside.read("rotor/prop_joint/thrust") / free, 1.0 / (1.0 - 1.0 / 16.0), 1e-9);
}

// Marine thrusters (hydrodynamics.md, "thrusters as rotors in water"): the open-water chart's
// T = K_T(J)·ρ·n²·D⁴ and Q = K_Q(J)·ρ·n²·D⁵ at J = V_a/(n·D), in the water's density over the
// share of the disc under the surface (a circular segment).
RotorParams charted() {
    RotorParams p{kRadius, kCt, kCp, 1.0};
    p.advance = {0.0, 0.4, 0.8};
    p.thrustTable = {0.40, 0.28, 0.10};  // an illustrative chart, of a B-series' shape
    p.torqueTable = {0.050, 0.040, 0.020};
    return p;
}

TEST(Thruster, TheChartGivesThrustAndTorqueAtTheAdvanceRatio) {
    const RotorParams p = charted();
    constexpr double kWater = 1025.0;
    const double omega = 2.0 * std::numbers::pi * 20.0;  // 20 rev/s
    const double d = 2.0 * kRadius;
    for (const double j : {0.0, 0.2, 0.6, 1.2}) {
        const double axial = j * 20.0 * d;
        const Load l = advanceLoad(p, omega, kWater, axial);
        const double kt = j <= 0.4   ? 0.40 + (0.28 - 0.40) * j / 0.4
                          : j <= 0.8 ? 0.28 + (0.10 - 0.28) * (j - 0.4) / 0.4
                                     : 0.10;
        const double kq = j <= 0.4   ? 0.050 + (0.040 - 0.050) * j / 0.4
                          : j <= 0.8 ? 0.040 + (0.020 - 0.040) * (j - 0.4) / 0.4
                                     : 0.020;
        EXPECT_NEAR(advanceRatio(p, omega, axial), j, 1e-12);
        EXPECT_NEAR(l.thrust, kt * kWater * 400.0 * std::pow(d, 4), 1e-9) << "J " << j;
        EXPECT_NEAR(l.torque, -kq * kWater * 400.0 * std::pow(d, 5), 1e-9) << "J " << j;
    }
    // Without a chart, the static coefficients.
    const RotorParams plain{kRadius, kCt, kCp, 1.0};
    EXPECT_EQ(advanceLoad(plain, omega, kWater, 3.0).thrust,
              staticLoad(plain, omega, kWater).thrust);
}

TEST(Thruster, TheWettedShareIsTheDiscsSegmentUnderTheSurface) {
    EXPECT_NEAR(immersion(0.0, 0.1), 0.5, 1e-12);
    EXPECT_NEAR(immersion(0.1, 0.1), 1.0, 1e-12);
    EXPECT_NEAR(immersion(-0.1, 0.1), 0.0, 1e-12);
    // Half the radius under: the segment (π − 2·acos(½) + 2·½·√(1 − ¼))/(2π)... from below.
    const double s = 0.5;
    EXPECT_NEAR(immersion(0.05, 0.1),
                0.5 + (std::asin(s) + s * std::sqrt(1.0 - s * s)) / std::numbers::pi, 1e-12);
    EXPECT_NEAR(immersion(0.05, 0.1), 0.8045, 1e-4);  // tabulated segment share
    EXPECT_EQ(immersion(0.01, 0.0), 1.0);             // a horizontal disc: in or out
    EXPECT_EQ(immersion(-0.01, 0.0), 0.0);
}

World pondRig(double density) {
    World world = propellerRig();
    world.schemaInstances.push_back({});
    SchemaInstance& water = world.schemaInstances.back();
    water.schema = "WaterAPI_1";
    water.prim = world.addPrim("/pond", "pond");
    world.prims.poseInBody.at(water.prim.index).position = {0.0, 0.0, 3.0};
    water.reals = {{"water:sizeX", 6.0},
                   {"water:sizeY", 6.0},
                   {"water:depth", 3.0},
                   {"water:density", density}};
    return world;
}

TEST(Thruster, UnderWaterItPushesWithTheWatersDensity) {
    constexpr double kSea = 1025.0;
    World world = pondRig(kSea);
    const double air = staticLoad({kRadius, kCt, kCp, 1.0}, 300.0, kRho).thrust;
    Rig rig = makeRig(world, withDensity());
    ASSERT_TRUE(rig.scheduler->wire(WorldView(world)));
    rig.plant->omega = 300.0;
    rig.plant->pose = {{0.0, 0.0, 1.5}, {1.0, 0.0, 0.0, 0.0}};  // 1.5 m down, the disc level
    ASSERT_TRUE(rig.scheduler->stepUntil(SimTime{5'000'000}).ok());
    EXPECT_NEAR(rig.read("rotor/prop_joint/thrust") / air, kSea / kRho, 1e-9);
    EXPECT_EQ(rig.read("rotor/prop_joint/immersion"), 1.0);
    // Its axis level (the disc upright) with the hub at the surface: half in.
    Rig half = makeRig(world, withDensity());
    ASSERT_TRUE(half.scheduler->wire(WorldView(world)));
    half.plant->omega = 300.0;
    const double c = std::sqrt(0.5);
    half.plant->pose = {{0.0, 0.0, 3.0}, {c, 0.0, c, 0.0}};  // z axis turned onto x
    ASSERT_TRUE(half.scheduler->stepUntil(SimTime{5'000'000}).ok());
    EXPECT_NEAR(half.read("rotor/prop_joint/immersion"), 0.5, 1e-9);
    EXPECT_NEAR(half.read("rotor/prop_joint/thrust") / air, (0.5 * kSea + 0.5 * kRho) / kRho, 1e-9);
}

TEST(Thruster, MovingThroughTheWaterItFollowsTheChart) {
    constexpr double kFresh = 1000.0;
    World world = pondRig(kFresh);
    SchemaInstance* rotor = nullptr;
    for (SchemaInstance& s : world.schemaInstances) {
        if (s.schema == "RotorAPI_1") {
            rotor = &s;
        }
    }
    ASSERT_NE(rotor, nullptr);
    rotor->realArrays = {{"aero:rotor:advanceRatios", {0.0, 0.4, 0.8}},
                         {"aero:rotor:thrustCoeffs", {0.40, 0.28, 0.10}},
                         {"aero:rotor:torqueCoeffs", {0.050, 0.040, 0.020}}};
    Rig rig = makeRig(world, withDensity());
    ASSERT_TRUE(rig.scheduler->wire(WorldView(world)));
    const double n = 15.0;
    const double d = 2.0 * kRadius;
    rig.plant->omega = 2.0 * std::numbers::pi * n;
    rig.plant->pose = {{0.0, 0.0, 1.0}, {1.0, 0.0, 0.0, 0.0}};
    rig.plant->twist = {0.0, 0.0, 0.6 * n * d, 0.0, 0.0, 0.0};  // J = 0.6 along the thrust
    ASSERT_TRUE(rig.scheduler->stepUntil(SimTime{5'000'000}).ok());
    EXPECT_NEAR(rig.read("rotor/prop_joint/advance_ratio"), 0.6, 1e-9);
    EXPECT_NEAR(rig.read("rotor/prop_joint/thrust"), 0.19 * kFresh * n * n * std::pow(d, 4), 1e-9);
    EXPECT_NEAR(rig.read("rotor/prop_joint/aero_torque"), -0.030 * kFresh * n * n * std::pow(d, 5),
                1e-9);
    // A chart whose columns differ in length stops the start.
    rotor->realArrays["aero:rotor:torqueCoeffs"] = {0.05, 0.04};
    Rig bad = makeRig(world, withDensity());
    EXPECT_FALSE(bad.scheduler->wire(WorldView(world)));
}

// Hsiao and Chirarattananon (2019): the momentum balance under a ceiling (their eq. 5 with
// eq. 9's α0) solved for γ, and the thrust at the same power γ^(2/3) of the free thrust.
TEST(RotorAero, CeilingEffectFollowsHsiaoAndChirarattananon) {
    for (const double dOverR : {0.25, 0.5, 1.0, 2.0}) {
        const double delta = 1.0 / dOverR;
        // ½u² − u − α0·δ²/16 = 0, u = v∞/v_i = 2γ.
        const double u = 1.0 + std::sqrt(1.0 + kCeilingAsymmetry * delta * delta / 8.0);
        EXPECT_NEAR(0.5 * u * u - u - kCeilingAsymmetry * delta * delta / 16.0, 0.0, 1e-12);
        EXPECT_NEAR(ceilingEffect(kRadius, dOverR * kRadius), std::cbrt(u * u / 4.0), 1e-12);
    }
    EXPECT_NEAR(ceilingEffect(kRadius, 0.5 * kRadius), 1.111, 1e-3);  // half a radius: +11%
    EXPECT_NEAR(ceilingEffect(kRadius, 2.0 * kRadius), 1.008, 1e-3);  // fades out
    EXPECT_EQ(ceilingEffect(kRadius, 6.0 * kRadius), 1.0);
}

TEST(RotorAero, UnderAFixedBoxThrustRisesAsTheCeilingEffectSays) {
    World world = propellerRig();
    // A slab 2 m up, the rotor at 2 m − 0.5 R under its underside, far above the ground.
    world.addCollider(kWorldBody, ShapeType::Box, {2.0, 2.0, 0.1},
                      Pose{{0.0, 0.0, 2.1}, {1.0, 0.0, 0.0, 0.0}}, 1.0);
    Rig rig = makeRig(world, withDensity());
    ASSERT_TRUE(rig.scheduler->wire(WorldView(world)));
    rig.plant->omega = 600.0;
    rig.plant->pose = {{0.0, 0.0, 2.0 - 0.5 * kRadius}, {1.0, 0.0, 0.0, 0.0}};
    ASSERT_TRUE(rig.scheduler->stepUntil(SimTime{5'000'000}).ok());
    const double free = staticLoad({kRadius, kCt, kCp, 1.0}, 600.0, kRho).thrust;
    const double ground = groundEffect(kRadius, 2.0 - 0.5 * kRadius);
    EXPECT_NEAR(rig.read("rotor/prop_joint/thrust") / free,
                ground * ceilingEffect(kRadius, 0.5 * kRadius), 1e-9);
}

TEST(RotorAero, ThrustLagsTheSpeedByTheInflowTimeConstant) {
    const World world = propellerRig();
    ModuleConfigData config = withDensity();
    config.values["inflow_time_constant"] = 0.05;  // s
    config.values["ground_effect"] = false;
    Rig rig = makeRig(world, std::move(config));
    ASSERT_TRUE(rig.scheduler->wire(WorldView(world)));
    rig.plant->omega = 600.0;
    ASSERT_TRUE(rig.scheduler->stepUntil(SimTime{50'000'000}).ok());  // one time constant
    const double full = staticLoad({kRadius, kCt, kCp, 1.0}, 600.0, kRho).thrust;
    // The speed reaches the rotor one physics step late (a feedback input), so the response
    // is 1 − e^(−(τ − 1 ms)/τ) at one time constant.
    EXPECT_NEAR(rig.read("rotor/prop_joint/thrust") / full, 1.0 - std::exp(-0.049 / 0.05), 2e-3);
    ASSERT_TRUE(rig.scheduler->stepUntil(SimTime{500'000'000}).ok());
    EXPECT_NEAR(rig.read("rotor/prop_joint/thrust") / full, 1.0, 1e-4);
}

TEST(RotorAero, TheIdealSubstituteHasNoLagAndNoGroundEffect) {
    const World world = propellerRig("cw");
    ModuleConfigData config = withDensity();
    config.values["inflow_time_constant"] = 0.05;  // ignored by the substitute
    Rig rig = makeRig(world, std::move(config), true);
    ASSERT_TRUE(rig.scheduler->wire(WorldView(world)));
    EXPECT_TRUE(rig.rotor->info().isSubstitute);
    rig.plant->omega = -600.0;  // a cw rotor
    rig.plant->pose = {{0.0, 0.0, kRadius}, {1.0, 0.0, 0.0, 0.0}};
    ASSERT_TRUE(rig.scheduler->stepUntil(SimTime{3'000'000}).ok());
    EXPECT_NEAR(rig.read("rotor/prop_joint/thrust"),
                staticLoad({kRadius, kCt, kCp, -1.0}, -600.0, kRho).thrust, 1e-9);
}

TEST(RotorAero, RefusesWithoutTheAirDensityAndOnAFixedJoint) {
    const World world = propellerRig();
    Rig noDensity = makeRig(world, ModuleConfigData{});
    auto wired = noDensity.scheduler->wire(WorldView(world));
    ASSERT_FALSE(wired);
    EXPECT_NE(to_string(wired.error().front()).find("air density"), std::string::npos);

    World fixed = propellerRig();
    fixed.joints.type.back() = JointType::Fixed;
    Rig onFixed = makeRig(fixed, withDensity());
    wired = onFixed.scheduler->wire(WorldView(fixed));
    ASSERT_FALSE(wired);
    EXPECT_NE(to_string(wired.error().front()).find("revolute or continuous"), std::string::npos)
        << to_string(wired.error().front());
}

/// A 10-inch two-blade propeller of the given hand for the bem tests: constant 4.5-inch pitch,
/// a thin cambered section. Made up for the test.
std::string propellerFile(const std::string& hand) {
    const std::string path = testing::TempDir() + "/test_" + hand + ".prop.yaml";
    std::ofstream out(path);
    out << "format: lagrange-propeller-1\nname: test 10x4.5 " << hand
        << "\ndiameter: 0.254\nblades: 2\nhand: " << hand
        << "\naerofoils:\n  thin: {lift_slope: 5.7, zero_lift_angle: -3, cl_max: 1.1, "
           "cl_min: -0.5, cd_min: 0.02, cl_at_cd_min: 0.3, cd_lift: 0.02}\nstations:\n";
    for (int i = 0; i <= 8; ++i) {
        const double r = 0.15 + 0.85 * i / 8.0;
        const double pitch = 0.1143;  // m
        const double twist =
            std::atan(pitch / (2.0 * std::numbers::pi * r * kRadius)) * 180.0 / std::numbers::pi;
        out << "  - {r: " << r << ", chord: " << 0.16 - 0.08 * r << ", twist: " << twist
            << ", aerofoil: thin}\n";
    }
    return path;
}

ModuleConfigData bem() {
    ModuleConfigData config = withDensity();
    config.values["fidelity"] = std::string("bem");
    config.values["ground_effect"] = false;
    return config;
}

double bemThrust(const World& world, double omega, double climb = 0.0,
                 std::vector<double>* wrench = nullptr, double* load = nullptr) {
    Rig rig = makeRig(world, bem());
    auto wired = rig.scheduler->wire(WorldView(world));
    EXPECT_TRUE(wired) << to_string(wired.error().front());
    EXPECT_EQ(rig.rotor->info().name, "bem");
    rig.plant->omega = omega;
    rig.plant->twist = {0.0, 0.0, climb, 0.0, 0.0, 0.0};
    EXPECT_TRUE(rig.scheduler->stepUntil(SimTime{3'000'000}).ok());
    if (wrench != nullptr) {
        *wrench = rig.wrench();
    }
    if (load != nullptr) {
        // The air's shaft torque about the joint axis (applied on the propeller, in its wrench).
        EXPECT_EQ(rig.read("joint/prop_joint/load_torque"), 0.0);
        *load = rig.read("rotor/prop_joint/aero_torque");
    }
    return rig.read("rotor/prop_joint/thrust");
}

TEST(RotorAeroBem, ThrustAndTorqueComeFromTheBlades) {
    const std::string file = propellerFile("ccw");
    std::string error;
    const Propeller p = loadPropeller(file, error);
    ASSERT_TRUE(error.empty()) << error;
    const BemLoad expected = PropellerTable::build(p, false).at(600.0, 0.0, {kRho, 0.0});
    std::vector<double> w;
    double load = 0.0;
    EXPECT_NEAR(bemThrust(propellerRig("ccw", file), 600.0, 0.0, &w, &load), expected.thrust, 1e-9);
    EXPECT_GT(expected.thrust, 2.0);  // a narrow 10-inch propeller at 5700 rpm: a few newtons
    EXPECT_NEAR(w[2], expected.thrust, 1e-9);
    EXPECT_NEAR(load, -expected.torque, 1e-12);  // against the rotation
    // Climbing, the blades meet the air at a smaller angle: less thrust.
    EXPECT_LT(bemThrust(propellerRig("ccw", file), 600.0, 8.0), 0.8 * expected.thrust);
}

TEST(RotorAeroBem, APusherThrustsAgainstItsJointAxis) {
    const std::string file = propellerFile("ccw");
    const double puller = bemThrust(propellerRig("ccw", file), 600.0);
    // Motor in front, propeller behind: the joint axis points back (from motor to propeller) and
    // the motor turns ccw seen from the front, which is −600 rad/s about that axis.
    std::vector<double> w;
    double load = 0.0;
    EXPECT_NEAR(bemThrust(propellerRig("ccw", file, "pusher"), -600.0, 0.0, &w, &load), puller,
                1e-9);
    EXPECT_NEAR(w[2], -puller, 1e-9);  // along −z: against the joint axis
    EXPECT_GT(load, 0.0);              // against the rotation, which is negative about the axis
}

TEST(RotorAeroBem, APropellerOfTheWrongHandGivesLittleThrust) {
    const double right = bemThrust(propellerRig("ccw", propellerFile("ccw")), 600.0);
    // A cw propeller on the ccw motor: it turns backwards, trailing edge first.
    const double wrong = bemThrust(propellerRig("ccw", propellerFile("cw")), 600.0);
    EXPECT_LT(std::abs(wrong), 0.5 * right);
    // The same propeller on a cw motor is right again.
    EXPECT_NEAR(bemThrust(propellerRig("cw", propellerFile("cw")), -600.0), right, 1e-9);
}

TEST(RotorAeroBem, FoldingBladesHangStoppedAndConeSpinning) {
    const std::string file =
        std::string(LAGRANGE_PROPELLERS) + "/generic_10in_folding_ccw.prop.yaml";
    const World world = propellerRig("ccw", file);
    Rig rig = makeRig(world, bem());
    auto wired = rig.scheduler->wire(WorldView(world));
    ASSERT_TRUE(wired) << to_string(wired.error().front());
    ASSERT_TRUE(rig.scheduler->stepUntil(SimTime{2'000'000}).ok());
    EXPECT_NEAR(rig.read("rotor/prop_joint/blade_angle"), -85.0 * std::numbers::pi / 180.0, 1e-9);
    EXPECT_EQ(rig.read("rotor/prop_joint/thrust"), 0.0);
    rig.plant->omega = 600.0;
    ASSERT_TRUE(rig.scheduler->stepUntil(SimTime{5'000'000}).ok());
    const double beta = rig.read("rotor/prop_joint/blade_angle");
    EXPECT_GT(beta, 0.0);
    std::string error;
    const BemLoad open =
        PropellerTable::build(loadPropeller(file, error), false).at(600.0, 0.0, {kRho, 0.0});
    EXPECT_NEAR(rig.read("rotor/prop_joint/thrust"), open.thrust * std::cos(beta), 1e-9);
}

/// Writes the air's velocity, as the aerodynamics domain does.
class Wind final : public IModule {
public:
    std::array<double, 3> velocity{};
    [[nodiscard]] ModuleInfo info() const override {
        return {"test", "wind", 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return 1_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec v;
        v.name = "env/wind/velocity";
        v.type = SignalType::Vec3;
        v.unit = units::m_per_s;
        v.frame = "world_local";
        id_ = d.output(v);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        bus.writeN(id_, velocity);
        return {};
    }
    Status step(const StepContext&, SignalBus& bus) override {
        bus.writeN(id_, velocity);
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    SignalId id_;
};

/// The rotor's wrench with the hub moving at `motion` through air moving at `air`.
std::vector<double> edgewiseWrench(const std::array<double, 3>& motion,
                                   const std::array<double, 3>& air, double* thrust = nullptr,
                                   const std::string& hand = "ccw") {
    const World world = propellerRig(hand, propellerFile(hand));
    ModuleConfigData config = bem();
    config.values["wind"] = true;
    Rig rig = makeRig(world, config);
    auto wind = std::make_unique<Wind>();
    Wind* w = wind.get();
    rig.scheduler->add(std::move(wind));
    auto wired = rig.scheduler->wire(WorldView(world));
    EXPECT_TRUE(wired) << to_string(wired.error().front());
    rig.plant->omega = hand == "ccw" ? 600.0 : -600.0;
    rig.plant->twist = {motion[0], motion[1], motion[2], 0.0, 0.0, 0.0};
    w->velocity = air;
    EXPECT_TRUE(rig.scheduler->stepUntil(SimTime{3'000'000}).ok());
    if (thrust != nullptr) {
        *thrust = rig.read("rotor/prop_joint/thrust");
    }
    return rig.wrench();
}

TEST(RotorAeroBem, FlyingThroughStillAirIsBeingStillInTheWind) {
    double moving = 0.0;
    double still = 0.0;
    const std::vector<double> a = edgewiseWrench({6.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, &moving);
    const std::vector<double> b = edgewiseWrench({0.0, 0.0, 0.0}, {-6.0, 0.0, 0.0}, &still);
    for (std::size_t k = 0; k < 6; ++k) {
        EXPECT_NEAR(a[k], b[k], 1e-12) << k;
    }
    double hover = 0.0;
    (void)edgewiseWrench({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, &hover);
    EXPECT_GT(moving, hover);  // translational lift
    EXPECT_LT(a[0], 0.0);      // the H-force drags against the motion
    EXPECT_NEAR(a[1], 0.0, 0.05 * std::abs(a[0]));
    // The mirror image: a cw propeller's H-force is the same, its rolling moment (about the
    // direction of flight here) the other way, so counter-rotating pairs cancel it.
    const std::vector<double> cw = edgewiseWrench({6.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, nullptr, "cw");
    EXPECT_NEAR(cw[0], a[0], 1e-9);
    // About x only the hub moment acts (the centre of mass is off along x): the advancing blade
    // rolls a ccw propeller one way and a cw one the other.
    EXPECT_GT(std::abs(a[3]), 1e-4);
    EXPECT_NEAR(cw[3], -a[3], 1e-9);
}

TEST(RotorAeroBem, RefusesARotorWithoutItsPropellerFile) {
    const World world = propellerRig();
    Rig rig = makeRig(world, bem());
    auto wired = rig.scheduler->wire(WorldView(world));
    ASSERT_FALSE(wired);
    EXPECT_NE(to_string(wired.error().front()).find("propeller file"), std::string::npos);
}

// Blade damage (rotor_aero.md §10): the shares of thrust and torque a rotor keeps, and the mass
// imbalance's force, against their closed forms. The imbalance is checked against the lost
// piece summed as point masses (de Alvear Cárdenas and de Visser 2024, eq. 7: F = m·r_cg·ω²).
TEST(BladeDamage, SharesAndImbalanceFollowTheirClosedForms) {
    // A tip of 20 % off one of two blades: that blade keeps 0.8³ of its thrust, 0.8⁴ of its torque.
    EXPECT_NEAR(damagedThrustShare(0.2, 2), 1.0 - (1.0 - 0.512) / 2.0, 1e-15);
    EXPECT_NEAR(damagedTorqueShare(0.2, 2), 1.0 - (1.0 - 0.4096) / 2.0, 1e-15);
    EXPECT_EQ(damagedThrustShare(0.0, 3), 1.0);
    EXPECT_NEAR(damagedThrustShare(1.0, 2), 0.5, 1e-15) << "a blade gone: half the thrust";
    EXPECT_NEAR(damagedThrustShare(1.0, 3), 2.0 / 3.0, 1e-15);
    // The thrust share by blade elements (∝ r²) against thrust ∝ area (the cruder bound): the
    // tip carries more than its area's share.
    EXPECT_LT(damagedThrustShare(0.2, 2), 1.0 - 0.2 / 2.0);

    const double bladeMass = 0.0026;  // kg
    const double omega = 600.0;       // rad/s
    for (const double d : {0.05, 0.2, 0.5, 1.0}) {
        // The lost piece, from (1−d)R to R, as 10⁵ point masses: Σ m_i·r_i·ω².
        constexpr int kPieces = 100'000;
        double sum = 0.0;
        for (int i = 0; i < kPieces; ++i) {
            const double r = kRadius * (1.0 - d + d * (i + 0.5) / kPieces);
            sum += bladeMass * d / kPieces * r * omega * omega;
        }
        EXPECT_NEAR(imbalanceForce(bladeMass, kRadius, d, omega), sum, 1e-9 * sum) << d;
    }
    // 20 % of a 2.6 g blade at 600 rad/s: 0.52 g at 0.9 R, about 21 N.
    EXPECT_NEAR(imbalanceForce(bladeMass, kRadius, 0.2, omega),
                0.2 * bladeMass * 0.9 * kRadius * omega * omega, 1e-12);
    // The lost thrust's moment about the hub: ∫ (T/B)·3r²/R³ · r dr over the lost tip.
    const double thrust = 5.0;  // N
    double moment = 0.0;
    constexpr int kSteps = 100'000;
    for (int i = 0; i < kSteps; ++i) {
        const double r = kRadius * (0.8 + 0.2 * (i + 0.5) / kSteps);
        moment +=
            thrust / 2.0 * 3.0 * r * r / (kRadius * kRadius * kRadius) * r * kRadius * 0.2 / kSteps;
    }
    EXPECT_NEAR(thrustAsymmetryMoment(thrust, kRadius, 0.2, 2), moment, 1e-9 * moment);
    // A propeller of the reference's size: its blade is its blades' share of its 11.9 g.
    EXPECT_NEAR(estimatedBladeMass(kReferencePropellerRadius, 2),
                kReferencePropellerMass * kBladeMassShare / 2.0, 1e-15);
    EXPECT_NEAR(estimatedBladeMass(kReferencePropellerRadius / 2.0, 2),
                estimatedBladeMass(kReferencePropellerRadius, 2) / 8.0, 1e-15);
}

/// Writes rotor/<j>/damage, as a branch strike would.
class Cutter final : public IModule {
public:
    explicit Cutter(std::string joint) : joint_(std::move(joint)) {}
    double damage{0.0};
    [[nodiscard]] ModuleInfo info() const override {
        return {"test", "cutter", 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return 1_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec s;
        s.name = "rotor/" + joint_ + "/damage";
        s.unit = units::one;
        out_ = d.output(s);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        bus.write(out_, damage);
        return {};
    }
    Status step(const StepContext&, SignalBus& bus) override {
        bus.write(out_, damage);
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    std::string joint_;
    SignalId out_;
};

// A damaged rotor in the module: its thrust and torque keep their shares, and its link takes the
// imbalance, drag and thrust asymmetry, turning with it (the lost piece fixed in the link's frame).
TEST(BladeDamage, TheLinkTakesTheTurningImbalance) {
    const World world = propellerRig();
    ModuleConfigData config = withDensity();
    config.values["ground_effect"] = false;
    Rig rig = makeRig(world, std::move(config));
    auto cutter = std::make_unique<Cutter>("prop_joint");
    Cutter* cut = cutter.get();
    rig.scheduler->add(std::move(cutter));
    auto wired = rig.scheduler->wire(WorldView(world));
    ASSERT_TRUE(wired) << to_string(wired.error().front());
    constexpr double kOmega = 600.0;
    constexpr double kDamage = 0.2;
    rig.plant->omega = kOmega;
    ASSERT_TRUE(rig.scheduler->stepUntil(SimTime{3'000'000}).ok());
    const Load whole = staticLoad({kRadius, kCt, kCp, 1.0}, kOmega, kRho);
    EXPECT_NEAR(rig.read("rotor/prop_joint/thrust"), whole.thrust, 1e-12) << "whole: unchanged";
    const std::vector<double> before = rig.wrench();
    EXPECT_EQ(before[0], 0.0);
    EXPECT_EQ(before[1], 0.0);

    cut->damage = kDamage;
    const double bladeMass = estimatedBladeMass(kRadius, 2);
    const double mass = imbalanceForce(bladeMass, kRadius, kDamage, kOmega);
    const double drag = dragAsymmetryForce(std::abs(whole.torque), kRadius, kDamage, 2);
    const double lift = thrustAsymmetryMoment(whole.thrust, kRadius, kDamage, 2);
    for (const double angle : {0.0, 0.7, 2.0}) {  // the link turned about its axis (z)
        rig.plant->pose = {{0.0, 0.0, 10.0}, {std::cos(angle / 2), 0.0, 0.0, std::sin(angle / 2)}};
        ASSERT_TRUE(rig.scheduler->stepUntil(SimTime{rig.scheduler->now().ns + 3'000'000}).ok());
        EXPECT_NEAR(rig.read("rotor/prop_joint/thrust"),
                    whole.thrust * damagedThrustShare(kDamage, 2), 1e-12);
        EXPECT_NEAR(rig.read("rotor/prop_joint/aero_torque"),
                    whole.torque * damagedTorqueShare(kDamage, 2), 1e-12);
        // The damaged blade lies along the link's x; it moves along its y (ccw, ω > 0).
        const std::array<double, 3> blade{std::cos(angle), std::sin(angle), 0.0};
        const std::array<double, 3> motion{-std::sin(angle), std::cos(angle), 0.0};
        const std::vector<double> w = rig.wrench();
        for (std::size_t k = 0; k < 2; ++k) {
            EXPECT_NEAR(w[k], -mass * blade[k] + drag * motion[k], 1e-9) << angle << " " << k;
        }
        EXPECT_NEAR(w[2], whole.thrust * damagedThrustShare(kDamage, 2), 1e-12);
        // The moment: the force's at the link's origin about its centre of mass (2 cm along the
        // link's x), the lost thrust's (blade × axis = (sin, −cos, 0) times −lift), the shaft's.
        const std::array<double, 3> com{0.02 * std::cos(angle), 0.02 * std::sin(angle), 0.0};
        const std::array<double, 3> lever{std::sin(angle), -std::cos(angle), 0.0};
        const double shaft = rig.read("rotor/prop_joint/aero_torque");
        EXPECT_NEAR(w[3], -(com[1] * w[2] - com[2] * w[1]) - lift * lever[0], 1e-9);
        EXPECT_NEAR(w[4], -(com[2] * w[0] - com[0] * w[2]) - lift * lever[1], 1e-9);
        EXPECT_NEAR(w[5], -(com[0] * w[1] - com[1] * w[0]) + shaft, 1e-9);
    }
    std::printf(
        "20 %% of a %.2f g blade off at %.0f rad/s: imbalance %.2f N, drag %.3f N, "
        "thrust moment %.4f N·m; thrust %.3f -> %.3f N\n",
        bladeMass * 1e3, kOmega, mass, drag, lift, whole.thrust,
        rig.read("rotor/prop_joint/thrust"));
}

// At the bem fidelity a damaged blade costs the same shares of the blades' thrust and torque,
// and shakes the link the same way.
TEST(BladeDamage, TheBemFidelityKeepsTheSameShares) {
    const std::string file = propellerFile("ccw");
    const World world = propellerRig("ccw", file);
    Rig rig = makeRig(world, bem());
    auto cutter = std::make_unique<Cutter>("prop_joint");
    cutter->damage = 0.3;
    rig.scheduler->add(std::move(cutter));
    ASSERT_TRUE(rig.scheduler->wire(WorldView(world)));
    rig.plant->omega = 600.0;
    ASSERT_TRUE(rig.scheduler->stepUntil(SimTime{3'000'000}).ok());
    std::string error;
    const BemLoad whole =
        PropellerTable::build(loadPropeller(file, error), false).at(600.0, 0.0, {kRho, 0.0});
    EXPECT_NEAR(rig.read("rotor/prop_joint/thrust"), whole.thrust * damagedThrustShare(0.3, 2),
                1e-9);
    EXPECT_NEAR(rig.read("rotor/prop_joint/aero_torque"),
                -whole.torque * damagedTorqueShare(0.3, 2), 1e-12);
    const std::vector<double> w = rig.wrench();
    const double mass = imbalanceForce(estimatedBladeMass(kRadius, 2), kRadius, 0.3, 600.0);
    const double drag = dragAsymmetryForce(whole.torque, kRadius, 0.3, 2);
    EXPECT_NEAR(std::hypot(w[0], w[1]), std::hypot(mass, drag), 1e-9);
}

}  // namespace
}  // namespace lagrange::rotor_aero
