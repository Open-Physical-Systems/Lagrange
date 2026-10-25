// The rotor_aero domain against the closed forms of aerodynamics.md §2: the coefficient model,
// the Cheeseman–Bennett ground effect, the first-order inflow lag, and the wrench a rotor puts on
// its link.
#include "lagrange/rotor_aero/RotorAero.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <cmath>
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
    // Lever arm: the force acts at the link's origin, 2 cm from its centre of mass.
    const std::array<double, 3> com = rotate(rig.plant->pose.orientation, {0.02, 0.0, 0.0});
    const std::array<double, 3> f{w[0], w[1], w[2]};
    EXPECT_NEAR(w[3], -(com[1] * f[2] - com[2] * f[1]), 1e-9);
    EXPECT_NEAR(w[4], -(com[2] * f[0] - com[0] * f[2]), 1e-9);
    EXPECT_NEAR(w[5], -(com[0] * f[1] - com[1] * f[0]), 1e-9);
    // The shaft torque loads the joint against its rotation.
    EXPECT_LT(rig.read("joint/prop_joint/load_torque"), 0.0);
    EXPECT_EQ(rig.read("joint/prop_joint/load_torque"), rig.read("rotor/prop_joint/aero_torque"));
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
        *load = rig.read("joint/prop_joint/load_torque");
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

TEST(RotorAeroBem, RefusesARotorWithoutItsPropellerFile) {
    const World world = propellerRig();
    Rig rig = makeRig(world, bem());
    auto wired = rig.scheduler->wire(WorldView(world));
    ASSERT_FALSE(wired);
    EXPECT_NE(to_string(wired.error().front()).find("propeller file"), std::string::npos);
}

}  // namespace
}  // namespace lagrange::rotor_aero
