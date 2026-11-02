// Two quadcopters, one above the other: the aerodynamics domain's downwash (rotor_aero.md §2)
// reaches the lower one's rotors through rotor/<j>/air_velocity, and its blade-element rotors
// lose thrust. Checked against Kiran, Ayanian and Breuer (2025), "Influence of static and dynamic
// downwash interactions on multi-quadrotor systems", RSS XXI (arXiv:2507.09463): stacked
// Crazyflies 4 l apart (l the centre-to-motor distance) left the lower one 65% of its hover
// thrust; beyond 19 l it had 95–98%.
#include "lagrange/aerodynamics/Aerodynamics.hpp"
#include "lagrange/fields/Field.hpp"
#include "lagrange/rotor_aero/RotorAero.hpp"
#include "lagrange/scheduler/Scheduler.hpp"

#include <cmath>
#include <cstdio>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange {
namespace {

using namespace literals;

constexpr double kRho = 1.225;     // kg/m³, ISA sea level
constexpr double kRadius = 0.127;  // m, the sample quadcopter's 10-inch propellers
constexpr double kArm = 0.25;      // m, its centre-to-motor distance (500 mm diagonal)
constexpr double kOmega = 565.0;   // rad/s, about its hover speed (5400 RPM)

/// Holds every rotor link still and spins every rotor joint at kOmega, as physics would report.
class Plant final : public IModule {
public:
    std::vector<std::string> joints, links;
    std::vector<double> spin;  // +1 ccw, −1 cw
    std::vector<std::array<double, 3>> positions;
    [[nodiscard]] ModuleInfo info() const override {
        return {"test", "plant", 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return 1_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        for (const std::string& j : joints) {
            SignalSpec v;
            v.name = "joint/" + j + "/velocity";
            v.unit = units::rad_per_s;
            velocity_.push_back(d.output(v));
        }
        for (const std::string& l : links) {
            SignalSpec p;
            p.name = "body/" + l + "/pose";
            p.type = SignalType::Pose;
            p.unit = units::m;
            p.frame = "world_local";
            pose_.push_back(d.output(p));
            SignalSpec t = p;
            t.name = "body/" + l + "/twist";
            t.type = SignalType::Wrench;
            t.unit = units::m_per_s;
            twist_.push_back(d.output(t));
        }
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
        for (std::size_t i = 0; i < velocity_.size(); ++i) {
            bus.write(velocity_[i], spin[i] * kOmega);
        }
        for (std::size_t i = 0; i < pose_.size(); ++i) {
            const auto& p = positions[i];
            bus.writeN(pose_[i], std::array<double, 7>{p[0], p[1], p[2], 1.0, 0.0, 0.0, 0.0});
            bus.writeN(twist_[i], std::array<double, 6>{});
        }
    }
    std::vector<SignalId> velocity_, pose_, twist_;
};

/// Adds a quadcopter (an X of four propellers on a root frame) at `centre`.
void addQuad(World& w, Plant& plant, const std::string& name, const std::array<double, 3>& centre) {
    const BodyId frame =
        w.addBody(name + "_frame", kWorldBody, {centre}, 1.5, {}, {0.02, 0.02, 0.04, 0, 0, 0});
    for (int i = 0; i < 4; ++i) {
        const double a = std::numbers::pi / 4.0 + i * std::numbers::pi / 2.0;
        const std::array<double, 3> at{kArm * std::cos(a), kArm * std::sin(a), 0.0};
        const std::string link = name + "_prop" + std::to_string(i);
        const BodyId prop = w.addBody(link, frame, {at}, 0.02, {}, {1e-5, 1e-5, 2e-5, 0, 0, 0});
        w.addJoint(link + "_joint", JointType::Continuous, prop, {0, 0, 1}, true);
        // Opposite corners turn the same way.
        const bool ccw = i % 2 == 0;
        w.schemaInstances.push_back({});
        SchemaInstance& rotor = w.schemaInstances.back();
        rotor.schema = "RotorAPI_1";
        rotor.prim = w.joints.prim.back();
        rotor.reals = {{"aero:rotor:radius", kRadius},
                       {"aero:rotor:thrustCoeff", 0.125},
                       {"aero:rotor:powerCoeff", 0.0507}};
        rotor.integers = {{"aero:rotor:bladeCount", 2}};
        rotor.tokens = {{"aero:rotor:direction", ccw ? "ccw" : "cw"}};
        rotor.assetPaths = {{"aero:rotor:propeller", std::string(LAGRANGE_TEST_PROPELLERS) +
                                                         "/generic_10in_2blade_" +
                                                         (ccw ? "ccw" : "cw") + ".prop.yaml"}};
        plant.joints.push_back(link + "_joint");
        plant.spin.push_back(ccw ? 1.0 : -1.0);
        plant.links.push_back(link);
        plant.positions.push_back({centre[0] + at[0], centre[1] + at[1], centre[2] + at[2]});
    }
}

/// The lower quadcopter's thrust, N, with the upper one `gap` above it (none at gap 0).
double lowerThrust(double gap, bool downwash = true) {
    World w;  // high above any ground
    auto plant = std::make_unique<Plant>();
    Plant& p = *plant;
    addQuad(w, p, "lower", {0.0, 0.0, 100.0});
    if (gap > 0.0) {
        addQuad(w, p, "upper", {0.0, 0.0, 100.0 + gap});
    }
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::move(plant));
    ModuleConfigData rotors;
    rotors.values[rotor_aero::kAirDensityKey] = kRho;
    rotors.values["fidelity"] = std::string("bem");
    rotors.values["ground_effect"] = false;
    rotors.values["rotor_air"] = downwash;
    scheduler.add(std::make_unique<rotor_aero::RotorAero>(), ModuleConfig(rotors));
    ModuleConfigData aero;
    aero.values["air_density"] = kRho;
    aero.values["downwash"] = downwash;
    scheduler.add(std::make_unique<aerodynamics::AeroDrag>(), ModuleConfig(aero));
    auto wired = scheduler.wire(WorldView(w));
    EXPECT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    if (!wired) {
        return 0.0;
    }
    EXPECT_TRUE(scheduler.stepUntil(SimTime{20'000'000}).ok());
    double total = 0.0;
    for (int i = 0; i < 4; ++i) {
        total += scheduler.bus().read(
            scheduler.bus().find("rotor/lower_prop" + std::to_string(i) + "_joint/thrust"));
    }
    return total;
}

TEST(DownwashPair, TheLowerQuadcopterLosesThrustAsMeasured) {
    const double alone = lowerThrust(0.0);
    ASSERT_GT(alone, 10.0);  // about the sample's hover thrust (2 kg)
    const double near = lowerThrust(4.0 * kArm) / alone;
    const double far = lowerThrust(19.0 * kArm) / alone;
    std::printf("alone %.2f N, at 4 l %.3f, at 19 l %.3f\n", alone, near, far);
    // Kiran et al.: 65% at 4 l (Crazyflies at constant voltage, so their rotors sped up a
    // little as the load fell: a fixed-speed rotor loses somewhat more).
    EXPECT_GT(near, 0.45);
    EXPECT_LT(near, 0.75);
    // 95–98% beyond 19 l; the mean jet is still there, so allow a little more loss.
    EXPECT_GT(far, 0.85);
    EXPECT_LT(far, 1.0);
    EXPECT_GT(far, near);
    // Off, the wake reaches nobody.
    EXPECT_NEAR(lowerThrust(4.0 * kArm, false) / alone, 1.0, 1e-9);
}

/// One quadcopter beside a wall (a 3 m thick box, its face at x = face), its rotors' thrust
/// (N) and the air going down through them (m/s), in addQuad's order.
struct Beside {
    std::vector<double> thrust, down, induced;
};
/// `onABody`: the wall on a fixed body of its own, as a URDF world's links load.
Beside besideAWall(double face, bool onABody = false) {
    World w;
    auto plant = std::make_unique<Plant>();
    addQuad(w, *plant, "quad", {0.0, 0.0, 100.0});
    if (onABody) {
        const BodyId tower = w.addBody("tower", kWorldBody, {{face + 1.5, 0.0, 50.0}}, 1000.0, {},
                                       {1.0, 1.0, 1.0, 0, 0, 0});
        w.addCollider(tower, ShapeType::Box, {1.5, 10.0, 110.0},
                      Pose{{0.0, 0.0, -50.0}, {1.0, 0.0, 0.0, 0.0}}, 1.0);
    } else {
        w.addCollider(kWorldBody, ShapeType::Box, {1.5, 10.0, 110.0},
                      Pose{{face + 1.5, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}}, 1.0);
    }
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::move(plant));
    ModuleConfigData rotors;
    rotors.values[rotor_aero::kAirDensityKey] = kRho;
    rotors.values["fidelity"] = std::string("bem");
    rotors.values["ground_effect"] = false;
    rotors.values["rotor_air"] = true;
    scheduler.add(std::make_unique<rotor_aero::RotorAero>(), ModuleConfig(rotors));
    ModuleConfigData aero;
    aero.values["air_density"] = kRho;
    scheduler.add(std::make_unique<aerodynamics::AeroDrag>(), ModuleConfig(aero));
    auto wired = scheduler.wire(WorldView(w));
    EXPECT_TRUE(wired);
    EXPECT_TRUE(scheduler.stepUntil(SimTime{20'000'000}).ok());
    Beside out;
    for (int i = 0; i < 4; ++i) {
        const std::string j = "rotor/quad_prop" + std::to_string(i) + "_joint/";
        out.thrust.push_back(scheduler.bus().read(scheduler.bus().find(j + "thrust")));
        out.down.push_back(-scheduler.bus().readN(scheduler.bus().find(j + "air_velocity"))[2]);
        out.induced.push_back(scheduler.bus().read(scheduler.bus().find(j + "induced_velocity")));
    }
    return out;
}

TEST(DownwashPair, AWallStarvesTheNearRotorsAndTheDroneRollsTowardsIt) {
    // The sample's rotor tips reach 0.177 + 0.127 = 0.304 m along x; the wall 0.25 D from them.
    constexpr double kDiameter = 2.0 * kRadius;
    constexpr double kTips = kArm * 0.70710678 + kRadius;
    const Beside near = besideAWall(kTips + 0.25 * kDiameter);
    // Props 0 and 3 are at +x, by the wall; 1 and 2 away from it.
    const double nearDown = (near.down[0] + near.down[3]) / 2.0;
    const double farDown = (near.down[1] + near.down[2]) / 2.0;
    // The CFD fit (rotor_aero.md §7): near less far, 3.4% of v_i at the wall, fading over 3.5 D.
    const double induced = (near.induced[0] + near.induced[1]) / 2.0;
    EXPECT_NEAR(nearDown - farDown, 0.0346 * std::exp(-0.25 / 3.5) * induced, 0.05);
    EXPECT_GT(nearDown, 0.0);
    EXPECT_LT(farDown, 0.0);
    // Fixed-speed rotors: less thrust by the wall, so it rolls towards it.
    EXPECT_LT(near.thrust[0], near.thrust[1]);
    EXPECT_LT(near.thrust[3], near.thrust[2]);
    // The same wall on a fixed body of its own (a URDF world's link) does the same.
    const Beside body = besideAWall(kTips + 0.25 * kDiameter, true);
    EXPECT_NEAR(body.down[0], near.down[0], 1e-9);
    EXPECT_NEAR(body.thrust[3], near.thrust[3], 1e-9);
    // Far from the wall nothing is left.
    const Beside far = besideAWall(kTips + 12.0 * kDiameter);
    EXPECT_NEAR(far.down[0], far.down[1], 1e-9);
    EXPECT_NEAR(far.thrust[0], far.thrust[1], 1e-9);
}

// The live air box (downwash_model: lbm, rotor_aero.md §8) on the CPU reference, coarse: the
// rotors of a hovering quadcopter draw air from round each other, inwards (OpenFOAM's hover case
// had about 1 m/s inwards at each disc, rotor_aero.md §7), and the box goes out as aero/air_box
// with the jet going down under the drone.
TEST(DownwashPair, TheLiveAirBoxDrawsTheRotorsTogether) {
    World w;
    auto plant = std::make_unique<Plant>();
    addQuad(w, *plant, "quad", {0.0, 0.0, 100.0});
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::move(plant));
    ModuleConfigData rotors;
    rotors.values[rotor_aero::kAirDensityKey] = kRho;
    rotors.values["fidelity"] = std::string("bem");
    rotors.values["ground_effect"] = false;
    rotors.values["rotor_air"] = true;
    scheduler.add(std::make_unique<rotor_aero::RotorAero>(), ModuleConfig(rotors));
    ModuleConfigData aero;
    aero.values["air_density"] = kRho;
    aero.values["downwash_model"] = std::string("lbm");
    aero.values["downwash_lbm_cell"] = 0.08;
    aero.values["downwash_lbm_cpu"] = true;
    scheduler.add(std::make_unique<aerodynamics::AeroDrag>(), ModuleConfig(aero));
    auto wired = scheduler.wire(WorldView(w));
    ASSERT_TRUE(wired) << to_string(wired.error().front());
    ASSERT_TRUE(scheduler.stepUntil(SimTime{600'000'000}).ok());
    const SignalBus& bus = scheduler.bus();
    for (int i = 0; i < 4; ++i) {
        const double a = std::numbers::pi / 4.0 + i * std::numbers::pi / 2.0;
        const auto air =
            bus.readN(bus.find("rotor/quad_prop" + std::to_string(i) + "_joint/air_velocity"));
        const double inwards = -(air[0] * std::cos(a) + air[1] * std::sin(a));
        std::printf("rotor %d: air %.3f %.3f %.3f, inwards %.3f m/s\n", i, air[0], air[1], air[2],
                    inwards);
        EXPECT_GT(inwards, 0.3) << i;
        EXPECT_LT(inwards, 2.5) << i;
    }
    fields::FieldSignal signal;
    const auto box = signal.read(bus, bus.find("aero/air_box"));
    ASSERT_NE(box, nullptr);
    const auto under = box->sampleVector({0.0, 0.0, 99.3});
    std::printf("0.7 m under the centre: %.2f m/s down\n", -under[2]);
    EXPECT_LT(under[2], -2.0);
}

}  // namespace
}  // namespace lagrange
