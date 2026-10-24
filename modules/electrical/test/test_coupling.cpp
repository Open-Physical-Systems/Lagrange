// Coupling stability between the electrical network and physics (architecture.md §8): a DC motor
// driven at its voltage limit turns a flywheel stepped at 1 ms, for inertias from 1e-6 to
// 1e-2 kg·m², against the same drive and flywheel integrated together at 1 µs.
#include "lagrange/electrical/Network.hpp"
#include "lagrange/scheduler/Scheduler.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::electrical {
namespace {

using namespace literals;

constexpr double kBusVolts = 51.2;  // 16 cells on a flat 3.2 V curve
constexpr double kResistance = 0.5;
constexpr double kInductance = 0.0005;
constexpr double kTorqueConstant = 0.05;
constexpr double kCommand = 1e3;  // N·m: far beyond what the bus allows, so the drive saturates

MotorParams motor() {
    MotorParams p;
    p.dq = false;
    p.resistance = kResistance;
    p.inductanceD = p.inductanceQ = kInductance;
    p.torqueConstant = kTorqueConstant;
    p.maxCurrent = 1e3;  // above the stall current: the voltage limits
    return p;
}

SchemaInstance& add(World& w, std::string schema, PrimId prim) {
    w.schemaInstances.push_back({});
    SchemaInstance& s = w.schemaInstances.back();
    s.schema = std::move(schema);
    s.prim = prim;
    return s;
}

/// A rotor of inertia `inertia` on a hinge, driven by a brushed DC motor on a stiff 51.2 V bus.
World rig(double inertia) {
    World w;
    const BodyId base = w.addBody("base", kWorldBody, Pose{}, 10.0, {}, {1, 1, 1, 0, 0, 0});
    const BodyId rotor =
        w.addBody("rotor", base, Pose{}, 0.1, {}, {inertia, inertia, inertia, 0, 0, 0});
    w.addJoint("rotor_joint", JointType::Continuous, rotor, {0, 0, 1}, true);
    const PrimId joint = w.joints.prim.back();
    const PrimId bus = w.addPrim("/rig/bus", "bus");
    add(w, "ElectricalBusAPI_1", bus).reals = {{"elec:bus:capacitance", 0.0047},
                                               {"elec:bus:undervoltageTrip", 10.0},
                                               {"elec:bus:overvoltageTrip", 100.0}};
    const PrimId pack = w.addPrim("/rig/battery", "battery");
    SchemaInstance& b = add(w, "BatteryAPI_1", pack);
    b.integers = {{"elec:battery:seriesCells", 16}};
    b.reals = {{"elec:battery:capacity", 1e9},
               {"elec:battery:r0", 1e-7},  // a stiff source: the bus stays at 51.2 V
               {"elec:battery:cutoffVoltage", 1.0},
               {"elec:battery:maxChargeCurrent", 1e3}};
    b.realArrays = {{"elec:battery:ocvSoc", {0.0, 1.0}}, {"elec:battery:ocvVoltage", {3.2, 3.2}}};
    SchemaInstance& port = add(w, "PortAPI_1", pack);
    port.instance = "out";
    port.relationships = {{"elec:port:out:connectedTo", {bus}}};
    SchemaInstance& m = add(w, "ElectricalMotorAPI_1", w.addPrim("/rig/motor", "motor"));
    m.tokens = {{"elec:motorType", "dc_brushed"}};
    m.reals = {{"elec:windingResistance", kResistance},
               {"elec:inductance", kInductance},
               {"elec:torqueConstant", kTorqueConstant},
               {"elec:maxCurrent", 1e3}};
    m.relationships = {{"elec:joint", {joint}}, {"elec:powerPort", {bus}}};
    return w;
}

/// The command, constant from t = 0.
class Command final : public IModule {
public:
    [[nodiscard]] ModuleInfo info() const override {
        return {"test", "command", 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return 1_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec spec;
        spec.name = "joint/rotor_joint/torque_command";
        spec.unit = units::N_m;
        id_ = d.output(spec);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        bus.write(id_, kCommand);
        return {};
    }
    Status step(const StepContext&, SignalBus& bus) override {
        bus.write(id_, kCommand);
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    SignalId id_;
};

/// Physics in miniature at 1 ms, as MuJoCo exchanges: the torque averaged over its step, the
/// velocity and the effective inertia out. Records ω and the motor current each step.
class Flywheel final : public IModule {
public:
    explicit Flywheel(double inertia) : inertia_(inertia) {}
    std::vector<double> speeds, currents;
    [[nodiscard]] ModuleInfo info() const override {
        return {"test", "flywheel", 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return 1_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec torque;
        torque.name = "joint/rotor_joint/torque";
        torque.unit = units::N_m;
        torque.hold = Hold::WindowAverage;
        torque_ = d.input(torque);
        SignalSpec current;
        current.name = "motor/motor/current_q";
        current.unit = units::A;  // read at the window's end, as the reference samples it
        current_ = d.input(current);
        SignalSpec speed;
        speed.name = "joint/rotor_joint/velocity";
        speed.unit = units::rad_per_s;
        speed_ = d.output(speed);
        SignalSpec inertia;
        inertia.name = "joint/rotor_joint/effective_inertia";
        inertia.unit = units::kg_m2;
        inertiaOut_ = d.output(inertia);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        velocity_ = 0.0;
        bus.write(speed_, 0.0);
        bus.write(inertiaOut_, inertia_);
        return {};
    }
    Status step(const StepContext& c, SignalBus& bus) override {
        velocity_ += bus.read(torque_) / inertia_ * c.dt.seconds();
        bus.write(speed_, velocity_);
        bus.write(inertiaOut_, inertia_);
        speeds.push_back(velocity_);
        currents.push_back(bus.read(current_));
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    double inertia_;
    double velocity_{0.0};
    SignalId torque_, current_, speed_, inertiaOut_;
};

ModuleConfig exchange(bool predict) {
    ModuleConfigData data;
    data.values.emplace("exchange_step", 1e-3);
    data.values.emplace("speed_prediction", predict);
    return ModuleConfig(data);
}

struct Trace {
    std::vector<double> speeds, currents;  // at the end of each millisecond
};

/// The same drive and flywheel integrated together at 1 µs: the monolithic reference.
Trace reference(double inertia, double seconds) {
    const MotorParams p = motor();
    MotorState s;
    double velocity = 0.0;
    Trace trace;
    constexpr int kPerMs = 1000;
    const int ms = static_cast<int>(std::lround(seconds * 1e3));
    for (int k = 0; k < ms; ++k) {
        for (int n = 0; n < kPerMs; ++n) {
            MotorInputs in;
            in.torqueCommand = kCommand;
            in.jointVelocity = velocity;
            in.busVoltage = kBusVolts;
            const MotorOutputs out = stepMotor(p, s, in, 1e-6);
            velocity += out.jointTorque / inertia * 1e-6;
        }
        trace.speeds.push_back(velocity);
        trace.currents.push_back(s.iq);
    }
    return trace;
}

double rmsError(const std::vector<double>& a, const std::vector<double>& b) {
    double error = 0.0;
    double scale = 0.0;
    for (std::size_t k = 0; k < std::min(a.size(), b.size()); ++k) {
        error += (a[k] - b[k]) * (a[k] - b[k]);
        scale += b[k] * b[k];
    }
    return std::sqrt(error / scale);
}

class Coupling : public ::testing::TestWithParam<double> {};

TEST_P(Coupling, TracksTheMonolithicSolutionWithoutGrowth) {
    const double inertia = GetParam();
    // τm = J·R/(Kt·Ke); run five of them (at least 50 ms, at most 10 s).
    const double tau = inertia * kResistance / (kTorqueConstant * kTorqueConstant);
    const double seconds = std::clamp(5.0 * tau, 0.05, 10.0);
    const World world = rig(inertia);
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<Command>());
    scheduler.add(std::make_unique<ElectricalNetwork>(), exchange(true));
    auto plant = std::make_unique<Flywheel>(inertia);
    Flywheel* flywheel = plant.get();
    scheduler.add(std::move(plant));
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_TRUE(wired) << to_string(wired.error().front());
    ASSERT_TRUE(scheduler.stepUntil(SimTime{static_cast<std::int64_t>(seconds * 1e9)}).ok());
    const Trace ref = reference(inertia, seconds);

    ASSERT_EQ(flywheel->speeds.size(), ref.speeds.size());
    EXPECT_LT(rmsError(flywheel->speeds, ref.speeds), 0.02) << "speed, J = " << inertia;
    EXPECT_LT(rmsError(flywheel->currents, ref.currents), 0.02) << "current, J = " << inertia;
    // No growth: never above the reference's peak (a light rotor overshoots the no-load speed
    // even in the reference: the drive is underdamped), and ending where the reference ends.
    const double peak = *std::max_element(ref.speeds.begin(), ref.speeds.end());
    EXPECT_LT(*std::max_element(flywheel->speeds.begin(), flywheel->speeds.end()), 1.01 * peak);
    EXPECT_NEAR(flywheel->speeds.back(), ref.speeds.back(), 0.01 * std::abs(ref.speeds.back()));
}

INSTANTIATE_TEST_SUITE_P(InertiaFrom1e6To1e2, Coupling,
                         // 6e-7: H/τm ≈ 8, near the limit with speed prediction.
                         ::testing::Values(6e-7, 1e-6, 1e-5, 1e-4, 1e-3, 1e-2));

TEST(CouplingCheck, RefusesAnExchangeThatWouldBeUnstable) {
    // J = 1e-5: τm = 2 ms. Without prediction a 1 ms exchange is 0.5 τm, at the limit; J = 4e-6
    // makes it 1.25 τm and is refused, naming the motor; with prediction it runs.
    const World world = rig(4e-6);
    Scheduler without;
    without.add(std::make_unique<Command>());
    without.add(std::make_unique<ElectricalNetwork>(), exchange(false));
    without.add(std::make_unique<Flywheel>(4e-6));
    auto refused = without.wire(WorldView(world));
    ASSERT_FALSE(refused);
    const std::string text = to_string(refused.error().front());
    EXPECT_NE(text.find("/rig/motor: the physics step 1"), std::string::npos) << text;
    EXPECT_NE(text.find("without speed prediction"), std::string::npos) << text;

    Scheduler with;
    with.add(std::make_unique<Command>());
    with.add(std::make_unique<ElectricalNetwork>(), exchange(true));
    with.add(std::make_unique<Flywheel>(4e-6));
    EXPECT_TRUE(with.wire(WorldView(world)));
}

TEST(RotorInertia, IsAddedToTheDrivenJointThroughTheGear) {
    World world = rig(1e-3);
    for (SchemaInstance& s : world.schemaInstances) {
        if (s.schema == "ElectricalMotorAPI_1") {
            s.reals["elec:rotorInertia"] = 0.00012;
            s.reals["elec:gearRatio"] = 20.0;
        }
    }
    applyRotorInertia(world);
    EXPECT_NEAR(world.joints.armature[0], 400.0 * 0.00012, 1e-15);
}

}  // namespace
}  // namespace lagrange::electrical
