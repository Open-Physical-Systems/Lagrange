// The autopilot bridges' ESCs (src/Escs.hpp) on a stand-in for the electrical substitute and the
// physics: a rotor of the sample quadcopter's inertia that the commanded torque turns against its
// propeller's drag. With ideal motors (the electrical domain off) the torque stays within the
// drive's and a throttle step settles without ringing; without them, the commands are the speed
// loop's as before, bit for bit.
#include "Escs.hpp"
#include "lagrange/mavlink/RotorTopSpeed.hpp"
#include "lagrange/scheduler/Scheduler.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::mavlink {
namespace {

using namespace literals;

constexpr double kRho = 1.225;             // kg/m³, ISA sea level
constexpr double kRotorInertia = 0.00009;  // kg·m² about the axis: the sample's 25 g propeller
constexpr double kSpeedGain = 0.02;        // N·m per rad/s: Px4Settings' default
constexpr int kRotors = 4;

/// A 2 kg quadcopter of the sample's class (as test_rotor_top_speed.cpp): four 10 in rotors
/// driven by 920 KV motors (25 A) from a 4S battery.
World quad() {
    World w;
    const BodyId base = w.addBody("base", kWorldBody, Pose{}, 1.9, {}, {0.02, 0.02, 0.04, 0, 0, 0});
    w.addJoint("base_free", JointType::Free, base, {0, 0, 1}, false);
    const PrimId bus = w.addPrim("/quad/power/dc_bus", "dc_bus");
    const PrimId battery = w.addPrim("/quad/power/battery", "battery");
    SchemaInstance b;
    b.schema = "BatteryAPI_1";
    b.prim = battery;
    b.integers = {{"elec:battery:seriesCells", 4}};
    b.realArrays = {{"elec:battery:ocvVoltage", {3.3, 3.7, 4.2}}};
    w.schemaInstances.push_back(b);
    SchemaInstance port;
    port.schema = "PortAPI_1";
    port.instance = "dc_out";
    port.prim = battery;
    port.relationships = {{"elec:port:dc_out:connectedTo", {bus}}};
    w.schemaInstances.push_back(port);
    for (int i = 1; i <= kRotors; ++i) {
        const std::string name = "rotor_" + std::to_string(i);
        const BodyId rotor = w.addBody(name + "_link", base, Pose{}, 0.025, {},
                                       {0.000045, 0.000045, kRotorInertia, 0, 0, 0});
        w.addJoint(name + "_joint", JointType::Continuous, rotor, {0, 0, 1}, true);
        const PrimId joint = w.joints.prim.back();
        SchemaInstance s;
        s.schema = "RotorAPI_1";
        s.prim = joint;
        s.reals = {{"aero:rotor:radius", 0.127},
                   {"aero:rotor:thrustCoeff", 0.125},
                   {"aero:rotor:powerCoeff", 0.0507}};
        w.schemaInstances.push_back(s);
        SchemaInstance m;
        m.schema = "ElectricalMotorAPI_1";
        m.prim = w.addPrim("/quad/" + name + "_motor", name + "_motor");
        m.tokens = {{"elec:motorType", "bldc"}};
        m.reals = {{"elec:torqueConstant", 0.008989},
                   {"elec:windingResistance", 0.1},
                   {"elec:inductance", 0.00003},
                   {"elec:maxCurrent", 25.0}};
        m.integers = {{"elec:polePairs", 7}};
        m.relationships = {{"elec:joint", {joint}}, {"elec:powerPort", {bus}}};
        w.schemaInstances.push_back(m);
    }
    return w;
}

const SchemaInstance& rotor1(const World& w) {
    return *WorldView(w).instancesOf("RotorAPI_1").front();
}

/// The propeller's drag torque per (rad/s)², as Escs computes it: CP·ρ·D⁵/(8π³).
double torquePerOmega2() {
    constexpr double pi = std::numbers::pi;
    return 0.0507 * kRho * std::pow(0.254, 5) / (8.0 * pi * pi * pi);
}

/// The ESCs as an autopilot bridge runs them, at `step`, with the output `throttle(t)` on every
/// rotor.
class EscModule final : public IModule {
public:
    EscModule(Duration step, bool idealMotors, std::function<double(double)> throttle)
        : step_(step), ideal_(idealMotors), throttle_(std::move(throttle)) {}
    [[nodiscard]] ModuleInfo info() const override {
        return {"bridge", "escs", 0, 1, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return step_; }
    Status declare(const WorldView& world, const ModuleConfig&, SignalDeclarer& d) override {
        return escs.declare(world, d, kRho, std::nullopt, kSpeedGain, "test", 1.0, ideal_,
                            step_.seconds());
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        escs.stop(bus);
        return {};
    }
    Status step(const StepContext& context, SignalBus& bus) override {
        const std::vector<double> u(kRotors, throttle_(context.t.seconds()));
        escs.drive(bus, u);
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

    Escs escs;

private:
    Duration step_;
    bool ideal_;
    std::function<double(double)> throttle_;
};

/// The ideal torque source and the physics for the rotors at 1 ms: J·dω/dt = τ − cq·ω·|ω|, in
/// ten substeps. Records, per step and rotor 1, the speed it started from and the torque.
class RotorPlant final : public IModule {
public:
    struct Sample {
        double t;       // s, the step's start
        double omega;   // rad/s at t (what the ESCs read)
        double torque;  // N·m commanded over the step
    };
    [[nodiscard]] ModuleInfo info() const override {
        return {"rigid_body", "rotor_plant", 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return 1_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        for (int i = 1; i <= kRotors; ++i) {
            const std::string base = "joint/rotor_" + std::to_string(i) + "_joint/";
            SignalSpec command;
            command.name = base + "torque_command";
            command.unit = units::N_m;
            commands_.push_back(d.input(command));
            SignalSpec velocity;
            velocity.name = base + "velocity";
            velocity.unit = units::rad_per_s;
            velocities_.push_back(d.output(velocity));
        }
        omega_.assign(kRotors, 0.0);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        for (int i = 0; i < kRotors; ++i) {
            bus.write(velocities_[i], 0.0);
        }
        return {};
    }
    Status step(const StepContext& context, SignalBus& bus) override {
        const double cq = torquePerOmega2();
        constexpr int kSub = 10;
        const double h = context.dt.seconds() / kSub;
        for (int i = 0; i < kRotors; ++i) {
            const double torque = bus.read(commands_[i]);
            if (i == 0) {
                samples.push_back({context.t.seconds(), omega_[0], torque});
            }
            for (int k = 0; k < kSub; ++k) {
                omega_[i] += h * (torque - (cq * omega_[i] * std::abs(omega_[i]))) / kRotorInertia;
            }
            bus.write(velocities_[i], omega_[i]);
        }
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

    std::vector<Sample> samples;

private:
    std::vector<SignalId> commands_, velocities_;
    std::vector<double> omega_;
};

struct Flight {
    std::vector<RotorPlant::Sample> samples;
    double torqueLimit;
    double gain;
};

/// Half throttle for 1 s, then 0.8 for 1 s, with the ESCs at `escStep`.
Flight fly(const World& world, Duration escStep, bool idealMotors) {
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    auto escs = std::make_unique<EscModule>(escStep, idealMotors,
                                            [](double t) { return t < 1.0 ? 0.5 : 0.8; });
    auto plant = std::make_unique<RotorPlant>();
    EscModule* e = escs.get();
    RotorPlant* p = plant.get();
    scheduler.add(std::move(escs));
    scheduler.add(std::move(plant));
    EXPECT_TRUE(scheduler.wire(WorldView(world)));
    EXPECT_TRUE(scheduler.stepUntil(SimTime{2'000'000'000}).ok());
    return {p->samples, e->escs.torqueLimit(0), e->escs.gain(0)};
}

/// The speed settles on `target` from `from` on (s) without ringing: once within 1 % it stays
/// within 1 %, it never overshoots by more than 0.5 %, and it ends within 0.1 %.
void expectSettles(const std::vector<RotorPlant::Sample>& samples, double from, double to,
                   double target, const std::string& what) {
    bool reached = false;
    double peak = 0.0;
    double last = 0.0;
    for (const RotorPlant::Sample& s : samples) {
        if (s.t < from || s.t >= to) {
            continue;
        }
        const double error = std::abs(s.omega - target) / target;
        if (reached) {
            EXPECT_LT(error, 0.01) << what << " left the band at t = " << s.t;
        }
        reached = reached || error < 0.01;
        peak = std::max(peak, s.omega);
        last = s.omega;
    }
    EXPECT_TRUE(reached) << what;
    EXPECT_LT(peak, target * 1.005) << what << ": overshoot";
    EXPECT_NEAR(last, target, target * 1e-3) << what;
}

class IdealMotors : public ::testing::TestWithParam<int> {};  // the ESC step, ms

TEST_P(IdealMotors, AThrottleStepSettlesWithoutRingingWithinTheDrivesTorque) {
    const World world = quad();
    const Duration escStep{GetParam() * 1'000'000LL};
    const RotorTopSpeed top = rotorTopSpeed(WorldView(world), rotor1(world), kRho);
    const Flight run = fly(world, escStep, true);
    // The bound: the propeller's torque at the motor's loaded top speed.
    EXPECT_DOUBLE_EQ(run.torqueLimit, torquePerOmega2() * top.speed * top.speed);
    // The gain: K·dt/J at most kSpeedLoopGain.
    EXPECT_LE(run.gain * escStep.seconds() / kRotorInertia, Escs::kSpeedLoopGain * (1 + 1e-12));
    EXPECT_LE(run.gain, kSpeedGain);
    double peak = 0.0;
    for (const RotorPlant::Sample& s : run.samples) {
        EXPECT_LE(std::abs(s.torque), run.torqueLimit * (1 + 1e-12)) << "t = " << s.t;
        peak = std::max(peak, std::abs(s.torque));
    }
    // The steps ask for more than the drive gives: the bound is what holds them.
    EXPECT_DOUBLE_EQ(peak, run.torqueLimit);
    expectSettles(run.samples, 0.0, 1.0, 0.5 * top.speed, "half throttle");
    expectSettles(run.samples, 1.0, 2.0, 0.8 * top.speed, "0.8 throttle");
}

// PX4's lockstep at the sample's 250 Hz IMU, and ArduPilot's 1 kHz exchange.
INSTANTIATE_TEST_SUITE_P(EscSteps, IdealMotors, ::testing::Values(4, 1));

TEST(Escs, WithTheElectricalDomainTheCommandsAreTheSpeedLoopsAsBefore) {
    const World world = quad();
    const RotorTopSpeed top = rotorTopSpeed(WorldView(world), rotor1(world), kRho);
    const Flight run = fly(world, 4_ms, false);
    EXPECT_EQ(run.torqueLimit, 0.0);
    EXPECT_EQ(run.gain, kSpeedGain);
    double peak = 0.0;
    std::size_t checked = 0;
    for (const RotorPlant::Sample& s : run.samples) {
        const auto ms = std::lround(s.t * 1000.0);
        if (ms % 4 != 0) {
            continue;  // the ESCs' command held between their steps
        }
        // The loop as it was: feed-forward of the propeller's drag at the set point plus a
        // proportional correction, unbounded (the drives' current loops bound it).
        const double u = s.t < 1.0 ? 0.5 : 0.8;
        const double target = 1.0 * u * (top.speed * std::sqrt(1.0));
        const double feedForward = 1.0 * torquePerOmega2() * target * target;
        EXPECT_EQ(s.torque, feedForward + kSpeedGain * (target - s.omega)) << "t = " << s.t;
        peak = std::max(peak, std::abs(s.torque));
        ++checked;
    }
    EXPECT_EQ(checked, 500U);
    // What the bound changes with ideal motors: unbounded, the step's kick is far above it.
    EXPECT_GT(peak, 5.0 * torquePerOmega2() * top.speed * top.speed);
}

}  // namespace
}  // namespace lagrange::mavlink
