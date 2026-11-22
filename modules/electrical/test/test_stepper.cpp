// The stepper and its chopper driver (RFC 0020 §4.5) against closed forms: the static torque
// curve and the holding torque per ampere, the pull-out torque at speed (the steady-state phase
// circuit), a stall that loses whole electrical periods and then steps on, microstep position
// with and without detent torque, and the network's stepper on a carriage stepped at 1 ms
// against the same carriage integrated with the driver at 10 µs.
#include "lagrange/electrical/Network.hpp"
#include "lagrange/electrical/Stepper.hpp"
#include "lagrange/scheduler/Scheduler.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <memory>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::electrical {
namespace {

using namespace literals;

constexpr double kPi = std::numbers::pi;

// The NEMA 17 of the gantry sample, STEPPERONLINE 17HS19-2004S1's drawing (A0842, rev. 0):
// 2.0 A per phase, 1.60 Ω, 3.00 mH at 1 kHz, 0.59 N·m holding torque (two phases on, full step),
// 1.8°, rotor inertia 82 g·cm². K_m is the torque per phase ampere: two phases at 2 A make a
// vector of 2√2 A, so K_m = 0.59 / (2√2).
constexpr double kResistance = 1.6;
constexpr double kInductance = 3.0e-3;
constexpr double kRatedCurrent = 2.0;
constexpr double kHoldingTorque = 0.59;
const double kKm = kHoldingTorque / (std::sqrt(2.0) * kRatedCurrent);
constexpr double kRotorInertia = 82e-7;  // kg·m²
constexpr double kBus = 24.0;            // V

StepperParams nema17(int microsteps = 16) {
    StepperParams p;
    p.motor.dq = false;
    p.motor.resistance = kResistance;
    p.motor.inductanceD = p.motor.inductanceQ = kInductance;
    p.motor.torqueConstant = kKm;
    p.motor.maxCurrent = kRatedCurrent;
    p.motor.resistanceTempCoeff = 0.0;
    p.holdCurrent = kRatedCurrent;
    p.microsteps = microsteps;
    return p;
}

/// Full steps per second to rotor rad/s.
double rotorSpeed(const StepperParams& p, double fullStepsPerSecond) {
    return fullStepsPerSecond * (kPi / 2.0) / p.rotorTeeth;
}

/// The mean shaft torque at a constant rotor speed and load angle δ (electrical rad behind the
/// step count), after the currents settle.
double meanTorque(const StepperParams& p, double fullStepsPerSecond, double delta,
                  double voltageAmplitude, double dt) {
    const double w = rotorSpeed(p, fullStepsPerSecond);
    const double we = w * p.rotorTeeth;
    StepperState s;
    StepperInputs in;
    in.busVoltage = kBus;
    in.voltageAmplitude = voltageAmplitude;
    double sum = 0.0;
    int n = 0;
    constexpr double kSettle = 0.05;  // s: many L/R time constants (1.9 ms)
    constexpr double kAverage = 0.05;
    for (int k = 0; k * dt < kSettle + kAverage; ++k) {
        const double t = k * dt;
        s.steps = std::floor(fullStepsPerSecond * p.microsteps * t);
        s.position = (we * t - delta) / p.rotorTeeth;
        s.velocity = w;
        const StepperOutputs out = stepStepper(p, s, in, dt);
        if (t >= kSettle) {
            sum += out.motorTorque;
            ++n;
        }
    }
    return sum / n;
}

/// The pull-out torque: the largest mean torque over the load angle.
double pullOut(const StepperParams& p, double fullStepsPerSecond, double voltageAmplitude,
               double dt) {
    double best = 0.0;
    constexpr int kAngles = 64;
    for (int k = 0; k <= kAngles; ++k) {
        best = std::max(best,
                        meanTorque(p, fullStepsPerSecond, k * kPi / kAngles, voltageAmplitude, dt));
    }
    return best;
}

TEST(Stepper, StaticTorqueIsTheSineOfTheRotorAngleAndHoldsKmTimesTheCurrent) {
    // τ(Δθ) = −K_m·I·sin(N_r·Δθ), its peak K_m·I a full step from the equilibrium.
    for (const double current : {0.5, 1.0, 1.5, 2.0}) {
        StepperParams p = nema17();
        p.holdCurrent = current;
        const double fullStep = kPi / 2.0 / p.rotorTeeth;  // rad
        double peak = 0.0;
        for (int k = -8; k <= 8; ++k) {
            const double offset = k * fullStep / 4.0;
            StepperState s;
            s.position = offset;
            StepperInputs in;
            in.busVoltage = kBus;
            StepperOutputs out;
            for (int n = 0; n < 4000; ++n) {  // 0.4 s: past the standstill delay
                s.stillFor += 1e-4;
                out = stepStepper(p, s, in, 1e-4);
            }
            const double expected = -kKm * current * std::sin(p.rotorTeeth * offset);
            EXPECT_NEAR(out.motorTorque, expected, 0.01 * kKm * current) << "Δθ = " << offset;
            peak = std::max(peak, std::abs(out.motorTorque));
        }
        EXPECT_NEAR(peak, kKm * current, 0.01 * kKm * current) << "holding torque at " << current;
    }
    // A microstepping driver puts one vector of the rated current on the rotor: 1/√2 of the
    // datasheet's two-phases-on holding torque.
    EXPECT_NEAR(kKm * kRatedCurrent, kHoldingTorque / std::sqrt(2.0), 1e-12);
}

TEST(Stepper, DropsToTheHoldCurrentAtStandstill) {
    StepperParams p = nema17();
    p.holdCurrent = 0.7;
    StepperState s;
    s.position = kPi / 2.0 / p.rotorTeeth;  // a full step off: the torque is K_m·I
    StepperInputs in;
    in.busVoltage = kBus;
    StepperOutputs out;
    for (int n = 0; n < 1000; ++n) {
        out = stepStepper(p, s, in, 1e-4);
    }
    EXPECT_NEAR(std::abs(out.motorTorque), kKm * kRatedCurrent, 1e-3);
    for (int n = 0; n < 3000; ++n) {
        s.stillFor += 1e-4;
        out = stepStepper(p, s, in, 1e-4);
    }
    EXPECT_NEAR(std::abs(out.motorTorque), kKm * 0.7, 1e-3);
}

TEST(Stepper, PullOutUnderVoltageDriveIsThePhaseCircuitsClosedForm) {
    // Sine voltages of amplitude V at ωe = N_r·ω: I = (V − j·K_m·ω·e^(−jδ))/Z, Z = R + jωeL, and
    // τ(δ) = (K_m/|Z|)·(V·sin(δ − α) − K_m·ω·cos α), so τ_po = (K_m/|Z|)·(V − K_m·ω·R/|Z|).
    // The RFC's I(ω) = V/√(R² + (N_r·ω·L)²) is its first term.
    const StepperParams p = nema17(256);
    for (const double fs : {100.0, 500.0, 1000.0, 2000.0, 4000.0}) {
        const double w = rotorSpeed(p, fs);
        const double z =
            std::abs(std::complex<double>(kResistance, w * p.rotorTeeth * kInductance));
        const double closed = kKm / z * (kBus - kKm * w * kResistance / z);
        const double model = pullOut(p, fs, kBus, 1e-5);
        std::printf("voltage drive %5.0f full steps/s: pull-out %.4f N·m, closed form %.4f\n", fs,
                    model, closed);
        EXPECT_NEAR(model, closed, 0.01 * closed) << fs << " full steps/s";
    }
}

TEST(Stepper, ChopperHoldsItsCurrentBelowTheCornerAndLosesTorqueAbove) {
    // Below the corner (V/|Z| = I: ωe·L = √((V/I)² − R²), 2500 full steps/s here) the chopper
    // keeps the rated current and the pull-out torque is K_m·I; above it the current, and the
    // torque, fall with speed. The RFC's K_m·min(I, V/|Z|) leaves the back-EMF and the square
    // wave of a saturated chopper out: within 15 % up to 2.4 times the corner (the voltage-drive
    // test above checks the motor's equations against the full closed form).
    const StepperParams p = nema17(16);
    const double corner = std::sqrt(std::pow(kBus / kRatedCurrent, 2) - kResistance * kResistance) /
                          kInductance / (2.0 * kPi) *
                          4.0;  // full steps/s: four per electrical period
    EXPECT_NEAR(corner, 2524.0, 1.0);
    double last = 1e9;
    for (const double fs : {100.0, 500.0, 1000.0, 3000.0, 4000.0, 5000.0, 6000.0}) {
        const double w = rotorSpeed(p, fs);
        const double z =
            std::abs(std::complex<double>(kResistance, w * p.rotorTeeth * kInductance));
        const double rfc = kKm * std::min(kRatedCurrent, kBus / z);
        const double model = pullOut(p, fs, 0.0, 1e-5);
        std::printf("chopper %5.0f full steps/s: pull-out %.4f N·m, K_m·min(I, V/|Z|) %.4f\n", fs,
                    model, rfc);
        if (fs <= 1000.0) {
            EXPECT_NEAR(model, kKm * kRatedCurrent, 0.01 * kKm * kRatedCurrent);
        } else {
            EXPECT_NEAR(model, rfc, 0.15 * rfc) << fs << " full steps/s";
        }
        EXPECT_LE(model, last + 1e-4);
        last = model;
    }
}

/// A rotor and its load at `dt`: the step rate ramps to `rate` full steps/s over `ramp` s and
/// holds; `load(t)` is the load torque at the shaft (N·m), viscous `damping` (N·m·s/rad) on top.
struct SpinResult {
    std::int64_t lost{0};
    double position{0.0};  // rad
    double steps{0.0};
};
template <class Load>
SpinResult spin(StepperParams p, StepperState& s, double rate, double ramp, double seconds,
                Load load, double damping, double inertia, double dt = 1e-5) {
    double fraction = 0.0;
    double t = 0.0;
    for (int k = 0; k * dt < seconds; ++k, t += dt) {
        const double now = ramp > 0.0 ? rate * std::min(1.0, t / ramp) : rate;
        fraction += now * p.microsteps * dt;
        const double whole = std::trunc(fraction);
        fraction -= whole;
        const double before = s.steps;
        s.steps += whole;
        s.stillFor = s.steps == before ? s.stillFor + dt : 0.0;
        StepperInputs in;
        in.busVoltage = kBus;
        in.inertia = inertia;
        in.loadTorque = load(t) + damping * s.velocity;
        stepStepper(p, s, in, dt);
    }
    return {lostSteps(p, s, s.position), s.position, s.steps};
}

TEST(Stepper, KeepsStepWithALoadBelowPullOutAndStallsAbove) {
    const StepperParams p = nema17(16);
    const double inertia = kRotorInertia * 2.0;  // the rotor and a pulley
    const double damping = 1e-4;      // N·m·s/rad: bearings and the load's friction (3 mN·m here)
    constexpr double kRate = 1000.0;  // full steps/s, reached in 0.5 s
    const double pullout = pullOut(p, kRate, 0.0, 1e-5);
    // The load rises over 0.2 s: one applied at once swings the rotor past its peak torque.
    const auto load = [&](double fraction) {
        return [=](double t) { return fraction * pullout * std::min(1.0, t / 0.2); };
    };
    StepperState below;
    const SpinResult kept = spin(p, below, kRate, 0.5, 1.0, load(0.95), damping, inertia);
    EXPECT_EQ(kept.lost, 0);
    EXPECT_NEAR(kept.position * p.rotorTeeth * 2.0 / kPi * p.microsteps, kept.steps,
                2.0 * p.microsteps)
        << "within a full step or two of the count (the load angle)";
    StepperState above;
    const SpinResult stalled = spin(p, above, kRate, 0.5, 1.0, load(1.05), damping, inertia);
    EXPECT_GT(stalled.lost, 100) << "it falls out of step and stays out";
    EXPECT_EQ(stalled.lost % 4, 0);
}

TEST(Stepper, AnOverloadSkipsWholeStepsAndTheMotorThenStepsOn) {
    StepperParams p = nema17(16);
    p.holdCurrent = kRatedCurrent;
    const double inertia = 10.0 * kRotorInertia;  // the rotor and its load
    const double damping = 2e-3;  // N·m·s/rad: enough to land on the next equilibrium
    StepperState s;
    const double holding = kKm * kRatedCurrent;
    // Held still, a load of 1.3× the holding torque for 10 ms, then nothing.
    spin(
        p, s, 0.0, 0.0, 0.3, [&](double t) { return t > 0.05 && t < 0.06 ? 1.3 * holding : 0.0; },
        damping, inertia);
    const std::int64_t lost = lostSteps(p, s, s.position);
    std::printf("an overload of 1.3× the holding torque for 10 ms lost %lld full steps\n",
                static_cast<long long>(lost));
    EXPECT_GT(lost, 0);
    EXPECT_EQ(lost % 4, 0) << "a slip lands four full steps on, never between";
    // Then 200 full steps forward: it steps on from where it landed, the loss unchanged, and
    // stops on the count less what it lost.
    spin(p, s, 200.0, 0.0, 1.0, [](double) { return 0.0; }, damping, inertia);
    const SpinResult onward =
        spin(p, s, 0.0, 0.0, 0.3, [](double) { return 0.0; }, damping, inertia);
    EXPECT_EQ(onward.lost, lost);
    const double fullStep = kPi / 2.0 / p.rotorTeeth;
    EXPECT_NEAR(onward.steps, 200.0 * p.microsteps, 1.0);
    EXPECT_NEAR(onward.position / fullStep, onward.steps / p.microsteps - static_cast<double>(lost),
                0.01);
}

TEST(Stepper, MicrostepsLandOnTheTableWithoutDetentAndWhereTheClosedFormSaysWithIt) {
    // Without detent each microstep's equilibrium is its electrical angle exactly. With detent
    // the rotor sits where K_m·I·sin(φ − θe) = T_d·sin(4θe), solved here by bisection.
    for (const double detent : {0.0, 0.03}) {
        StepperParams p = nema17(16);
        p.detentTorque = detent;
        const double ki = kKm * kRatedCurrent;
        double worst = 0.0;
        for (int k = 0; k <= p.microsteps; ++k) {
            const double phi = k * (kPi / 2.0) / p.microsteps;
            double lo = phi - kPi / 4.0;
            double hi = phi + kPi / 4.0;
            for (int n = 0; n < 100; ++n) {
                const double mid = 0.5 * (lo + hi);
                const double f = ki * std::sin(phi - mid) - detent * std::sin(4.0 * mid);
                (f > 0.0 ? lo : hi) = mid;
            }
            const double expected = 0.5 * (lo + hi) / p.rotorTeeth;
            StepperState s;
            s.steps = k;
            s.position = 0.0;
            spin(p, s, 0.0, 0.0, 0.3, [](double) { return 0.0; }, 2e-3, kRotorInertia);
            const double microstep = kPi / 2.0 / p.rotorTeeth / p.microsteps;
            EXPECT_NEAR(s.position, expected, 0.01 * microstep) << "microstep " << k;
            worst = std::max(worst, std::abs(s.position - phi / p.rotorTeeth) / microstep);
        }
        std::printf("detent %.3f N·m: worst microstep error %.3f microsteps\n", detent, worst);
        if (detent == 0.0) {
            EXPECT_LT(worst, 0.01);
        } else {
            EXPECT_GT(worst, 0.1) << "detent pulls microsteps toward the full steps";
        }
    }
}

// ---- The network's stepper on a carriage, physics stepped at 1 ms -------------------------

SchemaInstance& add(World& w, std::string schema, PrimId prim) {
    w.schemaInstances.push_back({});
    SchemaInstance& s = w.schemaInstances.back();
    s.schema = std::move(schema);
    s.prim = prim;
    return s;
}

constexpr double kBeltRatio = 2.0 * kPi / 0.04;  // motor rad per metre: a 20-tooth GT2 pulley
constexpr double kCarriage = 1.0;                // kg

World gantry() {
    World w;
    const BodyId base = w.addBody("base", kWorldBody, Pose{}, 10.0, {}, {1, 1, 1, 0, 0, 0});
    const BodyId carriage = w.addBody("carriage", base, Pose{}, kCarriage, {}, {1, 1, 1, 0, 0, 0});
    w.addJoint("x", JointType::Prismatic, carriage, {1, 0, 0}, true);
    const PrimId joint = w.joints.prim.back();
    const PrimId bus = w.addPrim("/g/bus", "bus");
    add(w, "ElectricalBusAPI_1", bus).reals = {{"elec:bus:capacitance", 0.001},
                                               {"elec:bus:undervoltageTrip", 10.0},
                                               {"elec:bus:overvoltageTrip", 40.0}};
    const PrimId pack = w.addPrim("/g/supply", "supply");
    SchemaInstance& b = add(w, "BatteryAPI_1", pack);
    b.integers = {{"elec:battery:seriesCells", 1}};
    b.reals = {{"elec:battery:capacity", 1e9},
               {"elec:battery:r0", 1e-3},
               {"elec:battery:cutoffVoltage", 1.0},
               {"elec:battery:maxChargeCurrent", 1e3}};
    b.realArrays = {{"elec:battery:ocvSoc", {0.0, 1.0}}, {"elec:battery:ocvVoltage", {kBus, kBus}}};
    SchemaInstance& port = add(w, "PortAPI_1", pack);
    port.instance = "out";
    port.relationships = {{"elec:port:out:connectedTo", {bus}}};
    SchemaInstance& m = add(w, "ElectricalMotorAPI_1", w.addPrim("/g/x_motor", "x_motor"));
    m.tokens = {{"elec:motorType", "stepper"}};
    m.reals = {{"elec:windingResistance", kResistance},
               {"elec:inductance", kInductance},
               {"elec:torqueConstant", kKm},
               {"elec:maxCurrent", kRatedCurrent},
               {"elec:gearRatio", kBeltRatio}};
    m.integers = {{"elec:stepper:microsteps", 16}};
    m.relationships = {{"elec:joint", {joint}}, {"elec:powerPort", {bus}}};
    return w;
}

/// The step rate (microsteps/s): up to `rate` over `ramp` s, held, and down over `ramp` s again
/// to stop at `run` s.
class StepRate final : public IModule {
public:
    StepRate(double rate, double ramp, double run) : rate_(rate), ramp_(ramp), run_(run) {}
    [[nodiscard]] ModuleInfo info() const override {
        return {"test", "step_rate", 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return 100_us; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec spec;
        spec.name = "joint/x/step_rate";
        spec.unit = units::Hz;
        id_ = d.output(spec);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        bus.write(id_, 0.0);
        return {};
    }
    Status step(const StepContext& c, SignalBus& bus) override {
        const double t = c.t.seconds();
        const double up = std::min(1.0, t / ramp_);
        const double down = std::clamp((run_ - t) / ramp_, 0.0, 1.0);
        bus.write(id_, rate_ * std::min(up, down));
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    double rate_, ramp_, run_;
    SignalId id_;
};

/// The carriage at 1 ms, as MuJoCo exchanges: the force averaged over its step, semi-implicit
/// Euler, with a viscous friction.
class Carriage final : public IModule {
public:
    static constexpr double kDamping = 20.0;  // N·s/m
    [[nodiscard]] ModuleInfo info() const override {
        return {"test", "carriage", 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return 1_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec force;
        force.name = "joint/x/torque";
        force.unit = units::N;
        force.hold = Hold::WindowAverage;
        force_ = d.input(force);
        SignalSpec position;
        position.name = "joint/x/position";
        position.unit = units::m;
        position_ = d.output(position);
        SignalSpec velocity;
        velocity.name = "joint/x/velocity";
        velocity.unit = units::m_per_s;
        velocity_ = d.output(velocity);
        SignalSpec inertia;
        inertia.name = "joint/x/effective_inertia";
        inertia.unit = units::kg;
        inertia_ = d.output(inertia);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        x = v = 0.0;
        bus.write(position_, 0.0);
        bus.write(velocity_, 0.0);
        bus.write(inertia_, mass());
        return {};
    }
    Status step(const StepContext& c, SignalBus& bus) override {
        const double h = c.dt.seconds();
        v += (bus.read(force_) - kDamping * v) / mass() * h;
        x += v * h;
        bus.write(position_, x);
        bus.write(velocity_, v);
        bus.write(inertia_, mass());
        return {};
    }
    Status saveState(StateWriter& w) const override {
        w.put(x);
        w.put(v);
        return {};
    }
    Status restoreState(StateReader& r) override {
        return r.get(x) && r.get(v) ? Status{}
                                    : Status::error(StatusCode::InvalidConfig, "carriage");
    }
    // The carriage and the rotor reflected through the belt (applyRotorInertia's job in a run).
    static double mass() { return kCarriage + kRotorInertia * kBeltRatio * kBeltRatio; }
    double x{0.0}, v{0.0};

private:
    SignalId force_, position_, velocity_, inertia_;
};

ModuleConfig exchange() {
    ModuleConfigData data;
    data.values.emplace("exchange_step", 1e-3);
    return ModuleConfig(data);
}

struct GantryRun {
    double position{0.0};
    double steps{0.0};
    double lost{0.0};
};

GantryRun runGantry(double rate, double ramp, double run, double seconds) {
    const World world = gantry();
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<StepRate>(rate, ramp, run));
    scheduler.add(std::make_unique<ElectricalNetwork>(), exchange());
    auto plant = std::make_unique<Carriage>();
    Carriage* carriage = plant.get();
    scheduler.add(std::move(plant));
    auto wired = scheduler.wire(WorldView(world));
    EXPECT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    EXPECT_TRUE(scheduler.stepUntil(SimTime{static_cast<std::int64_t>(seconds * 1e9)}).ok());
    GantryRun out;
    out.position = carriage->x;
    out.steps = scheduler.bus().read(scheduler.bus().find("motor/x_motor/steps"));
    out.lost = scheduler.bus().read(scheduler.bus().find("motor/x_motor/lost_steps"));
    return out;
}

TEST(StepperNetwork, MovesACarriageByItsStepsAcrossThePhysicsExchange) {
    // 16 000 microsteps/s (1000 full steps/s: 0.2 m/s on the belt) reached in 0.25 s, down again
    // by 0.75 s, then 0.25 s to settle; the carriage stops where the count says, to a microstep.
    const StepperParams p = [] {
        StepperParams q = nema17(16);
        q.motor.gearRatio = kBeltRatio;
        return q;
    }();
    const GantryRun run = runGantry(16000.0, 0.25, 0.75, 1.0);
    const double expected = run.steps / p.microstepsPerJoint();
    std::printf("carriage at %.6f m, the count says %.6f m (%.0f microsteps)\n", run.position,
                expected, run.steps);
    EXPECT_GT(run.steps, 5000.0);
    EXPECT_EQ(run.lost, 0.0);
    EXPECT_NEAR(run.position, expected, 1.0 / p.microstepsPerJoint());
}

TEST(StepperNetwork, AStepRateTooFastForTheLoadLosesStepsInWholePeriods) {
    // Asked for 3000 full steps/s at once, the 1 kg carriage cannot follow: steps are lost.
    const GantryRun run = runGantry(48000.0, 1e-3, 0.3, 0.5);  // and stopped at once
    std::printf("asked 3000 full steps/s at once: lost %.0f full steps\n", run.lost);
    EXPECT_GT(run.lost, 0.0);
    EXPECT_EQ(std::fmod(run.lost, 4.0), 0.0);
}

TEST(StepperNetwork, IsDeterministic) {
    const GantryRun a = runGantry(16000.0, 0.25, 0.5, 0.6);
    const GantryRun b = runGantry(16000.0, 0.25, 0.5, 0.6);
    EXPECT_EQ(a.position, b.position);
    EXPECT_EQ(a.steps, b.steps);
}

}  // namespace
}  // namespace lagrange::electrical
