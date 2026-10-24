// The FMU host (docs/design/formats.md §6) with the reference FMU, lagrange_models' dc_drive:
// the FMU path agrees with the built-in brushed model, mismatched maps are refused, the state
// round-trips, and the network hands a motor to its FMU at fidelity `fmu`.
#include "lagrange/electrical/FmuHost.hpp"
#include "lagrange/electrical/Models.hpp"
#include "lagrange/electrical/Network.hpp"
#include "lagrange/scheduler/Scheduler.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::electrical {
namespace {

using namespace literals;

const std::string kFmu = LAGRANGE_TEST_DC_DRIVE_FMU;

/// Writes constant values, or reads signals into rows, every 100 µs.
class Signals final : public IModule {
public:
    struct Column {
        std::string name;
        Unit unit;
        double value{0.0};
    };
    Signals(std::string name, std::vector<Column> columns, bool writer)
        : name_(std::move(name)), columns_(std::move(columns)), writer_(writer) {}
    std::vector<std::vector<double>> rows;
    [[nodiscard]] ModuleInfo info() const override {
        return {"test", name_, 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return 100_us; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        for (const Column& c : columns_) {
            SignalSpec spec;
            spec.name = c.name;
            spec.unit = c.unit;
            ids_.push_back(writer_ ? d.output(spec) : d.input(spec));
        }
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        for (std::size_t i = 0; writer_ && i < ids_.size(); ++i) {
            bus.write(ids_[i], columns_[i].value);
        }
        return {};
    }
    Status step(const StepContext&, SignalBus& bus) override {
        std::vector<double> row;
        for (std::size_t i = 0; i < ids_.size(); ++i) {
            if (writer_) {
                bus.write(ids_[i], columns_[i].value);
            } else {
                row.push_back(bus.read(ids_[i]));
            }
        }
        if (!writer_) {
            rows.push_back(std::move(row));
        }
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    std::string name_;
    std::vector<Column> columns_;
    bool writer_;
    std::vector<SignalId> ids_;
};

FmuBinding binding(std::string fmu = kFmu) {
    FmuBinding b;
    b.fmuPath = std::move(fmu);
    b.name = "drive";
    b.placeholders = {{"joint", "axle"}, {"bus", "dc"}, {"prim", "drive"}};
    b.attributes = {{"elec:windingResistance", 0.4},
                    {"elec:inductance", 0.0006},
                    {"elec:torqueConstant", 0.06},
                    {"elec:maxCurrent", 20.0}};
    return b;
}

TEST(FmuHost, TheReferenceFmuAgreesWithTheBuiltInBrushedModel) {
    // Same inputs, same parameters: the FMU (through FMI Library and the bus) and Models.hpp's
    // brushed drive give the same torque, bus current and loss at every step.
    constexpr double kCommand = 0.6, kSpeed = 120.0, kVolts = 24.0;
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<Signals>(
        "inputs",
        std::vector<Signals::Column>{{"joint/axle/torque_command", units::N_m, kCommand},
                                     {"joint/axle/velocity", units::rad_per_s, kSpeed},
                                     {"bus/dc/voltage", units::V, kVolts}},
        true));
    auto host = std::make_unique<FmuComponent>(binding());
    FmuComponent* fmu = host.get();
    scheduler.add(std::move(host));
    auto out = std::make_unique<Signals>(
        "outputs",
        std::vector<Signals::Column>{{"joint/axle/torque", units::N_m},
                                     {"motor/drive/bus_current", units::A},
                                     {"motor/drive/loss_power", units::W}},
        false);
    Signals* seen = out.get();
    scheduler.add(std::move(out));
    auto wired = scheduler.wire(WorldView(World{}));
    ASSERT_TRUE(wired) << to_string(wired.error().front());
    EXPECT_TRUE(fmu->info().deterministic);
    EXPECT_EQ(fmu->outputNames(),
              (std::vector<std::string>{"joint/axle/torque", "motor/drive/bus_current",
                                        "motor/drive/loss_power", "motor/drive/current_q"}));
    ASSERT_TRUE(scheduler.stepUntil(SimTime{20'000'000}).ok());  // 200 steps

    MotorParams p;
    p.dq = false;
    p.resistance = 0.4;
    p.inductanceD = p.inductanceQ = 0.0006;
    p.torqueConstant = 0.06;
    p.maxCurrent = 20.0;
    p.resistanceTempCoeff = 0.00393;
    MotorState s;
    MotorInputs in;
    in.torqueCommand = kCommand;
    in.jointVelocity = kSpeed;
    in.busVoltage = kVolts;
    ASSERT_EQ(seen->rows.size(), 200U);
    for (std::size_t k = 0; k < seen->rows.size(); ++k) {
        const MotorOutputs built = stepMotor(p, s, in, 1e-4);
        ASSERT_NEAR(seen->rows[k][0], built.jointTorque, 1e-12) << "step " << k;
        ASSERT_NEAR(seen->rows[k][1], built.busCurrent, 1e-12) << "step " << k;
        ASSERT_NEAR(seen->rows[k][2], built.lossPower, 1e-12) << "step " << k;
    }
    EXPECT_NEAR(seen->rows.back()[0], kCommand, 1e-3);  // it tracks the command
}

TEST(FmuHost, AMapThatDisagreesWithTheFmuIsRefused) {
    const std::filesystem::path dir = std::filesystem::path(::testing::TempDir()) / "fmu_bad_map";
    std::filesystem::create_directories(dir);
    const std::filesystem::path fmu = dir / "dc_drive.fmu";
    std::filesystem::copy_file(kFmu, fmu, std::filesystem::copy_options::overwrite_existing);
    std::ofstream(fmuMapPath(fmu.string())) << R"(fmu: dc_drive.fmu
fmi_version: "3.0"
step: 0.0001
inputs:
  - { variable: omega, signal: "joint/{joint}/velocity", unit: A }
  - { variable: no_such, signal: "x/{prim}/y", unit: V }
outputs:
  - { variable: tau_shaft, signal: "joint/{joint}/torque", unit: N.m }
)";
    Scheduler scheduler;
    scheduler.add(std::make_unique<FmuComponent>(binding(fmu.string())));
    auto wired = scheduler.wire(WorldView(World{}));
    ASSERT_FALSE(wired);
    const std::string text = to_string(wired.error().front());
    EXPECT_NE(text.find("omega is in"), std::string::npos) << text;
    EXPECT_NE(text.find("but the map says A"), std::string::npos) << text;
    EXPECT_NE(text.find("no variable no_such"), std::string::npos) << text;
}

TEST(FmuHost, ItsStateRoundTripsThroughTheFmu) {
    const auto build = [] {
        auto scheduler = std::make_unique<Scheduler>(SchedulerOptions{.deterministic = true});
        scheduler->add(std::make_unique<Signals>(
            "inputs",
            std::vector<Signals::Column>{{"joint/axle/torque_command", units::N_m, 0.8},
                                         {"joint/axle/velocity", units::rad_per_s, 50.0},
                                         {"bus/dc/voltage", units::V, 24.0}},
            true));
        scheduler->add(std::make_unique<FmuComponent>(binding()));
        EXPECT_TRUE(scheduler->wire(WorldView(World{})));
        return scheduler;
    };
    auto a = build();
    auto b = build();
    ASSERT_TRUE(a->stepUntil(SimTime{3'000'000}).ok());
    StateWriter saved;
    ASSERT_TRUE(a->saveState(saved).ok());
    ASSERT_TRUE(a->stepUntil(SimTime{6'000'000}).ok());
    StateReader reader(saved.bytes());
    ASSERT_TRUE(b->restoreState(reader).ok());
    ASSERT_TRUE(b->stepUntil(SimTime{6'000'000}).ok());
    EXPECT_EQ(a->stateHash().value(), b->stateHash().value());
}

TEST(FmuHost, AtFidelityFmuTheNetworkHandsTheMotorToItsFmu) {
    World w;
    const BodyId base = w.addBody("base", kWorldBody, Pose{}, 10.0, {}, {1, 1, 1, 0, 0, 0});
    const BodyId wheel = w.addBody("wheel", base, Pose{}, 0.5, {}, {0.01, 0.01, 0.01, 0, 0, 0});
    w.addJoint("axle", JointType::Continuous, wheel, {0, 0, 1}, true);
    const PrimId joint = w.joints.prim.back();
    const auto add = [&w](std::string schema, PrimId prim) -> SchemaInstance& {
        w.schemaInstances.push_back({});
        w.schemaInstances.back().schema = std::move(schema);
        w.schemaInstances.back().prim = prim;
        return w.schemaInstances.back();
    };
    const PrimId bus = w.addPrim("/rig/dc", "dc");
    add("ElectricalBusAPI_1", bus).reals = {{"elec:bus:capacitance", 0.001},
                                            {"elec:bus:undervoltageTrip", 10.0},
                                            {"elec:bus:overvoltageTrip", 40.0}};
    const PrimId pack = w.addPrim("/rig/battery", "battery");
    SchemaInstance& battery = add("BatteryAPI_1", pack);
    battery.integers = {{"elec:battery:seriesCells", 8}};
    battery.reals = {{"elec:battery:capacity", 36000.0},
                     {"elec:battery:r0", 0.002},
                     {"elec:battery:cutoffVoltage", 20.0},
                     {"elec:battery:maxChargeCurrent", 5.0}};
    battery.realArrays = {{"elec:battery:ocvSoc", {0.0, 1.0}},
                          {"elec:battery:ocvVoltage", {3.0, 3.0}}};
    SchemaInstance& port = add("PortAPI_1", pack);
    port.instance = "out";
    port.relationships = {{"elec:port:out:connectedTo", {bus}}};
    SchemaInstance& motor = add("ElectricalMotorAPI_1", w.addPrim("/rig/drive", "drive"));
    motor.reals = {{"elec:windingResistance", 0.4},
                   {"elec:inductance", 0.0006},
                   {"elec:torqueConstant", 0.06},
                   {"elec:maxCurrent", 20.0}};
    motor.assetPaths = {{"elec:behaviorModel", kFmu}};
    motor.relationships = {{"elec:joint", {joint}}, {"elec:powerPort", {bus}}};

    const std::vector<FmuBinding> bindings = motorFmuBindings(WorldView(w));
    ASSERT_EQ(bindings.size(), 1U);
    EXPECT_EQ(bindings[0].placeholders.at("joint"), "axle");
    EXPECT_EQ(bindings[0].placeholders.at("bus"), "dc");

    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<Signals>(
        "inputs",
        std::vector<Signals::Column>{{"joint/axle/torque_command", units::N_m, 0.6},
                                     {"joint/axle/velocity", units::rad_per_s, 100.0}},
        true));
    ModuleConfigData fidelity;
    fidelity.fidelity = "fmu";
    auto network = std::make_unique<ElectricalNetwork>();
    ElectricalNetwork* net = network.get();
    scheduler.add(std::move(network), ModuleConfig(fidelity));
    scheduler.add(std::make_unique<FmuComponent>(bindings[0]));
    auto out = std::make_unique<Signals>(
        "outputs",
        std::vector<Signals::Column>{{"joint/axle/torque", units::N_m},
                                     {"battery/battery/current", units::A},
                                     {"bus/dc/voltage", units::V}},
        false);
    Signals* seen = out.get();
    scheduler.add(std::move(out));
    auto wired = scheduler.wire(WorldView(w));
    ASSERT_TRUE(wired) << to_string(wired.error().front());
    EXPECT_TRUE(net->motors().empty());  // the FMU has it
    EXPECT_TRUE(net->idealJoints().empty());
    ASSERT_TRUE(scheduler.stepUntil(SimTime{50'000'000}).ok());
    const std::vector<double>& last = seen->rows.back();
    EXPECT_NEAR(last[0], 0.6, 1e-3);
    // The FMU's draw reaches the battery: 0.6 N·m at 100 rad/s is 60 W, plus the copper loss
    // of 10 A in 0.4 ohm, 40 W.
    EXPECT_NEAR(last[1] * last[2], 100.0, 1.0);
}

}  // namespace
}  // namespace lagrange::electrical
