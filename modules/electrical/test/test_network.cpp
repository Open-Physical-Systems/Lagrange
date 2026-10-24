// The electrical network module on a hand-built world with the sample AMR's power system,
// against the scenarios of docs/design/domains/electrical.md §8.
#include "lagrange/electrical/Network.hpp"
#include "lagrange/scheduler/Scheduler.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::electrical {
namespace {

using namespace literals;

constexpr std::array<const char*, 2> kWheels{"left_wheel_joint", "right_wheel_joint"};

/// Values of the sample AMR's overlay that the scenarios change.
struct Power {
    double windingResistance{0.12};
    double maxCurrent{8.0};
    double gearEfficiency{0.92};
    double r0{0.0015};
    int parallelCells{1};
    double initialSoc{1.0};
    double maxChargeCurrent{10.0};
    double brakeResistor{0.0};
    double loadPower{45.0};
    bool liftJoint{false};  // a third, motorless joint
};

SchemaInstance& add(World& w, std::string schema, PrimId prim) {
    w.schemaInstances.push_back({});
    SchemaInstance& s = w.schemaInstances.back();
    s.schema = std::move(schema);
    s.prim = prim;
    return s;
}

/// Two driven wheels on a chassis, two motors on a 48 V bus fed by a 16-cell LFP pack, and the
/// computer as a 45 W load: amr_diffdrive.overlay.yaml.
World poweredAmr(const Power& power = {}) {
    World w;
    const BodyId base = w.addBody("base_link", kWorldBody, Pose{}, 20.0, {}, {1, 1, 1, 0, 0, 0});
    std::vector<PrimId> joints;
    for (const char* name : kWheels) {
        const BodyId wheel = w.addBody(std::string(name) + "_link", base, Pose{}, 1.5, {},
                                       {0.01, 0.01, 0.01, 0, 0, 0});
        w.addJoint(name, JointType::Continuous, wheel, {0, 1, 0}, true);
        joints.push_back(w.joints.prim.back());
    }
    if (power.liftJoint) {
        const BodyId lift =
            w.addBody("lift_link", base, Pose{}, 1.0, {}, {0.01, 0.01, 0.01, 0, 0, 0});
        w.addJoint("lift_joint", JointType::Revolute, lift, {0, 0, 1}, true);
    }
    const PrimId bus = w.addPrim("/amr/power/dc_48v", "dc_48v");
    SchemaInstance& b = add(w, "ElectricalBusAPI_1", bus);
    b.reals = {{"elec:bus:capacitance", 0.0047},
               {"elec:bus:undervoltageTrip", 42.0},
               {"elec:bus:overvoltageTrip", 60.0},
               {"elec:bus:brakeResistor", power.brakeResistor}};
    const PrimId pack = w.addPrim("/amr/power/battery", "battery");
    SchemaInstance& battery = add(w, "BatteryAPI_1", pack);
    battery.integers = {{"elec:battery:seriesCells", 16},
                        {"elec:battery:parallelCells", power.parallelCells}};
    battery.reals = {{"elec:battery:capacity", 72000.0},
                     {"elec:battery:initialSoc", power.initialSoc},
                     {"elec:battery:r0", power.r0},
                     {"elec:battery:r1", 0.0008},
                     {"elec:battery:c1", 12000.0},
                     {"elec:battery:cutoffVoltage", 44.8},
                     {"elec:battery:maxChargeCurrent", power.maxChargeCurrent}};
    battery.realArrays = {{"elec:battery:ocvSoc", {0.0, 0.1, 0.5, 0.9, 1.0}},
                          {"elec:battery:ocvVoltage", {2.80, 3.20, 3.28, 3.33, 3.45}}};
    SchemaInstance& port = add(w, "PortAPI_1", pack);
    port.instance = "dc_out";
    port.relationships = {{"elec:port:dc_out:connectedTo", {bus}}};
    if (power.loadPower > 0.0) {
        SchemaInstance& load =
            add(w, "ElectricalLoadAPI_1", w.addPrim("/amr/power/computer", "computer"));
        load.reals = {{"elec:load:power", power.loadPower}, {"elec:load:minVoltage", 36.0}};
        load.tokens = {{"elec:load:model", "constant_power"}};
        load.relationships = {{"elec:powerPort", {bus}}};
    }
    for (std::size_t m = 0; m < 2; ++m) {
        const std::string name = m == 0 ? "left_drive_motor" : "right_drive_motor";
        SchemaInstance& motor =
            add(w, "ElectricalMotorAPI_1", w.addPrim("/amr/base_link/" + name, name));
        motor.reals = {{"elec:windingResistance", power.windingResistance},
                       {"elec:inductance", 0.00031},
                       {"elec:torqueConstant", 0.085},
                       {"elec:gearRatio", 20.0},
                       {"elec:gearEfficiency", power.gearEfficiency},
                       {"elec:maxCurrent", power.maxCurrent}};
        motor.integers = {{"elec:polePairs", 4}};
        motor.relationships = {{"elec:joint", {joints[m]}}, {"elec:powerPort", {bus}}};
    }
    return w;
}

/// Writes joint/<j>/torque_command = command(t) every millisecond, as a controller does.
class Commander final : public IModule {
public:
    Commander(std::vector<std::string> joints, std::function<double(double)> command)
        : joints_(std::move(joints)), command_(std::move(command)) {}
    [[nodiscard]] ModuleInfo info() const override {
        return {"test", "commander", 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return 1_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        for (const std::string& j : joints_) {
            SignalSpec spec;
            spec.name = "joint/" + j + "/torque_command";
            spec.unit = units::N_m;
            ids_.push_back(d.output(spec));
        }
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        for (const SignalId id : ids_) {
            bus.write(id, command_(0.0));
        }
        return {};
    }
    Status step(const StepContext& c, SignalBus& bus) override {
        for (const SignalId id : ids_) {
            bus.write(id, command_(c.t.seconds()));
        }
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    std::vector<std::string> joints_;
    std::function<double(double)> command_;
    std::vector<SignalId> ids_;
};

/// Physics in miniature: each joint turns a flywheel (kg·m² at the joint), or is held still.
class Flywheels final : public IModule {
public:
    Flywheels(double inertia, Duration step, double startVelocity = 0.0, bool stalled = false)
        : inertia_(inertia), step_(step), start_(startVelocity), stalled_(stalled) {}
    std::vector<double> velocity;
    [[nodiscard]] ModuleInfo info() const override {
        return {"test", "flywheels", 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return step_; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        for (const char* j : kWheels) {
            SignalSpec torque;
            torque.name = std::string("joint/") + j + "/torque";
            torque.unit = units::N_m;
            torque_.push_back(d.input(torque));
            SignalSpec speed;
            speed.name = std::string("joint/") + j + "/velocity";
            speed.unit = units::rad_per_s;
            speed_.push_back(d.output(speed));
        }
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        velocity.assign(kWheels.size(), stalled_ ? 0.0 : start_);
        for (std::size_t j = 0; j < kWheels.size(); ++j) {
            bus.write(speed_[j], velocity[j]);
        }
        return {};
    }
    Status step(const StepContext& c, SignalBus& bus) override {
        for (std::size_t j = 0; j < kWheels.size(); ++j) {
            if (!stalled_) {
                velocity[j] += bus.read(torque_[j]) / inertia_ * c.dt.seconds();
            }
            bus.write(speed_[j], velocity[j]);
        }
        return {};
    }
    Status saveState(StateWriter& w) const override {
        w.putDoubles(velocity);
        return {};
    }
    Status restoreState(StateReader& r) override {
        return r.getDoubles(velocity) ? Status{} : Status::error(StatusCode::InvalidConfig, "");
    }

private:
    double inertia_;
    Duration step_;
    double start_;
    bool stalled_;
    std::vector<SignalId> torque_;
    std::vector<SignalId> speed_;
};

/// Reads signals every 100 µs into rows.
class Recorder final : public IModule {
public:
    struct Column {
        std::string name;
        Unit unit;
    };
    explicit Recorder(std::vector<Column> columns) : columns_(std::move(columns)) {}
    std::vector<std::vector<double>> rows;
    [[nodiscard]] std::size_t column(const std::string& name) const {
        for (std::size_t c = 0; c < columns_.size(); ++c) {
            if (columns_[c].name == name) {
                return c;
            }
        }
        ADD_FAILURE() << "no column " << name;
        return 0;
    }
    [[nodiscard]] ModuleInfo info() const override {
        return {"test", "recorder", 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return 100_us; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        for (const Column& c : columns_) {
            SignalSpec spec;
            spec.name = c.name;
            spec.unit = c.unit;
            ids_.push_back(d.input(spec));
        }
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus&) override { return {}; }
    Status step(const StepContext&, SignalBus& bus) override {
        std::vector<double> row;
        for (const SignalId id : ids_) {
            row.push_back(bus.read(id));
        }
        rows.push_back(std::move(row));
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    std::vector<Column> columns_;
    std::vector<SignalId> ids_;
};

std::vector<Recorder::Column> standardColumns() {
    std::vector<Recorder::Column> c{
        {"bus/dc_48v/voltage", units::V},         {"bus/dc_48v/brownout", units::one},
        {"battery/battery/current", units::A},    {"battery/battery/soc", units::one},
        {"battery/battery/loss_power", units::W}, {"load/computer/current", units::A},
        {"load/computer/powered", units::one}};
    for (const char* m : {"left_drive_motor", "right_drive_motor"}) {
        for (const char* q : {"bus_current", "loss_power", "fault", "current_d", "current_q"}) {
            c.push_back({std::string("motor/") + m + "/" + q,
                         std::string_view(q) == "fault"        ? units::one
                         : std::string_view(q) == "loss_power" ? units::W
                                                               : units::A});
        }
    }
    for (const char* j : kWheels) {
        c.push_back({std::string("joint/") + j + "/torque", units::N_m});
        c.push_back({std::string("joint/") + j + "/velocity", units::rad_per_s});
    }
    return c;
}

struct Rig {
    std::unique_ptr<Scheduler> scheduler;
    ElectricalNetwork* network{nullptr};
    Recorder* recorder{nullptr};
    Flywheels* flywheels{nullptr};
};

Rig makeRun(std::function<double(double)> command, std::unique_ptr<Flywheels> plant,
            bool withRecorder = true) {
    Rig run;
    run.scheduler = std::make_unique<Scheduler>(SchedulerOptions{.deterministic = true});
    run.scheduler->add(std::make_unique<Commander>(
        std::vector<std::string>(kWheels.begin(), kWheels.end()), std::move(command)));
    auto network = std::make_unique<ElectricalNetwork>();
    run.network = network.get();
    run.scheduler->add(std::move(network));
    run.flywheels = plant.get();
    run.scheduler->add(std::move(plant));
    if (withRecorder) {
        auto recorder = std::make_unique<Recorder>(standardColumns());
        run.recorder = recorder.get();
        run.scheduler->add(std::move(recorder));
    }
    return run;
}

TEST(ElectricalNetwork, TheSampleAmrDrivesItsWheelsFromTheBattery) {
    const World world = poweredAmr();
    Rig run = makeRun([](double) { return 5.0; }, std::make_unique<Flywheels>(0.5, 1_ms));
    auto wired = run.scheduler->wire(WorldView(world));
    ASSERT_TRUE(wired) << to_string(wired.error().front());
    EXPECT_EQ(run.network->motors(),
              (std::vector<std::string>{"left_drive_motor", "right_drive_motor"}));
    EXPECT_EQ(run.network->buses(), (std::vector<std::string>{"dc_48v"}));
    ASSERT_TRUE(run.scheduler->stepUntil(SimTime{1'000'000'000}).ok());
    const Recorder& r = *run.recorder;
    const std::vector<double>& last = r.rows.back();
    EXPECT_NEAR(last[r.column("joint/left_wheel_joint/torque")], 5.0, 0.05);
    EXPECT_NEAR(run.flywheels->velocity[0], 10.0, 0.1);  // 5 N·m / 0.5 kg·m² × 1 s
    const double volts = last[r.column("bus/dc_48v/voltage")];
    EXPECT_LT(volts, 16 * 3.45);  // sagging under load
    EXPECT_GT(volts, 50.0);
    EXPECT_GT(last[r.column("battery/battery/current")], 0.0);
    EXPECT_LT(last[r.column("battery/battery/soc")], 1.0);
    EXPECT_EQ(last[r.column("load/computer/powered")], 1.0);
    EXPECT_NEAR(last[r.column("load/computer/current")], 45.0 / volts, 0.05);
    EXPECT_EQ(last[r.column("motor/left_drive_motor/fault")], 0.0);
}

TEST(ElectricalNetwork, EnergyBalancesOverADriveCycle) {
    // Battery energy out at the bus = mechanical work + copper loss + the load + the change of
    // energy in the bus capacitor and the windings, within 0.5 % (electrical.md §8). Gear
    // losses are left out (efficiency 1) so the mechanical work is torque × speed at the joint.
    Power power;
    power.gearEfficiency = 1.0;
    const World world = poweredAmr(power);
    // Accelerate, coast, then brake hard enough to regenerate.
    Rig run = makeRun([](double t) { return t < 1.0 ? 8.0 : (t < 1.5 ? 0.0 : -8.0); },
                      std::make_unique<Flywheels>(0.5, 100_us));
    ASSERT_TRUE(run.scheduler->wire(WorldView(world)));
    ASSERT_TRUE(run.scheduler->stepUntil(SimTime{2'500'000'000}).ok());
    const Recorder& r = *run.recorder;
    constexpr double kDt = 1e-4;
    const std::size_t v = r.column("bus/dc_48v/voltage");
    double battery = 0.0;
    double used = 0.0;
    double regenerated = 0.0;
    for (std::size_t k = 0; k < r.rows.size(); ++k) {
        const std::vector<double>& row = r.rows[k];
        battery += row[v] * row[r.column("battery/battery/current")] * kDt;
        used += row[v] * row[r.column("load/computer/current")] * kDt;
        for (std::size_t m = 0; m < 2; ++m) {
            const std::string motor = m == 0 ? "left_drive_motor" : "right_drive_motor";
            const std::string joint = kWheels[m];
            const double mechanical = row[r.column("joint/" + joint + "/torque")] *
                                      row[r.column("joint/" + joint + "/velocity")];
            used += (mechanical + row[r.column("motor/" + motor + "/loss_power")]) * kDt;
            regenerated += std::min(0.0, mechanical) * kDt;
        }
    }
    const std::vector<double>& first = r.rows.front();
    const std::vector<double>& last = r.rows.back();
    constexpr double kCapacitance = 0.0047;
    used += 0.5 * kCapacitance * (last[v] * last[v] - first[v] * first[v]);
    for (const char* m : {"left_drive_motor", "right_drive_motor"}) {
        const auto magnetic = [&](const std::vector<double>& row) {
            const double id = row[r.column(std::string("motor/") + m + "/current_d")];
            const double iq = row[r.column(std::string("motor/") + m + "/current_q")];
            return 1.5 * 0.5 * 0.00031 * (id * id + iq * iq);
        };
        used += magnetic(last) - magnetic(first);
    }
    EXPECT_LT(regenerated, -10.0);  // the cycle did regenerate
    EXPECT_NEAR(used, battery, 5e-3 * std::abs(battery)) << "battery " << battery << " J";
}

TEST(ElectricalNetwork, AStalledAmrOnALowBatteryBrownsOutAndRecovers) {
    // electrical.md §8: both motors stalled at full current on a low, weak battery. The bus
    // trips, the computer loses power and the drives trip; with the drives off the bus recovers
    // and the computer comes back; the drives reset once the command is withdrawn.
    Power power;
    power.windingResistance = 0.5;
    power.maxCurrent = 30.0;
    power.r0 = 0.05;
    power.initialSoc = 0.05;
    const World world = poweredAmr(power);
    Rig run = makeRun([](double t) { return t < 0.3 ? 50.0 : 0.0; },
                      std::make_unique<Flywheels>(0.5, 1_ms, 0.0, true));
    ASSERT_TRUE(run.scheduler->wire(WorldView(world)));
    ASSERT_TRUE(run.scheduler->stepUntil(SimTime{600'000'000}).ok());
    const Recorder& r = *run.recorder;
    const auto at = [&](double seconds, const std::string& column) {
        return r.rows[static_cast<std::size_t>(std::lround(seconds / 1e-4)) - 1][r.column(column)];
    };
    bool tripped = false;
    for (std::size_t k = 0; k + 1 < r.rows.size(); ++k) {
        if (r.rows[k][r.column("bus/dc_48v/brownout")] == 1.0) {
            tripped = true;
            // Loads see the trip at their next step (100 µs later): they are evaluated at the
            // bus voltage of the step before.
            EXPECT_EQ(r.rows[k + 1][r.column("load/computer/powered")], 0.0);
            break;
        }
    }
    EXPECT_TRUE(tripped);
    // Still commanded at 0.29 s: drives latched off, bus and computer back.
    EXPECT_EQ(at(0.29, "motor/left_drive_motor/fault"), 1.0);
    EXPECT_EQ(at(0.29, "joint/left_wheel_joint/torque"), 0.0);
    EXPECT_EQ(at(0.29, "bus/dc_48v/brownout"), 0.0);
    EXPECT_EQ(at(0.29, "load/computer/powered"), 1.0);
    // Command withdrawn: the drives reset.
    EXPECT_EQ(at(0.6, "motor/left_drive_motor/fault"), 0.0);
    EXPECT_EQ(at(0.6, "bus/dc_48v/brownout"), 0.0);
}

TEST(ElectricalNetwork, RegenerationIntoAFullBatteryRaisesTheBusUntilTheBrakeResistorTakesIt) {
    // Braking a spinning flywheel (about 150 W) while the battery accepts only 0.5 A and the
    // computer draws 45 W: without a brake
    // resistor the bus passes its overvoltage trip and the drives switch off; with one, the
    // resistor holds the bus near the trip and braking continues.
    for (const double brake : {0.0, 10.0}) {
        Power power;
        power.maxChargeCurrent = 0.5;
        power.brakeResistor = brake;
        const World world = poweredAmr(power);
        Rig run =
            makeRun([](double) { return -10.0; }, std::make_unique<Flywheels>(2.0, 1_ms, 15.0));
        ASSERT_TRUE(run.scheduler->wire(WorldView(world)));
        ASSERT_TRUE(run.scheduler->stepUntil(SimTime{1'000'000'000}).ok());
        const Recorder& r = *run.recorder;
        double highest = 0.0;
        bool switchedOff = false;
        for (const std::vector<double>& row : r.rows) {
            highest = std::max(highest, row[r.column("bus/dc_48v/voltage")]);
            switchedOff = switchedOff || row[r.column("motor/left_drive_motor/fault")] == 2.0;
            EXPECT_GE(row[r.column("battery/battery/current")], -0.5 - 1e-12);
        }
        EXPECT_GT(highest, 60.0) << "brake " << brake;
        if (brake > 0.0) {
            EXPECT_FALSE(switchedOff);
            EXPECT_LT(highest, 63.0);
            EXPECT_LT(run.flywheels->velocity[0], 15.0 - 3.0);  // braked: 10 N·m × 1 s / 2 kg·m²
        } else {
            EXPECT_TRUE(switchedOff);
        }
    }
}

TEST(ElectricalNetwork, JointsWithoutAMotorAreDrivenIdeally) {
    Power power;
    power.liftJoint = true;
    const World world = poweredAmr(power);
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<Commander>(
        std::vector<std::string>{"left_wheel_joint", "right_wheel_joint", "lift_joint"},
        [](double) { return 3.0; }));
    auto network = std::make_unique<ElectricalNetwork>();
    ElectricalNetwork* net = network.get();
    scheduler.add(std::move(network));
    scheduler.add(std::make_unique<Flywheels>(0.5, 1_ms));
    auto recorder = std::make_unique<Recorder>(
        std::vector<Recorder::Column>{{"joint/lift_joint/torque", units::N_m}});
    Recorder* r = recorder.get();
    scheduler.add(std::move(recorder));
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_TRUE(wired) << to_string(wired.error().front());
    EXPECT_EQ(net->idealJoints(), (std::vector<std::string>{"lift_joint"}));
    ASSERT_TRUE(scheduler.stepUntil(SimTime{10'000'000}).ok());
    EXPECT_EQ(r->rows.back()[0], 3.0);
}

TEST(ElectricalNetwork, RefusesAnIncompleteNetworkNamingEveryPrim) {
    World world = poweredAmr();
    for (SchemaInstance& s : world.schemaInstances) {
        if (s.schema == "ElectricalMotorAPI_1" &&
            world.prims.instanceNames[s.prim.index] == "left_drive_motor") {
            s.reals.erase("elec:torqueConstant");
        } else if (s.schema == "ElectricalMotorAPI_1") {
            s.reals["elec:backEmfConstant"] = 0.085;  // Kt, as if both were per RMS ampere
        } else if (s.schema == "ElectricalLoadAPI_1") {
            s.relationships["elec:powerPort"] = {world.joints.prim[0]};
        }
    }
    add(world, "ElectricalConverterAPI_1", world.addPrim("/amr/power/dcdc", "dcdc"));  // empty
    Scheduler scheduler;
    scheduler.add(std::make_unique<Commander>(
        std::vector<std::string>(kWheels.begin(), kWheels.end()), [](double) { return 0.0; }));
    scheduler.add(std::make_unique<ElectricalNetwork>());
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_FALSE(wired);
    std::string text;
    for (const WiringError& e : wired.error()) {
        text += to_string(e) + "\n";
    }
    EXPECT_NE(text.find("/amr/base_link/left_drive_motor: a motor needs"), std::string::npos)
        << text;
    EXPECT_NE(text.find("/amr/base_link/right_drive_motor: backEmfConstant 0.085"),
              std::string::npos)
        << text;
    EXPECT_NE(text.find("/amr/power/computer: elec:powerPort must lead to an ElectricalBusAPI_1"),
              std::string::npos)
        << text;
    EXPECT_NE(text.find("/amr/power/dcdc: a converter needs outputVoltage > 0"), std::string::npos)
        << text;
}

/// The sample AMR with a 12 V bus for its sensors, fed from the 48 V bus by a DC-DC converter
/// (efficiency 0.9, `limit` A), and a constant-power sensor load of `sensors` W on it.
World amrWithConverter(double sensors, double limit) {
    World w = poweredAmr();
    PrimId bus48{};
    for (const SchemaInstance& s : w.schemaInstances) {
        if (s.schema == "ElectricalBusAPI_1") {
            bus48 = s.prim;
        }
    }
    const PrimId bus12 = w.addPrim("/amr/power/dc_12v", "dc_12v");
    SchemaInstance& b = add(w, "ElectricalBusAPI_1", bus12);
    b.reals = {{"elec:bus:capacitance", 0.001},
               {"elec:bus:undervoltageTrip", 10.0},
               {"elec:bus:overvoltageTrip", 14.0}};
    SchemaInstance& converter =
        add(w, "ElectricalConverterAPI_1", w.addPrim("/amr/power/dcdc", "dcdc"));
    converter.reals = {{"elec:conv:outputVoltage", 12.0},
                       {"elec:conv:efficiency", 0.9},
                       {"elec:conv:maxOutputCurrent", limit}};
    converter.relationships = {{"elec:conv:input", {bus48}}, {"elec:conv:output", {bus12}}};
    SchemaInstance& load = add(w, "ElectricalLoadAPI_1", w.addPrim("/amr/sensors/rack", "rack"));
    load.reals = {{"elec:load:power", sensors}, {"elec:load:minVoltage", 10.5}};
    load.relationships = {{"elec:powerPort", {bus12}}};
    return w;
}

std::vector<Recorder::Column> converterColumns() {
    std::vector<Recorder::Column> c = standardColumns();
    c.insert(c.end(), {{"bus/dc_12v/voltage", units::V},
                       {"bus/dc_12v/brownout", units::one},
                       {"converter/dcdc/output_current", units::A},
                       {"converter/dcdc/input_current", units::A},
                       {"converter/dcdc/loss_power", units::W},
                       {"load/rack/current", units::A},
                       {"load/rack/powered", units::one}});
    return c;
}

Rig converterRun() {
    Rig run;
    run.scheduler = std::make_unique<Scheduler>(SchedulerOptions{.deterministic = true});
    run.scheduler->add(std::make_unique<Commander>(
        std::vector<std::string>(kWheels.begin(), kWheels.end()), [](double) { return 0.0; }));
    auto network = std::make_unique<ElectricalNetwork>();
    run.network = network.get();
    run.scheduler->add(std::move(network));
    auto plant = std::make_unique<Flywheels>(0.5, 1_ms);
    run.flywheels = plant.get();
    run.scheduler->add(std::move(plant));
    auto recorder = std::make_unique<Recorder>(converterColumns());
    run.recorder = recorder.get();
    run.scheduler->add(std::move(recorder));
    return run;
}

TEST(ElectricalNetwork, AConverterRegulatesItsBusAndDrawsItsPowerOverItsEfficiency) {
    const World world = amrWithConverter(24.0, 5.0);  // 2 A at 12 V, within the limit
    Rig run = converterRun();
    auto wired = run.scheduler->wire(WorldView(world));
    ASSERT_TRUE(wired) << to_string(wired.error().front());
    EXPECT_EQ(run.network->buses(), (std::vector<std::string>{"dc_48v", "dc_12v"}));
    ASSERT_TRUE(run.scheduler->stepUntil(SimTime{200'000'000}).ok());
    const Recorder& r = *run.recorder;
    const std::vector<double>& last = r.rows.back();
    EXPECT_NEAR(last[r.column("bus/dc_12v/voltage")], 12.0, 1e-9);
    EXPECT_NEAR(last[r.column("converter/dcdc/output_current")], 2.0, 1e-6);
    EXPECT_NEAR(last[r.column("load/rack/current")], 2.0, 1e-6);
    const double v48 = last[r.column("bus/dc_48v/voltage")];
    EXPECT_NEAR(last[r.column("converter/dcdc/input_current")], 24.0 / 0.9 / v48, 1e-4);
    EXPECT_NEAR(last[r.column("converter/dcdc/loss_power")], 24.0 * (1.0 / 0.9 - 1.0), 1e-4);
    // The pack carries the computer, the sensors and the converter's loss.
    EXPECT_NEAR(last[r.column("battery/battery/current")] * v48, 45.0 + 24.0 / 0.9, 0.05);
}

TEST(ElectricalNetwork, AConverterAtItsCurrentLimitLetsItsBusSagUntilTheLoadsDrop) {
    // 60 W of sensors at 12 V need 5 A from a converter limited to 3 A: the 12 V bus falls until
    // the load switches off at its minVoltage, the bus recovers, and the cycle repeats; the
    // converter never exceeds 3 A.
    const World world = amrWithConverter(60.0, 3.0);
    Rig run = converterRun();
    ASSERT_TRUE(run.scheduler->wire(WorldView(world)));
    ASSERT_TRUE(run.scheduler->stepUntil(SimTime{200'000'000}).ok());
    const Recorder& r = *run.recorder;
    double most = 0.0;
    bool dropped = false;
    for (const std::vector<double>& row : r.rows) {
        most = std::max(most, row[r.column("converter/dcdc/output_current")]);
        dropped = dropped || row[r.column("load/rack/powered")] == 0.0;
    }
    EXPECT_NEAR(most, 3.0, 1e-9);
    EXPECT_TRUE(dropped);
    EXPECT_EQ(r.rows.back()[r.column("bus/dc_48v/brownout")], 0.0);  // the 48 V side is fine
}

TEST(ElectricalNetwork, ConvertersAreRefusedWhenMiswired) {
    World world = amrWithConverter(24.0, 5.0);
    for (SchemaInstance& s : world.schemaInstances) {
        if (s.schema == "ElectricalConverterAPI_1") {
            s.relationships["elec:conv:output"] = s.relationships["elec:conv:input"];
        }
    }
    Scheduler scheduler;
    scheduler.add(std::make_unique<ElectricalNetwork>());
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_FALSE(wired);
    EXPECT_NE(to_string(wired.error().front())
                  .find("/amr/power/dcdc: a converter's input and output are the same bus"),
              std::string::npos)
        << to_string(wired.error().front());
}

TEST(ElectricalNetwork, ItIsDeterministicAndRestoresItsState) {
    const World world = poweredAmr();
    const auto command = [](double t) { return 6.0 * std::sin(3.0 * t); };
    Rig a = makeRun(command, std::make_unique<Flywheels>(0.5, 1_ms), false);
    Rig b = makeRun(command, std::make_unique<Flywheels>(0.5, 1_ms), false);
    ASSERT_TRUE(a.scheduler->wire(WorldView(world)));
    ASSERT_TRUE(b.scheduler->wire(WorldView(world)));
    ASSERT_TRUE(a.scheduler->stepUntil(SimTime{400'000'000}).ok());
    ASSERT_TRUE(b.scheduler->stepUntil(SimTime{400'000'000}).ok());
    EXPECT_EQ(a.scheduler->stateHash().value(), b.scheduler->stateHash().value());

    StateWriter saved;
    ASSERT_TRUE(a.scheduler->saveState(saved).ok());
    ASSERT_TRUE(a.scheduler->stepUntil(SimTime{700'000'000}).ok());
    const std::uint64_t expected = a.scheduler->stateHash().value();
    StateReader reader(saved.bytes());
    ASSERT_TRUE(b.scheduler->restoreState(reader).ok());
    ASSERT_TRUE(b.scheduler->stepUntil(SimTime{700'000'000}).ok());
    EXPECT_EQ(b.scheduler->stateHash().value(), expected);
}

}  // namespace
}  // namespace lagrange::electrical
