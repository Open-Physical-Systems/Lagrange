// The SPICE and Modelica exports (electrical.md §7). The netlist round trip of §8: the network
// runs a drive cycle, the netlist exported with the drives' recorded power runs in ngspice, and
// the bus voltages agree within 1 % (and the sag, the part the export can get wrong, within 3 %).
#include "lagrange/electrical/Export.hpp"
#include "lagrange/electrical/Network.hpp"
#include "lagrange/scheduler/Scheduler.hpp"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::electrical {
namespace {

using namespace literals;

constexpr std::array<const char*, 2> kWheels{"left_wheel_joint", "right_wheel_joint"};
constexpr std::array<const char*, 2> kMotors{"left_drive_motor", "right_drive_motor"};

SchemaInstance& add(World& w, std::string schema, PrimId prim) {
    w.schemaInstances.push_back({});
    SchemaInstance& s = w.schemaInstances.back();
    s.schema = std::move(schema);
    s.prim = prim;
    return s;
}

/// The sample AMR's power system with a weak pack (r0 0.05 ohm per cell, half charged), so the
/// drive cycle sags the bus by volts.
World weakPack() {
    World w;
    const BodyId base = w.addBody("base_link", kWorldBody, Pose{}, 20.0, {}, {1, 1, 1, 0, 0, 0});
    std::vector<PrimId> joints;
    for (const char* name : kWheels) {
        const BodyId wheel = w.addBody(std::string(name) + "_link", base, Pose{}, 1.5, {},
                                       {0.01, 0.01, 0.01, 0, 0, 0});
        w.addJoint(name, JointType::Continuous, wheel, {0, 1, 0}, true);
        joints.push_back(w.joints.prim.back());
    }
    const PrimId bus = w.addPrim("/amr/power/dc_48v", "dc_48v");
    add(w, "ElectricalBusAPI_1", bus).reals = {{"elec:bus:capacitance", 0.0047},
                                               {"elec:bus:undervoltageTrip", 30.0},
                                               {"elec:bus:overvoltageTrip", 60.0}};
    const PrimId pack = w.addPrim("/amr/power/battery", "battery");
    SchemaInstance& battery = add(w, "BatteryAPI_1", pack);
    battery.integers = {{"elec:battery:seriesCells", 16}, {"elec:battery:parallelCells", 1}};
    battery.reals = {{"elec:battery:capacity", 72000.0},
                     {"elec:battery:initialSoc", 0.5},
                     {"elec:battery:r0", 0.05},
                     {"elec:battery:r1", 0.01},
                     {"elec:battery:c1", 20.0},
                     {"elec:battery:cutoffVoltage", 40.0},
                     {"elec:battery:maxChargeCurrent", 50.0}};
    battery.realArrays = {{"elec:battery:ocvSoc", {0.0, 0.1, 0.5, 0.9, 1.0}},
                          {"elec:battery:ocvVoltage", {2.80, 3.20, 3.28, 3.33, 3.45}}};
    SchemaInstance& port = add(w, "PortAPI_1", pack);
    port.instance = "dc_out";
    port.relationships = {{"elec:port:dc_out:connectedTo", {bus}}};
    SchemaInstance& load =
        add(w, "ElectricalLoadAPI_1", w.addPrim("/amr/power/computer", "computer"));
    load.reals = {{"elec:load:power", 45.0}, {"elec:load:minVoltage", 36.0}};
    load.relationships = {{"elec:powerPort", {bus}}};
    for (std::size_t m = 0; m < 2; ++m) {
        SchemaInstance& motor = add(w, "ElectricalMotorAPI_1",
                                    w.addPrim(std::string("/amr/") + kMotors[m], kMotors[m]));
        motor.reals = {{"elec:windingResistance", 0.12}, {"elec:inductance", 0.00031},
                       {"elec:torqueConstant", 0.085},   {"elec:gearRatio", 20.0},
                       {"elec:gearEfficiency", 0.92},    {"elec:maxCurrent", 8.0}};
        motor.integers = {{"elec:polePairs", 4}};
        motor.relationships = {{"elec:joint", {joints[m]}}, {"elec:powerPort", {bus}}};
    }
    return w;
}

/// Commands both wheels, turns flywheels, and records the bus voltage and the drives' bus
/// currents every 100 µs.
class Bench final : public IModule {
public:
    std::vector<double> volts;
    std::array<std::vector<double>, 2> current;
    [[nodiscard]] ModuleInfo info() const override {
        return {"test", "bench", 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return 100_us; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        const auto spec = [](std::string name, Unit unit) {
            SignalSpec s;
            s.name = std::move(name);
            s.unit = unit;
            return s;
        };
        for (std::size_t m = 0; m < 2; ++m) {
            command_[m] =
                d.output(spec(std::string("joint/") + kWheels[m] + "/torque_command", units::N_m));
            speed_[m] =
                d.output(spec(std::string("joint/") + kWheels[m] + "/velocity", units::rad_per_s));
            SignalSpec torque = spec(std::string("joint/") + kWheels[m] + "/torque", units::N_m);
            torque.feedback = true;
            torque_[m] = d.input(torque);
            SignalSpec drawn = spec(std::string("motor/") + kMotors[m] + "/bus_current", units::A);
            drawn.feedback = true;
            current_[m] = d.input(drawn);
        }
        SignalSpec bus = spec("bus/dc_48v/voltage", units::V);
        bus.feedback = true;
        bus_ = d.input(bus);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        for (std::size_t m = 0; m < 2; ++m) {
            bus.write(command_[m], 8.0);
            bus.write(speed_[m], 0.0);
        }
        return {};
    }
    Status step(const StepContext& c, SignalBus& bus) override {
        volts.push_back(bus.read(bus_));
        for (std::size_t m = 0; m < 2; ++m) {
            current[m].push_back(bus.read(current_[m]));
            speed[m] += bus.read(torque_[m]) / 0.5 * c.dt.seconds();  // flywheels of 0.5 kg·m²
            bus.write(speed_[m], speed[m]);
            // Accelerate for 0.3 s, then brake (regenerating).
            bus.write(command_[m], c.t.seconds() < 0.3 ? 8.0 : -8.0);
        }
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    std::array<double, 2> speed{};
    std::array<SignalId, 2> command_, speed_, torque_, current_;
    SignalId bus_;
};

TEST(SpiceExport, TheNetlistRunsInNgspiceAsTheNetworkRan) {
    if (std::string(LAGRANGE_TEST_NGSPICE).empty()) {
        GTEST_SKIP() << "ngspice not found";
    }
    const World world = weakPack();
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<ElectricalNetwork>());
    auto bench = std::make_unique<Bench>();
    Bench* run = bench.get();
    scheduler.add(std::move(bench));
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_TRUE(wired) << to_string(wired.error().front());
    ASSERT_TRUE(scheduler.stepUntil(SimTime{500'000'000}).ok());
    // The bench reads feedback inputs: its row k holds the network's step k - 1. A drive's
    // power at that step is its bus current times the bus voltage it was computed with (the
    // step before), at the step's end time.
    const std::size_t n = run->volts.size();
    ExportOptions options;
    options.title = "weak_pack";
    options.duration = 0.5;
    options.step = 1e-5;
    const std::filesystem::path dir = std::filesystem::path(::testing::TempDir()) / "spice";
    std::filesystem::create_directories(dir);
    options.writeData = (dir / "bus.txt").string();
    for (std::size_t m = 0; m < 2; ++m) {
        std::vector<std::pair<double, double>>& power = options.drivePower[kMotors[m]];
        for (std::size_t k = 2; k < n; ++k) {
            power.emplace_back(static_cast<double>(k - 1) * 1e-4,
                               run->current[m][k] * run->volts[k - 1]);
        }
        power.insert(power.begin(), {0.0, 0.0});
    }
    auto netlist = spiceNetlist(WorldView(world), options);
    ASSERT_TRUE(netlist) << netlist.error();
    const std::filesystem::path cir = dir / "weak_pack.cir";
    std::ofstream(cir) << *netlist;
    const std::string command = std::string(LAGRANGE_TEST_NGSPICE) + " -b " + cir.string() + " > " +
                                (dir / "ngspice.log").string() + " 2>&1";
    ASSERT_EQ(std::system(command.c_str()), 0) << "see " << (dir / "ngspice.log").string();

    // ngspice's bus voltage (time, value rows), interpolated at the network's sample times.
    std::ifstream data(options.writeData);
    std::vector<std::pair<double, double>> spice;
    for (double t = 0.0, v = 0.0; data >> t >> v;) {
        spice.emplace_back(t, v);
    }
    ASSERT_GT(spice.size(), 100U);
    const auto at = [&spice](double t) {
        auto it = std::lower_bound(spice.begin(), spice.end(), std::pair{t, -1e300});
        if (it == spice.begin()) {
            return it->second;
        }
        if (it == spice.end()) {
            return spice.back().second;
        }
        const auto& [t1, v1] = *it;
        const auto& [t0, v0] = *(it - 1);
        return v0 + (v1 - v0) * (t - t0) / (t1 - t0);
    };
    const double start = run->volts[1];
    double worst = 0.0;
    double sagError = 0.0;
    double sagScale = 0.0;
    double deepest = 0.0;
    for (std::size_t k = 10; k < n; k += 10) {
        const double t = static_cast<double>(k - 1) * 1e-4;  // the value's time
        const double runtime = run->volts[k];
        const double other = at(t);
        worst = std::max(worst, std::abs(runtime - other) / runtime);
        sagError += (other - runtime) * (other - runtime);
        sagScale += (start - runtime) * (start - runtime);
        deepest = std::max(deepest, start - runtime);
    }
    RecordProperty("deepest_sag_V", std::to_string(deepest));
    RecordProperty("worst_relative_error", std::to_string(worst));
    RecordProperty("sag_rms_error", std::to_string(std::sqrt(sagError / sagScale)));
    EXPECT_GT(deepest, 2.0);  // the cycle sagged the bus by volts
    EXPECT_LT(worst, 0.01);
    EXPECT_LT(std::sqrt(sagError / sagScale), 0.03);
}

TEST(ModelicaExport, NamesEveryComponentAndConnectsItToItsBus) {
    // OpenModelica is not part of the build, so the model is checked for its structure only.
    auto model = modelicaModel(WorldView(weakPack()), ExportOptions{});
    ASSERT_TRUE(model) << model.error();
    const std::string& m = *model;
    for (const char* expected :
         {"model lagrange_power", "Basic.Capacitor dc_48v(C=0.0047, v(start=",
          "Sources.ConstantVoltage battery_ocv(V=52.48)", "Basic.Resistor battery_r0(R=0.8)",
          "Basic.Resistor battery_rc1_r(R=0.16)", "Basic.Capacitor battery_rc1_c(C=1.25)",
          "connect(battery_rc1_r.n, dc_48v.p)", "computer.i = (if dc_48v.v >= 36 then 45",
          "left_drive_motor.i = p_left_drive_motor.y / max(dc_48v.v", "experiment(StopTime=1",
          "end lagrange_power;"}) {
        EXPECT_NE(m.find(expected), std::string::npos) << expected << "\n" << m;
    }
}

TEST(SpiceExport, RefusesAStageWithoutABus) {
    EXPECT_FALSE(spiceNetlist(WorldView(World{})));
}

}  // namespace
}  // namespace lagrange::electrical
