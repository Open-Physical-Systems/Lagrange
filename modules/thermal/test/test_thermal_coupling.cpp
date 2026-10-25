// Coupling between thermal and electrical (architecture.md §8): the winding resistance rises
// with temperature, the copper loss with it. A stalled motor at a constant current heats with
// the thermal network at 100 ms and at 1 ms; the slow exchange must agree within 2 % RMS and
// settle at the same temperature (benign, as expected).
#include "lagrange/electrical/Network.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/thermal/ThermalNetwork.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::thermal {
namespace {

using namespace literals;

SchemaInstance& add(World& w, std::string schema, PrimId prim) {
    w.schemaInstances.push_back({});
    SchemaInstance& s = w.schemaInstances.back();
    s.schema = std::move(schema);
    s.prim = prim;
    return s;
}

/// A motor on a stiff 51.2 V bus, held still, with a thermal node cooled by air.
World stalledMotor() {
    World w;
    const BodyId base = w.addBody("base", kWorldBody, Pose{}, 10.0, {}, {1, 1, 1, 0, 0, 0});
    const BodyId rotor = w.addBody("rotor", base, Pose{}, 0.1, {}, {1e-3, 1e-3, 1e-3, 0, 0, 0});
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
               {"elec:battery:r0", 1e-6},
               {"elec:battery:cutoffVoltage", 1.0},
               {"elec:battery:maxChargeCurrent", 100.0}};
    b.realArrays = {{"elec:battery:ocvSoc", {0.0, 1.0}}, {"elec:battery:ocvVoltage", {3.2, 3.2}}};
    SchemaInstance& port = add(w, "PortAPI_1", pack);
    port.instance = "out";
    port.relationships = {{"elec:port:out:connectedTo", {bus}}};
    const PrimId motor = w.addPrim("/rig/motor", "motor");
    SchemaInstance& m = add(w, "ElectricalMotorAPI_1", motor);
    m.reals = {{"elec:windingResistance", 0.12},
               {"elec:inductance", 0.00031},
               {"elec:torqueConstant", 0.085},
               {"elec:maxCurrent", 50.0},
               {"elec:resistanceTempCoeff", 0.00393}};
    m.relationships = {{"elec:joint", {joint}}, {"elec:powerPort", {bus}}};
    add(w, "ThermalAPI_1", motor).reals = {{"thermal:heatCapacity", 225.0},
                                           {"thermal:convectionArea", 0.5}};
    return w;
}

/// The command (2 N·m: 23.5 A peak, about 100 W of copper loss) and a joint held still.
class Stall final : public IModule {
public:
    [[nodiscard]] ModuleInfo info() const override {
        return {"test", "stall", 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return 1_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec command;
        command.name = "joint/rotor_joint/torque_command";
        command.unit = units::N_m;
        command_ = d.output(command);
        SignalSpec speed;
        speed.name = "joint/rotor_joint/velocity";
        speed.unit = units::rad_per_s;
        speed_ = d.output(speed);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        bus.write(command_, 2.0);
        bus.write(speed_, 0.0);
        return {};
    }
    Status step(const StepContext&, SignalBus& bus) override {
        bus.write(command_, 2.0);
        bus.write(speed_, 0.0);
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    SignalId command_, speed_;
};

/// Reads the motor temperature once a second.
class Thermometer final : public IModule {
public:
    std::vector<double> seen;
    [[nodiscard]] ModuleInfo info() const override {
        return {"test", "thermometer", 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return 1_s; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec spec;
        spec.name = "thermal/motor/temperature";
        spec.unit = units::K;
        id_ = d.input(spec);

        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus&) override { return {}; }
    Status step(const StepContext&, SignalBus& bus) override {
        seen.push_back(bus.read(id_));

        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    SignalId id_;
};

std::vector<double> heatRun(Duration thermalStep) {
    const World world = stalledMotor();
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<Stall>());
    ModuleConfigData power;
    power.dt = 1_ms;  // the electrical network's largest step: the loop is capped to it
    scheduler.add(std::make_unique<electrical::ElectricalNetwork>(), ModuleConfig(power));
    ModuleConfigData heat;
    heat.dt = thermalStep;
    heat.values.emplace(kAmbientKey, 298.15);
    scheduler.add(std::make_unique<ThermalNetwork>(), ModuleConfig(heat));
    auto thermometer = std::make_unique<Thermometer>();
    Thermometer* t = thermometer.get();
    scheduler.add(std::move(thermometer));
    auto wired = scheduler.wire(WorldView(world));
    EXPECT_TRUE(wired) << to_string(wired.error().front());
    EXPECT_TRUE(scheduler.stepUntil(SimTime{300'000'000'000}).ok());  // 300 s, ~7 time constants
    return t->seen;
}

TEST(ThermalElectricalCoupling, TheSlowExchangeIsBenign) {
    const std::vector<double> slow = heatRun(100_ms);
    const std::vector<double> fine = heatRun(1_ms);
    ASSERT_EQ(slow.size(), fine.size());
    double error = 0.0;
    double scale = 0.0;
    for (std::size_t k = 0; k < slow.size(); ++k) {
        const double rise = fine[k] - 298.15;
        error += (slow[k] - fine[k]) * (slow[k] - fine[k]);
        scale += rise * rise;
    }
    EXPECT_LT(std::sqrt(error / scale), 0.02);
    // It heated (about 100 W into 5 W/K: some 20 K, more as the resistance rises), settled,
    // and both runs settled at the same temperature.
    EXPECT_GT(fine.back() - 298.15, 15.0);
    EXPECT_NEAR(slow.back(), fine.back(), 0.005 * (fine.back() - 298.15));
    EXPECT_NEAR(fine.back(), fine[fine.size() - 30], 0.05);  // 6.7 time constants in
}

}  // namespace
}  // namespace lagrange::thermal
