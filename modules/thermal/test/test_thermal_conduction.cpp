// The conduction fidelity on the bus (ThermalConduction.hpp): materials from presets, heat
// sources by power and by signal, losses on the link, the light that leaves, and the checks.
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/thermal/ThermalConduction.hpp"
#include "lagrange/thermal/ThermalNetwork.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::thermal {
namespace {

using namespace literals;

/// Writes a constant to a signal, or reads one, every 100 ms.
class Probe final : public IModule {
public:
    Probe(std::string signal, Unit unit, bool writer, double value = 0.0)
        : signal_(std::move(signal)), unit_(unit), writer_(writer), value_(value) {}
    std::vector<double> seen;
    [[nodiscard]] ModuleInfo info() const override {
        return {"test", writer_ ? "source" : "reader", 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return 100_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec spec;
        spec.name = signal_;
        spec.unit = unit_;
        id_ = writer_ ? d.output(spec) : d.input(spec);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        if (writer_) {
            bus.write(id_, value_);
        }
        return {};
    }
    Status step(const StepContext&, SignalBus& bus) override {
        if (writer_) {
            bus.write(id_, value_);
        } else {
            seen.push_back(bus.read(id_));
        }
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    std::string signal_;
    Unit unit_;
    bool writer_;
    double value_;
    SignalId id_;
};

constexpr double kAmbient = 298.15;  // K

/// A 40 × 40 × 1.6 mm board (FR-4) bolted to a 50 × 50 × 5 mm aluminium plate under it.
World boardOnPlate(const std::string& preset = "fr4") {
    World w;
    const BodyId plate =
        w.addBody("plate", kWorldBody, Pose{}, 0.034, {0, 0, 0}, {1e-5, 1e-5, 1e-5, 0, 0, 0});
    w.addCollider(plate, ShapeType::Box, {0.025, 0.025, 0.0025}, Pose{}, 0.5);
    const BodyId board = w.addBody("board", plate, Pose{{0.0, 0.0, 0.0033}, {1, 0, 0, 0}}, 0.005,
                                   {0, 0, 0}, {1e-6, 1e-6, 1e-6, 0, 0, 0});
    w.addCollider(board, ShapeType::Box, {0.02, 0.02, 0.0008}, Pose{}, 0.5);
    w.addJoint("board_joint", JointType::Fixed, board, {0, 0, 1}, false);
    SchemaInstance p;
    p.schema = "ThermalMaterialAPI_1";
    p.prim = w.bodies.prim[plate.index];
    p.tokens = {{"thermal:material:preset", "aluminium_6061"}};
    w.schemaInstances.push_back(p);
    SchemaInstance b;
    b.schema = "ThermalMaterialAPI_1";
    b.prim = w.bodies.prim[board.index];
    b.tokens = {{"thermal:material:preset", preset}};
    b.reals = {{"thermal:material:contactConductance", 3000.0},
               {"thermal:material:maxTemperature", 1000.0}};
    w.schemaInstances.push_back(b);
    SchemaInstance led;
    led.schema = "HeatSourceAPI_1";
    led.instance = "led_1";
    led.prim = w.bodies.prim[board.index];
    led.reals = {{"thermal:source:led_1:power", 3.0}, {"thermal:source:led_1:efficiency", 0.35}};
    led.realArrays = {{"thermal:source:led_1:position", {0.01, 0.0, 0.0005}},
                      {"thermal:source:led_1:size", {0.003, 0.003, 0.001}}};
    w.schemaInstances.push_back(led);
    SchemaInstance driver;
    driver.schema = "HeatSourceAPI_1";
    driver.instance = "driver";
    driver.prim = w.bodies.prim[board.index];
    driver.tokens = {{"thermal:source:driver:powerSignal", "driver/out/power"}};
    driver.realArrays = {{"thermal:source:driver:position", {-0.01, 0.0, 0.0005}},
                         {"thermal:source:driver:size", {0.004, 0.004, 0.001}}};
    w.schemaInstances.push_back(driver);
    return w;
}

ModuleConfig config(double cell = 0.002, Duration dt = Duration{100'000'000}) {
    ModuleConfigData data;
    data.fidelity = "conduction";
    data.dt = dt;
    data.values.emplace(kAmbientKey, kAmbient);
    data.values.emplace(kCellKey, cell);
    return ModuleConfig(data);
}

TEST(ThermalConduction, AnLedBoardOnAPlateHeatsUpAndSpreadsIt) {
    const World world = boardOnPlate();
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<Probe>("driver/out/power", units::W, true, 0.5));
    scheduler.add(std::make_unique<Probe>("motor/plate/loss_power", units::W, true, 1.0));
    auto module = std::make_unique<ThermalConduction>();
    ThermalConduction* heat = module.get();
    scheduler.add(std::move(module), config(0.002, Duration{10'000'000'000}));  // 10 s steps
    std::vector<Probe*> probes;
    for (const char* name :
         {"thermal/led_1/temperature", "thermal/board/max_temperature", "thermal/board/temperature",
          "thermal/plate/temperature", "thermal/led_1/heat", "thermal/led_1/radiant_power",
          "thermal/driver/heat", "thermal/board/heat_flow_out", "thermal/plate/heat_flow_out",
          "thermal/board/overtemp"}) {
        auto probe = std::make_unique<Probe>(name,
                                             std::string(name).ends_with("temperature") ? units::K
                                             : std::string(name).ends_with("overtemp")  ? units::one
                                                                                        : units::W,
                                             false);
        probes.push_back(probe.get());
        scheduler.add(std::move(probe));
    }
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_TRUE(wired) << to_string(wired.error().front());
    for (const std::string& note : heat->notes()) {
        std::printf("note: %s\n", note.c_str());
    }
    std::printf("%zu cells\n", heat->model().cellCount());
    ASSERT_TRUE(scheduler.stepUntil(SimTime{3 * 3'600'000'000'000}).ok());  // three hours
    const auto last = [&](std::size_t i) { return probes[i]->seen.back(); };
    std::printf("led %.2f K, board max %.2f mean %.2f, plate %.2f; out %.3f + %.3f W\n", last(0),
                last(1), last(2), last(3), last(7), last(8));
    EXPECT_NEAR(last(4), 3.0 * 0.65, 1e-12);  // the LED's heat: what is not light
    EXPECT_NEAR(last(5), 3.0 * 0.35, 1e-12);
    EXPECT_NEAR(last(6), 0.5, 1e-12);
    // Bare FR-4 spreads heat poorly: 1.95 W from a 3 mm square puts the LED hundreds of kelvin
    // over the board (spreading resistance about 1/(4·k·a), a few hundred K/W), which is why
    // LEDs sit on metal-core boards.
    EXPECT_GT(last(0), last(2) + 100.0);
    EXPECT_GT(last(2), last(3));  // the board is hotter than the plate it heats
    EXPECT_GT(last(3), kAmbient + 5.0);
    // Steady after three hours: what goes in (1.95 + 0.5 + 1.0 W) comes out.
    EXPECT_NEAR(last(7) + last(8), 3.45, 0.01);
    EXPECT_EQ(last(9), 0.0);
}

// A motor's losses heat the link that holds its stator, at its joint, as <motor>_mount; with the
// electrical network off (no loss signal) nothing heats.
TEST(ThermalConduction, AMotorsLossesHeatItsMount) {
    const auto world = [] {
        World w;
        const BodyId arm =
            w.addBody("arm", kWorldBody, Pose{}, 0.05, {0, 0, 0}, {1e-5, 1e-5, 1e-5, 0, 0, 0});
        w.addCollider(arm, ShapeType::Box, {0.1, 0.01, 0.005}, Pose{}, 0.5);
        const BodyId rotor = w.addBody("rotor", arm, Pose{{0.08, 0.0, 0.01}, {1, 0, 0, 0}}, 0.01,
                                       {0, 0, 0}, {1e-6, 1e-6, 1e-6, 0, 0, 0});
        w.addJoint("rotor_joint", JointType::Revolute, rotor, {0, 0, 1}, true);
        const PrimId motorPrim = w.addPrim("/bot/rotor_motor", "rotor_motor");
        SchemaInstance m;
        m.schema = "ElectricalMotorAPI_1";
        m.prim = motorPrim;
        const PrimId jointPrim = w.addPrim("/bot/rotor_joint", "rotor_joint");
        m.relationships = {{"elec:joint", {jointPrim}}};
        w.schemaInstances.push_back(m);
        SchemaInstance a;
        a.schema = "ThermalMaterialAPI_1";
        a.prim = w.bodies.prim[arm.index];
        a.tokens = {{"thermal:material:preset", "aluminium_6061"}};
        w.schemaInstances.push_back(a);
        return w;
    }();
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<Probe>("motor/rotor_motor/loss_power", units::W, true, 5.0));
    auto module = std::make_unique<ThermalConduction>();
    ThermalConduction* heat = module.get();
    scheduler.add(std::move(module), config(0.004));
    auto mount = std::make_unique<Probe>("thermal/rotor_motor_mount/heat", units::W, false);
    Probe* seen = mount.get();
    scheduler.add(std::move(mount));
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_TRUE(wired) << to_string(wired.error().front());
    ASSERT_TRUE(scheduler.stepUntil(SimTime{60'000'000'000}).ok());
    EXPECT_NEAR(seen->seen.back(), 5.0, 1e-12);
    // The heat went in at the arm's end, under the rotor: hottest there.
    const ConductionModel& model = heat->model();
    const auto cells = model.sourceCells(0, 0);
    ASSERT_FALSE(cells.empty());
    for (const std::size_t c : cells) {
        EXPECT_GT(model.centre(c)[0], 0.06);
    }
    const auto& t = heat->temperatures();
    EXPECT_NEAR(model.sourceMean(t, 0, 0), model.maximum(t, 0), 0.5);
    EXPECT_GT(model.maximum(t, 0) - model.minimum(t, 0), 0.05);
}

TEST(ThermalConduction, MaterialsAndSourcesAreChecked) {
    const auto refused = [](const World& world, ModuleConfig c) {
        Scheduler scheduler;
        scheduler.add(std::make_unique<Probe>("driver/out/power", units::W, true, 0.5));
        scheduler.add(std::make_unique<ThermalConduction>(), std::move(c));
        auto wired = scheduler.wire(WorldView(world));
        return wired ? std::string() : to_string(wired.error().front());
    };
    EXPECT_NE(refused(boardOnPlate("unobtainium"), config()).find("no thermal material preset"),
              std::string::npos);
    ModuleConfigData none;
    EXPECT_NE(refused(boardOnPlate(), ModuleConfig(none)).find("ambient temperature"),
              std::string::npos);
    World orphan = boardOnPlate();
    orphan.schemaInstances.erase(orphan.schemaInstances.begin() + 1);  // the board's material
    EXPECT_NE(refused(orphan, config()).find("no thermal material"), std::string::npos);
}

TEST(ThermalConduction, SavesAndRestoresItsTemperatures) {
    const World world = boardOnPlate();
    const auto make = [&](Scheduler& s) {
        s.add(std::make_unique<Probe>("driver/out/power", units::W, true, 0.5));
        auto m = std::make_unique<ThermalConduction>();
        ThermalConduction* raw = m.get();
        s.add(std::move(m), config(0.004));
        EXPECT_TRUE(s.wire(WorldView(world)));
        return raw;
    };
    Scheduler a(SchedulerOptions{.deterministic = true});
    ThermalConduction* ma = make(a);
    ASSERT_TRUE(a.stepUntil(SimTime{60'000'000'000}).ok());
    StateWriter writer;
    ASSERT_TRUE(ma->saveState(writer).ok());
    Scheduler b(SchedulerOptions{.deterministic = true});
    ThermalConduction* mb = make(b);
    StateReader reader(writer.bytes());
    ASSERT_TRUE(mb->restoreState(reader).ok());
    EXPECT_EQ(mb->temperatures(), ma->temperatures());
}

}  // namespace
}  // namespace lagrange::thermal
