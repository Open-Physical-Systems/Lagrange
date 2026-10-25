// The thermal module on the bus: heat sources found by name, derating written back, the site
// parameter required, and the ambient substitute.
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/thermal/ThermalNetwork.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::thermal {
namespace {

using namespace literals;

/// Writes a constant `value` to `signal`, or reads it, every 100 ms.
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

/// A motor node linked to the chassis node, as in the sample AMR's overlay.
World motorAndChassis(bool withCapacity = true) {
    World w;
    const PrimId motor = w.addPrim("/amr/base_link/left_drive_motor", "left_drive_motor");
    const PrimId chassis = w.addPrim("/amr/base_link", "base_link");
    w.schemaInstances.push_back({});
    SchemaInstance& m = w.schemaInstances.back();
    m.schema = "ThermalAPI_1";
    m.prim = motor;
    m.reals = {{"thermal:convectionArea", 0.02},
               {"thermal:deratingStart", 393.15},
               {"thermal:maxTemperature", 428.15}};
    if (withCapacity) {
        m.reals["thermal:heatCapacity"] = 225.0;
    }
    w.schemaInstances.push_back({});
    SchemaInstance& c = w.schemaInstances.back();
    c.schema = "ThermalAPI_1";
    c.prim = chassis;
    c.reals = {{"thermal:heatCapacity", 9000.0}, {"thermal:convectionArea", 0.9}};
    w.schemaInstances.push_back({});
    SchemaInstance& link = w.schemaInstances.back();
    link.schema = "ThermalLinkAPI_1";
    link.instance = "to_chassis";
    link.prim = motor;
    link.reals = {{"thermal:link:to_chassis:conductance", 0.5}};
    link.relationships = {{"thermal:link:to_chassis:target", {chassis}}};
    return w;
}

ModuleConfig withAmbient(double kelvin) {
    ModuleConfigData data;
    data.values.emplace(kAmbientKey, kelvin);
    return ModuleConfig(data);
}

TEST(ThermalNetwork, TheMotorHeatsFromItsLossAndDerates) {
    const World world = motorAndChassis();
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    constexpr double kLoss = 80.0;  // W: an overloaded motor
    scheduler.add(
        std::make_unique<Probe>("motor/left_drive_motor/loss_power", units::W, true, kLoss));
    auto module = std::make_unique<ThermalNetwork>();
    ThermalNetwork* thermal = module.get();
    scheduler.add(std::move(module), withAmbient(298.15));
    auto temperature =
        std::make_unique<Probe>("thermal/left_drive_motor/temperature", units::K, false);
    auto derating = std::make_unique<Probe>("thermal/left_drive_motor/derating", units::one, false);
    Probe* t = temperature.get();
    Probe* d = derating.get();
    scheduler.add(std::move(temperature));
    scheduler.add(std::move(derating));
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_TRUE(wired) << to_string(wired.error().front());
    EXPECT_EQ(thermal->nodes(), (std::vector<std::string>{"left_drive_motor", "base_link"}));
    EXPECT_EQ(thermal->sources()[0],
              (std::vector<std::string>{"motor/left_drive_motor/loss_power"}));
    // 80 W into 225 J/K: 0.36 K/s at first. After four hours it is at the steady state: the
    // motor's own convection (10 W/(m²·K) × 0.02 m²) in parallel with the link (0.5 W/K) in
    // series with the chassis's convection (10 × 0.9), past the derating start but below the
    // limit.
    ASSERT_TRUE(scheduler.stepUntil(SimTime{4 * 3'600'000'000'000}).ok());
    EXPECT_NEAR(t->seen.front(), 298.15 + kLoss * 0.1 / 225.0, 0.01);
    const double steady = 298.15 + kLoss / (0.2 + 1.0 / (1.0 / 0.5 + 1.0 / 9.0));
    EXPECT_NEAR(t->seen.back(), steady, 0.5);
    EXPECT_GT(steady, 393.15);
    EXPECT_LT(steady, 428.15);
    EXPECT_NEAR(d->seen.back(), (428.15 - t->seen.back()) / 35.0, 1e-9);
    EXPECT_EQ(d->seen.front(), 1.0);
}

TEST(ThermalNetwork, TheAmbientTemperatureIsRequiredAndCapacitiesChecked) {
    {
        const World world = motorAndChassis();
        Scheduler scheduler;
        scheduler.add(std::make_unique<ThermalNetwork>());  // no ambient temperature
        auto wired = scheduler.wire(WorldView(world));
        ASSERT_FALSE(wired);
        EXPECT_NE(to_string(wired.error().front()).find("ambient temperature"), std::string::npos);
    }
    {
        const World world = motorAndChassis(false);
        Scheduler scheduler;
        scheduler.add(std::make_unique<ThermalNetwork>(), withAmbient(298.15));
        auto wired = scheduler.wire(WorldView(world));
        ASSERT_FALSE(wired);
        EXPECT_NE(to_string(wired.error().front())
                      .find("/amr/base_link/left_drive_motor: a thermal node needs"),
                  std::string::npos)
            << to_string(wired.error().front());
    }
}

TEST(AmbientTemperature, HoldsEveryNodeAtTheAmbient) {
    const World world = motorAndChassis();
    Scheduler scheduler;
    scheduler.add(std::make_unique<AmbientTemperature>(), withAmbient(288.15));
    auto reader = std::make_unique<Probe>("thermal/base_link/temperature", units::K, false);
    Probe* r = reader.get();
    scheduler.add(std::move(reader));
    ASSERT_TRUE(scheduler.wire(WorldView(world)));
    ASSERT_TRUE(scheduler.stepUntil(SimTime{1'000'000'000}).ok());
    EXPECT_EQ(r->seen.back(), 288.15);
    Scheduler without;
    without.add(std::make_unique<AmbientTemperature>());
    EXPECT_FALSE(without.wire(WorldView(world)));
}

}  // namespace
}  // namespace lagrange::thermal
