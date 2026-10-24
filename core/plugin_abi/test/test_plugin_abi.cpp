// Roadmap 0.2-A11: a module built in C99 against lagrange_plugin.h loads, wires (units checked
// across the ABI) and steps in the scheduler, and keeps its state through save and restore.
#include "lagrange/plugin_abi/PluginLibrary.hpp"
#include "lagrange/scheduler/Scheduler.hpp"

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange {
namespace {

using namespace literals;

/// A C++ module that writes one scalar (`signal`) as f(t), or reads one and records it.
class Probe final : public IModule {
public:
    Probe(std::string name, std::string signal, Unit unit, bool writer,
          std::function<double(double)> f = {})
        : name_(std::move(name))
        , signal_(std::move(signal))
        , unit_(unit)
        , writer_(writer)
        , f_(std::move(f)) {}

    std::vector<double> seen;

    [[nodiscard]] ModuleInfo info() const override {
        return {"test", name_, 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return 1_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec spec;
        spec.name = signal_;
        spec.unit = unit_;
        id_ = writer_ ? d.output(spec) : d.input(spec);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        if (writer_) {
            bus.write(id_, f_(0.0));
        }
        return {};
    }
    Status step(const StepContext& c, SignalBus& bus) override {
        if (writer_) {
            bus.write(id_, f_((c.t + c.dt).seconds()));
        } else {
            seen.push_back(bus.read(id_));
        }
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    std::string name_;
    std::string signal_;
    Unit unit_;
    bool writer_;
    std::function<double(double)> f_;
    SignalId id_;
};

ModuleConfig wheel() {
    ModuleConfigData data;
    data.values["joint"] = std::string("wheel");
    return ModuleConfig(data);
}

TEST(PluginAbi, TheC99SampleLoads) {
    auto library = PluginLibrary::open(LAGRANGE_SAMPLE_PLUGIN);
    ASSERT_TRUE(library) << library.error();
    ASSERT_EQ(library->size(), 1U);
    EXPECT_EQ(library->label(0), "electrical/ideal_torque_source");
    const auto module = library->create(0);
    ASSERT_NE(module, nullptr);
    EXPECT_TRUE(module->info().isSubstitute);
    EXPECT_TRUE(module->info().deterministic);
    EXPECT_EQ(module->preferredStep(), 1_ms);
}

TEST(PluginAbi, ItRunsInTheSchedulerBetweenCppModules) {
    auto library = PluginLibrary::open(LAGRANGE_SAMPLE_PLUGIN);
    ASSERT_TRUE(library) << library.error();
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<Probe>("command", "joint/wheel/torque_command", units::N_m,
                                          true, [](double t) { return 2.0 * t; }));
    scheduler.add(library->create(0), wheel());
    auto sink = std::make_unique<Probe>("sink", "joint/wheel/torque", units::N_m, false);
    Probe* seen = sink.get();
    scheduler.add(std::move(sink));
    const World world;
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    ASSERT_TRUE(scheduler.stepUntil(SimTime{10'000'000}).ok());
    ASSERT_EQ(seen->seen.size(), 10U);
    // The plugin passes the command through in the same tick (writer before reader).
    for (std::size_t k = 0; k < seen->seen.size(); ++k) {
        EXPECT_DOUBLE_EQ(seen->seen[k], 2.0 * 0.001 * static_cast<double>(k + 1));
    }
}

TEST(PluginAbi, UnitsAreCheckedAcrossTheAbi) {
    auto library = PluginLibrary::open(LAGRANGE_SAMPLE_PLUGIN);
    ASSERT_TRUE(library) << library.error();
    Scheduler scheduler;
    scheduler.add(std::make_unique<Probe>("command", "joint/wheel/torque_command", units::N_m,
                                          true, [](double) { return 0.0; }));
    scheduler.add(library->create(0), wheel());
    // Reads the plugin's N·m output as a force.
    scheduler.add(std::make_unique<Probe>("sink", "joint/wheel/torque", units::N, false));
    const World world;
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_FALSE(wired);
    EXPECT_EQ(wired.error().front().kind, WiringError::Kind::UnitMismatch);
    EXPECT_NE(to_string(wired.error().front()).find("electrical/ideal_torque_source"),
              std::string::npos);
}

TEST(PluginAbi, ItsStateIsSavedAndRestored) {
    auto library = PluginLibrary::open(LAGRANGE_SAMPLE_PLUGIN);
    ASSERT_TRUE(library) << library.error();
    const World world;
    const auto build = [&](Scheduler& scheduler) {
        scheduler.add(std::make_unique<Probe>("command", "joint/wheel/torque_command",
                                              units::N_m, true, [](double t) { return t; }));
        scheduler.add(library->create(0), wheel());
        return static_cast<bool>(scheduler.wire(WorldView(world)));
    };
    Scheduler a(SchedulerOptions{.deterministic = true});
    ASSERT_TRUE(build(a));
    ASSERT_TRUE(a.stepUntil(SimTime{5'000'000}).ok());
    StateWriter writer;
    ASSERT_TRUE(a.saveState(writer).ok());
    Scheduler b(SchedulerOptions{.deterministic = true});
    ASSERT_TRUE(build(b));
    StateReader reader(writer.bytes());
    ASSERT_TRUE(b.restoreState(reader).ok());
    ASSERT_TRUE(a.stepUntil(SimTime{9'000'000}).ok());
    ASSERT_TRUE(b.stepUntil(SimTime{9'000'000}).ok());
    EXPECT_EQ(a.stateHash().value(), b.stateHash().value());
}

TEST(PluginAbi, BadLibrariesAreRefusedWithAReason) {
    auto missing = PluginLibrary::open("/nonexistent/plugin.so");
    ASSERT_FALSE(missing);
    EXPECT_NE(missing.error().find("cannot load plugin"), std::string::npos);

    auto other = PluginLibrary::open(LAGRANGE_SAMPLE_PLUGIN_OTHER_MAJOR);
    ASSERT_FALSE(other);
    EXPECT_NE(other.error().find("offers no module for plugin ABI 0.x"), std::string::npos)
        << other.error();
}

}  // namespace
}  // namespace lagrange
