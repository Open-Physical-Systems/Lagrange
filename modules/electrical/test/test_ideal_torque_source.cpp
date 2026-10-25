// The electrical domain's ideal substitute: commanded torque reaches the joint exactly.
#include "lagrange/electrical/IdealTorqueSource.hpp"
#include "lagrange/scheduler/Scheduler.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::electrical {
namespace {

using namespace literals;

/// Writes `signal` as f(t), or reads it each step.
class Probe final : public IModule {
public:
    Probe(std::string name, std::string signal, Unit unit, bool writer, double value = 0.0)
        : name_(std::move(name))
        , signal_(std::move(signal))
        , unit_(unit)
        , writer_(writer)
        , value_(value) {}
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
            bus.write(id_, value_);
        }
        return {};
    }
    Status step(const StepContext& c, SignalBus& bus) override {
        if (writer_) {
            bus.write(id_, value_ * (c.t + c.dt).seconds());
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
    double value_;
    SignalId id_;
};

TEST(IdealTorqueSource, CommandedTorqueReachesEveryJoint) {
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(
        std::make_unique<Probe>("left", "joint/left_wheel/torque_command", units::N_m, true, 2.0));
    scheduler.add(
        std::make_unique<Probe>("lift", "joint/lift/torque_command", units::N, true, 5.0));
    auto source = std::make_unique<IdealTorqueSource>();
    IdealTorqueSource* ideal = source.get();
    scheduler.add(std::move(source));
    auto wheel = std::make_unique<Probe>("wheel", "joint/left_wheel/torque", units::N_m, false);
    auto lift = std::make_unique<Probe>("liftreader", "joint/lift/torque", units::N, false);
    Probe* wheelSeen = wheel.get();
    Probe* liftSeen = lift.get();
    scheduler.add(std::move(wheel));
    scheduler.add(std::move(lift));
    const World world;
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    EXPECT_EQ(ideal->joints(), (std::vector<std::string>{"left_wheel", "lift"}));
    EXPECT_TRUE(ideal->info().isSubstitute);
    ASSERT_TRUE(scheduler.stepUntil(SimTime{5'000'000}).ok());
    ASSERT_EQ(wheelSeen->seen.size(), 5U);
    for (std::size_t k = 0; k < 5; ++k) {
        const double t = 0.001 * static_cast<double>(k + 1);
        EXPECT_DOUBLE_EQ(wheelSeen->seen[k], 2.0 * t);  // N·m, unchanged
        EXPECT_DOUBLE_EQ(liftSeen->seen[k], 5.0 * t);   // a slide: N, unchanged
    }
}

TEST(IdealTorqueSource, WithoutCommandsItDrivesNothing) {
    Scheduler scheduler;
    auto source = std::make_unique<IdealTorqueSource>();
    IdealTorqueSource* ideal = source.get();
    scheduler.add(std::move(source));
    const World world;
    ASSERT_TRUE(scheduler.wire(WorldView(world)));
    EXPECT_TRUE(ideal->joints().empty());
}

}  // namespace
}  // namespace lagrange::electrical
