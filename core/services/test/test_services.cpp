// Roadmap 0.2-A12: core services. Acceptance: two wrench writers on one body sum correctly.
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/services/Log.hpp"
#include "lagrange/services/WrenchSum.hpp"

#include <array>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::services {
namespace {

using namespace literals;

/// Writes a constant wrench as body/<body>/wrench/<source>, or reads body/<body>/external_wrench.
class WrenchProbe final : public IModule {
public:
    WrenchProbe(std::string body, std::string source, std::array<double, 6> wrench,
                Duration step = 1_ms, std::string frame = "world_local")
        : body_(std::move(body))
        , source_(std::move(source))
        , wrench_(wrench)
        , step_(step)
        , frame_(std::move(frame))
        , name_(source_.empty() ? "reader" : source_) {}

    std::vector<std::array<double, 6>> seen;

    [[nodiscard]] ModuleInfo info() const override {
        return {"test", name_, 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return step_; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec spec;
        spec.type = SignalType::Wrench;
        spec.unit = units::N;
        spec.frame = frame_;
        if (source_.empty()) {
            spec.name = "body/" + body_ + "/external_wrench";
            id_ = d.input(spec);
        } else {
            spec.name = "body/" + body_ + "/wrench/" + source_;
            id_ = d.output(spec);
        }
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        if (!source_.empty()) {
            bus.writeN(id_, wrench_);
        }
        return {};
    }
    Status step(const StepContext&, SignalBus& bus) override {
        if (!source_.empty()) {
            bus.writeN(id_, wrench_);
        } else {
            const std::span<const double> w = bus.readN(id_);
            std::array<double, 6> copy{};
            std::copy(w.begin(), w.end(), copy.begin());
            seen.push_back(copy);
        }
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    std::string body_;
    std::string source_;
    std::array<double, 6> wrench_;
    Duration step_;
    std::string frame_;
    std::string name_;  // ModuleInfo keeps a view of it
    SignalId id_;
};

TEST(WrenchSum, TwoWritersOnOneBodySum) {
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<WrenchProbe>("base", "aero",
                                                std::array<double, 6>{1, 2, 3, 0.1, 0.2, 0.3}));
    // A slower writer: its wrench holds between its steps and still adds in full.
    scheduler.add(std::make_unique<WrenchProbe>("base", "rotor",
                                                std::array<double, 6>{10, 0, -5, 0, 1, 0}, 10_ms));
    // Another body: summed on its own.
    scheduler.add(
        std::make_unique<WrenchProbe>("arm", "hydro", std::array<double, 6>{0, 0, 7, 0, 0, 0}));
    auto sum = std::make_unique<WrenchSum>();
    WrenchSum* collector = sum.get();
    scheduler.add(std::move(sum));
    auto reader = std::make_unique<WrenchProbe>("base", "", std::array<double, 6>{});
    WrenchProbe* base = reader.get();
    scheduler.add(std::move(reader));
    const World world;
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));

    ASSERT_EQ(collector->bodies().size(), 2U);
    EXPECT_EQ(collector->bodies()[0].name, "arm");
    EXPECT_EQ(collector->bodies()[1].name, "base");
    EXPECT_EQ(collector->bodies()[1].sources, (std::vector<std::string>{"aero", "rotor"}));

    ASSERT_TRUE(scheduler.stepUntil(SimTime{20'000'000}).ok());
    ASSERT_FALSE(base->seen.empty());
    const std::array<double, 6> expected{11, 2, -2, 0.1, 1.2, 0.3};
    for (std::size_t k = 0; k < 6; ++k) {
        EXPECT_DOUBLE_EQ(base->seen.back()[k], expected.at(k)) << "component " << k;
    }
}

TEST(WrenchSum, AContributionInAnotherFrameIsRefused) {
    Scheduler scheduler;
    scheduler.add(
        std::make_unique<WrenchProbe>("base", "aero", std::array<double, 6>{}, 1_ms, "body:base"));
    scheduler.add(std::make_unique<WrenchSum>());
    const World world;
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_FALSE(wired);
    EXPECT_NE(to_string(wired.error().front()).find("world_local"), std::string::npos);
}

TEST(WrenchSum, NoContributionsMeansNoOutputs) {
    Scheduler scheduler;
    auto sum = std::make_unique<WrenchSum>();
    WrenchSum* collector = sum.get();
    scheduler.add(std::move(sum));
    const World world;
    ASSERT_TRUE(scheduler.wire(WorldView(world)));
    EXPECT_TRUE(collector->bodies().empty());
}

TEST(Log, LinesGoToTheSinkThenBackToStderr) {
    std::vector<std::string> lines;
    setLogSink([&](LogLevel level, std::string_view source, std::string_view text) {
        lines.push_back(std::to_string(static_cast<int>(level)) + " " + std::string(source) + ": " +
                        std::string(text));
    });
    log(LogLevel::Warning, "electrical/acme", "bus voltage low");
    setLogSink({});
    log(LogLevel::Info, "test", "to stderr, not the sink");
    ASSERT_EQ(lines.size(), 1U);
    EXPECT_EQ(lines[0], "2 electrical/acme: bus voltage low");
}

}  // namespace
}  // namespace lagrange::services
