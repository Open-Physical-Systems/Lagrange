// Roadmap 0.2-H2. Acceptance: an injected nondeterminism (a thread-order dependent sum) is
// caught; a deterministic run passes.
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/services/DeterminismCheck.hpp"

#include <atomic>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::services {
namespace {

using namespace literals;

/// Integrates a value each step. With `threads` > 1 it adds 512 terms of very different size
/// from several threads in whatever order they run: floating-point addition is not
/// associative, so the low bits depend on scheduling.
class Accumulator final : public IModule {
public:
    explicit Accumulator(int threads) : threads_(threads) {}

    [[nodiscard]] ModuleInfo info() const override {
        return {"test", "accumulator", 0, 1, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return 1_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec out;
        out.name = "test/accumulator/value";
        out.unit = units::one;
        id_ = d.output(out);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        bus.write(id_, value_);
        return {};
    }
    Status step(const StepContext& context, SignalBus& bus) override {
        const auto term = [&](int k) {
            return std::ldexp(1.0 + 1e-3 * static_cast<double>(k + context.stepIndex % 7),
                              (k * 13) % 60 - 30);
        };
        if (threads_ <= 1) {
            for (int k = 0; k < kTerms; ++k) {
                value_ += term(k);
            }
        } else {
            std::atomic<double> sum{value_};
            std::vector<std::thread> pool;
            for (int t = 0; t < threads_; ++t) {
                pool.emplace_back([&, t] {
                    for (int k = t; k < kTerms; k += threads_) {
                        double seen = sum.load();
                        while (!sum.compare_exchange_weak(seen, seen + term(k))) {
                        }
                        std::this_thread::yield();  // let the threads interleave
                    }
                });
            }
            for (std::thread& thread : pool) {
                thread.join();
            }
            value_ = sum.load();
        }
        bus.write(id_, value_);
        return {};
    }
    Status saveState(StateWriter& writer) const override {
        writer.put(value_);
        return {};
    }
    Status restoreState(StateReader& reader) override {
        reader.get(value_);
        return {};
    }

private:
    static constexpr int kTerms = 512;
    int threads_;
    double value_{0.0};
    SignalId id_;
};

class SchedulerRun final : public CheckedRun {
public:
    explicit SchedulerRun(int threads) {
        scheduler_.add(std::make_unique<Accumulator>(threads));
        wired_ = static_cast<bool>(scheduler_.wire(WorldView(world_)));
    }
    Status stepUntil(SimTime t) override { return scheduler_.stepUntil(t); }
    [[nodiscard]] Expected<std::uint64_t, Status> stateHash() const override {
        return scheduler_.stateHash();
    }
    [[nodiscard]] bool wired() const { return wired_; }

private:
    World world_;
    Scheduler scheduler_{SchedulerOptions{.deterministic = true}};
    bool wired_{false};
};

auto maker(int threads) {
    return [threads]() -> Expected<std::unique_ptr<CheckedRun>, std::string> {
        auto run = std::make_unique<SchedulerRun>(threads);
        if (!run->wired()) {
            return Unexpected{std::string("wiring failed")};
        }
        return std::unique_ptr<CheckedRun>(std::move(run));
    };
}

TEST(DeterminismCheck, ADeterministicRunPasses) {
    const DeterminismReport report = checkDeterminism(maker(1), SimTime{2'000'000'000}, 100_ms);
    EXPECT_TRUE(report.error.empty()) << report.error;
    EXPECT_TRUE(report.identical);
    EXPECT_EQ(report.comparisons, 21U);
}

TEST(DeterminismCheck, AThreadOrderDependentSumIsCaught) {
    const DeterminismReport report = checkDeterminism(maker(8), SimTime{2'000'000'000}, 10_ms);
    EXPECT_TRUE(report.error.empty()) << report.error;
    EXPECT_FALSE(report.identical);
    ASSERT_TRUE(report.firstDivergence.has_value());
    EXPECT_GT(report.firstDivergence->ns, 0);
}

}  // namespace
}  // namespace lagrange::services
