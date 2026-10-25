// Scheduler overhead and signal bus access (roadmap H6). Baselines: docs/design/benchmarks.md.
#include "lagrange/scheduler/Scheduler.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <benchmark/benchmark.h>

namespace lagrange {
namespace {

using namespace literals;

/// A module that reads the previous module's output and writes its own: the least work a
/// wired module can do, so a tick costs the scheduler's bookkeeping plus one read and one write
/// per module.
class ChainModule final : public IModule {
public:
    ChainModule(int index, Duration step, int accesses)
        : name_("m" + std::to_string(index)), index_(index), step_(step), accesses_(accesses) {}

    [[nodiscard]] ModuleInfo info() const override {
        return {"bench", name_, 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return step_; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& declarer) override {
        SignalSpec out;
        out.name = "bench/" + name_ + "/x";
        out.unit = units::m;
        output_ = declarer.output(out);
        if (index_ > 0) {
            SignalSpec in;
            in.name = "bench/m" + std::to_string(index_ - 1) + "/x";
            in.unit = units::m;
            input_ = declarer.input(in);
        }
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        bus.write(output_, 0.0);
        return {};
    }
    Status step(const StepContext&, SignalBus& bus) override {
        double sum = 0.0;
        for (int k = 0; k < accesses_; ++k) {
            if (input_.valid()) {
                sum += bus.read(input_);
            }
            bus.write(output_, sum + 1.0);
        }
        benchmark::DoNotOptimize(sum);
        return {};
    }
    void shutdown() override {}
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    std::string name_;
    int index_;
    Duration step_;
    int accesses_;
    SignalId input_;
    SignalId output_;
};

/// `modules` chained modules; with `multirate`, every other one steps ten times as often.
std::unique_ptr<Scheduler> chain(int modules, bool multirate, int accesses) {
    auto scheduler = std::make_unique<Scheduler>(SchedulerOptions{.deterministic = true});
    for (int i = 0; i < modules; ++i) {
        const Duration dt = multirate && i % 2 == 1 ? 100_us : 1_ms;
        scheduler->add(std::make_unique<ChainModule>(i, dt, accesses));
    }
    static const World world;
    if (!scheduler->wire(WorldView(world))) {
        return nullptr;
    }
    return scheduler;
}

// Cost per module step of the scheduler itself (input preparation, commit, ordering): one
// read and one write per step. Reported as module steps per second.
void SchedulerTick(benchmark::State& state) {
    const int modules = static_cast<int>(state.range(0));
    const bool multirate = state.range(1) != 0;
    auto scheduler = chain(modules, multirate, 1);
    if (!scheduler) {
        state.SkipWithError("wiring failed");
        return;
    }
    // Module steps per base tick: all modules at 1 ms, or half at 100 µs (ticks of 100 µs).
    const double stepsPerTick = multirate ? modules / 2.0 + modules / 2.0 / 10.0 : modules;
    for (auto _ : state) {
        benchmark::DoNotOptimize(scheduler->stepTick());
    }
    state.SetItemsProcessed(
        static_cast<std::int64_t>(static_cast<double>(state.iterations()) * stepsPerTick));
}
BENCHMARK(SchedulerTick)
    ->ArgNames({"modules", "multirate"})
    ->Args({1, 0})
    ->Args({10, 0})
    ->Args({100, 0})
    ->Args({100, 1});

// Single bus reads and writes from inside a step, 1000 per step and kind, so the scheduler's
// share is negligible. Reported as accesses per second.
void BusAccess(benchmark::State& state) {
    constexpr int kAccesses = 1000;
    auto scheduler = chain(2, false, kAccesses);
    if (!scheduler) {
        state.SkipWithError("wiring failed");
        return;
    }
    for (auto _ : state) {
        benchmark::DoNotOptimize(scheduler->stepTick());
    }
    // Module 0 writes; module 1 reads and writes.
    state.SetItemsProcessed(state.iterations() * 3 * kAccesses);
}
BENCHMARK(BusAccess);

}  // namespace
}  // namespace lagrange
