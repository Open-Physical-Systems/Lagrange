#pragma once
// For the built-in kinds' tests: a world with one body, a source module that writes chosen
// scalar signals as another domain would, and a scheduler running it with the kind's module.
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/sensors/Kinds.hpp"

#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::sensors::test {

/// Writes each signal's current value (`values`, changed by the test between steps) every 100 µs.
class Source final : public IModule {
public:
    struct Signal {
        std::string name;
        Unit unit;
        double value{0.0};
    };
    explicit Source(std::shared_ptr<std::vector<Signal>> signals) : signals_(std::move(signals)) {}
    ModuleInfo info() const override { return {"test", "source", 0, 1, false, true}; }
    Duration preferredStep() const override { return Duration{100'000}; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        for (const Signal& s : *signals_) {
            SignalSpec spec;
            spec.name = s.name;
            spec.unit = s.unit;
            ids_.push_back(d.output(spec));
        }
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        return step({}, bus);
    }
    Status step(const StepContext&, SignalBus& bus) override {
        for (std::size_t i = 0; i < ids_.size(); ++i) {
            bus.write(ids_[i], (*signals_)[i].value);
        }
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    std::shared_ptr<std::vector<Source::Signal>> signals_;
    std::vector<SignalId> ids_;
};

struct Rig {
    World world;
    BodyId body;
    std::shared_ptr<std::vector<Source::Signal>> signals =
        std::make_shared<std::vector<Source::Signal>>();
    std::unique_ptr<Scheduler> scheduler;

    Rig() { body = world.addBody("base", kWorldBody, Pose{}, 1.0, {}, {1, 1, 1, 0, 0, 0}); }

    /// A prim carrying `schema`, with its attributes.
    SchemaInstance& addSchema(const std::string& primName, const std::string& schema) {
        SchemaInstance s;
        s.schema = schema;
        s.prim = world.addPrim("/robot/" + primName, primName);
        world.schemaInstances.push_back(std::move(s));
        return world.schemaInstances.back();
    }

    void sensor(const std::string& name, const std::string& kind, double rateHz,
                std::map<std::string, double, std::less<>> numbers,
                std::map<std::string, std::string, std::less<>> text = {}) {
        world.addCustomSensor(name, kind, body, Pose{}, rateHz, std::move(numbers),
                              std::move(text));
    }

    void source(const std::string& name, Unit unit, double value) {
        signals->push_back({name, unit, value});
    }
    void set(const std::string& name, double value) {
        for (auto& s : *signals) {
            if (s.name == name) {
                s.value = value;
            }
        }
    }

    /// Wires the source and every sensor's kind; false (and a failure) when it does not wire.
    bool wire(bool groundTruth = false, std::string* error = nullptr) {
        scheduler = std::make_unique<Scheduler>(SchedulerOptions{.deterministic = true});
        if (!signals->empty()) {
            scheduler->add(std::make_unique<Source>(signals));
        }
        for (std::uint32_t s = 0; s < world.sensors.names.size(); ++s) {
            auto module = createBuiltinKind(world, s, groundTruth, nullptr, nullptr);
            if (!module) {
                ADD_FAILURE() << "no built-in kind " << world.sensors.kind[s];
                return false;
            }
            scheduler->add(std::move(module));
        }
        auto wired = scheduler->wire(WorldView(world));
        if (!wired) {
            if (error != nullptr) {
                *error = to_string(wired.error().front());
            }
            return false;
        }
        return true;
    }

    bool runFor(double seconds) {
        const SimTime until{scheduler->now().ns + std::llround(seconds * 1e9)};
        return scheduler->stepUntil(until).ok();
    }
    [[nodiscard]] double read(const std::string& name) const {
        const SignalBus& bus = scheduler->bus();
        const SignalId id = bus.find(name);
        EXPECT_NE(id.index, SignalId::kInvalid) << name;
        return id.index == SignalId::kInvalid ? std::nan("") : bus.read(id);
    }
};

/// Mean and standard deviation of `n` readings of `signal`, one per `period` s.
inline std::pair<double, double> statistics(Rig& rig, const std::string& signal, int n,
                                            double period) {
    double sum = 0.0;
    double sum2 = 0.0;
    for (int i = 0; i < n; ++i) {
        EXPECT_TRUE(rig.runFor(period));
        const double v = rig.read(signal);
        sum += v;
        sum2 += v * v;
    }
    const double mean = sum / n;
    return {mean, std::sqrt(std::max(0.0, sum2 / n - mean * mean))};
}

}  // namespace lagrange::sensors::test
