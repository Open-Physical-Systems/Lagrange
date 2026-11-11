#pragma once
// Test helpers for the built-in kinds: modules that write chosen signals as functions of time,
// so a kind runs on a scheduler without a physics engine.

#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/sensors/Kinds.hpp"

#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::sensors::test {

/// Writes signals given by functions of the time at the end of each 1 ms step.
class Writer final : public IModule {
public:
    using Fn = std::function<std::vector<double>(double t)>;
    struct Out {
        SignalSpec spec;
        Fn value;
        SignalId id{};
    };

    Writer& add(std::string name, SignalType type, Unit unit, Fn value, std::uint32_t length = 1,
                std::string frame = {}) {
        SignalSpec s;
        s.name = std::move(name);
        s.type = type;
        s.unit = unit;
        s.arrayLength = length;
        s.frame = std::move(frame);
        outs_.push_back({s, std::move(value)});
        return *this;
    }
    /// body/<name>/pose at a position (identity orientation) and /twist with that velocity.
    Writer& body(const std::string& name, std::function<std::array<double, 3>(double)> where,
                 std::array<double, 3> velocity = {0, 0, 0}) {
        add(
            "body/" + name + "/pose", SignalType::Pose, units::m,
            [where](double t) {
                const auto p = where(t);
                return std::vector<double>{p[0], p[1], p[2], 1.0, 0.0, 0.0, 0.0};
            },
            1, "world_local");
        add(
            "body/" + name + "/twist", SignalType::Wrench, units::m_per_s,
            [velocity](double) {
                return std::vector<double>{velocity[0], velocity[1], velocity[2], 0.0, 0.0, 0.0};
            },
            1, "world_local");
        return *this;
    }

    ModuleInfo info() const override { return {"test", "writer", 0, 1, false, true}; }
    Duration preferredStep() const override { return Duration{1'000'000}; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        for (Out& o : outs_) {
            o.id = d.output(o.spec);
        }
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        write(bus, 0.0);
        return {};
    }
    Status step(const StepContext& c, SignalBus& bus) override {
        write(bus, (c.t + c.dt).seconds());
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    void write(SignalBus& bus, double t) {
        for (Out& o : outs_) {
            const std::vector<double> v = o.value(t);
            if (v.size() == 1) {
                bus.write(o.id, v[0]);
            } else {
                bus.writeN(o.id, v);
            }
        }
    }
    std::vector<Out> outs_;
};

/// A world, a scheduler with the writer and the kind of sensor 0, wired.
struct KindRun {
    World world;
    std::unique_ptr<Scheduler> scheduler;

    /// Wires `writer` (may be null) and the built-in kind of every custom sensor.
    bool wire(std::unique_ptr<Writer> writer, bool groundTruth = false) {
        scheduler = std::make_unique<Scheduler>(SchedulerOptions{.deterministic = true});
        if (writer) {
            scheduler->add(std::move(writer));
        }
        for (std::uint32_t s = 0; s < world.sensors.names.size(); ++s) {
            auto module = createBuiltinKind(world, s, groundTruth, nullptr, nullptr);
            if (!module) {
                ADD_FAILURE() << "no built-in kind for " << world.sensors.names[s];
                return false;
            }
            scheduler->add(std::move(module));
        }
        auto wired = scheduler->wire(WorldView(world));
        EXPECT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
        return static_cast<bool>(wired);
    }
    bool runTo(double seconds) {
        return scheduler->stepUntil(SimTime{static_cast<std::int64_t>(seconds * 1e9 + 0.5)}).ok();
    }
    [[nodiscard]] double read(const std::string& name) const {
        const SignalBus& bus = scheduler->bus();
        const SignalId id = bus.find(name);
        EXPECT_TRUE(id.valid()) << name;
        return id.valid() ? bus.read(id) : std::nan("");
    }
    [[nodiscard]] std::vector<double> readN(const std::string& name) const {
        const SignalBus& bus = scheduler->bus();
        const SignalId id = bus.find(name);
        EXPECT_TRUE(id.valid()) << name;
        if (!id.valid()) {
            return {};
        }
        const auto v = bus.readN(id);
        return {v.begin(), v.end()};
    }
};

/// A kinematic body (it moves, so it has pose inputs) at the origin.
inline BodyId movingBody(World& w, const std::string& name) {
    const BodyId b = w.addBody(name, kWorldBody, Pose{}, 1.0, {}, {1, 1, 1, 0, 0, 0});
    w.bodies.kinematic.back() = true;
    return b;
}

/// A fixed body at `at` (scenery).
inline BodyId fixedBody(World& w, const std::string& name, std::array<double, 3> at) {
    Pose p;
    p.position = at;
    return w.addBody(name, kWorldBody, p, 1.0, {}, {1, 1, 1, 0, 0, 0});
}

}  // namespace lagrange::sensors::test
