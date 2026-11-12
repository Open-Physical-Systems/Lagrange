#pragma once
// Shared by the environment kinds' tests (test_kinds_*.cpp): a module that writes given signals
// as functions of time, as physics, the weather or the aerodynamics domain would, and a run of
// one built-in kind against it.

#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/sensors/Kinds.hpp"

#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::sensors::test {

using namespace literals;

struct Feed {
    SignalSpec spec;
    std::function<std::vector<double>(double)> value;  // components at time t (s)
};

inline SignalSpec makeSpec(std::string name, SignalType type, Unit unit, std::string frame = {}) {
    SignalSpec s;
    s.name = std::move(name);
    s.type = type;
    s.unit = unit;
    s.frame = std::move(frame);
    return s;
}

/// A constant feed.
inline Feed constant(std::string name, SignalType type, Unit unit, std::vector<double> value,
                     std::string frame = {}) {
    return {makeSpec(std::move(name), type, unit, std::move(frame)),
            [value = std::move(value)](double) { return value; }};
}

/// Writes its feeds every millisecond.
class Source final : public IModule {
public:
    explicit Source(std::vector<Feed> feeds) : feeds_(std::move(feeds)) {}
    [[nodiscard]] ModuleInfo info() const override {
        return {"test", "source", 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return 1_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        ids_.clear();
        for (const Feed& f : feeds_) {
            ids_.push_back(d.output(f.spec));
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
        for (std::size_t i = 0; i < feeds_.size(); ++i) {
            const std::vector<double> v = feeds_[i].value(t);
            bus.writeN(ids_[i], v);
        }
    }
    std::vector<Feed> feeds_;
    std::vector<SignalId> ids_;
};

struct KindRun {
    World world;
    std::unique_ptr<Scheduler> scheduler;
    [[nodiscard]] double read(const std::string& name) const {
        const SignalBus& bus = scheduler->bus();
        const SignalId id = bus.find(name);
        EXPECT_TRUE(id.valid()) << name;
        return id.valid() ? bus.read(id) : 0.0;
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

/// Wires sensor 0 of `run.world` (a built-in kind) after a Source of `feeds`; null on failure
/// (the wiring errors are reported).
inline bool start(KindRun& run, std::vector<Feed> feeds, bool groundTruth = false,
                  ModuleConfigData config = {}) {
    run.scheduler = std::make_unique<Scheduler>(SchedulerOptions{.deterministic = true});
    if (!feeds.empty()) {
        run.scheduler->add(std::make_unique<Source>(std::move(feeds)));
    }
    auto module = createBuiltinKind(run.world, 0, groundTruth, nullptr, nullptr);
    EXPECT_TRUE(module) << "not a built-in kind";
    if (!module) {
        return false;
    }
    run.scheduler->add(std::move(module), ModuleConfig(std::move(config)));
    auto wired = run.scheduler->wire(WorldView(run.world));
    EXPECT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    return static_cast<bool>(wired);
}

/// A free body "frame" at the origin carrying custom sensor `name` of `kind`.
inline BodyId rig(World& w, const std::string& name, const std::string& kind, const Pose& mount,
                  double rateHz, std::map<std::string, double, std::less<>> settings = {},
                  std::map<std::string, std::string, std::less<>> text = {}) {
    const BodyId frame = w.addBody("frame", kWorldBody, Pose{}, 1.0, {}, {0.1, 0.1, 0.1, 0, 0, 0});
    w.addJoint("frame_free", JointType::Free, frame, {0, 0, 1}, false);
    w.addCustomSensor(name, kind, frame, mount, rateHz, std::move(settings), std::move(text));
    return frame;
}

/// The body's pose and twist feeds.
inline std::vector<Feed> bodyFeeds(const Pose& pose, const std::array<double, 6>& twist) {
    return {constant("body/frame/pose", SignalType::Pose, units::m,
                     {pose.position[0], pose.position[1], pose.position[2], pose.orientation[0],
                      pose.orientation[1], pose.orientation[2], pose.orientation[3]},
                     "world_local"),
            constant("body/frame/twist", SignalType::Wrench, units::m_per_s,
                     {twist[0], twist[1], twist[2], twist[3], twist[4], twist[5]}, "world_local")};
}

/// Steps to `t1` and saves the run, steps on to `t2` reading `signals` after every `every`;
/// restores and steps again: the restored run must read exactly as the first (NaN as NaN).
inline void expectRestoredRunRepeats(KindRun& r, const std::vector<std::string>& signals,
                                     std::int64_t t1, std::int64_t t2, std::int64_t every) {
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{t1}).ok());
    StateWriter saved;
    ASSERT_TRUE(r.scheduler->saveState(saved).ok());
    const auto pass = [&] {
        std::vector<std::vector<double>> out;
        for (std::int64_t t = t1 + every; t <= t2; t += every) {
            EXPECT_TRUE(r.scheduler->stepUntil(SimTime{t}).ok());
            for (const std::string& name : signals) {
                out.push_back(r.readN(name));
            }
        }
        return out;
    };
    const auto first = pass();
    StateReader reader(saved.bytes());
    ASSERT_TRUE(r.scheduler->restoreState(reader).ok());
    const auto second = pass();
    ASSERT_EQ(first.size(), second.size());
    for (std::size_t i = 0; i < first.size(); ++i) {
        ASSERT_EQ(first[i].size(), second[i].size());
        for (std::size_t k = 0; k < first[i].size(); ++k) {
            const double a = first[i][k], b = second[i][k];
            EXPECT_TRUE(a == b || (std::isnan(a) && std::isnan(b)))
                << signals[i % signals.size()] << "[" << k << "] at reading " << i << ": " << a
                << " then " << b;
        }
    }
}

}  // namespace lagrange::sensors::test
