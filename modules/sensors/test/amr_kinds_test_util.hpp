#pragma once
// Shared by the built-in kinds' tests (test_kinds_*.cpp): poses, a constant writer and a run.

#include "lagrange/rigid_body/MujocoBackend.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/sensors/Sensors.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::sensors::kindtest {

constexpr double kGravity = 9.80665;  // m/s², standard gravity (CGPM 1901)

inline Pose at(double x, double y, double z) {
    return Pose{{x, y, z}};
}

/// Rotation by `angle` about the unit axis (x, y, z), at (px, py, pz).
inline Pose turned(double angle, double x, double y, double z, double px = 0.0, double py = 0.0,
                   double pz = 0.0) {
    const double s = std::sin(angle / 2.0);
    return Pose{{px, py, pz}, {std::cos(angle / 2.0), s * x, s * y, s * z}};
}

inline std::array<double, 6> diagonal(double v) {
    return {v, v, v, 0.0, 0.0, 0.0};
}

/// Writes signals from a function of time every millisecond (torques, wrenches, pose commands).
class Writer final : public IModule {
public:
    struct Output {
        SignalSpec spec;
        std::function<std::vector<double>(double)> value;  // of the step's end time, s
    };
    explicit Writer(std::vector<Output> outputs) : outputs_(std::move(outputs)) {}
    [[nodiscard]] ModuleInfo info() const override { return {"controller", "test_writer", 0, 1}; }
    [[nodiscard]] Duration preferredStep() const override { return Duration{1'000'000}; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        ids_.clear();
        for (const Output& o : outputs_) {
            ids_.push_back(d.output(o.spec));
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
        for (std::size_t k = 0; k < outputs_.size(); ++k) {
            const std::vector<double> v = outputs_[k].value(t);
            bus.writeN(ids_[k], v);
        }
    }
    std::vector<Output> outputs_;
    std::vector<SignalId> ids_;
};

inline SignalSpec specOf(std::string name, SignalType type, Unit unit, std::string frame = {}) {
    SignalSpec s;
    s.name = std::move(name);
    s.type = type;
    s.unit = unit;
    s.frame = std::move(frame);
    return s;
}

/// A world with its scheduler: MuJoCo when asked, the writer, and the sensors (or their
/// ground-truth substitutes).
struct KindRun {
    std::unique_ptr<Scheduler> scheduler;

    [[nodiscard]] const SignalBus& bus() const { return scheduler->bus(); }
    [[nodiscard]] double read(const std::string& name) const {
        return bus().read(bus().find(name));
    }
    [[nodiscard]] std::vector<double> readN(const std::string& name) const {
        const std::span<const double> v = bus().readN(bus().find(name));
        return {v.begin(), v.end()};
    }
    [[nodiscard]] bool until(double seconds) const {
        return scheduler->stepUntil(SimTime{static_cast<std::int64_t>(std::llround(seconds * 1e9))})
            .ok();
    }
};

inline KindRun start(const World& world, bool physics, std::vector<Writer::Output> outputs,
                     bool groundTruth = false) {
    KindRun r;
    r.scheduler = std::make_unique<Scheduler>(SchedulerOptions{.deterministic = true});
    if (physics) {
        r.scheduler->add(std::make_unique<rigid_body::MujocoBackend>());
    }
    if (!outputs.empty()) {
        r.scheduler->add(std::make_unique<Writer>(std::move(outputs)));
    }
    for (auto& m : groundTruth ? createSensorSubstitutes(world) : createSensorModules(world)) {
        r.scheduler->add(std::move(m));
    }
    auto wired = r.scheduler->wire(WorldView(world));
    EXPECT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    if (!wired) {
        r.scheduler.reset();
    }
    return r;
}

}  // namespace lagrange::sensors::kindtest
