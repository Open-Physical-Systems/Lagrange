#include "lagrange/server/Pushes.hpp"

#include <algorithm>
#include <utility>

namespace lagrange::server {

void PushSchedule::replace(std::vector<Push> pushes) {
    const std::lock_guard<std::mutex> lock(mutex_);
    pushes_ = std::move(pushes);
    version_.fetch_add(1, std::memory_order_relaxed);
}

void PushSchedule::refresh(std::uint64_t& seen, std::vector<Push>& out) const {
    if (version_.load(std::memory_order_relaxed) == seen) {
        return;
    }
    const std::lock_guard<std::mutex> lock(mutex_);
    seen = version_.load(std::memory_order_relaxed);
    out = pushes_;
}

Pushes::Pushes(std::shared_ptr<PushSchedule> schedule, std::vector<std::string> bodies)
    : schedule_(std::move(schedule)), names_(std::move(bodies)) {}

ModuleInfo Pushes::info() const {
    return {"disturbance", "pushes", 1, 0};  // deterministic given its schedule
}

Duration Pushes::preferredStep() const {
    return Duration{1'000'000};  // 1 ms, the rigid-body step
}

Status Pushes::declare(const WorldView& world, const ModuleConfig& /*config*/,
                       SignalDeclarer& declarer) {
    bodies_.clear();
    const World::Bodies& bodies = world.bodies();
    for (const std::string& name : names_) {
        const auto it = std::find(bodies.names.begin(), bodies.names.end(), name);
        if (it == bodies.names.end()) {
            return Status::error(StatusCode::InvalidConfig, "pushes: no body " + name);
        }
        const auto b = static_cast<std::size_t>(it - bodies.names.begin());
        SignalSpec spec;
        spec.name =
            "body/" + std::string(world.signalInstanceName(bodies.prim[b])) + "/wrench/push";
        spec.type = SignalType::Wrench;
        spec.unit = units::N;
        spec.frame = "world_local";
        bodies_.push_back({name, declarer.output(std::move(spec))});
    }
    return {};
}

Status Pushes::init(const WorldView& /*world*/, const ModuleConfig& /*config*/, SignalBus& bus) {
    constexpr std::array<double, 6> kNone{};
    for (const Body& body : bodies_) {
        bus.writeN(body.wrench, kNone);
    }
    return {};
}

Status Pushes::step(const StepContext& context, SignalBus& bus) {
    schedule_->refresh(seen_, pushes_);
    const double from = context.t.seconds();
    const double dt = context.dt.seconds();
    const double to = from + dt;
    for (const Body& body : bodies_) {
        std::array<double, 6> mean{};
        for (const Push& push : pushes_) {
            if (push.body != body.name) {
                continue;
            }
            // The share of this step the push acts in.
            const double overlap =
                std::min(to, push.start + push.duration) - std::max(from, push.start);
            if (overlap <= 0.0) {
                continue;
            }
            for (std::size_t k = 0; k < mean.size(); ++k) {
                mean[k] += push.wrench[k] * overlap / dt;
            }
        }
        bus.writeN(body.wrench, mean);
    }
    return {};
}

Status Pushes::saveState(StateWriter& /*writer*/) const {
    return {};
}

Status Pushes::restoreState(StateReader& /*reader*/) {
    return {};
}

}  // namespace lagrange::server
