// Kind "v2x" (sensors.md §9): a V2X receiver (DSRC IEEE 802.11p or C-V2X PC5) of the SAE J2735
// messages the traffic domain's world would send: each traffic agent's Basic Safety Message
// (from traffic/agents) and each signalised approach's Signal Phase and Timing (from
// traffic/lights, sent by a roadside unit at the signal head). Messages come at 10 Hz (SAE
// J2945/1) and are lost with a probability that grows with distance.
//
// Invariant 1: the agents and the lights are read only from the traffic domain's signals. Their
// layout is the traffic domain's (traffic.md §8); the array lengths are found from the world as
// the traffic domain finds them: one agent per kinematic body named traffic_vehicle_<i> or
// traffic_pedestrian_<i>, one light per lane with a signal head (its lamps, kinematic bodies
// traffic_lamp_<lane>_red), in lane order.
#include "EnvCommon.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <limits>
#include <vector>

namespace lagrange::sensors::kinds {

// Shared with the tests by name; in an inline namespace of their own (no clash with
// other kinds' helpers).
inline namespace extra2 {

/// The share of messages received at `distance` (m): a logistic fall-off, ½ at `d50`, over a
/// width `width` (m). Illustrative, not fitted: measured DSRC packet delivery ratios fall in this
/// way with distance (Meireles et al., "Experimental study on the impact of vehicular
/// obstructions in VANETs", IEEE VNC 2010: over 80 % at 400 m with line of sight in their suburb,
/// to zero at 500 m with vehicles in the way); the defaults sit near the obstructed case.
double v2xDelivery(double distance, double d50, double width) {
    return 1.0 / (1.0 + std::exp((distance - d50) / width));
}

}  // namespace extra2

namespace {

constexpr double kHalfDistance = 300.0;  // m where half the messages are lost, illustrative
constexpr double kWidth = 50.0;          // m, illustrative
constexpr double kMaxRange = 1000.0;     // m: nothing heard beyond
// A sender counts as heard while its last message is this recent: five missed 10 Hz messages.
constexpr double kTimeout = 0.5;         // s
constexpr std::size_t kAgentStride = 8;  // traffic/agents: class, x, y, z, yaw, speed, l, w
constexpr std::size_t kLightStride = 2;  // traffic/lights: road, state
constexpr std::size_t kBsmStride = 10;   // what this sensor writes per agent
constexpr std::size_t kSpatStride = 7;   // per light

class V2x final : public KindModule {
public:
    V2x(const World& world, std::uint32_t sensor, bool groundTruth)
        : KindModule(world, sensor, groundTruth) {}

    Status declare(const WorldView& view, const ModuleConfig&, SignalDeclarer& d) override {
        const World& w = view.raw();
        d50_ = kindSetting(w, sensor_, "loss_distance", kHalfDistance);
        width_ = kindSetting(w, sensor_, "loss_width", kWidth);
        maxRange_ = kindSetting(w, sensor_, "range/max", kMaxRange);
        timeout_ = kindSetting(w, sensor_, "timeout", kTimeout);
        if (d50_ <= 0.0 || width_ <= 0.0 || maxRange_ <= 0.0 || timeout_ < 0.0) {
            return Status::error(StatusCode::InvalidConfig,
                                 "v2x " + name_ + ": loss_distance, loss_width, range/max > 0");
        }
        agents_ = 0;
        std::vector<std::pair<long, Vec3>> heads;  // lane, signal head position
        for (std::uint32_t b = 0; b < w.bodies.names.size(); ++b) {
            const std::string& n = w.bodies.names[b];
            if (!w.bodies.kinematic[b]) {
                continue;
            }
            if (n.starts_with("traffic_vehicle_") || n.starts_with("traffic_pedestrian_")) {
                ++agents_;
            }
            constexpr std::string_view kLamp = "traffic_lamp_";
            if (n.starts_with(kLamp) && n.ends_with("_red")) {
                long lane = -1;
                const char* first = n.data() + kLamp.size();
                const char* last = n.data() + n.size() - 4;
                const auto [ptr, ec] = std::from_chars(first, last, lane);
                if (ec == std::errc{} && ptr == last) {
                    heads.emplace_back(lane, w.bodies.poseInParent[b].position);
                }
            }
        }
        std::ranges::sort(heads, {}, &std::pair<long, Vec3>::first);
        heads_.clear();
        for (const auto& h : heads) {
            heads_.push_back(h.second);
        }
        SignalSpec agents = optionalNaN("traffic/agents", SignalType::ScalarArray, units::one);
        agents.arrayLength =
            static_cast<std::uint32_t>(std::max<std::size_t>(agents_, 1) * kAgentStride);
        SignalSpec lights = optionalNaN("traffic/lights", SignalType::ScalarArray, units::one);
        lights.arrayLength =
            static_cast<std::uint32_t>(std::max<std::size_t>(heads_.size(), 1) * kLightStride);
        agentsIn_ = d.input(agents);
        lightsIn_ = d.input(lights);
        motion_.declare(w, body_, d, false);
        SignalSpec bsm = kindSpec(base() + "/bsm", SignalType::ScalarArray, units::one);
        bsm.arrayLength =
            static_cast<std::uint32_t>(std::max<std::size_t>(agents_, 1) * kBsmStride);
        SignalSpec spat = kindSpec(base() + "/spat", SignalType::ScalarArray, units::one);
        spat.arrayLength =
            static_cast<std::uint32_t>(std::max<std::size_t>(heads_.size(), 1) * kSpatStride);
        bsm_ = d.output(bsm);
        spat_ = d.output(spat);
        bsmCount_ = d.output(kindSpec(base() + "/bsm_count", SignalType::Int, units::one));
        spatCount_ = d.output(kindSpec(base() + "/spat_count", SignalType::Int, units::one));
        valid_ = d.output(kindSpec(base() + "/valid", SignalType::Bool, units::one));
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        bsmRow_.assign(std::max<std::size_t>(agents_, 1) * kBsmStride, kNaN);
        spatRow_.assign(std::max<std::size_t>(heads_.size(), 1) * kSpatStride, kNaN);
        bsmLast_.assign(agents_, kNaN);
        spatLast_.assign(heads_.size(), kNaN);
        for (std::size_t i = 0; i < agents_; ++i) {
            bsmRow_[i * kBsmStride] = static_cast<double>(i);
            bsmRow_[i * kBsmStride + 1] = 0.0;
            bsmRow_[i * kBsmStride + 9] = kNaN;
        }
        for (std::size_t i = 0; i < heads_.size(); ++i) {
            spatRow_[i * kSpatStride] = static_cast<double>(i);
            spatRow_[i * kSpatStride + 1] = 0.0;
        }
        write(bus, 0, 0, false);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        const double now = (context.t + context.dt).seconds();
        const Vec3 me = compose(motion_.pose(bus), mount_).position;
        const std::span<const double> agents = bus.readN(agentsIn_);
        const std::span<const double> lights = bus.readN(lightsIn_);
        const bool any = (!agents.empty() && std::isfinite(agents[0])) ||
                         (!lights.empty() && std::isfinite(lights[0]));
        int heard = 0;
        for (std::size_t i = 0; i < agents_; ++i) {
            const std::span<const double> a = agents.subspan(i * kAgentStride, kAgentStride);
            double* row = &bsmRow_[i * kBsmStride];
            const Vec3 at{a[1], a[2], a[3]};
            const bool near = inRange(at, me);
            if (near && receive(context, i, at, me)) {
                row[2] = a[0];  // class: 0 car, 1 pedestrian
                row[3] = a[1];
                row[4] = a[2];
                row[5] = a[4];  // yaw, rad from +x
                row[6] = a[5];  // speed, m/s
                row[7] = a[6];  // length, m
                row[8] = a[7];  // width, m
                bsmLast_[i] = now;
            }
            heard += mark(row, bsmLast_[i], now, near, kBsmStride);
        }
        int signals = 0;
        for (std::size_t i = 0; i < heads_.size(); ++i) {
            const std::span<const double> l = lights.subspan(i * kLightStride, kLightStride);
            double* row = &spatRow_[i * kSpatStride];
            const bool near = std::isfinite(l[0]) && inRange(heads_[i], me);
            if (near && receive(context, agents_ + i, heads_[i], me)) {
                row[2] = l[0];  // road id
                row[3] = l[1];  // 0 red, 1 yellow, 2 green
                row[4] = heads_[i][0];
                row[5] = heads_[i][1];
                spatLast_[i] = now;
            }
            signals += mark(row, spatLast_[i], now, near, kSpatStride);
        }
        write(bus, heard, signals, any);
        return {};
    }

    // What was last heard from each sender, and when: a restored run keeps its rows and ages.
    Status saveState(StateWriter& w) const override {
        w.putDoubles(bsmRow_);
        w.putDoubles(spatRow_);
        w.putDoubles(bsmLast_);
        w.putDoubles(spatLast_);
        return {};
    }
    Status restoreState(StateReader& r) override {
        r.getDoubles(bsmRow_);
        r.getDoubles(spatRow_);
        r.getDoubles(bsmLast_);
        r.getDoubles(spatLast_);
        return r.failed()
                   ? Status::error(StatusCode::InvalidConfig, "v2x " + name_ + ": short state")
                   : Status{};
    }
    // The rows hold region-local x, y (bsm columns 3, 4; spat columns 4, 5).
    void shiftOrigin(const std::array<double, 3>& shift) override {
        for (std::size_t i = 0; i < agents_; ++i) {
            bsmRow_[i * kBsmStride + 3] -= shift[0];
            bsmRow_[i * kBsmStride + 4] -= shift[1];
        }
        for (std::size_t i = 0; i < heads_.size(); ++i) {
            spatRow_[i * kSpatStride + 4] -= shift[0];
            spatRow_[i * kSpatStride + 5] -= shift[1];
        }
    }

private:
    [[nodiscard]] bool inRange(const Vec3& at, const Vec3& me) const {
        return finite(at) && norm(sub(at, me)) <= maxRange_;
    }

    /// Whether a message from `at` is received this step (ground truth: every one in range).
    [[nodiscard]] bool receive(const StepContext& context, std::size_t sender, const Vec3& at,
                               const Vec3& me) const {
        if (groundTruth_) {
            return true;
        }
        random::Stream rng(context.seed, context.stepIndex, static_cast<std::uint32_t>(sender));
        return rng.uniform() < v2xDelivery(norm(sub(at, me)), d50_, width_);
    }

    /// The row's validity (1: in range and heard within `timeout`) and its age (last number).
    [[nodiscard]] int mark(double* row, double last, double now, bool near,
                           std::size_t stride) const {
        row[stride - 1] = std::isnan(last) ? kNaN : now - last;
        const bool valid = near && !std::isnan(last) && now - last <= timeout_ + 1e-9;
        row[1] = valid ? 1.0 : 0.0;
        return valid ? 1 : 0;
    }

    void write(SignalBus& bus, int bsm, int spat, bool any) {
        bus.writeN(bsm_, bsmRow_);
        bus.writeN(spat_, spatRow_);
        bus.write(bsmCount_, bsm);
        bus.write(spatCount_, spat);
        bus.write(valid_, any ? 1.0 : 0.0);
    }

    double d50_{kHalfDistance}, width_{kWidth}, maxRange_{kMaxRange}, timeout_{kTimeout};
    std::size_t agents_{0};
    std::vector<Vec3> heads_;
    std::vector<double> bsmRow_, spatRow_, bsmLast_, spatLast_;
    BodyMotion motion_;
    SignalId agentsIn_, lightsIn_, bsm_, spat_, bsmCount_, spatCount_, valid_;
};

}  // namespace

std::unique_ptr<IModule> makeV2x(const World& world, std::uint32_t sensor, bool groundTruth,
                                 const KindContext&) {
    return std::make_unique<V2x>(world, sensor, groundTruth);
}

}  // namespace lagrange::sensors::kinds
