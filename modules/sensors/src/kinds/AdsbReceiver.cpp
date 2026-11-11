// Kind "adsb_receiver" (sensors.md §9): a 1090 MHz ADS-B receiver (1090ES, RTCA DO-260B) that
// hears the other aircraft of the run. An aircraft is another robot with RotorAPI_1 rotors (or,
// with `aircraft: all`, every other robot whose root body is not kinematic), read only through
// its root body's pose and twist on the bus (invariant 1). Each squitters its airborne position
// and velocity about twice a second (DO-260B: every 0.4 to 0.6 s, at random), and a squitter is
// heard while the aircraft is within range and above the radio horizon.
#include "EnvCommon.hpp"
#include "lagrange/world/coords/Frames.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace lagrange::sensors::kinds {

// Shared with the tests by name; in an inline namespace of their own (no clash with
// other kinds' helpers).
inline namespace extra2 {

/// The radio horizon (m) between antennas `h1` and `h2` metres up: √(2kRh1) + √(2kRh2) over
/// the 4/3 effective Earth radius of a standard atmosphere (Bean and Dutton, Radio Meteorology,
/// 1966, ch. 1).
double radioHorizon(double h1, double h2) {
    constexpr double kEarthRadius = 6371.0e3;  // m, mean (IUGG)
    constexpr double kEffective = 4.0 / 3.0;   // 1, standard refraction
    const double r = 2.0 * kEffective * kEarthRadius;
    return std::sqrt(r * std::max(h1, 0.0)) + std::sqrt(r * std::max(h2, 0.0));
}

}  // namespace extra2

namespace {

// DO-260B §2.2.3.3.1: airborne position and velocity squitters every 0.4 to 0.6 s, uniformly.
constexpr double kMinInterval = 0.4;  // s
constexpr double kMaxInterval = 0.6;  // s
// Typical ground receiver reach (illustrative; real ones hear 200 km and more at altitude).
constexpr double kMaxRange = 100e3;  // m
// What a message carries: altitude in 25 ft steps (the Q bit, DO-260B §2.2.3.2.3.4), east and
// north velocity in 1 kt steps and the vertical rate in 64 ft/min steps (subtype 1,
// §2.2.3.2.6.1).
constexpr double kAltitudeStep = 25.0 * 0.3048;         // m
constexpr double kVelocityStep = 1852.0 / 3600.0;       // m/s, one knot
constexpr double kVerticalStep = 64.0 * 0.3048 / 60.0;  // m/s
// The receiver's antenna above its mount, for the horizon (illustrative: a mast on a vehicle).
constexpr double kAntennaHeight = 2.0;  // m
constexpr double kPositionNoise = 3.0;  // m, 1σ of the sender's GNSS per axis, illustrative
constexpr std::size_t kStride = 10;     // numbers per aircraft in `aircraft`

class AdsbReceiver final : public KindModule {
public:
    AdsbReceiver(const World& world, std::uint32_t sensor, bool groundTruth)
        : KindModule(world, sensor, groundTruth) {}

    Status declare(const WorldView& view, const ModuleConfig&, SignalDeclarer& d) override {
        const World& w = view.raw();
        const std::string fail = "adsb_receiver " + name_ + ": ";
        maxRange_ = kindSetting(w, sensor_, "range/max", kMaxRange);
        noise_ = kindSetting(w, sensor_, "position_noise", kPositionNoise);
        horizon_ = kindSetting(w, sensor_, "horizon", 1.0) != 0.0;
        antenna_ = kindSetting(w, sensor_, "antenna_height", kAntennaHeight);
        const std::string which = kindText(w, sensor_, "aircraft", "rotors");
        if (maxRange_ <= 0.0 || noise_ < 0.0 || (which != "rotors" && which != "all")) {
            return Status::error(StatusCode::InvalidConfig,
                                 fail + "range/max > 0, position_noise >= 0, aircraft rotors|all");
        }
        if (w.geoOrigin) {
            origin_.emplace(*w.geoOrigin);
        }
        region_ = w.region.worldEnu;
        // The robots with rotors: a RotorAPI_1 prim on a joint (its child's robot) or a body.
        const std::size_t bodies = w.bodies.names.size();
        std::uint32_t robots = static_cast<std::uint32_t>(w.robots.size());
        for (const std::uint32_t r : w.bodies.robot) {
            robots = std::max(robots, r + 1);
        }
        std::vector<bool> rotors(robots, false);
        for (const SchemaInstance* inst : view.instancesOf("RotorAPI_1")) {
            std::optional<std::uint32_t> body;
            for (std::uint32_t j = 0; j < w.joints.prim.size(); ++j) {
                if (w.joints.prim[j] == inst->prim) {
                    body = w.joints.child[j].index;
                }
            }
            if (!body && w.prims.body.at(inst->prim.index) != kWorldBody) {
                body = w.prims.body[inst->prim.index].index;
            }
            if (body && *body < bodies) {
                rotors[w.bodies.robot[*body]] = true;
            }
        }
        const std::uint32_t own = w.bodies.robot[body_];
        const std::vector<bool> moving = movingBodies(w);
        targets_.clear();
        for (std::uint32_t r = 0; r < robots; ++r) {
            if (r == own || (which == "rotors" && !rotors[r])) {
                continue;
            }
            for (std::uint32_t b = 0; b < bodies; ++b) {
                if (w.bodies.robot[b] == r && w.bodies.parent[b] == kWorldBody && moving[b] &&
                    !w.bodies.kinematic[b]) {
                    Target t;
                    t.id = r;
                    const std::string base = "body/" + w.bodies.names[b];
                    t.pose = d.input(
                        kindSpec(base + "/pose", SignalType::Pose, units::m, "world_local"));
                    t.twist = d.input(kindSpec(base + "/twist", SignalType::Wrench, units::m_per_s,
                                               "world_local"));
                    targets_.push_back(t);
                    break;
                }
            }
        }
        motion_.declare(w, body_, d, false);
        SignalSpec table = kindSpec(base() + "/aircraft", SignalType::ScalarArray, units::one);
        table.arrayLength =
            static_cast<std::uint32_t>(std::max<std::size_t>(targets_.size(), 1) * kStride);
        table_ = d.output(table);
        count_ = d.output(kindSpec(base() + "/count", SignalType::Int, units::one));
        geodetic_ = d.output(kindSpec(base() + "/geodetic", SignalType::Bool, units::one));
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        row_.assign(std::max<std::size_t>(targets_.size(), 1) * kStride, kNaN);
        for (std::size_t i = 0; i < targets_.size(); ++i) {
            Target& t = targets_[i];
            t.sent = 0;
            t.heard = false;
            t.last = -std::numeric_limits<double>::infinity();
            // Each sender starts at its own moment within the first interval.
            random::Stream rng(0, 0, static_cast<std::uint32_t>(i));
            t.next = rng.uniform() * kMaxInterval;
            row_[i * kStride] = t.id;
            row_[i * kStride + 1] = 0.0;
        }
        bus.writeN(table_, row_);
        bus.write(count_, 0.0);
        bus.write(geodetic_, origin_ ? 1.0 : 0.0);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        const double now = (context.t + context.dt).seconds();
        const Vec3 me = add(compose(motion_.pose(bus), mount_).position, region_);
        int count = 0;
        for (std::size_t i = 0; i < targets_.size(); ++i) {
            Target& t = targets_[i];
            const std::span<const double> p = bus.readN(t.pose);
            const std::span<const double> v = bus.readN(t.twist);
            const Vec3 at = add({p[0], p[1], p[2]}, region_);
            const double distance = norm(sub(at, me));
            const bool reach = distance <= maxRange_ &&
                               (!horizon_ || distance <= radioHorizon(me[2] + antenna_, at[2]));
            double* row = &row_[i * kStride];
            if (!reach) {
                std::fill(row + 2, row + kStride, kNaN);
                t.heard = false;
                continue;
            }
            bool fresh = groundTruth_;
            while (!groundTruth_ && t.next <= now) {  // the squitters sent since the last step
                random::Stream rng(context.seed, t.sent, static_cast<std::uint32_t>(i) + 1000U);
                t.next += kMinInterval + (kMaxInterval - kMinInterval) * rng.uniform();
                ++t.sent;
                fresh = true;
            }
            if (fresh) {
                Vec3 pos = at;
                Vec3 vel{v[0], v[1], v[2]};
                if (!groundTruth_) {
                    random::Stream rng(context.seed, context.stepIndex,
                                       static_cast<std::uint32_t>(i) + 2000U);
                    for (double& c : pos) {
                        c += noise_ * rng.normal();
                    }
                    pos[2] = quantize(pos[2], kAltitudeStep);
                    vel = {quantize(vel[0], kVelocityStep), quantize(vel[1], kVelocityStep),
                           quantize(vel[2], kVerticalStep)};
                }
                if (origin_) {
                    const coords::Geodetic g = origin_->toGeodetic({pos[0], pos[1], pos[2]});
                    row[2] = g.latitudeRad;
                    row[3] = g.longitudeRad;
                    row[4] = groundTruth_ ? g.heightM : quantize(g.heightM, kAltitudeStep);
                } else {
                    row[2] = pos[0];
                    row[3] = pos[1];
                    row[4] = pos[2];
                }
                row[5] = vel[0];
                row[6] = vel[1];
                row[7] = vel[2];
                row[9] = distance;
                t.last = now;
                t.heard = true;
            }
            if (t.heard) {
                row[8] = now - t.last;
                ++count;
            }
            row[1] = t.heard ? 1.0 : 0.0;
        }
        bus.writeN(table_, row_);
        bus.write(count_, count);
        return {};
    }

    // Each sender's squitter clock and last message, and the table they fill: a restored run
    // hears the same messages at the same moments as the uninterrupted one.
    Status saveState(StateWriter& w) const override {
        w.put<std::uint64_t>(targets_.size());
        for (const Target& t : targets_) {
            w.put(t.next);
            w.put(t.last);
            w.put(t.sent);
            w.put(t.heard);
        }
        w.putDoubles(row_);
        return {};
    }
    Status restoreState(StateReader& r) override {
        std::uint64_t n = 0;
        if (!r.get(n) || n != targets_.size()) {
            return Status::error(StatusCode::InvalidConfig,
                                 "adsb_receiver " + name_ + ": state of another run");
        }
        for (Target& t : targets_) {
            r.get(t.next);
            r.get(t.last);
            r.get(t.sent);
            r.get(t.heard);
        }
        r.getDoubles(row_);
        return r.failed() ? Status::error(StatusCode::InvalidConfig,
                                          "adsb_receiver " + name_ + ": short state")
                          : Status{};
    }

private:
    static Vec3 add(const Vec3& a, const Vec3& b) {
        return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
    }

    struct Target {
        double id{0.0};
        SignalId pose, twist;
        double next{0.0}, last{0.0};
        std::uint64_t sent{0};
        bool heard{false};
    };
    double maxRange_{kMaxRange}, noise_{kPositionNoise};
    double antenna_{kAntennaHeight};
    bool horizon_{true};
    std::optional<coords::GeoOrigin> origin_;
    Vec3 region_{};
    std::vector<Target> targets_;
    std::vector<double> row_;
    BodyMotion motion_;
    SignalId table_, count_, geodetic_;
};

}  // namespace

std::unique_ptr<IModule> makeAdsbReceiver(const World& world, std::uint32_t sensor,
                                          bool groundTruth, const KindContext&) {
    return std::make_unique<AdsbReceiver>(world, sensor, groundTruth);
}

}  // namespace lagrange::sensors::kinds
