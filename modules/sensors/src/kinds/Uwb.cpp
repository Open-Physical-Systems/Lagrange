// Built-in kind `uwb` (sensors.md §9): ultra-wideband two-way ranging from a tag on the robot
// to fixed anchors, as a Qorvo (Decawave) DW1000 or DW3000 does it.
//
// Anchors, in the order of the outputs: the points of the text parameter `anchors`
// ("x y z; x y z", m, region-local), then every world prim or body whose name starts with
// `uwb_anchor`, in load order. Each range is the straight distance from the tag (the sensor's
// mount) to the anchor, plus, when the line of sight passes through colliders, the excess delay
// of the pulse through them: (√εr − 1) per metre of material, εr the relative permittivity. A
// range beyond max_range is a dropout (NaN).
#include "KindModule.hpp"
#include "ObstacleScene.hpp"
#include "lagrange/random/Philox.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace lagrange::sensors::kinds {

namespace {

/// Ranging precision with line of sight, 1σ, m: "a precision of 10 cm" (Decawave DW1000 Datasheet
/// Version 2.10, 2015, p. 1; DW3000 Datasheet Version 1.1, 2020: "an accuracy of 10 cm"), taken
/// as 1σ, on the safe side: the DW1000's figure 11 (line-of-sight two-way ranging) spreads about
/// ±8 cm.
constexpr double kDefaultRangeNoiseM = 0.1;
/// Relative permittivity of concrete, 1–100 GHz (ITU-R P.2040-1 Table 3: εr' = 5.24, frequency
/// independent): UWB channels 5 (6.5 GHz) and 9 (8 GHz) fall in that band.
constexpr double kConcretePermittivity = 5.24;
/// Extra spread without line of sight, 1σ, m: illustrative (multipath fills in the first path).
constexpr double kDefaultNlosNoiseM = 0.3;
/// Range limit, m: illustrative for indoor anchors at 6.8 Mb/s (the DW1000 datasheet's figure 31
/// shows some 50 to 110 m in open line of sight at 6.8 Mb/s, by channel and preamble).
constexpr double kDefaultMaxRangeM = 50.0;
/// Material shorter than this along the line does not count (an anchor mounted on a wall).
constexpr double kSurfaceToleranceM = 0.01;

class Uwb final : public KindModule {
public:
    using KindModule::KindModule;

    Status declare(const WorldView& view, const ModuleConfig&, SignalDeclarer& d) override {
        const World& world = view.raw();
        anchors_.clear();
        poses_.reset(world);
        poses_.watch(d, body_);
        noise_ = kindSetting(world, sensor_, "range_noise", kDefaultRangeNoiseM);
        nlosNoise_ = kindSetting(world, sensor_, "nlos_noise", kDefaultNlosNoiseM);
        permittivity_ = kindSetting(world, sensor_, "nlos_permittivity", kConcretePermittivity);
        maxRange_ = kindSetting(world, sensor_, "max_range", kDefaultMaxRangeM);
        if (noise_ < 0.0 || nlosNoise_ < 0.0 || permittivity_ < 1.0 || maxRange_ <= 0.0) {
            return Status::error(StatusCode::InvalidConfig,
                                 "uwb " + name_ +
                                     ": range_noise and nlos_noise must not be negative, "
                                     "nlos_permittivity at least 1, max_range positive");
        }
        std::vector<Vec3> points;
        if (!parsePoints(kindText(world, sensor_, "anchors"), points)) {
            return Status::error(StatusCode::InvalidConfig,
                                 "uwb " + name_ + ": anchors is \"x y z; x y z; ...\" in metres");
        }
        for (const Vec3& p : points) {
            anchors_.push_back({true, 0, p, true});
        }
        // Prims and bodies by name, in prim order; a body's own prim stands for the body.
        for (const NamedPlace& place : namedPlaces(world, "uwb_anchor")) {
            anchors_.push_back({place.fixed, place.body, place.local});
        }
        if (anchors_.empty()) {
            return Status::error(StatusCode::InvalidConfig,
                                 "uwb " + name_ +
                                     " has no anchors: give `anchors` or name world prims "
                                     "uwb_anchor_*");
        }
        std::vector<bool> skip(world.bodies.names.size(), false);
        if (body_ < skip.size()) {
            skip[body_] = true;
        }
        for (const Anchor& a : anchors_) {
            if (!a.fixed) {
                skip[a.body] = true;
                poses_.watch(d, a.body);
            }
        }
        obstacles_.declare(world, poses_, d, [&](std::uint32_t c) {
            const BodyId owner = world.colliders.body[c];
            return owner == kWorldBody || !skip[owner.index];
        });
        const auto n = static_cast<std::uint32_t>(anchors_.size());
        SignalSpec ranges = kindSpec(base() + "/ranges", SignalType::ScalarArray, units::m);
        ranges.arrayLength = n;
        rangesOut_ = d.output(ranges);
        SignalSpec los = kindSpec(base() + "/los", SignalType::ScalarArray, units::one);
        los.arrayLength = n;
        losOut_ = d.output(los);
        countOut_ = d.output(kindSpec(base() + "/count", SignalType::Int, units::one));
        validOut_ = d.output(kindSpec(base() + "/valid", SignalType::Bool, units::one));
        ranges_.assign(n, kNaN);
        los_.assign(n, kNaN);
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        std::fill(ranges_.begin(), ranges_.end(), kNaN);
        std::fill(los_.begin(), los_.end(), kNaN);
        publish(bus, 0);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        obstacles_.place(bus, poses_);
        const Vec3 tag = compose(poses_.pose(bus, body_), mount_).position;
        random::Stream rng(context.seed, context.stepIndex, 0);
        int count = 0;
        for (std::size_t i = 0; i < anchors_.size(); ++i) {
            const Anchor& a = anchors_[i];
            const Vec3 at =
                a.fixed ? a.local
                        : compose(poses_.pose(bus, a.body), Pose{a.local, {1, 0, 0, 0}}).position;
            const double dx = at[0] - tag[0], dy = at[1] - tag[1], dz = at[2] - tag[2];
            const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
            const ObstacleScene::Crossing through = obstacles_.crossing(tag, at);
            const bool lineOfSight = through.inside < kSurfaceToleranceM;
            const double noise1 = rng.normal();  // drawn for every anchor: streams stay aligned
            const double noise2 = rng.normal();
            if (distance > maxRange_) {
                ranges_[i] = kNaN;
                los_[i] = kNaN;
                continue;
            }
            double range = distance;
            if (!groundTruth_) {
                if (!lineOfSight) {
                    range += (std::sqrt(permittivity_) - 1.0) * through.inside;
                    range += nlosNoise_ * noise2;
                }
                range = std::max(0.0, range + noise_ * noise1);
            }
            ranges_[i] = range;
            los_[i] = lineOfSight ? 1.0 : 0.0;
            ++count;
        }
        publish(bus, count);
        return {};
    }

private:
    struct Anchor {
        bool fixed{true};  // `local` is region-local; else in body `body`
        std::uint32_t body{0};
        Vec3 local{};
        bool fromText{false};  // given by `anchors`, in the region frame the run started in
    };

public:
    /// A floating-origin rebase rebuilds the run from shifted world tables (named prims move
    /// with them) but reads `anchors` again as written: those move here (QA 2026-11-12 #17).
    void shiftOrigin(const std::array<double, 3>& shift) override {
        for (Anchor& a : anchors_) {
            if (a.fromText) {
                for (std::size_t i = 0; i < 3; ++i) {
                    a.local[i] -= shift[i];
                }
            }
        }
    }

private:
    void publish(SignalBus& bus, int count) {
        bus.writeN(rangesOut_, ranges_);
        bus.writeN(losOut_, los_);
        bus.write(countOut_, count);
        bus.write(validOut_, count > 0 ? 1.0 : 0.0);
    }

    double noise_{kDefaultRangeNoiseM}, nlosNoise_{kDefaultNlosNoiseM};
    double permittivity_{kConcretePermittivity}, maxRange_{kDefaultMaxRangeM};
    std::vector<Anchor> anchors_;
    std::vector<double> ranges_, los_;
    BodyPoses poses_;
    ObstacleScene obstacles_;
    SignalId rangesOut_, losOut_, countOut_, validOut_;
};

}  // namespace

std::unique_ptr<IModule> createUwb(const World& world, std::uint32_t sensor, bool groundTruth,
                                   const KindContext&) {
    return std::make_unique<Uwb>(world, sensor, groundTruth);
}

}  // namespace lagrange::sensors::kinds
