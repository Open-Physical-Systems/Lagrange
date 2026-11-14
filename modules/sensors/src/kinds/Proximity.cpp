// Built-in kind `proximity` (sensors.md §9): a proximity skin of a collaborative robot, an array
// of short-range time-of-flight elements over a field of view: per element the distance to the
// nearest collider, and the nearest of all. (Not the `tof` and `ultrasonic` plugin kinds.)
#include "KindModule.hpp"
#include "ObstacleScene.hpp"
#include "lagrange/random/Philox.hpp"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace lagrange::sensors::kinds {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
/// Defaults of a short-range time-of-flight skin element after the ST VL53L0X (DocID029104 Rev 2,
/// April 2018): its field of view 25° (§5.1), its ranging standard deviation 4 % at 33 ms
/// (table 12, white target indoors) taken at the 0.3 m reach, 12 mm. The reach is illustrative.
constexpr double kDefaultMaxRangeM = 0.3;
constexpr double kDefaultNoiseM = 0.012;
constexpr double kDefaultFovRad = 0.44;  // 25°, the VL53L0X's field of view
constexpr int kMaxElements = 4096;

class Proximity final : public KindModule {
public:
    using KindModule::KindModule;

    Status declare(const WorldView& view, const ModuleConfig&, SignalDeclarer& d) override {
        const World& world = view.raw();
        const double rows = kindSetting(world, sensor_, "rows", 1.0);
        const double columns = kindSetting(world, sensor_, "columns", 1.0);
        const double fov = kindSetting(world, sensor_, "fov", kDefaultFovRad);
        const double verticalFov = kindSetting(world, sensor_, "vertical_fov", fov);
        minRange_ = kindSetting(world, sensor_, "min_range", 0.0);
        maxRange_ = kindSetting(world, sensor_, "max_range", kDefaultMaxRangeM);
        noise_ = kindSetting(world, sensor_, "noise", kDefaultNoiseM);
        if (rows < 1.0 || columns < 1.0 || rows * columns > kMaxElements || fov < 0.0 ||
            verticalFov < 0.0 || minRange_ < 0.0 || maxRange_ <= minRange_ || noise_ < 0.0) {
            return Status::error(StatusCode::InvalidConfig,
                                 "proximity " + name_ +
                                     ": rows and columns at least 1 (at most 4096 elements), "
                                     "fields of view and noise not negative, max_range above "
                                     "min_range");
        }
        const int nr = static_cast<int>(rows);
        const int nc = static_cast<int>(columns);
        directions_.clear();
        for (int r = 0; r < nr; ++r) {
            const double el = -verticalFov / 2.0 + (r + 0.5) * verticalFov / nr;
            for (int c = 0; c < nc; ++c) {
                const double az = -fov / 2.0 + (c + 0.5) * fov / nc;
                directions_.push_back(
                    {std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el)});
            }
        }
        poses_.reset(world);
        poses_.watch(d, body_);
        scene_.declare(world, poses_, d, [&](std::uint32_t c) {
            return world.colliders.body[c] == kWorldBody || world.colliders.body[c].index != body_;
        });
        SignalSpec out = kindSpec(base() + "/distances", SignalType::ScalarArray, units::m);
        out.arrayLength = static_cast<std::uint32_t>(directions_.size());
        distancesOut_ = d.output(out);
        minOut_ = d.output(kindSpec(base() + "/min_distance", SignalType::Scalar, units::m));
        detectedOut_ = d.output(kindSpec(base() + "/detected", SignalType::Bool, units::one));
        distances_.assign(directions_.size(), kInf);
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        std::fill(distances_.begin(), distances_.end(), kInf);
        bus.writeN(distancesOut_, distances_);
        bus.write(minOut_, kInf);
        bus.write(detectedOut_, 0.0);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        scene_.place(bus, poses_);
        const Pose at = compose(poses_.pose(bus, body_), mount_);
        random::Stream rng(context.seed, context.stepIndex, 0);
        double nearest = kInf;
        for (std::size_t i = 0; i < directions_.size(); ++i) {
            const Vec3 dir = rotate(at.orientation, directions_[i]);
            const services::RayHit hit = scene_.ray(at.position, dir, maxRange_);
            const double noise = rng.normal();  // drawn for every element: streams stay aligned
            double distance = hit.hit() ? hit.distance : kInf;
            if (std::isfinite(distance) && !groundTruth_) {
                distance += noise_ * noise;
            }
            if (!(distance >= minRange_ && distance <= maxRange_)) {
                distance = kInf;
            }
            distances_[i] = distance;
            nearest = std::min(nearest, distance);
        }
        bus.writeN(distancesOut_, distances_);
        bus.write(minOut_, nearest);
        bus.write(detectedOut_, std::isfinite(nearest) ? 1.0 : 0.0);
        return {};
    }

private:
    double minRange_{0.0}, maxRange_{kDefaultMaxRangeM}, noise_{kDefaultNoiseM};
    std::vector<Vec3> directions_;  // in the sensor frame, row-major
    std::vector<double> distances_;
    BodyPoses poses_;
    ObstacleScene scene_;
    SignalId distancesOut_, minOut_, detectedOut_;
};

}  // namespace

std::unique_ptr<IModule> createProximity(const World& world, std::uint32_t sensor, bool groundTruth,
                                         const KindContext&) {
    return std::make_unique<Proximity>(world, sensor, groundTruth);
}

}  // namespace lagrange::sensors::kinds
