// Kind `cliff` (sensors.md §9): a robot vacuum's cliff sensor, an infrared range finder looking
// down. It measures the range to the floor along the sensor's -z (one ray, or a ring of rays in a
// cone with the nearest return winning, as an IR spot's strongest return does) and reports a
// cliff when the floor is farther than its threshold or not seen at all.
#include "KindScene.hpp"
#include "lagrange/random/Philox.hpp"
#include "lagrange/sensors/Sensors.hpp"

#include <cmath>
#include <numbers>

namespace lagrange::sensors::kinds {

namespace {

// Defaults, illustrative (a short-range IR triangulation sensor a few centimetres above the
// floor, no particular part: Sharp's GP2Y0A51SK0F, for one, measures 2 to 15 cm); every one is a
// parameter.
constexpr double kDefaultMinRange = 0.005;  // m
constexpr double kDefaultMaxRange = 0.15;   // m
constexpr double kDefaultThreshold = 0.06;  // m: farther than this is a drop
constexpr int kMaxRays = 64;

class Cliff final : public KindModule {
public:
    Cliff(const World& world, std::uint32_t sensor, bool groundTruth)
        : KindModule(world, sensor, groundTruth) {}

    Status declare(const WorldView& world, const ModuleConfig&, SignalDeclarer& d) override {
        const World& raw = world.raw();
        minRange_ = kindSetting(raw, sensor_, "min_range", kDefaultMinRange);
        maxRange_ = kindSetting(raw, sensor_, "max_range", kDefaultMaxRange);
        threshold_ = kindSetting(raw, sensor_, "threshold", kDefaultThreshold);
        noise_ = kindSetting(raw, sensor_, "range_noise", 0.0);
        const int rays = static_cast<int>(kindSetting(raw, sensor_, "rays", 1.0));
        const double cone = kindSetting(raw, sensor_, "beam_angle", 0.0);  // half angle, rad
        if (maxRange_ <= minRange_ || threshold_ <= 0.0 || rays < 1 || rays > kMaxRays ||
            cone < 0.0 || cone >= std::numbers::pi / 2.0) {
            return Status::error(StatusCode::InvalidConfig,
                                 "cliff " + name_ +
                                     " needs max_range above min_range, a threshold above 0, "
                                     "1 to 64 rays and a beam_angle under 90°");
        }
        // One ray straight down, the others on a ring at the cone's half angle.
        rays_.assign(1, {0.0, 0.0, -1.0});
        for (int k = 1; k < rays; ++k) {
            const double a = 2.0 * std::numbers::pi * (k - 1) / (rays - 1);
            rays_.push_back(
                {std::sin(cone) * std::cos(a), std::sin(cone) * std::sin(a), -std::cos(cone)});
        }
        const bool own = kindSetting(raw, sensor_, "ignore_own_body", 1.0) != 0.0;
        scene_.declare(raw, d, body_, own ? body_ : KindScene::kNone);
        range_ = d.output(kindSpec(base() + "/range", SignalType::Scalar, units::m));
        cliff_ = d.output(kindSpec(base() + "/cliff", SignalType::Bool, units::one));
        return {};
    }

    Status init(const WorldView& world, const ModuleConfig&, SignalBus& bus) override {
        scene_.init(world.raw());
        bus.write(range_, kNoHit);
        bus.write(cliff_, 0.0);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        scene_.place(bus);
        const Pose sensor = compose(scene_.poses().at(bus, body_), mount_);
        double nearest = kNoHit;
        for (const std::array<double, 3>& ray : rays_) {
            nearest =
                std::min(nearest, scene_.cast(sensor.position, rotate(sensor.orientation, ray)));
        }
        if (!groundTruth_ && noise_ > 0.0 && std::isfinite(nearest)) {
            random::Stream rng(context.seed, context.stepIndex, 0);
            nearest += noise_ * rng.normal();
        }
        // Beyond its range it sees nothing: +inf, as a lidar's missing return. Closer than its
        // minimum it reads the minimum: the floor is there, so no cliff.
        const double range = nearest > maxRange_ ? kNoHit : std::max(nearest, minRange_);
        bus.write(range_, range);
        bus.write(cliff_, !std::isfinite(range) || range > threshold_ ? 1.0 : 0.0);
        return {};
    }

private:
    KindScene scene_;
    std::vector<std::array<double, 3>> rays_;
    double minRange_{0.0}, maxRange_{0.0}, threshold_{0.0}, noise_{0.0};
    SignalId range_, cliff_;
};

}  // namespace

std::unique_ptr<IModule> makeCliff(const World& world, std::uint32_t sensor, bool groundTruth,
                                   const KindContext&) {
    return std::make_unique<Cliff>(world, sensor, groundTruth);
}

}  // namespace lagrange::sensors::kinds
