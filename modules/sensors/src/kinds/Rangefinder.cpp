// Kind "rangefinder" (sensors.md §9): a single-beam laser rangefinder (LightWare SF11/C, Benewake
// TFmini-S): one ray along the sensor's +x against every collider.
#include "EnvCommon.hpp"
#include "lagrange/sensors/WeatherEffects.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lagrange::sensors::kinds {

namespace {

// Benewake TFmini-S (Datasheet SJ-PM-TFmini-S A01): 0.1 to 12 m on a 90 % white target,
// accuracy ±6 cm from 0.1 to 6 m and ±1 % from 6 to 12 m, resolution 1 cm, 100 Hz by default.
// The datasheet does not say what the ± is; taken as 2σ: σ = max(3 cm, 0.5 % of the range).
constexpr double kMinRange = 0.1;         // m
constexpr double kMaxRange = 12.0;        // m
constexpr double kNoise = 0.03;           // m, 1σ
constexpr double kNoiseFraction = 0.005;  // of the range, 1σ
constexpr double kResolution = 0.01;      // m

class Rangefinder final : public KindModule {
public:
    Rangefinder(const World& world, std::uint32_t sensor, bool groundTruth)
        : KindModule(world, sensor, groundTruth) {}

    Status declare(const WorldView& view, const ModuleConfig&, SignalDeclarer& d) override {
        const World& w = view.raw();
        minRange_ = kindSetting(w, sensor_, "range/min", kMinRange);
        maxRange_ = kindSetting(w, sensor_, "range/max", kMaxRange);
        noise_ = kindSetting(w, sensor_, "noise/stddev", kNoise);
        noiseFraction_ = kindSetting(w, sensor_, "noise/fraction", kNoiseFraction);
        resolution_ = kindSetting(w, sensor_, "resolution", kResolution);
        if (minRange_ < 0.0 || maxRange_ <= minRange_ || noise_ < 0.0 || noiseFraction_ < 0.0) {
            return Status::error(StatusCode::InvalidConfig,
                                 "rangefinder " + name_ + ": range or noise out of range");
        }
        motion_.declare(w, body_, d, false);
        scene_.declare(w, d);
        // The weather's near infrared extinction and the window's dirt (weather.md §2), when it
        // runs: they shorten the range the target is still seen at.
        extinction_ =
            d.input(optionalNaN("env/extinction/nir_905", SignalType::Scalar, units::per_m));
        contamination_ =
            d.input(optionalNaN(base() + "/contamination", SignalType::Scalar, units::one));
        range_ = d.output(kindSpec(base() + "/range", SignalType::Scalar, units::m));
        valid_ = d.output(kindSpec(base() + "/valid", SignalType::Bool, units::one));
        return {};
    }

    Status init(const WorldView& view, const ModuleConfig&, SignalBus& bus) override {
        scene_.init(view.raw());
        bus.write(range_, std::numeric_limits<double>::infinity());
        bus.write(valid_, 0.0);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        const Pose pose = compose(motion_.pose(bus), mount_);
        const Vec3 dir = rotate(pose.orientation, {1.0, 0.0, 0.0});
        scene_.update(bus);
        const double hit = scene_.cast(pose.position, dir, body_);
        double reach = maxRange_;
        if (!groundTruth_) {
            const double sigma = bus.read(extinction_);
            const double dirt = bus.read(contamination_);
            reach = lidarDetectionRange(maxRange_, std::isfinite(sigma) ? sigma : 0.0,
                                        std::isfinite(dirt) ? dirt : 0.0);
        }
        if (hit < minRange_) {
            // REP 117: −inf, something closer than it can measure (the ground under a landed
            // drone's altimeter), not +inf, nothing in range.
            bus.write(range_, -std::numeric_limits<double>::infinity());
            bus.write(valid_, 0.0);
            return {};
        }
        if (!(hit <= reach)) {
            bus.write(range_, std::numeric_limits<double>::infinity());  // REP 117: nothing
            bus.write(valid_, 0.0);
            return {};
        }
        double range = hit;
        if (!groundTruth_) {
            random::Stream rng(context.seed, context.stepIndex, 0);
            range += std::max(noise_, noiseFraction_ * hit) * rng.normal();
            range = std::clamp(quantize(range, resolution_), minRange_, maxRange_);
        }
        bus.write(range_, range);
        bus.write(valid_, 1.0);
        return {};
    }

private:
    double minRange_{kMinRange}, maxRange_{kMaxRange}, noise_{kNoise};
    double noiseFraction_{kNoiseFraction}, resolution_{kResolution};
    BodyMotion motion_;
    ColliderScene scene_;
    SignalId extinction_, contamination_, range_, valid_;
};

}  // namespace

std::unique_ptr<IModule> makeRangefinder(const World& world, std::uint32_t sensor, bool groundTruth,
                                         const KindContext&) {
    return std::make_unique<Rangefinder>(world, sensor, groundTruth);
}

}  // namespace lagrange::sensors::kinds
