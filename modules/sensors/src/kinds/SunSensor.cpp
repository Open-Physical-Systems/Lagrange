// Kind "sun_sensor" (sensors.md §9): a two-axis sun sensor looking along the sensor's +x.
#include "SunCommon.hpp"

#include <cmath>
#include <numbers>

namespace lagrange::sensors::kinds {

namespace {

// Solar MEMS nanoSSOC-A60 (product sheet, solar-mems.com, 2024): field of view ±60°, accuracy
// < 0.5° (3σ), precision < 0.1°. The noise is the accuracy / 3, its bias included.
constexpr double kHalfFieldOfView = 60.0 * std::numbers::pi / 180.0;  // rad
constexpr double kNoise = 0.5 / 3.0 * std::numbers::pi / 180.0;       // rad, 1σ per axis
/// Illustrative: the least direct normal illuminance the sensor still finds the sun in (lx); an
/// overcast sky or thick fog hides it.
constexpr double kMinDirect = 1000.0;

class SunSensor final : public KindModule {
public:
    SunSensor(const World& world, std::uint32_t sensor, bool groundTruth)
        : KindModule(world, sensor, groundTruth) {}

    Status declare(const WorldView& view, const ModuleConfig&, SignalDeclarer& d) override {
        const World& w = view.raw();
        halfFov_ = kindSetting(w, sensor_, "fov", 2.0 * kHalfFieldOfView) / 2.0;
        noise_ = kindSetting(w, sensor_, "noise/stddev", kNoise);
        minDirect_ = kindSetting(w, sensor_, "min_illuminance", kMinDirect);
        if (halfFov_ <= 0.0 || halfFov_ > std::numbers::pi || noise_ < 0.0) {
            return Status::error(StatusCode::InvalidConfig,
                                 "sun_sensor " + name_ + ": fov or noise out of range");
        }
        if (Status s = sun_.declare(w, sensor_, "sun_sensor " + name_, d); !s.ok()) {
            return s;
        }
        motion_.declare(w, body_, d, false);
        scene_.declare(w, d);
        direction_ =
            d.output(kindSpec(base() + "/sun_direction", SignalType::Vec3, units::one, "sensor"));
        visible_ = d.output(kindSpec(base() + "/sun_visible", SignalType::Bool, units::one));
        valid_ = d.output(kindSpec(base() + "/valid", SignalType::Bool, units::one));
        return {};
    }

    Status init(const WorldView& view, const ModuleConfig&, SignalBus& bus) override {
        scene_.init(view.raw());
        write(bus, {kNaN, kNaN, kNaN}, false, false);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        const Pose pose = compose(motion_.pose(bus), mount_);
        const Vec3 sun = sun_.direction(bus, context.t, pose.position);
        if (!finite(sun)) {
            write(bus, {kNaN, kNaN, kNaN}, false, false);
            return {};
        }
        double transmission = 1.0;
        const Daylight light = sun_.daylight(bus, sun, transmission);
        Vec3 local = rotate(conjugate(pose.orientation), sun);
        bool visible =
            local[0] >= std::cos(halfFov_) && light.directNormal * transmission >= minDirect_;
        if (visible) {
            scene_.update(bus);
            visible = !std::isfinite(scene_.cast(pose.position, sun, body_));
        }
        if (!visible) {
            write(bus, {kNaN, kNaN, kNaN}, false, true);
            return {};
        }
        if (!groundTruth_) {
            // Angular noise on the two axes across the boresight.
            random::Stream rng(context.seed, context.stepIndex, 0);
            local[1] += noise_ * rng.normal() * local[0];
            local[2] += noise_ * rng.normal() * local[0];
            const double n = norm(local);
            local = {local[0] / n, local[1] / n, local[2] / n};
        }
        write(bus, local, true, true);
        return {};
    }

private:
    void write(SignalBus& bus, const Vec3& v, bool visible, bool valid) {
        bus.writeN(direction_, v);
        bus.write(visible_, visible ? 1.0 : 0.0);
        bus.write(valid_, valid ? 1.0 : 0.0);
    }

    double halfFov_{kHalfFieldOfView}, noise_{kNoise}, minDirect_{kMinDirect};
    SunSource sun_;
    BodyMotion motion_;
    ColliderScene scene_;
    SignalId direction_, visible_, valid_;
};

}  // namespace

std::unique_ptr<IModule> makeSunSensor(const World& world, std::uint32_t sensor, bool groundTruth,
                                       const KindContext&) {
    return std::make_unique<SunSensor>(world, sensor, groundTruth);
}

}  // namespace lagrange::sensors::kinds
