// Kind "light_sensor" (sensors.md §9): an ambient light sensor (ROHM BH1750FVI) facing the sensor's
// +x, lit by the sun, the sky and the run's lamps (RFC 0021 §5).
#include "SunCommon.hpp"
#include "lagrange/lights/Lamp.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace lagrange::sensors::kinds {

namespace {

// ROHM BH1750FVI (datasheet 2011.11 Rev.D): 16-bit, 120 ms typical measurement in H-resolution
// mode; its accuracy, sensor out over actual lux, is 1.2 typical (0.96 to 1.44) at 1000 lx under a
// white LED (p. 1), and a reading is counts / 1.2 lx (p. 7), so one count is 1/1.2 lx at the
// default measurement time (the "1 lx" H-resolution of p. 1 is nominal) and the 65535 counts
// saturate at 54 612 lx; the 0.8 to 1.2 left part to part is the `gain` parameter, 1 by default.
// Its spectral response is "approximately human eye response" (p. 1, peak 560 nm): taken as
// photopic.
constexpr double kCountsPerLux = 1.2;                // the datasheet's conversion
constexpr double kResolution = 1.0 / kCountsPerLux;  // lx per count
constexpr double kMaxCounts = 65535.0;               // 16 bits

/// Illuminance has no unit in lagrange::units (cd·sr/m², sr dimensionless): a local one.
constexpr Unit kLux{{.length = -2, .luminous = 1}, 1.0, QuantityKind::Unspecified};

/// m: a lamp's light counts as blocked by a collider this much short of the lamp, so the lamp's
/// own lens, flush with a surface it is not modelled apart from, does not shade it.
constexpr double kOcclusionMargin = 1e-3;

/// A lamp of the run (LampAPI_1) as the sensor sees it: its model, its body's pose and its output.
struct SeenLamp {
    lights::LampSpec spec;
    BodyMotion motion;  // its body's; unused for a lamp on the world
    bool onWorld{false};
    SignalId intensity;  // lamp/<n>/intensity, cd; NaN (dark) without the lights domain
};

class LightSensor final : public KindModule {
public:
    LightSensor(const World& world, std::uint32_t sensor, bool groundTruth)
        : KindModule(world, sensor, groundTruth) {}

    Status declare(const WorldView& view, const ModuleConfig&, SignalDeclarer& d) override {
        const World& w = view.raw();
        resolution_ = kindSetting(w, sensor_, "resolution", kResolution);
        gain_ = kindSetting(w, sensor_, "gain", 1.0);
        noise_ = kindSetting(w, sensor_, "noise/fraction", 0.0);
        maximum_ = kindSetting(w, sensor_, "max_illuminance", kMaxCounts * resolution_);
        groundReflectance_ = kindSetting(w, sensor_, "ground_reflectance", 0.0);
        if (resolution_ < 0.0 || gain_ <= 0.0 || noise_ < 0.0 || groundReflectance_ < 0.0 ||
            groundReflectance_ > 1.0) {
            return Status::error(StatusCode::InvalidConfig,
                                 "light_sensor " + name_ + ": parameter out of range");
        }
        if (Status s = sun_.declare(w, sensor_, "light_sensor " + name_, d); !s.ok()) {
            return s;
        }
        motion_.declare(w, body_, d, false);
        scene_.declare(w, d);
        std::string error;
        std::vector<lights::LampSpec> specs = lights::lampsOf(view, error);
        if (!error.empty()) {
            return Status::error(StatusCode::InvalidConfig, "light_sensor " + name_ + ": " + error);
        }
        lamps_.clear();
        for (lights::LampSpec& spec : specs) {
            SeenLamp& lamp = lamps_.emplace_back();
            lamp.onWorld = spec.body == kWorldBody;
            if (!lamp.onWorld) {
                lamp.motion.declare(w, spec.body.index, d, false);
            }
            lamp.intensity = d.input(
                optionalNaN("lamp/" + spec.name + "/intensity", SignalType::Scalar, units::cd));
            lamp.spec = std::move(spec);
        }
        illuminance_ = d.output(kindSpec(base() + "/illuminance", SignalType::Scalar, kLux));
        valid_ = d.output(kindSpec(base() + "/valid", SignalType::Bool, units::one));
        return {};
    }

    Status init(const WorldView& view, const ModuleConfig&, SignalBus& bus) override {
        scene_.init(view.raw());
        bus.write(illuminance_, kNaN);
        bus.write(valid_, 0.0);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        const Pose pose = compose(motion_.pose(bus), mount_);
        const Vec3 normal = rotate(pose.orientation, {1.0, 0.0, 0.0});
        const Vec3 sun = sun_.direction(bus, context.t, pose.position);
        // Without a sun (no place and date, no weather) the lamps alone light a sensor that has
        // any; with neither it reads nothing.
        if (!finite(sun) && lamps_.empty()) {
            bus.write(illuminance_, kNaN);
            bus.write(valid_, 0.0);
            return {};
        }
        bool placed = false;
        const auto place = [&] {
            if (!placed) {
                scene_.update(bus);
                placed = true;
            }
        };
        double lux = 0.0;
        if (finite(sun)) {
            double transmission = 1.0;
            const Daylight light = sun_.daylight(bus, sun, transmission);
            bool blocked = false;
            if (light.directNormal > 0.0 && dot(sun, normal) > 0.0) {
                place();
                blocked = std::isfinite(scene_.cast(pose.position, sun, body_));
            }
            lux = surfaceIlluminance(light, sun, normal, blocked, transmission, groundReflectance_);
        }
        lux += lampIlluminance(context, bus, pose.position, normal, place);
        if (!groundTruth_) {
            random::Stream rng(context.seed, context.stepIndex, 0);
            lux = gain_ * lux * (1.0 + noise_ * rng.normal());
            lux = std::clamp(quantize(lux, resolution_), 0.0, maximum_);
        }
        bus.write(illuminance_, lux);
        bus.write(valid_, 1.0);
        return {};
    }

private:
    /// Σ E = I·f(θ)·cos(incidence)/d² over the lamps (RFC 0021 §5), each with one occlusion ray
    /// from the sensor to the lamp through every collider but the sensor's and the lamp's own.
    template <class Place>
    double lampIlluminance(const StepContext& context, const SignalBus& bus, const Vec3& at,
                           const Vec3& normal, Place& place) {
        double sum = 0.0;
        const double t = context.t.seconds();
        for (const SeenLamp& lamp : lamps_) {
            const double intensity = bus.read(lamp.intensity);
            if (!(intensity > 0.0)) {
                continue;
            }
            const Pose lampPose = lamp.onWorld
                                      ? lamp.spec.poseInBody
                                      : compose(lamp.motion.pose(bus), lamp.spec.poseInBody);
            const double e = lights::illuminance(lamp.spec, intensity, lampPose, at, normal, t);
            if (!(e > 0.0)) {
                continue;
            }
            const Vec3 to = sub(lampPose.position, at);
            const double d = norm(to);
            if (d > kOcclusionMargin) {
                place();
                const Vec3 dir{to[0] / d, to[1] / d, to[2] / d};
                const std::uint32_t own =
                    lamp.onWorld ? ColliderScene::kNoBody : lamp.spec.body.index;
                if (scene_.cast(at, dir, body_, own) < d - kOcclusionMargin) {
                    continue;
                }
            }
            sum += e;
        }
        return sum;
    }

    double resolution_{kResolution}, gain_{1.0}, noise_{0.0}, maximum_{kMaxCounts};
    double groundReflectance_{0.0};
    SunSource sun_;
    BodyMotion motion_;
    std::vector<SeenLamp> lamps_;
    ColliderScene scene_;
    SignalId illuminance_, valid_;
};

}  // namespace

std::unique_ptr<IModule> makeLightSensor(const World& world, std::uint32_t sensor, bool groundTruth,
                                         const KindContext&) {
    return std::make_unique<LightSensor>(world, sensor, groundTruth);
}

}  // namespace lagrange::sensors::kinds
