// Kind "visibility_sensor" (sensors.md §9): a forward-scatter visibility sensor (Vaisala PWD22)
// reporting the meteorological optical range from the air's extinction: the weather's, plus the
// fires' smoke where the sensor is (RFC 0022 §4, §6), with the visibility of a light-reflecting
// and a light-emitting sign through that smoke, S = C/σ.
#include "EnvCommon.hpp"
#include "lagrange/fields/Field.hpp"
#include "lagrange/sensors/EnvironmentSensors.hpp"
#include "lagrange/sensors/WeatherEffects.hpp"

#include <algorithm>
#include <cmath>

namespace lagrange::sensors::kinds {

namespace {

// Vaisala PWD22 (PWD series datasheet B210385EN-F): MOR 10 to 20 000 m, accuracy ±10 % from 10 to
// 10 000 m and ±15 % from 10 to 20 km. User's Guide M210543EN-B (2004), p. 25: instantaneous
// (15 s) values averaged from the extinction coefficient into 1 min and 10 min outputs; table 24:
// "Time constant 60 seconds", update interval 15 s. Here ±10 % is 2σ (±15 % above 10 km), and the
// averaging a first-order lag of 60 s on the extinction (the 1 min output is a running mean).
constexpr double kMinRange = 10.0;       // m
constexpr double kMaxRange = 20000.0;    // m
constexpr double kNoiseFraction = 0.05;  // of the MOR, 1σ
constexpr double kFarRatio = 1.5;        // ±15 % over ±10 % beyond kFar
constexpr double kFar = 10000.0;         // m
constexpr double kTau = 60.0;            // s

class VisibilitySensor final : public KindModule {
public:
    VisibilitySensor(const World& world, std::uint32_t sensor, bool groundTruth)
        : KindModule(world, sensor, groundTruth) {}

    Status declare(const WorldView& view, const ModuleConfig& config, SignalDeclarer& d) override {
        const World& w = view.raw();
        minRange_ = kindSetting(w, sensor_, "range/min", kMinRange);
        maxRange_ = kindSetting(w, sensor_, "range/max", kMaxRange);
        noise_ = kindSetting(w, sensor_, "noise/fraction", kNoiseFraction);
        farNoise_ = kindSetting(w, sensor_, "noise/fraction_far", kFarRatio * noise_);
        tau_ = kindSetting(w, sensor_, "time_constant", kTau);
        if (minRange_ <= 0.0 || maxRange_ <= minRange_ || noise_ < 0.0 || farNoise_ < 0.0 ||
            tau_ < 0.0) {
            return Status::error(StatusCode::InvalidConfig,
                                 "visibility_sensor " + name_ + ": parameter out of range");
        }
        extinctionIn_ =
            d.input(optionalNaN("env/extinction/visible", SignalType::Scalar, units::per_m));
        mor_ = d.output(kindSpec(base() + "/visibility", SignalType::Scalar, units::m));
        extinction_ = d.output(kindSpec(base() + "/extinction", SignalType::Scalar, units::per_m));
        valid_ = d.output(kindSpec(base() + "/valid", SignalType::Bool, units::one));
        // The fires' smoke (config "smoke"): σ at the sensor, read at its own rate.
        smoke_ = config.get<bool>("smoke").value_or(false);
        if (smoke_) {
            SignalSpec field = kindSpec("env/smoke/extinction", SignalType::Field3D, units::per_m);
            field.optional = true;
            field.feedback = true;
            smokeIn_ = d.input(field);
            onWorld_ = body_ == kWorldBody.index;  // a fixed meter in the scenery
            if (!onWorld_) {
                motion_.declare(w, body_, d, false);
            }
            reflecting_ =
                d.output(kindSpec(base() + "/sign_visibility", SignalType::Scalar, units::m));
            emitting_ =
                d.output(kindSpec(base() + "/light_visibility", SignalType::Scalar, units::m));
        }
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        sigma_ = kNaN;
        write(bus, kNaN, kNaN, false);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        double sigma = bus.read(extinctionIn_);
        if (smoke_) {
            const std::shared_ptr<const fields::Field> field = smokeField_.read(bus, smokeIn_);
            const double local =
                field
                    ? std::max(field->sampleScalar(
                                   compose(onWorld_ ? Pose{} : motion_.pose(bus), mount_).position),
                               0.0)
                    : 0.0;
            sigma = (std::isfinite(sigma) ? sigma : 0.0) + local;
        }
        if (!std::isfinite(sigma)) {
            write(bus, kNaN, kNaN, false);  // the weather is off
            return {};
        }
        if (groundTruth_) {
            write(bus, meteorologicalOpticalRange(sigma), sigma, true);
            return {};
        }
        sigma_ = firstOrderStep(sigma_, std::max(sigma, 0.0), context.dt.seconds(), tau_);
        random::Stream rng(context.seed, context.stepIndex, 0);
        const double trueMor = meteorologicalOpticalRange(sigma_);
        const double fraction = trueMor > kFar ? farNoise_ : noise_;
        const double measured = trueMor * (1.0 + fraction * rng.normal());
        const double mor = std::clamp(measured, minRange_, maxRange_);
        // The signs' visibility from what it measured, before its MOR is held to its range.
        write(bus, mor, -std::log(kMorContrast) / mor, true,
              -std::log(kMorContrast) / std::max(measured, 0.0));
        return {};
    }

    Status saveState(StateWriter& w) const override {
        w.put(sigma_);
        return {};
    }
    Status restoreState(StateReader& r) override {
        r.get(sigma_);
        return r.failed() ? Status::error(StatusCode::Internal, "visibility_sensor: short state")
                          : Status{};
    }

private:
    /// `signSigma`: the σ the sign visibilities come from (NaN: `sigma`).
    void write(SignalBus& bus, double mor, double sigma, bool valid, double signSigma = kNaN) {
        bus.write(mor_, mor);
        bus.write(extinction_, sigma);
        bus.write(valid_, valid ? 1.0 : 0.0);
        if (smoke_) {  // S = C/σ of what it measured, up to its range (smoke is metres)
            const auto sign = [&](double factor) {
                return valid ? std::min(smokeVisibility(std::isnan(signSigma) ? sigma : signSigma,
                                                        factor),
                                        maxRange_)
                             : kNaN;
            };
            bus.write(reflecting_, sign(kVisibilityReflecting));
            bus.write(emitting_, sign(kVisibilityEmitting));
        }
    }

    double minRange_{kMinRange}, maxRange_{kMaxRange}, noise_{kNoiseFraction}, tau_{kTau};
    double farNoise_{kFarRatio * kNoiseFraction};
    double sigma_{kNaN};
    SignalId extinctionIn_, mor_, extinction_, valid_;
    bool smoke_{false};
    SignalId smokeIn_, reflecting_, emitting_;
    fields::FieldSignal smokeField_;
    BodyMotion motion_;
    bool onWorld_{false};
};

}  // namespace

std::unique_ptr<IModule> makeVisibilitySensor(const World& world, std::uint32_t sensor,
                                              bool groundTruth, const KindContext&) {
    return std::make_unique<VisibilitySensor>(world, sensor, groundTruth);
}

}  // namespace lagrange::sensors::kinds
