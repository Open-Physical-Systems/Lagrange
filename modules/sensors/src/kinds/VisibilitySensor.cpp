// Kind "visibility_sensor" (sensors.md §9): a forward-scatter visibility sensor (Vaisala PWD22)
// reporting the meteorological optical range from the air's extinction.
#include "EnvCommon.hpp"
#include "lagrange/sensors/EnvironmentSensors.hpp"

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

    Status declare(const WorldView& view, const ModuleConfig&, SignalDeclarer& d) override {
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
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        sigma_ = kNaN;
        write(bus, kNaN, kNaN, false);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        const double sigma = bus.read(extinctionIn_);
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
        const double mor =
            std::clamp(trueMor * (1.0 + fraction * rng.normal()), minRange_, maxRange_);
        write(bus, mor, -std::log(kMorContrast) / mor, true);
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
    void write(SignalBus& bus, double mor, double sigma, bool valid) {
        bus.write(mor_, mor);
        bus.write(extinction_, sigma);
        bus.write(valid_, valid ? 1.0 : 0.0);
    }

    double minRange_{kMinRange}, maxRange_{kMaxRange}, noise_{kNoiseFraction}, tau_{kTau};
    double farNoise_{kFarRatio * kNoiseFraction};
    double sigma_{kNaN};
    SignalId extinctionIn_, mor_, extinction_, valid_;
};

}  // namespace

std::unique_ptr<IModule> makeVisibilitySensor(const World& world, std::uint32_t sensor,
                                              bool groundTruth, const KindContext&) {
    return std::make_unique<VisibilitySensor>(world, sensor, groundTruth);
}

}  // namespace lagrange::sensors::kinds
