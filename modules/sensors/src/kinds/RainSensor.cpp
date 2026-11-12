// Kind "rain_sensor" (sensors.md §9): an optical rain gauge (Hydreon RG-15): rain rate, the rain
// fallen since the start, and whether it rains.
#include "EnvCommon.hpp"
#include "lagrange/sensors/EnvironmentSensors.hpp"

#include <algorithm>
#include <cmath>

namespace lagrange::sensors::kinds {

namespace {

// Hydreon RG-15 (Rain Gauge Model RG-15 Instructions, SW rev 1.000, 2022.07.18): "±10% accuracy
// under controlled conditions, accuracy may vary near/below freezing", resolution 0.2 mm (low) or
// 0.02 mm (high). ±10 % taken as 2σ of a gain error drawn once per run. Hydreon defines no
// "raining" rate: its rain event runs from the first drop to 60 min after the last.
constexpr double kResolution = 0.2e-3;          // m of water, low resolution
constexpr double kGainError = 0.05;             // 1σ, of the reading
constexpr double kThreshold = 0.1e-3 / 3600.0;  // m/s: 0.1 mm/h, illustrative "raining" threshold

class RainSensor final : public KindModule {
public:
    RainSensor(const World& world, std::uint32_t sensor, bool groundTruth)
        : KindModule(world, sensor, groundTruth) {}

    Status declare(const WorldView& view, const ModuleConfig&, SignalDeclarer& d) override {
        const World& w = view.raw();
        resolution_ = kindSetting(w, sensor_, "resolution", kResolution);
        gainError_ = kindSetting(w, sensor_, "noise/gain", kGainError);
        threshold_ = kindSetting(w, sensor_, "threshold", kThreshold);
        tau_ = kindSetting(w, sensor_, "time_constant", 0.0);
        snow_ = kindSetting(w, sensor_, "snow", 0.0) > 0.5;
        if (resolution_ < 0.0 || gainError_ < 0.0 || threshold_ < 0.0 || tau_ < 0.0) {
            return Status::error(StatusCode::InvalidConfig,
                                 "rain_sensor " + name_ + ": parameter out of range");
        }
        rainIn_ = d.input(optionalNaN("env/rain_rate", SignalType::Scalar, units::m_per_s));
        if (snow_) {
            snowIn_ = d.input(optionalNaN("env/snow_rate", SignalType::Scalar, units::m_per_s));
        }
        rate_ = d.output(kindSpec(base() + "/rain_rate", SignalType::Scalar, units::m_per_s));
        accumulation_ = d.output(kindSpec(base() + "/accumulation", SignalType::Scalar, units::m));
        raining_ = d.output(kindSpec(base() + "/raining", SignalType::Bool, units::one));
        valid_ = d.output(kindSpec(base() + "/valid", SignalType::Bool, units::one));
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        filtered_ = kNaN;
        total_ = 0.0;
        write(bus, kNaN, kNaN, false, false);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        double rate = bus.read(rainIn_);
        if (!std::isfinite(rate)) {
            write(bus, kNaN, kNaN, false, false);  // the weather is off
            return {};
        }
        if (snowIn_.valid() && std::isfinite(bus.read(snowIn_))) {
            rate += bus.read(snowIn_);  // melted water, as a heated gauge reads it
        }
        const double dt = context.dt.seconds();
        if (groundTruth_) {
            total_ += rate * dt;
            write(bus, rate, total_, rate > threshold_, true);
            return {};
        }
        // The gauge's gain error is its own, drawn once (step 0's stream), then a lag.
        random::Stream rng(context.seed, 0, 0);
        const double gain = std::max(0.0, 1.0 + gainError_ * rng.normal());
        filtered_ = firstOrderStep(filtered_, gain * rate, dt, tau_);
        total_ += filtered_ * dt;
        const double shown =
            resolution_ > 0.0 ? std::floor(total_ / resolution_) * resolution_ : total_;
        write(bus, filtered_, shown, filtered_ > threshold_, true);
        return {};
    }

    Status saveState(StateWriter& w) const override {
        w.put(filtered_);
        w.put(total_);
        return {};
    }
    Status restoreState(StateReader& r) override {
        r.get(filtered_);
        r.get(total_);
        return r.failed() ? Status::error(StatusCode::Internal, "rain_sensor: short state")
                          : Status{};
    }

private:
    void write(SignalBus& bus, double rate, double total, bool raining, bool valid) {
        bus.write(rate_, rate);
        bus.write(accumulation_, total);
        bus.write(raining_, raining ? 1.0 : 0.0);
        bus.write(valid_, valid ? 1.0 : 0.0);
    }

    double resolution_{kResolution}, gainError_{kGainError}, threshold_{kThreshold}, tau_{0.0};
    bool snow_{false};
    double filtered_{kNaN}, total_{0.0};
    SignalId rainIn_, snowIn_, rate_, accumulation_, raining_, valid_;
};

}  // namespace

std::unique_ptr<IModule> makeRainSensor(const World& world, std::uint32_t sensor, bool groundTruth,
                                        const KindContext&) {
    return std::make_unique<RainSensor>(world, sensor, groundTruth);
}

}  // namespace lagrange::sensors::kinds
