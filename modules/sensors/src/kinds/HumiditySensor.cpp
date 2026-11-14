// Kind "humidity_sensor" (sensors.md §9): a relative humidity and temperature sensor (Sensirion
// SHT4x) reading the weather's air through its own response time.
#include "EnvCommon.hpp"
#include "lagrange/sensors/EnvironmentSensors.hpp"

#include <algorithm>
#include <cmath>

namespace lagrange::sensors::kinds {

namespace {

// Sensirion SHT40 (SHT4x datasheet Version 7.3, June 2026, tables 1 and 2): RH accuracy ±1.8 %RH
// typical (±3.5 max from 10 to 90 %RH, figure 2), temperature ±0.2 °C typical; repeatability at
// high precision 0.08 %RH and 0.04 °C, "three times the standard deviation (3σ)" (footnote);
// response time τ63 4 s for RH (at 25 °C, 1 m/s air; 6 s in Version 1, October 2020) and 2 s for
// temperature (on its own, a board slows it); resolution 0.01 %RH and 0.01 °C. Repeatability / 3
// is the noise (1σ); the accuracy is a fixed offset the user sets (`offset/humidity`,
// `offset/temperature`).
constexpr double kHumidityNoise = 0.0008 / 3.0;   // of 1 (0.08 %RH at 3σ), 1σ
constexpr double kTemperatureNoise = 0.04 / 3.0;  // K (0.04 °C at 3σ), 1σ
constexpr double kHumidityTau = 4.0;              // s
constexpr double kTemperatureTau = 2.0;           // s
constexpr double kHumidityResolution = 0.0001;    // of 1 (0.01 %RH)
constexpr double kTemperatureResolution = 0.01;   // K

class HumiditySensor final : public KindModule {
public:
    HumiditySensor(const World& world, std::uint32_t sensor, bool groundTruth)
        : KindModule(world, sensor, groundTruth) {}

    Status declare(const WorldView& view, const ModuleConfig& config, SignalDeclarer& d) override {
        const World& w = view.raw();
        humidityNoise_ = kindSetting(w, sensor_, "noise/humidity", kHumidityNoise);
        temperatureNoise_ = kindSetting(w, sensor_, "noise/temperature", kTemperatureNoise);
        humidityTau_ = kindSetting(w, sensor_, "time_constant/humidity", kHumidityTau);
        temperatureTau_ = kindSetting(w, sensor_, "time_constant/temperature", kTemperatureTau);
        humidityOffset_ = kindSetting(w, sensor_, "offset/humidity", 0.0);
        temperatureOffset_ = kindSetting(w, sensor_, "offset/temperature", 0.0);
        humidityResolution_ = kindSetting(w, sensor_, "resolution/humidity", kHumidityResolution);
        temperatureResolution_ =
            kindSetting(w, sensor_, "resolution/temperature", kTemperatureResolution);
        if (humidityNoise_ < 0.0 || temperatureNoise_ < 0.0 || humidityTau_ < 0.0 ||
            temperatureTau_ < 0.0) {
            return Status::error(StatusCode::InvalidConfig,
                                 "humidity_sensor " + name_ + ": negative noise or time constant");
        }
        siteTemperature_ = config.get<double>("ambient_temperature").value_or(kNaN);
        humidityIn_ = d.input(optionalNaN("env/relative_humidity", SignalType::Scalar, units::one));
        temperatureIn_ = d.input(optionalNaN("env/air_temperature", SignalType::Scalar, units::K));
        humidity_ =
            d.output(kindSpec(base() + "/relative_humidity", SignalType::Scalar, units::one));
        temperature_ = d.output(kindSpec(base() + "/temperature", SignalType::Scalar, units::K));
        valid_ = d.output(kindSpec(base() + "/valid", SignalType::Bool, units::one));
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        rh_ = t_ = kNaN;
        write(bus, kNaN, kNaN, false);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        const double humidity = bus.read(humidityIn_);
        double temperature = bus.read(temperatureIn_);
        if (!std::isfinite(temperature)) {
            temperature = siteTemperature_;
        }
        // Without the weather there is no humidity (NaN, valid 0); the temperature may still be
        // the site's ambient one.
        const bool valid = std::isfinite(humidity);
        if (groundTruth_) {
            write(bus, humidity, temperature, valid);
            return {};
        }
        const double dt = context.dt.seconds();
        rh_ = valid ? firstOrderStep(rh_, humidity, dt, humidityTau_) : kNaN;
        t_ = std::isfinite(temperature) ? firstOrderStep(t_, temperature, dt, temperatureTau_)
                                        : kNaN;
        random::Stream rng(context.seed, context.stepIndex, 0);
        const double rhNoise = humidityNoise_ * rng.normal();
        const double tNoise = temperatureNoise_ * rng.normal();
        const double rh =
            valid ? std::clamp(quantize(rh_ + humidityOffset_ + rhNoise, humidityResolution_), 0.0,
                               1.0)
                  : kNaN;
        const double t = quantize(t_ + temperatureOffset_ + tNoise, temperatureResolution_);
        write(bus, rh, t, valid);
        return {};
    }

    Status saveState(StateWriter& w) const override {
        w.put(rh_);
        w.put(t_);
        return {};
    }
    Status restoreState(StateReader& r) override {
        r.get(rh_);
        r.get(t_);
        return r.failed() ? Status::error(StatusCode::Internal, "humidity_sensor: short state")
                          : Status{};
    }

private:
    void write(SignalBus& bus, double rh, double t, bool valid) {
        bus.write(humidity_, rh);
        bus.write(temperature_, t);
        bus.write(valid_, valid ? 1.0 : 0.0);
    }

    double humidityNoise_{kHumidityNoise}, temperatureNoise_{kTemperatureNoise};
    double humidityTau_{kHumidityTau}, temperatureTau_{kTemperatureTau};
    double humidityOffset_{0.0}, temperatureOffset_{0.0};
    double humidityResolution_{kHumidityResolution}, temperatureResolution_{kTemperatureResolution};
    double siteTemperature_{kNaN};
    double rh_{kNaN}, t_{kNaN};
    SignalId humidityIn_, temperatureIn_, humidity_, temperature_, valid_;
};

}  // namespace

std::unique_ptr<IModule> makeHumiditySensor(const World& world, std::uint32_t sensor,
                                            bool groundTruth, const KindContext&) {
    return std::make_unique<HumiditySensor>(world, sensor, groundTruth);
}

}  // namespace lagrange::sensors::kinds
