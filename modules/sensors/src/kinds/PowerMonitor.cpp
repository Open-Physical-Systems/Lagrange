// Kinds `current_sensor` and `voltage_sensor` (sensors.md §9): one current or voltage signal of
// the run (any signal in A or V: a bus voltage, a motor's bus current, a branch current) as a
// monitor reads it.
//
// Models:
// - `ina226` (both; the default): TI INA226 shunt and bus monitor, datasheet SBOS547C (rev. Aug.
//   2026) §5.5 and Table 7-3/7-4/7-5: shunt voltage step 2.5 µV over ±81.92 mV, bus voltage step
//   1.25 mV over 0 to 40.96 V (inputs up to 36 V), conversion times 140 µs to 8.244 ms, averages
//   1 to 1024. Its integrating ADC averages the signal over the conversions: the reading is the
//   window's mean, new every conversion time × channels × averages.
// - `hall` (current): an Allegro ACS712 Hall sensor read by a microcontroller's ADC; its output is
//   VCC/2 + sensitivity × I (ACS712-DS Rev. 22, 2024, performance characteristics tables: 185 mV/A
//   for the ELCTR-05B, 100 mV/A for -20A, 66 mV/A for -30A; noise 21, 11, 7 mV peak to peak).
// - `divider` (voltage): a resistor divider into a microcontroller's ADC, as a flight controller's
//   power module reads its battery.
#include "Converter.hpp"
#include "KindModule.hpp"
#include "lagrange/services/Log.hpp"

#include <array>
#include <memory>
#include <string>

namespace lagrange::sensors::kinds {
namespace {

// INA226, SBOS547C §5.5 Electrical Characteristics.
constexpr double kInaShuntStepV = 2.5e-6;          // shunt voltage, 1 LSB
constexpr double kInaShuntRangeV = 81.92e-3;       // shunt voltage input range, ±
constexpr double kInaBusStepV = 1.25e-3;           // bus voltage, 1 LSB
constexpr double kInaBusFullScaleV = 40.96;        // bus ADC full scale (inputs up to 36 V)
constexpr double kInaDefaultConversionS = 1.1e-3;  // CT = 100, the power-on default
// The conversion times (Table 7-4/7-5) and averages (Table 7-3) the INA226 offers.
constexpr std::array<double, 8> kInaConversionS{140e-6, 204e-6,   332e-6,   588e-6,
                                                1.1e-3, 2.116e-3, 4.156e-3, 8.244e-3};
constexpr std::array<double, 8> kInaAverages{1, 4, 16, 64, 128, 256, 512, 1024};
// ACS712ELCTR-05B-T (Allegro datasheet, x05B performance characteristics, VCC = 5 V).
constexpr double kAcs712Sensitivity = 0.185;       // V/A
constexpr double kAcs712NoisePeakToPeakV = 0.021;  // V, CF = 47 nF, 2 kHz bandwidth
// Peak to peak taken as 6 σ of Gaussian noise (a convention, not the datasheet's).
constexpr double kPeakToPeakSigmas = 6.0;
constexpr double kDefaultSupplyV = 5.0;   // the ACS712's VCC
constexpr double kDefaultAdcBits = 12.0;  // a typical microcontroller ADC
constexpr double kDefaultAdcReferenceV = 3.3;
constexpr double kDefaultShuntOhm = 0.002;           // illustrative; give your board's
constexpr double kDefaultDividerRatio = 1.0 / 11.0;  // illustrative (10 kΩ over 1 kΩ)

enum class Model { Ina226, Hall, Divider };

/// The listed value nearest to `v`.
template <std::size_t N>
double nearest(const std::array<double, N>& list, double v) {
    double best = list[0];
    for (double x : list) {
        if (std::abs(x - v) < std::abs(best - v)) {
            best = x;
        }
    }
    return best;
}

constexpr double kMaxAdcBits = 32.0;  // bits: no converter resolves more

class PowerMonitor final : public KindModule {
public:
    PowerMonitor(const World& world, std::uint32_t sensor, bool groundTruth, bool current)
        : KindModule(world, sensor, groundTruth)
        , current_(current)
        , signal_(ownSignalName(world, sensor, kindText(world, sensor, "signal"))) {
        const std::string model = kindText(world, sensor, "model", "ina226");
        if (model == "ina226") {
            model_ = Model::Ina226;
        } else if (current && model == "hall") {
            model_ = Model::Hall;
        } else if (!current && model == "divider") {
            model_ = Model::Divider;
        } else {
            problem_ = "model '" + model + "' is not one of " +
                       (current ? "ina226, hall" : "ina226, divider");
        }
        if (signal_.empty()) {
            problem_ = "names no <signal> to measure";
        }
        Converter c;
        if (model_ == Model::Ina226) {
            // The reading is in the measured unit: the shunt's steps become current steps.
            const double conversion =
                nearest(kInaConversionS,
                        kindSetting(world, sensor, "conversion_time", kInaDefaultConversionS));
            const double averages =
                nearest(kInaAverages, kindSetting(world, sensor, "averages", 1.0));
            const double channels =
                std::clamp(std::round(kindSetting(world, sensor, "channels", 2.0)), 1.0, 2.0);
            // A new reading every conversion × channels × averages (2.2 ms by default), on the
            // kinds' grid (2 ms): the exact 2.2 ms beside a 1 Hz sensor repeats only after 11 s.
            step_ = kindStep(conversion * channels * averages * 1e9, false, true);
            c.samples = static_cast<std::uint32_t>(averages);
            if (current) {
                shunt_ = kindSetting(world, sensor, "shunt_resistance", kDefaultShuntOhm);
                scale_ = 1.0 / shunt_;  // V at the shunt → A
                c.lsb = kInaShuntStepV;
                c.low = -kInaShuntRangeV;
                c.high = kInaShuntRangeV;
            } else {
                c.lsb = kInaBusStepV;
                c.low = 0.0;
                c.high = kInaBusFullScaleV;
            }
            c.gainError = kindSetting(world, sensor, "gain_error", 0.0);
            c.offset = kindSetting(world, sensor, "offset", 0.0);  // V at the ADC's input
            c.noise = std::max(0.0, kindSetting(world, sensor, "noise", 0.0));
        } else {
            const double supply = kindSetting(world, sensor, "supply", kDefaultSupplyV);
            const double bits = kindSetting(world, sensor, "adc/bits", kDefaultAdcBits);
            const double reference =
                kindSetting(world, sensor, "adc/reference",
                            model_ == Model::Hall ? supply : kDefaultAdcReferenceV);
            c.lsb = reference / std::exp2(std::round(bits));
            c.low = 0.0;
            c.high = reference;
            c.samples = static_cast<std::uint32_t>(
                std::clamp(kindSetting(world, sensor, "averages", 1.0), 1.0, 4096.0));
            c.gainError = kindSetting(world, sensor, "gain_error", 0.0);
            c.offset = kindSetting(world, sensor, "offset", 0.0);  // V at the ADC's input
            if (model_ == Model::Hall) {
                zero_ = kindSetting(world, sensor, "zero_output", supply / 2.0);
                scale_ = kindSetting(world, sensor, "sensitivity", kAcs712Sensitivity);
                c.noise = kindSetting(world, sensor, "noise",
                                      kAcs712NoisePeakToPeakV / kPeakToPeakSigmas);
                scale_ = 1.0 / scale_;  // V → A
            } else {
                const double ratio = kindSetting(world, sensor, "ratio", kDefaultDividerRatio);
                scale_ = 1.0 / ratio;
                c.noise = std::max(0.0, kindSetting(world, sensor, "noise", 0.0));
            }
        }
        conv_ = c;
        // A shunt, sensitivity or divider ratio of 0 or below read NaN, ±inf or the wrong sign
        // with `valid` 1; an ADC reference of 0 or below gave a range whose top is under its
        // bottom (std::clamp's precondition broken).
        if (problem_.empty() && !(std::isfinite(scale_) && scale_ > 0.0)) {
            problem_ = "shunt_resistance, sensitivity and ratio must be above 0";
        } else if (problem_.empty() && !(conv_.high > conv_.low)) {
            problem_ = "adc/reference (or supply) must be above 0";
        } else if (problem_.empty() && model_ != Model::Ina226 &&
                   !(std::round(kindSetting(world, sensor, "adc/bits", kDefaultAdcBits)) >= 1.0 &&
                     kindSetting(world, sensor, "adc/bits", kDefaultAdcBits) <= kMaxAdcBits)) {
            problem_ = "adc/bits must be from 1 to 32";
        } else if (problem_.empty() && !(conv_.gainError > -1.0)) {
            problem_ = "gain_error must be above -1";
        }
    }

    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        if (!problem_.empty()) {
            return Status::error(StatusCode::InvalidConfig, kind_ + " " + name_ + ": " + problem_);
        }
        const Unit unit = current_ ? units::A : units::V;
        SignalSpec in = kindSpec(signal_, SignalType::Scalar, unit);
        in.optional = true;  // its domain may be off: the reading is NaN
        in.defaultValue = kNaN;
        // The INA226 integrates over its conversions; a microcontroller's ADC samples.
        in.hold = model_ == Model::Ina226 ? Hold::WindowAverage : Hold::ZeroOrder;
        in_ = d.input(in);
        out_ = d.output(
            kindSpec(base() + (current_ ? "/current" : "/voltage"), SignalType::Scalar, unit));
        valid_ = d.output(kindSpec(base() + "/valid", SignalType::Scalar, units::one));
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        if (!bus.connected(in_)) {
            // A misspelt <signal> read NaN for ever without a word (QA 2026-11-12 #13).
            services::log(services::LogLevel::Warning, "sensors",
                          kind_ + " " + name_ + ": no module writes " + signal_ +
                              " (a misspelt <signal>, or its domain is off): it reads NaN, "
                              "valid 0");
        }
        bus.write(out_, kNaN);
        bus.write(valid_, 0.0);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        const double truth = bus.read(in_);
        random::Stream rng(context.seed, context.stepIndex, 0);
        bus.write(out_, groundTruth_ ? truth : measure(truth, rng));
        bus.write(valid_, std::isnan(truth) ? 0.0 : 1.0);
        return {};
    }

    /// What the chain reads for `truth` (A or V), noise from `rng`.
    [[nodiscard]] double measure(double truth, random::Stream& rng) const {
        if (std::isnan(truth)) {
            return truth;
        }
        // At the converter's input: the shunt's voltage, the Hall output, the divided voltage.
        const double atAdc = zero_ + truth / scale_;
        return (conv_.measure(atAdc, rng, false) - zero_) * scale_;
    }

private:
    bool current_;
    std::string signal_;
    Model model_{Model::Ina226};
    std::string problem_;
    Converter conv_;
    double shunt_{0.0};
    double scale_{1.0};  // measured unit per volt at the converter
    double zero_{0.0};   // V at the converter for zero
    SignalId in_, out_, valid_;
};

}  // namespace

std::unique_ptr<IModule> createCurrentSensor(const World& world, std::uint32_t sensor,
                                             bool groundTruth, const KindContext&) {
    return std::make_unique<PowerMonitor>(world, sensor, groundTruth, true);
}

std::unique_ptr<IModule> createVoltageSensor(const World& world, std::uint32_t sensor,
                                             bool groundTruth, const KindContext&) {
    return std::make_unique<PowerMonitor>(world, sensor, groundTruth, false);
}

}  // namespace lagrange::sensors::kinds
