// Kind `tension_sensor` (sensors.md §9): a strain-gauge load cell where a tether meets its body
// (`end` attach) or its station (`end` anchor), read by a bridge ADC. The cell gives
// sensitivity × excitation × F / capacity volts (a full bridge's rated output), off its line by a
// bow of `nonlinearity` × capacity at half load and apart by `hysteresis` × capacity between
// loading and unloading there. The ADC is an Avia HX711 by default (datasheet: 24-bit two's
// complement; channel A at gain 128 spans ±0.5·AVDD/128, ±20 mV at 5 V; input noise 50 nV rms at
// gain 128 and 10 SPS).
#include "Converter.hpp"
#include "KindModule.hpp"

#include <memory>
#include <string>
#include <vector>

namespace lagrange::sensors::kinds {
namespace {

// HX711 (Avia Semiconductor datasheet), channel A.
constexpr double kHx711Bits = 24.0;
constexpr double kHx711Gain = 128.0;
constexpr double kHx711NoiseV = 50e-9;         // rms, gain 128, RATE = 0 (10 SPS)
constexpr double kHx711HalfSpanPerVolt = 0.5;  // full scale = ±0.5 · AVDD / gain
// Illustrative cell: give your own from its data sheet.
constexpr double kDefaultCapacityN = 500.0;
constexpr double kDefaultSensitivity = 2e-3;  // V/V at capacity: 2 mV/V, a common rated output
constexpr double kDefaultExcitationV = 5.0;
constexpr double kDefaultRatedStrain = 1e-3;  // strain at the gauges at capacity
// The bow peaks at half load: 4·x·(1 − x) is 1 there.
constexpr double kBowPeak = 4.0;

class TensionSensor final : public KindModule {
public:
    TensionSensor(const World& world, std::uint32_t sensor, bool groundTruth, std::string tether,
                  std::string problem)
        : KindModule(world, sensor, groundTruth)
        , tether_(std::move(tether))
        , problem_(std::move(problem)) {
        const std::string end = kindText(world, sensor, "end", "attach");
        if (end != "attach" && end != "anchor") {
            problem_ = "end '" + end + "' is not attach or anchor";
        }
        anchor_ = end == "anchor";
        capacity_ = kindSetting(world, sensor, "capacity", kDefaultCapacityN);
        voltsPerNewton_ = kindSetting(world, sensor, "sensitivity", kDefaultSensitivity) *
                          kindSetting(world, sensor, "excitation", kDefaultExcitationV) / capacity_;
        nonlinearity_ = kindSetting(world, sensor, "nonlinearity", 0.0);
        hysteresis_ = kindSetting(world, sensor, "hysteresis", 0.0);
        ratedStrain_ = kindSetting(world, sensor, "rated_strain", kDefaultRatedStrain);
        cell_.gainError = kindSetting(world, sensor, "gain_error", 0.0);
        cell_.offset = kindSetting(world, sensor, "offset", 0.0);               // N, zero balance
        cell_.noise = std::max(0.0, kindSetting(world, sensor, "noise", 0.0));  // N
        const double gain = kindSetting(world, sensor, "adc/gain", kHx711Gain);
        const double fullScale = kHx711HalfSpanPerVolt *
                                 kindSetting(world, sensor, "excitation", kDefaultExcitationV) /
                                 gain;
        adc_.high = kindSetting(world, sensor, "adc/full_scale", fullScale);
        adc_.low = -adc_.high;
        adc_.lsb = 2.0 * adc_.high / std::exp2(kindSetting(world, sensor, "adc/bits", kHx711Bits));
        adc_.noise = kindSetting(world, sensor, "adc/noise", kHx711NoiseV);
        adc_.samples = static_cast<std::uint32_t>(
            std::clamp(kindSetting(world, sensor, "adc/samples", 1.0), 1.0, 4096.0));
        if (!(capacity_ > 0.0) || !(voltsPerNewton_ > 0.0)) {
            problem_ = "needs capacity, sensitivity and excitation above zero";
        }
    }

    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        if (!problem_.empty()) {
            return Status::error(StatusCode::InvalidConfig,
                                 "tension_sensor " + name_ + ": " + problem_);
        }
        SignalSpec in =
            kindSpec("tether/" + tether_ + (anchor_ ? "/tension_anchor" : "/tension_attach"),
                     SignalType::Scalar, units::N);
        in.optional = true;  // the tether domain may be off: the reading is NaN
        in.defaultValue = kNaN;
        in_ = d.input(in);
        force_ = d.output(kindSpec(base() + "/force", SignalType::Scalar, units::N));
        strain_ = d.output(kindSpec(base() + "/strain", SignalType::Scalar, units::one));
        valid_ = d.output(kindSpec(base() + "/valid", SignalType::Scalar, units::one));
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        previous_ = 0.0;
        rising_ = 1.0;
        bus.write(force_, kNaN);
        bus.write(strain_, kNaN);
        bus.write(valid_, 0.0);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        const double truth = bus.read(in_);
        double force = truth;
        if (!groundTruth_ && !std::isnan(truth)) {
            if (truth != previous_) {
                rising_ = truth > previous_ ? 1.0 : -1.0;
            }
            previous_ = truth;
            random::Stream rng(context.seed, context.stepIndex, 0);
            force =
                adc_.measure(cellForce(truth, rng) * voltsPerNewton_, rng, false) / voltsPerNewton_;
        }
        bus.write(force_, force);
        bus.write(strain_, force / capacity_ * ratedStrain_);
        bus.write(valid_, std::isnan(truth) ? 0.0 : 1.0);
        return {};
    }

    Status saveState(StateWriter& w) const override {
        w.put(previous_);
        w.put(rising_);
        return {};
    }
    Status restoreState(StateReader& r) override {
        r.get(previous_);
        r.get(rising_);
        return {};
    }

private:
    /// The force the cell's output stands for: gain, zero balance, bow, hysteresis, noise.
    [[nodiscard]] double cellForce(double f, random::Stream& rng) const {
        const double x = std::clamp(std::abs(f) / capacity_, 0.0, 1.0);
        const double bow = kBowPeak * x * (1.0 - x) * capacity_;
        const double sign = f < 0.0 ? -1.0 : 1.0;
        // Loading reads low, unloading high, by half the hysteresis each.
        const double hysteresis = -rising_ * sign * 0.5 * hysteresis_ * bow;
        return cell_.measure(f + sign * nonlinearity_ * bow + hysteresis, rng, false);
    }

    std::string tether_;
    std::string problem_;
    bool anchor_{false};
    double capacity_{kDefaultCapacityN};
    double voltsPerNewton_{0.0};
    double nonlinearity_{0.0}, hysteresis_{0.0}, ratedStrain_{kDefaultRatedStrain};
    Converter cell_;  // in N
    Converter adc_;   // in V at the bridge's output
    SignalId in_, force_, strain_, valid_;
    // Dynamic state.
    double previous_{0.0};
    double rising_{1.0};
};

}  // namespace

std::unique_ptr<IModule> createTensionSensor(const World& world, std::uint32_t sensor,
                                             bool groundTruth, const KindContext&) {
    // The tether named by `tether`, else the only TetherAPI_1 prim of the stage.
    std::string tether = ownPrimName(world, sensor, kindText(world, sensor, "tether"));
    std::string problem;
    if (tether.empty()) {
        std::vector<std::string> tethers;
        std::vector<std::string> own;  // on the sensor's robot, when the run has several
        for (const SchemaInstance& s : world.schemaInstances) {
            if (s.schema == "TetherAPI_1") {
                tethers.push_back(world.prims.instanceNames.at(s.prim.index));
                if (onSensorsRobot(world, sensor, s.prim)) {
                    own.push_back(tethers.back());
                }
            }
        }
        if (tethers.size() > 1 && own.size() == 1) {
            tethers = own;
        }
        if (tethers.size() == 1) {
            tether = tethers[0];
        } else {
            problem = tethers.empty()
                          ? "the stage has no tether (TetherAPI_1); name one with <tether>"
                          : "the stage has several tethers; name one with <tether>";
        }
    }
    return std::make_unique<TensionSensor>(world, sensor, groundTruth, tether, problem);
}

}  // namespace lagrange::sensors::kinds
