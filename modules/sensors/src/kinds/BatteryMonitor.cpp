// Kind `battery_monitor` (sensors.md §9): a smart battery's fuel gauge (a TI BQ40Z50 class part)
// or a flight controller's power module, reading one battery of the electrical domain. Its
// registers follow the Smart Battery Data Specification Rev 1.1 (SBS-IF, December 11, 1998),
// §5.1: Temperature() in 0.1 K (§5.1.9), Voltage() in mV (§5.1.10), Current() in mA, signed
// (§5.1.11), RelativeStateOfCharge() in % (§5.1.14), RemainingCapacity() in mAh (§5.1.16, with
// CAPACITY_MODE 0; 10 mWh with 1); those are the default steps here, in SI units.
#include "Converter.hpp"
#include "KindModule.hpp"
#include "lagrange/services/Log.hpp"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace lagrange::sensors::kinds {
namespace {

// SBS 1.1 §5.1 register steps, in SI.
constexpr double kSbsVoltageStepV = 1e-3;       // Voltage(): mV
constexpr double kSbsCurrentStepA = 1e-3;       // Current(): mA
constexpr double kSbsTemperatureStepK = 0.1;    // Temperature(): 0.1 K
constexpr double kSbsSocStep = 0.01;            // RelativeStateOfCharge(): %
constexpr double kSbsCapacityStepC = 3.6;       // RemainingCapacity(): mAh = 3.6 C
constexpr double kCoulombsPerAmpHour = 3600.0;  // C per A·h

enum Quantity : std::uint32_t { kVoltage, kCurrent, kTemperature, kSoc, kCapacity, kCount };

class BatteryMonitor final : public KindModule {
public:
    BatteryMonitor(const World& world, std::uint32_t sensor, bool groundTruth, std::string battery,
                   double capacity, std::string problem)
        : KindModule(world, sensor, groundTruth)
        , battery_(std::move(battery))
        , capacity_(capacity)
        , problem_(std::move(problem)) {
        const auto make = [&](const char* prefix, double step, bool bipolar) {
            Converter c;
            c.lsb = step;
            return converterFrom(world, sensor, prefix, c, bipolar);
        };
        conv_[kVoltage] = make("voltage/", kSbsVoltageStepV, false);
        conv_[kCurrent] = make("current/", kSbsCurrentStepA, true);
        conv_[kTemperature] = make("temperature/", kSbsTemperatureStepK, false);
        conv_[kSoc] = make("soc/", kSbsSocStep, false);
        conv_[kSoc].low = std::max(conv_[kSoc].low, 0.0);
        conv_[kSoc].high = std::min(conv_[kSoc].high, 1.0);
        conv_[kCapacity] = make("remaining_capacity/", kSbsCapacityStepC, false);
    }

    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        if (!problem_.empty()) {
            return Status::error(StatusCode::InvalidConfig,
                                 "battery_monitor " + name_ + ": " + problem_);
        }
        const auto input = [&](const std::string& name, Unit unit) {
            SignalSpec s = kindSpec(name, SignalType::Scalar, unit);
            s.optional = true;  // the electrical or thermal domain may be off: reads NaN
            s.defaultValue = kNaN;
            return d.input(s);
        };
        const std::string b = "battery/" + battery_ + "/";
        in_[kVoltage] = input(b + "terminal_voltage", units::V);
        in_[kCurrent] = input(b + "current", units::A);
        in_[kTemperature] = input("thermal/" + battery_ + "/temperature", units::K);
        in_[kSoc] = input(b + "soc", units::one);
        const std::string o = base() + "/";
        out_[kVoltage] = d.output(kindSpec(o + "voltage", SignalType::Scalar, units::V));
        out_[kCurrent] = d.output(kindSpec(o + "current", SignalType::Scalar, units::A));
        out_[kTemperature] = d.output(kindSpec(o + "temperature", SignalType::Scalar, units::K));
        out_[kSoc] = d.output(kindSpec(o + "soc", SignalType::Scalar, units::one));
        out_[kCapacity] =
            d.output(kindSpec(o + "remaining_capacity", SignalType::Scalar, units::C));
        valid_ = d.output(kindSpec(o + "valid", SignalType::Scalar, units::one));
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        if (!bus.connected(in_[kVoltage])) {
            // A misspelt <battery> read NaN for ever without a word (QA 2026-11-12 #13).
            services::log(services::LogLevel::Warning, "sensors",
                          "battery_monitor " + name_ + ": no module writes battery/" + battery_ +
                              "/terminal_voltage (no battery '" + battery_ +
                              "' in the run, or the electrical domain is off): it reads NaN, "
                              "valid 0");
        }
        for (std::uint32_t q = 0; q < kCount; ++q) {
            bus.write(out_[q], kNaN);
        }
        bus.write(valid_, 0.0);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        std::array<double, kCount> truth{};
        for (std::uint32_t q = 0; q < kCapacity; ++q) {
            truth[q] = bus.read(in_[q]);
        }
        truth[kCapacity] = capacity_ > 0.0 ? truth[kSoc] * capacity_ : kNaN;
        for (std::uint32_t q = 0; q < kCount; ++q) {
            random::Stream rng(context.seed, context.stepIndex, q);
            bus.write(out_[q], conv_[q].measure(truth[q], rng, groundTruth_));
        }
        bus.write(valid_, std::isnan(truth[kVoltage]) ? 0.0 : 1.0);
        return {};
    }

private:
    std::string battery_;
    double capacity_;  // C, the pack's rated charge; NaN: unknown
    std::string problem_;
    std::array<Converter, kCount> conv_{};
    std::array<SignalId, kCount> in_{};
    std::array<SignalId, kCount> out_{};
    SignalId valid_;
};

}  // namespace

std::unique_ptr<IModule> createBatteryMonitor(const World& world, std::uint32_t sensor,
                                              bool groundTruth, const KindContext&) {
    // The battery named by `battery`, else the only BatteryAPI_1 prim of the stage.
    const std::string wanted = ownPrimName(world, sensor, kindText(world, sensor, "battery"));
    std::vector<const SchemaInstance*> batteries;
    for (const SchemaInstance& s : world.schemaInstances) {
        if (s.schema == "BatteryAPI_1" &&
            (wanted.empty() || world.prims.instanceNames.at(s.prim.index) == wanted)) {
            batteries.push_back(&s);
        }
    }
    if (wanted.empty() && batteries.size() > 1) {
        // Several robots in the run, each with its pack: the one on the sensor's own robot.
        std::vector<const SchemaInstance*> own;
        for (const SchemaInstance* s : batteries) {
            if (onSensorsRobot(world, sensor, s->prim)) {
                own.push_back(s);
            }
        }
        if (own.size() == 1) {
            batteries = own;
        }
    }
    std::string battery = wanted;
    std::string problem;
    // A capacity in A·h may be given when the stage has no BatteryAPI_1 prim of that name.
    double capacity = kindSetting(world, sensor, "capacity", kNaN) * kCoulombsPerAmpHour;
    if (batteries.size() == 1) {
        battery = world.prims.instanceNames.at(batteries[0]->prim.index);
        capacity = batteries[0]->real("elec:battery:capacity").value_or(capacity);
    } else if (wanted.empty()) {
        problem = batteries.empty()
                      ? "the stage has no battery (BatteryAPI_1); name one with <battery>"
                      : "the stage has several batteries; name one with <battery>";
    }
    return std::make_unique<BatteryMonitor>(world, sensor, groundTruth, battery, capacity,
                                            std::move(problem));
}

}  // namespace lagrange::sensors::kinds
