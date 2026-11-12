// Kind `esc_telemetry` (sensors.md §9): what a drone's ESC reports of its motor, as BLHeli_32 and
// AM32 ESCs send it:
// - `kiss` (the default): KISS serial telemetry, a 10-byte frame at 115200 baud per request,
//   the flight controller asking one ESC after another on a shared wire (Betaflight
//   src/main/sensors/esc_sensor.c): temperature in °C, voltage in 0.01 V, current in 0.01 A,
//   consumption in mAh, speed in 100 eRPM, all unsigned.
// - `dshot`: bidirectional DShot, the eRPM in every reply as its period in µs with a 3-bit
//   exponent and 9-bit mantissa (period = m << e), and Extended DShot Telemetry frames
//   (github.com/bird-sanctuary/extended-dshot-telemetry, v2.1.1): temperature in 1 °C (0 to 255),
//   voltage in 0.25 V (0 to 63.75), current in 1 A (0 to 255), sent now and then (its example
//   schedule: each once a second).
// eRPM = mechanical RPM × pole pairs, and the motor turns `elec:gearRatio` times the joint.
#include "Converter.hpp"
#include "KindModule.hpp"

#include <array>
#include <memory>
#include <numbers>
#include <optional>
#include <string>

namespace lagrange::sensors::kinds {
namespace {

constexpr double kZeroCelsiusK = 273.15;
constexpr double kSecondsPerMinute = 60.0;
constexpr double kRadPerRevolution = 2.0 * std::numbers::pi;
// KISS telemetry (Betaflight esc_sensor.c).
constexpr double kKissBaud = 115200.0;          // bit/s
constexpr double kKissFrameBits = 10.0 * 10.0;  // 10 bytes, 8N1: 10 bit times each
constexpr double kKissVoltageStepV = 0.01;
constexpr double kKissCurrentStepA = 0.01;
constexpr double kKissErpmStep = 100.0;
constexpr double kKissCapacityStepC = 3.6;  // mAh
constexpr double kUint16Max = 65535.0;
constexpr double kByteMax = 255.0;
// Bidirectional DShot eRPM: period in µs = mantissa (9 bits) << exponent (3 bits).
constexpr std::int64_t kDshotMantissaMax = 511;
constexpr int kDshotExponentMax = 7;
// Extended DShot Telemetry v2.1.1 steps and ranges.
constexpr double kEdtVoltageStepV = 0.25;
constexpr double kEdtVoltageMaxV = 63.75;
constexpr double kEdtCurrentStepA = 1.0;
constexpr double kEdtDefaultRateHz = 1.0;  // its example schedule

enum Quantity : std::uint32_t { kCurrent, kVoltage, kTemperature, kCount };

/// The eRPM a bidirectional DShot reply carries for `erpm`: the period in whole µs, encoded with
/// a 9-bit mantissa and 3-bit exponent, decoded back; 0 below what the code can carry.
double dshotErpm(double erpm) {
    if (!(erpm > 0.0)) {
        return 0.0;
    }
    const auto period = static_cast<std::int64_t>(std::llround(kSecondsPerMinute * 1e6 / erpm));
    int e = 0;
    while ((period >> e) > kDshotMantissaMax && e < kDshotExponentMax) {
        ++e;
    }
    const std::int64_t m = period >> e;
    if (m > kDshotMantissaMax || m == 0) {
        return 0.0;  // slower than 511 << 7 µs per electrical turn: reported as stopped
    }
    return kSecondsPerMinute * 1e6 / static_cast<double>(m << e);
}

/// The eRPM a KISS frame carries: in 100 eRPM steps, 16 bits.
double kissErpm(double erpm) {
    return std::clamp(std::round(erpm / kKissErpmStep), 0.0, kUint16Max) * kKissErpmStep;
}

class EscTelemetry final : public KindModule {
public:
    struct Setup {
        std::string joint, motor, bus, thermal, problem;
        double polePairs{0.0};
        double gearRatio{1.0};
    };

    EscTelemetry(const World& world, std::uint32_t sensor, bool groundTruth, Setup setup)
        : KindModule(world, sensor, groundTruth), s_(std::move(setup)) {
        const std::string protocol = kindText(world, sensor, "protocol", "kiss");
        kiss_ = protocol == "kiss";
        if (!kiss_ && protocol != "dshot") {
            s_.problem = "protocol '" + protocol + "' is not kiss or dshot";
        }
        Converter current, voltage, temperature;
        temperature.lsb = 1.0;  // °C, both protocols
        temperature.low = 0.0;
        temperature.high = kByteMax;
        current.low = 0.0;
        voltage.low = 0.0;
        if (kiss_) {
            current.lsb = kKissCurrentStepA;
            current.high = kUint16Max * kKissCurrentStepA;
            voltage.lsb = kKissVoltageStepV;
            voltage.high = kUint16Max * kKissVoltageStepV;
            // One frame per request, the ESCs on the wire asked in turn.
            const double escs = std::max(1.0, kindSetting(world, sensor, "escs", 4.0));
            const double baud = kindSetting(world, sensor, "baud", kKissBaud);
            const double fastest = baud / (kKissFrameBits * escs);
            step_ = kindStep(1e9 / std::max(std::min(world.sensors.rateHz[sensor], fastest), 1e-3),
                             true);
        } else {
            current.lsb = kEdtCurrentStepA;
            current.high = kByteMax * kEdtCurrentStepA;
            voltage.lsb = kEdtVoltageStepV;
            voltage.high = kEdtVoltageMaxV;
            extendedPeriod_ =
                kindPeriod(kindSetting(world, sensor, "extended_rate", kEdtDefaultRateHz));
        }
        step_ = kindStep(static_cast<double>(step_.ns), true, true);
        conv_[kCurrent] = converterFrom(world, sensor, "current/", current, false);
        conv_[kVoltage] = converterFrom(world, sensor, "voltage/", voltage, false);
        conv_[kTemperature] = converterFrom(world, sensor, "temperature/", temperature, false);
    }

    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        if (!s_.problem.empty()) {
            return Status::error(StatusCode::InvalidConfig,
                                 "esc_telemetry " + name_ + ": " + s_.problem);
        }
        const auto optional = [&](const std::string& name, Unit unit, Hold hold) {
            SignalSpec spec = kindSpec(name, SignalType::Scalar, unit);
            spec.optional = true;  // its domain may be off: that reading is NaN
            spec.defaultValue = kNaN;
            spec.hold = hold;
            return d.input(spec);
        };
        velocity_ = optional("joint/" + s_.joint + "/velocity", units::rad_per_s, Hold::ZeroOrder);
        in_[kCurrent] = s_.motor.empty() ? SignalId{}
                                         : optional("motor/" + s_.motor + "/bus_current", units::A,
                                                    Hold::WindowAverage);
        in_[kVoltage] = s_.bus.empty()
                            ? SignalId{}
                            : optional("bus/" + s_.bus + "/voltage", units::V, Hold::ZeroOrder);
        in_[kTemperature] = s_.thermal.empty() ? SignalId{}
                                               : optional("thermal/" + s_.thermal + "/temperature",
                                                          units::K, Hold::ZeroOrder);
        const std::string o = base() + "/";
        speed_ = d.output(kindSpec(o + "speed", SignalType::Scalar, units::rad_per_s));
        out_[kCurrent] = d.output(kindSpec(o + "current", SignalType::Scalar, units::A));
        out_[kVoltage] = d.output(kindSpec(o + "voltage", SignalType::Scalar, units::V));
        out_[kTemperature] = d.output(kindSpec(o + "temperature", SignalType::Scalar, units::K));
        consumption_ = d.output(kindSpec(o + "consumption", SignalType::Scalar, units::C));
        valid_ = d.output(kindSpec(o + "valid", SignalType::Scalar, units::one));
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        charge_ = 0.0;
        nextExtended_ = 0;
        extended_.fill(kNaN);
        bus.write(speed_, kNaN);
        for (std::uint32_t q = 0; q < kCount; ++q) {
            bus.write(out_[q], kNaN);
        }
        bus.write(consumption_, kNaN);
        bus.write(valid_, 0.0);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        const double jointSpeed = bus.read(velocity_);
        std::array<double, kCount> truth{};
        for (std::uint32_t q = 0; q < kCount; ++q) {
            truth[q] = in_[q].index == SignalId::kInvalid ? kNaN : bus.read(in_[q]);
        }
        if (!std::isnan(truth[kCurrent])) {
            charge_ += truth[kCurrent] * context.dt.seconds();  // the window's mean × its length
        }
        // The motor's shaft, then electrical revolutions per minute.
        const double motorSpeed = std::abs(jointSpeed) * s_.gearRatio;
        const double erpm = motorSpeed * kSecondsPerMinute / kRadPerRevolution * s_.polePairs;
        double readErpm = erpm;
        if (!groundTruth_) {
            readErpm = kiss_ ? kissErpm(erpm) : dshotErpm(erpm);
        }
        bus.write(speed_, std::isnan(erpm)
                              ? kNaN
                              : readErpm / s_.polePairs * kRadPerRevolution / kSecondsPerMinute);
        // DShot sends its extended frames now and then; KISS sends everything every frame.
        const bool fresh = kiss_ || context.t.ns >= nextExtended_;
        if (fresh) {
            if (!kiss_) {
                nextExtended_ = context.t.ns + extendedPeriod_.ns;
            }
            truth[kTemperature] -= kZeroCelsiusK;  // the protocols count in °C
            for (std::uint32_t q = 0; q < kCount; ++q) {
                random::Stream rng(context.seed, context.stepIndex, q);
                extended_[q] = conv_[q].measure(truth[q], rng, groundTruth_);
            }
            extended_[kTemperature] += kZeroCelsiusK;
        }
        for (std::uint32_t q = 0; q < kCount; ++q) {
            bus.write(out_[q], extended_[q]);
        }
        double consumption = kNaN;
        if (kiss_ && !s_.motor.empty()) {
            consumption = groundTruth_ ? charge_
                                       : std::clamp(std::round(charge_ / kKissCapacityStepC), 0.0,
                                                    kUint16Max) *
                                                 kKissCapacityStepC +
                                             0.0;  // never -0
        }
        bus.write(consumption_, consumption);
        bus.write(valid_, std::isnan(jointSpeed) ? 0.0 : 1.0);
        return {};
    }

    Status saveState(StateWriter& w) const override {
        w.put(charge_);
        w.put(nextExtended_);
        w.putDoubles(extended_);
        return {};
    }
    Status restoreState(StateReader& r) override {
        r.get(charge_);
        r.get(nextExtended_);
        r.getDoubles(extended_);
        return {};
    }

private:
    Setup s_;
    bool kiss_{true};
    Duration extendedPeriod_{};
    std::array<Converter, kCount> conv_{};
    SignalId velocity_;
    std::array<SignalId, kCount> in_{};
    SignalId speed_, consumption_, valid_;
    std::array<SignalId, kCount> out_{};
    // Dynamic state.
    double charge_{0.0};  // C drawn since the start
    std::int64_t nextExtended_{0};
    std::array<double, kCount> extended_{};
};

/// The bus a prim feeds from: the prim itself, or where its ports connect, a few hops at most
/// (as the electrical network finds it, electrical.md §2).
std::string busOf(const World& world, PrimId prim) {
    constexpr int kMaxHops = 8;
    for (int hop = 0; hop < kMaxHops; ++hop) {
        std::optional<PrimId> next;
        for (const SchemaInstance& s : world.schemaInstances) {
            if (s.prim.index != prim.index) {
                continue;
            }
            if (s.schema == "ElectricalBusAPI_1") {
                return world.prims.instanceNames.at(prim.index);
            }
            if (s.schema == "PortAPI_1" && !next) {
                const auto to = s.rel("elec:port:" + s.instance + ":connectedTo");
                if (!to.empty()) {
                    next = to[0];
                }
            }
        }
        if (!next) {
            return {};
        }
        prim = *next;
    }
    return {};
}

}  // namespace

std::unique_ptr<IModule> createEscTelemetry(const World& world, std::uint32_t sensor,
                                            bool groundTruth, const KindContext&) {
    EscTelemetry::Setup s;
    s.joint = ownPrimName(world, sensor, kindText(world, sensor, "joint"));
    if (s.joint.empty()) {
        s.problem = "names no <joint>";
    }
    // The motor on that joint (ElectricalMotorAPI_1's elec:joint), if there is one.
    for (const SchemaInstance& m : world.schemaInstances) {
        const auto joint = m.rel("elec:joint");
        if (m.schema != "ElectricalMotorAPI_1" || joint.empty() ||
            world.prims.instanceNames.at(joint[0].index) != s.joint) {
            continue;
        }
        s.motor = world.prims.instanceNames.at(m.prim.index);
        s.polePairs = static_cast<double>(m.integer("elec:polePairs").value_or(1));
        s.gearRatio = m.real("elec:gearRatio").value_or(1.0);
        const auto port = m.rel("elec:powerPort");
        if (!port.empty()) {
            s.bus = busOf(world, port[0]);
        }
        break;
    }
    s.polePairs = kindSetting(world, sensor, "pole_pairs", s.polePairs);
    s.gearRatio = kindSetting(world, sensor, "gear_ratio", s.gearRatio);
    s.bus = ownPrimName(world, sensor, kindText(world, sensor, "bus", s.bus));
    s.thermal = ownPrimName(world, sensor, kindText(world, sensor, "temperature_node", s.motor));
    if (s.problem.empty() && !(s.gearRatio > 0.0)) {
        s.problem = "gear_ratio must be above 0 (it read a speed of 0)";
    }
    if (s.problem.empty() && !(kindSetting(world, sensor, "baud", 1.0) > 0.0)) {
        s.problem = "baud must be above 0";
    }
    if (s.problem.empty() && !(s.polePairs >= 1.0)) {
        s.problem =
            "joint '" + s.joint + "' has no motor (ElectricalMotorAPI_1); give <pole_pairs>";
    }
    return std::make_unique<EscTelemetry>(world, sensor, groundTruth, std::move(s));
}

}  // namespace lagrange::sensors::kinds
