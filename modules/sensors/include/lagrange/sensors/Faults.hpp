#pragma once

#include "lagrange/module/Expected.hpp"
#include "lagrange/module/IModule.hpp"

#include <limits>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace lagrange::sensors {

/// What goes wrong with a sensor (sensors.md "Sensor faults (as built)").
enum class FaultMode : std::uint8_t {
    Dropout,     // samples are lost (no new value, or NaN) in the window, each with a probability
    Stuck,       // the value freezes at the last one delivered before the window
    Bias,        // a step offset of `value` from `at`
    Drift,       // an offset growing by `rate` per second from `at`
    Noise,       // extra Gaussian noise of `sigma` in the window
    Disconnect,  // no new value (or NaN) from `at` for good
};

/// One fault of one sensor's quantity, or of all its quantities (`quantity` "*").
struct SensorFault {
    std::string sensor;         // the sensor's name: its signals are sensor/<sensor>/<q>
    std::string quantity{"*"};  // "specific_force", "ranges", ... or "*": all of them
    FaultMode mode{FaultMode::Dropout};
    double at{0.0};  // s: the fault starts with the first sample stamped at or after
    double duration{std::numeric_limits<double>::infinity()};  // s ("for"); disconnect: forever
    double probability{1.0};  // dropout: the chance that a sample in the window is lost
    bool nan{false};          // dropout, disconnect: deliver NaN instead of nothing
    double value{0.0};        // bias, in the quantity's unit
    double rate{0.0};         // drift, in the quantity's unit per second
    double sigma{0.0};        // noise: standard deviation, in the quantity's unit
    int component{-1};        // bias, drift, noise, stuck: one component only (0, 1, ...)
};

[[nodiscard]] std::string_view to_string(FaultMode mode);

/// `NAME:QUANTITY=MODE[,key=value...]` (or `NAME=MODE...` for all quantities), as
/// `--sensor-fault` takes it. Keys: at, for, probability, nan, value, rate, sigma, component.
/// Checks what can be checked without the run: the keys each mode needs and their ranges.
[[nodiscard]] Expected<SensorFault, std::string> parseSensorFault(std::string_view text);
/// The same fault as text that parseSensorFault reads back.
[[nodiscard]] std::string to_string(const SensorFault& fault);
/// The keys and ranges of one fault, as parseSensorFault checks them; empty when fine. For faults
/// made otherwise (a profile's sensor_faults).
[[nodiscard]] std::string checkSensorFault(const SensorFault& fault);

/// The faults applied to what `module` writes: every `sensor/<n>/<q>` it outputs that a fault
/// names is changed before it commits, so ROS, plugins, the delay line, recordings, Python and
/// the viewer all get the faulted value. Each faulted sensor also gets `sensor/<n>/fault/active`
/// (Bool: a fault's window is open) and `sensor/<n>/fault/dropped` (Int: samples lost so far).
/// Random draws come from the module's own seed and step (random::Stream), so a run and a
/// restored checkpoint repeat. With no faults the module is returned as it is (no cost); a module
/// that writes none of the faulted sensors' signals passes through.
[[nodiscard]] std::unique_ptr<IModule> withFaults(std::unique_ptr<IModule> module,
                                                  std::vector<SensorFault> faults);

/// After wiring: each fault's sensor writes something and its quantity exists. Empty when fine,
/// else what is wrong ("--sensor-fault imu:speed=...: sensor imu has no quantity speed; ...").
[[nodiscard]] std::string checkFaultsWired(const SignalBus& bus,
                                           std::span<const SensorFault> faults);

}  // namespace lagrange::sensors
