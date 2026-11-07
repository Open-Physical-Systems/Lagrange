#pragma once

#include "lagrange/module/IModule.hpp"

#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace lagrange::sensors {

/// A sensor's outputs as the robot gets them: for every sensor with `latency` or `jitter` in its
/// settings, each `sensor/<n>/<q>` (numbers and buffers) is copied to `sensor/<n>/delayed/<q>`
/// that much later, with `sensor/<n>/delayed/stamp` (s) the time the value was sampled, -1 until
/// the first one arrives. The sensor's own signals stay on time: they are the ground truth.
///
/// The delay of a sample is a pure function of the module's seed, the sensor and the sampling
/// time, so all of a sensor's signals share it and a restored run draws the same ones. Add it
/// after every module whose sensors it delays (SignalDeclarer::declaredOutputs). sensors.md.
class DelayLine final : public IModule {
public:
    /// Whether any sensor of the world asks for a delay: without one the module is not added.
    [[nodiscard]] static bool needed(const WorldView& world);

    [[nodiscard]] ModuleInfo info() const override {
        return {.domain = "sensors", .name = "delay_line", .versionMajor = 1};
    }
    [[nodiscard]] Duration preferredStep() const override { return Duration{1'000'000}; }
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

private:
    struct Sample {
        SimTime stamp{};
        SimTime due{};
        std::vector<double> values;            // numbers
        std::shared_ptr<const Buffer> buffer;  // or a buffer
    };
    struct Line {
        SignalId in, out;
        bool handle{false};
        std::uint32_t sensor{0};  // index into sensors_
        SimTime last{};           // stamp of the last sample taken
        SimTime lastDue{};        // never out of order
        std::deque<Sample> waiting;
    };
    struct Sensor {
        double latency{0.0}, jitter{0.0};
        SignalId stamp;
        bool arrived{false};  // during step
    };
    std::vector<Sensor> sensors_;
    std::vector<Line> lines_;
};

}  // namespace lagrange::sensors
