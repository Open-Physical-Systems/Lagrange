#pragma once

#include "lagrange/random/Philox.hpp"

#include <array>
#include <functional>
#include <map>
#include <string>
#include <string_view>

namespace lagrange::sensors {

/// The error model of one IMU channel, the three axes of the gyroscope or the accelerometer
/// (docs/design/domains/sensors.md §2). Values in the channel's unit: rad/s for the gyroscope,
/// m/s² for the accelerometer. Every term is off at its default.
///
/// Settings (World::Sensors::settings), with `<p>` = "gyro" or "accel":
///   <p>_noise                         white noise, standard deviation per sample (SDFormat
///                                     <noise><stddev>)
///   <p>_noise_density                 white noise as a datasheet density N, unit/√Hz; when set,
///                                     the per-sample deviation is N·√rate
///   <p>_bias_mean, <p>_bias_stddev    turn-on bias: drawn once per run and axis from
///                                     Normal(mean, stddev) with a random sign (SDFormat
///                                     <bias_mean>, <bias_stddev>)
///   <p>_dynamic_bias_stddev           bias instability as a first-order Gauss-Markov process:
///   <p>_dynamic_bias_correlation_time the deviation of its driving noise, unit/√s, and its
///                                     correlation time τ, s (SDFormat <dynamic_bias_stddev>,
///                                     <dynamic_bias_correlation_time>, read as Gazebo does);
///                                     its stationary deviation is stddev·√(τ/2)
///   <p>_random_walk                   rate random walk K, unit/√s: the bias moves by K·√dt
///                                     per sample (the +1/2 slope of an Allan deviation plot)
///   <p>_range                         saturation at ±range
///   <p>_precision                     quantisation step (SDFormat <precision>)
///
/// Order per sample: true value + turn-on bias + Gauss-Markov bias + random-walk bias + white
/// noise, then saturation, then quantisation.
struct ImuNoise {
    double white{0.0};
    double biasMean{0.0}, biasStddev{0.0};
    double dynamicBiasStddev{0.0}, dynamicBiasCorrelationTime{0.0};
    double randomWalk{0.0};
    double range{0.0};
    double precision{0.0};

    using Settings = std::map<std::string, double, std::less<>>;
    /// Reads the settings of channel `prefix` for a sensor sampled at `rateHz`.
    [[nodiscard]] static ImuNoise from(const Settings& settings, std::string_view prefix,
                                       double rateHz);
    [[nodiscard]] bool hasBias() const;
};

/// The state of one channel: the bias of each axis. Pure function of the module seed and the
/// sample index, so it is deterministic, and it is saved with the module's state.
class ImuChannel {
public:
    ImuChannel() = default;
    ImuChannel(const ImuNoise& noise, double dtSeconds);

    /// Draws the turn-on bias. Called once, before the first sample.
    void start(random::Stream& rng);
    /// Moves the dynamic and random-walk biases by one sample.
    void advance(random::Stream& rng);
    /// The measured value of one axis: bias, `whiteNoise` (a standard normal draw), saturation,
    /// quantisation.
    [[nodiscard]] double measure(std::size_t axis, double truth, double whiteNoise) const;

    [[nodiscard]] const std::array<double, 3>& bias() const { return total_; }
    /// For saveState: turn-on, dynamic and random-walk bias of each axis.
    [[nodiscard]] std::array<double, 9> state() const;
    void setState(const std::array<double, 9>& state);

private:
    void sum();

    ImuNoise noise_;
    double phi_{0.0};         // Gauss-Markov decay per sample, exp(-dt / tau)
    double drive_{0.0};       // Gauss-Markov driving deviation per sample
    double stationary_{0.0};  // Gauss-Markov stationary deviation, stddev·sqrt(tau / 2)
    double walkStep_{0.0};    // random-walk deviation per sample, K·√dt
    std::array<double, 3> turnOn_{}, dynamic_{}, walk_{}, total_{};
};

}  // namespace lagrange::sensors
