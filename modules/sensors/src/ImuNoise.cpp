#include "lagrange/sensors/ImuNoise.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace lagrange::sensors {

namespace {

double get(const ImuNoise::Settings& settings, std::string_view prefix, std::string_view key) {
    const auto it = settings.find(std::string(prefix) + "_" + std::string(key));
    return it == settings.end() ? 0.0 : it->second;
}

}  // namespace

ImuNoise ImuNoise::from(const Settings& settings, std::string_view prefix, double rateHz) {
    ImuNoise noise;
    noise.white = get(settings, prefix, "noise");
    if (const double density = get(settings, prefix, "noise_density"); density > 0.0) {
        noise.white = density * std::sqrt(rateHz);
    }
    noise.biasMean = get(settings, prefix, "bias_mean");
    noise.biasStddev = get(settings, prefix, "bias_stddev");
    noise.dynamicBiasStddev = get(settings, prefix, "dynamic_bias_stddev");
    noise.dynamicBiasCorrelationTime = get(settings, prefix, "dynamic_bias_correlation_time");
    noise.randomWalk = get(settings, prefix, "random_walk");
    noise.range = get(settings, prefix, "range");
    noise.precision = get(settings, prefix, "precision");
    return noise;
}

bool ImuNoise::hasBias() const {
    return biasMean != 0.0 || biasStddev > 0.0 ||
           (dynamicBiasStddev > 0.0 && dynamicBiasCorrelationTime > 0.0) || randomWalk > 0.0;
}

ImuChannel::ImuChannel(const ImuNoise& noise, double dtSeconds) : noise_(noise) {
    if (noise.dynamicBiasStddev > 0.0 && noise.dynamicBiasCorrelationTime > 0.0) {
        // db/dt = -b/tau + sigma_b w(t), sigma_b the driving noise as SDFormat and Gazebo read
        // <dynamic_bias_stddev> (gz-sensors GaussianNoiseModel.cc): stationary deviation
        // sigma_b sqrt(tau / 2), discretised exactly.
        const double tau = noise.dynamicBiasCorrelationTime;
        phi_ = std::exp(-dtSeconds / tau);
        stationary_ = noise.dynamicBiasStddev * std::sqrt(tau / 2.0);
        drive_ = stationary_ * std::sqrt(1.0 - phi_ * phi_);
    }
    walkStep_ = noise.randomWalk * std::sqrt(dtSeconds);
}

void ImuChannel::start(random::Stream& rng) {
    for (std::size_t i = 0; i < 3; ++i) {
        // Normal(mean, stddev) with a random sign, so a positive mean gives either sign.
        const double magnitude = noise_.biasMean + noise_.biasStddev * rng.normal();
        turnOn_.at(i) = rng.uniform() < 0.5 ? -magnitude : magnitude;
        // The Gauss-Markov bias starts in its stationary distribution.
        dynamic_.at(i) = drive_ > 0.0 ? stationary_ * rng.normal() : 0.0;
    }
    sum();
}

void ImuChannel::advance(random::Stream& rng) {
    for (std::size_t i = 0; i < 3; ++i) {
        if (drive_ > 0.0) {
            dynamic_.at(i) = phi_ * dynamic_.at(i) + drive_ * rng.normal();
        }
        if (walkStep_ > 0.0) {
            walk_.at(i) += walkStep_ * rng.normal();
        }
    }
    sum();
}

double ImuChannel::measure(std::size_t axis, double truth, double whiteNoise) const {
    double value = truth + total_.at(axis) + noise_.white * whiteNoise;
    if (noise_.range > 0.0) {
        value = std::clamp(value, -noise_.range, noise_.range);
    }
    if (noise_.precision > 0.0) {
        value = noise_.precision * std::round(value / noise_.precision);
    }
    return value;
}

std::array<double, 9> ImuChannel::state() const {
    return {turnOn_[0],  turnOn_[1], turnOn_[2], dynamic_[0], dynamic_[1],
            dynamic_[2], walk_[0],   walk_[1],   walk_[2]};
}

void ImuChannel::setState(const std::array<double, 9>& state) {
    for (std::size_t i = 0; i < 3; ++i) {
        turnOn_.at(i) = state.at(i);
        dynamic_.at(i) = state.at(3 + i);
        walk_.at(i) = state.at(6 + i);
    }
    sum();
}

void ImuChannel::sum() {
    for (std::size_t i = 0; i < 3; ++i) {
        total_.at(i) = turnOn_.at(i) + dynamic_.at(i) + walk_.at(i);
    }
}

}  // namespace lagrange::sensors
