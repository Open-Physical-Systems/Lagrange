#pragma once

// The module of docs/guide/writing-a-module.md: wheel odometry for a differential drive.

#include "lagrange/module/IModule.hpp"

#include <string>

namespace lagrange::tutorial {

/// Integrates the two wheel speeds of a differential drive into the distance the robot has
/// travelled and its heading change, as a wheel-encoder odometer would (no slip modelled).
///
/// Inputs:  joint/<left>/velocity, joint/<right>/velocity    rad/s
/// Outputs: base/odometer/distance                           m
///          base/odometer/heading                            rad
class Odometer final : public IModule {
public:
    struct Settings {
        std::string leftJoint{"left_wheel_joint"};
        std::string rightJoint{"right_wheel_joint"};
        double wheelRadiusM{0.10};
        double trackWidthM{0.46};  // between the wheel contact points
    };

    explicit Odometer(Settings settings);

    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

private:
    Settings settings_;
    SignalId left_, right_, distance_, heading_;
    double distanceM_{0.0};
    double headingRad_{0.0};
};

}  // namespace lagrange::tutorial
