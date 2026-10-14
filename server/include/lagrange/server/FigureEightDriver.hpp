#pragma once

#include "lagrange/module/IModule.hpp"

#include <string>

namespace lagrange::server {

/// A built-in demo driver for a differential-drive robot: it asks for a figure-eight (a left
/// circle, then a right one) and turns that into wheel torques with a PI loop on wheel speed.
///
/// It stands in for the robot's own stack until the ROS 2 bridge and ros2_control arrive
/// (roadmap 0.2-F). Like any module it talks only through the bus: wheel velocities in (one-step
/// feedback, since the physics reads the torques it writes), wheel torques out.
class FigureEightDriver final : public IModule {
public:
    struct Settings {
        std::string leftJoint;
        std::string rightJoint;
        double wheelRadiusM{0.10};
        double trackWidthM{0.46};      // distance between the wheel contact points
        double speedMps{0.6};          // forward speed once started
        double turnRadiusM{1.5};       // radius of each loop of the eight
        double rampSeconds{2.0};       // time to reach full speed from rest
        double gainNmPerRadPerS{2.0};  // proportional gain on wheel-speed error
        double integralGainNmPerRad{8.0};
        double torqueLimitNm{10.0};  // the wheel's effort limit
    };

    explicit FigureEightDriver(Settings settings);

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
    SignalId leftVelocity_, rightVelocity_, leftTorque_, rightTorque_;
    SignalId linearCommand_, angularCommand_;
    double leftIntegral_{0.0};
    double rightIntegral_{0.0};
};

}  // namespace lagrange::server
