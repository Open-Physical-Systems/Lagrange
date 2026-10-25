#pragma once

#include "lagrange/module/IModule.hpp"
#include "lagrange/server/DiffDrive.hpp"

#include <functional>
#include <optional>
#include <string>

namespace lagrange::server {

/// Drives a road vehicle (VehicleAPI_1), the first of the world: writes vehicle/<v>/throttle,
/// brake and steering_angle; reads vehicle/<v>/speed and the chassis's pose.
///
/// The command is `external` (the ROS 2 bridge's /cmd_vel: speed and turn rate), turned into a
/// speed PI on throttle, the brake when the car is faster than asked or is to stand still, and
/// the bicycle model's steering angle atan(L·ω/v). Until the first external command, with
/// `demo`, it drives a 5 m circle round the world's origin at 3 m/s (pure pursuit, so it stays
/// on it), which fits the sample city block's junction; after it, a missing command stops it.
class CarController final : public IModule {
public:
    CarController(std::function<std::optional<VelocityCommand>()> external, bool demo)
        : external_(std::move(external)), demo_(demo) {}

    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

    /// Where the command comes from: "demo", "ros" or "stopped".
    [[nodiscard]] const char* source() const { return source_; }

private:
    std::function<std::optional<VelocityCommand>()> external_;
    bool demo_;
    bool externalSeen_{false};
    const char* source_{"demo"};
    double wheelbase_{0.0};
    double maxSteer_{0.0};
    double integral_{0.0};
    SignalId speed_, pose_, throttle_, brake_, steering_;
};

}  // namespace lagrange::server
