#pragma once

#include "lagrange/module/IModule.hpp"
#include "lagrange/server/DiffDrive.hpp"

#include <functional>
#include <optional>
#include <string>

namespace lagrange::server {

/// What drives a car: a speed (m/s, negative reverses) with a turn rate (rad/s, /cmd_vel), or
/// with the bicycle model's road-wheel angle (rad, positive left; /ackermann_cmd), which then
/// stands for the turn rate.
struct CarCommand {
    double speed{0.0};
    double turnRate{0.0};
    std::optional<double> steeringAngle;
};

/// Drives a road vehicle (VehicleAPI_1), the first of the world: writes vehicle/<v>/throttle,
/// brake and steering_angle; reads vehicle/<v>/speed and the chassis's pose.
///
/// The command is `external` (the ROS 2 bridge's /cmd_vel or /ackermann_cmd), turned into a
/// speed PI on throttle, the brake when the car is faster than asked or is to stand still, and
/// the steering angle: the command's own, or the bicycle model's atan(L·ω/v) for a turn rate.
/// Until the first external command, with `demo`, it drives a 5 m circle round the world's origin
/// at 3 m/s (pure pursuit, so it stays on it), which fits the sample city block's junction; after
/// it, a missing command stops it. When the demo is held where it is (a post at a corner, a
/// stopped car of the traffic: QA 2026-11-14, the sedan stood still in the town against a
/// crossing's post), it backs away with the steering the other way, as a driver turning in three
/// moves does, and goes on.
class CarController final : public IModule {
public:
    CarController(std::function<std::optional<CarCommand>()> external, bool demo)
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
    std::function<std::optional<CarCommand>()> external_;
    bool demo_;
    bool externalSeen_{false};
    const char* source_{"demo"};
    double wheelbase_{0.0};
    double maxSteer_{0.0};
    double integral_{0.0};
    double stalled_{0.0};       // s the demo has been held at a standstill
    double reversing_{0.0};     // s of backing away left
    double reverseSteer_{0.0};  // rad while backing away
    double yielding_{0.0};      // s of standing still left after backing away
    double tries_{0.0};         // backings away in a row, without driving freely between
    double free_{0.0};          // s driven freely since the last backing away
    SignalId speed_, pose_, throttle_, brake_, steering_;
};

}  // namespace lagrange::server
