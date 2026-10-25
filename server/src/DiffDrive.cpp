#include "lagrange/server/DiffDrive.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

namespace lagrange::server {

namespace {

using namespace literals;

SignalSpec scalar(std::string name, Unit unit) {
    SignalSpec spec;
    spec.name = std::move(name);
    spec.unit = unit;
    return spec;
}

constexpr char kLinear[] = "base/command/linear_velocity";
constexpr char kAngular[] = "base/command/angular_velocity";

}  // namespace

// --- command -----------------------------------------------------------------------------------

DriveCommand::DriveCommand(Settings settings,
                           std::function<std::optional<VelocityCommand>()> external)
    : settings_(settings), external_(std::move(external)) {}

ModuleInfo DriveCommand::info() const {
    return {"controller", "command", 0, 1, false, !external_, CapabilityTier::Tier0};
}

Duration DriveCommand::preferredStep() const {
    return 1_ms;
}

Status DriveCommand::declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) {
    linear_ = d.output(scalar(kLinear, units::m_per_s));
    angular_ = d.output(scalar(kAngular, units::rad_per_s));
    return {};
}

Status DriveCommand::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    bus.write(linear_, 0.0);
    bus.write(angular_, 0.0);
    return {};
}

Status DriveCommand::step(const StepContext& context, SignalBus& bus) {
    std::optional<VelocityCommand> command = external_ ? external_() : std::nullopt;
    if (command) {
        externalSeen_ = true;
        source_ = "ros";
    } else if (externalSeen_ || !settings_.demo) {
        command = VelocityCommand{};  // the external command went stale: stop
        source_ = "stopped";
    } else {
        // Demo: forward speed ramps up; the turn rate is a smoothed square wave, one full
        // loop to the left, then one to the right.
        const Settings& s = settings_;
        const double t = context.t.seconds();
        const double v = s.speedMps * std::min(1.0, t / s.rampSeconds);
        const double loopSeconds = 2.0 * std::numbers::pi * s.turnRadiusM / s.speedMps;
        const double phase = std::sin(std::numbers::pi * t / loopSeconds);
        command = VelocityCommand{v, (v / s.turnRadiusM) * std::tanh(8.0 * phase)};
        source_ = "demo";
    }
    bus.write(linear_, command->linearMps);
    bus.write(angular_, command->angularRadps);
    return {};
}

Status DriveCommand::saveState(StateWriter& writer) const {
    writer.put(externalSeen_);
    return {};
}

Status DriveCommand::restoreState(StateReader& reader) {
    reader.get(externalSeen_);
    return reader.failed() ? Status::error(StatusCode::InvalidConfig, "short command state")
                           : Status{};
}

// --- wheels ------------------------------------------------------------------------------------

DiffDriveController::DiffDriveController(Settings settings) : settings_(std::move(settings)) {}

ModuleInfo DiffDriveController::info() const {
    return {"controller", "diff_drive", 0, 1, false, true, CapabilityTier::Tier0};
}

Duration DiffDriveController::preferredStep() const {
    return 1_ms;
}

Status DiffDriveController::declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) {
    linear_ = d.input(scalar(kLinear, units::m_per_s));
    angular_ = d.input(scalar(kAngular, units::rad_per_s));
    const auto velocity = [&](const std::string& joint) {
        SignalSpec spec = scalar("joint/" + joint + "/velocity", units::rad_per_s);
        spec.feedback = true;  // the physics reads our torques: take its previous step
        return d.input(spec);
    };
    leftVelocity_ = velocity(settings_.leftJoint);
    rightVelocity_ = velocity(settings_.rightJoint);
    leftTorque_ = d.output(scalar("joint/" + settings_.leftJoint + "/torque", units::N_m));
    rightTorque_ = d.output(scalar("joint/" + settings_.rightJoint + "/torque", units::N_m));
    return {};
}

Status DiffDriveController::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    bus.write(leftTorque_, 0.0);
    bus.write(rightTorque_, 0.0);
    return {};
}

Status DiffDriveController::step(const StepContext& context, SignalBus& bus) {
    const Settings& s = settings_;
    const double v = bus.read(linear_);
    const double omega = bus.read(angular_);
    const double dt = context.dt.seconds();
    const auto wheel = [&](double target, SignalId velocity, double& integral) {
        const double error = target - bus.read(velocity);
        const double unclamped = s.gainNmPerRadPerS * error + s.integralGainNmPerRad * integral;
        // Anti-windup: integrate only while the torque is not at its limit.
        if (std::abs(unclamped) < s.torqueLimitNm) {
            integral += error * dt;
        }
        return std::clamp(unclamped, -s.torqueLimitNm, s.torqueLimitNm);
    };
    const double halfTrack = s.trackWidthM / 2.0;
    bus.write(leftTorque_,
              wheel((v - omega * halfTrack) / s.wheelRadiusM, leftVelocity_, leftIntegral_));
    bus.write(rightTorque_,
              wheel((v + omega * halfTrack) / s.wheelRadiusM, rightVelocity_, rightIntegral_));
    return {};
}

Status DiffDriveController::saveState(StateWriter& writer) const {
    writer.put(leftIntegral_);
    writer.put(rightIntegral_);
    return {};
}

Status DiffDriveController::restoreState(StateReader& reader) {
    reader.get(leftIntegral_);
    reader.get(rightIntegral_);
    return reader.failed() ? Status::error(StatusCode::InvalidConfig, "short controller state")
                           : Status{};
}

}  // namespace lagrange::server
