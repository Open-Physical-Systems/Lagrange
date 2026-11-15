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
constexpr char kLateral[] = "base/command/lateral_velocity";

/// The demo of a robot that moves sideways: a square without turning (each side `side` seconds
/// at `v`), then a full turn on the spot at `v / radius`.
VelocityCommand holonomicDemo(double t, double v, double radius) {
    constexpr double kSideSeconds = 4.0;
    const double turnSeconds = 2.0 * std::numbers::pi * radius / v;
    const double period = 4.0 * kSideSeconds + turnSeconds;
    const double phase = std::fmod(t, period);
    if (phase >= 4.0 * kSideSeconds) {
        return {0.0, v / radius, 0.0};
    }
    switch (static_cast<int>(phase / kSideSeconds)) {
        case 0:
            return {v, 0.0, 0.0};
        case 1:
            return {0.0, 0.0, v};
        case 2:
            return {-v, 0.0, 0.0};
        default:
            return {0.0, 0.0, -v};
    }
}

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
    if (settings_.demo && !settings_.lateral && !settings_.basePose.empty()) {
        SignalSpec pose = scalar(settings_.basePose, units::m);
        pose.type = SignalType::Pose;
        pose.frame = "world_local";
        pose.feedback = true;
        pose_ = d.input(pose);
    }
    linear_ = d.output(scalar(kLinear, units::m_per_s));
    angular_ = d.output(scalar(kAngular, units::rad_per_s));
    if (settings_.lateral) {
        lateral_ = d.output(scalar(kLateral, units::m_per_s));
    }
    return {};
}

Status DriveCommand::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    eightStarted_ = false;
    eightSide_ = 1.0;
    eightTurned_ = 0.0;
    bus.write(linear_, 0.0);
    bus.write(angular_, 0.0);
    if (settings_.lateral) {
        bus.write(lateral_, 0.0);
    }
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
        if (s.lateral) {
            command = holonomicDemo(t, s.speedMps, s.turnRadiusM);
            command->linearMps *= v / s.speedMps;
            command->lateralMps *= v / s.speedMps;
        }
        const double loopSeconds = 2.0 * std::numbers::pi * s.turnRadiusM / s.speedMps;
        const double phase = std::sin(std::numbers::pi * t / loopSeconds);
        if (!s.lateral && pose_.valid()) {
            command = VelocityCommand{v, figureEightTurn(v, bus)};
        } else if (!s.lateral) {
            command = VelocityCommand{v, (v / s.turnRadiusM) * std::tanh(8.0 * phase)};
        }
        source_ = "demo";
    }
    bus.write(linear_, command->linearMps);
    bus.write(angular_, command->angularRadps);
    if (settings_.lateral) {
        bus.write(lateral_, command->lateralMps);
    }
    return {};
}

double DriveCommand::figureEightTurn(double v, SignalBus& bus) {
    constexpr double kRadialGain = 2.0;   // rad of heading per m off the circle (as atan)
    constexpr double kHeadingGain = 2.0;  // 1/s: rad/s of turn per rad of heading error
    const std::span<const double> p = bus.readN(pose_);
    if (p[3] == 0.0 && p[4] == 0.0 && p[5] == 0.0 && p[6] == 0.0) {
        return 0.0;  // no pose written yet
    }
    const double yaw =
        std::atan2(2.0 * (p[3] * p[6] + p[4] * p[5]), 1.0 - 2.0 * (p[5] * p[5] + p[6] * p[6]));
    const double r = settings_.turnRadiusM;
    const auto wrap = [](double a) { return std::remainder(a, 2.0 * std::numbers::pi); };
    const auto centre = [&] {
        // The loop's centre: a radius to the side of the start, left for the left loop.
        return std::array<double, 2>{eightStart_[0] - eightSide_ * r * std::sin(eightHeading_),
                                     eightStart_[1] + eightSide_ * r * std::cos(eightHeading_)};
    };
    if (!eightStarted_) {
        eightStarted_ = true;
        eightStart_ = {p[0], p[1]};
        eightHeading_ = yaw;
        eightSide_ = 1.0;
        eightTurned_ = 0.0;
        const auto c = centre();
        eightBearing_ = std::atan2(p[1] - c[1], p[0] - c[0]);
    }
    auto c = centre();
    double bearing = std::atan2(p[1] - c[1], p[0] - c[0]);
    eightTurned_ += eightSide_ * wrap(bearing - eightBearing_);
    eightBearing_ = bearing;
    if (eightTurned_ >= 2.0 * std::numbers::pi) {
        // Back where the loops touch: the other loop, the other way round.
        eightSide_ = -eightSide_;
        eightTurned_ = 0.0;
        c = centre();
        bearing = std::atan2(p[1] - c[1], p[0] - c[0]);
        eightBearing_ = bearing;
    }
    // Along the circle's tangent, leaning in when outside it and out when inside.
    const double off = std::hypot(p[0] - c[0], p[1] - c[1]) - r;
    const double wanted =
        bearing + eightSide_ * (0.5 * std::numbers::pi + std::atan(kRadialGain * off));
    return eightSide_ * v / r + kHeadingGain * wrap(wanted - yaw);
}

Status DriveCommand::saveState(StateWriter& writer) const {
    writer.put(externalSeen_);
    writer.put(eightStarted_);
    writer.put(eightStart_[0]);
    writer.put(eightStart_[1]);
    writer.put(eightHeading_);
    writer.put(eightSide_);
    writer.put(eightTurned_);
    writer.put(eightBearing_);
    return {};
}

Status DriveCommand::restoreState(StateReader& reader) {
    reader.get(externalSeen_);
    reader.get(eightStarted_);
    reader.get(eightStart_[0]);
    reader.get(eightStart_[1]);
    reader.get(eightHeading_);
    reader.get(eightSide_);
    reader.get(eightTurned_);
    reader.get(eightBearing_);
    return reader.failed() ? Status::error(StatusCode::InvalidConfig, "short command state")
                           : Status{};
}

void DriveCommand::shiftOrigin(const std::array<double, 3>& shift) {
    eightStart_[0] -= shift[0];
    eightStart_[1] -= shift[1];
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
    // Torque commands: the electrical domain (or its ideal substitute) turns them into torque.
    leftTorque_ = d.output(scalar("joint/" + settings_.leftJoint + "/torque_command", units::N_m));
    rightTorque_ =
        d.output(scalar("joint/" + settings_.rightJoint + "/torque_command", units::N_m));
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
