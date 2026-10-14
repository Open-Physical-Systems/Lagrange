#include "lagrange/server/FigureEightDriver.hpp"

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

}  // namespace

FigureEightDriver::FigureEightDriver(Settings settings) : settings_(std::move(settings)) {}

ModuleInfo FigureEightDriver::info() const {
    return {"controller", "figure_eight", 0, 1, false, true, CapabilityTier::Tier0};
}

Duration FigureEightDriver::preferredStep() const {
    return 1_ms;
}

Status FigureEightDriver::declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) {
    const auto velocity = [&](const std::string& joint) {
        SignalSpec spec = scalar("joint/" + joint + "/velocity", units::rad_per_s);
        spec.feedback = true;  // the physics reads our torques: take its previous step
        return d.input(spec);
    };
    leftVelocity_ = velocity(settings_.leftJoint);
    rightVelocity_ = velocity(settings_.rightJoint);
    leftTorque_ = d.output(scalar("joint/" + settings_.leftJoint + "/torque", units::N_m));
    rightTorque_ = d.output(scalar("joint/" + settings_.rightJoint + "/torque", units::N_m));
    linearCommand_ = d.output(scalar("base/command/linear_velocity", units::m_per_s));
    angularCommand_ = d.output(scalar("base/command/angular_velocity", units::rad_per_s));
    return {};
}

Status FigureEightDriver::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    for (const SignalId id : {leftTorque_, rightTorque_, linearCommand_, angularCommand_}) {
        bus.write(id, 0.0);
    }
    return {};
}

Status FigureEightDriver::step(const StepContext& context, SignalBus& bus) {
    const Settings& s = settings_;
    const double t = context.t.seconds();
    // Forward speed ramps up; the turn rate is a smoothed square wave: one loop to the left,
    // one to the right, each a full circle of the turn radius.
    const double v = s.speedMps * std::min(1.0, t / s.rampSeconds);
    const double loopSeconds = 2.0 * std::numbers::pi * s.turnRadiusM / s.speedMps;
    const double phase = std::sin(std::numbers::pi * t / loopSeconds);
    const double omega = (v / s.turnRadiusM) * std::tanh(8.0 * phase);

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
    bus.write(linearCommand_, v);
    bus.write(angularCommand_, omega);
    return {};
}

Status FigureEightDriver::saveState(StateWriter& writer) const {
    writer.put(leftIntegral_);
    writer.put(rightIntegral_);
    return {};
}

Status FigureEightDriver::restoreState(StateReader& reader) {
    reader.get(leftIntegral_);
    reader.get(rightIntegral_);
    return reader.failed() ? Status::error(StatusCode::InvalidConfig, "short driver state")
                           : Status{};
}

}  // namespace lagrange::server
