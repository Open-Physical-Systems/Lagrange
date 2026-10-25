// The road vehicle's driver (CarDrive.hpp).
#include "lagrange/module/StateIO.hpp"
#include "lagrange/server/CarDrive.hpp"
#include "lagrange/vehicle_dynamics/VehicleDynamics.hpp"

#include <algorithm>
#include <cmath>

namespace lagrange::server {

using namespace literals;

namespace {

constexpr double kSpeedGain = 0.4;      // throttle per m/s of speed error
constexpr double kSpeedIntegral = 0.2;  // throttle per m of accumulated error
constexpr double kBrakeGain = 0.3;      // brake per m/s too fast
constexpr double kHoldSpeed = 0.3;      // m/s: below it, a stop command holds the brake
constexpr double kHoldBrake = 0.3;
constexpr double kMinSteerSpeed = 1.0;  // m/s, the speed the steering angle assumes when slower
constexpr double kDemoRadius = 5.0;     // m, the demo circle round the origin
constexpr double kDemoSpeed = 3.0;      // m/s
constexpr double kLookahead = 4.0;      // m, pure pursuit

SignalSpec scalar(std::string name, Unit unit) {
    SignalSpec s;
    s.name = std::move(name);
    s.unit = unit;
    return s;
}

}  // namespace

ModuleInfo CarController::info() const {
    return {"controller", "car", 0, 1, false, !external_, CapabilityTier::Tier0};
}

Duration CarController::preferredStep() const {
    return 10_ms;
}

Status CarController::declare(const WorldView& world, const ModuleConfig&, SignalDeclarer& d) {
    auto vehicles = vehicle_dynamics::vehiclesOf(world);
    if (!vehicles) {
        return Status::error(StatusCode::InvalidConfig, vehicles.error());
    }
    if (vehicles->empty()) {
        return Status::error(StatusCode::InvalidConfig, "the car controller needs a vehicle");
    }
    const vehicle_dynamics::Vehicle& v = vehicles->front();
    wheelbase_ = v.wheelbase;
    maxSteer_ = v.maxSteerAngle;
    SignalSpec pose = scalar("body/" + world.bodies().names[v.chassis] + "/pose", units::m);
    pose.type = SignalType::Pose;
    pose.frame = "world_local";
    pose.feedback = true;
    pose_ = d.input(pose);
    SignalSpec speed = scalar("vehicle/" + v.name + "/speed", units::m_per_s);
    speed.feedback = true;
    speed_ = d.input(speed);
    throttle_ = d.output(scalar("vehicle/" + v.name + "/throttle", units::one));
    brake_ = d.output(scalar("vehicle/" + v.name + "/brake", units::one));
    steering_ = d.output(scalar("vehicle/" + v.name + "/steering_angle", units::rad));
    return {};
}

Status CarController::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    integral_ = 0.0;
    bus.write(throttle_, 0.0);
    bus.write(brake_, kHoldBrake);
    bus.write(steering_, 0.0);
    return {};
}

Status CarController::step(const StepContext& context, SignalBus& bus) {
    const double dt = context.dt.seconds();
    const double speed = bus.read(speed_);
    double target = 0.0;
    double turn = 0.0;
    std::optional<double> pursuit;  // a steering angle from the demo's pure pursuit
    if (const auto command = external_ ? external_() : std::nullopt) {
        externalSeen_ = true;
        source_ = "ros";
        target = command->linearMps;
        turn = command->angularRadps;
    } else if (externalSeen_ || !demo_) {
        source_ = "stopped";
    } else {
        source_ = "demo";
        // Pure pursuit of a point on the circle a lookahead ahead (anticlockwise).
        const std::span<const double> p = bus.readN(pose_);
        const double yaw =
            std::atan2(2.0 * (p[3] * p[6] + p[4] * p[5]), 1.0 - 2.0 * (p[5] * p[5] + p[6] * p[6]));
        const double angle = std::atan2(p[1], p[0]) + kLookahead / kDemoRadius;
        const double gx = kDemoRadius * std::cos(angle) - p[0];
        const double gy = kDemoRadius * std::sin(angle) - p[1];
        const double alpha = std::atan2(gy, gx) - yaw;
        const double distance = std::max(std::hypot(gx, gy), 1.0);
        pursuit = std::atan(2.0 * wheelbase_ * std::sin(alpha) / distance);
        target = kDemoSpeed;
    }
    double throttle = 0.0;
    double brake = 0.0;
    if (std::abs(target) < 1e-3 && std::abs(speed) < kHoldSpeed) {
        brake = kHoldBrake;  // stand still
        integral_ = 0.0;
    } else {
        const double error = target - speed;
        // Slowing down while moving the asked way: brake rather than reverse the drive.
        const bool tooFast = speed * target >= 0.0 && std::abs(speed) > std::abs(target) + 0.2;
        if (tooFast || (target * speed < 0.0 && std::abs(speed) > kHoldSpeed)) {
            brake = std::clamp(kBrakeGain * std::abs(error), 0.0, 1.0);
            integral_ = 0.0;
        } else {
            integral_ = std::clamp(integral_ + error * dt, -5.0, 5.0);
            throttle = std::clamp(kSpeedGain * error + kSpeedIntegral * integral_, -1.0, 1.0);
        }
    }
    const double v =
        std::abs(target) > kMinSteerSpeed ? target : std::copysign(kMinSteerSpeed, target);
    const double steer =
        std::clamp(pursuit ? *pursuit : std::atan(wheelbase_ * turn / v), -maxSteer_, maxSteer_);
    bus.write(throttle_, throttle);
    bus.write(brake_, brake);
    bus.write(steering_, steer);
    return {};
}

Status CarController::saveState(StateWriter& writer) const {
    writer.put(integral_);
    writer.put(externalSeen_);
    return {};
}

Status CarController::restoreState(StateReader& reader) {
    if (!reader.get(integral_) || !reader.get(externalSeen_)) {
        return Status::error(StatusCode::Internal, "car controller: truncated state");
    }
    return {};
}

}  // namespace lagrange::server
