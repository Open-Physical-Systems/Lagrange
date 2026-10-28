#include "lagrange/server/SwerveDrive.hpp"

#include <algorithm>
#include <array>
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

constexpr double kStillMps = 1e-3;  // below this a module keeps its angle

/// An angle in (-pi, pi].
double wrap(double angle) {
    return std::remainder(angle, 2.0 * std::numbers::pi);
}

}  // namespace

SwerveController::SwerveController(Settings settings) : settings_(std::move(settings)) {}

ModuleInfo SwerveController::info() const {
    return {"controller", "swerve_drive", 0, 1, false, true, CapabilityTier::Tier0};
}

Duration SwerveController::preferredStep() const {
    return 1_ms;
}

Status SwerveController::declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) {
    linear_ = d.input(scalar("base/command/linear_velocity", units::m_per_s));
    lateral_ = d.input(scalar("base/command/lateral_velocity", units::m_per_s));
    angular_ = d.input(scalar("base/command/angular_velocity", units::rad_per_s));
    const auto state = [&](const std::string& name, Unit unit) {
        SignalSpec spec = scalar(name, unit);
        spec.feedback = true;  // the physics reads our torques: take its previous step
        return d.input(spec);
    };
    signals_.clear();
    for (const Module& m : settings_.modules) {
        Signals s;
        s.steerAngle = state("joint/" + m.steerJoint + "/position", units::rad);
        s.steerRate = state("joint/" + m.steerJoint + "/velocity", units::rad_per_s);
        s.wheelSpeed = state("joint/" + m.wheelJoint + "/velocity", units::rad_per_s);
        s.steerTorque = d.output(scalar("joint/" + m.steerJoint + "/torque_command", units::N_m));
        s.wheelTorque = d.output(scalar("joint/" + m.wheelJoint + "/torque_command", units::N_m));
        signals_.push_back(s);
    }
    wheelIntegral_.assign(settings_.modules.size(), 0.0);
    return {};
}

Status SwerveController::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    for (const Signals& s : signals_) {
        bus.write(s.steerTorque, 0.0);
        bus.write(s.wheelTorque, 0.0);
    }
    return {};
}

Status SwerveController::step(const StepContext& context, SignalBus& bus) {
    const Settings& c = settings_;
    const double vx = bus.read(linear_);
    const double vy = bus.read(lateral_);
    const double wz = bus.read(angular_);
    const double dt = context.dt.seconds();
    for (std::size_t i = 0; i < c.modules.size(); ++i) {
        const Module& m = c.modules[i];
        const Signals& s = signals_[i];
        const double angle = bus.read(s.steerAngle);
        // The wheel's ground velocity at the module, in the base frame.
        const double ux = vx - wz * m.yM;
        const double uy = vy + wz * m.xM;
        double speed = std::hypot(ux, uy);
        double target = angle;
        if (speed > kStillMps) {
            target = std::atan2(uy, ux);
            // Never turn a module past 90°: steer the other way and reverse the wheel.
            if (std::abs(wrap(target - angle)) > std::numbers::pi / 2.0) {
                target = wrap(target + std::numbers::pi);
                speed = -speed;
            }
        } else {
            speed = 0.0;
        }
        const double error = wrap(target - angle);
        speed *= std::max(0.0, std::cos(error));  // drive only along where the wheel points
        bus.write(s.steerTorque, std::clamp(c.steerGainNmPerRad * error -
                                                c.steerDampingNmPerRadPerS * bus.read(s.steerRate),
                                            -m.steerTorqueLimitNm, m.steerTorqueLimitNm));
        const double wheelError = speed / c.wheelRadiusM - bus.read(s.wheelSpeed);
        const double unclamped =
            c.wheelGainNmPerRadPerS * wheelError + c.wheelIntegralGainNmPerRad * wheelIntegral_[i];
        // Anti-windup: integrate only while the torque is not at its limit.
        if (std::abs(unclamped) < m.wheelTorqueLimitNm) {
            wheelIntegral_[i] += wheelError * dt;
        }
        bus.write(s.wheelTorque,
                  std::clamp(unclamped, -m.wheelTorqueLimitNm, m.wheelTorqueLimitNm));
    }
    return {};
}

Status SwerveController::saveState(StateWriter& writer) const {
    for (const double integral : wheelIntegral_) {
        writer.put(integral);
    }
    return {};
}

Status SwerveController::restoreState(StateReader& reader) {
    for (double& integral : wheelIntegral_) {
        reader.get(integral);
    }
    return reader.failed() ? Status::error(StatusCode::InvalidConfig, "short controller state")
                           : Status{};
}

std::optional<SwerveController::Settings> findSwerveModules(const Model& robot) {
    const World& world = robot.world;
    const auto actuated = [&](std::size_t j) {
        return std::find(robot.actuatedJoints.begin(), robot.actuatedJoints.end(),
                         world.joints.names[j]) != robot.actuatedJoints.end();
    };
    SwerveController::Settings settings;
    std::vector<std::array<double, 2>> at;
    for (std::size_t steer = 0; steer < world.joints.names.size(); ++steer) {
        const bool turns = world.joints.type[steer] == JointType::Revolute ||
                           world.joints.type[steer] == JointType::Continuous;
        if (!actuated(steer) || !turns || world.joints.axis[steer][2] < 0.9) {
            continue;
        }
        for (std::size_t wheel = 0; wheel < world.joints.names.size(); ++wheel) {
            if (!actuated(wheel) ||
                world.joints.parent[wheel].index != world.joints.child[steer].index) {
                continue;
            }
            SwerveController::Module module;
            module.steerJoint = world.joints.names[steer];
            module.wheelJoint = world.joints.names[wheel];
            module.steerTorqueLimitNm = world.joints.effortLimit[steer] > 0.0
                                            ? world.joints.effortLimit[steer]
                                            : module.steerTorqueLimitNm;
            module.wheelTorqueLimitNm = world.joints.effortLimit[wheel] > 0.0
                                            ? world.joints.effortLimit[wheel]
                                            : module.wheelTorqueLimitNm;
            const Pose& pose = robot.bodyRestPose[world.joints.child[steer].index];
            at.push_back({pose.position[0], pose.position[1]});
            settings.modules.push_back(module);
            for (std::size_t c = 0; c < world.colliders.body.size(); ++c) {
                if (world.colliders.body[c].index == world.joints.child[wheel].index &&
                    (world.colliders.shape[c] == ShapeType::Cylinder ||
                     world.colliders.shape[c] == ShapeType::Sphere)) {
                    settings.wheelRadiusM = world.colliders.size[c][0];
                }
            }
            break;
        }
    }
    if (settings.modules.size() < 3) {
        return std::nullopt;
    }
    // The base turns about the modules' centre.
    double cx = 0.0;
    double cy = 0.0;
    for (const auto& p : at) {
        cx += p[0] / static_cast<double>(at.size());
        cy += p[1] / static_cast<double>(at.size());
    }
    for (std::size_t i = 0; i < at.size(); ++i) {
        settings.modules[i].xM = at[i][0] - cx;
        settings.modules[i].yM = at[i][1] - cy;
    }
    return settings;
}

}  // namespace lagrange::server
