#include "lagrange/server/JointControl.hpp"

#include <algorithm>
#include <numbers>

namespace lagrange::server {

namespace {

constexpr double kBandwidthHz =
    8.0;  // of the position and velocity loops (a choice, stable at 1 kHz)

SignalSpec spec(std::string name, Unit unit) {
    SignalSpec s;
    s.name = std::move(name);
    s.type = SignalType::Scalar;
    s.unit = unit;
    return s;
}

}  // namespace

void JointTargets::set(const std::string& joint, JointTarget target) {
    const std::lock_guard<std::mutex> lock(mutex_);
    targets_[joint] = target;
}

std::map<std::string, JointTarget> JointTargets::all() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return targets_;
}

JointControl::JointControl(std::shared_ptr<JointTargets> targets,
                           std::map<std::string, std::string> driveModes)
    : targets_(std::move(targets)), driveModes_(std::move(driveModes)) {}

ModuleInfo JointControl::info() const {
    return {"controller", "joint_control", 1, 0};  // deterministic given its commands
}

Duration JointControl::preferredStep() const {
    return Duration{1'000'000};
}

Status JointControl::declare(const WorldView& world, const ModuleConfig&, SignalDeclarer& d) {
    joints_.clear();
    const World::Joints& joints = world.joints();
    for (std::size_t j = 0; j < joints.names.size(); ++j) {
        if (!joints.actuated[j]) {
            continue;
        }
        const bool slide = joints.type[j] == JointType::Prismatic;
        const std::string base = "joint/" + std::string(world.signalInstanceName(joints.prim[j]));
        Joint out;
        out.name = joints.names[j];
        out.limit = joints.effortLimit[j];
        // Feedback: physics reads the torque these make.
        SignalSpec position = spec(base + "/position", slide ? units::m : units::rad);
        SignalSpec velocity = spec(base + "/velocity", slide ? units::m_per_s : units::rad_per_s);
        SignalSpec inertia = spec(base + "/effective_inertia", slide ? units::kg : units::kg_m2);
        position.feedback = velocity.feedback = inertia.feedback = true;
        out.position = d.input(position);
        out.velocity = d.input(velocity);
        out.inertia = d.input(inertia);
        const auto drive = driveModes_.find(out.name);
        out.mode = drive == driveModes_.end() ? "torque" : drive->second;
        if (out.mode == "velocity") {
            out.torque = d.output(
                spec(base + "/velocity_command", slide ? units::m_per_s : units::rad_per_s));
        } else if (out.mode == "voltage") {
            out.torque = d.output(spec(base + "/voltage_command", units::V));
        } else {
            out.torque = d.output(spec(base + "/torque_command", slide ? units::N : units::N_m));
        }
        joints_.push_back(std::move(out));
    }
    return {};
}

Status JointControl::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    for (const Joint& j : joints_) {
        bus.write(j.torque, 0.0);
    }
    return {};
}

Status JointControl::step(const StepContext&, SignalBus& bus) {
    const double omega = 2.0 * std::numbers::pi * kBandwidthHz;
    const std::map<std::string, JointTarget> targets = targets_->all();
    for (const Joint& j : joints_) {
        const auto it = targets.find(j.name);
        double torque = 0.0;
        if (j.mode != "torque") {
            // The motor closes the loop: it is told a speed, or volts.
            double out = 0.0;
            if (it != targets.end()) {
                const JointTarget& t = it->second;
                using Mode = JointTarget::Mode;
                if (j.mode == "voltage") {
                    out = t.mode == Mode::Voltage || t.mode == Mode::Effort ? t.value : 0.0;
                } else if (t.mode == Mode::Velocity) {
                    out = t.value;
                } else if (t.mode == Mode::Position) {
                    out = omega * (t.value - bus.read(j.position));
                }
            }
            bus.write(j.torque, out);
            continue;
        }
        if (it != targets.end()) {
            const double inertia = bus.read(j.inertia);
            const JointTarget& t = it->second;
            switch (t.mode) {
                case JointTarget::Mode::Position:
                    torque = inertia * (omega * omega * (t.value - bus.read(j.position)) -
                                        2.0 * omega * bus.read(j.velocity));
                    break;
                case JointTarget::Mode::Velocity:
                    torque = inertia * 2.0 * omega * (t.value - bus.read(j.velocity));
                    break;
                case JointTarget::Mode::Effort:
                    torque = t.value;
                    break;
                case JointTarget::Mode::Voltage:  // no motor to take volts
                case JointTarget::Mode::None:
                    break;
            }
        }
        if (j.limit > 0.0) {
            torque = std::clamp(torque, -j.limit, j.limit);
        }
        bus.write(j.torque, torque);
    }
    return {};
}

// The targets come from outside and outlive a rebuilt run.
Status JointControl::saveState(StateWriter&) const {
    return {};
}

Status JointControl::restoreState(StateReader&) {
    return {};
}

}  // namespace lagrange::server
