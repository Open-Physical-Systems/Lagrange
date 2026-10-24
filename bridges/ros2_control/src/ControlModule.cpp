// The scheduler module that steps the controller manager and turns commands into joint torque.
#include "lagrange/ros2_control/ControllerHost.hpp"
#include "lagrange/ros2_control/JointTable.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace lagrange::ros2_control {

namespace {

using namespace literals;

SignalSpec spec(std::string name, Unit unit) {
    SignalSpec out;
    out.name = std::move(name);
    out.type = SignalType::Scalar;
    out.unit = unit;
    return out;
}

/// A joint state the controllers read: the physics reads our torque, so take its previous
/// step (as a real controller reads the last sample) to break the loop.
SignalSpec feedback(std::string name, Unit unit) {
    SignalSpec out = spec(std::move(name), unit);
    out.feedback = true;
    return out;
}

class ControlModule final : public IModule {
public:
    ControlModule(ControllerHost& host, std::vector<ControlJoint> joints)
        : host_(host), joints_(std::move(joints)) {}

    // Not deterministic: commands come from controllers configured over ROS at run time.
    [[nodiscard]] ModuleInfo info() const override {
        return {"control", "ros2_control", 0, 1, false, false, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return 1_ms; }

    Status declare(const WorldView& world, const ModuleConfig&, SignalDeclarer& d) override {
        const World::Joints& table = world.joints();
        for (ControlJoint& control : joints_) {
            const auto it = std::find(table.names.begin(), table.names.end(), control.name);
            if (it == table.names.end()) {
                return Status::error(StatusCode::InvalidConfig,
                                     "<ros2_control> names joint '" + control.name +
                                         "', which the robot does not have");
            }
            const auto j = static_cast<std::size_t>(it - table.names.begin());
            if (!table.actuated[j]) {
                return Status::error(StatusCode::InvalidConfig,
                                     "joint '" + control.name + "' has no command interface");
            }
            const bool prismatic = table.type[j] == JointType::Prismatic;
            const std::string base =
                "joint/" + std::string(world.signalInstanceName(table.prim[j]));
            Joint joint;
            joint.slot = control.name;
            joint.gains = control.gains;
            joint.effortLimit = table.effortLimit[j] > 0.0 ? table.effortLimit[j] : INFINITY;
            joint.position =
                d.input(feedback(base + "/position", prismatic ? units::m : units::rad));
            joint.velocity = d.input(
                feedback(base + "/velocity", prismatic ? units::m_per_s : units::rad_per_s));
            // The command: the electrical domain (or its ideal substitute) turns it into the
            // torque.
            joint.torque =
                d.output(spec(base + "/torque_command", prismatic ? units::N : units::N_m));
            signals_.push_back(joint);
        }
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        const unsigned int rate = std::max(host_.updateRateHz(), 1U);
        periodNs_ = std::max<std::int64_t>(1'000'000'000 / rate, 1'000'000);
        lastCycleNs_ = -periodNs_;
        for (const Joint& joint : signals_) {
            bus.write(joint.torque, 0.0);
        }
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        JointTable& table = JointTable::instance();
        // Commands computed now act during this step, so the controllers see its end time;
        // that is never 0, which the controller manager refuses.
        const std::int64_t now = (context.t + context.dt).ns;
        {
            const std::lock_guard<std::mutex> lock(table.mutex());
            for (const Joint& joint : signals_) {
                JointSlot& slot = table[joint.slot];
                slot.position = bus.read(joint.position);
                slot.velocity = bus.read(joint.velocity);
                slot.effort = joint.applied;
            }
        }
        if (now - lastCycleNs_ >= periodNs_) {
            // read -> update -> write at the controller rate, in simulation time: lockstep.
            host_.cycle(now, lastCycleNs_ < 0 ? periodNs_ : now - lastCycleNs_);
            lastCycleNs_ = now;
        }
        const double dt = context.dt.seconds();
        const std::lock_guard<std::mutex> lock(table.mutex());
        for (Joint& joint : signals_) {
            const JointSlot& slot = table[joint.slot];
            double torque = 0.0;
            if (slot.mode != joint.mode) {
                joint.integral = 0.0;  // a new controller starts from rest
                joint.mode = slot.mode;
            }
            switch (slot.mode) {
                case CommandMode::Effort:
                    torque = slot.commandEffort;
                    break;
                case CommandMode::Velocity: {
                    const double error = slot.commandVelocity - slot.velocity;
                    joint.integral += error * dt;
                    torque = joint.gains.velocityP * error + joint.gains.velocityI * joint.integral;
                    break;
                }
                case CommandMode::Position:
                    torque = joint.gains.positionP * (slot.commandPosition - slot.position) -
                             joint.gains.positionD * slot.velocity;
                    break;
                case CommandMode::None:
                    break;
            }
            const double clamped = std::clamp(torque, -joint.effortLimit, joint.effortLimit);
            // Anti-windup: stop integrating while the motor saturates.
            if (clamped != torque && slot.mode == CommandMode::Velocity) {
                joint.integral -= (slot.commandVelocity - slot.velocity) * dt;
            }
            joint.applied = clamped;
            bus.write(joint.torque, clamped);
        }
        return {};
    }

    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    struct Joint {
        std::string slot;
        ServoGains gains;
        double effortLimit{INFINITY};
        SignalId position, velocity, torque;
        CommandMode mode{CommandMode::None};
        double integral{0.0};
        double applied{0.0};
    };
    ControllerHost& host_;
    std::vector<ControlJoint> joints_;
    std::vector<Joint> signals_;
    std::int64_t periodNs_{10'000'000};
    std::int64_t lastCycleNs_{0};
};

}  // namespace

JointTable& JointTable::instance() {
    static JointTable table;
    return table;
}

std::unique_ptr<IModule> createControlModule(ControllerHost& host,
                                             std::vector<ControlJoint> joints) {
    return std::make_unique<ControlModule>(host, std::move(joints));
}

}  // namespace lagrange::ros2_control
