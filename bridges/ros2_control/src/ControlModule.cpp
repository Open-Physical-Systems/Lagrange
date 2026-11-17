// The scheduler module that steps the controller manager and turns commands into joint torque.
#include "lagrange/ros2_control/ControllerHost.hpp"
#include "lagrange/ros2_control/JointTable.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
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
            // An encoder victim of EMI on the joint's prim (EmiSusceptibilityAPI_1, encoder).
            SignalSpec glitch = spec("emi/" + std::string(world.signalInstanceName(table.prim[j])) +
                                         "/glitch_probability",
                                     units::one);
            glitch.optional = true;
            joint.glitch = d.input(glitch);
            // A joint with a servo (RFC 0020 §4.3, §5): the position command is the servo's goal
            // (its board closes the loop), and the effort state is its present load.
            for (const SchemaInstance* servo : world.instancesOf("ServoAPI_1")) {
                if (servo->prim != table.prim[j]) {
                    continue;
                }
                const std::string act =
                    "act/" + std::string(world.signalInstanceName(table.prim[j])) + "/";
                joint.servoVelocity =
                    servo->token("act:servo:operatingMode").value_or("position") == "velocity";
                joint.servo = true;
                joint.torque = joint.servoVelocity
                                   ? d.output(spec(act + "goal_velocity", units::rad_per_s))
                                   : d.output(spec(act + "goal_position", units::rad));
                joint.load = d.input(feedback(act + "present_load", units::N_m));
            }
            // The command: the electrical domain (or its ideal substitute) turns it into the
            // torque.
            if (!joint.servo) {
                joint.torque =
                    d.output(spec(base + "/torque_command", prismatic ? units::N : units::N_m));
            }
            signals_.push_back(joint);
        }
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        const unsigned int rate = std::max(host_.updateRateHz(), 1U);
        periodNs_ = std::max<std::int64_t>(1'000'000'000 / rate, 1'000'000);
        lastCycleNs_ = -periodNs_;
        for (const Joint& joint : signals_) {
            // A servo keeps its goal until a controller gives one (NaN: no command).
            bus.write(joint.torque, joint.servo ? std::numeric_limits<double>::quiet_NaN() : 0.0);
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
                slot.effort = joint.servo ? bus.read(joint.load) : joint.applied;
                slot.glitchProbability = bus.read(joint.glitch);
            }
        }
        bool cycled = false;
        double cyclePeriod = 0.0;
        if (now - lastCycleNs_ >= periodNs_) {
            // read -> update -> write at the controller rate, in simulation time: lockstep.
            const std::int64_t period = lastCycleNs_ < 0 ? periodNs_ : now - lastCycleNs_;
            host_.cycle(now, period);
            lastCycleNs_ = now;
            cycled = true;
            cyclePeriod = static_cast<double>(period) * 1e-9;
        }
        const double dt = context.dt.seconds();
        const std::lock_guard<std::mutex> lock(table.mutex());
        for (Joint& joint : signals_) {
            const JointSlot& slot = table[joint.slot];
            if (joint.servo) {
                // Position (or, for a servo in velocity mode, velocity) commands are its goal;
                // others it cannot follow, and it keeps its last goal.
                double goal = std::numeric_limits<double>::quiet_NaN();
                if (!joint.servoVelocity && slot.mode == CommandMode::Position) {
                    goal = slot.commandPosition;
                } else if (joint.servoVelocity && slot.mode == CommandMode::Velocity) {
                    goal = slot.commandVelocity;
                }
                bus.write(joint.torque, goal);
                continue;
            }
            double torque = 0.0;
            if (slot.mode != joint.mode) {
                joint.integral = 0.0;  // a new controller starts from rest
                joint.mode = slot.mode;
                joint.lastCommand = slot.commandPosition;
                joint.commandRate = 0.0;
            } else if (cycled && slot.mode == CommandMode::Position) {
                // The commanded velocity: how fast the controller moves the set point (a
                // trajectory controller's spline), the drive's velocity feedforward.
                joint.commandRate = (slot.commandPosition - joint.lastCommand) / cyclePeriod;
                joint.lastCommand = slot.commandPosition;
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
                case CommandMode::Position: {
                    const double error = slot.commandPosition - slot.position;
                    joint.integral += error * dt;
                    // D on the velocity error: damping without holding a moving joint back.
                    torque = joint.gains.positionP * error +
                             joint.gains.positionI * joint.integral +
                             joint.gains.positionD * (joint.commandRate - slot.velocity);
                    break;
                }
                case CommandMode::None:
                    break;
            }
            const double clamped = std::clamp(torque, -joint.effortLimit, joint.effortLimit);
            // Anti-windup: stop integrating while the motor saturates.
            if (clamped != torque && slot.mode == CommandMode::Velocity) {
                joint.integral -= (slot.commandVelocity - slot.velocity) * dt;
            } else if (clamped != torque && slot.mode == CommandMode::Position) {
                joint.integral -= (slot.commandPosition - slot.position) * dt;
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
        SignalId position, velocity, torque, glitch;
        // A servo's joint: `torque` is then its act/<s>/goal_position (or goal_velocity), and
        // `load` its present load.
        bool servo{false}, servoVelocity{false};
        SignalId load;
        CommandMode mode{CommandMode::None};
        double integral{0.0};
        double applied{0.0};
        double lastCommand{0.0};  // rad or m: the position command of the last controller cycle
        double commandRate{0.0};  // rad/s or m/s: how fast the command moved over that cycle
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
