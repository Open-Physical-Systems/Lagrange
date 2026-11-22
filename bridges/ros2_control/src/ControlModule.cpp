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

/// kg/s, a nozzle's flow (actuators' Nozzles.cpp).
constexpr Unit kKgPerS{{.mass = 1, .time = -1}, 1.0, QuantityKind::Unspecified};

/// The unit and type of a GPIO interface's signal: the <param>s' when given, else those of the
/// actuators domain's signal of that name (RFC 0020 §4: grippers, nozzles, servos, solenoids).
Expected<SignalSpec, std::string> gpioSpec(const std::string& gpio,
                                           const ControlGpio::Interface& interface) {
    struct Known {
        const char* leaf;
        Unit unit;
        bool flag;
    };
    static const Known kKnown[] = {
        {"command", units::one, false},
        {"pressure_command", units::Pa, false},
        {"pulse_width", units::s, false},
        {"goal_position", units::rad, false},
        {"goal_velocity", units::rad_per_s, false},
        {"goal_current", units::A, false},
        {"torque_enable", units::one, false},
        {"current", units::A, false},
        {"voltage", units::V, false},
        {"force", units::N, false},
        {"position", units::m, false},
        {"power", units::W, false},
        {"pressure", units::Pa, false},
        {"gap", units::m, false},
        {"flow", kKgPerS, false},
        {"tank_mass", units::kg, false},
        {"present_position", units::rad, false},
        {"present_velocity", units::rad_per_s, false},
        {"present_load", units::N_m, false},
        {"present_current", units::A, false},
        {"held", units::one, true},
        {"holding", units::one, true},
        {"sealed", units::one, true},
        {"overload", units::one, true},
    };
    SignalSpec out = spec(interface.signal, units::one);
    const std::string leaf = interface.signal.substr(interface.signal.rfind('/') + 1);
    bool found = false;
    for (const Known& k : kKnown) {
        if (leaf == k.leaf) {
            out.unit = k.unit;
            out.type = k.flag ? SignalType::Bool : SignalType::Scalar;
            found = true;
        }
    }
    if (!interface.unit.empty()) {
        const UnitParse parsed = parseUnit(interface.unit);
        if (!parsed.unit) {
            return Unexpected{"gpio " + gpio + "/" + interface.name + ": unit '" + interface.unit +
                              "' is not one Lagrange reads"};
        }
        out.unit = *parsed.unit;
        found = true;
    }
    if (!interface.type.empty()) {
        if (interface.type != "bool" && interface.type != "scalar") {
            return Unexpected{"gpio " + gpio + "/" + interface.name + ": type '" + interface.type +
                              "' is not bool or scalar"};
        }
        out.type = interface.type == "bool" ? SignalType::Bool : SignalType::Scalar;
    }
    if (!found) {
        return Unexpected{"gpio " + gpio + "/" + interface.name + ": " + interface.signal +
                          " is not an actuator signal Lagrange knows; give its <param "
                          "name=\"unit\"> (and <param name=\"type\">bool</param> for a flag)"};
    }
    return out;
}

class ControlModule final : public IModule {
public:
    ControlModule(ControllerHost& host, std::vector<ControlJoint> joints,
                  std::vector<ControlGpio> gpios)
        : host_(host), joints_(std::move(joints)), gpios_(std::move(gpios)) {}

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
            // A joint on a stepper (RFC 0020 §4.5): the position command goes to its driver,
            // converted to microsteps there; the effort state is the torque it makes.
            for (const SchemaInstance* motor : world.instancesOf("ElectricalMotorAPI_1")) {
                const auto driven = motor->rel("elec:joint");
                if (joint.servo || driven.size() != 1 || driven[0] != table.prim[j] ||
                    motor->token("elec:motorType").value_or("pmsm") != "stepper") {
                    continue;
                }
                joint.stepper = true;
                joint.torque =
                    d.output(spec(base + "/position_command", prismatic ? units::m : units::rad));
                joint.load = d.input(feedback(base + "/torque", prismatic ? units::N : units::N_m));
            }
            // The command: the electrical domain (or its ideal substitute) turns it into the
            // torque.
            if (!joint.servo && !joint.stepper) {
                joint.torque =
                    d.output(spec(base + "/torque_command", prismatic ? units::N : units::N_m));
            }
            signals_.push_back(joint);
        }
        // GPIOs (RFC 0020 §5): each interface to its signal, with the actuator signal's unit.
        gpioSignals_.clear();
        for (const ControlGpio& gpio : gpios_) {
            for (const bool command : {true, false}) {
                for (const ControlGpio::Interface& interface :
                     command ? gpio.commands : gpio.states) {
                    auto typed = gpioSpec(gpio.name, interface);
                    if (!typed) {
                        return Status::error(StatusCode::InvalidConfig, typed.error());
                    }
                    GpioSignal g;
                    g.gpio = gpio.name;
                    g.interface = interface.name;
                    g.command = command;
                    if (command) {
                        g.id = d.output(*typed);
                    } else {
                        // Read as the controllers would sample it: the last step's value; none
                        // when nothing in the run writes it (NaN to the controller).
                        typed->feedback = true;
                        typed->optional = true;
                        typed->defaultValue = std::numeric_limits<double>::quiet_NaN();
                        g.id = d.input(*typed);
                    }
                    gpioSignals_.push_back(std::move(g));
                }
            }
        }
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        const unsigned int rate = std::max(host_.updateRateHz(), 1U);
        periodNs_ = std::max<std::int64_t>(1'000'000'000 / rate, 1'000'000);
        lastCycleNs_ = -periodNs_;
        for (const Joint& joint : signals_) {
            // A servo keeps its goal until a controller gives one (NaN: no command).
            bus.write(joint.torque, joint.servo || joint.stepper
                                        ? std::numeric_limits<double>::quiet_NaN()
                                        : 0.0);
        }
        for (const GpioSignal& g : gpioSignals_) {
            if (g.command) {
                bus.write(g.id, std::numeric_limits<double>::quiet_NaN());  // none yet
            }
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
                slot.effort = joint.servo || joint.stepper ? bus.read(joint.load) : joint.applied;
                slot.glitchProbability = bus.read(joint.glitch);
            }
            for (const GpioSignal& g : gpioSignals_) {
                if (!g.command) {
                    table.gpio(g.gpio).states[g.interface] = bus.read(g.id);
                }
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
        for (const GpioSignal& g : gpioSignals_) {
            if (g.command) {
                const GpioSlot& slot = table.gpio(g.gpio);
                const auto value = slot.commands.find(g.interface);
                bus.write(g.id, value != slot.commands.end()
                                    ? value->second
                                    : std::numeric_limits<double>::quiet_NaN());
            }
        }
        for (Joint& joint : signals_) {
            const JointSlot& slot = table[joint.slot];
            if (joint.stepper) {
                // A position command is the driver's; without one it keeps its step count.
                bus.write(joint.torque, slot.mode == CommandMode::Position
                                            ? slot.commandPosition
                                            : std::numeric_limits<double>::quiet_NaN());
                continue;
            }
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
        bool stepper{false};  // `torque` is joint/<j>/position_command, `load` joint/<j>/torque
        SignalId load;
        CommandMode mode{CommandMode::None};
        double integral{0.0};
        double applied{0.0};
        double lastCommand{0.0};  // rad or m: the position command of the last controller cycle
        double commandRate{0.0};  // rad/s or m/s: how fast the command moved over that cycle
    };
    struct GpioSignal {
        std::string gpio, interface;
        bool command{false};
        SignalId id;
    };
    ControllerHost& host_;
    std::vector<ControlJoint> joints_;
    std::vector<ControlGpio> gpios_;
    std::vector<GpioSignal> gpioSignals_;
    std::vector<Joint> signals_;
    std::int64_t periodNs_{10'000'000};
    std::int64_t lastCycleNs_{0};
};

}  // namespace

JointTable& JointTable::instance() {
    static JointTable table;
    return table;
}

std::unique_ptr<IModule> createControlModule(ControllerHost& host, std::vector<ControlJoint> joints,
                                             std::vector<ControlGpio> gpios) {
    return std::make_unique<ControlModule>(host, std::move(joints), std::move(gpios));
}

}  // namespace lagrange::ros2_control
