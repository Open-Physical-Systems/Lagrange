#include "lagrange/actuators/Servos.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lagrange::actuators {

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
/// The winding temperature read when no thermal model writes one, K (the electrical domain's
/// own default for its temperature inputs, 25 °C).
constexpr double kAmbientK = 298.15;
/// How close to the current limit counts as "at the limit" for the overload rule on a drive that
/// closes the loop itself (position path): a reporting threshold, not a device value.
constexpr double kAtLimit = 0.99;
/// Hops through PortAPI_1 connections from a motor's elec:powerPort to its bus (as the network).
constexpr int kMaxPortHops = 8;

SignalSpec scalar(std::string name, Unit unit) {
    SignalSpec s;
    s.name = std::move(name);
    s.type = SignalType::Scalar;
    s.unit = unit;
    return s;
}
SignalSpec optional(std::string name, Unit unit, double fallback) {
    SignalSpec s = scalar(std::move(name), unit);
    s.optional = true;
    s.defaultValue = fallback;
    return s;
}
SignalSpec feedback(std::string name, Unit unit, double fallback, bool isOptional) {
    SignalSpec s = scalar(std::move(name), unit);
    s.feedback = true;
    s.optional = isOptional;
    s.defaultValue = fallback;
    return s;
}

bool same(double a, double b) {
    return a == b || (std::isnan(a) && std::isnan(b));
}

/// The bus a motor's elec:powerPort leads to, through ports' connectedTo; nullopt for none.
std::optional<PrimId> busOf(const WorldView& world, PrimId prim) {
    for (int hop = 0; hop < kMaxPortHops; ++hop) {
        std::optional<PrimId> next;
        for (const SchemaInstance* s : world.instancesOf("ElectricalBusAPI_1")) {
            if (s->prim == prim) {
                return prim;
            }
        }
        for (const SchemaInstance* s : world.instancesOf("PortAPI_1")) {
            if (s->prim != prim) {
                continue;
            }
            const auto targets = s->rel("elec:port:" + s->instance + ":connectedTo");
            if (!targets.empty()) {
                next = targets[0];
                break;
            }
        }
        if (!next) {
            return std::nullopt;
        }
        prim = *next;
    }
    return std::nullopt;
}

}  // namespace

std::vector<ServoSpec> servosOf(const WorldView& world, bool electrical, std::string& error) {
    std::vector<ServoSpec> out;
    const World::Joints& joints = world.joints();
    const auto fail = [&](const std::string& who, const std::string& why) {
        error = "servo " + who + ": " + why;
        return std::vector<ServoSpec>{};
    };
    for (const SchemaInstance* s : world.instancesOf("ServoAPI_1")) {
        ServoSpec spec;
        spec.prim = s->prim;
        spec.name = std::string(world.signalInstanceName(s->prim));
        const auto j = std::find(joints.prim.begin(), joints.prim.end(), s->prim);
        if (j == joints.prim.end()) {
            return fail(spec.name, "ServoAPI_1 must be on a joint");
        }
        const auto index = static_cast<std::size_t>(j - joints.prim.begin());
        if (joints.type[index] != JointType::Revolute &&
            joints.type[index] != JointType::Continuous) {
            return fail(spec.name, "a servo turns a revolute or continuous joint");
        }
        spec.joint = joints.names[index];
        spec.start = joints.initialPosition[index];
        spec.lower = joints.lowerLimit[index];
        spec.upper = joints.upperLimit[index];
        if (!std::isfinite(spec.lower) || !std::isfinite(spec.upper)) {
            spec.lower = spec.upper = 0.0;  // no range
        }
        ServoParams& p = spec.params;
        const std::string kind(s->token("act:servo:kind").value_or("pwm"));
        if (kind != "pwm" && kind != "smart") {
            return fail(spec.name, "act:servo:kind " + kind + " is not pwm or smart");
        }
        p.kind = kind == "smart" ? ServoParams::Kind::Smart : ServoParams::Kind::Pwm;
        const std::string mode(s->token("act:servo:operatingMode").value_or("position"));
        p.mode = mode == "velocity"  ? ServoParams::OperatingMode::Velocity
                 : mode == "current" ? ServoParams::OperatingMode::Current
                                     : ServoParams::OperatingMode::Position;
        if (mode != "position" && mode != "velocity" && mode != "current") {
            return fail(spec.name, "act:servo:operatingMode " + mode +
                                       " is not position, velocity or current");
        }
        if (p.kind == ServoParams::Kind::Pwm && p.mode != ServoParams::OperatingMode::Position) {
            return fail(spec.name, "a pwm servo follows a position only");
        }
        const auto kp = s->real("act:servo:kp");
        if (!kp || !(*kp > 0.0)) {
            return fail(spec.name, "act:servo:kp must be given and over 0");
        }
        p.kp = *kp;
        if (p.kind == ServoParams::Kind::Pwm) {
            const auto pulseMin = s->real("act:servo:pulseMin");
            const auto pulseMax = s->real("act:servo:pulseMax");
            const auto angleMin = s->real("act:servo:angleMin");
            const auto angleMax = s->real("act:servo:angleMax");
            if (!pulseMin || !pulseMax || !angleMin || !angleMax || !(*pulseMin > 0.0) ||
                !(*pulseMax > 0.0) || *pulseMin == *pulseMax) {
                return fail(spec.name,
                            "a pwm servo needs act:servo:pulseMin and pulseMax (over 0, not "
                            "equal), angleMin and angleMax");
            }
            p.pulseMin = *pulseMin;
            p.pulseMax = *pulseMax;
            p.angleMin = *angleMin;
            p.angleMax = *angleMax;
        }
        p.deadband = s->real("act:servo:deadband").value_or(0.0);
        p.ki = s->real("act:servo:ki").value_or(0.0);
        p.kd = s->real("act:servo:kd").value_or(0.0);
        p.velocityKp = s->real("act:servo:velocityKp").value_or(0.0);
        p.velocityKi = s->real("act:servo:velocityKi").value_or(0.0);
        p.profileVelocity = s->real("act:servo:profileVelocity").value_or(0.0);
        p.profileAcceleration = s->real("act:servo:profileAcceleration").value_or(0.0);
        p.resolution = s->real("act:servo:resolution").value_or(0.0);
        p.stallTorque = s->real("act:servo:stallTorque").value_or(0.0);
        p.noLoadSpeed = s->real("act:servo:noLoadSpeed").value_or(0.0);
        p.overloadTime = s->real("act:servo:overloadTime").value_or(0.0);
        p.maxTemperature = s->real("act:servo:maxTemperature").value_or(0.0);
        for (const double v : {p.deadband, p.ki, p.kd, p.velocityKp, p.velocityKi,
                               p.profileVelocity, p.profileAcceleration, p.resolution,
                               p.stallTorque, p.noLoadSpeed, p.overloadTime, p.maxTemperature}) {
            if (!(v >= 0.0)) {
                return fail(spec.name, "its gains, limits and times must not be negative");
            }
        }
        if (p.mode == ServoParams::OperatingMode::Velocity && !(p.velocityKp > 0.0)) {
            return fail(spec.name, "velocity mode needs act:servo:velocityKp over 0");
        }
        // Its motor: the ElectricalMotorAPI_1 on this joint.
        const SchemaInstance* motor = nullptr;
        for (const SchemaInstance* m : world.instancesOf("ElectricalMotorAPI_1")) {
            const auto joint = m->rel("elec:joint");
            if (joint.size() == 1 && joint[0] == s->prim) {
                motor = m;
            }
        }
        std::string control;
        if (motor != nullptr) {
            spec.motor = std::string(world.signalInstanceName(motor->prim));
            if (motor->token("elec:motorType").value_or("pmsm") != "dc_brushed") {
                return fail(spec.name, "its motor " + spec.motor + " must be dc_brushed");
            }
            control = std::string(motor->token("elec:controlMode").value_or("torque"));
            if (control != "voltage" && control != "position") {
                return fail(spec.name, "its motor " + spec.motor +
                                           " must be in elec:controlMode voltage (the board's "
                                           "loop here) or position (the loop on the drive)");
            }
            const double ratio = motor->real("elec:gearRatio").value_or(1.0);
            const double kt = motor->real("elec:torqueConstant").value_or(0.0);
            spec.armature.resistance = motor->real("elec:windingResistance").value_or(0.0);
            spec.armature.backEmf = ratio * kt;  // Ke = Kt for a brushed motor
            spec.armature.torquePerAmp =
                ratio * kt * motor->real("elec:gearEfficiency").value_or(1.0);
            // A hobby board's bare H-bridge has no current limit; a smart servo's does.
            spec.armature.maxCurrent = p.kind == ServoParams::Kind::Smart
                                           ? motor->real("elec:maxCurrent").value_or(0.0)
                                           : 0.0;
            const auto port = motor->rel("elec:powerPort");
            if (const auto bus = port.empty() ? std::nullopt : busOf(world, port[0])) {
                spec.bus = std::string(world.signalInstanceName(*bus));
            }
        }
        if (motor != nullptr && electrical) {
            if (control == "position") {
                spec.drive = ServoSpec::Drive::Position;
                if (p.mode != ServoParams::OperatingMode::Position) {
                    return fail(spec.name,
                                "a motor in position mode runs the position loop "
                                "only: put it in voltage mode for velocity or "
                                "current");
                }
            } else {
                spec.drive = ServoSpec::Drive::Voltage;
                if (spec.bus.empty()) {
                    return fail(spec.name, "its motor's elec:powerPort leads to no bus");
                }
            }
        } else {
            spec.drive = ServoSpec::Drive::Ideal;
            if (!(p.stallTorque > 0.0) || !(p.noLoadSpeed > 0.0)) {
                return fail(spec.name,
                            motor == nullptr
                                ? "without a motor it needs act:servo:stallTorque and noLoadSpeed"
                                : "with the electrical domain off it needs "
                                  "act:servo:stallTorque and noLoadSpeed");
            }
            if (p.mode == ServoParams::OperatingMode::Current) {
                return fail(spec.name, "current mode needs its motor and the electrical domain");
            }
        }
        out.push_back(std::move(spec));
    }
    return out;
}

ModuleInfo Servos::info() const {
    const char* name = mode_ == Mode::Full    ? "servos"
                       : mode_ == Mode::Ideal ? "servos_ideal"
                       : mode_ == Mode::Hold  ? "servos_constant"
                                              : "servos_zero";
    return {"actuators", name, 1, 0, mode_ != Mode::Full};
}

Duration Servos::preferredStep() const {
    return Duration{1'000'000};
}

Status Servos::declare(const WorldView& world, const ModuleConfig& config, SignalDeclarer& d) {
    std::string error;
    const bool electrical = config.get<double>(kElectricalKey).value_or(1.0) != 0.0;
    specs_ = servosOf(world, electrical, error);
    if (!error.empty()) {
        return Status::error(StatusCode::InvalidConfig, error);
    }
    if (mode_ == Mode::Ideal) {
        // A perfect board: the same loop with nothing of the electronics' imperfections.
        for (ServoSpec& s : specs_) {
            s.params.deadband = s.params.resolution = 0.0;
            s.params.profileVelocity = s.params.profileAcceleration = 0.0;
            s.params.overloadTime = s.params.maxTemperature = 0.0;
        }
    }
    in_.assign(specs_.size(), In{});
    out_.assign(specs_.size(), Out{});
    for (std::size_t k = 0; k < specs_.size(); ++k) {
        const ServoSpec& s = specs_[k];
        const std::string joint = "joint/" + s.name + "/";
        const std::string act = "act/" + s.name + "/";
        In& in = in_[k];
        Out& o = out_[k];
        in.position = d.input(feedback(joint + "position", units::rad, 0.0, false));
        in.velocity = d.input(feedback(joint + "velocity", units::rad_per_s, 0.0, false));
        in.goalPosition = d.input(optional(act + "goal_position", units::rad, kNaN));
        in.pulseWidth = d.input(optional(act + "pulse_width", units::s, kNaN));
        in.goalVelocity = d.input(optional(act + "goal_velocity", units::rad_per_s, kNaN));
        in.goalCurrent = d.input(optional(act + "goal_current", units::A, kNaN));
        in.torqueEnable = d.input(optional(act + "torque_enable", units::one, 1.0));
        if (s.drive != ServoSpec::Drive::Ideal) {
            const std::string motor = "motor/" + s.motor + "/";
            in.current = d.input(feedback(motor + "current_q", units::A, 0.0, false));
            in.torque = d.input(feedback(joint + "torque", units::N_m, 0.0, false));
            in.temperature =
                d.input(feedback("thermal/" + s.motor + "/temperature", units::K, kAmbientK, true));
            o.presentCurrent = d.output(scalar(act + "present_current", units::A));
            o.temperature = d.output(scalar(act + "temperature", units::K));
        }
        if (s.drive == ServoSpec::Drive::Voltage) {
            in.busVoltage = d.input(feedback("bus/" + s.bus + "/voltage", units::V, 0.0, false));
            o.command = d.output(scalar(joint + "voltage_command", units::V));
        } else if (s.drive == ServoSpec::Drive::Position) {
            o.command = d.output(scalar(joint + "position_command", units::rad));
        } else {
            o.command = d.output(scalar(joint + "torque_command", units::N_m));
        }
        o.presentPosition = d.output(scalar(act + "present_position", units::rad));
        o.presentVelocity = d.output(scalar(act + "present_velocity", units::rad_per_s));
        o.presentLoad = d.output(scalar(act + "present_load", units::N_m));
        o.goal = d.output(scalar(act + "goal", units::rad));
        SignalSpec overload = scalar(act + "overload", units::one);
        overload.type = SignalType::Bool;
        o.overload = d.output(overload);
    }
    return {};
}

Status Servos::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    state_.assign(specs_.size(), State{});
    for (std::size_t k = 0; k < specs_.size(); ++k) {
        const ServoSpec& s = specs_[k];
        State& st = state_[k];
        st.goal = s.params.mode == ServoParams::OperatingMode::Position ? s.start : 0.0;
        st.profile.position = s.start;
        st.busGoalPosition = st.busPulseWidth = st.busGoalVelocity = st.busGoalCurrent = kNaN;
        const Out& o = out_[k];
        const bool off = mode_ == Mode::Off;
        bus.write(o.command, s.drive == ServoSpec::Drive::Position ? (off ? kNaN : s.start) : 0.0);
        bus.write(o.presentPosition, quantise(s.start, s.params.resolution));
        bus.write(o.presentVelocity, 0.0);
        bus.write(o.presentLoad, 0.0);
        bus.write(o.goal, s.start);
        bus.write(o.overload, 0.0);
        if (o.presentCurrent.valid()) {
            bus.write(o.presentCurrent, 0.0);
            bus.write(o.temperature, kAmbientK);
        }
    }
    seenGeneration_ = commands_ ? commands_->generation() : 0;
    return {};
}

Status Servos::step(const StepContext& context, SignalBus& bus) {
    const double dt = context.dt.seconds();
    const bool fresh = commands_ && commands_->generation() != seenGeneration_;
    if (fresh) {
        seenGeneration_ = commands_->generation();
    }
    for (std::size_t k = 0; k < specs_.size(); ++k) {
        const ServoSpec& s = specs_[k];
        const ServoParams& p = s.params;
        const In& in = in_[k];
        const Out& o = out_[k];
        State& st = state_[k];
        using OperatingMode = ServoParams::OperatingMode;

        // Commands: a change on the bus (a ROS topic, ros2_control) or one given through the
        // API, the latest wins. A pulse goes through the servo's own map.
        const auto seen = [&](SignalId id, double& last, double& into) {
            const double value = bus.read(id);
            if (!same(value, last)) {
                last = value;
                if (std::isfinite(value)) {
                    into = value;
                    return true;
                }
            }
            return false;
        };
        double value = 0.0;
        double pulse = kNaN;
        const double previousEnable = st.enable;
        if (p.mode == OperatingMode::Position) {
            seen(in.goalPosition, st.busGoalPosition, st.goal);
            if (seen(in.pulseWidth, st.busPulseWidth, pulse)) {
                // No pulse (0) and a hobby servo lets go, as it does when the signal stops.
                st.goal = pulse > 0.0 ? pulseToAngle(p, pulse) : st.goal;
                st.enable = pulse > 0.0 ? 1.0 : 0.0;
            }
            if (fresh && commands_->take(s.name + "/position", value)) {
                st.goal = value;
            }
        } else if (p.mode == OperatingMode::Velocity) {
            seen(in.goalVelocity, st.busGoalVelocity, st.goal);
            if (fresh && commands_->take(s.name + "/velocity", value)) {
                st.goal = value;
            }
        } else {
            seen(in.goalCurrent, st.busGoalCurrent, st.goal);
            if (fresh && commands_->take(s.name + "/current", value)) {
                st.goal = value;
            }
        }
        const double enableOnBus = bus.read(in.torqueEnable);
        if (!same(enableOnBus, st.busEnable)) {
            st.busEnable = enableOnBus;
            st.enable = enableOnBus;
        }
        if (fresh && commands_->take(s.name + "/torque_enable", value)) {
            st.enable = value;
        }
        if (previousEnable <= 0.5 && st.enable > 0.5) {
            st.shutdown = Shutdown{};  // switched off and on: the shutdown is cleared
        }
        if (mode_ == Mode::Hold) {
            st.goal = p.mode == OperatingMode::Position ? s.start : 0.0;
            st.enable = 1.0;
        }
        if (p.mode == OperatingMode::Position && s.lower < s.upper) {
            st.goal = std::clamp(st.goal, s.lower, s.upper);
        }
        const bool torqueOn = mode_ != Mode::Off && st.enable > 0.5 && !st.shutdown.tripped();

        const double position = bus.read(in.position);
        const double velocity = bus.read(in.velocity);
        if (p.mode == OperatingMode::Position) {
            if (torqueOn) {
                stepProfile(st.profile, st.goal, p.profileVelocity, p.profileAcceleration, dt);
            } else {
                st.profile = Profile{position, 0.0};  // it starts again from where it is
            }
        }

        double command = 0.0;
        bool saturated = false;
        double load = 0.0;
        if (s.drive == ServoSpec::Drive::Position) {
            command = torqueOn ? st.profile.position : kNaN;
            // The drive's duty is not on the bus: saturated when the proportional term alone
            // asks for full duty, or the current is at the drive's limit.
            const double current = bus.read(in.current);
            const double error =
                deadband(st.profile.position - quantise(position, p.resolution), p.deadband);
            saturated = torqueOn && (std::abs(p.kp * error) >= 1.0 ||
                                     (s.armature.maxCurrent > 0.0 &&
                                      std::abs(current) >= kAtLimit * s.armature.maxCurrent));
            load = bus.read(in.torque);
        } else {
            const double supply =
                s.drive == ServoSpec::Drive::Voltage ? bus.read(in.busVoltage) : 1.0;
            double uMin = -1.0;
            double uMax = 1.0;
            if (s.drive == ServoSpec::Drive::Voltage) {
                currentLimitedDuty(s.armature, supply, velocity, uMin, uMax);
            }
            double u = 0.0;
            if (!torqueOn) {
                st.loop = LoopState{};
            } else if (p.mode == OperatingMode::Position) {
                u = positionDuty(p, st.loop, st.profile.position, st.profile.velocity, position,
                                 velocity, dt, uMin, uMax);
            } else if (p.mode == OperatingMode::Velocity) {
                u = velocityDuty(p, st.loop, st.goal, velocity, dt, uMin, uMax);
            } else {
                u = std::clamp(currentDuty(s.armature, supply, st.goal, velocity), uMin, uMax);
            }
            saturated = torqueOn && u != 0.0 && (u <= uMin || u >= uMax);
            if (s.drive == ServoSpec::Drive::Voltage) {
                command = u * supply;
                load = bus.read(in.torque);
            } else {
                command = torqueOn ? idealTorque(p, u, velocity) : 0.0;
                load = command;
            }
        }
        const double temperature = in.temperature.valid() ? bus.read(in.temperature) : kAmbientK;
        if (p.kind == ServoParams::Kind::Smart && mode_ == Mode::Full) {
            stepShutdown(p, st.shutdown, saturated, temperature, dt);
        }
        st.lastTorque = load;
        bus.write(o.command, command);
        bus.write(o.presentPosition, quantise(position, p.resolution));
        bus.write(o.presentVelocity, velocity);
        bus.write(o.presentLoad, load);
        bus.write(o.goal, p.mode == OperatingMode::Position ? st.profile.position : st.goal);
        bus.write(o.overload, st.shutdown.tripped() ? 1.0 : 0.0);
        if (o.presentCurrent.valid()) {
            bus.write(o.presentCurrent, bus.read(in.current));
            bus.write(o.temperature, temperature);
        }
    }
    return {};
}

// Field by field: the structs' padding is not state (and would differ between runs).
Status Servos::saveState(StateWriter& writer) const {
    writer.put(static_cast<std::uint64_t>(state_.size()));
    for (const State& s : state_) {
        for (const double v :
             {s.goal, s.busGoalPosition, s.busPulseWidth, s.busGoalVelocity, s.busGoalCurrent,
              s.enable, s.busEnable, s.lastTorque, s.profile.position, s.profile.velocity,
              s.loop.integral, s.shutdown.saturatedFor}) {
            writer.put(v);
        }
        writer.put(static_cast<std::uint8_t>((s.shutdown.overload ? 1 : 0) |
                                             (s.shutdown.overheated ? 2 : 0)));
    }
    return {};
}

Status Servos::restoreState(StateReader& reader) {
    std::uint64_t count = 0;
    if (!reader.get(count) || count != state_.size()) {
        return Status::error(StatusCode::Internal, "actuators: the state is of other servos");
    }
    for (State& s : state_) {
        std::uint8_t flags = 0;
        bool ok = true;
        for (double* v :
             {&s.goal, &s.busGoalPosition, &s.busPulseWidth, &s.busGoalVelocity, &s.busGoalCurrent,
              &s.enable, &s.busEnable, &s.lastTorque, &s.profile.position, &s.profile.velocity,
              &s.loop.integral, &s.shutdown.saturatedFor}) {
            ok = ok && reader.get(*v);
        }
        if (!ok || !reader.get(flags)) {
            return Status::error(StatusCode::Internal, "actuators: short servo state");
        }
        s.shutdown.overload = (flags & 1U) != 0;
        s.shutdown.overheated = (flags & 2U) != 0;
    }
    return {};
}

}  // namespace lagrange::actuators
