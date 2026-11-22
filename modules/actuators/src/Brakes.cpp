#include "lagrange/actuators/Brakes.hpp"

#include <algorithm>
#include <cmath>

namespace lagrange::actuators {

namespace {

/// The release and dropout currents as shares of the rated current, when not given: a drawing
/// default of the order datasheets give for pull-in and drop-out voltages, TODO(verify).
constexpr double kReleaseShare = 0.7;
constexpr double kDropoutShare = 0.3;

SignalSpec scalar(std::string name, Unit unit) {
    SignalSpec s;
    s.name = std::move(name);
    s.type = SignalType::Scalar;
    s.unit = unit;
    return s;
}

bool same(double a, double b) {
    return a == b || (std::isnan(a) && std::isnan(b));
}

}  // namespace

std::vector<BrakeSpec> brakesOf(const WorldView& world, std::string& error) {
    std::vector<BrakeSpec> out;
    const World::Joints& joints = world.joints();
    for (const SchemaInstance* s : world.instancesOf("BrakeAPI_1")) {
        BrakeSpec spec;
        spec.prim = s->prim;
        spec.name = std::string(world.signalInstanceName(s->prim));
        const auto fail = [&](const std::string& why) {
            error = "brake " + spec.name + ": " + why;
            return std::vector<BrakeSpec>{};
        };
        const auto j = std::find(joints.prim.begin(), joints.prim.end(), s->prim);
        if (j == joints.prim.end()) {
            return fail("BrakeAPI_1 must be on a joint");
        }
        const auto index = static_cast<std::size_t>(j - joints.prim.begin());
        const JointType type = joints.type[index];
        if (type != JointType::Revolute && type != JointType::Continuous &&
            type != JointType::Prismatic) {
            return fail("a brake holds a revolute, continuous or prismatic joint");
        }
        spec.joint = joints.names[index];
        spec.slide = type == JointType::Prismatic;
        BrakeParams& p = spec.params;
        const char* holding = spec.slide ? "act:brake:holdingForce" : "act:brake:holdingTorque";
        const char* dynamic = spec.slide ? "act:brake:dynamicForce" : "act:brake:dynamicTorque";
        p.holding = s->real(holding).value_or(0.0);
        if (!(p.holding > 0.0)) {
            return fail(std::string(holding) + " must be given and over 0");
        }
        p.dynamic = s->real(dynamic).value_or(p.holding);
        p.coilResistance = s->real("act:brake:coilResistance").value_or(0.0);
        p.coilInductance = s->real("act:brake:coilInductance").value_or(0.0);
        p.ratedVoltage = s->real("act:brake:ratedVoltage").value_or(0.0);
        if (!(p.coilResistance > 0.0) || !(p.ratedVoltage > 0.0)) {
            return fail("act:brake:coilResistance and ratedVoltage must be given and over 0");
        }
        if (!(p.dynamic >= 0.0) || !(p.coilInductance >= 0.0)) {
            return fail("its dynamic torque and coil inductance must not be negative");
        }
        const double full = p.ratedVoltage / p.coilResistance;
        p.releaseCurrent = s->real("act:brake:releaseCurrent").value_or(kReleaseShare * full);
        p.dropoutCurrent = s->real("act:brake:dropoutCurrent").value_or(kDropoutShare * full);
        if (!(p.dropoutCurrent >= 0.0) || !(p.dropoutCurrent < p.releaseCurrent) ||
            !(p.releaseCurrent <= full)) {
            return fail(
                "act:brake:dropoutCurrent must be under releaseCurrent, and that at most "
                "the rated current ratedVoltage / coilResistance");
        }
        // One writer of the joint's friction limit: a lead screw's motor writes it too.
        for (const SchemaInstance* motor : world.instancesOf("ElectricalMotorAPI_1")) {
            const auto driven = motor->rel("elec:joint");
            if (driven.size() == 1 && driven[0] == s->prim &&
                motor->real("elec:gearBackEfficiency").value_or(-1.0) >= 0.0) {
                return fail(
                    "its joint's motor has elec:gearBackEfficiency, whose friction limit "
                    "the brake's would clash with");
            }
        }
        out.push_back(std::move(spec));
    }
    return out;
}

ModuleInfo Brakes::info() const {
    const char* name = mode_ == Mode::Full    ? "brakes"
                       : mode_ == Mode::Ideal ? "brakes_ideal"
                       : mode_ == Mode::Hold  ? "brakes_constant"
                                              : "brakes_zero";
    return {"actuators", name, 1, 0, mode_ != Mode::Full};
}

Duration Brakes::preferredStep() const {
    return Duration{1'000'000};
}

Status Brakes::declare(const WorldView& world, const ModuleConfig&, SignalDeclarer& d) {
    std::string error;
    specs_ = brakesOf(world, error);
    if (!error.empty()) {
        return Status::error(StatusCode::InvalidConfig, error);
    }
    in_.assign(specs_.size(), In{});
    out_.assign(specs_.size(), Out{});
    for (std::size_t k = 0; k < specs_.size(); ++k) {
        const BrakeSpec& s = specs_[k];
        const std::string joint = "joint/" + s.joint + "/";
        const std::string act = "act/" + s.name + "/";
        const Unit effort = s.slide ? units::N : units::N_m;
        SignalSpec velocity =
            scalar(joint + "velocity", s.slide ? units::m_per_s : units::rad_per_s);
        velocity.feedback = true;
        in_[k].velocity = d.input(velocity);
        SignalSpec release = scalar(act + "release", units::one);
        release.optional = true;
        release.defaultValue = 0.0;
        in_[k].release = d.input(release);
        Out& o = out_[k];
        o.friction = d.output(scalar(joint + "friction_limit", effort));
        o.current = d.output(scalar(act + "current", units::A));
        o.voltage = d.output(scalar(act + "voltage", units::V));
        o.power = d.output(scalar(act + "power", units::W));
        SignalSpec engaged = scalar(act + "engaged", units::one);
        engaged.type = SignalType::Bool;
        o.engaged = d.output(engaged);
        o.torque = d.output(scalar(act + "torque", effort));
    }
    return {};
}

Status Brakes::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    state_.assign(specs_.size(), State{});
    for (std::size_t k = 0; k < specs_.size(); ++k) {
        const Out& o = out_[k];
        State& st = state_[k];
        st.brake.engaged = mode_ != Mode::Off;
        const double friction = brakeFriction(specs_[k].params, st.brake, 0.0);
        bus.write(o.friction, friction);
        bus.write(o.torque, friction);
        bus.write(o.engaged, st.brake.engaged ? 1.0 : 0.0);
        for (const SignalId id : {o.current, o.voltage, o.power}) {
            bus.write(id, 0.0);
        }
    }
    seenGeneration_ = commands_ ? commands_->generation() : 0;
    return {};
}

Status Brakes::step(const StepContext& context, SignalBus& bus) {
    const double dt = context.dt.seconds();
    const bool fresh = commands_ && commands_->generation() != seenGeneration_;
    if (fresh) {
        seenGeneration_ = commands_->generation();
    }
    for (std::size_t k = 0; k < specs_.size(); ++k) {
        const BrakeSpec& s = specs_[k];
        const BrakeParams& p = s.params;
        State& st = state_[k];
        const Out& o = out_[k];
        // A change on the bus (a ROS topic, a GPIO) or one through the API: the latest wins.
        const double onBus = bus.read(in_[k].release);
        if (!same(onBus, st.busCommand)) {
            st.busCommand = onBus;
            if (std::isfinite(onBus)) {
                st.command = onBus;
            }
        }
        double value = 0.0;
        if (fresh && commands_->take(s.name, value) && std::isfinite(value)) {
            st.command = value;
        }
        const double voltage = mode_ == Mode::Off || mode_ == Mode::Hold
                                   ? 0.0
                                   : std::clamp(st.command, 0.0, 1.0) * p.ratedVoltage;
        if (mode_ == Mode::Ideal) {
            st.brake.current = voltage / p.coilResistance;
            st.brake.engaged = st.brake.current < p.releaseCurrent;
        } else {
            stepBrake(p, st.brake, voltage, dt);
        }
        if (mode_ == Mode::Off) {
            st.brake.engaged = false;
        } else if (mode_ == Mode::Hold) {
            st.brake.engaged = true;
        }
        const double friction = brakeFriction(p, st.brake, bus.read(in_[k].velocity));
        bus.write(o.friction, friction);
        bus.write(o.torque, friction);
        bus.write(o.current, st.brake.current);
        bus.write(o.voltage, voltage);
        bus.write(o.power, voltage * st.brake.current);
        bus.write(o.engaged, st.brake.engaged ? 1.0 : 0.0);
    }
    return {};
}

Status Brakes::saveState(StateWriter& writer) const {
    writer.put(static_cast<std::uint64_t>(state_.size()));
    for (const State& s : state_) {
        writer.put(s.brake.current);
        writer.put(static_cast<std::uint8_t>(s.brake.engaged ? 1 : 0));
        writer.put(s.command);
        writer.put(s.busCommand);
    }
    return {};
}

Status Brakes::restoreState(StateReader& reader) {
    std::uint64_t count = 0;
    if (!reader.get(count) || count != state_.size()) {
        return Status::error(StatusCode::Internal, "actuators: the state is of other brakes");
    }
    for (State& s : state_) {
        std::uint8_t engaged = 0;
        if (!reader.get(s.brake.current) || !reader.get(engaged) || !reader.get(s.command) ||
            !reader.get(s.busCommand)) {
            return Status::error(StatusCode::Internal, "actuators: short brake state");
        }
        s.brake.engaged = engaged != 0;
    }
    return {};
}

}  // namespace lagrange::actuators
