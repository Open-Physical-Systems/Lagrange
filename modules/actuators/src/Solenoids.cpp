#include "lagrange/actuators/Solenoids.hpp"

#include <algorithm>
#include <cmath>

namespace lagrange::actuators {

namespace {

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

std::vector<SolenoidSpec> solenoidsOf(const WorldView& world, std::string& error) {
    std::vector<SolenoidSpec> out;
    const World::Joints& joints = world.joints();
    for (const SchemaInstance* s : world.instancesOf("SolenoidAPI_1")) {
        SolenoidSpec spec;
        spec.prim = s->prim;
        spec.name = std::string(world.signalInstanceName(s->prim));
        const auto fail = [&](const std::string& why) {
            error = "solenoid " + spec.name + ": " + why;
            return std::vector<SolenoidSpec>{};
        };
        const auto j = std::find(joints.prim.begin(), joints.prim.end(), s->prim);
        if (j == joints.prim.end()) {
            return fail("SolenoidAPI_1 must be on a joint");
        }
        const auto index = static_cast<std::size_t>(j - joints.prim.begin());
        if (joints.type[index] != JointType::Prismatic) {
            return fail("a solenoid's plunger slides: SolenoidAPI_1 goes on a prismatic joint");
        }
        spec.joint = joints.names[index];
        SolenoidParams& p = spec.params;
        const std::span<const double> strokes = s->realArray("act:solenoid:inductanceStrokes");
        const std::span<const double> values = s->realArray("act:solenoid:inductances");
        if (strokes.size() != values.size()) {
            return fail("act:solenoid:inductanceStrokes and inductances must be as long");
        }
        p.position.assign(strokes.begin(), strokes.end());
        p.inductance.assign(values.begin(), values.end());
        p.resistance = s->real("act:solenoid:resistance").value_or(0.0);
        p.springRate = s->real("act:solenoid:springRate").value_or(0.0);
        p.springPreload = s->real("act:solenoid:springPreload").value_or(0.0);
        p.stroke = s->real("act:solenoid:stroke").value_or(0.0);
        p.ratedVoltage = s->real("act:solenoid:ratedVoltage").value_or(0.0);
        if (std::string why = checkSolenoid(p); !why.empty()) {
            return fail(why);
        }
        out.push_back(std::move(spec));
    }
    return out;
}

ModuleInfo Solenoids::info() const {
    const char* name = mode_ == Mode::Full    ? "solenoids"
                       : mode_ == Mode::Ideal ? "solenoids_ideal"
                                              : "solenoids_zero";
    return {"actuators", name, 1, 0, mode_ != Mode::Full};
}

Duration Solenoids::preferredStep() const {
    return Duration{100'000};  // 100 µs: a small coil's L/R is a millisecond or two
}

Status Solenoids::declare(const WorldView& world, const ModuleConfig&, SignalDeclarer& d) {
    std::string error;
    specs_ = solenoidsOf(world, error);
    if (!error.empty()) {
        return Status::error(StatusCode::InvalidConfig, error);
    }
    in_.assign(specs_.size(), In{});
    out_.assign(specs_.size(), Out{});
    for (std::size_t k = 0; k < specs_.size(); ++k) {
        const SolenoidSpec& s = specs_[k];
        const std::string joint = "joint/" + s.joint + "/";
        const std::string act = "act/" + s.name + "/";
        SignalSpec position = scalar(joint + "position", units::m);
        position.feedback = true;
        in_[k].position = d.input(position);
        SignalSpec command = scalar(act + "command", units::one);
        command.optional = true;
        command.defaultValue = 0.0;
        in_[k].command = d.input(command);
        Out& o = out_[k];
        o.force = d.output(scalar(joint + "torque_command", units::N));
        o.current = d.output(scalar(act + "current", units::A));
        o.voltage = d.output(scalar(act + "voltage", units::V));
        o.pull = d.output(scalar(act + "force", units::N));
        o.plunger = d.output(scalar(act + "position", units::m));
        o.power = d.output(scalar(act + "power", units::W));
    }
    return {};
}

Status Solenoids::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    state_.assign(specs_.size(), State{});
    for (std::size_t k = 0; k < specs_.size(); ++k) {
        const Out& o = out_[k];
        bus.write(o.force, solenoidSpring(specs_[k].params, 0.0));
        for (const SignalId id : {o.current, o.voltage, o.pull, o.plunger, o.power}) {
            bus.write(id, 0.0);
        }
    }
    seenGeneration_ = commands_ ? commands_->generation() : 0;
    return {};
}

Status Solenoids::step(const StepContext& context, SignalBus& bus) {
    const double dt = context.dt.seconds();
    const bool fresh = commands_ && commands_->generation() != seenGeneration_;
    if (fresh) {
        seenGeneration_ = commands_->generation();
    }
    for (std::size_t k = 0; k < specs_.size(); ++k) {
        const SolenoidSpec& s = specs_[k];
        const SolenoidParams& p = s.params;
        State& st = state_[k];
        const Out& o = out_[k];
        // A change on the bus (a ROS topic) or one through the API: the latest wins.
        const double onBus = bus.read(in_[k].command);
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
        const double duty = mode_ == Mode::Off ? 0.0 : std::clamp(st.command, 0.0, 1.0);
        const double voltage = duty * p.ratedVoltage;
        const double x = std::clamp(bus.read(in_[k].position), 0.0, p.stroke);
        double current = 0.0;
        if (mode_ == Mode::Ideal) {
            current = voltage / p.resistance;
            st.flux = current * solenoidInductance(p, x);
        } else {
            st.flux = stepSolenoidFlux(p, st.flux, voltage, x, dt);
            current = st.flux / solenoidInductance(p, x);
        }
        const double pull = solenoidPull(p, current, x);
        bus.write(o.force, pull + solenoidSpring(p, x));
        bus.write(o.current, current);
        bus.write(o.voltage, voltage);
        bus.write(o.pull, pull);
        bus.write(o.plunger, x);
        bus.write(o.power, voltage * current);
    }
    return {};
}

Status Solenoids::saveState(StateWriter& writer) const {
    writer.put(static_cast<std::uint64_t>(state_.size()));
    for (const State& s : state_) {
        writer.put(s.flux);
        writer.put(s.command);
        writer.put(s.busCommand);
    }
    return {};
}

Status Solenoids::restoreState(StateReader& reader) {
    std::uint64_t count = 0;
    if (!reader.get(count) || count != state_.size()) {
        return Status::error(StatusCode::Internal, "actuators: the state is of other solenoids");
    }
    for (State& s : state_) {
        if (!reader.get(s.flux) || !reader.get(s.command) || !reader.get(s.busCommand)) {
            return Status::error(StatusCode::Internal, "actuators: short solenoid state");
        }
    }
    return {};
}

}  // namespace lagrange::actuators
