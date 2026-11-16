// The lights domain (RFC 0021 §3); rules on the class in Lights.hpp.
#include "lagrange/lights/Lights.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace lagrange::lights {

namespace {

// A lamp counts as on (lamp/<n>/on) while its output is above this share of full: a lamp with
// a rise time never quite reaches zero.
constexpr double kOnShare = 0.01;

SignalSpec spec(std::string name, SignalType type, Unit unit) {
    SignalSpec s;
    s.name = std::move(name);
    s.type = type;
    s.unit = unit;
    return s;
}

SignalSpec optionalInput(std::string name, Unit unit, double fallback, bool feedback) {
    SignalSpec s = spec(std::move(name), SignalType::Scalar, unit);
    s.optional = true;
    s.defaultValue = fallback;
    s.feedback = feedback;
    return s;
}

}  // namespace

std::optional<Mode> Lights::substitute(std::string_view token) {
    if (token.empty() || token == "zero") {
        return Mode::Zero;
    }
    if (token == "constant") {
        return Mode::Constant;
    }
    return std::nullopt;
}

ModuleInfo Lights::info() const {
    return {"lights",
            mode_ == Mode::Model  ? "lamps"
            : mode_ == Mode::Zero ? "zero"
                                  : "constant",
            1, 0, mode_ != Mode::Model};
}

Duration Lights::preferredStep() const {
    return Duration{10'000'000};  // 10 ms: a 1.5 Hz indicator's edges within a step
}

Status Lights::declare(const WorldView& world, const ModuleConfig&, SignalDeclarer& d) {
    std::string error;
    specs_ = lampsOf(world, error);
    if (!error.empty()) {
        return Status::error(StatusCode::InvalidConfig, error);
    }
    io_.assign(specs_.size(), {});
    solidAngle_.clear();
    for (std::size_t k = 0; k < specs_.size(); ++k) {
        const LampSpec& lamp = specs_[k];
        Io& io = io_[k];
        solidAngle_.push_back(effectiveSolidAngle(lamp));
        const std::string base = "lamp/" + lamp.name + "/";
        if (mode_ == Mode::Model) {
            io.command = d.input(optionalInput(base + "command", units::one,
                                               std::clamp(lamp.initialCommand, 0.0, 1.0), false));
            io.blink = d.input(optionalInput(base + "blink", units::one, 0.0, false));
        }
        if (lamp.load && mode_ != Mode::Zero) {
            // The network reads what the lamp draws and tells it whether it is powered: read
            // its outputs from its previous step to break the loop.
            io.powered =
                d.input(optionalInput("load/" + lamp.name + "/powered", units::one, 1.0, true));
            if (!lamp.bus.empty() && lamp.voltageExponent > 0.0) {
                io.voltage =
                    d.input(optionalInput("bus/" + lamp.bus + "/voltage", units::V, 0.0, true));
                io.hasVoltage = true;
            }
        }
        io.intensity = d.output(spec(base + "intensity", SignalType::Scalar, units::cd));
        io.on = d.output(spec(base + "on", SignalType::Bool, units::one));
        io.level = d.output(spec(base + "level", SignalType::Scalar, units::one));
        if (lamp.efficacy > 0.0) {
            io.loss = d.output(spec(base + "loss_power", SignalType::Scalar, units::W));
            if (lamp.load) {
                io.demand = d.output(
                    spec("load/" + lamp.name + "/power_demand", SignalType::Scalar, units::W));
            }
        }
    }
    return {};
}

Status Lights::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    output_.assign(specs_.size(), 0.0);
    nominal_.assign(specs_.size(), 0.0);
    for (std::size_t k = 0; k < specs_.size(); ++k) {
        const double command =
            mode_ == Mode::Zero ? 0.0 : std::clamp(specs_[k].initialCommand, 0.0, 1.0);
        // Lit from the start, as a lamp left on before the run began.
        output_[k] = command * specs_[k].intensity;
        write(k, output_[k], command, bus);
    }
    return {};
}

void Lights::write(std::size_t k, double output, double command, SignalBus& bus) const {
    const LampSpec& lamp = specs_[k];
    const Io& io = io_[k];
    bus.write(io.intensity, output);
    bus.write(io.on, output > kOnShare * lamp.intensity ? 1.0 : 0.0);
    bus.write(io.level, output / lamp.intensity);
    if (io.loss.valid()) {
        // Drawn at the commanded flux even while the output ramps: no filament inrush.
        const double power = lamp.intensity * solidAngle_[k] * command / lamp.efficacy;
        bus.write(io.loss, lamp.heatFraction * power);
        if (io.demand.valid()) {
            bus.write(io.demand, power);
        }
    }
}

Status Lights::step(const StepContext& context, SignalBus& bus) {
    const double t = context.t.seconds();
    const double dt = context.dt.seconds();
    for (std::size_t k = 0; k < specs_.size(); ++k) {
        const LampSpec& lamp = specs_[k];
        const Io& io = io_[k];
        if (mode_ == Mode::Zero) {
            write(k, 0.0, 0.0, bus);
            continue;
        }
        double command = std::clamp(lamp.initialCommand, 0.0, 1.0);
        bool blinking = false;
        if (mode_ == Mode::Model) {
            const double asked = bus.read(io.command);
            command = std::isfinite(asked) ? std::clamp(asked, 0.0, 1.0) : 0.0;
            blinking = bus.read(io.blink) > 0.5;
        }
        if (blinking && !blinkOn(t, lamp.blinkFrequency, lamp.blinkDuty)) {
            command = 0.0;
        }
        if (io.powered.valid() && bus.read(io.powered) < 0.5) {
            command = 0.0;  // a brownout, or below the load's minVoltage
        }
        double scale = 1.0;
        if (io.hasVoltage) {
            const double v = bus.read(io.voltage);
            if (nominal_[k] <= 0.0 && v > 0.0) {
                nominal_[k] = v;
            }
            if (nominal_[k] > 0.0 && v < nominal_[k]) {
                scale = std::pow(std::max(v, 0.0) / nominal_[k], lamp.voltageExponent);
            }
        }
        const double target = command * lamp.intensity * scale;
        const double follow = lamp.riseTime > 0.0 ? 1.0 - std::exp(-dt / lamp.riseTime) : 1.0;
        output_[k] += (target - output_[k]) * follow;
        write(k, output_[k], command, bus);
    }
    return {};
}

Status Lights::saveState(StateWriter& writer) const {
    writer.putDoubles(output_);
    writer.putDoubles(nominal_);
    return {};
}

Status Lights::restoreState(StateReader& reader) {
    if (!reader.getDoubles(output_) || !reader.getDoubles(nominal_)) {
        return Status::error(StatusCode::Internal, "lights: short state");
    }
    return {};
}

}  // namespace lagrange::lights
