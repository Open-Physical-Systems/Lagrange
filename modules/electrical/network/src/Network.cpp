// The electrical domain at the averaged fidelity; rules on the class in Network.hpp.
#include "lagrange/electrical/Network.hpp"
#include "lagrange/services/Log.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <optional>
#include <string_view>

namespace lagrange::electrical {

using namespace literals;

namespace {

constexpr std::string_view kSource = "electrical/averaged_network";
constexpr double kAmbientK = 298.15;  // K: the temperature inputs' default (25 °C), as the schemas'
                                      // referenceTemperature fallback
// A bus without elec:bus:capacitance gets this, with a warning (electrical.md §2).
constexpr double kDefaultBusCapacitanceF = 1e-6;
// How far an authored backEmfConstant may differ from the one the torque constant implies
// before the two are taken to be in different conventions (RMS, line-line): datasheets round.
constexpr double kBackEmfTolerance = 0.03;
// elec:velocityLoopBandwidth's fallback, rad/s: 10 Hz (illustrative, as the schema says).
constexpr double kVelocityLoopRadS = 62.83;
// A tripped drive resets once its command is this small (N·m): the controller asks for nothing.
constexpr double kResetTorqueNm = 1e-6;
// Ports followed from a component to its bus.
constexpr int kMaxPortHops = 4;

/// The inertia a joint turns at rest, about its axis (kg·m²): its armature and its child
/// body's inertia about the axis through the body origin (axis in the child frame, as URDF
/// writes it). The child's own children are left out: an estimate for the startup check.
double jointInertia(const WorldView& world, std::size_t j) {
    const World::Joints& joints = world.joints();
    const World::Bodies& bodies = world.bodies();
    const std::size_t b = joints.child[j].index;
    if (joints.type[j] == JointType::Prismatic) {  // what it moves: the child's mass
        return joints.armature[j] + (b < bodies.mass.size() ? bodies.mass[b] : 0.0);
    }
    const std::array<double, 3>& a = joints.axis[j];
    const double norm = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (b >= bodies.mass.size() || norm <= 0.0) {
        return joints.armature[j];
    }
    const std::array<double, 3> u{a[0] / norm, a[1] / norm, a[2] / norm};
    const std::array<double, 6>& i = bodies.inertiaLocal[b];  // xx yy zz xy xz yz, at the COM
    const double aboutCom = u[0] * u[0] * i[0] + u[1] * u[1] * i[1] + u[2] * u[2] * i[2] +
                            2.0 * (u[0] * u[1] * i[3] + u[0] * u[2] * i[4] + u[1] * u[2] * i[5]);
    const std::array<double, 3>& c = bodies.comLocal[b];
    const double along = c[0] * u[0] + c[1] * u[1] + c[2] * u[2];
    const double offAxis = c[0] * c[0] + c[1] * c[1] + c[2] * c[2] - along * along;
    return joints.armature[j] + aboutCom + bodies.mass[b] * offAxis;
}

/// Collects "prim: problem" lines for one error status at the end of declare.
struct Problems {
    std::vector<std::string> lines;
    void add(std::string_view where, const std::string& what) {
        lines.push_back(std::string(where) + ": " + what);
    }
    [[nodiscard]] Status status() const {
        if (lines.empty()) {
            return {};
        }
        std::string message = "the electrical network cannot be built:";
        for (const std::string& line : lines) {
            message += "\n  " + line;
        }
        return Status::error(StatusCode::InvalidConfig, message);
    }
};

SignalSpec scalar(std::string name, Unit unit) {
    SignalSpec spec;
    spec.name = std::move(name);
    spec.unit = unit;
    return spec;
}

SignalSpec optionalInput(std::string name, Unit unit, double fallback) {
    SignalSpec spec = scalar(std::move(name), unit);
    spec.optional = true;
    spec.defaultValue = fallback;
    spec.feedback = true;
    return spec;
}

}  // namespace

std::map<std::string, std::string> driveModes(const WorldView& world) {
    std::map<std::string, std::string> out;
    for (const SchemaInstance* s : world.instancesOf("ElectricalMotorAPI_1")) {
        const std::string mode(s->token("elec:controlMode").value_or("torque"));
        const auto joint = s->rel("elec:joint");
        if (mode != "torque" && joint.size() == 1) {
            out[std::string(world.signalInstanceName(joint[0]))] = mode;
        }
    }
    return out;
}

ModuleInfo ElectricalNetwork::info() const {
    return {"electrical", "averaged_network", 0, 1, false, true, CapabilityTier::Tier0};
}

Duration ElectricalNetwork::preferredStep() const {
    return 100_us;
}

Status ElectricalNetwork::declare(const WorldView& world, const ModuleConfig& config,
                                  SignalDeclarer& d) {
    motors_.clear();
    batteries_.clear();
    buses_.clear();
    loads_.clear();
    converters_.clear();
    ideal_.clear();
    predict_ = false;
    exchangeSteps_ = 0;
    const std::optional<double> exchange = config.get<double>("exchange_step");
    const double electricalStep = config.dt().value_or(preferredStep()).seconds();
    if (exchange && *exchange > 0.0) {
        exchangeSteps_ =
            static_cast<std::uint32_t>(std::max(1.0, std::round(*exchange / electricalStep)));
        predict_ = config.get<bool>("speed_prediction").value_or(true);
    }
    const bool fmuFidelity = config.fidelity() == "fmu";
    if (!config.fidelity().empty() && config.fidelity() != "averaged" && !fmuFidelity) {
        return Status::error(StatusCode::InvalidConfig,
                             "electrical fidelity " + std::string(config.fidelity()) +
                                 " is not available; this module runs averaged or fmu");
    }
    Problems problems;
    const auto name = [&world](PrimId prim) { return std::string(world.signalInstanceName(prim)); };
    const auto path = [&world](PrimId prim) { return std::string(world.primPath(prim)); };

    // Buses first: everything else connects to one.
    std::map<std::uint32_t, std::size_t> busOfPrim;
    for (const SchemaInstance* s : world.instancesOf("ElectricalBusAPI_1")) {
        Bus bus;
        bus.name = name(s->prim);
        const auto uv = s->real("elec:bus:undervoltageTrip");
        const auto ov = s->real("elec:bus:overvoltageTrip");
        if (!uv || !ov) {
            problems.add(path(s->prim),
                         "a bus needs elec:bus:undervoltageTrip and overvoltageTrip");
            continue;
        }
        bus.undervoltage = *uv;
        bus.overvoltage = *ov;
        bus.brakeResistance = s->real("elec:bus:brakeResistor").value_or(0.0);
        if (const auto c = s->real("elec:bus:capacitance"); c && *c > 0.0) {
            bus.capacitance = *c;
        } else {
            bus.capacitance = kDefaultBusCapacitanceF;
            services::log(services::LogLevel::Warning, kSource,
                          path(s->prim) + " has no elec:bus:capacitance; using 1 µF");
        }
        busOfPrim.emplace(s->prim.index, buses_.size());
        buses_.push_back(std::move(bus));
    }
    // A component's bus: the prim itself, or where its ports connect, a few hops at most.
    std::map<std::uint32_t, std::vector<const SchemaInstance*>> portsOfPrim;
    for (const SchemaInstance* s : world.instancesOf("PortAPI_1")) {
        portsOfPrim[s->prim.index].push_back(s);
    }
    const auto busOf = [&](PrimId prim) -> std::optional<std::size_t> {
        for (int hop = 0; hop < kMaxPortHops; ++hop) {
            if (const auto found = busOfPrim.find(prim.index); found != busOfPrim.end()) {
                return found->second;
            }
            const auto ports = portsOfPrim.find(prim.index);
            if (ports == portsOfPrim.end()) {
                return std::nullopt;
            }
            std::optional<PrimId> next;
            for (const SchemaInstance* port : ports->second) {
                const auto targets = port->rel("elec:port:" + port->instance + ":connectedTo");
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
    };
    const auto busThrough = [&](const SchemaInstance* s,
                                const std::string& rel) -> std::optional<std::size_t> {
        const auto targets = s->rel(rel);
        const auto bus = targets.empty() ? std::nullopt : busOf(targets[0]);
        if (!bus) {
            problems.add(path(s->prim), rel + " must lead to an ElectricalBusAPI_1 prim");
        }
        return bus;
    };

    for (const SchemaInstance* s : world.instancesOf("BatteryAPI_1")) {
        Battery battery;
        battery.name = name(s->prim);
        BatteryParams& p = battery.params;
        const auto series = s->integer("elec:battery:seriesCells");
        const auto capacity = s->real("elec:battery:capacity");
        const auto r0 = s->real("elec:battery:r0");
        const auto cutoff = s->real("elec:battery:cutoffVoltage");
        const auto charge = s->real("elec:battery:maxChargeCurrent");
        p.ocvSoc.assign(s->realArray("elec:battery:ocvSoc").begin(),
                        s->realArray("elec:battery:ocvSoc").end());
        p.ocvVoltage.assign(s->realArray("elec:battery:ocvVoltage").begin(),
                            s->realArray("elec:battery:ocvVoltage").end());
        if (!series || *series < 1 || !capacity || *capacity <= 0.0 || !r0 || *r0 <= 0.0 ||
            !cutoff || !charge || p.ocvSoc.empty() ||
            !std::is_sorted(p.ocvSoc.begin(), p.ocvSoc.end()) ||
            std::adjacent_find(p.ocvSoc.begin(), p.ocvSoc.end()) != p.ocvSoc.end()) {
            problems.add(path(s->prim),
                         "a battery needs seriesCells, capacity, r0 > 0, cutoffVoltage, "
                         "maxChargeCurrent and an ascending ocvSoc curve");
            continue;
        }
        p.seriesCells = static_cast<int>(*series);
        p.parallelCells =
            std::max(1, static_cast<int>(s->integer("elec:battery:parallelCells").value_or(1)));
        p.capacity = *capacity;
        p.r0 = *r0;
        p.r1 = s->real("elec:battery:r1").value_or(0.0);
        p.c1 = s->real("elec:battery:c1").value_or(0.0);
        p.r2 = s->real("elec:battery:r2").value_or(0.0);
        p.c2 = s->real("elec:battery:c2").value_or(0.0);
        p.referenceTemperature = s->real("elec:battery:referenceTemperature").value_or(kAmbientK);
        p.r0TempCoeff = s->real("elec:battery:r0TempCoeff").value_or(0.0);
        p.cutoffVoltage = *cutoff;
        p.maxChargeCurrent = *charge;
        battery.initialSoc = s->real("elec:battery:initialSoc").value_or(1.0);
        const auto bus = busOf(s->prim);
        if (!bus) {
            problems.add(path(s->prim),
                         "the battery's port (PortAPI_1 connectedTo) must lead to an "
                         "ElectricalBusAPI_1 prim");
            continue;
        }
        battery.bus = *bus;
        batteries_.push_back(std::move(battery));
    }

    const World::Joints& joints = world.joints();
    std::map<std::string, bool, std::less<>> motorJoint;
    fmuMotors_.clear();
    for (const SchemaInstance* s : world.instancesOf("ElectricalMotorAPI_1")) {
        if (fmuFidelity && s->assetPath("elec:behaviorModel")) {
            // Its FMU simulates it; the network only feeds its bus.
            const auto joint = s->rel("elec:joint");
            const auto bus = busThrough(s, "elec:powerPort");
            if (joint.size() != 1 || !bus) {
                problems.add(path(s->prim), "a motor needs one elec:joint and a bus");
                continue;
            }
            motorJoint[name(joint[0])] = true;
            fmuMotors_.push_back({name(s->prim), *bus, {}});
            continue;
        }
        Motor motor;
        motor.name = name(s->prim);
        const std::string where = path(s->prim);
        MotorParams& p = motor.params;
        const auto r = s->real("elec:windingResistance");
        const auto l = s->real("elec:inductance");
        const auto kt = s->real("elec:torqueConstant");
        const auto imax = s->real("elec:maxCurrent");
        const auto joint = s->rel("elec:joint");
        if (!r || !l || !kt || *kt <= 0.0 || !imax || *imax <= 0.0 || joint.size() != 1) {
            problems.add(where,
                         "a motor needs windingResistance, inductance, torqueConstant > 0, "
                         "maxCurrent > 0 and one elec:joint");
            continue;
        }
        const std::string_view type = s->token("elec:motorType").value_or("pmsm");
        p.dq = type != "dc_brushed";
        p.resistance = *r;
        p.inductanceD = *l;
        p.inductanceQ = p.dq ? s->real("elec:inductanceQ").value_or(*l) : *l;
        p.torqueConstant = *kt;
        p.polePairs = static_cast<int>(s->integer("elec:polePairs").value_or(1));
        p.gearRatio = s->real("elec:gearRatio").value_or(1.0);
        p.gearEfficiency = s->real("elec:gearEfficiency").value_or(1.0);
        p.maxCurrent = *imax;
        p.referenceTemperature = s->real("elec:referenceTemperature").value_or(kAmbientK);
        p.resistanceTempCoeff = s->real("elec:resistanceTempCoeff").value_or(0.0);
        p.currentLoopBandwidth = s->real("elec:currentLoopBandwidth").value_or(6283.0);
        if (p.polePairs < 1 || p.gearRatio <= 0.0 || p.gearEfficiency <= 0.0) {
            problems.add(where, "polePairs, gearRatio and gearEfficiency must be positive");
            continue;
        }
        if (const auto ke = s->real("elec:backEmfConstant")) {
            const double implied = p.backEmfConstant();
            if (std::abs(*ke - implied) > kBackEmfTolerance * implied) {
                problems.add(where, "backEmfConstant " + std::to_string(*ke) +
                                        " V·s/rad does not match torqueConstant (it implies " +
                                        std::to_string(implied) +
                                        " in the peak dq convention: Ke = Kt/1.5 for pmsm and "
                                        "bldc, Ke = Kt for dc_brushed); one of them is in "
                                        "another convention (RMS or line-line?)");
                continue;
            }
        }
        motor.mode = std::string(s->token("elec:controlMode").value_or("torque"));
        motor.velocityBandwidth = s->real("elec:velocityLoopBandwidth").value_or(kVelocityLoopRadS);
        if (motor.velocityBandwidth <= 0.0) {
            problems.add(where, "elec:velocityLoopBandwidth must be positive");
            continue;
        }
        motor.joint = name(joint[0]);
        const auto j = std::find(joints.names.begin(), joints.names.end(), motor.joint);
        if (j == joints.names.end()) {
            problems.add(where, "elec:joint " + path(joint[0]) + " is not a joint of the robot");
            continue;
        }
        motor.slide =
            joints.type[static_cast<std::size_t>(j - joints.names.begin())] == JointType::Prismatic;
        const auto bus = busThrough(s, "elec:powerPort");
        if (!bus) {
            continue;
        }
        if (p.currentLoopBandwidth * electricalStep > 1.0) {
            services::log(services::LogLevel::Warning, kSource,
                          where + ": the current loop's bandwidth (" +
                              std::to_string(p.currentLoopBandwidth) +
                              " rad/s) is above what the electrical step resolves; it runs at " +
                              std::to_string(1.0 / electricalStep) + " rad/s. Use a smaller dt");
        }
        motor.bus = *bus;
        motor.worldInertia =
            jointInertia(world, static_cast<std::size_t>(j - joints.names.begin()));
        if (exchange && *exchange > 0.0 && motor.worldInertia > 0.0) {
            // Mechanical time constant at the joint: J·R / ((N·Kt)·(N·Ke)).
            const double n = p.gearRatio;
            const double tau = motor.worldInertia * p.resistance /
                               (n * n * p.torqueConstant * p.backEmfConstant());
            const double ratio = *exchange / tau;
            const double limit = predict_ ? kMaxStepRatioPredicted : kMaxStepRatio;
            if (ratio > limit) {
                problems.add(where, "the physics step " + std::to_string(*exchange * 1e3) +
                                        " ms is " + std::to_string(ratio) +
                                        " mechanical time constants of this motor (τm = " +
                                        std::to_string(tau * 1e3) + " ms; at most " +
                                        std::to_string(limit) +
                                        (predict_ ? "" : " without speed prediction") +
                                        "): the exchange would be unstable. Use a physics dt "
                                        "below " +
                                        std::to_string(limit * tau * 1e3) + " ms" +
                                        (predict_ ? "" : ", or turn speed_prediction on"));
                continue;
            }
        }
        motorJoint[motor.joint] = true;
        motors_.push_back(std::move(motor));
    }
    for (const SchemaInstance* s : world.instancesOf("ElectricalLoadAPI_1")) {
        Load load;
        load.name = name(s->prim);
        const auto power = s->real("elec:load:power");
        const std::string_view model = s->token("elec:load:model").value_or("constant_power");
        if (!power) {
            problems.add(path(s->prim), "a load needs elec:load:power");
            continue;
        }
        load.power = *power;
        load.resistive = model == "resistive";
        load.minVoltage = s->real("elec:load:minVoltage").value_or(0.0);
        const auto bus = busThrough(s, "elec:powerPort");
        if (!bus) {
            continue;
        }
        load.bus = *bus;
        loads_.push_back(std::move(load));
    }
    for (const SchemaInstance* s : world.instancesOf("ElectricalConverterAPI_1")) {
        Converter converter;
        converter.name = name(s->prim);
        const auto voltage = s->real("elec:conv:outputVoltage");
        const auto current = s->real("elec:conv:maxOutputCurrent");
        converter.efficiency = s->real("elec:conv:efficiency").value_or(1.0);
        if (!voltage || *voltage <= 0.0 || !current || *current <= 0.0 ||
            converter.efficiency <= 0.0 || converter.efficiency > 1.0) {
            problems.add(path(s->prim),
                         "a converter needs outputVoltage > 0, maxOutputCurrent > 0 and an "
                         "efficiency in (0, 1]");
            continue;
        }
        converter.outputVoltage = *voltage;
        converter.maxCurrent = *current;
        const auto in = busThrough(s, "elec:conv:input");
        const auto out = busThrough(s, "elec:conv:output");
        if (!in || !out) {
            continue;
        }
        if (*in == *out) {
            problems.add(path(s->prim), "a converter's input and output are the same bus");
            continue;
        }
        const bool shared = std::ranges::any_of(
            converters_, [&](const Converter& other) { return other.output == *out; });
        if (shared) {
            problems.add(path(s->prim), "two converters feed bus " + buses_[*out].name +
                                            "; one converter per output bus is simulated");
            continue;
        }
        converter.input = *in;
        converter.output = *out;
        converters_.push_back(std::move(converter));
    }
    if (Status status = problems.status(); !status.ok()) {
        return status;
    }

    // Signals (electrical.md §3).
    for (Motor& m : motors_) {
        const std::string joint = "joint/" + m.joint + "/";
        const std::string motor = "motor/" + m.name + "/";
        const Unit effort = m.slide ? units::N : units::N_m;
        const Unit speed = m.slide ? units::m_per_s : units::rad_per_s;
        if (m.mode == "velocity") {
            m.velocityCommand = d.input(scalar(joint + "velocity_command", speed));
        } else if (m.mode == "voltage") {
            m.voltageCommand = d.input(scalar(joint + "voltage_command", units::V));
        } else {
            // A drive nothing commands makes no torque (a robot without a controller).
            SignalSpec command = scalar(joint + "torque_command", effort);
            command.optional = true;
            m.command = d.input(command);
        }
        SignalSpec velocity = scalar(joint + "velocity", speed);
        // Predicting, the drive integrates between samples itself and needs the sample as is.
        velocity.hold = predict_ ? Hold::ZeroOrder : Hold::LinearExtrapolate;
        velocity.feedback = true;
        m.velocity = d.input(velocity);
        m.effectiveInertia = d.input(
            optionalInput(joint + "effective_inertia", m.slide ? units::kg : units::kg_m2, 0.0));
        m.temperature =
            d.input(optionalInput("thermal/" + m.name + "/temperature", units::K, kAmbientK));
        m.derating = d.input(optionalInput("thermal/" + m.name + "/derating", units::one, 1.0));
        m.torque = d.output(scalar(joint + "torque", effort));
        m.currentD = d.output(scalar(motor + "current_d", units::A));
        m.currentQ = d.output(scalar(motor + "current_q", units::A));
        m.voltageD = d.output(scalar(motor + "voltage_d", units::V));
        m.voltageQ = d.output(scalar(motor + "voltage_q", units::V));
        m.busCurrent = d.output(scalar(motor + "bus_current", units::A));
        m.loss = d.output(scalar(motor + "loss_power", units::W));
        m.fault = d.output(scalar(motor + "fault", units::one));
    }
    for (FmuMotor& m : fmuMotors_) {
        SignalSpec current = scalar("motor/" + m.name + "/bus_current", units::A);
        current.feedback = true;  // its FMU reads the bus voltage the network writes
        m.busCurrent = d.input(current);
    }
    for (Battery& b : batteries_) {
        const std::string battery = "battery/" + b.name + "/";
        b.temperature =
            d.input(optionalInput("thermal/" + b.name + "/temperature", units::K, kAmbientK));
        b.soc = d.output(scalar(battery + "soc", units::one));
        b.terminalVoltage = d.output(scalar(battery + "terminal_voltage", units::V));
        b.currentOut = d.output(scalar(battery + "current", units::A));
        b.loss = d.output(scalar(battery + "loss_power", units::W));
        b.power = d.output(scalar(battery + "power", units::W));  // at its terminals
    }
    for (Bus& bus : buses_) {
        bus.voltageOut = d.output(scalar("bus/" + bus.name + "/voltage", units::V));
        bus.brownoutOut = d.output(scalar("bus/" + bus.name + "/brownout", units::one));
    }
    for (Converter& c : converters_) {
        const std::string converter = "converter/" + c.name + "/";
        c.outputCurrent = d.output(scalar(converter + "output_current", units::A));
        c.inputCurrent = d.output(scalar(converter + "input_current", units::A));
        c.loss = d.output(scalar(converter + "loss_power", units::W));
    }
    for (Load& l : loads_) {
        SignalSpec demand = scalar("load/" + l.name + "/power_demand", units::W);
        demand.optional = true;
        demand.defaultValue = l.power;
        l.demand = d.input(demand);
        l.currentOut = d.output(scalar("load/" + l.name + "/current", units::A));
        l.poweredOut = d.output(scalar("load/" + l.name + "/powered", units::one));
    }
    // Commands for joints without a motor are delivered ideally, as the substitute does.
    constexpr std::string_view kCommand = "/torque_command";
    for (const SignalSpec& command : d.declaredOutputs("joint/")) {
        const std::string_view text = command.name;
        if (!text.ends_with(kCommand)) {
            continue;
        }
        const std::string joint(text.substr(6, text.size() - 6 - kCommand.size()));
        if (motorJoint.contains(joint)) {
            continue;
        }
        SignalSpec out = command;
        out.name = "joint/" + joint + "/torque";
        ideal_.push_back({joint, d.input(command), d.output(out)});
    }
    return {};
}

Status ElectricalNetwork::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    batteryTemperature_.assign(batteries_.size(), kAmbientK);
    converterInput_.assign(converters_.size(), 0.0);
    limitedScratch_.assign(batteries_.size(), false);
    for (Battery& b : batteries_) {
        b.state = BatteryState{};
        b.state.soc = std::clamp(b.initialSoc, 0.0, 1.0);
        b.state.cut = b.params.openCircuitVoltage(b.state.soc) <= b.params.cutoffVoltage;
        b.current = 0.0;
    }
    // Each bus starts at its batteries' open-circuit voltage, as after a long rest.
    for (std::size_t i = 0; i < buses_.size(); ++i) {
        Bus& node = buses_[i];
        double conductance = 0.0;
        double injected = 0.0;
        for (const Battery& b : batteries_) {
            if (b.bus == i && !b.state.cut) {
                const Thevenin t = theveninOf(b.params, b.state, kAmbientK);
                conductance += 1.0 / t.resistance;
                injected += t.emf / t.resistance;
            }
        }
        node.voltage = conductance > 0.0 ? injected / conductance : 0.0;
        node.brownout = node.voltage < node.undervoltage;
        node.overvoltageTripped = false;
    }
    // A converter's output bus starts regulated when its input is up (input buses first: a
    // chain of converters is in declaration order).
    for (Converter& c : converters_) {
        c.current = 0.0;
        const Bus& in = buses_[c.input];
        Bus& out = buses_[c.output];
        if (!in.brownout && in.voltage > 0.0) {
            out.voltage = std::max(out.voltage, c.outputVoltage);
            out.brownout = out.voltage < out.undervoltage;
        }
        for (const SignalId id : {c.outputCurrent, c.inputCurrent, c.loss}) {
            bus.write(id, 0.0);
        }
    }
    for (const Bus& node : buses_) {
        bus.write(node.voltageOut, node.voltage);
        bus.write(node.brownoutOut, node.brownout ? 1.0 : 0.0);
    }
    for (Load& l : loads_) {
        // A resistive load draws its power at the voltage its bus starts at.
        const double v = buses_[l.bus].voltage;
        l.resistance = v > 0.0 && l.power > 0.0 ? v * v / l.power : 0.0;
        bus.write(l.currentOut, 0.0);
        bus.write(l.poweredOut, 0.0);
    }
    for (Motor& m : motors_) {
        m.state = MotorState{};
        m.tripped = false;
        m.predicted = m.loadEstimate = m.lastSample = m.torqueSum = 0.0;
        m.stepsSince = 0;
        m.synced = false;
        for (const SignalId id : {m.torque, m.currentD, m.currentQ, m.voltageD, m.voltageQ,
                                  m.busCurrent, m.loss, m.fault}) {
            bus.write(id, 0.0);
        }
    }
    for (const Battery& b : batteries_) {
        bus.write(b.soc, b.state.soc);
        bus.write(b.terminalVoltage, buses_[b.bus].voltage);
        bus.write(b.currentOut, 0.0);
        bus.write(b.loss, 0.0);
        bus.write(b.power, 0.0);
    }
    for (const Passthrough& p : ideal_) {
        bus.write(p.torque, 0.0);
    }
    return {};
}

void ElectricalNetwork::solveBus(std::size_t b, double dt, std::vector<double>& temperatures) {
    Bus& node = buses_[b];
    // Node equation with the capacitor's backward-Euler companion (conductance C/dt):
    //   (C/dt + Σ 1/R_i + G_brake)·V = C/dt·V_old + Σ E_i/R_i − I_sink
    // A battery that would charge faster than its limit becomes a current source at the limit.
    const double companion = node.capacitance / dt;
    std::vector<bool>& limited = limitedScratch_;  // sized in init: no allocation here
    std::fill(limited.begin(), limited.end(), false);
    double voltage = node.voltage;
    Converter* feeder = nullptr;  // the converter regulating this bus, if any and running
    for (Converter& c : converters_) {
        if (c.output == b) {
            c.current = 0.0;
            const Bus& in = buses_[c.input];
            feeder = !in.brownout && in.voltage > 0.0 ? &c : nullptr;
        }
    }
    for (std::size_t pass = 0; pass <= batteries_.size(); ++pass) {
        double conductance = companion;
        double injected = companion * node.voltage - node.sink;
        if (node.overvoltageTripped && node.brakeResistance > 0.0) {
            conductance += 1.0 / node.brakeResistance;
        }
        for (std::size_t i = 0; i < batteries_.size(); ++i) {
            const Battery& battery = batteries_[i];
            if (battery.bus != b || battery.state.cut) {
                continue;
            }
            if (limited[i]) {
                injected -= battery.params.maxChargeCurrent;
                continue;
            }
            const Thevenin t = theveninOf(battery.params, battery.state, temperatures[i]);
            conductance += 1.0 / t.resistance;
            injected += t.emf / t.resistance;
        }
        voltage = std::max(0.0, injected / conductance);
        if (feeder != nullptr && voltage < feeder->outputVoltage) {
            // The current that holds the set point, within the converter's limit.
            const double needed = conductance * feeder->outputVoltage - injected;
            feeder->current = std::min(needed, feeder->maxCurrent);
            voltage = needed <= feeder->maxCurrent
                          ? feeder->outputVoltage
                          : std::max(0.0, (injected + feeder->maxCurrent) / conductance);
        } else if (feeder != nullptr) {
            feeder->current = 0.0;
        }
        bool changed = false;
        for (std::size_t i = 0; i < batteries_.size(); ++i) {
            Battery& battery = batteries_[i];
            if (battery.bus != b || battery.state.cut) {
                continue;
            }
            if (limited[i]) {
                battery.current = -battery.params.maxChargeCurrent;
                continue;
            }
            const Thevenin t = theveninOf(battery.params, battery.state, temperatures[i]);
            battery.current = (t.emf - voltage) / t.resistance;
            if (battery.current < -battery.params.maxChargeCurrent) {
                limited[i] = true;
                changed = true;
            }
        }
        if (!changed) {
            break;
        }
    }
    node.voltage = voltage;
}

Status ElectricalNetwork::step(const StepContext& context, SignalBus& bus) {
    const double dt = context.dt.seconds();
    for (Bus& node : buses_) {
        node.sink = 0.0;
    }
    for (Motor& m : motors_) {
        const Bus& node = buses_[m.bus];
        MotorInputs in;
        if (m.mode == "torque") {
            in.torqueCommand = bus.read(m.command);
        }
        const double sample = bus.read(m.velocity);
        if (predict_) {
            // Resync to physics every H; estimate the load from the last window:
            // J·Δω/H = mean drive torque − load.
            const double inertia =
                bus.read(m.effectiveInertia) > 0.0 ? bus.read(m.effectiveInertia) : m.worldInertia;
            if (!m.synced || m.stepsSince >= exchangeSteps_) {
                if (m.synced && m.stepsSince > 0 && inertia > 0.0) {
                    const double window = m.stepsSince * dt;
                    m.loadEstimate =
                        m.torqueSum / m.stepsSince - inertia * (sample - m.lastSample) / window;
                }
                m.predicted = sample;
                m.lastSample = sample;
                m.torqueSum = 0.0;
                m.stepsSince = 0;
                m.synced = true;
            }
            in.jointVelocity = m.predicted;
            in.inertia = inertia;
            in.loadTorque = m.loadEstimate;
        } else {
            in.jointVelocity = sample;
        }
        // A velocity or voltage drive resets once it is asked for nothing, as a torque one does.
        double asked = in.torqueCommand;
        if (m.mode == "velocity") {
            asked = bus.read(m.velocityCommand);
        } else if (m.mode == "voltage") {
            asked = bus.read(m.voltageCommand);
        }
        if (node.brownout) {
            m.tripped = true;
        } else if (m.tripped && std::abs(asked) < kResetTorqueNm) {
            m.tripped = false;
        }
        if (m.mode == "voltage") {
            in.voltageMode = true;
            in.voltageCommand = asked;
        } else if (m.mode == "velocity") {
            // PI on the inertia the joint turns: J s² + Kp s + Ki with both poles at the bandwidth
            // over 2, so Kp = J ω, Ki = J ω² / 4. Held by what the drive can give.
            const double effective = bus.read(m.effectiveInertia);
            const double inertia = effective > 0.0 ? effective : m.worldInertia;
            const double kp = inertia * m.velocityBandwidth;
            const double ki = kp * m.velocityBandwidth / 4.0;
            const double limit = m.params.gearRatio * m.params.gearEfficiency *
                                 m.params.torqueConstant * m.params.maxCurrent *
                                 std::clamp(bus.read(m.derating), 0.0, 1.0);
            const double error = asked - in.jointVelocity;
            const double wanted = kp * error + m.velocityIntegral;
            if (m.tripped) {
                m.velocityIntegral = 0.0;
            } else if (std::abs(wanted) < limit) {  // anti-windup: hold it at the limit
                m.velocityIntegral += ki * error * dt;
            }
            in.torqueCommand = std::clamp(wanted, -limit, limit);
        }
        const bool overvoltageOff = node.overvoltageTripped && node.brakeResistance <= 0.0;
        const bool off = m.tripped || overvoltageOff;
        in.busVoltage = node.voltage;
        in.temperature = bus.read(m.temperature);
        in.derating = bus.read(m.derating);
        in.enabled = !off;
        const MotorOutputs out = stepMotor(m.params, m.state, in, dt);
        if (predict_) {
            const double inertia =
                bus.read(m.effectiveInertia) > 0.0 ? bus.read(m.effectiveInertia) : m.worldInertia;
            m.torqueSum += out.jointTorque;
            ++m.stepsSince;
            if (inertia > 0.0) {
                m.predicted = out.jointVelocity;
            }
        }
        buses_[m.bus].sink += out.busCurrent;
        bus.write(m.torque, out.jointTorque);
        bus.write(m.currentD, m.state.id);
        bus.write(m.currentQ, m.state.iq);
        bus.write(m.voltageD, out.vd);
        bus.write(m.voltageQ, out.vq);
        bus.write(m.busCurrent, out.busCurrent);
        bus.write(m.loss, out.lossPower);
        bus.write(m.fault, m.tripped ? 1.0 : (overvoltageOff ? 2.0 : 0.0));
    }
    for (const FmuMotor& m : fmuMotors_) {
        buses_[m.bus].sink += bus.read(m.busCurrent);
    }
    for (Load& l : loads_) {
        Bus& node = buses_[l.bus];
        const bool powered = !node.brownout && node.voltage >= l.minVoltage && node.voltage > 0.0;
        double current = 0.0;
        if (powered) {
            current = l.resistive && l.resistance > 0.0
                          ? node.voltage / l.resistance
                          : std::max(0.0, bus.read(l.demand)) / node.voltage;
        }
        node.sink += current;
        bus.write(l.currentOut, current);
        bus.write(l.poweredOut, powered ? 1.0 : 0.0);
    }
    // A converter draws its last step's output power over its efficiency, as a load does.
    std::fill(converterInput_.begin(), converterInput_.end(), 0.0);
    for (std::size_t k = 0; k < converters_.size(); ++k) {
        const Converter& c = converters_[k];
        const Bus& in = buses_[c.input];
        if (in.voltage > 0.0 && !in.brownout) {
            converterInput_[k] = buses_[c.output].voltage * c.current / c.efficiency / in.voltage;
            buses_[c.input].sink += converterInput_[k];
        }
    }
    for (std::size_t i = 0; i < batteries_.size(); ++i) {
        batteryTemperature_[i] = bus.read(batteries_[i].temperature);
        batteries_[i].current = 0.0;
    }
    for (std::size_t b = 0; b < buses_.size(); ++b) {
        solveBus(b, dt, batteryTemperature_);
        Bus& node = buses_[b];
        if (node.voltage < node.undervoltage) {
            node.brownout = true;
        } else if (node.voltage >= node.undervoltage * (1.0 + kRecoveryFraction)) {
            node.brownout = false;
        }
        if (node.voltage > node.overvoltage) {
            node.overvoltageTripped = true;
        } else if (node.voltage <= node.overvoltage * (1.0 - kRecoveryFraction)) {
            node.overvoltageTripped = false;
        }
        bus.write(node.voltageOut, node.voltage);
        bus.write(node.brownoutOut, node.brownout ? 1.0 : 0.0);
    }
    for (std::size_t i = 0; i < batteries_.size(); ++i) {
        Battery& b = batteries_[i];
        const BatteryParams& p = b.params;
        const Thevenin t = theveninOf(p, b.state, batteryTemperature_[i]);
        // Loss in the series resistance and in the RC pairs' resistors (per cell × cells).
        const double cells = static_cast<double>(p.seriesCells) * p.parallelCells;
        double loss = b.current * b.current * t.resistance;
        if (p.r1 > 0.0 && p.c1 > 0.0) {
            loss += cells * b.state.v1 * b.state.v1 / p.r1;
        }
        if (p.r2 > 0.0 && p.c2 > 0.0) {
            loss += cells * b.state.v2 * b.state.v2 / p.r2;
        }
        if (!b.state.cut) {
            advanceBattery(p, b.state, b.current, dt);
        }
        bus.write(b.soc, b.state.soc);
        bus.write(b.terminalVoltage, b.state.cut ? 0.0 : buses_[b.bus].voltage);
        bus.write(b.currentOut, b.current);
        bus.write(b.loss, loss);
        bus.write(b.power, b.state.cut ? 0.0 : buses_[b.bus].voltage * b.current);
    }
    for (std::size_t k = 0; k < converters_.size(); ++k) {
        const Converter& c = converters_[k];
        const double out = buses_[c.output].voltage * c.current;
        bus.write(c.outputCurrent, c.current);
        bus.write(c.inputCurrent, converterInput_[k]);
        bus.write(c.loss, out * (1.0 / c.efficiency - 1.0));
    }
    for (const Passthrough& p : ideal_) {
        bus.write(p.torque, bus.read(p.command));
    }
    return {};
}

Status ElectricalNetwork::saveState(StateWriter& writer) const {
    for (const Motor& m : motors_) {
        writer.put(m.state);
        writer.put(m.tripped);
        writer.put(m.velocityIntegral);
        writer.put(m.predicted);
        writer.put(m.loadEstimate);
        writer.put(m.lastSample);
        writer.put(m.torqueSum);
        writer.put(m.stepsSince);
        writer.put(m.synced);
    }
    for (const Battery& b : batteries_) {
        writer.put(b.state.soc);
        writer.put(b.state.v1);
        writer.put(b.state.v2);
        writer.put(b.state.cut);
    }
    for (const Bus& node : buses_) {
        writer.put(node.voltage);
        writer.put(node.brownout);
        writer.put(node.overvoltageTripped);
    }
    for (const Converter& c : converters_) {
        writer.put(c.current);
    }
    return {};
}

Status ElectricalNetwork::restoreState(StateReader& reader) {
    bool ok = true;
    for (Motor& m : motors_) {
        ok = ok && reader.get(m.state) && reader.get(m.tripped) && reader.get(m.velocityIntegral) &&
             reader.get(m.predicted) && reader.get(m.loadEstimate) && reader.get(m.lastSample) &&
             reader.get(m.torqueSum) && reader.get(m.stepsSince) && reader.get(m.synced);
    }
    for (Battery& b : batteries_) {
        ok = ok && reader.get(b.state.soc) && reader.get(b.state.v1) && reader.get(b.state.v2) &&
             reader.get(b.state.cut);
    }
    for (Bus& node : buses_) {
        ok = ok && reader.get(node.voltage) && reader.get(node.brownout) &&
             reader.get(node.overvoltageTripped);
    }
    for (Converter& c : converters_) {
        ok = ok && reader.get(c.current);
    }
    return ok ? Status{} : Status::error(StatusCode::InvalidConfig, "electrical state truncated");
}

std::vector<std::string> ElectricalNetwork::motors() const {
    std::vector<std::string> out;
    for (const Motor& m : motors_) {
        out.push_back(m.name);
    }
    return out;
}

std::vector<std::string> ElectricalNetwork::buses() const {
    std::vector<std::string> out;
    for (const Bus& b : buses_) {
        out.push_back(b.name);
    }
    return out;
}

std::vector<std::string> ElectricalNetwork::idealJoints() const {
    std::vector<std::string> out;
    for (const Passthrough& p : ideal_) {
        out.push_back(p.joint);
    }
    return out;
}

std::vector<FmuBinding> motorFmuBindings(const WorldView& world) {
    std::vector<FmuBinding> out;
    std::map<std::uint32_t, std::vector<const SchemaInstance*>> portsOfPrim;
    for (const SchemaInstance* s : world.instancesOf("PortAPI_1")) {
        portsOfPrim[s->prim.index].push_back(s);
    }
    std::map<std::uint32_t, bool> isBus;
    for (const SchemaInstance* s : world.instancesOf("ElectricalBusAPI_1")) {
        isBus[s->prim.index] = true;
    }
    for (const SchemaInstance* s : world.instancesOf("ElectricalMotorAPI_1")) {
        const auto model = s->assetPath("elec:behaviorModel");
        const auto joint = s->rel("elec:joint");
        const auto power = s->rel("elec:powerPort");
        if (!model || joint.size() != 1 || power.empty()) {
            continue;
        }
        // The bus: the power port's target, or where its port connects (a few hops).
        PrimId bus = power[0];
        for (int hop = 0; hop < kMaxPortHops && !isBus.contains(bus.index); ++hop) {
            const auto ports = portsOfPrim.find(bus.index);
            if (ports == portsOfPrim.end()) {
                break;
            }
            for (const SchemaInstance* port : ports->second) {
                const auto targets = port->rel("elec:port:" + port->instance + ":connectedTo");
                if (!targets.empty()) {
                    bus = targets[0];
                    break;
                }
            }
        }
        FmuBinding binding;
        binding.fmuPath = std::string(*model);
        binding.name = std::string(world.signalInstanceName(s->prim));
        binding.placeholders = {{"joint", std::string(world.signalInstanceName(joint[0]))},
                                {"bus", std::string(world.signalInstanceName(bus))},
                                {"prim", binding.name}};
        for (const auto& [key, value] : s->reals) {
            binding.attributes.emplace(key, value);
        }
        out.push_back(std::move(binding));
    }
    return out;
}

void applyRotorInertia(World& world) {
    const WorldView view(world);
    for (const SchemaInstance* s : view.instancesOf("ElectricalMotorAPI_1")) {
        const auto joint = s->rel("elec:joint");
        const double rotor = s->real("elec:rotorInertia").value_or(0.0);
        const double n = s->real("elec:gearRatio").value_or(1.0);
        if (joint.size() != 1 || rotor <= 0.0) {
            continue;
        }
        const std::string_view name = view.signalInstanceName(joint[0]);
        for (std::size_t j = 0; j < world.joints.names.size(); ++j) {
            if (world.joints.names[j] == name) {
                world.joints.armature[j] += n * n * rotor;
            }
        }
    }
}

}  // namespace lagrange::electrical
