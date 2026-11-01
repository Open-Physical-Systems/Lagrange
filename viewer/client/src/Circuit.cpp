// A power system as a circuit; see Circuit.hpp.
#include "lagrange/viewer/Circuit.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <optional>
#include <set>
#include <sstream>

namespace lagrange::viewer {

namespace {

const AttributeInfo* find(const PrimAttributes& prim, std::string_view name) {
    for (const AttributeInfo& a : prim.attributes) {
        if (a.name == name) {
            return &a;
        }
    }
    return nullptr;
}

double real(const PrimAttributes& prim, std::string_view name, double fallback = 0.0) {
    const AttributeInfo* a = find(prim, name);
    return a != nullptr && !a->numbers.empty() ? a->numbers[0] : fallback;
}

bool hasSchema(const PrimData& prim, std::string_view schema) {
    return std::find(prim.apiSchemas.begin(), prim.apiSchemas.end(), schema) !=
           prim.apiSchemas.end();
}

/// A pack's open-circuit voltage at its state of charge, and its curve when it is not flat.
double packVoltage(const PrimAttributes& prim, double soc, double cells, CircuitPart& part) {
    const AttributeInfo* socs = find(prim, "elec:battery:ocvSoc");
    const AttributeInfo* volts = find(prim, "elec:battery:ocvVoltage");
    if (socs == nullptr || volts == nullptr || volts->numbers.empty() ||
        socs->numbers.size() != volts->numbers.size()) {
        return 0.0;
    }
    const std::vector<double>& x = socs->numbers;
    const std::vector<double>& y = volts->numbers;
    const auto [low, high] = std::minmax_element(y.begin(), y.end());
    double cell = y.front();
    if (soc >= x.back()) {
        cell = y.back();
    } else if (soc > x.front()) {
        const auto upper = std::upper_bound(x.begin(), x.end(), soc);
        const auto k = static_cast<std::size_t>(upper - x.begin());
        cell = y[k - 1] + (soc - x[k - 1]) / (x[k] - x[k - 1]) * (y[k] - y[k - 1]);
    }
    if (*high > *low) {
        part.ocvSoc = x;
        part.ocvVolts.clear();
        for (const double v : y) {
            part.ocvVolts.push_back(v * cells);
        }
    }
    return cell * cells;
}

CircuitPart partOf(const SceneStatics& statics, const NetTerminal& terminal,
                   const PrimAttributes* attributes, const JointOf& jointOf) {
    CircuitPart part;
    const PrimData& prim = statics.prims[terminal.prim];
    part.name = prim.name;
    part.prim = terminal.prim;
    part.currentSignal = terminal.currentSignal;
    if (hasSchema(prim, "BatteryAPI_1")) {
        part.kind = CircuitPart::Kind::Battery;
    } else if (hasSchema(prim, "ElectricalMotorAPI_1")) {
        part.kind = CircuitPart::Kind::Motor;
    } else if (hasSchema(prim, "ElectricalLoadAPI_1")) {
        part.kind = CircuitPart::Kind::PowerLoad;
    } else if (hasSchema(prim, "ElectricalConverterAPI_1")) {
        part.kind = CircuitPart::Kind::Converter;
    }
    if (attributes == nullptr) {
        return part;
    }
    const PrimAttributes& a = *attributes;
    switch (part.kind) {
        case CircuitPart::Kind::Battery: {
            const double series = real(a, "elec:battery:seriesCells", 1.0);
            const double parallel = std::max(real(a, "elec:battery:parallelCells", 1.0), 1.0);
            const double scale = series / parallel;
            part.soc = real(a, "elec:battery:initialSoc", 1.0);
            part.voltage = packVoltage(a, part.soc, series, part);
            part.r0 = real(a, "elec:battery:r0") * scale;
            for (const AttributeInfo& attribute : a.attributes) {
                if (attribute.name.starts_with("elec:port:") &&
                    attribute.name.ends_with(":wireResistance") && !attribute.numbers.empty()) {
                    part.r0 += attribute.numbers[0];
                }
            }
            for (const auto& [r, c] : std::array{std::pair{"elec:battery:r1", "elec:battery:c1"},
                                                 std::pair{"elec:battery:r2", "elec:battery:c2"}}) {
                const double resistance = real(a, r);
                const double capacitance = real(a, c);
                if (resistance > 0.0 && capacitance > 0.0) {
                    part.pairs.emplace_back(resistance * scale, capacitance / scale);
                } else {
                    part.r0 += resistance * scale;  // a pair without capacitance is a resistance
                }
            }
            part.capacity = real(a, "elec:battery:capacity");
            part.cutoff = real(a, "elec:battery:cutoffVoltage");
            part.maxCharge = real(a, "elec:battery:maxChargeCurrent");
            break;
        }
        case CircuitPart::Kind::PowerLoad: {
            part.power = real(a, "elec:load:power");
            part.minVoltage = real(a, "elec:load:minVoltage");
            const AttributeInfo* model = find(a, "elec:load:model");
            if (model != nullptr && model->value == "resistive") {
                part.kind = CircuitPart::Kind::Resistor;  // its resistance needs the bus: below
            }
            break;
        }
        case CircuitPart::Kind::Motor: {
            if (const AttributeInfo* joint = find(a, "elec:joint");
                joint != nullptr && !joint->targets.empty() &&
                joint->targets[0] < statics.prims.size()) {
                part.joint = jointOf ? jointOf(joint->targets[0]) : std::string{};
                if (part.joint.empty()) {
                    part.joint = statics.prims[joint->targets[0]].name;
                }
            }
            if (const AttributeInfo* type = find(a, "elec:motorType"); type != nullptr) {
                part.motorType = type->value;
            }
            part.windingResistance = real(a, "elec:windingResistance");
            part.inductance = real(a, "elec:inductance");
            part.torqueConstant = real(a, "elec:torqueConstant");
            part.maxCurrent = real(a, "elec:maxCurrent");
            part.gear = real(a, "elec:gearRatio", 1.0);
            part.efficiency = real(a, "elec:gearEfficiency", 1.0);
            part.rotorInertia = real(a, "elec:rotorInertia");
            part.polePairs = static_cast<int>(real(a, "elec:polePairs", 1.0));
            break;
        }
        case CircuitPart::Kind::Converter: {
            part.outputVolts = real(a, "elec:conv:outputVoltage");
            part.maxOutput = real(a, "elec:conv:maxOutputCurrent");
            part.efficiency = real(a, "elec:conv:efficiency", 1.0);
            // On its output net it names where it draws from; on its input net it is only a load.
            if (terminal.isSource()) {
                if (const AttributeInfo* input = find(a, "elec:conv:input");
                    input != nullptr && !input->targets.empty()) {
                    for (const ElectricalNet& net : statics.nets) {
                        if (net.busPrim == input->targets[0]) {
                            part.inputBus = net.name;
                        }
                    }
                }
            }
            break;
        }
        default:
            break;
    }
    return part;
}

std::string number(double value) {
    std::array<char, 40> text{};
    std::snprintf(text.data(), text.size(), "%.10g", value);
    return text.data();
}

/// A name a netlist can carry, and that the importer gives back.
std::string identifier(std::string_view name) {
    std::string out;
    for (const char c : name) {
        out += std::isalnum(static_cast<unsigned char>(c)) != 0 ? c : '_';
    }
    return out;
}

std::string nodeName(std::string_view name) {
    std::string out = identifier(name);
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
}

}  // namespace

std::vector<std::uint32_t> circuitPrims(const SceneStatics& statics) {
    std::set<std::uint32_t> prims;
    for (const ElectricalNet& net : statics.nets) {
        if (net.busPrim < statics.prims.size()) {
            prims.insert(net.busPrim);
        }
        for (const NetTerminal& terminal : net.terminals) {
            if (terminal.prim < statics.prims.size()) {
                prims.insert(terminal.prim);
            }
        }
    }
    return {prims.begin(), prims.end()};
}

Circuit circuitOf(const SceneStatics& statics, const AttributesOf& attributes,
                  const JointOf& jointOf) {
    Circuit circuit;
    for (const ElectricalNet& net : statics.nets) {
        CircuitBus bus;
        bus.name = net.name;
        bus.prim = net.busPrim;
        bus.voltageSignal = net.voltageSignal;
        if (net.busPrim < statics.prims.size()) {
            if (const PrimAttributes* a = attributes(net.busPrim); a != nullptr) {
                bus.capacitance = real(*a, "elec:bus:capacitance");
                bus.undervoltage = real(*a, "elec:bus:undervoltageTrip");
                bus.overvoltage = real(*a, "elec:bus:overvoltageTrip");
            }
        }
        for (const NetTerminal& terminal : net.terminals) {
            if (terminal.prim < statics.prims.size()) {
                bus.parts.push_back(partOf(statics, terminal, attributes(terminal.prim), jointOf));
            }
        }
        const double start = startVoltage(bus);
        for (CircuitPart& part : bus.parts) {
            if (part.kind == CircuitPart::Kind::Resistor && part.power > 0.0) {
                part.resistance = start * start / part.power;
            }
        }
        circuit.buses.push_back(std::move(bus));
    }
    return circuit;
}

std::string engineering(double value, const char* unit) {
    static constexpr std::array<std::pair<double, const char*>, 7> kScales{
        {{1e6, "M"}, {1e3, "k"}, {1.0, ""}, {1e-3, "m"}, {1e-6, "µ"}, {1e-9, "n"}, {1e-12, "p"}}};
    double scale = 1.0;
    const char* prefix = "";
    if (value != 0.0) {
        for (const auto& [factor, name] : kScales) {
            scale = factor;
            prefix = name;
            if (std::abs(value) >= factor) {
                break;
            }
        }
    }
    std::array<char, 48> text{};
    std::snprintf(text.data(), text.size(), "%.4g %s%s", value / scale, prefix, unit);
    return text.data();
}

std::string partValues(const CircuitPart& part) {
    std::string out;
    switch (part.kind) {
        case CircuitPart::Kind::Battery:
            if (part.voltage > 0.0) {
                out = engineering(part.voltage, "V") + " · r0 " + engineering(part.r0, "Ω");
                for (const auto& [r, c] : part.pairs) {
                    out += " · " + engineering(r, "Ω") + "‖" + engineering(c, "F");
                }
            }
            break;
        case CircuitPart::Kind::Resistor:
            if (part.resistance > 0.0) {
                out = engineering(part.resistance, "Ω");
            }
            break;
        case CircuitPart::Kind::PowerLoad:
            if (part.power > 0.0) {
                out = engineering(part.power, "W") + " constant";
            }
            break;
        case CircuitPart::Kind::Converter:
            if (part.outputVolts > 0.0) {
                out =
                    engineering(part.outputVolts, "V") + " · efficiency " + number(part.efficiency);
            }
            break;
        case CircuitPart::Kind::Motor:
            if (part.torqueConstant > 0.0) {
                out = engineering(part.windingResistance, "Ω") + " · " +
                      engineering(part.inductance, "H") + " · ×" + number(part.gear);
            }
            break;
        default:
            break;
    }
    return out;
}

std::string busValues(const CircuitBus& bus) {
    std::string out;
    const auto add = [&out](const std::string& text) { out += (out.empty() ? "" : " · ") + text; };
    if (bus.undervoltage > 0.0) {
        add("UV " + engineering(bus.undervoltage, "V"));
    }
    if (bus.overvoltage > 0.0) {
        add("OV " + engineering(bus.overvoltage, "V"));
    }
    if (bus.capacitance > 0.0) {
        add("C " + engineering(bus.capacitance, "F"));
    }
    return out;
}

double startVoltage(const CircuitBus& bus) {
    for (const CircuitPart& part : bus.parts) {
        if (part.kind == CircuitPart::Kind::Converter && !part.inputBus.empty()) {
            return part.outputVolts;  // it regulates its output from the start
        }
    }
    double current = 0.0;
    double conductance = 0.0;
    for (const CircuitPart& part : bus.parts) {
        if (part.kind == CircuitPart::Kind::Battery && part.r0 > 0.0) {
            current += part.voltage / part.r0;
            conductance += 1.0 / part.r0;
        }
    }
    return conductance > 0.0 ? current / conductance : 0.0;
}

std::vector<std::string> circuitProblems(const Circuit& circuit) {
    std::vector<std::string> problems;
    std::set<std::string> names;
    const auto claim = [&](const std::string& name) {
        if (name.empty()) {
            problems.emplace_back("a part has no name");
        } else if (!names.insert(identifier(name)).second) {
            problems.push_back("the name " + name + " is used twice");
        }
    };
    if (circuit.buses.empty()) {
        problems.emplace_back("the circuit has no bus");
    }
    for (const CircuitBus& bus : circuit.buses) {
        claim(nodeName(bus.name));
        bool fed = false;
        if (bus.undervoltage <= 0.0 || bus.overvoltage <= bus.undervoltage) {
            problems.push_back("bus " + bus.name + ": set undervoltage and a higher overvoltage");
        }
        for (const CircuitPart& part : bus.parts) {
            // A converter is a terminal on both its buses; its input side is not a new name.
            const bool inputSide =
                part.kind == CircuitPart::Kind::Converter && part.inputBus.empty();
            if (!inputSide) {
                claim(part.name);
            }
            const std::string what = part.name + ": ";
            switch (part.kind) {
                case CircuitPart::Kind::Battery:
                    fed = true;
                    if (part.voltage <= 0.0 || part.r0 <= 0.0 || part.capacity <= 0.0 ||
                        part.maxCharge <= 0.0) {
                        problems.push_back(what + "set voltage, r0, capacity and max charge");
                    } else if (part.cutoff >= part.voltage) {
                        problems.push_back(what + "the cutoff must be below its voltage");
                    }
                    break;
                case CircuitPart::Kind::Resistor:
                    if (part.resistance <= 0.0) {
                        problems.push_back(what + "set its resistance");
                    }
                    break;
                case CircuitPart::Kind::PowerLoad:
                    if (part.power <= 0.0) {
                        problems.push_back(what + "set its power");
                    }
                    break;
                case CircuitPart::Kind::Converter:
                    if (!inputSide) {
                        fed = true;
                        if (part.outputVolts <= 0.0 || part.maxOutput <= 0.0 ||
                            part.efficiency <= 0.0 || part.efficiency > 1.0 ||
                            part.inputBus == bus.name) {
                            problems.push_back(what +
                                               "set its voltage, max current, efficiency up to 1 "
                                               "and another bus to draw from");
                        }
                    }
                    break;
                case CircuitPart::Kind::Motor:
                    if (part.joint.empty() || part.windingResistance <= 0.0 ||
                        part.inductance <= 0.0 || part.torqueConstant <= 0.0 ||
                        part.maxCurrent <= 0.0) {
                        problems.push_back(what + "set joint, R, L, Kt and max current");
                    }
                    break;
                default:
                    problems.push_back(what + "a netlist cannot hold this part");
                    break;
            }
        }
        if (!fed) {
            problems.push_back("bus " + bus.name + ": no battery feeds it");
        }
    }
    return problems;
}

std::string netlistOf(const Circuit& circuit, const std::string& title) {
    std::ostringstream out;
    std::ostringstream notes;
    out << title << "\n";
    for (const CircuitBus& bus : circuit.buses) {
        const std::string node = nodeName(bus.name);
        const double start = startVoltage(bus);
        out << "* bus " << bus.name << "\n";
        if (bus.capacitance > 0.0) {
            out << "C_" << node << " " << node << " 0 " << number(bus.capacitance)
                << " IC=" << number(start) << "\n";
        }
        notes << "*lagrange bus " << node << " undervoltage=" << number(bus.undervoltage)
              << " overvoltage=" << number(bus.overvoltage) << "\n";
        for (const CircuitPart& part : bus.parts) {
            const std::string id = identifier(part.name);
            switch (part.kind) {
                case CircuitPart::Kind::Battery: {
                    out << "V_" << id << " " << id << "_in 0 DC " << number(part.voltage) << "\n";
                    std::string at = id + "_in";
                    for (std::size_t i = 0; i < part.pairs.size(); ++i) {
                        const std::string next = id + "_rc" + std::to_string(i + 1);
                        out << "R_" << next << " " << at << " " << next << " "
                            << number(part.pairs[i].first) << "\n"
                            << "C_" << next << " " << at << " " << next << " "
                            << number(part.pairs[i].second) << " IC=0\n";
                        at = next;
                    }
                    out << "R_" << id << " " << at << " " << node << " " << number(part.r0) << "\n";
                    notes << "*lagrange battery V_" << id << " capacity=" << number(part.capacity)
                          << " cutoff=" << number(part.cutoff)
                          << " max_charge=" << number(part.maxCharge)
                          << " soc=" << number(part.soc);
                    if (!part.ocvSoc.empty()) {
                        const auto list = [&notes](const std::vector<double>& values) {
                            for (std::size_t i = 0; i < values.size(); ++i) {
                                notes << (i == 0 ? "" : ",") << number(values[i]);
                            }
                        };
                        notes << " ocv_soc=";
                        list(part.ocvSoc);
                        notes << " ocv=";
                        list(part.ocvVolts);
                    }
                    notes << "\n";
                    break;
                }
                case CircuitPart::Kind::Resistor:
                    out << "R_" << id << " " << node << " 0 " << number(part.resistance) << "\n";
                    break;
                case CircuitPart::Kind::PowerLoad:
                    out << "B_" << id << " " << node << " 0 I = " << number(part.power)
                        << " / max(V(" << node << "), 0.001) * ((V(" << node
                        << ") >= " << number(part.minVoltage) << ") ? 1 : 0)\n";
                    break;
                case CircuitPart::Kind::Converter:
                    if (!part.inputBus.empty()) {
                        const std::string in = nodeName(part.inputBus);
                        out << "V_" << id << " " << node << " 0 DC " << number(part.outputVolts)
                            << "\n"
                            << "B_" << id << "_in " << in << " 0 I = -I(V_" << id << ") * "
                            << number(part.outputVolts) << " / " << number(part.efficiency)
                            << " / max(V(" << in << "), 0.001) * ((V(" << in
                            << ") >= 0) ? 1 : 0)\n";
                        notes << "*lagrange converter " << id << " source=V_" << id << " in=" << in
                              << " max_current=" << number(part.maxOutput)
                              << " efficiency=" << number(part.efficiency) << " input_sink=B_" << id
                              << "_in\n";
                    }
                    break;
                case CircuitPart::Kind::Motor:
                    notes << "*lagrange motor " << part.joint << " node=" << node << " name=" << id
                          << " r=" << number(part.windingResistance)
                          << " l=" << number(part.inductance)
                          << " kt=" << number(part.torqueConstant)
                          << " imax=" << number(part.maxCurrent) << " gear=" << number(part.gear)
                          << " efficiency=" << number(part.efficiency)
                          << " pole_pairs=" << part.polePairs;
                    if (!part.motorType.empty()) {
                        notes << " type=" << part.motorType;
                    }
                    if (part.rotorInertia > 0.0) {
                        notes << " rotor_inertia=" << number(part.rotorInertia);
                    }
                    notes << "\n";
                    break;
                default:
                    out << "* " << part.name << " is left out: a netlist cannot hold it\n";
            }
        }
    }
    out << notes.str() << ".tran 1m 1 uic\n.end\n";
    return out.str();
}

}  // namespace lagrange::viewer
