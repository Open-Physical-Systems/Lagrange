// The power system as an ngspice netlist or a Modelica model; rules in Export.hpp.
#include "lagrange/electrical/Export.hpp"
#include "lagrange/electrical/Models.hpp"

#include <cctype>
#include <cstdio>
#include <optional>
#include <sstream>

namespace lagrange::electrical {

namespace {

constexpr double kDefaultBusCapacitanceF = 1e-6;  // as the network (electrical.md §2)
constexpr double kMinVoltage = 1e-3;  // V: the sources' I = P/V divides by at least this
constexpr int kMaxPortHops = 4;

/// What the export needs of the electrical schemas.
struct Topology {
    struct Bus {
        std::string name;
        double capacitance{kDefaultBusCapacitanceF};
        double initialVoltage{0.0};
    };
    struct Battery {
        std::string name;
        BatteryParams params;
        double ocv{0.0};
        double wireResistance{0.0};
        std::size_t bus{0};
    };
    struct Load {
        std::string name;
        bool resistive{false};
        double power{0.0}, minVoltage{0.0};
        std::size_t bus{0};
    };
    struct Drive {
        std::string name;
        std::size_t bus{0};
    };
    std::vector<Bus> buses;
    std::vector<Battery> batteries;
    std::vector<Load> loads;
    std::vector<Drive> drives;
};

std::string identifier(std::string_view name) {
    std::string out;
    for (const char c : name) {
        out += std::isalnum(static_cast<unsigned char>(c)) != 0 ? c : '_';
    }
    return out;
}

std::string number(double value) {
    char text[32];
    std::snprintf(text, sizeof(text), "%.10g", value);
    return text;
}

Expected<Topology, std::string> readTopology(const WorldView& world) {
    Topology t;
    std::map<std::uint32_t, std::size_t> busOfPrim;
    for (const SchemaInstance* s : world.instancesOf("ElectricalBusAPI_1")) {
        Topology::Bus bus;
        bus.name = std::string(world.signalInstanceName(s->prim));
        if (const auto c = s->real("elec:bus:capacitance"); c && *c > 0.0) {
            bus.capacitance = *c;
        }
        busOfPrim.emplace(s->prim.index, t.buses.size());
        t.buses.push_back(std::move(bus));
    }
    if (t.buses.empty()) {
        return Unexpected{std::string("the stage has no ElectricalBusAPI_1 prims to export")};
    }
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
    const auto through = [&](const SchemaInstance* s) -> std::optional<std::size_t> {
        const auto targets = s->rel("elec:powerPort");
        return targets.empty() ? std::nullopt : busOf(targets[0]);
    };
    std::string problems;
    for (const SchemaInstance* s : world.instancesOf("BatteryAPI_1")) {
        Topology::Battery b;
        b.name = std::string(world.signalInstanceName(s->prim));
        BatteryParams& p = b.params;
        p.seriesCells = static_cast<int>(s->integer("elec:battery:seriesCells").value_or(0));
        p.parallelCells =
            std::max(1, static_cast<int>(s->integer("elec:battery:parallelCells").value_or(1)));
        p.ocvSoc.assign(s->realArray("elec:battery:ocvSoc").begin(),
                        s->realArray("elec:battery:ocvSoc").end());
        p.ocvVoltage.assign(s->realArray("elec:battery:ocvVoltage").begin(),
                            s->realArray("elec:battery:ocvVoltage").end());
        p.r0 = s->real("elec:battery:r0").value_or(0.0);
        p.r1 = s->real("elec:battery:r1").value_or(0.0);
        p.c1 = s->real("elec:battery:c1").value_or(0.0);
        p.r2 = s->real("elec:battery:r2").value_or(0.0);
        p.c2 = s->real("elec:battery:c2").value_or(0.0);
        const auto bus = busOf(s->prim);
        if (p.seriesCells < 1 || p.r0 <= 0.0 || p.ocvSoc.empty() || !bus) {
            problems += "\n  " + std::string(world.primPath(s->prim)) +
                        ": a battery needs seriesCells, r0, an OCV curve and a port to a bus";
            continue;
        }
        b.ocv = p.openCircuitVoltage(s->real("elec:battery:initialSoc").value_or(1.0));
        for (const SchemaInstance* port : portsOfPrim[s->prim.index]) {
            b.wireResistance +=
                port->real("elec:port:" + port->instance + ":wireResistance").value_or(0.0);
        }
        b.bus = *bus;
        t.batteries.push_back(std::move(b));
    }
    for (const SchemaInstance* s : world.instancesOf("ElectricalLoadAPI_1")) {
        Topology::Load l;
        l.name = std::string(world.signalInstanceName(s->prim));
        l.power = s->real("elec:load:power").value_or(0.0);
        l.minVoltage = s->real("elec:load:minVoltage").value_or(0.0);
        l.resistive = s->token("elec:load:model").value_or("constant_power") == "resistive";
        const auto bus = through(s);
        if (!bus) {
            problems += "\n  " + std::string(world.primPath(s->prim)) + ": no bus";
            continue;
        }
        l.bus = *bus;
        t.loads.push_back(std::move(l));
    }
    for (const SchemaInstance* s : world.instancesOf("ElectricalMotorAPI_1")) {
        const auto bus = through(s);
        if (!bus) {
            problems += "\n  " + std::string(world.primPath(s->prim)) + ": no bus";
            continue;
        }
        t.drives.push_back({std::string(world.signalInstanceName(s->prim)), *bus});
    }
    if (!problems.empty()) {
        return Unexpected{"the network cannot be exported:" + problems};
    }
    // Each bus starts at its batteries' open-circuit voltage, as the network does.
    for (std::size_t i = 0; i < t.buses.size(); ++i) {
        double conductance = 0.0;
        double injected = 0.0;
        for (const Topology::Battery& b : t.batteries) {
            if (b.bus == i) {
                const Thevenin th = theveninOf(b.params, BatteryState{}, 298.15);
                const double r = th.resistance + b.wireResistance;
                conductance += 1.0 / r;
                injected += b.ocv / r;
            }
        }
        t.buses[i].initialVoltage = conductance > 0.0 ? injected / conductance : 0.0;
    }
    return t;
}

/// Pack values of a per-cell RC pair: S in series, P in parallel.
std::pair<double, double> packPair(const BatteryParams& p, double r, double c) {
    const double s = p.seriesCells;
    const double n = p.parallelCells;
    return {r * s / n, c * n / s};
}

}  // namespace

Expected<std::string, std::string> spiceNetlist(const WorldView& world,
                                                const ExportOptions& options) {
    auto read = readTopology(world);
    if (!read) {
        return Unexpected{read.error()};
    }
    const Topology& t = *read;
    std::ostringstream out;
    const auto node = [&t](std::size_t bus) { return "n_" + identifier(t.buses[bus].name); };
    out << "* " << options.title << ": Lagrange electrical network (averaged)\n"
        << "* Generated by lagrange export spice (docs/design/domains/electrical.md section 7).\n"
        << "* Drives and loads are behavioural current sources; battery open-circuit voltages are\n"
        << "* held at the initial state of charge.\n";
    for (std::size_t i = 0; i < t.buses.size(); ++i) {
        out << "* bus " << t.buses[i].name << "\n"
            << "C_" << identifier(t.buses[i].name) << " " << node(i) << " 0 "
            << number(t.buses[i].capacitance) << " IC=" << number(t.buses[i].initialVoltage)
            << "\n";
    }
    for (const Topology::Battery& b : t.batteries) {
        const std::string id = identifier(b.name);
        const BatteryParams& p = b.params;
        out << "* battery " << b.name << " (" << p.seriesCells << "s" << p.parallelCells << "p)\n"
            << "V_" << id << "_ocv " << id << "_ocv 0 DC " << number(b.ocv) << "\n";
        std::string at = id + "_ocv";
        int n = 0;
        const auto series = [&](const std::string& element, double r, double c) {
            const std::string next = id + "_n" + std::to_string(++n);
            out << "R_" << id << "_" << element << " " << at << " " << next << " " << number(r)
                << "\n";
            if (c > 0.0) {
                out << "C_" << id << "_" << element << " " << at << " " << next << " " << number(c)
                    << "\n";
            }
            at = next;
        };
        series("r0", p.r0 * p.seriesCells / p.parallelCells, 0.0);
        for (const auto& [r, c] : {std::pair{p.r1, p.c1}, std::pair{p.r2, p.c2}}) {
            if (r > 0.0) {
                const auto [rp, cp] = packPair(p, r, c);
                series("rc" + std::to_string(n), rp, cp);
            }
        }
        out << "R_" << id << "_wire " << at << " " << node(b.bus) << " "
            << number(std::max(b.wireResistance, 1e-9)) << "\n";
    }
    for (const Topology::Load& l : t.loads) {
        const std::string bus = node(l.bus);
        if (l.resistive) {
            const double v = t.buses[l.bus].initialVoltage;
            out << "* load " << l.name << " (resistive)\nR_" << identifier(l.name) << " " << bus
                << " 0 " << number(v * v / l.power) << "\n";
        } else {
            out << "* load " << l.name << " (constant power, off below " << number(l.minVoltage)
                << " V)\nB_" << identifier(l.name) << " " << bus << " 0 I = " << number(l.power)
                << " / max(V(" << bus << "), " << number(kMinVoltage) << ") * ((V(" << bus
                << ") >= " << number(l.minVoltage) << ") ? 1 : 0)\n";
        }
    }
    for (const Topology::Drive& d : t.drives) {
        const std::string id = identifier(d.name);
        const std::string bus = node(d.bus);
        out << "* drive " << d.name << ": its electrical power (W) as a control voltage\n";
        if (const auto power = options.drivePower.find(d.name); power != options.drivePower.end()) {
            out << "V_p_" << id << " p_" << id << " 0 PWL(";
            bool first = true;
            for (const auto& [time, watts] : power->second) {
                out << (first ? "" : " ") << number(time) << " " << number(watts);
                first = false;
            }
            out << ")\n";
        } else {
            out << ".param p_" << id << "=0\nV_p_" << id << " p_" << id << " 0 DC {p_" << id
                << "}\n";
        }
        out << "B_" << id << " " << bus << " 0 I = V(p_" << id << ") / max(V(" << bus << "), "
            << number(kMinVoltage) << ")\n";
    }
    out << ".tran " << number(options.step) << " " << number(options.duration) << " uic\n";
    if (!options.writeData.empty()) {
        out << ".control\nrun\nwrdata " << options.writeData;
        for (std::size_t i = 0; i < t.buses.size(); ++i) {
            out << " v(" << node(i) << ")";
        }
        out << "\n.endc\n";
    }
    out << ".end\n";
    return out.str();
}

Expected<std::string, std::string> modelicaModel(const WorldView& world,
                                                 const ExportOptions& options) {
    auto read = readTopology(world);
    if (!read) {
        return Unexpected{read.error()};
    }
    const Topology& t = *read;
    const std::string model = identifier(options.title);
    std::ostringstream decl;
    std::ostringstream eq;
    decl << "model " << model
         << " \"Lagrange electrical network (averaged), generated by lagrange export modelica\"\n"
         << "  import Modelica.Electrical.Analog.Basic;\n"
         << "  import Modelica.Electrical.Analog.Sources;\n"
         << "  Basic.Ground ground;\n";
    for (const Topology::Bus& b : t.buses) {
        const std::string id = identifier(b.name);
        decl << "  Basic.Capacitor " << id << "(C=" << number(b.capacitance)
             << ", v(start=" << number(b.initialVoltage) << ", fixed=true)) \"bus\";\n";
        eq << "  connect(" << id << ".n, ground.p);\n";
    }
    const auto bus = [&t](std::size_t i) { return identifier(t.buses[i].name) + ".p"; };
    for (const Topology::Battery& b : t.batteries) {
        const std::string id = identifier(b.name);
        const BatteryParams& p = b.params;
        decl << "  Sources.ConstantVoltage " << id << "_ocv(V=" << number(b.ocv) << ");\n"
             << "  Basic.Resistor " << id
             << "_r0(R=" << number(p.r0 * p.seriesCells / p.parallelCells) << ");\n";
        eq << "  connect(" << id << "_ocv.n, ground.p);\n"
           << "  connect(" << id << "_ocv.p, " << id << "_r0.p);\n";
        std::string at = id + "_r0.n";
        int n = 0;
        for (const auto& [r, c] : {std::pair{p.r1, p.c1}, std::pair{p.r2, p.c2}}) {
            if (r <= 0.0) {
                continue;
            }
            const auto [rp, cp] = packPair(p, r, c);
            const std::string pair = id + "_rc" + std::to_string(++n);
            decl << "  Basic.Resistor " << pair << "_r(R=" << number(rp) << ");\n";
            eq << "  connect(" << at << ", " << pair << "_r.p);\n";
            if (cp > 0.0) {
                decl << "  Basic.Capacitor " << pair << "_c(C=" << number(cp) << ");\n";
                eq << "  connect(" << pair << "_r.p, " << pair << "_c.p);\n"
                   << "  connect(" << pair << "_r.n, " << pair << "_c.n);\n";
            }
            at = pair + "_r.n";
        }
        if (b.wireResistance > 0.0) {
            decl << "  Basic.Resistor " << id << "_wire(R=" << number(b.wireResistance) << ");\n";
            eq << "  connect(" << at << ", " << id << "_wire.p);\n";
            at = id + "_wire.n";
        }
        eq << "  connect(" << at << ", " << bus(b.bus) << ");\n";
    }
    for (const Topology::Load& l : t.loads) {
        const std::string id = identifier(l.name);
        const std::string v = identifier(t.buses[l.bus].name) + ".v";
        if (l.resistive) {
            const double v0 = t.buses[l.bus].initialVoltage;
            decl << "  Basic.Resistor " << id << "(R=" << number(v0 * v0 / l.power) << ");\n";
            eq << "  connect(" << id << ".p, " << bus(l.bus) << ");\n"
               << "  connect(" << id << ".n, ground.p);\n";
        } else {
            decl << "  Sources.SignalCurrent " << id << " \"constant power " << number(l.power)
                 << " W\";\n";
            eq << "  connect(" << id << ".p, " << bus(l.bus) << ");\n"
               << "  connect(" << id << ".n, ground.p);\n"
               << "  " << id << ".i = (if " << v << " >= " << number(l.minVoltage) << " then "
               << number(l.power) << " / max(" << v << ", " << number(kMinVoltage)
               << ") else 0);\n";
        }
    }
    for (const Topology::Drive& d : t.drives) {
        const std::string id = identifier(d.name);
        const std::string v = identifier(t.buses[d.bus].name) + ".v";
        decl << "  Sources.SignalCurrent " << id << " \"averaged drive\";\n";
        const auto power = options.drivePower.find(d.name);
        if (power != options.drivePower.end()) {
            decl << "  Modelica.Blocks.Sources.TimeTable p_" << id << "(table=[";
            bool first = true;
            for (const auto& [time, watts] : power->second) {
                decl << (first ? "" : "; ") << number(time) << ", " << number(watts);
                first = false;
            }
            decl << "]) \"electrical power, W\";\n";
        } else {
            decl << "  Modelica.Blocks.Sources.Constant p_" << id
                 << "(k=0) \"electrical power, W: edit\";\n";
        }
        eq << "  connect(" << id << ".p, " << bus(d.bus) << ");\n"
           << "  connect(" << id << ".n, ground.p);\n"
           << "  " << id << ".i = p_" << id << ".y / max(" << v << ", " << number(kMinVoltage)
           << ");\n";
    }
    std::ostringstream out;
    out << decl.str() << "equation\n"
        << eq.str() << "  annotation(experiment(StopTime=" << number(options.duration)
        << ", Interval=" << number(options.step) << "));\nend " << model << ";\n";
    return out.str();
}

}  // namespace lagrange::electrical
