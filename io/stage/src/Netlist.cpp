// A SPICE netlist as an overlay: the battery, bus and load schemas the electrical network reads.
// The netlist stays a file ngspice runs: what SPICE cannot say (a battery's capacity, which joint
// a motor turns) is in `*lagrange` comment lines. Rules in docs/design/formats.md §9.1.
#include "Netlist.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <exception>
#include <filesystem>
#include <map>
#include <optional>
#include <regex>
#include <set>
#include <sstream>
#include <utility>
#include <vector>

namespace lagrange::stage {

namespace {

constexpr std::size_t kMaxRcPairs = 2;  // the battery model's r1 c1 and r2 c2

std::string lower(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text;
}

bool isGround(const std::string& node) {
    return node == "0" || node == "gnd" || node == "gnd!";
}

/// A SPICE number: 4.7k, 100u, 1meg. Anything after the scale letters is a unit, as in 10uF.
std::optional<double> number(const std::string& token) {
    std::size_t used = 0;
    double value = 0.0;
    try {
        value = std::stod(token, &used);
    } catch (const std::exception&) {
        return std::nullopt;
    }
    const std::string rest = lower(token.substr(used));
    if (!rest.empty() && std::isalpha(static_cast<unsigned char>(rest[0])) == 0) {
        return std::nullopt;
    }
    if (rest.rfind("meg", 0) == 0) {
        return value * 1e6;
    }
    if (rest.rfind("mil", 0) == 0) {
        return value * 25.4e-6;
    }
    if (rest.empty()) {
        return value;
    }
    switch (rest[0]) {
        case 't':
            return value * 1e12;
        case 'g':
            return value * 1e9;
        case 'k':
            return value * 1e3;
        case 'm':
            return value * 1e-3;
        case 'u':
            return value * 1e-6;
        case 'n':
            return value * 1e-9;
        case 'p':
            return value * 1e-12;
        case 'f':
            return value * 1e-15;
        default:
            return value;  // a unit alone: 12V, 10ohm
    }
}

/// A prim's name for an element or a node: letters first, as USD wants.
std::string primName(const std::string& text, char prefix) {
    std::string name;
    for (const char c : text) {
        name += std::isalnum(static_cast<unsigned char>(c)) != 0 ? c : '_';
    }
    if (name.empty() || std::isalpha(static_cast<unsigned char>(name[0])) == 0) {
        name.insert(name.begin(), prefix);
    }
    return name;
}

/// V_a and Va are the battery a, V1 stays V1: the name is what the signals are called.
std::string elementName(const std::string& element) {
    std::string rest = element.substr(1);
    if (!rest.empty() && rest[0] == '_') {
        rest.erase(0, 1);
    }
    if (rest.empty() || std::isalpha(static_cast<unsigned char>(rest[0])) == 0) {
        return primName(element, 'x');
    }
    return primName(rest, 'x');
}

std::vector<std::string> split(const std::string& line) {
    std::istringstream in(line);
    std::vector<std::string> out;
    for (std::string token; in >> token;) {
        out.push_back(token);
    }
    return out;
}

/// key=value words; the words before the first of them are returned in `plain`.
std::map<std::string, std::string> keysOf(const std::vector<std::string>& words, std::size_t from,
                                          std::vector<std::string>& plain) {
    std::map<std::string, std::string> keys;
    for (std::size_t i = from; i < words.size(); ++i) {
        const std::size_t at = words[i].find('=');
        if (at == std::string::npos) {
            plain.push_back(words[i]);
        } else {
            keys[lower(words[i].substr(0, at))] = words[i].substr(at + 1);
        }
    }
    return keys;
}

struct Element {
    std::string name;  // as written
    std::string a, b;  // nodes, lower case
    double value{0.0};
    int line{0};
};

struct Sink {  // a B source: constant power
    Element element;
    double minVoltage{0.0};
};

struct Directive {
    std::vector<std::string> plain;
    std::map<std::string, std::string> keys;
    int line{0};
};

struct Circuit {
    std::vector<Element> sources, resistors, capacitors;
    std::vector<Sink> sinks;
    std::map<std::string, Directive> batteries;  // by lower-case element name
    std::map<std::string, Directive> buses;      // by node
    std::vector<Directive> motors;
};

class Problems {
public:
    void add(int line, const std::string& text) {
        text_ += "\n  line " + std::to_string(line) + ": " + text;
    }
    void add(const std::string& text) { text_ += "\n  " + text; }
    [[nodiscard]] bool any() const { return !text_.empty(); }
    [[nodiscard]] const std::string& text() const { return text_; }

private:
    std::string text_;
};

const std::map<char, std::string>& notSimulated() {
    static const std::map<char, std::string> names{
        {'d', "a diode"},
        {'q', "a transistor"},
        {'m', "a MOSFET"},
        {'j', "a JFET"},
        {'l', "an inductor"},
        {'x', "a subcircuit"},
        {'i', "a current source"},
        {'e', "a controlled source"},
        {'f', "a controlled source"},
        {'g', "a controlled source"},
        {'h', "a controlled source"},
        {'s', "a switch"},
        {'w', "a switch"},
        {'k', "a coupling"},
        {'t', "a transmission line"},
    };
    return names;
}

/// Dot commands about the analysis, which do not change the circuit.
bool isAnalysis(const std::string& command) {
    static const std::set<std::string> analysis{
        ".tran",  ".op",      ".dc",   ".ac",    ".options", ".option", ".print", ".plot",
        ".probe", ".save",    ".temp", ".title", ".end",     ".ends",   ".ic",    ".nodeset",
        ".meas",  ".measure", ".four", ".noise", ".sens",    ".tf",     ".width",
    };
    return analysis.contains(command);
}

void readElement(const std::string& text, int line, Circuit& c, Problems& problems) {
    const std::vector<std::string> words = split(text);
    const std::string& name = words[0];
    const char kind = static_cast<char>(std::tolower(static_cast<unsigned char>(name[0])));
    if (kind == 'b') {
        // What `lagrange export spice` writes for a constant-power load.
        static const std::regex form(
            R"(^\s*\S+\s+(\S+)\s+(\S+)\s+i\s*=\s*([^\s/]+)\s*/\s*max\s*\(\s*v\(\s*([^\s)]+)\s*\)\s*,\s*[^\s)]+\s*\))"
            R"((?:\s*\*\s*\(\s*\(\s*v\(\s*([^\s)]+)\s*\)\s*>=\s*([^\s)]+)\s*\)\s*\?\s*1\s*:\s*0\s*\))?\s*$)",
            std::regex::icase);
        std::smatch m;
        const std::string low = lower(text);
        const auto power = std::regex_match(low, m, form) ? number(m[3].str()) : std::nullopt;
        if (!power || m[1].str() != m[4].str() || !isGround(m[2].str()) ||
            (m[5].matched && m[5].str() != m[1].str())) {
            problems.add(line,
                         name +
                             ": only a constant-power source is simulated: "
                             "I = P / max(V(n), e), with an optional * ((V(n) >= m) ? 1 : 0)");
            return;
        }
        Sink sink;
        sink.element = {name, m[1].str(), m[2].str(), *power, line};
        if (m[6].matched) {
            const auto least = number(m[6].str());
            if (!least) {
                problems.add(line, name + ": cannot read " + m[6].str());
                return;
            }
            sink.minVoltage = *least;
        }
        c.sinks.push_back(std::move(sink));
        return;
    }
    if (kind != 'r' && kind != 'c' && kind != 'v') {
        const auto known = notSimulated().find(kind);
        problems.add(line, name + (known != notSimulated().end()
                                       ? ": " + known->second + " is not simulated"
                                       : ": this kind of element is not simulated"));
        return;
    }
    if (words.size() < 4) {
        problems.add(line, name + ": needs two nodes and a value");
        return;
    }
    Element e{name, lower(words[1]), lower(words[2]), 0.0, line};
    std::size_t at = 3;
    if (kind == 'v' && lower(words[3]) == "dc") {
        at = 4;
    }
    const auto value = at < words.size() ? number(words[at]) : std::nullopt;
    if (!value) {
        problems.add(line, name + (kind == 'v' ? ": only a DC voltage is simulated" : "") +
                               (kind != 'v' ? ": its value must be a plain number" : ""));
        return;
    }
    e.value = *value;
    for (std::size_t i = at + 1; i < words.size(); ++i) {
        // A capacitor's start voltage belongs to the analysis: the network starts from the
        // batteries. Anything else changes what the element is.
        if (kind != 'c' || lower(words[i]).rfind("ic=", 0) != 0) {
            problems.add(line, name + ": cannot read " + words[i]);
            return;
        }
    }
    if (e.value <= 0.0) {
        problems.add(line, name + ": its value must be above zero");
        return;
    }
    (kind == 'r' ? c.resistors : kind == 'c' ? c.capacitors : c.sources).push_back(std::move(e));
}

void readDirective(const std::string& text, int line, Circuit& c, Problems& problems) {
    const std::vector<std::string> words = split(text);
    if (words.size() < 3) {
        problems.add(line, "*lagrange needs: battery ELEMENT, bus NODE or motor JOINT, and values");
        return;
    }
    Directive d;
    d.line = line;
    d.keys = keysOf(words, 3, d.plain);
    if (!d.plain.empty()) {
        problems.add(line, "*lagrange: " + d.plain[0] + " is not key=value");
        return;
    }
    const std::string kind = lower(words[1]);
    if (kind == "battery") {
        c.batteries[lower(words[2])] = std::move(d);
    } else if (kind == "bus") {
        c.buses[lower(words[2])] = std::move(d);
    } else if (kind == "motor") {
        d.plain.push_back(words[2]);  // joint names keep their case
        c.motors.push_back(std::move(d));
    } else {
        problems.add(line, "*lagrange " + words[1] + ": not battery, bus or motor");
    }
}

Circuit readCircuit(const std::string& text, Problems& problems) {
    Circuit c;
    std::vector<std::pair<int, std::string>> lines;
    std::istringstream in(text);
    int number = 0;
    for (std::string line; std::getline(in, line);) {
        ++number;
        if (number == 1) {
            continue;  // a SPICE file's first line is its title, whatever it says
        }
        const auto first = line.find_first_not_of(" \t\r");
        if (first == std::string::npos) {
            continue;
        }
        if (line[first] == '+' && !lines.empty()) {
            lines.back().second += " " + line.substr(first + 1);
            continue;
        }
        lines.emplace_back(number, line.substr(first));
    }
    bool control = false;
    for (auto& [at, line] : lines) {
        while (!line.empty() && std::isspace(static_cast<unsigned char>(line.back())) != 0) {
            line.pop_back();
        }
        const std::string low = lower(line);
        if (control) {
            control = low.rfind(".endc", 0) != 0;
            continue;
        }
        if (low.rfind(".control", 0) == 0) {
            control = true;
            continue;
        }
        if (line[0] == '*') {
            if (low.rfind("*lagrange", 0) == 0 && low.size() > 9 && std::isspace(low[9]) != 0) {
                readDirective(line, at, c, problems);
            }
            continue;
        }
        for (const char mark : {';', '$'}) {
            if (const auto comment = line.find(mark); comment != std::string::npos) {
                line.erase(comment);
            }
        }
        if (line[0] == '.') {
            const std::string command = lower(split(line)[0]);
            if (!isAnalysis(command)) {
                problems.add(at, command + " is not read: write the circuit out with plain values");
            }
            continue;
        }
        readElement(line, at, c, problems);
    }
    return c;
}

struct Battery {
    const Element* source{nullptr};
    std::string bus;
    double r0{0.0};
    std::vector<std::pair<double, double>> pairs;  // R, C
};

/// Follows each source's series chain to the node it feeds, marking what it uses.
std::vector<Battery> followChains(const Circuit& c, std::set<const Element*>& used,
                                  const std::set<std::string>& attached, Problems& problems) {
    std::set<std::string> sourceNodes;
    for (const Element& v : c.sources) {
        sourceNodes.insert(v.a);
    }
    // A node's elements between two nodes, neither of them ground.
    const auto between = [&c, &used](const std::string& node) {
        std::map<std::string, std::vector<const Element*>> neighbours;
        for (const std::vector<Element>* list : {&c.resistors, &c.capacitors}) {
            for (const Element& e : *list) {
                if (used.contains(&e) || isGround(e.a) || isGround(e.b)) {
                    continue;
                }
                if (e.a == node) {
                    neighbours[e.b].push_back(&e);
                } else if (e.b == node) {
                    neighbours[e.a].push_back(&e);
                }
            }
        }
        return neighbours;
    };
    std::vector<Battery> out;
    for (const Element& v : c.sources) {
        if (!isGround(v.b)) {
            problems.add(v.line, v.name + ": a battery's minus side must be ground, node 0");
            continue;
        }
        Battery b;
        b.source = &v;
        std::string at = v.a;
        bool ok = true;
        while (true) {
            auto neighbours = between(at);
            const bool first = at == v.a;
            if (!first && (attached.contains(at) || neighbours.size() != 1)) {
                break;  // the node it feeds
            }
            if (neighbours.size() != 1) {
                problems.add(v.line, v.name + ": needs one series resistance from " + at +
                                         " towards what it feeds");
                ok = false;
                break;
            }
            const auto& [next, elements] = *neighbours.begin();
            const Element* r = nullptr;
            const Element* cap = nullptr;
            for (const Element* e : elements) {
                const bool resistor = std::tolower(static_cast<unsigned char>(e->name[0])) == 'r';
                (resistor ? r : cap) = e;
            }
            if (r == nullptr || elements.size() > 2 || (elements.size() == 2 && cap == nullptr)) {
                problems.add(elements[0]->line,
                             elements[0]->name + ": between " + at + " and " + next +
                                 " a battery takes one resistor, or one resistor and capacitor");
                ok = false;
                break;
            }
            if (sourceNodes.contains(next)) {
                problems.add(r->line, r->name + " joins two batteries with nothing to feed");
                ok = false;
                break;
            }
            if (cap != nullptr) {
                b.pairs.emplace_back(r->value, cap->value);
                used.insert(cap);
            } else {
                b.r0 += r->value;
            }
            used.insert(r);
            at = next;
        }
        if (!ok) {
            continue;
        }
        if (b.r0 <= 0.0) {
            problems.add(v.line, v.name + ": needs a plain series resistor (its internal r0)");
            continue;
        }
        if (b.pairs.size() > kMaxRcPairs) {
            problems.add(v.line, v.name + ": a battery takes at most two RC pairs");
            continue;
        }
        b.bus = at;
        out.push_back(std::move(b));
    }
    return out;
}

/// A value a directive must give.
std::optional<double> required(const Directive& d, const std::string& what, const std::string& key,
                               const std::string& unit, Problems& problems) {
    const auto found = d.keys.find(key);
    if (found == d.keys.end()) {
        problems.add(d.line, what + " needs " + key + "= (" + unit + ")");
        return std::nullopt;
    }
    const auto value = number(found->second);
    if (!value) {
        problems.add(d.line, what + ": cannot read " + key + "=" + found->second);
    }
    return value;
}

std::optional<double> optionalValue(const Directive& d, const std::string& what,
                                    const std::string& key, Problems& problems) {
    const auto found = d.keys.find(key);
    if (found == d.keys.end()) {
        return std::nullopt;
    }
    const auto value = number(found->second);
    if (!value) {
        problems.add(d.line, what + ": cannot read " + key + "=" + found->second);
    }
    return value;
}

void refuseUnknownKeys(const Directive& d, const std::string& what,
                       const std::set<std::string>& known, Problems& problems) {
    for (const auto& [key, value] : d.keys) {
        if (!known.contains(key)) {
            problems.add(d.line, what + ": " + key + "= is not a value it takes");
        }
    }
}

}  // namespace

bool isNetlistPath(const std::string& path) {
    const std::string extension = lower(std::filesystem::path(path).extension().string());
    return extension == ".cir" || extension == ".sp" || extension == ".spice" ||
           extension == ".net" || extension == ".ckt";
}

Expected<YAML::Node, std::string> overlayOfNetlist(const std::string& text) {
    Problems problems;
    const Circuit c = readCircuit(text, problems);

    // Nodes something hangs on: a chain stops at the first of them.
    std::set<std::string> attached;
    for (const std::vector<Element>* list : {&c.resistors, &c.capacitors}) {
        for (const Element& e : *list) {
            if (isGround(e.a) != isGround(e.b)) {
                attached.insert(isGround(e.a) ? e.b : e.a);
            }
        }
    }
    for (const Sink& s : c.sinks) {
        attached.insert(s.element.a);
    }
    std::map<const Directive*, std::string> motorNode;
    for (const Directive& m : c.motors) {
        const auto node = m.keys.find("node");
        if (node == m.keys.end()) {
            problems.add(m.line, "motor " + m.plain[0] + " needs node= (the bus it draws from)");
        } else {
            motorNode[&m] = lower(node->second);
            attached.insert(lower(node->second));
        }
    }

    std::set<const Element*> used;
    const std::vector<Battery> batteries = followChains(c, used, attached, problems);
    if (c.sources.empty()) {
        problems.add("no voltage source: a netlist needs a battery to feed its bus");
    }
    std::set<std::string> buses;
    for (const Battery& b : batteries) {
        buses.insert(b.bus);
    }
    for (const std::vector<Element>* list : {&c.resistors, &c.capacitors}) {
        for (const Element& e : *list) {
            if (used.contains(&e)) {
                continue;
            }
            if (isGround(e.a) && isGround(e.b)) {
                problems.add(e.line, e.name + " has both ends on ground");
            } else if (!isGround(e.a) && !isGround(e.b)) {
                problems.add(e.line, e.name + " joins " + e.a + " and " + e.b +
                                         ": elements between two buses are not simulated yet");
            } else if (const std::string& node = isGround(e.a) ? e.b : e.a; !buses.contains(node)) {
                problems.add(e.line, e.name + ": no battery feeds its node " + node);
            }
        }
    }

    // The bus's start voltage, which the network starts from: a resistor is its power there.
    std::map<std::string, std::pair<double, double>> sums;  // by bus: Σ V/r, Σ 1/r
    for (const Battery& b : batteries) {
        sums[b.bus].first += b.source->value / b.r0;
        sums[b.bus].second += 1.0 / b.r0;
    }
    const auto startVoltage = [&sums](const std::string& bus) {
        const auto& [current, conductance] = sums.at(bus);
        return current / conductance;
    };

    YAML::Node entries(YAML::NodeType::Sequence);
    std::set<std::string> names;
    const auto claim = [&names, &problems](const std::string& name, int line) {
        if (!names.insert(name).second) {
            problems.add(line, "the name " + name + " is used twice");
        }
    };
    const auto powerPath = [](const std::string& name) { return "power/" + name; };
    std::map<std::string, std::string> busName;
    for (const std::string& node : buses) {
        const std::string name = primName(node, 'n');
        busName[node] = name;
        claim(name, 0);
        double capacitance = 0.0;
        for (const Element& e : c.capacitors) {
            if (!used.contains(&e) && (e.a == node || e.b == node)) {
                capacitance += e.value;
            }
        }
        YAML::Node bus;
        if (capacitance > 0.0) {
            bus["elec:bus:capacitance"] = capacitance;
        }
        if (const auto found = c.buses.find(node); found != c.buses.end()) {
            const std::string what = "bus " + node;
            refuseUnknownKeys(found->second, what, {"undervoltage", "overvoltage"}, problems);
            if (const auto v = optionalValue(found->second, what, "undervoltage", problems)) {
                bus["elec:bus:undervoltageTrip"] = *v;
            }
            if (const auto v = optionalValue(found->second, what, "overvoltage", problems)) {
                bus["elec:bus:overvoltageTrip"] = *v;
            }
        }
        YAML::Node entry;
        entry["create"] = powerPath(name);
        if (bus.size() == 0) {
            // The schema with its fallbacks: an empty map, not a null.
            bus = YAML::Node(YAML::NodeType::Map);
        }
        entry["schemas"]["ElectricalBusAPI_1"] = bus;
        entries.push_back(entry);
    }
    for (const auto& [node, directive] : c.buses) {
        if (!buses.contains(node)) {
            problems.add(directive.line, "*lagrange bus " + node + ": no battery feeds that node");
        }
    }

    for (const Battery& b : batteries) {
        const Element& v = *b.source;
        const auto directive = c.batteries.find(lower(v.name));
        if (directive == c.batteries.end()) {
            problems.add(v.line, v.name +
                                     ": SPICE cannot say what a battery holds. Add: *lagrange "
                                     "battery " +
                                     v.name +
                                     " capacity=COULOMBS cutoff=VOLTS max_charge=AMPS [soc=0..1]");
            continue;
        }
        const Directive& d = directive->second;
        const std::string what = "battery " + v.name;
        refuseUnknownKeys(d, what, {"capacity", "cutoff", "max_charge", "soc"}, problems);
        const auto capacity = required(d, what, "capacity", "C", problems);
        const auto cutoff = required(d, what, "cutoff", "V", problems);
        const auto charge = required(d, what, "max_charge", "A", problems);
        const auto soc = optionalValue(d, what, "soc", problems);
        if (!capacity || !cutoff || !charge) {
            continue;
        }
        const std::string name = elementName(v.name);
        claim(name, v.line);
        YAML::Node battery;
        battery["elec:battery:seriesCells"] = 1;
        battery["elec:battery:capacity"] = *capacity;
        battery["elec:battery:initialSoc"] = soc.value_or(1.0);
        // A DC source holds one voltage whatever the charge.
        battery["elec:battery:ocvSoc"].push_back(0.0);
        battery["elec:battery:ocvSoc"].push_back(1.0);
        battery["elec:battery:ocvVoltage"].push_back(v.value);
        battery["elec:battery:ocvVoltage"].push_back(v.value);
        battery["elec:battery:r0"] = b.r0;
        for (std::size_t i = 0; i < b.pairs.size(); ++i) {
            const std::string n = std::to_string(i + 1);
            battery["elec:battery:r" + n] = b.pairs[i].first;
            battery["elec:battery:c" + n] = b.pairs[i].second;
        }
        battery["elec:battery:cutoffVoltage"] = *cutoff;
        battery["elec:battery:maxChargeCurrent"] = *charge;
        YAML::Node port;
        port["elec:port:dc_out:connectedTo"] = powerPath(busName.at(b.bus));
        YAML::Node entry;
        entry["create"] = powerPath(name);
        entry["schemas"]["BatteryAPI_1"] = battery;
        entry["schemas"]["PortAPI_1:dc_out"] = port;
        entries.push_back(entry);
    }
    for (const auto& [element, directive] : c.batteries) {
        const bool found = std::any_of(c.sources.begin(), c.sources.end(),
                                       [&](const Element& v) { return lower(v.name) == element; });
        if (!found) {
            problems.add(directive.line, "*lagrange battery " + element + ": no such source");
        }
    }

    const auto addLoad = [&](const Element& e, const std::string& node, const std::string& model,
                             double power, double minVoltage) {
        const std::string name = elementName(e.name);
        claim(name, e.line);
        YAML::Node load;
        load["elec:load:model"] = model;
        load["elec:load:power"] = power;
        if (minVoltage > 0.0) {
            load["elec:load:minVoltage"] = minVoltage;
        }
        load["elec:powerPort"] = powerPath(busName.at(node));
        YAML::Node entry;
        entry["create"] = powerPath(name);
        entry["schemas"]["ElectricalLoadAPI_1"] = load;
        entries.push_back(entry);
    };
    for (const Element& e : c.resistors) {
        const std::string& node = isGround(e.a) ? e.b : e.a;
        if (!used.contains(&e) && isGround(e.a) != isGround(e.b) && buses.contains(node)) {
            const double v = startVoltage(node);
            addLoad(e, node, "resistive", v * v / e.value, 0.0);
        }
    }
    for (const Sink& s : c.sinks) {
        if (!buses.contains(s.element.a)) {
            problems.add(s.element.line,
                         s.element.name + ": no battery feeds its node " + s.element.a);
            continue;
        }
        addLoad(s.element, s.element.a, "constant_power", s.element.value, s.minVoltage);
    }

    for (const Directive& m : c.motors) {
        const std::string& joint = m.plain[0];
        const std::string what = "motor " + joint;
        refuseUnknownKeys(m, what,
                          {"node", "name", "type", "r", "l", "kt", "imax", "gear", "efficiency",
                           "pole_pairs", "rotor_inertia"},
                          problems);
        const auto r = required(m, what, "r", "ohm", problems);
        const auto l = required(m, what, "l", "H", problems);
        const auto kt = required(m, what, "kt", "N m/A", problems);
        const auto imax = required(m, what, "imax", "A", problems);
        const auto node = motorNode.find(&m);
        if (node == motorNode.end() || !r || !l || !kt || !imax) {
            continue;
        }
        if (!buses.contains(node->second)) {
            problems.add(m.line, what + ": no battery feeds its node " + node->second);
            continue;
        }
        const auto named = m.keys.find("name");
        const std::string name =
            primName(named != m.keys.end() ? named->second : joint + "_motor", 'm');
        claim(name, m.line);
        YAML::Node motor;
        if (const auto type = m.keys.find("type"); type != m.keys.end()) {
            motor["elec:motorType"] = type->second;
        }
        motor["elec:windingResistance"] = *r;
        motor["elec:inductance"] = *l;
        motor["elec:torqueConstant"] = *kt;
        motor["elec:maxCurrent"] = *imax;
        if (const auto v = optionalValue(m, what, "gear", problems)) {
            motor["elec:gearRatio"] = *v;
        }
        if (const auto v = optionalValue(m, what, "efficiency", problems)) {
            motor["elec:gearEfficiency"] = *v;
        }
        if (const auto v = optionalValue(m, what, "pole_pairs", problems)) {
            motor["elec:polePairs"] = static_cast<int>(*v);
        }
        if (const auto v = optionalValue(m, what, "rotor_inertia", problems)) {
            motor["elec:rotorInertia"] = *v;
        }
        motor["elec:joint"] = joint;
        motor["elec:powerPort"] = powerPath(busName.at(node->second));
        YAML::Node entry;
        entry["prim"] = joint;
        entry["create_child"] = name;
        entry["schemas"]["ElectricalMotorAPI_1"] = motor;
        entries.push_back(entry);
    }

    if (problems.any()) {
        return Unexpected{"the netlist cannot be simulated:" + problems.text()};
    }
    YAML::Node root;
    root["schema_version"] = 1;
    root["apply"] = entries;
    return root;
}

}  // namespace lagrange::stage
