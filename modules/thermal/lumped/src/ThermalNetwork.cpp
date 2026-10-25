// The thermal domain at the lumped fidelity; rules on the class in ThermalNetwork.hpp.
#include "lagrange/thermal/ThermalNetwork.hpp"

#include <map>
#include <string_view>

namespace lagrange::thermal {

using namespace literals;

namespace {

constexpr std::string_view kLoss = "/loss_power";

SignalSpec scalar(std::string name, Unit unit) {
    SignalSpec spec;
    spec.name = std::move(name);
    spec.unit = unit;
    return spec;
}

SignalSpec optional(std::string name, Unit unit, double fallback) {
    SignalSpec spec = scalar(std::move(name), unit);
    spec.optional = true;
    spec.defaultValue = fallback;
    return spec;
}

/// The node name of a "<class>/<node>/loss_power" signal, or empty.
std::string_view lossNode(std::string_view name) {
    if (!name.ends_with(kLoss)) {
        return {};
    }
    const std::string_view head = name.substr(0, name.size() - kLoss.size());
    const std::size_t slash = head.find('/');
    if (slash == std::string_view::npos || head.find('/', slash + 1) != std::string_view::npos) {
        return {};
    }
    return head.substr(slash + 1);
}

}  // namespace

ModuleInfo ThermalNetwork::info() const {
    return {"thermal", "lumped", 0, 1, false, true, CapabilityTier::Tier0};
}

Duration ThermalNetwork::preferredStep() const {
    return 100_ms;
}

Status ThermalNetwork::declare(const WorldView& world, const ModuleConfig& config,
                               SignalDeclarer& d) {
    names_.clear();
    sourceNames_.clear();
    initial_.clear();
    limits_.clear();
    outputs_.clear();
    sources_.clear();
    const std::optional<double> ambient = config.get<double>(kAmbientKey);
    if (!ambient || *ambient <= 0.0) {
        return Status::error(StatusCode::InvalidConfig,
                             "the thermal domain needs the site's ambient temperature "
                             "(profile thermal.ambient_temperature, K); it has no default");
    }
    if (!config.fidelity().empty() && config.fidelity() != "lumped") {
        return Status::error(StatusCode::InvalidConfig,
                             "thermal fidelity " + std::string(config.fidelity()) +
                                 " is not available; this module runs lumped");
    }
    std::string problems;
    std::vector<Node> nodes;
    std::map<std::uint32_t, std::size_t> nodeOfPrim;
    for (const SchemaInstance* s : world.instancesOf("ThermalAPI_1")) {
        const std::string path(world.primPath(s->prim));
        const auto capacity = s->real("thermal:heatCapacity");
        if (!capacity || *capacity <= 0.0) {
            problems += "\n  " + path + ": a thermal node needs thermal:heatCapacity > 0";
            continue;
        }
        Node node;
        node.capacity = *capacity;
        node.convection = s->real("thermal:convectionCoeff").value_or(10.0) *
                          s->real("thermal:convectionArea").value_or(0.0);
        node.radiation = s->real("thermal:emissivity").value_or(0.9) * kStefanBoltzmann *
                         s->real("thermal:radiativeArea").value_or(0.0);
        node.solarArea = s->real("thermal:solarAbsorptivity").value_or(0.7) *
                         s->real("thermal:solarArea").value_or(0.0);
        Limits limits;
        if (const auto max = s->real("thermal:maxTemperature")) {
            limits.hasMax = true;
            limits.maxTemperature = *max;
            limits.deratingStart = s->real("thermal:deratingStart").value_or(*max);
        }
        nodeOfPrim.emplace(s->prim.index, nodes.size());
        nodes.push_back(node);
        names_.emplace_back(world.signalInstanceName(s->prim));
        initial_.push_back(s->real("thermal:initialTemperature").value_or(*ambient));
        limits_.push_back(limits);
    }
    std::vector<Link> links;
    for (const SchemaInstance* s : world.instancesOf("ThermalLinkAPI_1")) {
        const std::string path(world.primPath(s->prim));
        const std::string base = "thermal:link:" + s->instance + ":";
        const auto conductance = s->real(base + "conductance");
        const auto target = s->rel(base + "target");
        const auto from = nodeOfPrim.find(s->prim.index);
        const auto to = target.size() == 1 ? nodeOfPrim.find(target[0].index) : nodeOfPrim.end();
        if (!conductance || *conductance <= 0.0 || from == nodeOfPrim.end() ||
            to == nodeOfPrim.end()) {
            problems += "\n  " + path + ": link " + s->instance +
                        " needs a conductance > 0, and a target, both thermal nodes";
            continue;
        }
        links.push_back({from->second, to->second, *conductance});
    }
    if (!problems.empty()) {
        return Status::error(StatusCode::InvalidConfig,
                             "the thermal network cannot be built:" + problems);
    }
    network_ = LumpedNetwork(std::move(nodes), std::move(links));

    sourceNames_.resize(names_.size());
    for (const SignalSpec& spec : d.declaredOutputs("")) {
        const std::string_view node = lossNode(spec.name);
        if (node.empty()) {
            continue;
        }
        for (std::size_t i = 0; i < names_.size(); ++i) {
            if (names_[i] == node) {
                SignalSpec in = scalar(spec.name, units::W);
                in.hold = Hold::WindowAverage;
                sources_.push_back({i, d.input(in)});
                sourceNames_[i].push_back(spec.name);
            }
        }
    }
    air_ = d.input(optional("env/air_temperature", units::K, *ambient));
    sky_ = d.input(optional("env/sky_temperature", units::K, *ambient));
    irradiance_ = d.input(optional("env/solar_irradiance", units::W_per_m2, 0.0));
    for (const std::string& name : names_) {
        const std::string p = "thermal/" + name + "/";
        outputs_.push_back({d.output(scalar(p + "temperature", units::K)),
                            d.output(scalar(p + "derating", units::one)),
                            d.output(scalar(p + "overtemp", units::one)),
                            d.output(scalar(p + "heat_flow_out", units::W))});
    }
    return {};
}

Status ThermalNetwork::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    temperatures_ = initial_;
    heat_.assign(temperatures_.size(), 0.0);
    heatOut_.assign(temperatures_.size(), 0.0);
    for (std::size_t i = 0; i < temperatures_.size(); ++i) {
        const Limits& l = limits_[i];
        bus.write(outputs_[i].temperature, temperatures_[i]);
        bus.write(outputs_[i].derating,
                  l.hasMax ? derating(temperatures_[i], l.deratingStart, l.maxTemperature) : 1.0);
        bus.write(outputs_[i].overtemp,
                  l.hasMax && temperatures_[i] >= l.maxTemperature ? 1.0 : 0.0);
        bus.write(outputs_[i].heatOut, 0.0);
    }
    return {};
}

Status ThermalNetwork::step(const StepContext& context, SignalBus& bus) {
    std::fill(heat_.begin(), heat_.end(), 0.0);
    for (const Source& s : sources_) {
        heat_[s.node] += bus.read(s.signal);
    }
    const Surroundings around{bus.read(air_), bus.read(sky_), bus.read(irradiance_)};
    network_.step(temperatures_, heat_, around, context.dt.seconds(), heatOut_);
    for (std::size_t i = 0; i < temperatures_.size(); ++i) {
        const Limits& l = limits_[i];
        const double t = temperatures_[i];
        bus.write(outputs_[i].temperature, t);
        bus.write(outputs_[i].derating,
                  l.hasMax ? derating(t, l.deratingStart, l.maxTemperature) : 1.0);
        bus.write(outputs_[i].overtemp, l.hasMax && t >= l.maxTemperature ? 1.0 : 0.0);
        bus.write(outputs_[i].heatOut, heatOut_[i]);
    }
    return {};
}

Status ThermalNetwork::saveState(StateWriter& writer) const {
    writer.putDoubles(temperatures_);
    return {};
}

Status ThermalNetwork::restoreState(StateReader& reader) {
    return reader.getDoubles(temperatures_)
               ? Status{}
               : Status::error(StatusCode::InvalidConfig, "thermal state truncated");
}

}  // namespace lagrange::thermal
