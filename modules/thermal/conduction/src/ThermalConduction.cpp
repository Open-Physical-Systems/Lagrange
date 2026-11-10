// The thermal domain at the conduction fidelity; rules on the class in ThermalConduction.hpp.
#include "lagrange/thermal/ThermalConduction.hpp"
#include "lagrange/thermal/ThermalNetwork.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <string_view>

namespace lagrange::thermal {

using namespace literals;

namespace {

constexpr std::string_view kLoss = "/loss_power";
constexpr double kDefaultCell = 0.002;  // m: a few cells across a small part
constexpr std::size_t kDefaultMaxCells = 200000;

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

/// "<class>/<name>/loss_power" → name, else empty.
std::string_view lossName(std::string_view name) {
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

/// The body a prim is on: its own when it is a body's prim, else the loader's nearest body.
BodyId bodyOfPrim(const World& w, PrimId prim) {
    for (std::size_t b = 0; b < w.bodies.prim.size(); ++b) {
        if (w.bodies.prim[b] == prim) {
            return BodyId{static_cast<std::uint32_t>(b)};
        }
    }
    return prim.index < w.prims.body.size() ? w.prims.body[prim.index] : kWorldBody;
}

/// Where a prim is in its body's frame (identity for the body's own prim).
Pose poseInBodyOf(const World& w, PrimId prim) {
    for (const PrimId p : w.bodies.prim) {
        if (p == prim) {
            return Pose{};
        }
    }
    return prim.index < w.prims.poseInBody.size() ? w.prims.poseInBody[prim.index] : Pose{};
}

std::array<double, 3> vec3(std::span<const double> v, std::array<double, 3> fallback) {
    return v.size() == 3 ? std::array<double, 3>{v[0], v[1], v[2]} : fallback;
}

}  // namespace

Expected<ThermalSetup, std::string> thermalSetupOf(const WorldView& world, double ambient,
                                                   double cell) {
    const World& raw = world.raw();
    const std::vector<Pose> rest = restPoses(raw);
    ThermalSetup out;
    std::string problems;
    std::map<std::uint32_t, std::size_t> partOfBody;
    for (const SchemaInstance* s : world.instancesOf("ThermalMaterialAPI_1")) {
        const std::string path(world.primPath(s->prim));
        const BodyId body = bodyOfPrim(raw, s->prim);
        if (body == kWorldBody) {
            problems += "\n  " + path + ": a thermal material needs a link (a body), not scenery";
            continue;
        }
        if (partOfBody.contains(body.index)) {
            problems += "\n  " + path + ": its link has a thermal material already";
            continue;
        }
        SolidPart part;
        part.name = raw.bodies.names[body.index];
        part.rest = rest.at(body.index);
        ThermalMaterial m;
        if (const auto preset = s->token("thermal:material:preset"); preset && !preset->empty()) {
            const auto found = thermalMaterialPreset(*preset);
            if (!found) {
                problems += "\n  " + path + ": no thermal material preset " + std::string(*preset);
                continue;
            }
            m = *found;
        }
        m.conductivity = s->real("thermal:material:conductivity").value_or(m.conductivity);
        m.density = s->real("thermal:material:density").value_or(m.density);
        m.specificHeat = s->real("thermal:material:specificHeat").value_or(m.specificHeat);
        m.emissivity = s->real("thermal:material:emissivity").value_or(m.emissivity);
        if (const auto along = s->real("thermal:material:conductivityAlong")) {
            m.conductivityAlong = *along;
        }
        if (!(m.conductivity > 0.0) || !(m.density > 0.0) || !(m.specificHeat > 0.0) ||
            (m.conductivityAlong && !(*m.conductivityAlong > 0.0)) || m.emissivity < 0.0 ||
            m.emissivity > 1.0) {
            problems += "\n  " + path +
                        ": a thermal material needs conductivity, density and specificHeat "
                        "over 0 and an emissivity of 0 to 1 (a preset, or each authored)";
            continue;
        }
        part.material = m;
        std::array<double, 3> axis = vec3(s->realArray("thermal:material:axis"), {0.0, 0.0, 1.0});
        const double norm = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
        if (!(norm > 0.0)) {
            problems += "\n  " + path + ": thermal:material:axis must not be zero";
            continue;
        }
        part.axis = {axis[0] / norm, axis[1] / norm, axis[2] / norm};
        if (const auto contact = s->real("thermal:material:contactConductance")) {
            if (!(*contact > 0.0)) {
                problems += "\n  " + path + ": thermal:material:contactConductance must be over 0";
                continue;
            }
            part.contactConductance = *contact;
        }
        part.initialTemperature = s->real("thermal:material:initialTemperature").value_or(ambient);
        part.cell = cell;
        for (std::size_t c = 0; c < raw.colliders.body.size(); ++c) {
            if (raw.colliders.body[c] != body) {
                continue;
            }
            const ShapeType type = raw.colliders.shape[c];
            if (type == ShapeType::Plane || type == ShapeType::HeightField) {
                continue;
            }
            SolidShape shape;
            shape.type = type;
            shape.size = raw.colliders.size[c];
            shape.pose = raw.colliders.poseInBody[c];
            if (type == ShapeType::ConvexHull && raw.colliders.hull[c] < raw.hulls.size()) {
                const ConvexHull& hull = raw.hulls[raw.colliders.hull[c]];
                shape.points = hull.points;
                shape.faces = hull.faces;
            }
            part.shapes.push_back(std::move(shape));
        }
        partOfBody.emplace(body.index, out.parts.size());
        out.bodies.push_back(body);
        out.maxTemperature.push_back(s->real("thermal:material:maxTemperature"));
        out.parts.push_back(std::move(part));
    }
    // Each part's parent: the nearest ancestor body with a part.
    for (std::size_t p = 0; p < out.parts.size(); ++p) {
        BodyId up = raw.bodies.parent.at(out.bodies[p].index);
        while (up != kWorldBody) {
            if (const auto found = partOfBody.find(up.index); found != partOfBody.end()) {
                out.parts[p].parent = found->second;
                break;
            }
            up = raw.bodies.parent.at(up.index);
        }
    }
    // Heat sources, on the part of their prim's body, their box moved into the body frame.
    for (const SchemaInstance* s : world.instancesOf("HeatSourceAPI_1")) {
        const std::string path(world.primPath(s->prim));
        const BodyId body = bodyOfPrim(raw, s->prim);
        const auto found = partOfBody.find(body.index);
        if (body == kWorldBody || found == partOfBody.end()) {
            problems += "\n  " + path + ": heat source " + s->instance +
                        " is on a link with no thermal material";
            continue;
        }
        const std::string base = "thermal:source:" + s->instance + ":";
        HeatSourceSpec spec;
        spec.name = s->instance;
        spec.part = found->second;
        spec.power = s->real(base + "power").value_or(0.0);
        spec.efficiency = s->real(base + "efficiency").value_or(0.0);
        if (spec.power < 0.0 || spec.efficiency < 0.0 || spec.efficiency > 1.0) {
            problems += "\n  " + path + ": heat source " + s->instance +
                        " needs a power of 0 or more and an efficiency of 0 to 1";
            continue;
        }
        const Pose inBody = poseInBodyOf(raw, s->prim);
        spec.box.centre =
            compose(inBody, Pose{vec3(s->realArray(base + "position"), {0.0, 0.0, 0.0}),
                                 {1.0, 0.0, 0.0, 0.0}})
                .position;
        spec.box.size = vec3(s->realArray(base + "size"), {0.001, 0.001, 0.001});
        if (const auto signal = s->token(base + "powerSignal"); signal && !signal->empty()) {
            spec.powerSignal = std::string(*signal);
        }
        out.sources.push_back(std::move(spec));
    }
    // Motors without a lumped node of their own: their losses heat the link that holds the
    // stator, at the joint (motor/<m>/loss_power, when the electrical network runs).
    for (const SchemaInstance* motor : world.instancesOf("ElectricalMotorAPI_1")) {
        if (!world.instancesOf("ThermalAPI_1").empty()) {
            bool lumped = false;
            for (const SchemaInstance* node : world.instancesOf("ThermalAPI_1")) {
                lumped = lumped || node->prim == motor->prim;
            }
            if (lumped) {
                continue;
            }
        }
        const std::span<const PrimId> joint = motor->rel("elec:joint");
        if (joint.empty()) {
            continue;
        }
        const std::string_view jointName = world.signalInstanceName(joint[0]);
        for (std::size_t j = 0; j < raw.joints.names.size(); ++j) {
            if (raw.joints.names[j] != jointName || raw.joints.child[j] == kWorldBody ||
                raw.joints.parent[j] == kWorldBody) {
                continue;
            }
            const auto found = partOfBody.find(raw.joints.parent[j].index);
            if (found == partOfBody.end()) {
                continue;
            }
            const std::string name(world.signalInstanceName(motor->prim));
            if (out.parts[found->second].name == name) {
                continue;  // its link is named like it: the losses already go there
            }
            // Named for the mount, not the motor: thermal/<motor>/temperature is its winding's
            // (a lumped node's), which the electrical network reads.
            HeatSourceSpec spec;
            spec.name = name + "_mount";
            spec.part = found->second;
            spec.powerSignal = "motor/" + name + "/loss_power";
            spec.optional = true;
            spec.box.centre = raw.bodies.poseInParent.at(raw.joints.child[j].index).position;
            spec.box.size = {kMotorLossBox, kMotorLossBox, kMotorLossBox};
            out.sources.push_back(std::move(spec));
        }
    }
    if (!problems.empty()) {
        return Unexpected{"heat conduction cannot be set up:" + problems};
    }
    // In part order, each part's boxes in the same order as the sources.
    std::stable_sort(
        out.sources.begin(), out.sources.end(),
        [](const HeatSourceSpec& a, const HeatSourceSpec& b) { return a.part < b.part; });
    for (const HeatSourceSpec& spec : out.sources) {
        out.parts[spec.part].sources.push_back(spec.box);
    }
    return out;
}

ModuleInfo ThermalConduction::info() const {
    return {"thermal", "conduction", 0, 1, false, true, CapabilityTier::Tier0};
}

Duration ThermalConduction::preferredStep() const {
    return 100_ms;
}

Status ThermalConduction::declare(const WorldView& world, const ModuleConfig& config,
                                  SignalDeclarer& d) {
    notes_.clear();
    bodies_.clear();
    parts_.clear();
    sources_.clear();
    const std::optional<double> ambient = config.get<double>(kAmbientKey);
    if (!ambient || *ambient <= 0.0) {
        return Status::error(StatusCode::InvalidConfig,
                             "the thermal domain needs the site's ambient temperature "
                             "(profile thermal.ambient_temperature, K); it has no default");
    }
    const double cell = config.get<double>(kCellKey).value_or(kDefaultCell);
    const auto maxCells = static_cast<std::size_t>(
        config.get<double>(kMaxCellsKey).value_or(static_cast<double>(kDefaultMaxCells)));
    if (!(cell > 0.0) || maxCells < 1) {
        return Status::error(StatusCode::InvalidConfig,
                             "thermal.cell must be over 0 m and thermal.max_cells at least 1");
    }
    fixedCoefficient_ = config.get<double>(kSurfaceCoefficientKey);
    const World& raw = world.raw();
    rest_ = restPoses(raw);
    const std::vector<bool> moving = movingBodies(raw);
    auto setup = thermalSetupOf(world, *ambient, cell);
    if (!setup) {
        return Status::error(StatusCode::InvalidConfig, setup.error());
    }
    bodies_ = setup->bodies;
    for (std::size_t p = 0; p < setup->parts.size(); ++p) {
        PartIo io;
        io.maxAllowed = setup->maxTemperature[p];
        parts_.push_back(std::move(io));
    }
    for (const HeatSourceSpec& spec : setup->sources) {
        SourceIo io;
        io.part = spec.part;
        io.power = spec.power;
        io.efficiency = spec.efficiency;
        if (!spec.powerSignal.empty()) {
            SignalSpec in = scalar(spec.powerSignal, units::W);
            in.hold = Hold::WindowAverage;
            in.optional = spec.optional;
            io.powerSignal = d.input(in);
        }
        const std::string out = "thermal/" + spec.name + "/";
        io.temperature = d.output(scalar(out + "temperature", units::K));
        io.heat = d.output(scalar(out + "heat", units::W));
        io.radiant = d.output(scalar(out + "radiant_power", units::W));
        sources_.push_back(io);
    }
    std::string error;
    auto model = ConductionModel::build(std::move(setup->parts), maxCells, notes_, error);
    if (!model) {
        return Status::error(StatusCode::InvalidConfig, "heat conduction: " + error);
    }
    model_ = std::move(*model);
    // Losses written before this module on a part's link prim (motor/<m>/loss_power names the
    // motor; a part takes the losses named like its link).
    const std::vector<SignalSpec> declared = d.declaredOutputs("");
    for (std::size_t p = 0; p < parts_.size(); ++p) {
        const std::string& name = model_.parts()[p].name;
        for (const SignalSpec& spec : declared) {
            if (lossName(spec.name) == name) {
                SignalSpec in = scalar(spec.name, units::W);
                in.hold = Hold::WindowAverage;
                parts_[p].losses.push_back(d.input(in));
            }
        }
        const std::string body = "body/" + name + "/";
        if (moving.at(bodies_[p].index)) {
            SignalSpec pose;
            pose.name = body + "pose";
            pose.type = SignalType::Pose;
            pose.unit = units::m;
            pose.frame = "world_local";
            pose.feedback = true;
            pose.optional = true;  // without physics the part stays at its rest pose
            parts_[p].pose = d.input(pose);
            SignalSpec airspeed = optional(body + "airspeed", units::m_per_s, 0.0);
            airspeed.feedback = true;
            parts_[p].airspeed = d.input(airspeed);
        }
        const std::string out = "thermal/" + name + "/";
        parts_[p].temperature = d.output(scalar(out + "temperature", units::K));
        parts_[p].maxTemperature = d.output(scalar(out + "max_temperature", units::K));
        parts_[p].minTemperature = d.output(scalar(out + "min_temperature", units::K));
        parts_[p].heatOut = d.output(scalar(out + "heat_flow_out", units::W));
        parts_[p].overtemp = d.output(scalar(out + "overtemp", units::one));
    }
    air_ = d.input(optional("env/air_temperature", units::K, *ambient));
    sky_ = d.input(optional("env/sky_temperature", units::K, *ambient));
    irradiance_ = d.input(optional("env/solar_irradiance", units::W_per_m2, 0.0));
    return {};
}

Status ThermalConduction::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    temperatures_ = model_.initialTemperatures();
    inputs_.assign(parts_.size(), PartInput{});
    sourceHeat_.assign(sources_.size(), 0.0);
    for (std::size_t p = 0; p < parts_.size(); ++p) {
        inputs_[p].pose = rest_.at(bodies_[p].index);
        const PartIo& io = parts_[p];
        bus.write(io.temperature, model_.mean(temperatures_, p));
        bus.write(io.maxTemperature, model_.maximum(temperatures_, p));
        bus.write(io.minTemperature, model_.minimum(temperatures_, p));
        bus.write(io.heatOut, 0.0);
        bus.write(io.overtemp, 0.0);
    }
    std::vector<std::size_t> nth(parts_.size(), 0);
    for (const SourceIo& s : sources_) {
        bus.write(s.temperature, model_.sourceMean(temperatures_, s.part, nth[s.part]++));
        bus.write(s.heat, 0.0);
        bus.write(s.radiant, 0.0);
    }
    return {};
}

Status ThermalConduction::step(const StepContext& context, SignalBus& bus) {
    for (std::size_t p = 0; p < parts_.size(); ++p) {
        PartIo& io = parts_[p];
        PartInput& in = inputs_[p];
        if (io.pose.valid()) {
            const std::span<const double> v = bus.readN(io.pose);
            const double q =
                v.size() == 7 ? v[3] * v[3] + v[4] * v[4] + v[5] * v[5] + v[6] * v[6] : 0.0;
            if (q > 0.0) {
                in.pose = Pose{{v[0], v[1], v[2]}, {v[3], v[4], v[5], v[6]}};
            }
            in.airspeed = bus.read(io.airspeed);
        }
        in.heat = 0.0;
        for (const SignalId loss : io.losses) {
            in.heat += bus.read(loss);
        }
    }
    for (std::size_t i = 0; i < sources_.size(); ++i) {
        const SourceIo& s = sources_[i];
        const double power =
            s.powerSignal.valid() ? std::max(0.0, bus.read(s.powerSignal)) : s.power;
        sourceHeat_[i] = (1.0 - s.efficiency) * power;
    }
    const Surroundings around{bus.read(air_), bus.read(sky_), bus.read(irradiance_)};
    model_.step(temperatures_, inputs_, sourceHeat_, around, context.dt.seconds(),
                fixedCoefficient_);
    for (std::size_t p = 0; p < parts_.size(); ++p) {
        const PartIo& io = parts_[p];
        const double hottest = model_.maximum(temperatures_, p);
        bus.write(io.temperature, model_.mean(temperatures_, p));
        bus.write(io.maxTemperature, hottest);
        bus.write(io.minTemperature, model_.minimum(temperatures_, p));
        bus.write(io.heatOut, model_.heatOut(p));
        bus.write(io.overtemp, io.maxAllowed && hottest >= *io.maxAllowed ? 1.0 : 0.0);
    }
    std::vector<std::size_t> nth(parts_.size(), 0);
    for (std::size_t i = 0; i < sources_.size(); ++i) {
        const SourceIo& s = sources_[i];
        const double power =
            s.powerSignal.valid() ? std::max(0.0, bus.read(s.powerSignal)) : s.power;
        bus.write(s.temperature, model_.sourceMean(temperatures_, s.part, nth[s.part]++));
        bus.write(s.heat, sourceHeat_[i]);
        bus.write(s.radiant, s.efficiency * power);
    }
    return {};
}

Status ThermalConduction::saveState(StateWriter& writer) const {
    writer.putDoubles(temperatures_);
    return {};
}

Status ThermalConduction::restoreState(StateReader& reader) {
    std::vector<double> t(model_.cellCount());
    if (!reader.getDoubles(t)) {
        return Status::error(StatusCode::InvalidConfig, "heat conduction state truncated");
    }
    temperatures_ = std::move(t);
    return {};
}

}  // namespace lagrange::thermal
