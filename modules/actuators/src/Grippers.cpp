#include "lagrange/actuators/Grippers.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lagrange::actuators {

namespace {

SignalSpec spec(std::string name, SignalType type, Unit unit, std::string frame = {}) {
    SignalSpec s;
    s.name = std::move(name);
    s.type = type;
    s.unit = unit;
    s.frame = std::move(frame);
    return s;
}

Vec3 sub(const Vec3& a, const Vec3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
Vec3 scale(const Vec3& a, double s) {
    return {a[0] * s, a[1] * s, a[2] * s};
}
Vec3 pointIn(const Pose& pose, const Vec3& local) {
    return compose(pose, Pose{local, {1.0, 0.0, 0.0, 0.0}}).position;
}

Vec3 vec3Of(const SchemaInstance& s, const char* name, const Vec3& fallback) {
    const auto values = s.realArray(name);
    if (values.size() == 3) {
        return {values[0], values[1], values[2]};
    }
    return fallback;
}

Vec3 unit(const Vec3& v) {
    const double n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    return n > 0.0 ? scale(v, 1.0 / n) : Vec3{};
}

/// Below this a magnet "holds" what it pulls (act/<g>/holding), m: in contact, as far as the
/// physics' contact layer allows (a reporting threshold, not a model value).
constexpr double kMagnetContact = 0.002;

/// A prim's body and its pose in it. A relationship or a schema on a link names the body's own
/// prim, which a URDF robot's prim table does not map to the body: look it up directly.
std::pair<BodyId, Pose> bodyAndPose(const WorldView& world, PrimId prim) {
    const std::vector<PrimId>& own = world.raw().bodies.prim;
    for (std::uint32_t b = 0; b < own.size(); ++b) {
        if (own[b] == prim) {
            return {BodyId{b}, Pose{}};
        }
    }
    return {world.primBody(prim), world.primPoseInBody(prim)};
}

}  // namespace

void Commands::set(const std::string& name, double value) {
    const std::lock_guard<std::mutex> lock(mutex_);
    pending_[name] = value;
    generation_.fetch_add(1);
}

bool Commands::take(const std::string& name, double& value) {
    const std::lock_guard<std::mutex> lock(mutex_);
    const auto it = pending_.find(name);
    if (it == pending_.end()) {
        return false;
    }
    value = it->second;
    pending_.erase(it);
    return true;
}

std::vector<GripperSpec> grippersOf(const WorldView& world, std::string& error) {
    std::vector<GripperSpec> out;
    const auto place = [&](GripperSpec& g, const SchemaInstance& s) {
        g.prim = s.prim;
        g.name = std::string(world.signalInstanceName(s.prim));
        const auto [body, pose] = bodyAndPose(world, s.prim);
        g.body = body;
        g.inBody = pose;
    };
    const auto required = [&](const GripperSpec& g, const SchemaInstance& s, const char* attr,
                              double& into) {
        const auto v = s.real(attr);
        if (!v || !(*v > 0.0)) {
            if (error.empty()) {
                error = "gripper " + g.name + ": " + attr + " must be given and over 0";
            }
            return false;
        }
        into = *v;
        return true;
    };
    for (const SchemaInstance* s : world.instancesOf("SuctionGripperAPI_1")) {
        GripperSpec g;
        g.kind = GripperSpec::Kind::Suction;
        place(g, *s);
        SuctionParams& p = g.suction;
        if (!required(g, *s, "act:suction:diameter", p.diameter) ||
            !required(g, *s, "act:suction:volume", p.volume) ||
            !required(g, *s, "act:suction:flow", p.flow) ||
            !required(g, *s, "act:suction:ultimatePressure", p.ultimatePressure)) {
            return {};
        }
        p.sealGap = s->real("act:suction:sealGap").value_or(p.sealGap);
        p.maxTilt = s->real("act:suction:maxTilt").value_or(p.maxTilt);
        p.leakConductance = s->real("act:suction:leakConductance").value_or(0.0);
        p.axis = unit(vec3Of(*s, "act:suction:axis", p.axis));
        if (!(p.sealGap >= 0.0) || !(p.maxTilt >= 0.0) || !(p.leakConductance >= 0.0) ||
            p.axis == Vec3{}) {
            error = "gripper " + g.name +
                    ": act:suction:sealGap, maxTilt and leakConductance must not be negative, and "
                    "the axis not zero";
            return {};
        }
        if (p.ultimatePressure >= kStandardPressure) {
            error = "gripper " + g.name +
                    ": act:suction:ultimatePressure is absolute and must be below the atmosphere";
            return {};
        }
        out.push_back(std::move(g));
    }
    for (const SchemaInstance* s : world.instancesOf("MagnetGripperAPI_1")) {
        GripperSpec g;
        g.kind = GripperSpec::Kind::Magnet;
        place(g, *s);
        MagnetParams& p = g.magnet;
        const std::string kind(s->token("act:magnet:kind").value_or("electro"));
        if (kind != "electro" && kind != "electropermanent") {
            error = "gripper " + g.name + ": act:magnet:kind " + kind +
                    " is not electro or electropermanent";
            return {};
        }
        p.electropermanent = kind == "electropermanent";
        if (!required(g, *s, "act:magnet:turns", p.turns) ||
            !required(g, *s, "act:magnet:poleArea", p.poleArea) ||
            !required(g, *s, "act:magnet:coilResistance", p.coilResistance) ||
            !required(g, *s, "act:magnet:ratedVoltage", p.ratedVoltage)) {
            return {};
        }
        p.ironPathLength = s->real("act:magnet:ironPathLength").value_or(0.0);
        if (p.ironPathLength > 0.0 &&
            !required(g, *s, "act:magnet:ironPermeability", p.ironPermeability)) {
            return {};
        }
        p.saturation = s->real("act:magnet:saturation").value_or(0.0);
        p.coilInductance = s->real("act:magnet:coilInductance").value_or(0.0);
        p.axis = unit(vec3Of(*s, "act:magnet:axis", p.axis));
        if (p.ironPathLength < 0.0 || p.saturation < 0.0 || p.coilInductance < 0.0 ||
            p.axis == Vec3{}) {
            error = "gripper " + g.name +
                    ": act:magnet:ironPathLength, saturation and coilInductance must not be "
                    "negative, and the axis not zero";
            return {};
        }
        out.push_back(std::move(g));
    }
    // Signal names must not collide.
    for (std::size_t i = 0; i < out.size(); ++i) {
        for (std::size_t j = i + 1; j < out.size(); ++j) {
            if (out[i].name == out[j].name) {
                error = "two grippers are named " + out[i].name +
                        ": a prim has one gripper, suction or magnet";
                return {};
            }
        }
    }
    return out;
}

ModuleInfo Grippers::info() const {
    const char* name = mode_ == Mode::Full    ? "grippers"
                       : mode_ == Mode::Ideal ? "ideal"
                                              : "constant";
    return {"actuators", name, 1, 0, mode_ != Mode::Full};
}

Duration Grippers::preferredStep() const {
    return Duration{1'000'000};
}

Status Grippers::declare(const WorldView& world, const ModuleConfig& config, SignalDeclarer& d) {
    std::string error;
    specs_ = grippersOf(world, error);
    if (!error.empty()) {
        return Status::error(StatusCode::InvalidConfig, error);
    }
    sitePressure_ = config.get<double>(kPressureKey).value_or(kStandardPressure);
    siteTemperature_ = config.get<double>(kTemperatureKey).value_or(kReferenceTemperature);
    if (!(sitePressure_ > 0.0) || !(siteTemperature_ > 0.0)) {
        return Status::error(StatusCode::InvalidConfig,
                             "actuators: the site's pressure and temperature must be over 0");
    }
    const World& raw = world.raw();
    rest_ = restPoses(raw);
    shapeBody_ = raw.colliders.body;
    const std::vector<bool> moving = movingBodies(raw);
    const std::size_t bodyCount = raw.bodies.names.size();
    bodies_.assign(bodyCount, Body{});
    anySuction_ = anyMagnet_ = false;
    for (const GripperSpec& g : specs_) {
        (g.kind == GripperSpec::Kind::Suction ? anySuction_ : anyMagnet_) = true;
    }
    // What a part is made of, by body: SensorMaterialAPI_1 on the body's prim or below it.
    for (const SchemaInstance* s : world.instancesOf("SensorMaterialAPI_1")) {
        const BodyId b = bodyAndPose(world, s->prim).first;
        if (b == kWorldBody || b.index >= bodyCount) {
            continue;
        }
        if (const auto k = s->real("sensmat:airPermeability")) {
            bodies_[b.index].airPermeability = std::max(0.0, *k);
        }
        if (const auto mu = s->real("sensmat:relativePermeability")) {
            bodies_[b.index].relativePermeability = *mu;
        }
    }
    // The bodies the rays can find and the grippers ride on.
    std::vector<bool> wanted(bodyCount, false);
    for (const BodyId b : raw.colliders.body) {
        if (b != kWorldBody && b.index < bodyCount && moving[b.index]) {
            wanted[b.index] = true;
        }
    }
    for (const GripperSpec& g : specs_) {
        if (g.body != kWorldBody && g.body.index < bodyCount && moving[g.body.index]) {
            wanted[g.body.index] = true;
        }
    }
    for (std::uint32_t b = 0; b < bodyCount; ++b) {
        Body& body = bodies_[b];
        body.com = raw.bodies.comLocal[b];
        if (!wanted[b]) {
            continue;
        }
        body.moving = true;
        const std::string base =
            "body/" + std::string(world.signalInstanceName(raw.bodies.prim[b])) + "/";
        SignalSpec pose = spec(base + "pose", SignalType::Pose, units::m, "world_local");
        pose.feedback = true;
        body.pose = d.input(pose);
        if (mode_ != Mode::Never && !raw.bodies.kinematic[b]) {
            body.pushable = true;
            if (anySuction_) {
                body.wrenchSuction = d.output(
                    spec(base + "wrench/suction", SignalType::Wrench, units::N, "world_local"));
            }
            if (anyMagnet_) {
                body.wrenchMagnet = d.output(
                    spec(base + "wrench/magnet", SignalType::Wrench, units::N, "world_local"));
            }
        }
    }
    SignalSpec pressure = spec("env/pressure", SignalType::Scalar, units::Pa);
    pressure.optional = true;
    pressure.feedback = true;
    pressure.defaultValue = std::numeric_limits<double>::quiet_NaN();
    envPressure_ = d.input(pressure);
    SignalSpec temperature = spec("env/air_temperature", SignalType::Scalar, units::K);
    temperature.optional = true;
    temperature.feedback = true;
    temperature.defaultValue = std::numeric_limits<double>::quiet_NaN();
    envTemperature_ = d.input(temperature);
    out_.assign(specs_.size(), Out{});
    for (std::size_t k = 0; k < specs_.size(); ++k) {
        const GripperSpec& g = specs_[k];
        const std::string base = "act/" + g.name + "/";
        Out& o = out_[k];
        // 1 on, 0 off. A scalar, as the ROS bridge's topic inputs write std_msgs/Bool.
        SignalSpec command = spec(base + "command", SignalType::Scalar, units::one);
        command.optional = true;
        o.command = d.input(command);
        o.holding = d.output(spec(base + "holding", SignalType::Int, units::one));
        o.held = d.output(spec(base + "held", SignalType::Bool, units::one));
        o.force = d.output(spec(base + "force", SignalType::Scalar, units::N));
        if (g.kind == GripperSpec::Kind::Suction) {
            o.pressure = d.output(spec(base + "pressure", SignalType::Scalar, units::Pa));
            o.sealed = d.output(spec(base + "sealed", SignalType::Bool, units::one));
        } else {
            o.current = d.output(spec(base + "current", SignalType::Scalar, units::A));
            o.gap = d.output(spec(base + "gap", SignalType::Scalar, units::m));
            o.power = d.output(spec(base + "power", SignalType::Scalar, units::W));
        }
    }
    suctionWrench_.assign(bodyCount, std::array<double, 6>{});
    magnetWrench_.assign(bodyCount, std::array<double, 6>{});
    return {};
}

Status Grippers::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    state_.assign(specs_.size(), State{});
    for (std::size_t k = 0; k < specs_.size(); ++k) {
        State& s = state_[k];
        s.pressure = sitePressure_;
        const Out& o = out_[k];
        bus.write(o.holding, -1.0);
        bus.write(o.held, 0.0);
        bus.write(o.force, 0.0);
        if (specs_[k].kind == GripperSpec::Kind::Suction) {
            bus.write(o.pressure, s.pressure);
            bus.write(o.sealed, 0.0);
        } else {
            bus.write(o.current, 0.0);
            bus.write(o.gap, kMagnetReach);
            bus.write(o.power, 0.0);
        }
    }
    for (const Body& b : bodies_) {
        if (b.wrenchSuction.valid()) {
            bus.writeN(b.wrenchSuction, std::array<double, 6>{});
        }
        if (b.wrenchMagnet.valid()) {
            bus.writeN(b.wrenchMagnet, std::array<double, 6>{});
        }
    }
    seenGeneration_ = commands_ ? commands_->generation() : 0;
    return {};
}

Pose Grippers::bodyPose(BodyId body, const SignalBus& bus) const {
    if (body == kWorldBody || body.index >= bodies_.size()) {
        return Pose{};
    }
    const Body& b = bodies_[body.index];
    if (b.moving) {
        const std::span<const double> p = bus.readN(b.pose);
        if (p.size() == 7) {
            return Pose{{p[0], p[1], p[2]}, {p[3], p[4], p[5], p[6]}};
        }
    }
    return rest_[body.index];
}

void Grippers::push(BodyId body, bool magnet, const Vec3& force, const Vec3& at, const Pose& pose) {
    if (body == kWorldBody || body.index >= bodies_.size() || !bodies_[body.index].pushable) {
        return;
    }
    auto& w = (magnet ? magnetWrench_ : suctionWrench_)[body.index];
    const Vec3 com = pointIn(pose, bodies_[body.index].com);
    const Vec3 torque = cross(sub(at, com), force);
    for (std::size_t a = 0; a < 3; ++a) {
        w.at(a) += force.at(a);
        w.at(3 + a) += torque.at(a);
    }
}

Status Grippers::step(const StepContext& context, SignalBus& bus) {
    const double dt = context.dt.seconds();
    double pAtm = bus.read(envPressure_);
    if (!std::isfinite(pAtm) || !(pAtm > 0.0)) {
        pAtm = sitePressure_;
    }
    double temperature = bus.read(envTemperature_);
    if (!std::isfinite(temperature) || !(temperature > 0.0)) {
        temperature = siteTemperature_;
    }
    // Commands: a change on the bus (a ROS topic), or one given through the API, last wins.
    const bool fresh = commands_ && commands_->generation() != seenGeneration_;
    if (fresh) {
        seenGeneration_ = commands_->generation();
    }
    for (std::size_t k = 0; k < specs_.size(); ++k) {
        State& s = state_[k];
        const double onBus = bus.read(out_[k].command);
        if (onBus != s.busCommand) {
            s.busCommand = onBus;
            s.command = onBus;
        }
        double given = 0.0;
        if (fresh && commands_->take(specs_[k].name, given)) {
            s.command = given;
        }
    }
    for (auto& w : suctionWrench_) {
        w = {};
    }
    for (auto& w : magnetWrench_) {
        w = {};
    }
    const services::SceneQuery* scene = mode_ != Mode::Never && scene_ ? scene_(bus) : nullptr;
    for (std::size_t k = 0; k < specs_.size(); ++k) {
        const GripperSpec& g = specs_[k];
        State& s = state_[k];
        const Out& o = out_[k];
        const bool on = s.command > 0.5;
        const Pose bodyAt = bodyPose(g.body, bus);
        const Pose face = compose(bodyAt, g.inBody);
        const Vec3& axisLocal =
            g.kind == GripperSpec::Kind::Suction ? g.suction.axis : g.magnet.axis;
        const Vec3 axis = rotate(face.orientation, axisLocal);
        double force = 0.0;
        BodyId target = kWorldBody;
        bool holding = false;
        if (g.kind == GripperSpec::Kind::Suction) {
            const SuctionParams& p = g.suction;
            Seal seal;
            if (scene != nullptr) {
                seal = findSeal(*scene, p, face.position, axis, g.body, shapeBody_);
            }
            if (mode_ == Mode::Full) {
                SuctionInputs in;
                in.on = on;
                in.sealed = seal.sealed;
                in.atmosphere = pAtm;
                in.temperature = temperature;
                if (seal.sealed && seal.body != kWorldBody && seal.body.index < bodies_.size()) {
                    in.porousConductance =
                        bodies_[seal.body.index].airPermeability * suctionArea(p);
                }
                s.pressure = stepSuctionPressure(p, s.pressure, dt, in);
            } else {
                s.pressure = mode_ == Mode::Ideal && on && seal.sealed ? p.ultimatePressure : pAtm;
            }
            if (seal.sealed) {
                force = suctionForce(p, s.pressure, pAtm);
                target = seal.body;
            }
            holding = force > 0.0;
            bus.write(o.pressure, s.pressure);
            bus.write(o.sealed, seal.sealed ? 1.0 : 0.0);
        } else {
            const MagnetParams& p = g.magnet;
            Gap gap;
            if (scene != nullptr) {
                gap = findGap(*scene, face.position, axis, g.body, shapeBody_);
            }
            const double voltage = on ? p.ratedVoltage : 0.0;
            double drawn = 0.0;    // A, what the coil takes
            double working = 0.0;  // A, the current whose flux pulls
            if (mode_ == Mode::Full) {
                if (p.electropermanent) {
                    s.latched = on;  // switched; holds with no current
                    working = s.latched ? p.ratedVoltage / p.coilResistance : 0.0;
                } else {
                    s.current = stepCoilCurrent(p, s.current, voltage, dt);
                    drawn = working = s.current;
                }
            } else if (mode_ == Mode::Ideal) {
                working = on ? p.ratedVoltage / p.coilResistance : 0.0;
                drawn = p.electropermanent ? 0.0 : working;
                s.current = drawn;
            }
            const bool found = gap.shape != services::RayHit::kNone;
            if (found) {
                target = gap.shape < shapeBody_.size() ? shapeBody_[gap.shape] : kWorldBody;
                const double mu = target != kWorldBody && target.index < bodies_.size()
                                      ? bodies_[target.index].relativePermeability
                                      : 1.0;
                force = magnetForce(p, working, gap.gap, mu);
            }
            holding = force > 0.0 && gap.gap <= kMagnetContact;
            bus.write(o.current, drawn);
            bus.write(o.gap, found ? gap.gap : kMagnetReach);
            bus.write(o.power, drawn * drawn * p.coilResistance);
        }
        if (force > 0.0) {
            // The part is pulled toward the gripper, the gripper toward the part, at the face.
            const bool magnet = g.kind == GripperSpec::Kind::Magnet;
            push(target, magnet, scale(axis, -force), face.position, bodyPose(target, bus));
            push(g.body, magnet, scale(axis, force), face.position, bodyAt);
        }
        const bool onBody = holding && target != kWorldBody;
        bus.write(o.force, force);
        bus.write(o.holding, onBody ? static_cast<double>(target.index) : -1.0);
        bus.write(o.held, holding ? 1.0 : 0.0);
    }
    for (std::size_t b = 0; b < bodies_.size(); ++b) {
        if (bodies_[b].wrenchSuction.valid()) {
            bus.writeN(bodies_[b].wrenchSuction, suctionWrench_[b]);
        }
        if (bodies_[b].wrenchMagnet.valid()) {
            bus.writeN(bodies_[b].wrenchMagnet, magnetWrench_[b]);
        }
    }
    return {};
}

Status Grippers::saveState(StateWriter& writer) const {
    writer.put(static_cast<std::uint64_t>(state_.size()));
    for (const State& s : state_) {
        writer.put(s.command);
        writer.put(s.busCommand);
        writer.put(s.pressure);
        writer.put(s.current);
        writer.put(static_cast<std::uint8_t>(s.latched ? 1 : 0));
    }
    return {};
}

Status Grippers::restoreState(StateReader& reader) {
    std::uint64_t count = 0;
    if (!reader.get(count) || count != state_.size()) {
        return Status::error(StatusCode::Internal, "actuators: the state is of other grippers");
    }
    for (State& s : state_) {
        std::uint8_t latched = 0;
        if (!reader.get(s.command) || !reader.get(s.busCommand) || !reader.get(s.pressure) ||
            !reader.get(s.current) || !reader.get(latched)) {
            return Status::error(StatusCode::Internal, "actuators: short state");
        }
        s.latched = latched != 0;
    }
    return {};
}

}  // namespace lagrange::actuators
