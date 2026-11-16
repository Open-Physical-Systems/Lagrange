// The nozzle module (RFC 0022 §8); rules in Nozzles.hpp.
#include "lagrange/actuators/Nozzles.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lagrange::actuators {

namespace {

// kg/s: no named unit in core, the dimension is what wiring checks.
constexpr Unit kKgPerS{{.mass = 1, .time = -1}, 1.0, QuantityKind::Unspecified};

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
Vec3 unit(const Vec3& v) {
    const double n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    return n > 0.0 ? scale(v, 1.0 / n) : Vec3{};
}

/// A prim's body and its pose in it (a schema on a link names the body's own prim).
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

NozzleSet nozzlesOf(const WorldView& world, std::string& error) {
    NozzleSet out;
    const auto fail = [&](const std::string& why) {
        error = why;
        return NozzleSet{};
    };
    const auto tankOf = [&](PrimId prim, std::string& why) -> int {
        for (std::size_t i = 0; i < out.tanks.size(); ++i) {
            if (out.tanks[i].prim == prim) {
                return static_cast<int>(i);
            }
        }
        for (const SchemaInstance* s : world.instancesOf("TankAPI_1")) {
            if (s->prim != prim) {
                continue;
            }
            TankSpec t;
            t.prim = prim;
            t.name = std::string(world.signalInstanceName(prim));
            const auto [body, pose] = bodyAndPose(world, prim);
            t.body = body;
            t.inBody = pose;
            const auto capacity = s->real("act:tank:capacity");
            t.initial = s->real("act:tank:initial").value_or(1.0);
            t.density = s->real("act:tank:density").value_or(kWaterDensity);
            if (!capacity || !(*capacity > 0.0)) {
                why = "tank " + t.name + ": act:tank:capacity must be given and over 0";
                return -1;
            }
            t.capacity = *capacity;
            if (!(t.initial >= 0.0 && t.initial <= 1.0) || !(t.density > 0.0)) {
                why = "tank " + t.name +
                      ": act:tank:initial must be 0 to 1 and act:tank:density over 0";
                return -1;
            }
            out.tanks.push_back(std::move(t));
            return static_cast<int>(out.tanks.size() - 1);
        }
        why = std::string("act:nozzle:tank names ") + std::string(world.primPath(prim)) +
              ", which has no TankAPI_1";
        return -1;
    };
    for (const SchemaInstance* s : world.instancesOf("NozzleAPI_1")) {
        NozzleSpec z;
        z.prim = s->prim;
        z.name = std::string(world.signalInstanceName(s->prim));
        const auto [body, pose] = bodyAndPose(world, s->prim);
        z.body = body;
        z.inBody = pose;
        NozzleParams& p = z.params;
        const auto required = [&](const char* attr, double& into) {
            const auto v = s->real(attr);
            if (!v || !(*v > 0.0)) {
                return false;
            }
            into = *v;
            return true;
        };
        if (!required("act:nozzle:diameter", p.diameter) ||
            !required("act:nozzle:dischargeCoefficient", p.dischargeCoefficient) ||
            !required("act:nozzle:pressure", p.pressure)) {
            return fail("nozzle " + z.name +
                        ": act:nozzle:diameter, dischargeCoefficient and pressure must be given "
                        "and over 0");
        }
        if (p.dischargeCoefficient > 1.0) {
            return fail("nozzle " + z.name + ": act:nozzle:dischargeCoefficient is 0 to 1");
        }
        const std::string agent(s->token("act:nozzle:agent").value_or("water"));
        if (agent == "water") {
            p.agent = Agent::Water;
        } else if (agent == "foam") {
            p.agent = Agent::Foam;
        } else if (agent == "co2") {
            p.agent = Agent::Co2;
        } else {
            return fail("nozzle " + z.name + ": act:nozzle:agent " + agent +
                        " is not water, foam or co2");
        }
        p.foamExpansion = s->real("act:nozzle:foamExpansion").value_or(0.0);
        if (p.agent == Agent::Foam && !(p.foamExpansion > 0.0)) {
            return fail("nozzle " + z.name + ": a foam nozzle needs act:nozzle:foamExpansion");
        }
        p.sprayHalfAngle = s->real("act:nozzle:sprayHalfAngle").value_or(0.0);
        p.pumpEfficiency = s->real("act:nozzle:pumpEfficiency").value_or(0.0);
        if (!(p.sprayHalfAngle >= 0.0 && p.sprayHalfAngle < 1.5) ||
            !(p.pumpEfficiency >= 0.0 && p.pumpEfficiency <= 1.0)) {
            return fail("nozzle " + z.name +
                        ": act:nozzle:sprayHalfAngle must be 0 to 1.5 rad and pumpEfficiency 0 "
                        "to 1");
        }
        const auto axis = s->realArray("act:nozzle:axis");
        if (axis.size() == 3) {
            p.axis = unit(Vec3{axis[0], axis[1], axis[2]});
        }
        if (p.axis == Vec3{}) {
            return fail("nozzle " + z.name + ": act:nozzle:axis is zero");
        }
        if (const auto tank = s->rel("act:nozzle:tank"); !tank.empty()) {
            std::string why;
            z.tank = tankOf(tank.front(), why);
            if (z.tank < 0) {
                return fail("nozzle " + z.name + ": " + why);
            }
        }
        for (const SchemaInstance* load : world.instancesOf("ElectricalLoadAPI_1")) {
            if (load->prim == s->prim && load->token("elec:load:model").value_or("") == "signal") {
                z.signalLoad = true;
            }
        }
        out.nozzles.push_back(std::move(z));
    }
    for (std::size_t i = 0; i < out.nozzles.size(); ++i) {
        for (std::size_t j = i + 1; j < out.nozzles.size(); ++j) {
            if (out.nozzles[i].name == out.nozzles[j].name) {
                return fail("two nozzles are named " + out.nozzles[i].name);
            }
        }
    }
    return out;
}

ModuleInfo Nozzles::info() const {
    return {"actuators", mode_ == Mode::Full ? "nozzles" : "nozzles_shut", 1, 0,
            mode_ != Mode::Full};
}

Duration Nozzles::preferredStep() const {
    return Duration{10'000'000};  // 10 ms: a jet's path, a tank's level
}

Status Nozzles::declare(const WorldView& world, const ModuleConfig&, SignalDeclarer& d) {
    std::string error;
    set_ = nozzlesOf(world, error);
    if (!error.empty()) {
        return Status::error(StatusCode::InvalidConfig, error);
    }
    const World& raw = world.raw();
    gravity_ = raw.gravityLocal;
    rest_ = restPoses(raw);
    shapeBody_ = raw.colliders.body;
    const std::vector<bool> moving = movingBodies(raw);
    const std::size_t bodyCount = raw.bodies.names.size();
    bodies_.assign(bodyCount, Body{});
    std::vector<bool> nozzleOn(bodyCount, false), tankOn(bodyCount, false);
    for (const NozzleSpec& z : set_.nozzles) {
        if (z.body != kWorldBody && z.body.index < bodyCount) {
            nozzleOn[z.body.index] = true;
        }
    }
    for (const TankSpec& t : set_.tanks) {
        if (t.body != kWorldBody && t.body.index < bodyCount) {
            tankOn[t.body.index] = true;
        }
    }
    // The bodies joined to each other by joints (a robot): the jet passes through its own.
    std::vector<std::uint32_t> group(bodyCount);
    for (std::uint32_t b = 0; b < bodyCount; ++b) {
        group[b] = b;
    }
    const auto root = [&group](std::uint32_t b) {
        while (group[b] != b) {
            b = group[b] = group[group[b]];
        }
        return b;
    };
    for (std::size_t j = 0; j < raw.joints.parent.size(); ++j) {
        const BodyId a = raw.joints.parent[j];
        const BodyId c = raw.joints.child[j];
        if (a != kWorldBody && c != kWorldBody && a.index < bodyCount && c.index < bodyCount) {
            group[root(a.index)] = root(c.index);
        }
    }
    for (std::uint32_t b = 0; b < bodyCount; ++b) {
        Body& body = bodies_[b];
        body.com = raw.bodies.comLocal[b];
        body.robot = root(b);
        if (!moving[b]) {
            continue;
        }
        body.moving = true;
        const std::string base =
            "body/" + std::string(world.signalInstanceName(raw.bodies.prim[b])) + "/";
        SignalSpec pose = spec(base + "pose", SignalType::Pose, units::m, "world_local");
        pose.feedback = true;
        body.pose = d.input(pose);
        if (raw.bodies.kinematic[b]) {
            continue;
        }
        if (nozzleOn[b]) {
            body.wrenchNozzle =
                d.output(spec(base + "wrench/nozzle", SignalType::Wrench, units::N, "world_local"));
        }
        if (tankOn[b]) {
            body.wrenchTank =
                d.output(spec(base + "wrench/tank", SignalType::Wrench, units::N, "world_local"));
        }
    }
    out_.assign(set_.nozzles.size(), Out{});
    for (std::size_t k = 0; k < set_.nozzles.size(); ++k) {
        const NozzleSpec& z = set_.nozzles[k];
        const std::string base = "act/" + z.name + "/";
        Out& o = out_[k];
        SignalSpec command = spec(base + "command", SignalType::Scalar, units::one);
        command.optional = true;
        o.command = d.input(command);
        SignalSpec pressure = spec(base + "pressure_command", SignalType::Scalar, units::Pa);
        pressure.optional = true;
        o.pressureCommand = d.input(pressure);
        o.flow = d.output(spec(base + "flow", SignalType::Scalar, kKgPerS));
        o.velocity = d.output(spec(base + "velocity", SignalType::Scalar, units::m_per_s));
        o.impact = d.output(spec(base + "impact", SignalType::Vec3, units::m, "world_local"));
        o.footprint = d.output(spec(base + "footprint", SignalType::Scalar, units::m));
        o.landed = d.output(spec(base + "landed", SignalType::Bool, units::one));
        SignalSpec path = spec(base + "path", SignalType::ScalarArray, units::m, "world_local");
        path.arrayLength = 3 * kJetPathPoints;
        o.path = d.output(path);
        o.tankMass = d.output(spec(base + "tank_mass", SignalType::Scalar, units::kg));
        o.pumpPower = d.output(spec(base + "pump_power", SignalType::Scalar, units::W));
        o.reaction = d.output(spec(base + "reaction", SignalType::Scalar, units::N));
        if (z.signalLoad) {
            o.loadDemand =
                d.output(spec("load/" + z.name + "/power_demand", SignalType::Scalar, units::W));
        }
    }
    nozzleWrench_.assign(bodyCount, std::array<double, 6>{});
    tankWrench_.assign(bodyCount, std::array<double, 6>{});
    return {};
}

Status Nozzles::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    state_.assign(set_.nozzles.size(), State{});
    tankMass_.clear();
    for (const TankSpec& t : set_.tanks) {
        tankMass_.push_back(t.initial * t.fullMass());
    }
    jets_.assign(set_.nozzles.size(), JetPath{});
    const double nan = std::numeric_limits<double>::quiet_NaN();
    std::vector<double> none(3 * kJetPathPoints, nan);
    for (std::size_t k = 0; k < set_.nozzles.size(); ++k) {
        const Out& o = out_[k];
        const int tank = set_.nozzles[k].tank;
        bus.write(o.flow, 0.0);
        bus.write(o.velocity, 0.0);
        bus.writeN(o.impact, Vec3{nan, nan, nan});
        bus.write(o.footprint, 0.0);
        bus.write(o.landed, 0.0);
        bus.writeN(o.path, none);
        bus.write(o.tankMass, tank >= 0 ? tankMass_[static_cast<std::size_t>(tank)] : 0.0);
        bus.write(o.pumpPower, 0.0);
        bus.write(o.reaction, 0.0);
        if (o.loadDemand.valid()) {
            bus.write(o.loadDemand, 0.0);
        }
    }
    for (const Body& b : bodies_) {
        if (b.wrenchNozzle.valid()) {
            bus.writeN(b.wrenchNozzle, std::array<double, 6>{});
        }
        if (b.wrenchTank.valid()) {
            bus.writeN(b.wrenchTank, std::array<double, 6>{});
        }
    }
    seenGeneration_ = commands_ ? commands_->generation() : 0;
    return {};
}

Pose Nozzles::bodyPose(BodyId body, const SignalBus& bus) const {
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

void Nozzles::push(BodyId body, bool tank, const Vec3& force, const Vec3& at, const Pose& pose) {
    if (body == kWorldBody || body.index >= bodies_.size()) {
        return;
    }
    const Body& b = bodies_[body.index];
    if (!(tank ? b.wrenchTank : b.wrenchNozzle).valid()) {
        return;
    }
    auto& w = (tank ? tankWrench_ : nozzleWrench_)[body.index];
    const Vec3 com = compose(pose, Pose{b.com, {1.0, 0.0, 0.0, 0.0}}).position;
    const Vec3 torque = cross(sub(at, com), force);
    for (std::size_t a = 0; a < 3; ++a) {
        w.at(a) += force.at(a);
        w.at(3 + a) += torque.at(a);
    }
}

Status Nozzles::step(const StepContext& context, SignalBus& bus) {
    const double dt = context.dt.seconds();
    const bool fresh = commands_ && commands_->generation() != seenGeneration_;
    if (fresh) {
        seenGeneration_ = commands_->generation();
    }
    for (std::size_t k = 0; k < set_.nozzles.size(); ++k) {
        State& s = state_[k];
        const double onBus = bus.read(out_[k].command);
        if (onBus != s.busCommand) {
            s.busCommand = onBus;
            s.command = onBus;
        }
        double given = 0.0;
        if (fresh && commands_->take(set_.nozzles[k].name, given)) {
            s.command = given;
        }
    }
    for (auto& w : nozzleWrench_) {
        w = {};
    }
    for (auto& w : tankWrench_) {
        w = {};
    }
    const services::SceneQuery* scene = scene_ ? scene_(bus) : nullptr;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    // What each tank gives this step, shared among its open nozzles by their demand.
    std::vector<double> demand(set_.tanks.size(), 0.0);
    std::vector<double> wanted(set_.nozzles.size(), 0.0);
    std::vector<double> speed(set_.nozzles.size(), 0.0);
    std::vector<double> density(set_.nozzles.size(), kWaterDensity);
    std::vector<double> pressure(set_.nozzles.size(), 0.0);
    for (std::size_t k = 0; k < set_.nozzles.size(); ++k) {
        const NozzleSpec& z = set_.nozzles[k];
        const double opening = mode_ == Mode::Full ? std::clamp(state_[k].command, 0.0, 1.0) : 0.0;
        const double commanded = bus.read(out_[k].pressureCommand);
        pressure[k] = commanded > 0.0 ? commanded : z.params.pressure;
        if (z.tank >= 0) {
            density[k] = set_.tanks[static_cast<std::size_t>(z.tank)].density;
        }
        speed[k] = exitVelocity(z.params.dischargeCoefficient, pressure[k], density[k]);
        wanted[k] = nozzleMassFlow(density[k], nozzleArea(z.params), speed[k], opening);
        if (z.tank >= 0) {
            demand[static_cast<std::size_t>(z.tank)] += wanted[k] * dt;
        }
    }
    std::vector<double> share(set_.tanks.size(), 1.0);
    for (std::size_t i = 0; i < set_.tanks.size(); ++i) {
        if (demand[i] > tankMass_[i]) {
            share[i] = demand[i] > 0.0 ? tankMass_[i] / demand[i] : 0.0;
        }
        tankMass_[i] = std::max(0.0, tankMass_[i] - demand[i] * share[i]);
    }
    std::vector<double> path(3 * kJetPathPoints, nan);
    for (std::size_t k = 0; k < set_.nozzles.size(); ++k) {
        const NozzleSpec& z = set_.nozzles[k];
        const Out& o = out_[k];
        const double flow =
            z.tank >= 0 ? wanted[k] * share[static_cast<std::size_t>(z.tank)] : wanted[k];
        const Pose bodyAt = bodyPose(z.body, bus);
        const Pose exit = compose(bodyAt, z.inBody);
        const Vec3 dir = rotate(exit.orientation, z.params.axis);
        JetPath& jet = jets_[k];
        jet = JetPath{};
        std::ranges::fill(path, nan);
        if (flow > 0.0) {
            // The jet passes through its own robot and through whatever the exit is inside.
            const std::uint32_t robot = z.body != kWorldBody && z.body.index < bodies_.size()
                                            ? bodies_[z.body.index].robot
                                            : 0;
            std::vector<std::uint32_t> around;
            if (scene != nullptr) {
                around = scene->overlapSphere(exit.position, 1e-4);
            }
            const auto skip = [&](std::uint32_t shape) {
                if (std::ranges::find(around, shape) != around.end()) {
                    return true;
                }
                const BodyId owner = shape < shapeBody_.size() ? shapeBody_[shape] : kWorldBody;
                return z.body != kWorldBody && owner != kWorldBody &&
                       owner.index < bodies_.size() && bodies_[owner.index].robot == robot;
            };
            jet = traceJet(exit.position, scale(dir, speed[k]), gravity_, scene, skip);
            // Up to kJetPathPoints of it, evenly along its points, the last at its end.
            const std::size_t n = jet.points.size();
            const std::size_t m = std::min<std::size_t>(n, kJetPathPoints);
            for (std::size_t i = 0; i < m; ++i) {
                const std::size_t from = m > 1 ? i * (n - 1) / (m - 1) : 0;
                std::copy(jet.points[from].begin(), jet.points[from].end(),
                          path.begin() + static_cast<std::ptrdiff_t>(3 * i));
            }
            // The reaction ṁ v0 against the jet, at the exit.
            push(z.body, false, scale(dir, -flow * speed[k]), exit.position, bodyAt);
        }
        bus.write(o.flow, flow);
        bus.write(o.velocity, flow > 0.0 ? speed[k] : 0.0);
        bus.writeN(o.impact, jet.landed ? jet.impact : Vec3{nan, nan, nan});
        bus.write(o.footprint, jet.landed ? footprintRadius(z.params.diameter,
                                                            z.params.sprayHalfAngle, jet.length)
                                          : 0.0);
        bus.write(o.landed, jet.landed ? 1.0 : 0.0);
        bus.writeN(o.path, path);
        bus.write(o.tankMass, z.tank >= 0 ? tankMass_[static_cast<std::size_t>(z.tank)] : 0.0);
        const double power = pumpPower(pressure[k], flow / density[k], z.params.pumpEfficiency);
        bus.write(o.pumpPower, power);
        bus.write(o.reaction, flow * speed[k]);
        if (o.loadDemand.valid()) {
            bus.write(o.loadDemand, power);
        }
    }
    // Each tank's agent as a weight at its prim.
    for (std::size_t i = 0; i < set_.tanks.size(); ++i) {
        const TankSpec& t = set_.tanks[i];
        const Pose bodyAt = bodyPose(t.body, bus);
        push(t.body, true, scale(gravity_, tankMass_[i]), compose(bodyAt, t.inBody).position,
             bodyAt);
    }
    for (std::size_t b = 0; b < bodies_.size(); ++b) {
        if (bodies_[b].wrenchNozzle.valid()) {
            bus.writeN(bodies_[b].wrenchNozzle, nozzleWrench_[b]);
        }
        if (bodies_[b].wrenchTank.valid()) {
            bus.writeN(bodies_[b].wrenchTank, tankWrench_[b]);
        }
    }
    return {};
}

Status Nozzles::saveState(StateWriter& writer) const {
    writer.put(static_cast<std::uint64_t>(state_.size()));
    for (const State& s : state_) {
        writer.put(s.command);
        writer.put(s.busCommand);
    }
    writer.put(static_cast<std::uint64_t>(tankMass_.size()));
    for (const double m : tankMass_) {
        writer.put(m);
    }
    return {};
}

Status Nozzles::restoreState(StateReader& reader) {
    std::uint64_t count = 0;
    if (!reader.get(count) || count != state_.size()) {
        return Status::error(StatusCode::Internal, "actuators: the state is of other nozzles");
    }
    for (State& s : state_) {
        if (!reader.get(s.command) || !reader.get(s.busCommand)) {
            return Status::error(StatusCode::Internal, "actuators: short nozzle state");
        }
    }
    if (!reader.get(count) || count != tankMass_.size()) {
        return Status::error(StatusCode::Internal, "actuators: the state is of other tanks");
    }
    for (double& m : tankMass_) {
        if (!reader.get(m)) {
            return Status::error(StatusCode::Internal, "actuators: short tank state");
        }
    }
    return {};
}

}  // namespace lagrange::actuators
