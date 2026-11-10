#include "lagrange/tether/Tethers.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lagrange::tether {

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

Vec3 pointIn(const Pose& pose, const Vec3& local) {
    return compose(pose, Pose{local, {1.0, 0.0, 0.0, 0.0}}).position;
}

Vec3 vec3Of(const SchemaInstance& s, const char* name) {
    const auto values = s.realArray(name);
    if (values.size() == 3) {
        return {values[0], values[1], values[2]};
    }
    return {};
}

}  // namespace

std::vector<TetherSpec> tethersOf(const WorldView& world, std::string& error) {
    std::vector<TetherSpec> out;
    for (const SchemaInstance* s : world.instancesOf("TetherAPI_1")) {
        TetherSpec t;
        t.prim = s->prim;
        t.name = std::string(world.signalInstanceName(s->prim));
        t.anchorBody = world.primBody(s->prim);
        t.anchorInBody = pointIn(world.primPoseInBody(s->prim), vec3Of(*s, "tether:anchorPoint"));
        const auto attach = s->rel("tether:attach");
        if (attach.empty()) {
            error = "tether " + t.name + ": tether:attach names no body";
            return {};
        }
        t.attachBody = world.primBody(attach[0]);
        t.attachInBody = pointIn(world.primPoseInBody(attach[0]), vec3Of(*s, "tether:attachPoint"));
        CableParams& p = t.params;
        p.length = s->real("tether:length").value_or(0.0);
        p.diameter = s->real("tether:diameter").value_or(p.diameter);
        p.linearDensity = s->real("tether:linearDensity").value_or(p.linearDensity);
        p.axialStiffness = s->real("tether:axialStiffness").value_or(p.axialStiffness);
        p.dampingRatio = s->real("tether:dampingRatio").value_or(p.dampingRatio);
        p.bendStiffness = s->real("tether:bendStiffness").value_or(p.bendStiffness);
        p.friction = s->real("tether:friction").value_or(p.friction);
        p.dragCoeff = s->real("tether:dragCoeff").value_or(p.dragCoeff);
        p.segmentLength = s->real("tether:segmentLength").value_or(p.segmentLength);
        p.winchTension = s->real("tether:winchTension").value_or(p.winchTension);
        p.winchSpeed = s->real("tether:winchSpeed").value_or(p.winchSpeed);
        p.maxLength = s->real("tether:maxLength").value_or(0.0);
        const std::string winch = std::string(s->token("tether:winch").value_or("fixed"));
        if (winch == "tension") {
            p.winch = Winch::Tension;
        } else if (winch == "free") {
            p.winch = Winch::Free;
        } else if (winch != "fixed") {
            error =
                "tether " + t.name + ": tether:winch " + winch + " is not fixed, tension or free";
            return {};
        }
        const auto positive = [&](double v, const char* what) {
            if (!(v > 0.0)) {
                error = "tether " + t.name + ": " + what + " must be over 0";
            }
            return v > 0.0;
        };
        if (!positive(p.length, "tether:length") || !positive(p.diameter, "tether:diameter") ||
            !positive(p.linearDensity, "tether:linearDensity") ||
            !positive(p.axialStiffness, "tether:axialStiffness") ||
            !positive(p.segmentLength, "tether:segmentLength")) {
            return {};
        }
        if (p.maxLength > 0.0 && p.maxLength < p.length) {
            error = "tether " + t.name + ": tether:maxLength is shorter than tether:length";
            return {};
        }
        out.push_back(std::move(t));
    }
    return out;
}

ModuleInfo Tethers::info() const {
    return {"tether", zero_ ? "zero" : "xpbd_cable", 1, 0, zero_};
}

Duration Tethers::preferredStep() const {
    return Duration{1'000'000};
}

int Tethers::movingIndex(BodyId body) const {
    if (body == kWorldBody || body.index >= bodyIndex_.size()) {
        return -1;
    }
    return bodyIndex_[body.index];
}

Status Tethers::declare(const WorldView& world, const ModuleConfig& config, SignalDeclarer& d) {
    std::string error;
    specs_ = tethersOf(world, error);
    if (!error.empty()) {
        return Status::error(StatusCode::InvalidConfig, error);
    }
    out_.assign(specs_.size(), Out{});
    if (zero_) {
        for (std::size_t k = 0; k < specs_.size(); ++k) {
            const std::string base = "tether/" + specs_[k].name + "/";
            out_[k].tensionAttach =
                d.output(spec(base + "tension_attach", SignalType::Scalar, units::N));
            out_[k].tensionAnchor =
                d.output(spec(base + "tension_anchor", SignalType::Scalar, units::N));
        }
        return {};
    }
    const World& raw = world.raw();
    rest_ = restPoses(raw);
    robotOfBody_ = raw.bodies.robot;
    gravity_ = world.gravityLocal();
    airDensity_ = std::max(0.0, config.get<double>("air_density").value_or(0.0));
    const std::vector<bool> moving = movingBodies(raw);
    const std::size_t bodyCount = raw.bodies.names.size();
    std::vector<bool> wanted(bodyCount, false);
    for (std::size_t c = 0; c < raw.colliders.body.size(); ++c) {
        const BodyId b = raw.colliders.body[c];
        if (b != kWorldBody && b.index < bodyCount && moving[b.index]) {
            wanted[b.index] = true;
        }
    }
    for (const TetherSpec& t : specs_) {
        for (const BodyId b : {t.anchorBody, t.attachBody}) {
            if (b != kWorldBody && b.index < bodyCount && moving[b.index]) {
                wanted[b.index] = true;
            }
        }
    }
    bodies_.clear();
    bodyIndex_.assign(bodyCount, -1);
    for (std::uint32_t b = 0; b < bodyCount; ++b) {
        if (!wanted[b]) {
            continue;
        }
        Body body;
        body.id = BodyId{b};
        body.com = raw.bodies.comLocal[b];
        const std::string base = "body/" + raw.bodies.names[b] + "/";
        SignalSpec pose = spec(base + "pose", SignalType::Pose, units::m, "world_local");
        SignalSpec twist = spec(base + "twist", SignalType::Wrench, units::m_per_s, "world_local");
        pose.feedback = twist.feedback = true;
        body.pose = d.input(pose);
        body.twist = d.input(twist);
        if (!raw.bodies.kinematic[b]) {
            body.wrench =
                d.output(spec(base + "wrench/tether", SignalType::Wrench, units::N, "world_local"));
            body.hasWrench = true;
        }
        bodyIndex_[b] = static_cast<int>(bodies_.size());
        bodies_.push_back(body);
    }
    // Every collider, placed now for scenery, each step for what moves.
    obstacles_.clear();
    colliderInBody_.clear();
    colliderBody_.clear();
    for (std::size_t c = 0; c < raw.colliders.body.size(); ++c) {
        Obstacle o;
        o.type = raw.colliders.shape[c];
        o.size = raw.colliders.size[c];
        const std::uint32_t extra = raw.colliders.hull[c];
        if (o.type == ShapeType::ConvexHull && extra < raw.hulls.size()) {
            o.hull = &raw.hulls[extra];
        } else if (o.type == ShapeType::HeightField && extra < raw.heightfields.size()) {
            o.field = &raw.heightfields[extra];
        }
        o.reach = reachOf(o.type, o.size, o.hull);
        const BodyId b = raw.colliders.body[c];
        o.body = movingIndex(b);
        o.robot = o.body >= 0 ? static_cast<int>(raw.bodies.robot.at(b.index)) : -1;
        const Pose& local = raw.colliders.poseInBody[c];
        if (b == kWorldBody) {
            o.pose = local;
        } else {
            o.pose = compose(rest_.at(b.index), local);
        }
        obstacles_.push_back(o);
        colliderInBody_.push_back(local);
        colliderBody_.push_back(b);
    }
    SignalSpec wind = spec("env/wind/velocity", SignalType::Vec3, units::m_per_s, "world_local");
    wind.optional = true;
    wind.feedback = true;
    wind_ = d.input(wind);
    hasWind_ = true;
    // Outputs per cable; the node array holds the most masses the drum allows.
    cables_.clear();
    for (std::size_t k = 0; k < specs_.size(); ++k) {
        const TetherSpec& t = specs_[k];
        cables_.emplace_back(t.params, Vec3{}, Vec3{1.0, 0.0, 0.0}, bodies_.size());
        const std::string base = "tether/" + t.name + "/";
        Out& o = out_[k];
        o.tensionAttach = d.output(spec(base + "tension_attach", SignalType::Scalar, units::N));
        o.tensionAnchor = d.output(spec(base + "tension_anchor", SignalType::Scalar, units::N));
        o.length = d.output(spec(base + "length", SignalType::Scalar, units::m));
        o.winchSpeed = d.output(spec(base + "winch_speed", SignalType::Scalar, units::m_per_s));
        o.contacts = d.output(spec(base + "contacts", SignalType::Int, units::one));
        o.caught = d.output(spec(base + "caught", SignalType::Bool, units::one));
        SignalSpec nodes = spec(base + "nodes", SignalType::ScalarArray, units::m, "world_local");
        nodes.arrayLength = static_cast<std::uint32_t>(3 * cables_.back().capacity());
        o.nodes = d.output(nodes);
    }
    wrenches_.assign(bodies_.size(), std::array<double, 6>{});
    return {};
}

Pose Tethers::bodyPose(BodyId body, const SignalBus& bus) const {
    if (body == kWorldBody) {
        return Pose{};
    }
    const int i = movingIndex(body);
    if (i >= 0) {
        const std::span<const double> p = bus.readN(bodies_[static_cast<std::size_t>(i)].pose);
        if (p.size() == 7) {
            return Pose{{p[0], p[1], p[2]}, {p[3], p[4], p[5], p[6]}};
        }
    }
    return body.index < rest_.size() ? rest_[body.index] : Pose{};
}

Status Tethers::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    for (std::size_t k = 0; k < specs_.size(); ++k) {
        bus.write(out_[k].tensionAttach, 0.0);
        bus.write(out_[k].tensionAnchor, 0.0);
    }
    if (zero_) {
        return {};
    }
    // Lay each cable out between its ends where the world puts them at the start.
    for (std::size_t k = 0; k < specs_.size(); ++k) {
        const TetherSpec& t = specs_[k];
        const auto at = [&](BodyId b, const Vec3& local) {
            const Pose pose =
                b == kWorldBody ? Pose{} : (b.index < rest_.size() ? rest_[b.index] : Pose{});
            return pointIn(pose, local);
        };
        cables_[k] = Cable(t.params, at(t.anchorBody, t.anchorInBody),
                           at(t.attachBody, t.attachInBody), bodies_.size());
        const auto robotOf = [&](BodyId b) {
            return movingIndex(b) >= 0 ? static_cast<int>(robotOfBody_.at(b.index)) : -1;
        };
        cables_[k].setEndRobots(robotOf(t.anchorBody), robotOf(t.attachBody));
        const Out& o = out_[k];
        bus.write(o.length, cables_[k].length());
        bus.write(o.winchSpeed, 0.0);
        bus.write(o.contacts, 0.0);
        bus.write(o.caught, 0.0);
        nodeBuffer_.assign(3 * cables_[k].capacity(), std::numeric_limits<double>::quiet_NaN());
        const auto nodes = cables_[k].nodes();
        for (std::size_t i = 0; i < nodes.size(); ++i) {
            std::copy(nodes[i].begin(), nodes[i].end(), nodeBuffer_.begin() + 3 * i);
        }
        bus.writeN(o.nodes, nodeBuffer_);
    }
    for (const Body& body : bodies_) {
        if (body.hasWrench) {
            bus.writeN(body.wrench, std::array<double, 6>{});
        }
    }
    return {};
}

Status Tethers::step(const StepContext& context, SignalBus& bus) {
    if (zero_) {
        for (const Out& o : out_) {
            bus.write(o.tensionAttach, 0.0);
            bus.write(o.tensionAnchor, 0.0);
        }
        return {};
    }
    const double dt = context.dt.seconds();
    // Place what moves.
    for (std::size_t c = 0; c < obstacles_.size(); ++c) {
        Obstacle& o = obstacles_[c];
        if (o.body < 0) {
            continue;
        }
        const Body& body = bodies_[static_cast<std::size_t>(o.body)];
        const Pose pose = bodyPose(body.id, bus);
        o.pose = compose(pose, colliderInBody_[c]);
        o.com = pointIn(pose, body.com);
        const std::span<const double> v = bus.readN(body.twist);
        if (v.size() == 6) {
            // The twist's linear part is the body origin's; carry it to the centre of mass.
            const Vec3 w{v[3], v[4], v[5]};
            const Vec3 r = sub(o.com, pose.position);
            const Vec3 spin = cross(w, r);
            o.velocity = {v[0] + spin[0], v[1] + spin[1], v[2] + spin[2]};
            o.angular = w;
        }
    }
    Air air;
    air.gravity = gravity_;
    air.density = airDensity_;
    if (hasWind_) {
        const std::span<const double> w = bus.readN(wind_);
        if (w.size() == 3 && std::isfinite(w[0]) && std::isfinite(w[1]) && std::isfinite(w[2])) {
            air.wind = {w[0], w[1], w[2]};
        }
    }
    for (auto& w : wrenches_) {
        w = {};
    }
    const auto addWrench = [&](BodyId b, const Vec3& force, const Vec3& at, const Pose& pose) {
        const int i = movingIndex(b);
        if (i < 0) {
            return;
        }
        auto& w = wrenches_[static_cast<std::size_t>(i)];
        const Vec3 com = pointIn(pose, bodies_[static_cast<std::size_t>(i)].com);
        const Vec3 torque = cross(sub(at, com), force);
        for (std::size_t a = 0; a < 3; ++a) {
            w.at(a) += force.at(a);
            w.at(3 + a) += torque.at(a);
        }
    };
    for (std::size_t k = 0; k < specs_.size(); ++k) {
        const TetherSpec& t = specs_[k];
        Cable& cable = cables_[k];
        const Pose anchorPose = bodyPose(t.anchorBody, bus);
        const Pose attachPose = bodyPose(t.attachBody, bus);
        const Vec3 anchor = pointIn(anchorPose, t.anchorInBody);
        const Vec3 attach = pointIn(attachPose, t.attachInBody);
        cable.step(dt, anchor, attach, obstacles_, air);
        // What the cable did to the bodies: contact pushes, the pull at each end.
        const auto pushes = cable.pushes();
        for (std::size_t i = 0; i < pushes.size(); ++i) {
            auto& w = wrenches_[i];
            for (std::size_t a = 0; a < 3; ++a) {
                w.at(a) += pushes[i].force.at(a);
                w.at(3 + a) += pushes[i].torque.at(a);
            }
        }
        addWrench(t.attachBody, cable.forceOnAttach(), attach, attachPose);
        addWrench(t.anchorBody, cable.forceOnAnchor(), anchor, anchorPose);
        const auto nodes = cable.nodes();
        const Out& o = out_[k];
        bus.write(o.tensionAttach, cable.tensionAttach());
        bus.write(o.tensionAnchor, cable.tensionAnchor());
        bus.write(o.length, cable.length());
        bus.write(o.winchSpeed, cable.winchSpeed());
        bus.write(o.contacts, static_cast<double>(cable.contacts()));
        bus.write(o.caught, cable.caught() ? 1.0 : 0.0);
        nodeBuffer_.assign(3 * cable.capacity(), std::numeric_limits<double>::quiet_NaN());
        for (std::size_t i = 0; i < nodes.size(); ++i) {
            std::copy(nodes[i].begin(), nodes[i].end(), nodeBuffer_.begin() + 3 * i);
        }
        bus.writeN(o.nodes, nodeBuffer_);
    }
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
        if (bodies_[i].hasWrench) {
            bus.writeN(bodies_[i].wrench, wrenches_[i]);
        }
    }
    return {};
}

Status Tethers::saveState(StateWriter& writer) const {
    std::vector<double> state;
    for (const Cable& cable : cables_) {
        state.clear();
        cable.save(state);
        writer.put(static_cast<std::uint64_t>(state.size()));
        writer.putDoubles(state);
    }
    return {};
}

Status Tethers::restoreState(StateReader& reader) {
    std::vector<double> state;
    for (Cable& cable : cables_) {
        std::uint64_t size = 0;
        if (!reader.get(size) || size > (1U << 24)) {
            return Status::error(StatusCode::Internal, "tether: short state");
        }
        state.assign(static_cast<std::size_t>(size), 0.0);
        if (!reader.getDoubles(state) || !cable.restore(state)) {
            return Status::error(StatusCode::Internal, "tether: state does not fit the cable");
        }
    }
    return {};
}

void Tethers::shiftOrigin(const std::array<double, 3>& shift) {
    for (Cable& cable : cables_) {
        cable.shift({-shift[0], -shift[1], -shift[2]});
    }
}

}  // namespace lagrange::tether
