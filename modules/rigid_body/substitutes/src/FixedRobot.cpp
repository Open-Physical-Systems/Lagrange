#include "lagrange/rigid_body/FixedRobot.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <cmath>
#include <string>

namespace lagrange::rigid_body {

using namespace literals;

namespace {

bool isActuatable(JointType type) {
    return type == JointType::Revolute || type == JointType::Continuous ||
           type == JointType::Prismatic;
}

SignalSpec spec(std::string name, SignalType type, Unit unit, std::string frame = {}) {
    SignalSpec out;
    out.name = std::move(name);
    out.type = type;
    out.unit = unit;
    out.frame = std::move(frame);
    return out;
}

/// An input the substitute can do without (nothing commands a clamped robot).
SignalSpec optionalFlow(std::string name, SignalType type, Unit unit, std::string frame = {}) {
    SignalSpec out = spec(std::move(name), type, unit, std::move(frame));
    out.hold = Hold::WindowAverage;
    out.optional = true;
    return out;
}

}  // namespace

ModuleInfo FixedRobot::info() const {
    return {"rigid_body", "fixed", 0, 1, true, true, CapabilityTier::Tier0};
}

Duration FixedRobot::preferredStep() const {
    return 1_ms;  // as the backend it stands in for, so consumers see the same rates
}

Status FixedRobot::declare(const WorldView& world, const ModuleConfig&, SignalDeclarer& d) {
    const World& raw = world.raw();
    const World::Joints& joints = world.joints();
    for (std::uint32_t j = 0; j < joints.names.size(); ++j) {
        if (!isActuatable(joints.type[j])) {
            continue;
        }
        const bool prismatic = joints.type[j] == JointType::Prismatic;
        const Unit effort = prismatic ? units::N : units::N_m;
        const std::string base = "joint/" + std::string(world.signalInstanceName(joints.prim[j]));
        Joint s;
        s.position = d.output(
            spec(base + "/position", SignalType::Scalar, prismatic ? units::m : units::rad));
        s.velocity = d.output(spec(base + "/velocity", SignalType::Scalar,
                                   prismatic ? units::m_per_s : units::rad_per_s));
        if (joints.actuated[j]) {
            s.torque = d.input(optionalFlow(base + "/torque", SignalType::Scalar, effort));
            s.loadTorque = d.input(optionalFlow(base + "/load_torque", SignalType::Scalar, effort));
            s.acceleration = d.output(spec(base + "/acceleration", SignalType::Scalar,
                                           prismatic ? units::m_per_s2 : units::rad_per_s2));
            s.effectiveInertia = d.output(spec(base + "/effective_inertia", SignalType::Scalar,
                                               prismatic ? units::kg : units::kg_m2));
            s.constraintTorque =
                d.output(spec(base + "/constraint_torque", SignalType::Scalar, effort));
            // The child body's mass along a slide, or its moment of inertia about the axis
            // through its centre of mass (the parallel-axis term is left out: the joint is
            // clamped, so this value only sizes a motor model's load).
            const std::uint32_t child = joints.child[j].index;
            if (prismatic) {
                s.inertia = raw.bodies.mass[child];
            } else {
                const std::array<double, 3>& a = joints.axis[j];
                const std::array<double, 6>& i = raw.bodies.inertiaLocal[child];
                s.inertia = a[0] * a[0] * i[0] + a[1] * a[1] * i[1] + a[2] * a[2] * i[2] +
                            2.0 * (a[0] * a[1] * i[3] + a[0] * a[2] * i[4] + a[1] * a[2] * i[5]);
            }
        }
        joints_.push_back(s);
    }
    const std::vector<bool> moving = movingBodies(raw);
    const std::vector<Pose> rest = restPoses(raw);
    const World::Bodies& bodies = world.bodies();
    const std::array<double, 3> g = world.gravityLocal();
    for (std::uint32_t b = 0; b < bodies.names.size(); ++b) {
        if (!moving[b]) {
            continue;
        }
        const std::string base = "body/" + std::string(world.signalInstanceName(bodies.prim[b]));
        Body s;
        (void)d.input(
            optionalFlow(base + "/external_wrench", SignalType::Wrench, units::N, "world_local"));
        s.pose = d.output(spec(base + "/pose", SignalType::Pose, units::m, "world_local"));
        s.twist =
            d.output(spec(base + "/twist", SignalType::Wrench, units::m_per_s, "world_local"));
        s.acceleration =
            d.output(spec(base + "/acceleration", SignalType::Wrench, units::m_per_s2, "body"));
        const Pose& p = rest[b];
        s.restPose = {p.position[0],    p.position[1],    p.position[2],   p.orientation[0],
                      p.orientation[1], p.orientation[2], p.orientation[3]};
        // At rest the proper acceleration is the support force per mass: -g, in the body frame.
        const std::array<double, 3> up = rotate(conjugate(p.orientation), {-g[0], -g[1], -g[2]});
        s.properAcceleration = {up[0], up[1], up[2], 0.0, 0.0, 0.0};
        bodies_.push_back(s);
    }
    SignalSpec friction = spec("env/friction_scale", SignalType::Scalar, units::one);
    friction.optional = true;
    friction.defaultValue = 1.0;
    (void)d.input(friction);
    return {};
}

Status FixedRobot::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    for (const Joint& j : joints_) {
        bus.write(j.position, 0.0);
        bus.write(j.velocity, 0.0);
        if (j.acceleration.valid()) {
            bus.write(j.acceleration, 0.0);
            bus.write(j.effectiveInertia, j.inertia);
            bus.write(j.constraintTorque, 0.0);
        }
    }
    for (const Body& b : bodies_) {
        bus.writeN(b.pose, b.restPose);
        const std::array<double, 6> zero{};
        bus.writeN(b.twist, zero);
        bus.writeN(b.acceleration, b.properAcceleration);
    }
    return {};
}

Status FixedRobot::step(const StepContext&, SignalBus& bus) {
    for (const Joint& j : joints_) {
        bus.write(j.position, 0.0);
        bus.write(j.velocity, 0.0);
        if (j.acceleration.valid()) {
            const double torque = bus.read(j.torque) + bus.read(j.loadTorque);
            bus.write(j.acceleration, 0.0);
            bus.write(j.effectiveInertia, j.inertia);
            bus.write(j.constraintTorque, -torque);  // the clamp holds the joint still
        }
    }
    for (const Body& b : bodies_) {
        bus.writeN(b.pose, b.restPose);
        const std::array<double, 6> zero{};
        bus.writeN(b.twist, zero);
        bus.writeN(b.acceleration, b.properAcceleration);
    }
    return {};
}

}  // namespace lagrange::rigid_body
