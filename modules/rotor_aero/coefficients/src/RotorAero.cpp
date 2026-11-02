// The rotor_aero domain: the coefficient model and its ideal substitute (aerodynamics.md §2, §5).
#include "lagrange/rotor_aero/RotorAero.hpp"
#include "lagrange/services/Log.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <numbers>

namespace lagrange::rotor_aero {

using namespace literals;

namespace {

SignalSpec scalar(std::string name, Unit unit) {
    SignalSpec spec;
    spec.name = std::move(name);
    spec.unit = unit;
    return spec;
}

std::array<double, 3> cross(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

}  // namespace

Load staticLoad(const RotorParams& p, double omega, double rho) {
    const double n = omega / (2.0 * std::numbers::pi);  // rev/s
    const double d = 2.0 * p.radius;
    const double d4 = d * d * d * d;
    const double thrust = p.thrustCoeff * rho * n * n * d4;
    const double torque = p.powerCoeff * rho * n * n * d4 * d / (2.0 * std::numbers::pi);
    // Thrust along the axis when turning in the rotor's direction; the drag torque always
    // opposes the rotation.
    return {std::copysign(thrust, p.spinSign * omega), -std::copysign(torque, omega)};
}

double groundEffect(double radius, double height) {
    const double z = std::max(height, kGroundEffectMinHeight * radius);
    const double ratio = radius / (4.0 * z);
    return 1.0 / (1.0 - ratio * ratio);
}

ModuleInfo RotorAero::info() const {
    if (ideal_) {
        return {"rotor_aero", "ideal", 0, 1, true, true, CapabilityTier::Tier0};
    }
    return {"rotor_aero", bem_ ? "bem" : "coefficients", 0, 1, false, true, CapabilityTier::Tier0};
}

Duration RotorAero::preferredStep() const {
    return 1_ms;
}

Status RotorAero::declare(const WorldView& world, const ModuleConfig& config, SignalDeclarer& d) {
    rotors_.clear();
    const std::optional<double> rho = config.get<double>(kAirDensityKey);
    if (!world.instancesOf("RotorAPI_1").empty() && (!rho || *rho <= 0.0)) {
        return Status::error(StatusCode::InvalidConfig,
                             "rotors need the site's air density (profile rotor_aero.air_density, "
                             "kg/m³; ISA sea level is 1.225); it has no default");
    }
    rho_ = rho.value_or(0.0);
    lag_ = ideal_ ? 0.0 : std::max(0.0, config.get<double>("inflow_time_constant").value_or(0.0));
    bem_ = !ideal_ && config.get<std::string>("fidelity").value_or("coefficients") == "bem";
    speedOfSound_ = std::max(0.0, config.get<double>("speed_of_sound").value_or(0.0));
    gravity_ = world.gravityLocal();
    wind_ = bem_ && config.get<bool>("wind").value_or(false);
    // Each rotor's own air (the aerodynamics domain's wind plus other robots' downwash).
    rotorAir_ = bem_ && config.get<bool>("rotor_air").value_or(false);
    if (wind_) {
        SignalSpec wind = scalar("env/wind/velocity", units::m_per_s);
        wind.type = SignalType::Vec3;
        wind.frame = "world_local";
        wind.feedback = true;
        windIn_ = d.input(wind);
    }
    std::map<std::string, std::shared_ptr<const PropellerTable>> tables;  // by file
    groundEffect_ = !ideal_ && config.get<bool>("ground_effect").value_or(true);

    // The ground: the first plane fixed to the world.
    const World::Colliders& colliders = world.colliders();
    hasGround_ = false;
    for (std::size_t c = 0; c < colliders.body.size() && !hasGround_; ++c) {
        if (colliders.body[c] == kWorldBody && colliders.shape[c] == ShapeType::Plane) {
            hasGround_ = true;
            groundPoint_ = colliders.poseInBody[c].position;
            groundNormal_ = rotate(colliders.poseInBody[c].orientation, {0.0, 0.0, 1.0});
        }
    }

    std::vector<std::string> problems;
    const World::Joints& joints = world.joints();
    const World::Bodies& bodies = world.bodies();
    for (const SchemaInstance* s : world.instancesOf("RotorAPI_1")) {
        const std::string where(world.primPath(s->prim));
        const auto j = std::ranges::find(joints.prim, s->prim) - joints.prim.begin();
        if (static_cast<std::size_t>(j) >= joints.prim.size()) {
            problems.push_back(where + ": RotorAPI_1 belongs on a joint prim");
            continue;
        }
        const auto radius = s->real("aero:rotor:radius");
        const auto ct = s->real("aero:rotor:thrustCoeff");
        const auto cp = s->real("aero:rotor:powerCoeff");
        const auto blades = s->integer("aero:rotor:bladeCount");
        const auto direction = s->token("aero:rotor:direction");
        if (!radius || *radius <= 0.0 || !ct || *ct < 0.0 || !cp || *cp < 0.0 || !blades ||
            *blades < 1 || !direction || (*direction != "cw" && *direction != "ccw")) {
            problems.push_back(where +
                               ": a rotor needs radius > 0, thrustCoeff and powerCoeff >= 0, "
                               "bladeCount >= 1 and direction cw or ccw");
            continue;
        }
        if (joints.type[j] != JointType::Revolute && joints.type[j] != JointType::Continuous) {
            problems.push_back(where + ": a rotor turns on a revolute or continuous joint");
            continue;
        }
        Rotor r;
        r.joint = std::string(world.signalInstanceName(s->prim));
        r.params = {*radius, *ct, *cp, *direction == "ccw" ? 1.0 : -1.0};
        r.mountSign = s->token("aero:rotor:mount").value_or("puller") == "pusher" ? -1.0 : 1.0;
        if (bem_) {
            const auto file = s->assetPath("aero:rotor:propeller");
            if (!file) {
                problems.push_back(where +
                                   ": the bem fidelity needs the rotor's propeller file "
                                   "(aero:rotor:propeller)");
                continue;
            }
            std::string error;
            const Propeller p = loadPropeller(std::string(*file), error);
            if (!error.empty()) {
                problems.push_back(where + ": " + error);
                continue;
            }
            if (std::abs(p.radius - *radius) > 0.01 * *radius || p.blades != *blades) {
                problems.push_back(
                    where + ": radius and bladeCount must be the propeller file's (" +
                    std::to_string(p.radius) + " m, " + std::to_string(p.blades) + " blades)");
                continue;
            }
            r.handSign = p.hand == "ccw" ? 1.0 : -1.0;
            if (r.handSign != r.params.spinSign) {
                services::log(services::LogLevel::Warning, "rotor_aero",
                              where + ": a " + p.hand + " propeller (" + p.name +
                                  ") on a motor turning " + std::string(*direction) +
                                  ": it turns the wrong way and gives little thrust");
            }
            auto& table = tables[std::string(*file)];
            if (!table) {
                table = std::make_shared<const PropellerTable>(
                    PropellerTable::build(p, speedOfSound_ > 0.0));
            }
            r.propeller = table;
            if (p.folding) {
                r.folding = FoldingBlade(p);
            }
        } else if (const auto file = s->assetPath("aero:rotor:propeller"); file && !ideal_) {
            // Folding blades are mechanics, whatever the aerodynamics: the file gives them.
            std::string error;
            const Propeller p = loadPropeller(std::string(*file), error);
            if (error.empty() && p.folding) {
                r.folding = FoldingBlade(p);
            }
        }
        const std::array<double, 3>& a = joints.axis[j];
        const double norm = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
        r.axis = {a[0] / norm, a[1] / norm, a[2] / norm};
        const std::size_t child = joints.child[j].index;
        r.com = bodies.comLocal[child];
        const std::string link(world.signalInstanceName(bodies.prim[child]));
        const std::string base = "joint/" + r.joint + "/";
        SignalSpec velocity = scalar(base + "velocity", units::rad_per_s);
        velocity.feedback = true;
        r.velocity = d.input(velocity);
        SignalSpec pose = scalar("body/" + link + "/pose", units::m);
        pose.type = SignalType::Pose;
        pose.frame = "world_local";
        pose.feedback = true;
        r.pose = d.input(pose);
        if (bem_) {
            SignalSpec twist = scalar("body/" + link + "/twist", units::m_per_s);
            twist.type = SignalType::Wrench;
            twist.frame = "world_local";
            twist.feedback = true;
            r.twist = d.input(twist);
        }
        const std::string rotor = "rotor/" + r.joint + "/";
        r.thrustOut = d.output(scalar(rotor + "thrust", units::N));
        r.torqueOut = d.output(scalar(rotor + "aero_torque", units::N_m));
        r.inducedOut = d.output(scalar(rotor + "induced_velocity", units::m_per_s));
        SignalSpec thrustAxis = scalar(rotor + "thrust_axis", units::one);
        thrustAxis.type = SignalType::Vec3;
        thrustAxis.frame = "world_local";
        r.axisOut = d.output(thrustAxis);
        if (rotorAir_) {
            SignalSpec air = scalar(rotor + "air_velocity", units::m_per_s);
            air.type = SignalType::Vec3;
            air.frame = "world_local";
            air.feedback = true;
            r.airIn = d.input(air);
        }
        SignalSpec wrench = scalar("body/" + link + "/wrench/rotor", units::N);
        wrench.type = SignalType::Wrench;
        wrench.frame = "world_local";
        r.wrenchOut = d.output(wrench);
        r.loadOut = d.output(scalar(base + "load_torque", units::N_m));
        if (r.folding) {
            r.bladeAngleOut = d.output(scalar(rotor + "blade_angle", units::rad));
        }
        rotors_.push_back(std::move(r));
    }
    if (!problems.empty()) {
        std::string message = "the rotors cannot be built:";
        for (const std::string& p : problems) {
            message += "\n  " + p;
        }
        return Status::error(StatusCode::InvalidConfig, message);
    }
    return {};
}

Status RotorAero::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    for (Rotor& r : rotors_) {
        r.thrust = 0.0;
        for (const SignalId id : {r.thrustOut, r.torqueOut, r.inducedOut, r.loadOut}) {
            bus.write(id, 0.0);
        }
        bus.writeN(r.wrenchOut, std::array<double, 6>{});
        bus.writeN(r.axisOut, std::array<double, 3>{});
        if (r.folding) {
            // Stopped: as its weight hangs it, with the link as it rests (axis in the world's
            // frame, as robots are built).
            const double gravityAlong =
                r.mountSign *
                (gravity_[0] * r.axis[0] + gravity_[1] * r.axis[1] + gravity_[2] * r.axis[2]);
            r.bladeAngle = r.folding->angle(0.0, 0.0, 0.0, gravityAlong);
            bus.write(r.bladeAngleOut, r.bladeAngle);
        }
    }
    return {};
}

Status RotorAero::step(const StepContext& context, SignalBus& bus) {
    const double dt = context.dt.seconds();
    // The exact first-order response over the step to a thrust held at its new value.
    const double keep = lag_ > 0.0 ? std::exp(-dt / lag_) : 0.0;
    for (Rotor& r : rotors_) {
        const double omega = bus.read(r.velocity);
        const std::span<const double> v = bus.readN(r.pose);
        const Pose hub{{v[0], v[1], v[2]}, {v[3], v[4], v[5], v[6]}};
        const std::array<double, 3> jointAxis = rotate(hub.orientation, r.axis);
        // Thrust direction: along the joint axis for a puller, against it for a pusher.
        const std::array<double, 3> axis{r.mountSign * jointAxis[0], r.mountSign * jointAxis[1],
                                         r.mountSign * jointAxis[2]};
        Load load;
        double induced = 0.0;
        std::array<double, 3> inPlane{};    // N, world: the rotor's in-plane force (bem)
        std::array<double, 3> hubMoment{};  // N·m, world: its hub moment (bem)
        if (r.propeller) {
            // The propeller's rotation in its own hand about its thrust direction, and the
            // hub's speed into the air along it (the air arriving from the front).
            const double spin = r.handSign * r.mountSign * omega;
            // The hub's velocity through the air: axial along the thrust, the rest across the disc.
            const std::span<const double> hubTwist = bus.readN(r.twist);
            std::array<double, 3> through{hubTwist[0], hubTwist[1], hubTwist[2]};
            if (rotorAir_ || wind_) {
                const std::span<const double> w = bus.readN(rotorAir_ ? r.airIn : windIn_);
                for (std::size_t k = 0; k < 3; ++k) {
                    through[k] -= w[k];
                }
            }
            const double axial = through[0] * axis[0] + through[1] * axis[1] + through[2] * axis[2];
            const std::array<double, 3> across{through[0] - axial * axis[0],
                                               through[1] - axial * axis[1],
                                               through[2] - axial * axis[2]};
            const double edgewise = std::hypot(across[0], across[1], across[2]);
            BemLoad bem = r.propeller->at(spin, axial, {rho_, speedOfSound_}, edgewise);
            if (edgewise > 0.0) {
                // e1: the way the air moves across the disc; e2 = t × e1 in the propeller's hand.
                const std::array<double, 3> e1{-across[0] / edgewise, -across[1] / edgewise,
                                               -across[2] / edgewise};
                const std::array<double, 3> t2 = cross(axis, e1);
                // A cw propeller is the mirror image of a ccw one: e2 turns over, and with it
                // the sense of a moment (a pseudovector), so its rolling moment is the other way.
                const double hand = r.handSign;
                for (std::size_t k = 0; k < 3; ++k) {
                    inPlane[k] = bem.inPlane[0] * e1[k] + bem.inPlane[1] * hand * t2[k];
                    hubMoment[k] = hand * bem.hubMoment[0] * e1[k] + bem.hubMoment[1] * t2[k];
                }
            }
            if (r.folding) {
                // Blades on their hinges: coned or folded, the disc gives the axial share.
                const double gravityAlong =
                    gravity_[0] * axis[0] + gravity_[1] * axis[1] + gravity_[2] * axis[2];
                r.bladeAngle = r.folding->angle(omega, bem.thrust, bem.thrustRadius, gravityAlong);
                bem.thrust *= std::cos(r.bladeAngle);
                bem.torque *= std::cos(r.bladeAngle);
                bus.write(r.bladeAngleOut, r.bladeAngle);
            }
            // The air's torque about the joint axis: against the propeller's rotation in its hand.
            load = {bem.thrust, -r.handSign * r.mountSign * bem.torque};
            induced = bem.inducedVelocity;
        } else {
            // About the thrust direction, then back about the joint axis.
            const Load l = staticLoad(r.params, r.mountSign * omega, rho_);
            load = {l.thrust, r.mountSign * l.torque};
            if (r.folding) {
                // The coefficients have no thrust distribution: it acts at 0.75 R, the usual
                // representative radius of a propeller.
                constexpr double kThrustRadius = 0.75;
                const double gravityAlong =
                    gravity_[0] * axis[0] + gravity_[1] * axis[1] + gravity_[2] * axis[2];
                r.bladeAngle = r.folding->angle(omega, load.thrust, kThrustRadius * r.params.radius,
                                                gravityAlong);
                load.thrust *= std::cos(r.bladeAngle);
                load.torque *= std::cos(r.bladeAngle);
                bus.write(r.bladeAngleOut, r.bladeAngle);
            }
        }
        double target = load.thrust;
        if (groundEffect_ && hasGround_) {
            const std::array<double, 3> above{hub.position[0] - groundPoint_[0],
                                              hub.position[1] - groundPoint_[1],
                                              hub.position[2] - groundPoint_[2]};
            const double height = above[0] * groundNormal_[0] + above[1] * groundNormal_[1] +
                                  above[2] * groundNormal_[2];
            target *= groundEffect(r.params.radius, height);
        }
        r.thrust = keep * r.thrust + (1.0 - keep) * target;
        const std::array<double, 3> force{r.thrust * axis[0] + inPlane[0],
                                          r.thrust * axis[1] + inPlane[1],
                                          r.thrust * axis[2] + inPlane[2]};
        // Applied at the link's origin; the backend takes torques about the centre of mass.
        const std::array<double, 3> com = rotate(hub.orientation, r.com);
        const std::array<double, 3> moment = cross({-com[0], -com[1], -com[2]}, force);
        bus.writeN(r.wrenchOut,
                   std::array<double, 6>{force[0], force[1], force[2], moment[0] + hubMoment[0],
                                         moment[1] + hubMoment[1], moment[2] + hubMoment[2]});
        const double area = std::numbers::pi * r.params.radius * r.params.radius;
        bus.write(r.thrustOut, r.thrust);
        bus.writeN(r.axisOut, axis);
        bus.write(r.torqueOut, load.torque);
        bus.write(r.loadOut, load.torque);
        bus.write(r.inducedOut,
                  r.propeller ? induced : std::sqrt(std::abs(r.thrust) / (2.0 * rho_ * area)));
    }
    return {};
}

Status RotorAero::saveState(StateWriter& writer) const {
    for (const Rotor& r : rotors_) {
        writer.put(r.thrust);
    }
    return {};
}

Status RotorAero::restoreState(StateReader& reader) {
    bool ok = true;
    for (Rotor& r : rotors_) {
        ok = ok && reader.get(r.thrust);
    }
    return ok ? Status{} : Status::error(StatusCode::InvalidConfig, "rotor state truncated");
}

std::vector<std::string> RotorAero::rotors() const {
    std::vector<std::string> out;
    for (const Rotor& r : rotors_) {
        out.push_back(r.joint);
    }
    return out;
}

}  // namespace lagrange::rotor_aero
