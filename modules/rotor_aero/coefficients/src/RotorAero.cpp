// The rotor_aero domain: the coefficient model and its ideal substitute (aerodynamics.md §2, §5).
#include "lagrange/rotor_aero/RotorAero.hpp"
#include "lagrange/services/Log.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numbers>
#include <span>
#include <string>

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

/// The moving water surface's height (m) under `at`, bilinear between cell centres of a grid
/// of `cells` a side (as hydrodynamics::gridHeight); 0 outside it or without one.
double surfaceHeight(std::span<const double> heights, std::span<const double> corner, int cells,
                     double cell, const std::array<double, 3>& at) {
    const auto n = static_cast<std::size_t>(cells);
    if (corner.size() != 2 || heights.size() != n * n || !std::isfinite(corner[0]) ||
        !std::isfinite(corner[1])) {
        return 0.0;
    }
    const double fx = (at[0] - corner[0]) / cell - 0.5;
    const double fy = (at[1] - corner[1]) / cell - 0.5;
    if (!(fx >= 0.0 && fy >= 0.0 && fx <= cells - 1 && fy <= cells - 1)) {
        return 0.0;
    }
    const int col = std::min(static_cast<int>(fx), cells - 2);
    const int row = std::min(static_cast<int>(fy), cells - 2);
    const double tx = fx - col;
    const double ty = fy - row;
    const auto h = [&](int x, int y) {
        return heights[static_cast<std::size_t>(y) * n + static_cast<std::size_t>(x)];
    };
    return (1.0 - ty) * ((1.0 - tx) * h(col, row) + tx * h(col + 1, row)) +
           ty * ((1.0 - tx) * h(col, row + 1) + tx * h(col + 1, row + 1));
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

double advanceRatio(const RotorParams& p, double omega, double axial) {
    const double n = std::abs(omega) / (2.0 * std::numbers::pi);
    const double d = 2.0 * p.radius;
    if (n * d * kMaxAdvance <= std::abs(axial)) {
        return std::copysign(kMaxAdvance, axial);
    }
    return axial / (n * d);
}

namespace {
/// y(x) on the table, linear between points, held at the ends.
double onChart(const std::vector<double>& xs, const std::vector<double>& ys, double x) {
    if (x <= xs.front()) {
        return ys.front();
    }
    if (x >= xs.back()) {
        return ys.back();
    }
    const auto above = std::upper_bound(xs.begin(), xs.end(), x);
    const auto k = static_cast<std::size_t>(above - xs.begin()) - 1;
    return ys[k] + (x - xs[k]) / (xs[k + 1] - xs[k]) * (ys[k + 1] - ys[k]);
}
}  // namespace

Load advanceLoad(const RotorParams& p, double omega, double rho, double axial) {
    if (p.advance.size() < 2) {
        return staticLoad(p, omega, rho);
    }
    const double n = omega / (2.0 * std::numbers::pi);  // rev/s
    const double d = 2.0 * p.radius;
    const double d4 = d * d * d * d;
    const double j = advanceRatio(p, omega, axial);
    // A chart is measured turning the right way; turned the other way the same chart, mirrored,
    // stands in (a simplification, as staticLoad's).
    const double thrust = onChart(p.advance, p.thrustTable, j) * rho * n * n * d4;
    const double torque = onChart(p.advance, p.torqueTable, j) * rho * n * n * d4 * d;
    return {std::copysign(1.0, p.spinSign * omega) * thrust, -std::copysign(torque, omega)};
}

double immersion(double depth, double extent) {
    if (extent <= 1e-9) {
        return depth > 0.0 ? 1.0 : 0.0;
    }
    const double s = std::clamp(depth / extent, -1.0, 1.0);
    return 0.5 + (std::asin(s) + s * std::sqrt(1.0 - s * s)) / std::numbers::pi;
}

double groundEffect(double radius, double height) {
    const double z = std::max(height, kGroundEffectMinHeight * radius);
    const double ratio = radius / (4.0 * z);
    return 1.0 / (1.0 - ratio * ratio);
}

double ceilingEffect(double radius, double distance) {
    if (distance <= 0.0 || distance >= kCeilingReach * radius) {
        return 1.0;
    }
    const double delta = std::min(radius / distance, kCeilingMaxRatio);
    // Their momentum balance (eq. 5 with eq. 9's α0): ½u² − u − α0·δ²/16 = 0 for u = v∞/v_i,
    // and γ = u/2.
    const double gamma = 0.5 + 0.5 * std::sqrt(1.0 + kCeilingAsymmetry * delta * delta / 8.0);
    return std::cbrt(gamma * gamma);
}

double damagedThrustShare(double damage, int blades) {
    const double kept = 1.0 - std::clamp(damage, 0.0, 1.0);
    return 1.0 - (1.0 - kept * kept * kept) / std::max(blades, 1);
}

double damagedTorqueShare(double damage, int blades) {
    const double kept = 1.0 - std::clamp(damage, 0.0, 1.0);
    return 1.0 - (1.0 - kept * kept * kept * kept) / std::max(blades, 1);
}

double imbalanceForce(double bladeMass, double radius, double damage, double omega) {
    const double d = std::clamp(damage, 0.0, 1.0);
    return d * bladeMass * radius * (1.0 - d / 2.0) * omega * omega;
}

double thrustAsymmetryMoment(double thrust, double radius, double damage, int blades) {
    const double kept = 1.0 - std::clamp(damage, 0.0, 1.0);
    return thrust / std::max(blades, 1) * 0.75 * radius * (1.0 - kept * kept * kept * kept);
}

double dragAsymmetryForce(double torque, double radius, double damage, int blades) {
    const double kept = 1.0 - std::clamp(damage, 0.0, 1.0);
    return 4.0 * torque / (3.0 * radius * std::max(blades, 1)) * (1.0 - kept * kept * kept);
}

double estimatedBladeMass(double radius, int blades) {
    const double scale = radius / kReferencePropellerRadius;
    return kReferencePropellerMass * kBladeMassShare * scale * scale * scale / std::max(blades, 1);
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
    densityIn_ = SignalId{};
    if (config.get<bool>(kWeatherAirKey).value_or(false)) {
        SignalSpec density = scalar("env/air_density", units::kg_per_m3);
        density.feedback = true;
        densityIn_ = d.input(density);
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

    // Water (WaterAPI_1): its surface is the ground for a rotor above it (rotors.md, "Over
    // water"). The prim's position is the still surface's middle; 10 m a side without a size,
    // as the water domain reads it.
    water_.clear();
    for (const SchemaInstance* s : config.get<bool>("water_surface").value_or(true)
                                       ? world.instancesOf("WaterAPI_1")
                                       : std::vector<const SchemaInstance*>{}) {
        constexpr double kDefaultSize = 10.0;       // m
        constexpr double kDefaultDepth = 2.0;       // m, WaterAPI_1's fallback
        constexpr double kDefaultDensity = 1000.0;  // kg/m³, WaterAPI_1's fallback
        Pond pond{world.raw().prims.poseInBody.at(s->prim.index).position,
                  s->real("water:sizeX").value_or(kDefaultSize) / 2.0,
                  s->real("water:sizeY").value_or(kDefaultSize) / 2.0,
                  s->real("water:depth").value_or(kDefaultDepth),
                  s->real("water:density").value_or(kDefaultDensity),
                  {},
                  {}};
        surfaceCells_ = static_cast<int>(config.get<std::int64_t>("surface_cells").value_or(0));
        surfaceCell_ = config.get<double>("surface_cell").value_or(0.0);
        if (surfaceCells_ > 1 && surfaceCell_ > 0.0) {
            const std::string base =
                "water/" + world.raw().prims.instanceNames.at(s->prim.index) + "/";
            SignalSpec heights = scalar(base + "surface", units::m);
            heights.type = SignalType::ScalarArray;
            heights.frame = "world_local";
            heights.arrayLength = static_cast<std::uint32_t>(surfaceCells_ * surfaceCells_);
            heights.optional = heights.feedback = true;
            SignalSpec origin = heights;
            origin.name = base + "surface_origin";
            origin.arrayLength = 2;
            origin.defaultValue = std::numeric_limits<double>::quiet_NaN();
            pond.heights = d.input(heights);
            pond.origin = d.input(origin);
        }
        water_.push_back(pond);
    }

    // Ceilings: the fixed boxes of the scenery (on the world, or on a body that does not move,
    // placed through its chain of fixed parents), for the ceiling effect.
    ceilings_.clear();
    {
        const World& raw = world.raw();
        const std::vector<bool> moving = movingBodies(raw);
        for (std::size_t c = 0; c < colliders.body.size(); ++c) {
            const BodyId b = colliders.body[c];
            if (colliders.shape[c] != ShapeType::Box || (b != kWorldBody && moving[b.index])) {
                continue;
            }
            Pose pose = colliders.poseInBody[c];
            for (BodyId up = b; up != kWorldBody;) {
                pose = compose(raw.bodies.poseInParent[up.index], pose);
                up = raw.bodies.parent[up.index];
            }
            ceilings_.push_back({pose.position, pose.orientation, colliders.size[c]});
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
        r.blades = static_cast<int>(*blades);
        r.bladeMass = estimatedBladeMass(*radius, r.blades);
        // An open-water chart (a marine thruster's, or any propeller's K_T(J), K_Q(J)).
        {
            const auto j0 = s->realArray("aero:rotor:advanceRatios");
            const auto kt = s->realArray("aero:rotor:thrustCoeffs");
            const auto kq = s->realArray("aero:rotor:torqueCoeffs");
            if (!j0.empty() || !kt.empty() || !kq.empty()) {
                bool rising = j0.size() >= 2;
                for (std::size_t k = 1; k < j0.size(); ++k) {
                    rising = rising && j0[k] > j0[k - 1];
                }
                if (!rising || kt.size() != j0.size() || kq.size() != j0.size()) {
                    problems.push_back(where +
                                       ": aero:rotor:advanceRatios (two or more, rising), "
                                       "thrustCoeffs and torqueCoeffs must be as long");
                    continue;
                }
                r.params.advance.assign(j0.begin(), j0.end());
                r.params.thrustTable.assign(kt.begin(), kt.end());
                r.params.torqueTable.assign(kq.begin(), kq.end());
            }
        }
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
                r.bladeMass = p.folding->bladeMass;
            }
        } else if (const auto file = s->assetPath("aero:rotor:propeller"); file && !ideal_) {
            // Folding blades are mechanics, whatever the aerodynamics: the file gives them.
            std::string error;
            const Propeller p = loadPropeller(std::string(*file), error);
            if (error.empty() && p.folding) {
                r.folding = FoldingBlade(p);
                r.bladeMass = p.folding->bladeMass;
            }
        }
        const std::array<double, 3>& a = joints.axis[j];
        const double norm = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
        r.axis = {a[0] / norm, a[1] / norm, a[2] / norm};
        {
            // The damaged blade: along the child frame's axis least along the joint's, made
            // square to it (any direction in the disc will do; it turns with the link).
            std::size_t k = 0;
            for (std::size_t i = 1; i < 3; ++i) {
                if (std::abs(r.axis[i]) < std::abs(r.axis[k])) {
                    k = i;
                }
            }
            std::array<double, 3> e{};
            e[k] = 1.0;
            for (std::size_t i = 0; i < 3; ++i) {
                e[i] -= r.axis[k] * r.axis[i];
            }
            const double en = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
            r.bladeDir = {e[0] / en, e[1] / en, e[2] / en};
        }
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
        if (bem_ || !r.params.advance.empty()) {
            SignalSpec twist = scalar("body/" + link + "/twist", units::m_per_s);
            twist.type = SignalType::Wrench;
            twist.frame = "world_local";
            twist.feedback = true;
            r.twist = d.input(twist);
        }
        const std::string rotor = "rotor/" + r.joint + "/";
        if (!ideal_) {
            // A blade's lost share (rotor_aero.md §10): branch strikes write it; none, it is whole.
            SignalSpec damage = scalar(rotor + "damage", units::one);
            damage.optional = damage.feedback = true;
            r.damageIn = d.input(damage);
        }
        r.thrustOut = d.output(scalar(rotor + "thrust", units::N));
        r.torqueOut = d.output(scalar(rotor + "aero_torque", units::N_m));
        r.inducedOut = d.output(scalar(rotor + "induced_velocity", units::m_per_s));
        if (!water_.empty()) {
            r.immersionOut = d.output(scalar(rotor + "immersion", units::one));
        }
        if (!r.params.advance.empty()) {
            r.advanceOut = d.output(scalar(rotor + "advance_ratio", units::one));
        }
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
        for (const SignalId id : {r.immersionOut, r.advanceOut}) {
            if (id.valid()) {
                bus.write(id, 0.0);
            }
        }
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
    if (densityIn_.valid()) {
        if (const double rho = bus.read(densityIn_); rho > 0.0) {
            rho_ = rho;  // the weather's air, once it has written it
        }
    }
    for (Rotor& r : rotors_) {
        const double omega = bus.read(r.velocity);
        const std::span<const double> v = bus.readN(r.pose);
        const Pose hub{{v[0], v[1], v[2]}, {v[3], v[4], v[5], v[6]}};
        const std::array<double, 3> jointAxis = rotate(hub.orientation, r.axis);
        // Thrust direction: along the joint axis for a puller, against it for a pusher.
        const std::array<double, 3> axis{r.mountSign * jointAxis[0], r.mountSign * jointAxis[1],
                                         r.mountSign * jointAxis[2]};
        // In water (WaterAPI_1): the share of the disc under the surface, and the fluid's density
        // over the disc (a thruster breaking the surface keeps that share of its water's thrust).
        double wet = 0.0;
        double rho = rho_;
        for (const Pond& w : water_) {
            if (std::abs(hub.position[0] - w.centre[0]) > w.halfX ||
                std::abs(hub.position[1] - w.centre[1]) > w.halfY) {
                continue;
            }
            double surface = w.centre[2];
            if (w.heights.valid()) {
                surface += surfaceHeight(bus.readN(w.heights), bus.readN(w.origin), surfaceCells_,
                                         surfaceCell_, hub.position);
            }
            const double depth = surface - hub.position[2];
            if (depth > w.depth + r.params.radius) {
                continue;  // under the water's floor: not in it
            }
            const double extent =
                r.params.radius * std::sqrt(std::max(0.0, 1.0 - jointAxis[2] * jointAxis[2]));
            const double share = immersion(depth, extent);
            if (share > wet) {
                wet = share;
                rho = share * w.density + (1.0 - share) * rho_;
            }
        }
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
            BemLoad bem = r.propeller->at(spin, axial, {rho, speedOfSound_}, edgewise);
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
            // The hub's speed into the fluid along the thrust direction, for the chart.
            double axial = 0.0;
            if (r.twist.valid()) {
                const std::span<const double> hubTwist = bus.readN(r.twist);
                axial = hubTwist[0] * axis[0] + hubTwist[1] * axis[1] + hubTwist[2] * axis[2];
                if (r.advanceOut.valid()) {
                    bus.write(r.advanceOut, advanceRatio(r.params, omega, axial));
                }
            }
            const Load l = advanceLoad(r.params, r.mountSign * omega, rho, axial);
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
        if (groundEffect_ && wet == 0.0 && (hasGround_ || !water_.empty())) {
            // The nearest surface under the hub: the ground, or a body of water's surface.
            double height = std::numeric_limits<double>::infinity();
            if (hasGround_) {
                const std::array<double, 3> above{hub.position[0] - groundPoint_[0],
                                                  hub.position[1] - groundPoint_[1],
                                                  hub.position[2] - groundPoint_[2]};
                height = above[0] * groundNormal_[0] + above[1] * groundNormal_[1] +
                         above[2] * groundNormal_[2];
            }
            for (const Pond& w : water_) {
                double over = hub.position[2] - w.centre[2];
                if (w.heights.valid()) {
                    over -= surfaceHeight(bus.readN(w.heights), bus.readN(w.origin), surfaceCells_,
                                          surfaceCell_, hub.position);
                }
                if (over >= 0.0 && std::abs(hub.position[0] - w.centre[0]) <= w.halfX &&
                    std::abs(hub.position[1] - w.centre[1]) <= w.halfY) {
                    height = std::min(height, over);
                }
            }
            if (std::isfinite(height)) {
                target *= groundEffect(r.params.radius, height);
            }
        }
        if (groundEffect_ && !ceilings_.empty()) {
            // The nearest fixed box straight up the thrust axis (slab test in its frame).
            double nearest = std::numeric_limits<double>::infinity();
            for (const Ceiling& box : ceilings_) {
                const std::array<double, 4> back = conjugate(box.orientation);
                const std::array<double, 3> o =
                    rotate(back, {hub.position[0] - box.centre[0], hub.position[1] - box.centre[1],
                                  hub.position[2] - box.centre[2]});
                const std::array<double, 3> d = rotate(back, axis);
                double enter = 0.0;
                double leave = std::numeric_limits<double>::infinity();
                for (std::size_t k = 0; k < 3 && enter <= leave; ++k) {
                    if (std::abs(d[k]) < 1e-12) {
                        if (std::abs(o[k]) > box.half[k]) {
                            leave = -1.0;
                        }
                        continue;
                    }
                    double a = (-box.half[k] - o[k]) / d[k];
                    double b = (box.half[k] - o[k]) / d[k];
                    if (a > b) {
                        std::swap(a, b);
                    }
                    enter = std::max(enter, a);
                    leave = std::min(leave, b);
                }
                if (enter <= leave && enter > 0.0) {
                    nearest = std::min(nearest, enter);
                }
            }
            target *= ceilingEffect(r.params.radius, nearest);
        }
        r.thrust = keep * r.thrust + (1.0 - keep) * target;
        // A damaged blade (rotor_aero.md §10): less thrust and torque, and forces that turn with
        // the rotor. Nothing changes, bit for bit, while it is whole.
        double thrust = r.thrust;
        const double damage = r.damageIn.valid() ? std::clamp(bus.read(r.damageIn), 0.0, 1.0) : 0.0;
        if (damage > 0.0) {
            const double radius = r.params.radius;
            const std::array<double, 3> blade = rotate(hub.orientation, r.bladeDir);
            // Where the damaged blade moves: about the joint axis, the way it turns.
            const std::array<double, 3> turn = cross(jointAxis, blade);
            const double way = omega < 0.0 ? -1.0 : 1.0;
            const double mass = imbalanceForce(r.bladeMass, radius, damage, omega);
            const double drag = dragAsymmetryForce(std::abs(load.torque), radius, damage, r.blades);
            // The lost thrust's moment about the hub, out of the balance of the whole blades'.
            const double lift = thrustAsymmetryMoment(r.thrust, radius, damage, r.blades);
            const std::array<double, 3> lever = cross(blade, axis);
            for (std::size_t k = 0; k < 3; ++k) {
                inPlane[k] += -mass * blade[k] + way * drag * turn[k];
                hubMoment[k] -= lift * lever[k];
            }
            thrust *= damagedThrustShare(damage, r.blades);
            load.torque *= damagedTorqueShare(damage, r.blades);
        }
        const std::array<double, 3> force{thrust * axis[0] + inPlane[0],
                                          thrust * axis[1] + inPlane[1],
                                          thrust * axis[2] + inPlane[2]};
        // Applied at the link's origin; the backend takes torques about the centre of mass.
        const std::array<double, 3> com = rotate(hub.orientation, r.com);
        const std::array<double, 3> moment = cross({-com[0], -com[1], -com[2]}, force);
        // The air's torque acts on the propeller alone, from outside, about the joint axis. As a
        // joint torque (load_torque) the frame would take it back as a reaction, so that a
        // steady difference between the rotors' speeds would turn nothing: a multirotor yaws by
        // the motors' reactions to the air's torque, which only reach the frame when the air's
        // torque does not.
        const std::array<double, 3> air{load.torque * jointAxis[0], load.torque * jointAxis[1],
                                        load.torque * jointAxis[2]};
        bus.writeN(r.wrenchOut, std::array<double, 6>{force[0], force[1], force[2],
                                                      moment[0] + hubMoment[0] + air[0],
                                                      moment[1] + hubMoment[1] + air[1],
                                                      moment[2] + hubMoment[2] + air[2]});
        const double area = std::numbers::pi * r.params.radius * r.params.radius;
        bus.write(r.thrustOut, thrust);
        bus.writeN(r.axisOut, axis);
        bus.write(r.torqueOut, load.torque);
        bus.write(r.loadOut, 0.0);  // in the wrench above instead
        bus.write(r.inducedOut,
                  r.propeller ? induced : std::sqrt(std::abs(thrust) / (2.0 * rho * area)));
        if (r.immersionOut.valid()) {
            bus.write(r.immersionOut, wet);
        }
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
