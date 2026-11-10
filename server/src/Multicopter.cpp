// The built-in multicopter flight controller (Multicopter.hpp).
#include "lagrange/server/Multicopter.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace lagrange::server {

using namespace literals;

namespace {

constexpr double kGravity = 9.80665;  // m/s², standard gravity (the world's gravity is along −z)
constexpr const char* kAirDensityKey = "air_density";  // as lagrange::rotor_aero
// Landing by hand (Settings::waitForCommand): the set point may go this far below the start, so
// the drone presses onto the ground, and it counts as landed this close to its start height and
// this slow vertically.
constexpr double kLandBelowM = 0.05;  // m
constexpr double kLandedM = 0.03;     // m
constexpr double kLandedMps = 0.05;   // m/s
constexpr double kGroundRayM = 1e4;   // m: how far down it looks for the ground
// The leash: a velocity command moves the set point no further ahead of the drone, along each
// commanded axis, than the position loop turns into this acceleration (kLeashAccel / kp). A
// drone that cannot follow (pressed under a ceiling, against a wall) is then asked for a bounded
// push, and comes back as soon as the stick does, instead of chasing a set point that ran off
// tens of metres while it was held (PX4's position controller bounds its set point likewise).
constexpr double kLeashAccel = 0.5 * kGravity;  // m/s²
// The set point's velocity closes on the stick's with this time constant, within the
// acceleration and jerk limits (Settings), so it settles without a long tail.
constexpr double kRampTimeS = 0.3;    // s
constexpr double kSettledMps = 0.02;  // m/s: this close, the ramp has reached the stick

using Vec3 = std::array<double, 3>;
using Mat3 = std::array<Vec3, 3>;  // rows

Mat3 rotation(const std::array<double, 4>& q) {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    return {{{1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
             {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
             {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)}}};
}

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

double dot(const Vec3& a, const Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

Vec3 normalized(const Vec3& v) {
    const double n = std::sqrt(dot(v, v));
    return n > 0.0 ? Vec3{v[0] / n, v[1] / n, v[2] / n} : Vec3{0.0, 0.0, 1.0};
}

/// Inverse of a 4×4 matrix by Gauss–Jordan elimination; false if singular.
bool invert(std::array<std::array<double, 4>, 4> a, std::array<std::array<double, 4>, 4>& out) {
    for (std::size_t r = 0; r < 4; ++r) {
        out[r] = {0.0, 0.0, 0.0, 0.0};
        out[r][r] = 1.0;
    }
    for (std::size_t c = 0; c < 4; ++c) {
        std::size_t pivot = c;
        for (std::size_t r = c + 1; r < 4; ++r) {
            if (std::abs(a[r][c]) > std::abs(a[pivot][c])) {
                pivot = r;
            }
        }
        if (std::abs(a[pivot][c]) < 1e-12) {
            return false;
        }
        std::swap(a[c], a[pivot]);
        std::swap(out[c], out[pivot]);
        const double scale = a[c][c];
        for (std::size_t k = 0; k < 4; ++k) {
            a[c][k] /= scale;
            out[c][k] /= scale;
        }
        for (std::size_t r = 0; r < 4; ++r) {
            if (r != c) {
                const double f = a[r][c];
                for (std::size_t k = 0; k < 4; ++k) {
                    a[r][k] -= f * a[c][k];
                    out[r][k] -= f * out[c][k];
                }
            }
        }
    }
    return true;
}

}  // namespace

bool MulticopterController::canFly(const WorldView& world) {
    const auto rotors = world.instancesOf("RotorAPI_1");
    if (rotors.size() < 3) {
        return false;
    }
    const World::Joints& joints = world.joints();
    std::optional<std::uint32_t> base;
    for (const SchemaInstance* s : rotors) {
        const auto j = std::ranges::find(joints.prim, s->prim) - joints.prim.begin();
        if (static_cast<std::size_t>(j) >= joints.prim.size()) {
            return false;
        }
        const std::uint32_t parent = joints.parent[j].index;
        if (base && *base != parent) {
            return false;
        }
        base = parent;
    }
    return true;
}

ModuleInfo MulticopterController::info() const {
    // Commands from outside (ROS) are not deterministic.
    return {"control", "multicopter", 0, 1, false, !commands_, CapabilityTier::Tier0};
}

Duration MulticopterController::preferredStep() const {
    return 1_ms;
}

Status MulticopterController::declare(const WorldView& world, const ModuleConfig& config,
                                      SignalDeclarer& d) {
    rotors_.clear();
    const std::optional<double> rho = config.get<double>(kAirDensityKey);
    if (!rho || *rho <= 0.0) {
        return Status::error(StatusCode::InvalidConfig,
                             "the multicopter controller needs the air density (as rotor_aero)");
    }
    const World::Joints& joints = world.joints();
    const World::Bodies& bodies = world.bodies();
    std::uint32_t base = 0;
    for (const SchemaInstance* s : world.instancesOf("RotorAPI_1")) {
        const auto j =
            static_cast<std::size_t>(std::ranges::find(joints.prim, s->prim) - joints.prim.begin());
        base = joints.parent[j].index;
        const std::size_t child = joints.child[j].index;
        const double radius = s->real("aero:rotor:radius").value_or(0.0);
        const double diameter = 2.0 * radius;
        const double ct = s->real("aero:rotor:thrustCoeff").value_or(0.0);
        const double cp = s->real("aero:rotor:powerCoeff").value_or(0.0);
        // T = CT·ρ·n²·D⁴ and Q = CP·ρ·n²·D⁵/(2π) with n = ω/(2π).
        const double perOmega2 = *rho / (4.0 * std::numbers::pi * std::numbers::pi);
        Rotor r;
        r.position = bodies.poseInParent[child].position;
        // The motor's sense about the thrust direction, and about the joint axis (which a pusher
        // has pointing the other way).
        r.direction = s->token("aero:rotor:direction").value_or("ccw") == "ccw" ? 1.0 : -1.0;
        r.spin = r.direction *
                 (s->token("aero:rotor:mount").value_or("puller") == "pusher" ? -1.0 : 1.0);
        r.thrustPerOmega2 = ct * perOmega2 * std::pow(diameter, 4);
        r.torquePerOmega2 = cp * perOmega2 * std::pow(diameter, 5) / (2.0 * std::numbers::pi);
        const std::string joint = "joint/" + std::string(world.signalInstanceName(s->prim)) + "/";
        SignalSpec velocity;
        velocity.name = joint + "velocity";
        velocity.unit = units::rad_per_s;
        velocity.feedback = true;
        r.velocity = d.input(velocity);
        SignalSpec command;
        command.name = joint + "torque_command";
        command.unit = units::N_m;
        r.command = d.output(command);
        rotors_.push_back(r);
    }
    // The whole robot's mass; the base's principal inertia stands for the airframe's.
    mass_ = 0.0;
    for (std::size_t b = 0; b < bodies.mass.size(); ++b) {
        mass_ += bodies.mass[b];
    }
    inertia_ = {bodies.inertiaLocal[base][0], bodies.inertiaLocal[base][1],
                bodies.inertiaLocal[base][2]};
    // Mixer: (T, τx, τy, τz) = A · thrusts for the first four rotors; with more, the pseudo-
    // inverse would be needed (hexacopters come later).
    if (rotors_.size() != 4) {
        return Status::error(StatusCode::Unsupported,
                             "the built-in controller mixes four rotors; this robot has " +
                                 std::to_string(rotors_.size()));
    }
    std::array<std::array<double, 4>, 4> a{};
    for (std::size_t i = 0; i < 4; ++i) {
        const Rotor& r = rotors_[i];
        const double yawPerThrust = -r.direction * r.torquePerOmega2 / r.thrustPerOmega2;
        a[0][i] = 1.0;
        a[1][i] = r.position[1];   // roll: y × thrust
        a[2][i] = -r.position[0];  // pitch: −x × thrust
        a[3][i] = yawPerThrust;    // a ccw rotor's drag turns the body cw
    }
    std::array<std::array<double, 4>, 4> inverse{};
    if (!invert(a, inverse)) {
        return Status::error(StatusCode::InvalidConfig, "the rotor layout cannot control yaw");
    }
    mixer_.assign(4, {});
    for (std::size_t i = 0; i < 4; ++i) {
        mixer_[i] = inverse[i];
    }
    const std::string link = "body/" + std::string(world.signalInstanceName(bodies.prim[base]));
    SignalSpec pose;
    pose.name = link + "/pose";
    pose.type = SignalType::Pose;
    pose.unit = units::m;
    pose.frame = "world_local";
    pose.feedback = true;
    pose_ = d.input(pose);
    SignalSpec twist;
    twist.name = link + "/twist";
    twist.type = SignalType::Wrench;
    twist.unit = units::m_per_s;
    twist.frame = "world_local";
    twist.feedback = true;
    twist_ = d.input(twist);
    designDensity_ = *rho;
    densityIn_ = SignalId{};
    if (config.get<bool>("weather_air").value_or(false)) {  // rotor_aero::kWeatherAirKey
        SignalSpec density;
        density.name = "env/air_density";
        density.unit = units::kg_per_m3;
        density.feedback = true;
        densityIn_ = d.input(density);
    }
    return {};
}

Status MulticopterController::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    homed_ = false;
    for (const Rotor& r : rotors_) {
        bus.write(r.command, 0.0);
    }
    return {};
}

Status MulticopterController::step(const StepContext& context, SignalBus& bus) {
    const std::span<const double> p = bus.readN(pose_);
    const std::span<const double> v = bus.readN(twist_);
    const Vec3 position{p[0], p[1], p[2]};
    // The surface straight under the drone (fixed scenery and the ground), when it was given.
    const auto surfaceBelow = [&]() -> std::optional<double> {
        if (!ground_) {
            return std::nullopt;
        }
        const services::RayHit hit = ground_->ray(position, {0.0, 0.0, -1.0}, kGroundRayM);
        if (!hit.hit()) {
            return std::nullopt;
        }
        return position[2] - hit.distance;
    };
    if (!homed_) {
        home_ = position;
        homed_ = true;
        if (const std::optional<double> surface = surfaceBelow()) {
            standHeight_ = position[2] - *surface;
            standKnown_ = true;
        }
        setpoint_ = {home_[0], home_[1], home_[2] + settings_.altitudeM};
        heading_ = 0.0;
        airborne_ = false;
    }
    const Mat3 rot = rotation({p[3], p[4], p[5], p[6]});  // world from body
    const auto grounded = [&]() {
        for (const Rotor& r : rotors_) {
            bus.write(r.command, 0.0);
        }
        return Status{};
    };
    if (settings_.waitForCommand && !airborne_) {
        // On the ground until a command asks to climb; then the set point starts where it is.
        const std::optional<FlightCommand> command = commands_ ? commands_() : std::nullopt;
        if (!command || command->upMps <= 0.0) {
            return grounded();
        }
        airborne_ = true;
        setpoint_ = position;
        // Where it takes off from is its start: a robot spawned or placed after the first step
        // (--spawn) was seen at its file's pose then.
        home_ = position;
        if (const std::optional<double> surface = surfaceBelow()) {
            standHeight_ = position[2] - *surface;
            standKnown_ = true;
        }
        velocity_ = {};
        accel_ = {};
        held_ = {};
        heading_ = std::atan2(rot[1][0], rot[0][0]);
    } else if (!settings_.waitForCommand && context.t.seconds() < settings_.takeoffAfterS) {
        return grounded();
    }
    const Vec3 velocity{v[0], v[1], v[2]};
    const Vec3 omegaWorld{v[3], v[4], v[5]};
    const Vec3 omega{dot({rot[0][0], rot[1][0], rot[2][0]}, omegaWorld),
                     dot({rot[0][1], rot[1][1], rot[2][1]}, omegaWorld),
                     dot({rot[0][2], rot[1][2], rot[2][2]}, omegaWorld)};

    // Position loop: the acceleration wanted, plus gravity, times the mass.
    const double wp = 2.0 * std::numbers::pi * settings_.positionHz;
    const double kp = wp * wp;
    const double kd = 2.0 * settings_.damping * wp;
    // A velocity command moves the set point (forward and left in the heading frame) and turns
    // the heading; without one it holds where it is. The set point's velocity follows the stick
    // within the acceleration limits, so letting go brakes it to a stop.
    Vec3 stick{};  // m/s, world: the velocity the command asks for
    bool angleMode = false;
    std::array<double, 2> tilt{};  // angle mode: the stick's share towards world x and y
    double yawRate = 0.0;
    const double dt = context.dt.seconds();
    const std::optional<FlightCommand> command = commands_ ? commands_() : std::nullopt;
    // Where it would stand here: on the surface under it, or at its start without the scenery.
    double standing = home_[2];
    if (standKnown_) {
        if (const std::optional<double> surface = surfaceBelow()) {
            standing = *surface + standHeight_;
        }
    }
    if (command) {
        // Taking off or landing by hand, near the ground, only up and down count: moving the set
        // point sideways while the legs still hold the drone would tip it over on the ground.
        const bool nearGround =
            settings_.waitForCommand && position[2] < standing + settings_.minHeightM;
        double forward = nearGround ? 0.0 : command->forwardMps;
        double left = nearGround ? 0.0 : command->leftMps;
        yawRate = nearGround ? 0.0 : command->yawRateRadps;
        const double planar = std::hypot(forward, left);
        if (planar > settings_.maxHorizontalMps) {
            forward *= settings_.maxHorizontalMps / planar;
            left *= settings_.maxHorizontalMps / planar;
        }
        const double c = std::cos(heading_);
        const double s = std::sin(heading_);
        stick = {c * forward - s * left, s * forward + c * left,
                 std::clamp(command->upMps, -settings_.maxVerticalMps, settings_.maxVerticalMps)};
        angleMode = command->mode == FlightCommand::Mode::Angle;
        if (angleMode) {
            tilt = {stick[0] / settings_.maxHorizontalMps, stick[1] / settings_.maxHorizontalMps};
            stick[0] = stick[1] = 0.0;
        }
    }
    {
        // The acceleration that would close the gap to the stick in kRampTimeS, within the
        // acceleration limits; the acceleration then moves towards it within the jerk limit,
        // and is cut where it would carry the velocity past the stick.
        const double jerk = settings_.maxJerkMps3 * dt;
        Vec3 aim{(stick[0] - velocity_[0]) / kRampTimeS, (stick[1] - velocity_[1]) / kRampTimeS,
                 (stick[2] - velocity_[2]) / kRampTimeS};
        const double planar = std::hypot(aim[0], aim[1]);
        if (planar > settings_.maxHorizontalAccelMps2) {
            aim[0] *= settings_.maxHorizontalAccelMps2 / planar;
            aim[1] *= settings_.maxHorizontalAccelMps2 / planar;
        }
        aim[2] =
            std::clamp(aim[2], -settings_.maxVerticalAccelMps2, settings_.maxVerticalAccelMps2);
        for (std::size_t i = 0; i < 3; ++i) {
            accel_[i] += std::clamp(aim[i] - accel_[i], -jerk, jerk);
            const double next = velocity_[i] + accel_[i] * dt;
            if ((stick[i] - velocity_[i]) * (stick[i] - next) <= 0.0 ||
                std::abs(stick[i] - next) < kSettledMps) {
                velocity_[i] = stick[i];  // reached: no overshoot
                accel_[i] = 0.0;
            } else {
                velocity_[i] = next;
            }
        }
    }
    if (angleMode) {
        velocity_[0] = velocity_[1] = 0.0;
        accel_[0] = accel_[1] = 0.0;
    }
    const double leash = kLeashAccel / kp;
    for (std::size_t i = 0; i < 3; ++i) {
        if (held_[i] && stick[i] == 0.0) {
            // Let go while something held it back (a ceiling, a wall): there is nothing to brake,
            // so the set point stays where the drone is while the push ramps down with the
            // velocity (easing off, not dropping it all at once), and it then holds there.
            setpoint_[i] = position[i];
            held_[i] = velocity_[i] != 0.0;
            continue;
        }
        setpoint_[i] += velocity_[i] * dt;
        if (velocity_[i] > 0.0 && setpoint_[i] > position[i] + leash) {
            setpoint_[i] = position[i] + leash;
            held_[i] = true;
        } else if (velocity_[i] < 0.0 && setpoint_[i] < position[i] - leash) {
            setpoint_[i] = position[i] - leash;
            held_[i] = true;
        } else if (velocity_[i] == 0.0) {
            held_[i] = false;
        }
    }
    if (command) {
        // Commanded by hand it may come down to the ground (a little below its start, so that
        // it sits); else it stays minHeightM up.
        const double floor =
            settings_.waitForCommand ? standing - kLandBelowM : home_[2] + settings_.minHeightM;
        if (setpoint_[2] < floor) {
            setpoint_[2] = floor;
            velocity_[2] = 0.0;
            accel_[2] = 0.0;
        }
        if (settings_.waitForCommand && command->upMps < 0.0 && position[2] < standing + kLandedM &&
            std::abs(velocity[2]) < kLandedMps) {
            airborne_ = false;  // landed: rotors off until the next climb
            velocity_ = {};
            accel_ = {};
            return grounded();
        }
        heading_ += std::clamp(yawRate, -settings_.maxYawRateRadps, settings_.maxYawRateRadps) * dt;
    }
    // The set point's velocity and its change, fed forward.
    const Vec3 wanted = velocity_;
    const Vec3 accel = accel_;
    if (angleMode) {
        // Nothing holds it in place: back in position mode it holds where it then is.
        setpoint_[0] = position[0];
        setpoint_[1] = position[1];
    }
    Vec3 force{};
    for (std::size_t i = angleMode ? 2 : 0; i < 3; ++i) {
        force[i] =
            mass_ * (kp * (setpoint_[i] - position[i]) + kd * (wanted[i] - velocity[i]) + accel[i]);
    }
    force[2] += mass_ * kGravity;
    if (angleMode) {
        // Tilted by the stick alone: full stick is maxAngleTiltRad; centred, it flies level.
        const double lean = std::tan(settings_.maxAngleTiltRad);
        force[0] = force[2] * lean * tilt[0];
        force[1] = force[2] * lean * tilt[1];
    }
    // Never ask for more than 30° of tilt.
    const double horizontal = std::hypot(force[0], force[1]);
    const double maxHorizontal = force[2] * std::tan(std::numbers::pi / 6.0);
    if (force[2] > 0.0 && horizontal > maxHorizontal) {
        force[0] *= maxHorizontal / horizontal;
        force[1] *= maxHorizontal / horizontal;
    }
    const Vec3 bodyZ{rot[0][2], rot[1][2], rot[2][2]};
    const double thrust = std::max(0.0, dot(force, bodyZ));

    // Attitude: the body z axis along the force, the nose towards the heading.
    const Vec3 z = normalized(force);
    const Vec3 y = normalized(cross(z, {std::cos(heading_), std::sin(heading_), 0.0}));
    const Vec3 x = cross(y, z);
    const Mat3 want{{{x[0], y[0], z[0]}, {x[1], y[1], z[1]}, {x[2], y[2], z[2]}}};
    // e_R = ½ vee(R_dᵀR − RᵀR_d) (Lee et al. eq. 11).
    Mat3 e{};
    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t c = 0; c < 3; ++c) {
            double dr = 0.0;
            double rd = 0.0;
            for (std::size_t k = 0; k < 3; ++k) {
                dr += want[k][r] * rot[k][c];
                rd += rot[k][r] * want[k][c];
            }
            e[r][c] = dr - rd;
        }
    }
    const Vec3 errorR{0.5 * e[2][1], 0.5 * e[0][2], 0.5 * e[1][0]};
    const double wa = 2.0 * std::numbers::pi * settings_.attitudeHz;
    Vec3 torque{};
    for (std::size_t i = 0; i < 3; ++i) {
        torque[i] = inertia_[i] * (-wa * wa * errorR[i] - 2.0 * settings_.damping * wa * omega[i]);
    }

    // Mixer and each rotor's speed loop, for the air the rotors are in.
    double airShare = 1.0;
    if (densityIn_.valid()) {
        if (const double rho = bus.read(densityIn_); rho > 0.0) {
            airShare = rho / designDensity_;
        }
    }
    for (std::size_t i = 0; i < rotors_.size(); ++i) {
        const Rotor& r = rotors_[i];
        const std::array<double, 4>& m = mixer_[i];
        const double f =
            std::max(0.0, m[0] * thrust + m[1] * torque[0] + m[2] * torque[1] + m[3] * torque[2]);
        const double speed = r.spin * std::sqrt(f / (r.thrustPerOmega2 * airShare));
        const double now = bus.read(r.velocity);
        // The propeller's drag at the wanted speed, plus a proportional correction.
        const double feedForward = r.spin * r.torquePerOmega2 * airShare * speed * speed;
        bus.write(r.command, feedForward + settings_.speedGain * (speed - now));
    }
    return {};
}

Status MulticopterController::saveState(StateWriter& writer) const {
    writer.put(homed_);
    for (const double v : home_) {
        writer.put(v);
    }
    for (const double v : setpoint_) {
        writer.put(v);
    }
    writer.put(heading_);
    writer.put(airborne_);
    writer.put(standHeight_);
    writer.put(standKnown_);
    for (const double v : velocity_) {
        writer.put(v);
    }
    for (const double a : accel_) {
        writer.put(a);
    }
    for (const bool h : held_) {
        writer.put(h);
    }
    return {};
}

void MulticopterController::shiftOrigin(const std::array<double, 3>& shift) {
    for (std::size_t i = 0; i < 3; ++i) {
        home_.at(i) -= shift.at(i);
        setpoint_.at(i) -= shift.at(i);
    }
}

Status MulticopterController::restoreState(StateReader& reader) {
    bool ok = reader.get(homed_);
    for (double& v : home_) {
        ok = ok && reader.get(v);
    }
    for (double& v : setpoint_) {
        ok = ok && reader.get(v);
    }
    ok = ok && reader.get(heading_);
    ok = ok && reader.get(airborne_);
    ok = ok && reader.get(standHeight_);
    ok = ok && reader.get(standKnown_);
    for (double& v : velocity_) {
        ok = ok && reader.get(v);
    }
    for (double& a : accel_) {
        ok = ok && reader.get(a);
    }
    for (bool& h : held_) {
        ok = ok && reader.get(h);
    }
    return ok ? Status{} : Status::error(StatusCode::InvalidConfig, "controller state truncated");
}

}  // namespace lagrange::server
