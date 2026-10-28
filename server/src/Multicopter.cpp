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
    if (!homed_) {
        home_ = position;
        homed_ = true;
        setpoint_ = {home_[0], home_[1], home_[2] + settings_.altitudeM};
        heading_ = 0.0;
    }
    if (context.t.seconds() < settings_.takeoffAfterS) {
        for (const Rotor& r : rotors_) {
            bus.write(r.command, 0.0);
        }
        return {};
    }
    const Mat3 rot = rotation({p[3], p[4], p[5], p[6]});  // world from body
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
    // the heading; without one it holds where it is.
    Vec3 wanted{};  // m/s, world: the velocity the set point moves at
    if (const std::optional<FlightCommand> command = commands_ ? commands_() : std::nullopt) {
        const double dt = context.dt.seconds();
        double forward = command->forwardMps;
        double left = command->leftMps;
        const double planar = std::hypot(forward, left);
        if (planar > settings_.maxHorizontalMps) {
            forward *= settings_.maxHorizontalMps / planar;
            left *= settings_.maxHorizontalMps / planar;
        }
        const double c = std::cos(heading_);
        const double s = std::sin(heading_);
        wanted = {c * forward - s * left, s * forward + c * left,
                  std::clamp(command->upMps, -settings_.maxVerticalMps, settings_.maxVerticalMps)};
        for (std::size_t i = 0; i < 3; ++i) {
            setpoint_[i] += wanted[i] * dt;
        }
        if (setpoint_[2] < home_[2] + settings_.minHeightM) {
            setpoint_[2] = home_[2] + settings_.minHeightM;
            wanted[2] = 0.0;
        }
        heading_ += std::clamp(command->yawRateRadps, -settings_.maxYawRateRadps,
                               settings_.maxYawRateRadps) *
                    dt;
    }
    Vec3 force{};
    for (std::size_t i = 0; i < 3; ++i) {
        force[i] = mass_ * (kp * (setpoint_[i] - position[i]) + kd * (wanted[i] - velocity[i]));
    }
    force[2] += mass_ * kGravity;
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

    // Mixer and each rotor's speed loop.
    for (std::size_t i = 0; i < rotors_.size(); ++i) {
        const Rotor& r = rotors_[i];
        const std::array<double, 4>& m = mixer_[i];
        const double f =
            std::max(0.0, m[0] * thrust + m[1] * torque[0] + m[2] * torque[1] + m[3] * torque[2]);
        const double speed = r.spin * std::sqrt(f / r.thrustPerOmega2);
        const double now = bus.read(r.velocity);
        // The propeller's drag at the wanted speed, plus a proportional correction.
        const double feedForward = r.spin * r.torquePerOmega2 * speed * speed;
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
    return ok ? Status{} : Status::error(StatusCode::InvalidConfig, "controller state truncated");
}

}  // namespace lagrange::server
