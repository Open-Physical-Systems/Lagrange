// The vehicle dynamics domain (VehicleDynamics.hpp, domains/vehicle_dynamics.md).
#include "lagrange/module/StateIO.hpp"
#include "lagrange/vehicle_dynamics/VehicleDynamics.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <map>

namespace lagrange::vehicle_dynamics {

using namespace literals;

namespace {

constexpr Vec3 kUp{0.0, 0.0, 1.0};
constexpr double kBumpStopFactor = 10.0;   // bump stop rate over the spring rate
constexpr double kMaxSlipRatio = 1.5;      // the relaxation states' limits: a locked or
constexpr double kMaxSlipAngle = 1.2;      // spinning wheel, a sideways slide (rad)
constexpr double kAbsSlip = 0.12;          // ABS: slip ratio at which it starts to release
constexpr double kAbsBand = 0.08;          // and over which it releases fully
constexpr double kMinShiftInterval = 0.8;  // s between automatic shifts
constexpr double kSteerBandwidth = 2.0 * 3.141592653589793 * 15.0;  // rad/s, steering PD
constexpr double kIdealTimeConstant = 0.05;  // s, the ideal substitute's tracking
// Below this speed the tyre's relaxation behaves as an undamped spring (the wheel's spin
// against the tread, the chassis against the contact), so damping is blended in, fading to
// none at it; damping ratio of that mode.
constexpr double kLowSpeed = 3.0;  // m/s
constexpr double kLowSpeedDamping = 0.7;
constexpr double kBrakeSpeed = 0.5;  // rad/s, over which the brake torque reaches its full value

SignalSpec spec(std::string name, SignalType type, Unit unit, std::string frame = {}) {
    SignalSpec s;
    s.name = std::move(name);
    s.type = type;
    s.unit = unit;
    s.frame = std::move(frame);
    return s;
}

SignalSpec feedback(std::string name, SignalType type, Unit unit, std::string frame = {}) {
    SignalSpec s = spec(std::move(name), type, unit, std::move(frame));
    s.feedback = true;
    return s;
}

SignalSpec optional(std::string name, Unit unit, double fallback = 0.0,
                    SignalType type = SignalType::Scalar) {
    SignalSpec s = spec(std::move(name), type, unit);
    s.optional = true;
    s.defaultValue = fallback;
    return s;
}

Vec3 add(const Vec3& a, const Vec3& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}
Vec3 sub(const Vec3& a, const Vec3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
Vec3 mul(const Vec3& a, double k) {
    return {a[0] * k, a[1] * k, a[2] * k};
}
double dot(const Vec3& a, const Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
Vec3 normalized(const Vec3& a) {
    const double n = std::sqrt(dot(a, a));
    return n > 0.0 ? mul(a, 1.0 / n) : Vec3{1.0, 0.0, 0.0};
}
double sign(double x) {
    return x >= 0.0 ? 1.0 : -1.0;
}

/// `child`'s rest pose in `parent`'s frame.
Pose relativeTo(const Pose& parent, const Pose& child) {
    const std::array<double, 4> inverse = conjugate(parent.orientation);
    return {rotate(inverse, sub(child.position, parent.position)),
            multiply(inverse, child.orientation)};
}

/// Relaxation state x after `dt` of dx/dt = slip − rate·x (exact for constant inputs).
double relax(double x, double slipVelocity, double rate, double dt) {
    if (rate * dt < 1e-9) {
        return x + slipVelocity * dt;
    }
    const double decay = std::exp(-rate * dt);
    return x * decay + slipVelocity * (1.0 - decay) / rate;
}

}  // namespace

double ackermannAngle(double delta, double wheelbase, double y) {
    if (delta == 0.0 || wheelbase <= 0.0) {
        return delta;
    }
    const double t = std::tan(delta);
    return std::atan(wheelbase * t / (wheelbase - y * t));
}

double engineTorque(const Vehicle& v, double speed) {
    const std::vector<double>& w = v.engineSpeeds;
    if (w.empty() || speed < w.front() || speed > w.back()) {
        return 0.0;
    }
    const auto it = std::upper_bound(w.begin(), w.end(), speed);
    if (it == w.end()) {
        return v.engineTorques.back();
    }
    const std::size_t i = static_cast<std::size_t>(it - w.begin());
    const double f = (speed - w[i - 1]) / (w[i] - w[i - 1]);
    return v.engineTorques[i - 1] + f * (v.engineTorques[i] - v.engineTorques[i - 1]);
}

Expected<std::vector<Vehicle>, std::string> vehiclesOf(const WorldView& world) {
    const World& raw = world.raw();
    const std::vector<Pose> rest = restPoses(raw);
    std::vector<Vehicle> out;
    const auto bodyOfPrim = [&](PrimId prim) -> std::optional<std::uint32_t> {
        for (std::uint32_t b = 0; b < raw.bodies.prim.size(); ++b) {
            if (raw.bodies.prim[b] == prim) {
                return b;
            }
        }
        return std::nullopt;
    };
    const auto jointOfPrim = [&](PrimId prim) -> std::optional<std::uint32_t> {
        for (std::uint32_t j = 0; j < raw.joints.prim.size(); ++j) {
            if (raw.joints.prim[j] == prim) {
                return j;
            }
        }
        return std::nullopt;
    };
    const std::vector<const SchemaInstance*> wheelSchemas = world.instancesOf("WheelAPI_1");
    for (const SchemaInstance* inst : world.instancesOf("VehicleAPI_1")) {
        const std::string where = std::string(world.primPath(inst->prim)) + ": ";
        const auto chassis = bodyOfPrim(inst->prim);
        if (!chassis) {
            return Unexpected{where + "VehicleAPI_1 must be on a rigid body (the chassis)"};
        }
        Vehicle v;
        v.chassis = *chassis;
        v.name = raw.robots.empty() ? raw.bodies.names[*chassis]
                                    : raw.robots.at(raw.bodies.robot[*chassis]);
        v.engine = inst->token("vehicle:powertrain").value_or("electric") == "ice_map";
        v.maxDriveTorque = inst->real("vehicle:maxDriveTorque").value_or(0.0);
        v.maxSteerAngle = inst->real("vehicle:maxSteerAngle").value_or(0.0);
        v.steerRate = inst->real("vehicle:steerRate").value_or(1.0);
        v.steerTimeConstant = inst->real("vehicle:steerTimeConstant").value_or(0.05);
        const auto brake = inst->real("vehicle:maxBrakeTorque");
        if (!brake || *brake < 0.0) {
            return Unexpected{where + "vehicle:maxBrakeTorque is required"};
        }
        v.maxBrakeTorque = *brake;
        v.brakeBias = inst->real("vehicle:brakeBias").value_or(0.6);
        v.abs = inst->integer("vehicle:abs").value_or(0) != 0;
        const auto array = [&](const char* name) {
            const std::span<const double> a = inst->realArray(name);
            return std::vector<double>(a.begin(), a.end());
        };
        v.engineSpeeds = array("vehicle:engineSpeeds");
        v.engineTorques = array("vehicle:engineTorques");
        v.gearRatios = array("vehicle:gearRatios");
        v.idleSpeed = inst->real("vehicle:engineIdleSpeed").value_or(80.0);
        v.finalDrive = inst->real("vehicle:finalDriveRatio").value_or(1.0);
        v.shiftUp = inst->real("vehicle:shiftUpSpeed").value_or(600.0);
        v.shiftDown = inst->real("vehicle:shiftDownSpeed").value_or(250.0);
        v.efficiency = inst->real("vehicle:driveEfficiency").value_or(0.92);
        v.antiRollFront = inst->real("vehicle:antiRollStiffnessFront").value_or(0.0);
        v.antiRollRear = inst->real("vehicle:antiRollStiffnessRear").value_or(0.0);
        if (v.engine) {
            if (v.engineSpeeds.size() < 2 || v.engineSpeeds.size() != v.engineTorques.size() ||
                v.gearRatios.empty() ||
                !std::is_sorted(v.engineSpeeds.begin(), v.engineSpeeds.end())) {
                return Unexpected{where +
                                  "an ice_map powertrain needs vehicle:engineSpeeds "
                                  "(increasing) and vehicle:engineTorques of the same "
                                  "length, and vehicle:gearRatios"};
            }
        } else if (v.maxDriveTorque <= 0.0) {
            return Unexpected{where + "an electric powertrain needs vehicle:maxDriveTorque"};
        }
        const Pose& chassisRest = rest[v.chassis];
        const std::uint32_t robot = raw.bodies.robot[v.chassis];
        for (std::uint32_t b = 0; b < raw.bodies.names.size(); ++b) {
            if (raw.bodies.robot[b] == robot) {
                v.mass += raw.bodies.mass[b];
            }
        }
        for (const SchemaInstance* w : wheelSchemas) {
            const auto joint = jointOfPrim(w->prim);
            const std::string at = std::string(world.primPath(w->prim)) + ": ";
            if (!joint) {
                return Unexpected{at + "WheelAPI_1 must be on a joint (the wheel's spin joint)"};
            }
            // The wheel belongs to this vehicle if the chassis is above its body.
            const BodyId wheelBody = raw.joints.child[*joint];
            bool ours = false;
            for (BodyId b = raw.bodies.parent[wheelBody.index]; b != kWorldBody;
                 b = raw.bodies.parent[b.index]) {
                if (b.index == v.chassis) {
                    ours = true;
                    break;
                }
            }
            if (!ours) {
                continue;
            }
            Wheel wheel;
            wheel.name = std::string(world.signalInstanceName(w->prim));
            wheel.joint = *joint;
            const Pose& wheelRest = rest[wheelBody.index];
            wheel.anchor = relativeTo(chassisRest, wheelRest).position;
            const Vec3 axisChassis = rotate(conjugate(chassisRest.orientation),
                                            rotate(wheelRest.orientation, raw.joints.axis[*joint]));
            wheel.spin = sign(axisChassis[1]);
            {
                const Vec3& a = raw.joints.axis[*joint];  // in the wheel's frame
                const std::array<double, 6>& in = raw.bodies.inertiaLocal[wheelBody.index];
                wheel.spinInertia = a[0] * a[0] * in[0] + a[1] * a[1] * in[1] + a[2] * a[2] * in[2];
            }
            const auto required = [&](const char* name, double& out) -> bool {
                const auto value = w->real(name);
                if (!value || *value <= 0.0) {
                    return false;
                }
                out = *value;
                return true;
            };
            if (!required("wheel:radius", wheel.radius) ||
                !required("wheel:restLength", wheel.restLength) ||
                !required("wheel:travel", wheel.travel) ||
                !required("wheel:springStiffness", wheel.springStiffness) ||
                !required("wheel:damping", wheel.damping)) {
                return Unexpected{at +
                                  "wheel:radius, wheel:restLength, wheel:travel, "
                                  "wheel:springStiffness and wheel:damping are required and "
                                  "positive"};
            }
            const std::string_view axle = w->token("wheel:axle").value_or("");
            if (axle != "front" && axle != "rear") {
                return Unexpected{at + "wheel:axle is required: front or rear"};
            }
            wheel.front = axle == "front";
            wheel.width = w->real("wheel:width").value_or(0.2);
            wheel.steered = w->integer("wheel:steered").value_or(0) != 0;
            wheel.driven = w->integer("wheel:driven").value_or(0) != 0;
            wheel.friction = w->real("wheel:friction").value_or(1.0);
            wheel.relaxationLength = w->real("wheel:relaxationLength").value_or(0.3);
            wheel.longitudinalStiffness = w->real("wheel:longitudinalStiffness").value_or(0.0);
            wheel.corneringStiffness = w->real("wheel:corneringStiffness").value_or(0.0);
            if (const auto tir = w->assetPath("wheel:tyre")) {
                auto mf = MagicFormula::fromTir(std::string(*tir));
                if (!mf) {
                    return Unexpected{at + mf.error()};
                }
                wheel.tyre = *mf;
            }
            if (wheel.steered) {
                if (wheel.front != true) {
                    return Unexpected{at + "only front wheels steer"};
                }
                if (v.maxSteerAngle <= 0.0) {
                    return Unexpected{where +
                                      "vehicle:maxSteerAngle is required with a steered "
                                      "wheel"};
                }
                const std::span<const PrimId> steer = w->rel("wheel:steerJoint");
                if (!steer.empty()) {
                    const auto sj = jointOfPrim(steer[0]);
                    if (!sj) {
                        return Unexpected{at + "wheel:steerJoint must name a joint"};
                    }
                    wheel.steerJoint = *sj;
                    const BodyId knuckle = raw.joints.child[*sj];
                    const Vec3 steerAxis =
                        rotate(conjugate(chassisRest.orientation),
                               rotate(rest[knuckle.index].orientation, raw.joints.axis[*sj]));
                    wheel.steerSign = sign(steerAxis[2]);
                    // What the steering joint turns: the knuckle and the wheel, about z.
                    for (const BodyId b : {knuckle, wheelBody}) {
                        const Pose rel = relativeTo(rest[knuckle.index], rest[b.index]);
                        const double d2 =
                            rel.position[0] * rel.position[0] + rel.position[1] * rel.position[1];
                        wheel.steerInertia +=
                            raw.bodies.inertiaLocal[b.index][2] + raw.bodies.mass[b.index] * d2;
                    }
                }
            }
            v.wheels.push_back(std::move(wheel));
        }
        if (v.wheels.size() < 3) {
            return Unexpected{where + "a vehicle needs at least three WheelAPI_1 wheels"};
        }
        double front = 0.0, rear = 0.0;
        int nf = 0, nr = 0;
        for (const Wheel& w : v.wheels) {
            (w.front ? front : rear) += w.anchor[0];
            ++(w.front ? nf : nr);
        }
        if (nf == 0 || nr == 0) {
            return Unexpected{where + "a vehicle needs front and rear wheels"};
        }
        v.wheelbase = front / nf - rear / nr;
        // The model's wheel positions are the ride height: each spring carries its static
        // share of the weight there (by the centre of mass between the axles).
        Vec3 com{};
        for (std::uint32_t b = 0; b < raw.bodies.names.size(); ++b) {
            if (raw.bodies.robot[b] == robot) {
                const Vec3 c =
                    add(rest[b].position, rotate(rest[b].orientation, raw.bodies.comLocal[b]));
                com = add(com, mul(c, raw.bodies.mass[b] / v.mass));
            }
        }
        const double comX = relativeTo(chassisRest, Pose{com, {1.0, 0.0, 0.0, 0.0}}).position[0];
        const double g = std::sqrt(dot(raw.gravityLocal, raw.gravityLocal));
        const double frontShare =
            std::clamp((comX - rear / nr) / std::max(v.wheelbase, 1e-6), 0.0, 1.0);
        for (Wheel& w : v.wheels) {
            w.preload = v.mass * g * (w.front ? frontShare / nf : (1.0 - frontShare) / nr);
        }
        out.push_back(std::move(v));
    }
    return out;
}

ModuleInfo VehicleDynamics::info() const {
    return ideal_ ? ModuleInfo{"vehicle_dynamics", "ideal", 0, 1, true, true, CapabilityTier::Tier0}
                  : ModuleInfo{"vehicle_dynamics",   "raycast", 0, 1, false, true,
                               CapabilityTier::Tier0};
}

Duration VehicleDynamics::preferredStep() const {
    return 1_ms;
}

Status VehicleDynamics::declare(const WorldView& world, const ModuleConfig& config,
                                SignalDeclarer& d) {
    auto vehicles = vehiclesOf(world);
    if (!vehicles) {
        return Status::error(StatusCode::InvalidConfig, vehicles.error());
    }
    if (ground_ == nullptr && !vehicles->empty()) {
        return Status::error(StatusCode::InvalidConfig,
                             "vehicle dynamics needs a scene query for the ground");
    }
    pacejka_ = config.get<std::string>("tire_model").value_or("pacejka") != "linear";
    const World& raw = world.raw();
    vehicles_.clear();
    for (Vehicle& v : *vehicles) {
        for (const Wheel& w : v.wheels) {
            if (!ideal_ && pacejka_ && !w.tyre) {
                return Status::error(StatusCode::InvalidConfig,
                                     w.name +
                                         ": the pacejka tyre model needs wheel:tyre (a .tir "
                                         "file); or set tire_model: linear");
            }
            if (!ideal_ && !pacejka_ &&
                (w.longitudinalStiffness <= 0.0 || w.corneringStiffness <= 0.0)) {
                return Status::error(StatusCode::InvalidConfig,
                                     w.name +
                                         ": the linear tyre model needs "
                                         "wheel:longitudinalStiffness and "
                                         "wheel:corneringStiffness");
            }
        }
        VehicleState s;
        s.comLocal = raw.bodies.comLocal[v.chassis];
        const std::string base = "vehicle/" + v.name + "/";
        s.throttle = d.input(optional(base + "throttle", units::one));
        s.brake = d.input(optional(base + "brake", units::one));
        s.steering = d.input(optional(base + "steering_angle", units::rad));
        s.gearIn = d.input(optional(base + "gear_command", units::one, 0.0, SignalType::Int));
        const std::string chassis = "body/" + raw.bodies.names[v.chassis];
        s.pose = d.input(feedback(chassis + "/pose", SignalType::Pose, units::m, "world_local"));
        s.twist = d.input(
            feedback(chassis + "/twist", SignalType::Wrench, units::m_per_s, "world_local"));
        s.frictionScale = d.input(optional("env/friction_scale", units::one, 1.0));
        s.wrench = d.output(
            spec(chassis + "/wrench/vehicle", SignalType::Wrench, units::N, "world_local"));
        s.speed = d.output(spec(base + "speed", SignalType::Scalar, units::m_per_s));
        s.engineSpeed = d.output(spec(base + "engine_speed", SignalType::Scalar, units::rad_per_s));
        s.gear = d.output(spec(base + "gear", SignalType::Int, units::one));
        for (const Wheel& w : v.wheels) {
            WheelState ws;
            const std::string joint = "joint/" + w.name + "/";
            const std::string wheel = "wheel/" + w.name + "/";
            ws.velocity =
                d.input(feedback(joint + "velocity", SignalType::Scalar, units::rad_per_s));
            ws.loadTorque = d.output(spec(joint + "load_torque", SignalType::Scalar, units::N_m));
            if (w.driven) {
                ws.driveCommand =
                    d.output(spec(joint + "torque_command", SignalType::Scalar, units::N_m));
            }
            ws.slipRatio = d.output(spec(wheel + "slip_ratio", SignalType::Scalar, units::one));
            ws.slipAngle = d.output(spec(wheel + "slip_angle", SignalType::Scalar, units::rad));
            ws.normalForce = d.output(spec(wheel + "normal_force", SignalType::Scalar, units::N));
            ws.steerAngleOut =
                d.output(spec(wheel + "steer_angle", SignalType::Scalar, units::rad));
            if (w.steerJoint) {
                const std::string steer =
                    "joint/" +
                    std::string(world.signalInstanceName(raw.joints.prim[*w.steerJoint])) + "/";
                ws.steerPosition =
                    d.input(feedback(steer + "position", SignalType::Scalar, units::rad));
                ws.steerVelocity =
                    d.input(feedback(steer + "velocity", SignalType::Scalar, units::rad_per_s));
                ws.steerCommand =
                    d.output(spec(steer + "torque_command", SignalType::Scalar, units::N_m));
            }
            s.wheels.push_back(ws);
        }
        s.v = std::move(v);
        vehicles_.push_back(std::move(s));
    }
    return {};
}

Status VehicleDynamics::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    for (VehicleState& s : vehicles_) {
        s.delta = 0.0;
        s.currentGear = 1;
        s.shiftHold = 0.0;
        s.idealSpeed = 0.0;
        bus.writeN(s.wrench, std::array<double, 6>{});
        bus.write(s.speed, 0.0);
        bus.write(s.engineSpeed, s.v.engine ? s.v.idleSpeed : 0.0);
        bus.write(s.gear, s.v.engine ? 1.0 : 0.0);
        for (WheelState& w : s.wheels) {
            w.longDeflection = w.latDeflection = w.compression = 0.0;
            for (const SignalId id : {w.loadTorque, w.driveCommand, w.slipRatio, w.slipAngle,
                                      w.normalForce, w.steerAngleOut, w.steerCommand}) {
                if (id.valid()) {
                    bus.write(id, 0.0);
                }
            }
        }
    }
    return {};
}

Status VehicleDynamics::step(const StepContext& context, SignalBus& bus) {
    const double dt = context.dt.seconds();
    for (VehicleState& s : vehicles_) {
        stepVehicle(s, dt, bus);
    }
    return {};
}

void VehicleDynamics::stepVehicle(VehicleState& s, double dt, SignalBus& bus) {
    const Vehicle& v = s.v;
    const std::span<const double> p = bus.readN(s.pose);
    const std::span<const double> tw = bus.readN(s.twist);
    const Pose pose{{p[0], p[1], p[2]}, {p[3], p[4], p[5], p[6]}};
    const Vec3 lin{tw[0], tw[1], tw[2]};
    const Vec3 ang{tw[3], tw[4], tw[5]};
    const Vec3 com = add(pose.position, rotate(pose.orientation, s.comLocal));
    const Vec3 up = rotate(pose.orientation, kUp);
    const Vec3 forward = rotate(pose.orientation, {1.0, 0.0, 0.0});
    const Vec3 forwardLevel = normalized({forward[0], forward[1], 0.0});
    const double speed = dot(lin, forwardLevel);
    const double throttle = std::clamp(bus.read(s.throttle), -1.0, 1.0);
    const double brake = std::clamp(bus.read(s.brake), 0.0, 1.0);
    const double muEnv = bus.read(s.frictionScale);

    // Steering: rate limit, then the actuator's first-order lag.
    const double command = std::clamp(bus.read(s.steering), -v.maxSteerAngle, v.maxSteerAngle);
    const double limited =
        s.delta + std::clamp(command - s.delta, -v.steerRate * dt, v.steerRate * dt);
    s.delta += (limited - s.delta) * (1.0 - std::exp(-dt / std::max(v.steerTimeConstant, 1e-6)));

    // Suspension: compression along the chassis's up from each wheel's mount.
    const std::size_t n = v.wheels.size();
    std::vector<double> fz(n, 0.0), mu(n, 1.0), steer(n, 0.0);
    std::vector<Vec3> contact(n);
    std::vector<bool> grounded(n, false);
    for (std::size_t i = 0; i < n; ++i) {
        const Wheel& w = v.wheels[i];
        WheelState& ws = s.wheels[i];
        const Vec3 mount =
            add(pose.position, rotate(pose.orientation, add(w.anchor, mul(kUp, w.restLength))));
        // Down to where the spring's preload is spent (full droop).
        const services::RayHit hit = ground_->ray(
            mount, mul(up, -1.0), w.restLength + w.radius + w.preload / w.springStiffness);
        steer[i] = w.steered ? ackermannAngle(s.delta, v.wheelbase, w.anchor[1]) : 0.0;
        if (!hit.hit()) {
            ws.compression = 0.0;
            continue;
        }
        grounded[i] = true;
        contact[i] = sub(mount, mul(up, hit.distance));
        const double x = w.restLength - (hit.distance - w.radius);
        const Vec3 pointVelocity = add(lin, cross(ang, sub(mount, com)));
        const double rate = -dot(pointVelocity, up);
        double force = w.preload + w.springStiffness * x + w.damping * rate;
        if (x > w.travel) {
            force += kBumpStopFactor * w.springStiffness * (x - w.travel);
        }
        ws.compression = x;
        fz[i] = std::max(0.0, force);
        const double surface =
            hit.shape < groundFriction_.size() ? groundFriction_[hit.shape] : 1.0;
        mu[i] = surface * muEnv;
    }
    // Anti-roll bars: across each axle, the more compressed side takes more load.
    for (const bool frontAxle : {true, false}) {
        const double k = frontAxle ? v.antiRollFront : v.antiRollRear;
        if (k <= 0.0) {
            continue;
        }
        for (std::size_t i = 0; i < n; ++i) {
            for (std::size_t j = i + 1; j < n; ++j) {
                const Wheel& a = v.wheels[i];
                const Wheel& b = v.wheels[j];
                if (a.front != frontAxle || b.front != frontAxle ||
                    sign(a.anchor[1]) == sign(b.anchor[1]) || !grounded[i] || !grounded[j]) {
                    continue;
                }
                const double transfer = k * (s.wheels[i].compression - s.wheels[j].compression);
                fz[i] = std::max(0.0, fz[i] + transfer);
                fz[j] = std::max(0.0, fz[j] - transfer);
            }
        }
    }

    // Powertrain: the torque at the driven wheels.
    double driveTotal = 0.0;
    double engineSpeed = 0.0;
    int driven = 0;
    double wheelSpeed = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        if (v.wheels[i].driven) {
            ++driven;
            wheelSpeed += v.wheels[i].spin * bus.read(s.wheels[i].velocity);
        }
    }
    wheelSpeed = driven > 0 ? wheelSpeed / driven : 0.0;
    if (v.engine) {
        const auto requested = static_cast<int>(std::lround(bus.read(s.gearIn)));
        const int gears = static_cast<int>(v.gearRatios.size());
        s.shiftHold += dt;
        if (requested > 0) {
            s.currentGear = std::min(requested, gears);
        } else if (s.shiftHold > kMinShiftInterval) {
            const double spinning =
                std::abs(wheelSpeed) * v.gearRatios[s.currentGear - 1] * v.finalDrive;
            if (spinning > v.shiftUp && s.currentGear < gears) {
                ++s.currentGear;
                s.shiftHold = 0.0;
            } else if (spinning < v.shiftDown && s.currentGear > 1) {
                --s.currentGear;
                s.shiftHold = 0.0;
            }
        }
        // Backwards: first gear's ratio, reversed.
        const bool reverse = throttle < 0.0;
        const double ratio = v.gearRatios[reverse ? 0 : s.currentGear - 1] * v.finalDrive;
        // Below idle the clutch slips: the engine runs at idle.
        engineSpeed = std::max(v.idleSpeed, std::abs(wheelSpeed) * ratio);
        driveTotal = std::abs(throttle) * engineTorque(v, engineSpeed) * ratio * v.efficiency *
                     (reverse ? -1.0 : 1.0);
        bus.write(s.gear, reverse ? -1.0 : static_cast<double>(s.currentGear));
    } else {
        driveTotal = throttle * v.maxDriveTorque;
        engineSpeed = std::abs(wheelSpeed);
        bus.write(s.gear, 0.0);
    }

    // Brakes per wheel: the axle's share of the pedal, split across the axle.
    int frontCount = 0;
    for (const Wheel& w : v.wheels) {
        frontCount += w.front ? 1 : 0;
    }
    const int rearCount = static_cast<int>(n) - frontCount;

    std::array<double, 6> wrench{};
    const double idealGain = 1.0 / kIdealTimeConstant;
    for (std::size_t i = 0; i < n; ++i) {
        const Wheel& w = v.wheels[i];
        WheelState& ws = s.wheels[i];
        const double omega = bus.read(ws.velocity);
        // Heading of the wheel on the level ground.
        const double c = std::cos(steer[i]);
        const double sn = std::sin(steer[i]);
        const Vec3 left = rotate(pose.orientation, {0.0, 1.0, 0.0});
        const Vec3 heading =
            normalized({forward[0] * c + left[0] * sn, forward[1] * c + left[1] * sn, 0.0});
        const Vec3 lateral = cross(kUp, heading);
        double fx = 0.0;
        double fy = 0.0;
        double kappa = 0.0;
        double alpha = 0.0;
        if (grounded[i]) {
            const Vec3 vc = add(lin, cross(ang, sub(contact[i], com)));
            const double vx = dot(vc, heading);
            const double vy = dot(vc, lateral);
            const double surfaceSpeed = w.spin * omega * w.radius;
            const double sigma = std::max(w.relaxationLength, 1e-3);
            const double rate = std::abs(vx) / sigma;
            ws.longDeflection = relax(ws.longDeflection, surfaceSpeed - vx, rate, dt);
            ws.latDeflection = relax(ws.latDeflection, vy, rate, dt);
            ws.longDeflection =
                std::clamp(ws.longDeflection, -kMaxSlipRatio * sigma, kMaxSlipRatio * sigma);
            ws.latDeflection =
                std::clamp(ws.latDeflection, -kMaxSlipAngle * sigma, kMaxSlipAngle * sigma);
            kappa = ws.longDeflection / sigma;
            alpha = ws.latDeflection / sigma;
            if (!ideal_) {
                const auto tyre = [&](double k, double a) {
                    return pacejka_ ? pacejka(*w.tyre, k, a, fz[i], mu[i])
                                    : linearTyre(w.longitudinalStiffness, w.corneringStiffness,
                                                 w.friction * mu[i], k, a, fz[i]);
                };
                const TyreForce f = tyre(kappa, alpha);
                fx = f.fx;
                fy = f.fy;
                // Low-speed damping of the relaxation springs (critically damped-ish).
                const double blend = std::max(0.0, 1.0 - std::abs(vx) / kLowSpeed);
                if (blend > 0.0 && fz[i] > 0.0) {
                    constexpr double kProbe = 1e-4;
                    const double kx = std::abs(tyre(kProbe, 0.0).fx) / kProbe;  // N per unit slip
                    const double ky = std::abs(tyre(0.0, kProbe).fy) / kProbe;  // N per rad
                    const double inertia = std::max(w.spinInertia, 1e-3);
                    const double cx = 2.0 * kLowSpeedDamping *
                                      std::sqrt(kx / sigma * inertia / (w.radius * w.radius));
                    const double share = v.mass / static_cast<double>(n);
                    const double cy = 2.0 * kLowSpeedDamping * std::sqrt(ky / sigma * share);
                    fx += blend * cx * (surfaceSpeed - vx);
                    fy -= blend * cy * vy;
                    const double limit = mu[i] * fz[i] * 1.2;
                    fx = std::clamp(fx, -limit, limit);
                    fy = std::clamp(fy, -limit, limit);
                }
            }
        } else {
            ws.longDeflection = ws.latDeflection = 0.0;
        }
        const Vec3 force = add(add(mul(heading, fx), mul(lateral, fy)), mul(up, fz[i]));
        if (grounded[i]) {
            const Vec3 torque = cross(sub(contact[i], com), force);
            for (std::size_t k = 0; k < 3; ++k) {
                wrench[k] += force[k];
                wrench[3 + k] += torque[k];
            }
        }
        // The wheel joint: the tyre's reaction, the brake, the drive.
        const double axleBrake = v.maxBrakeTorque * (w.front ? v.brakeBias : 1.0 - v.brakeBias) /
                                 std::max(1, w.front ? frontCount : rearCount);
        double brakeTorque = brake * axleBrake;
        if (v.abs && brake > 0.0 && kappa < -kAbsSlip) {
            brakeTorque *= std::clamp(1.0 - (-kappa - kAbsSlip) / kAbsBand, 0.0, 1.0);
        }
        // The brake never reverses the wheel: at most what stops it within the step.
        brakeTorque = std::min(brakeTorque, std::max(w.spinInertia, 1e-3) * std::abs(omega) / dt);
        double load = -w.spin * fx * w.radius - brakeTorque * std::tanh(omega / kBrakeSpeed);
        if (ideal_) {
            // The wheels roll with the ideal speed.
            load = (w.spin * s.idealSpeed / w.radius - omega) * idealGain * 0.5;
        }
        bus.write(ws.loadTorque, load);
        if (ws.driveCommand.valid()) {
            bus.write(ws.driveCommand, ideal_ || driven == 0 ? 0.0 : w.spin * driveTotal / driven);
        }
        if (ws.steerCommand.valid()) {
            // PD on the road-wheel angle, critically damped at kSteerBandwidth.
            const double inertia = std::max(w.steerInertia, 1e-3);
            const double target = w.steerSign * steer[i];
            const double torque = inertia * kSteerBandwidth * kSteerBandwidth *
                                      (target - bus.read(ws.steerPosition)) -
                                  2.0 * inertia * kSteerBandwidth * bus.read(ws.steerVelocity);
            bus.write(ws.steerCommand, torque);
        }
        bus.write(ws.slipRatio, kappa);
        bus.write(ws.slipAngle, alpha);
        bus.write(ws.normalForce, fz[i]);
        bus.write(ws.steerAngleOut, steer[i]);
    }

    if (ideal_) {
        // Kinematic bicycle: speed from throttle and brake, yaw rate v·tan δ / L.
        double radius = 0.0;
        for (const Wheel& w : v.wheels) {
            radius += w.radius / static_cast<double>(n);
        }
        const double driveForce =
            v.engine ? std::abs(throttle) *
                           engineTorque(v, v.engineSpeeds.empty() ? 0.0 : v.engineSpeeds.back()) *
                           v.gearRatios[0] * v.finalDrive * sign(throttle) / radius
                     : throttle * v.maxDriveTorque / radius;
        const double brakeForce = brake * v.maxBrakeTorque / radius;
        s.idealSpeed += driveForce / v.mass * dt;
        if (brakeForce > 0.0) {
            const double slow = brakeForce / v.mass * dt;
            s.idealSpeed =
                std::abs(s.idealSpeed) <= slow ? 0.0 : s.idealSpeed - sign(s.idealSpeed) * slow;
        }
        const Vec3 target = mul(forwardLevel, s.idealSpeed);
        const double yawRate =
            v.wheelbase > 0.0 ? s.idealSpeed * std::tan(s.delta) / v.wheelbase : 0.0;
        // Rough yaw inertia: the vehicle's mass over its footprint.
        double izz = 0.0;
        for (const Wheel& w : v.wheels) {
            izz += v.mass / static_cast<double>(n) *
                   (w.anchor[0] * w.anchor[0] + w.anchor[1] * w.anchor[1]);
        }
        wrench[0] += v.mass * (target[0] - lin[0]) * idealGain;
        wrench[1] += v.mass * (target[1] - lin[1]) * idealGain;
        wrench[5] += izz * (yawRate - ang[2]) * idealGain;
        // Hold roll and pitch level, as the tyres would.
        wrench[3] += -izz * ang[0] * idealGain;
        wrench[4] += -izz * ang[1] * idealGain;
    }
    bus.writeN(s.wrench, wrench);
    bus.write(s.speed, ideal_ ? s.idealSpeed : speed);
    bus.write(s.engineSpeed, engineSpeed);
}

Status VehicleDynamics::saveState(StateWriter& writer) const {
    for (const VehicleState& s : vehicles_) {
        writer.put(s.delta);
        writer.put(s.currentGear);
        writer.put(s.shiftHold);
        writer.put(s.idealSpeed);
        for (const WheelState& w : s.wheels) {
            writer.put(w.longDeflection);
            writer.put(w.latDeflection);
            writer.put(w.compression);
        }
    }
    return {};
}

Status VehicleDynamics::restoreState(StateReader& reader) {
    for (VehicleState& s : vehicles_) {
        bool ok = reader.get(s.delta) && reader.get(s.currentGear) && reader.get(s.shiftHold) &&
                  reader.get(s.idealSpeed);
        for (WheelState& w : s.wheels) {
            ok = ok && reader.get(w.longDeflection) && reader.get(w.latDeflection) &&
                 reader.get(w.compression);
        }
        if (!ok) {
            return Status::error(StatusCode::Internal, "vehicle dynamics: truncated state");
        }
    }
    return {};
}

}  // namespace lagrange::vehicle_dynamics
