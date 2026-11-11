// Kind "vehicle_can" (sensors.md §9): what a production car's own sensors put on its CAN bus.
// Per wheel the ABS wheel-speed sensor (a toothed tone ring: speed from the time between its
// teeth's edges), the steering wheel angle sensor (the road wheels' angle times the steering
// ratio), the ESP sensor cluster (yaw rate, longitudinal and lateral acceleration, with noise) and
// the gear. Only on a VehicleAPI_1 robot: its wheels are the WheelAPI_1 joints under the chassis.
//
// Inputs (invariant 1): each wheel joint's position and velocity (physics), each steering
// joint's position, or the vehicle domain's wheel/<j>/steer_angle where a steered wheel has no
// steering joint, the chassis's pose and twist, and vehicle/<v>/gear (optional: NaN without
// the vehicle domain).
#include "EnvCommon.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>
#include <vector>

namespace lagrange::sensors::kinds {

// Shared with the tests by name; in an inline namespace of their own (no clash with
// other kinds' helpers).
inline namespace extra2 {

/// The bicycle-model steering angle (rad) from a road wheel's angle `wheel` (rad) at `y` metres
/// left of the centre line, for a wheelbase `wheelbase`: the inverse of Ackermann's
/// tan δw = L·tan δ / (L − y·tan δ) (vehicle_dynamics' ackermannAngle).
double bicycleAngle(double wheel, double wheelbase, double y) {
    if (wheel == 0.0 || wheelbase <= 0.0) {
        return wheel;
    }
    const double t = std::tan(wheel);
    return std::atan(wheelbase * t / (wheelbase + y * t));
}

}  // namespace extra2

namespace {

constexpr double kTwoPi = 2.0 * std::numbers::pi;
// A passive or active ABS tone ring of 48 teeth (illustrative: 43 to 100 are common).
constexpr double kTeeth = 48.0;
// Steering wheel to road wheel, about 15:1 to 16:1 on passenger cars (illustrative).
constexpr double kSteeringRatio = 15.5;
// Steering angle sensor resolution 0.1° (illustrative of the CAN signals of such sensors).
constexpr double kSteeringResolution = 0.1 * std::numbers::pi / 180.0;  // rad
// ESP cluster noise, 1σ (illustrative of MEMS yaw-rate and acceleration sensors).
constexpr double kYawRateNoise = 0.1 * std::numbers::pi / 180.0;  // rad/s
constexpr double kAccelerationNoise = 0.03;                       // m/s²

double sign(double v) {
    return v >= 0.0 ? 1.0 : -1.0;
}

Pose relativePose(const Pose& frame, const Pose& p) {
    const std::array<double, 4> inv = conjugate(frame.orientation);
    return {rotate(inv, sub(p.position, frame.position)), multiply(inv, p.orientation)};
}

class VehicleCan final : public KindModule {
public:
    VehicleCan(const World& world, std::uint32_t sensor, bool groundTruth)
        : KindModule(world, sensor, groundTruth) {}

    Status declare(const WorldView& view, const ModuleConfig&, SignalDeclarer& d) override {
        const World& w = view.raw();
        const std::string fail = "vehicle_can " + name_ + ": ";
        teeth_ = kindSetting(w, sensor_, "teeth", kTeeth);
        ratio_ = kindSetting(w, sensor_, "steering_ratio", kSteeringRatio);
        steerResolution_ = kindSetting(w, sensor_, "steering_resolution", kSteeringResolution);
        yawNoise_ = kindSetting(w, sensor_, "yaw_rate_noise", kYawRateNoise);
        accelNoise_ = kindSetting(w, sensor_, "acceleration_noise", kAccelerationNoise);
        if (teeth_ < 1.0 || ratio_ <= 0.0 || steerResolution_ < 0.0 || yawNoise_ < 0.0 ||
            accelNoise_ < 0.0) {
            return Status::error(StatusCode::InvalidConfig,
                                 fail + "teeth >= 1, steering_ratio > 0, noise >= 0");
        }
        // The vehicle: a VehicleAPI_1 body of the sensor's robot.
        const std::uint32_t robot = w.bodies.robot[body_];
        std::optional<std::uint32_t> chassis;
        for (const SchemaInstance* inst : view.instancesOf("VehicleAPI_1")) {
            for (std::uint32_t b = 0; b < w.bodies.prim.size(); ++b) {
                if (w.bodies.prim[b] == inst->prim && w.bodies.robot[b] == robot) {
                    chassis = b;
                }
            }
        }
        if (!chassis) {
            return Status::error(StatusCode::InvalidConfig,
                                 fail + "needs a VehicleAPI_1 robot (a car); its robot has none");
        }
        chassis_ = *chassis;
        std::string vehicle = robot < w.robots.size() ? w.robots[robot] : w.bodies.names[chassis_];
        const std::vector<Pose> rest = restPoses(w);
        const Pose& chassisRest = rest[chassis_];
        const auto jointOf = [&](PrimId prim) -> std::optional<std::uint32_t> {
            for (std::uint32_t j = 0; j < w.joints.prim.size(); ++j) {
                if (w.joints.prim[j] == prim) {
                    return j;
                }
            }
            return std::nullopt;
        };
        wheels_.clear();
        double front = 0.0, rear = 0.0;
        int nf = 0, nr = 0;
        for (const SchemaInstance* inst : view.instancesOf("WheelAPI_1")) {
            const auto joint = jointOf(inst->prim);
            if (!joint) {
                continue;
            }
            bool ours = false;
            for (BodyId b = w.bodies.parent[w.joints.child[*joint].index]; b != kWorldBody;
                 b = w.bodies.parent[b.index]) {
                ours = ours || b.index == chassis_;
            }
            if (!ours) {
                continue;
            }
            Wheel wheel;
            wheel.name = std::string(view.signalInstanceName(inst->prim));
            wheel.radius = inst->real("wheel:radius").value_or(0.0);
            if (wheel.radius <= 0.0) {
                return Status::error(StatusCode::InvalidConfig,
                                     fail + wheel.name + ": wheel:radius is required");
            }
            const Pose wheelRest = rest[w.joints.child[*joint].index];
            wheel.y = relativePose(chassisRest, wheelRest).position[1];
            const bool isFront = inst->token("wheel:axle").value_or("") == "front";
            const double x = relativePose(chassisRest, wheelRest).position[0];
            (isFront ? front : rear) += x;
            ++(isFront ? nf : nr);
            const std::string joint_ = "joint/" + wheel.name + "/";
            wheel.position = d.input(kindSpec(joint_ + "position", SignalType::Scalar, units::rad));
            wheel.velocity =
                d.input(kindSpec(joint_ + "velocity", SignalType::Scalar, units::rad_per_s));
            if (inst->integer("wheel:steered").value_or(0) != 0) {
                wheel.steered = true;
                const std::span<const PrimId> steer = inst->rel("wheel:steerJoint");
                const auto sj = steer.empty() ? std::nullopt : jointOf(steer[0]);
                if (sj) {
                    // +1 when a positive joint angle turns the wheel left (its axis up).
                    const Pose knuckle = rest[w.joints.child[*sj].index];
                    const Vec3 axis = rotate(conjugate(chassisRest.orientation),
                                             rotate(knuckle.orientation, w.joints.axis[*sj]));
                    wheel.steerSign = sign(axis[2]);
                    wheel.steer = d.input(kindSpec(
                        "joint/" + std::string(view.signalInstanceName(w.joints.prim[*sj])) +
                            "/position",
                        SignalType::Scalar, units::rad));
                } else {
                    wheel.steer = d.input(optionalNaN("wheel/" + wheel.name + "/steer_angle",
                                                      SignalType::Scalar, units::rad));
                }
            }
            wheels_.push_back(std::move(wheel));
        }
        if (wheels_.empty()) {
            return Status::error(StatusCode::InvalidConfig, fail + "its vehicle has no wheels");
        }
        wheelbase_ = nf > 0 && nr > 0 ? front / nf - rear / nr : 0.0;
        const std::string body = "body/" + w.bodies.names[chassis_];
        if (!movingBodies(w)[chassis_]) {
            return Status::error(StatusCode::InvalidConfig, fail + "its chassis never moves");
        }
        pose_ = d.input(kindSpec(body + "/pose", SignalType::Pose, units::m, "world_local"));
        twist_ =
            d.input(kindSpec(body + "/twist", SignalType::Wrench, units::m_per_s, "world_local"));
        gear_ = d.input(optionalNaN("vehicle/" + vehicle + "/gear", SignalType::Int, units::one));
        // The mount in the chassis: where the ESP cluster sits.
        mountInChassis_ = relativePose(chassisRest, compose(rest[body_], mount_));
        g_ = w.gravityLocal;
        const auto n = static_cast<std::uint32_t>(wheels_.size());
        SignalSpec speeds =
            kindSpec(base() + "/wheel_speeds", SignalType::ScalarArray, units::m_per_s);
        speeds.arrayLength = n;
        wheelSpeeds_ = d.output(speeds);
        speed_ = d.output(kindSpec(base() + "/speed", SignalType::Scalar, units::m_per_s));
        steering_ =
            d.output(kindSpec(base() + "/steering_wheel_angle", SignalType::Scalar, units::rad));
        yawRate_ = d.output(kindSpec(base() + "/yaw_rate", SignalType::Scalar, units::rad_per_s));
        ax_ = d.output(
            kindSpec(base() + "/longitudinal_acceleration", SignalType::Scalar, units::m_per_s2));
        ay_ = d.output(
            kindSpec(base() + "/lateral_acceleration", SignalType::Scalar, units::m_per_s2));
        gearOut_ = d.output(kindSpec(base() + "/gear", SignalType::Scalar, units::one));
        valid_ = d.output(kindSpec(base() + "/valid", SignalType::Bool, units::one));
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        started_ = false;
        for (Wheel& wh : wheels_) {
            wh.lastEdge = kNaN;
            wh.speed = 0.0;
        }
        speeds_.assign(wheels_.size(), 0.0);
        bus.writeN(wheelSpeeds_, speeds_);
        for (const SignalId id : {speed_, steering_, yawRate_, ax_, ay_}) {
            bus.write(id, 0.0);
        }
        bus.write(gearOut_, kNaN);
        bus.write(valid_, 0.0);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        const double dt = context.dt.seconds();
        const std::span<const double> p = bus.readN(pose_);
        const Pose chassis{{p[0], p[1], p[2]}, {p[3], p[4], p[5], p[6]}};
        const Pose sensor = compose(chassis, mountInChassis_);
        const std::span<const double> t = bus.readN(twist_);
        const Vec3 omega{t[3], t[4], t[5]};
        const Vec3 arm = sub(sensor.position, chassis.position);
        const Vec3 spin = cross(omega, arm);
        const Vec3 v{t[0] + spin[0], t[1] + spin[1], t[2] + spin[2]};  // the sensor's, world
        // Wheel speeds as an ABS controller measures them: the time between tooth edges.
        const double now = (context.t + context.dt).seconds();
        double sum = 0.0;
        for (std::size_t i = 0; i < wheels_.size(); ++i) {
            Wheel& wh = wheels_[i];
            const double angle = bus.read(wh.position);
            speeds_[i] = groundTruth_ ? std::abs(bus.read(wh.velocity)) * wh.radius
                                      : toothSpeed(wh, angle, now, dt);
            wh.angle = angle;
            sum += speeds_[i];
        }
        // Steering wheel angle: the steered wheels' bicycle angle, averaged, times the ratio.
        double delta = 0.0;
        int steered = 0;
        for (const Wheel& wh : wheels_) {
            if (wh.steered) {
                const double angle = wh.steerSign * bus.read(wh.steer);
                if (std::isfinite(angle)) {
                    delta += bicycleAngle(angle, wheelbase_, wh.y);
                    ++steered;
                }
            }
        }
        double steering = steered > 0 ? ratio_ * delta / steered : 0.0;
        // ESP: yaw rate and specific force in the sensor frame (gravity shows when tilted).
        const std::array<double, 4> inv = conjugate(sensor.orientation);
        double yawRate = rotate(inv, omega)[2];
        // The first step has no earlier velocity: it reads as at rest.
        const double rate = started_ && dt > 0.0 ? 1.0 / dt : 0.0;
        const Vec3 accel{(v[0] - lastV_[0]) * rate - g_[0], (v[1] - lastV_[1]) * rate - g_[1],
                         (v[2] - lastV_[2]) * rate - g_[2]};
        const Vec3 a = rotate(inv, accel);
        double ax = a[0], ay = a[1];
        if (!groundTruth_) {
            steering = quantize(steering, steerResolution_);
            yawRate += yawNoise_ * gaussian(context, 0);
            ax += accelNoise_ * gaussian(context, 1);
            ay += accelNoise_ * gaussian(context, 2);
        }
        lastV_ = v;
        started_ = true;
        bus.writeN(wheelSpeeds_, speeds_);
        bus.write(speed_, sum / static_cast<double>(wheels_.size()));
        bus.write(steering_, steering);
        bus.write(yawRate_, yawRate);
        bus.write(ax_, ax);
        bus.write(ay_, ay);
        bus.write(gearOut_, bus.read(gear_));
        bus.write(valid_, 1.0);
        return {};
    }

    // The last velocity (acceleration by difference) and each wheel's tooth clock: a restored
    // run reads as the uninterrupted one, with no acceleration spike at the first step.
    Status saveState(StateWriter& w) const override {
        w.put(started_);
        w.put(lastV_);
        w.put<std::uint64_t>(wheels_.size());
        for (const Wheel& wh : wheels_) {
            w.put(wh.angle);
            w.put(wh.lastEdge);
            w.put(wh.speed);
        }
        w.putDoubles(speeds_);
        return {};
    }
    Status restoreState(StateReader& r) override {
        std::uint64_t n = 0;
        r.get(started_);
        r.get(lastV_);
        if (!r.get(n) || n != wheels_.size()) {
            return Status::error(StatusCode::InvalidConfig,
                                 "vehicle_can " + name_ + ": state of another run");
        }
        for (Wheel& wh : wheels_) {
            r.get(wh.angle);
            r.get(wh.lastEdge);
            r.get(wh.speed);
        }
        r.getDoubles(speeds_);
        return r.failed() ? Status::error(StatusCode::InvalidConfig,
                                          "vehicle_can " + name_ + ": short state")
                          : Status{};
    }

private:
    struct Wheel {
        std::string name;
        double radius{0.0}, y{0.0}, steerSign{1.0};
        double angle{0.0};      // rad, at the last step
        double lastEdge{kNaN};  // s, when the last tooth edge passed
        double speed{0.0};      // m/s, the last measurement
        bool steered{false};
        SignalId position, velocity, steer;
    };
    /// The wheel's speed from its tooth edges: the edges passed in this step over the time since
    /// the last edge before them, each edge's moment interpolated within the step (the angle
    /// taken as linear in it). Without an edge it falls as one tooth over the time since the
    /// last, as a period-measuring controller's estimate does when the wheel slows or stops.
    double toothSpeed(Wheel& wh, double angle, double now, double dt) const {
        const double pitch = kTwoPi / teeth_;  // rad per tooth
        const double before = std::floor(wh.angle / pitch);
        const double after = std::floor(angle / pitch);
        const double edges = std::abs(after - before);
        if (!started_ || dt <= 0.0) {
            return 0.0;
        }
        if (edges > 0.0) {
            // The last edge crossed: going forward the one at `after`, backward the one above it.
            const double edge = (angle > wh.angle ? after : after + 1.0) * pitch;
            const double at = now - dt + (edge - wh.angle) / (angle - wh.angle) * dt;
            if (!std::isnan(wh.lastEdge) && at > wh.lastEdge) {
                wh.speed = edges * pitch * wh.radius / (at - wh.lastEdge);
            }
            wh.lastEdge = at;
        } else if (!std::isnan(wh.lastEdge)) {
            wh.speed = std::min(wh.speed, pitch * wh.radius / (now - wh.lastEdge));
        }
        return wh.speed;
    }

    double teeth_{kTeeth}, ratio_{kSteeringRatio}, steerResolution_{kSteeringResolution};
    double yawNoise_{kYawRateNoise}, accelNoise_{kAccelerationNoise}, wheelbase_{0.0};
    std::uint32_t chassis_{0};
    std::vector<Wheel> wheels_;
    std::vector<double> speeds_;
    Pose mountInChassis_;
    Vec3 g_{}, lastV_{};
    bool started_{false};
    SignalId pose_, twist_, gear_, wheelSpeeds_, speed_, steering_, yawRate_, ax_, ay_, gearOut_,
        valid_;
};

}  // namespace

std::unique_ptr<IModule> makeVehicleCan(const World& world, std::uint32_t sensor, bool groundTruth,
                                        const KindContext&) {
    return std::make_unique<VehicleCan>(world, sensor, groundTruth);
}

}  // namespace lagrange::sensors::kinds
