// Kind `wheel_odometry` (sensors.md §9): dead reckoning of a differential drive from its wheel
// encoders, as a robot's base controller computes it. It counts each wheel's encoder ticks
// (joint/<j>/position quantised to the encoder's counts per revolution), turns them into distance
// with the radius it believes (its parameter, with an error if asked), and integrates the
// unicycle model exactly over each step. Nothing models slip: the body slides when the physics
// says so and the wheels' turns no longer match its motion, so slip shows up in the odometry as
// it does on a real robot. The odometry frame is the body's pose at the first step, flat.
#include "KindScene.hpp"

#include <cmath>
#include <numbers>

namespace lagrange::sensors::kinds {

namespace {

constexpr double kTwoPi = 2.0 * std::numbers::pi;

class WheelOdometry final : public KindModule {
public:
    WheelOdometry(const World& world, std::uint32_t sensor, bool groundTruth)
        : KindModule(world, sensor, groundTruth) {}

    Status declare(const WorldView& world, const ModuleConfig&, SignalDeclarer& d) override {
        const World& raw = world.raw();
        const std::string fail = "wheel_odometry " + name_ + ": ";
        std::array<std::uint32_t, 2> joint{KindScene::kNone, KindScene::kNone};
        const std::string left = ownJointName(raw, sensor_, kindText(raw, sensor_, "left_joint"));
        const std::string right = ownJointName(raw, sensor_, kindText(raw, sensor_, "right_joint"));
        if (!left.empty() || !right.empty()) {
            joint = {findJoint(raw, left, KindScene::kNone),
                     findJoint(raw, right, KindScene::kNone)};
            if (left.empty() || right.empty() || joint[0] == KindScene::kNone ||
                joint[1] == KindScene::kNone) {
                return Status::error(StatusCode::InvalidConfig,
                                     fail + "left_joint and right_joint must name two joints");
            }
        } else if (!detect(raw, joint)) {
            return Status::error(StatusCode::InvalidConfig,
                                 fail +
                                     "no left_joint and right_joint, and the body has not "
                                     "exactly two driven wheels");
        }
        // What the robot believes: its parameters, else the wheel colliders and positions.
        const std::vector<Pose> rest = restPoses(raw);
        const Pose& baseRest = rest[body_];
        double radius = kindSetting(raw, sensor_, "wheel_radius", 0.0);
        if (radius <= 0.0) {
            radius = wheelRadius(raw, raw.joints.child[joint[0]].index);
        }
        double separation = kindSetting(raw, sensor_, "wheel_separation",
                                        kindSetting(raw, sensor_, "track_width", 0.0));
        if (separation <= 0.0) {
            const Pose a = relative(baseRest, rest[raw.joints.child[joint[0]].index]);
            const Pose b = relative(baseRest, rest[raw.joints.child[joint[1]].index]);
            separation = std::abs(a.position[1] - b.position[1]);
        }
        if (radius <= 0.0 || separation <= 0.0) {
            return Status::error(StatusCode::InvalidConfig,
                                 fail + "needs wheel_radius and wheel_separation (m)");
        }
        const double error = kindSetting(raw, sensor_, "radius_error", 0.0);
        radius_[0] = radius * (1.0 + kindSetting(raw, sensor_, "radius_error_left", error));
        radius_[1] = radius * (1.0 + kindSetting(raw, sensor_, "radius_error_right", error));
        separation_ = separation * (1.0 + kindSetting(raw, sensor_, "separation_error", 0.0));
        if (!(radius_[0] > 0.0 && radius_[1] > 0.0 && separation_ > 0.0)) {
            // An error of -1 or below believes a wheel of no size or no track: the turn
            // dψ = (s_r − s_l)/b would divide by zero and the pose read NaN for ever.
            return Status::error(StatusCode::InvalidConfig,
                                 fail + "radius_error and separation_error must be above -1");
        }
        counts_ = kindSetting(raw, sensor_, "encoder_resolution", 0.0);  // per revolution
        if (counts_ < 0.0) {
            return Status::error(StatusCode::InvalidConfig, fail + "encoder_resolution below 0");
        }
        for (std::size_t w = 0; w < 2; ++w) {
            const std::uint32_t j = joint[w];
            if (raw.joints.type[j] != JointType::Revolute &&
                raw.joints.type[j] != JointType::Continuous) {
                return Status::error(StatusCode::InvalidConfig,
                                     fail + raw.joints.names[j] + " is not a wheel joint");
            }
            // Which way the wheel turns to roll forward: its axis a in the base frame, the wheel
            // centre moving as ω·a × (r·ẑ), whose forward part is ω·r·a_y.
            const Pose wheel = relative(baseRest, rest[raw.joints.child[j].index]);
            const std::array<double, 3> a = rotate(wheel.orientation, raw.joints.axis[j]);
            sign_[w] = a[1] >= 0.0 ? 1.0 : -1.0;
            position_[w] = d.input(kindSpec(
                "joint/" + std::string(world.signalInstanceName(raw.joints.prim[j])) + "/position",
                SignalType::Scalar, units::rad));
        }
        poses_.watch(raw, d, body_);
        if (!poses_.moves(body_)) {
            return Status::error(StatusCode::InvalidConfig, fail + "its body never moves");
        }
        twist_ = d.input(kindSpec("body/" + raw.bodies.names[body_] + "/twist", SignalType::Wrench,
                                  units::m_per_s, "world_local"));
        pose_ = d.output(kindSpec(base() + "/pose", SignalType::Pose, units::m, "odom"));
        yawOut_ = d.output(kindSpec(base() + "/yaw", SignalType::Scalar, units::rad));
        velocity_ =
            d.output(kindSpec(base() + "/twist", SignalType::Wrench, units::m_per_s, "body"));
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        started_ = false;
        x_ = y_ = yaw_ = 0.0;
        write(bus, 0.0, 0.0);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        const Pose body = poses_.at(bus, body_);
        if (!started_) {
            // The odometry frame: where the body starts, flat.
            const double yaw = yawOf(body.orientation);
            origin_ = {{body.position[0], body.position[1], 0.0},
                       {std::cos(yaw / 2.0), 0.0, 0.0, std::sin(yaw / 2.0)}};
            for (std::size_t w = 0; w < 2; ++w) {
                last_[w] = ticks(bus.read(position_[w]));
            }
            started_ = true;
            write(bus, 0.0, 0.0);
            return {};
        }
        const double dt = context.dt.seconds();
        if (groundTruth_) {
            // Exact: the body's own pose in the odometry frame and its speeds in its frame.
            const Pose p = relative(origin_, body);
            x_ = p.position[0];
            y_ = p.position[1];
            yaw_ = yawOf(p.orientation);
            const std::span<const double> t = bus.readN(twist_);
            const std::array<double, 3> v = rotate(conjugate(body.orientation), {t[0], t[1], t[2]});
            const std::array<double, 3> w = rotate(conjugate(body.orientation), {t[3], t[4], t[5]});
            write(bus, v[0], w[2]);
            return {};
        }
        std::array<double, 2> travel{};
        for (std::size_t w = 0; w < 2; ++w) {
            const double now = ticks(bus.read(position_[w]));
            travel[w] = sign_[w] * (now - last_[w]) * angle() * radius_[w];
            last_[w] = now;
        }
        const double ds = (travel[0] + travel[1]) / 2.0;
        const double dyaw = (travel[1] - travel[0]) / separation_;
        // Exact over the step for constant wheel speeds: an arc (a line when it does not turn).
        const double chord = std::abs(dyaw) > 1e-12 ? 2.0 * ds / dyaw * std::sin(dyaw / 2.0) : ds;
        x_ += chord * std::cos(yaw_ + dyaw / 2.0);
        y_ += chord * std::sin(yaw_ + dyaw / 2.0);
        yaw_ = std::remainder(yaw_ + dyaw, kTwoPi);
        write(bus, dt > 0.0 ? ds / dt : 0.0, dt > 0.0 ? dyaw / dt : 0.0);
        return {};
    }

    Status saveState(StateWriter& w) const override {
        w.put(started_);
        w.put(x_);
        w.put(y_);
        w.put(yaw_);
        w.put(last_);
        w.put(origin_);
        return {};
    }
    Status restoreState(StateReader& r) override {
        r.get(started_);
        r.get(x_);
        r.get(y_);
        r.get(yaw_);
        r.get(last_);
        r.get(origin_);
        return r.failed() ? Status::error(StatusCode::InvalidConfig, "short state") : Status{};
    }
    void shiftOrigin(const std::array<double, 3>& shift) override {
        origin_.position[0] -= shift[0];
        origin_.position[1] -= shift[1];
    }

private:
    /// The encoder's reading: whole counts, or the exact angle without an encoder resolution.
    [[nodiscard]] double ticks(double angleRad) const {
        if (counts_ <= 0.0 || groundTruth_) {
            return angleRad;
        }
        return std::floor(angleRad / kTwoPi * counts_);
    }
    [[nodiscard]] double angle() const {
        return counts_ <= 0.0 || groundTruth_ ? 1.0 : kTwoPi / counts_;
    }

    void write(SignalBus& bus, double v, double w) {
        bus.writeN(pose_, std::array<double, 7>{x_, y_, 0.0, std::cos(yaw_ / 2.0), 0.0, 0.0,
                                                std::sin(yaw_ / 2.0)});
        bus.write(yawOut_, yaw_);
        bus.writeN(velocity_, std::array<double, 6>{v, 0.0, 0.0, 0.0, 0.0, w});
    }

    /// Two driven hinges whose wheels hang directly off the sensor's body: left is +y. A robot
    /// added to a run (spawned, --add-robot) is passive, none of its joints driven: then two
    /// such hinges, driven or not.
    bool detect(const World& world, std::array<std::uint32_t, 2>& joint) const {
        std::vector<std::uint32_t> wheels;
        for (const bool driven : {true, false}) {
            wheels.clear();
            for (std::uint32_t j = 0; j < world.joints.names.size(); ++j) {
                const bool hinge = world.joints.type[j] == JointType::Revolute ||
                                   world.joints.type[j] == JointType::Continuous;
                if (hinge && (!driven || world.joints.actuated[j]) &&
                    world.joints.parent[j].index == body_ &&
                    wheelRadius(world, world.joints.child[j].index) > 0.0) {
                    wheels.push_back(j);
                }
            }
            if (wheels.size() == 2) {
                break;
            }
        }
        if (wheels.size() != 2) {
            return false;
        }
        const std::vector<Pose> rest = restPoses(world);
        const double y0 =
            relative(rest[body_], rest[world.joints.child[wheels[0]].index]).position[1];
        const double y1 =
            relative(rest[body_], rest[world.joints.child[wheels[1]].index]).position[1];
        joint = y0 > y1 ? std::array{wheels[0], wheels[1]} : std::array{wheels[1], wheels[0]};
        return true;
    }

    /// A wheel's radius: its first cylinder or sphere collider's, else 0.
    static double wheelRadius(const World& world, std::uint32_t body) {
        for (std::size_t c = 0; c < world.colliders.body.size(); ++c) {
            if (world.colliders.body[c].index == body &&
                (world.colliders.shape[c] == ShapeType::Cylinder ||
                 world.colliders.shape[c] == ShapeType::Sphere)) {
                return world.colliders.size[c][0];
            }
        }
        return 0.0;
    }

    BodyPoses poses_;
    std::array<SignalId, 2> position_;
    SignalId twist_, pose_, yawOut_, velocity_;
    std::array<double, 2> radius_{}, sign_{1.0, 1.0}, last_{};
    double separation_{0.0}, counts_{0.0};
    bool started_{false};
    double x_{0.0}, y_{0.0}, yaw_{0.0};
    Pose origin_;
};

}  // namespace

std::unique_ptr<IModule> makeWheelOdometry(const World& world, std::uint32_t sensor,
                                           bool groundTruth, const KindContext&) {
    return std::make_unique<WheelOdometry>(world, sensor, groundTruth);
}

}  // namespace lagrange::sensors::kinds
