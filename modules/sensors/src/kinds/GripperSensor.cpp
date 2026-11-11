// Built-in kind `gripper_sensor` (sensors.md §9): what a gripper's controller knows about its
// grasp: the opening, the motor current, whether it holds something, and whether that slips.
//
// Fingers are joints (text parameter `fingers`); each finger is the joint's child body. The
// opening is width_offset + width_scale · Σ finger positions. The current is the motors' torque
// current motor/<m>/current_q when `motors` names them, else |joint/<j>/torque| / torque_constant
// summed over the fingers (a proxy). A finger touches when body/<f>/contact_force exceeds
// contact_threshold; an object is held when every finger touches and the gripper is open wider
// than closed_width (fingers pressing on each other are not an object). The object is the body
// `object` names, else a moving body outside the gripper's body tree within reach of the finger.
// It slips when the object's velocity at the finger, less the finger's, has a part across the
// contact force (the tangential velocity) above slip_threshold.
#include "KindModule.hpp"
#include "ObstacleScene.hpp"
#include "lagrange/random/Philox.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace lagrange::sensors::kinds {

namespace {

constexpr double kDefaultContactThresholdN = 0.5;   // illustrative
constexpr double kDefaultSlipThresholdMps = 0.005;  // illustrative: 5 mm/s
constexpr double kDefaultClosedWidthM = 0.001;
constexpr double kDefaultContactMarginM = 0.005;  // how far from a finger an object is sought

double norm(const Vec3& v) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

class GripperSensor final : public KindModule {
public:
    using KindModule::KindModule;

    Status declare(const WorldView& view, const ModuleConfig&, SignalDeclarer& d) override {
        const World& world = view.raw();
        const auto fail = [&](const std::string& why) {
            return Status::error(StatusCode::InvalidConfig, "gripper_sensor " + name_ + ": " + why);
        };
        widthScale_ = kindSetting(world, sensor_, "width_scale", 1.0);
        widthOffset_ = kindSetting(world, sensor_, "width_offset", 0.0);
        torqueConstant_ = kindSetting(world, sensor_, "torque_constant", 1.0);
        contactThreshold_ =
            kindSetting(world, sensor_, "contact_threshold", kDefaultContactThresholdN);
        slipThreshold_ = kindSetting(world, sensor_, "slip_threshold", kDefaultSlipThresholdMps);
        closedWidth_ = kindSetting(world, sensor_, "closed_width", kDefaultClosedWidthM);
        widthNoise_ = kindSetting(world, sensor_, "width_noise", 0.0);
        currentNoise_ = kindSetting(world, sensor_, "current_noise", 0.0);
        const double margin = kindSetting(world, sensor_, "contact_margin", kDefaultContactMarginM);
        if (torqueConstant_ <= 0.0 || contactThreshold_ < 0.0 || slipThreshold_ < 0.0 ||
            widthNoise_ < 0.0 || currentNoise_ < 0.0 || margin < 0.0) {
            return fail(
                "torque_constant must be positive; thresholds, noises and contact_margin not "
                "negative");
        }
        poses_.reset(world);
        fingers_.clear();
        for (const std::string& word : words(kindText(world, sensor_, "fingers"))) {
            const std::string joint = ownJointName(world, sensor_, word);
            const auto it = std::ranges::find(world.joints.names, joint);
            if (it == world.joints.names.end()) {
                return fail("no joint '" + joint + "' (fingers)");
            }
            const auto j = static_cast<std::size_t>(it - world.joints.names.begin());
            Finger f;
            f.body = world.joints.child[j].index;
            if (!poses_.moving(f.body)) {
                return fail("finger joint '" + joint + "' does not move");
            }
            const std::string base =
                "joint/" + std::string(view.signalInstanceName(world.joints.prim[j]));
            const bool prismatic = world.joints.type[j] == JointType::Prismatic;
            f.position = d.input(kindSpec(base + "/position", SignalType::Scalar,
                                          prismatic ? units::m : units::rad));
            f.torque = d.input(optionalKindSpec(base + "/torque", SignalType::Scalar,
                                                prismatic ? units::N : units::N_m));
            f.contact = d.input(kindSpec("body/" + world.bodies.names[f.body] + "/contact_force",
                                         SignalType::Vec3, units::N, "world_local"));
            poses_.watch(d, f.body, true);
            // How far a finger reaches from its body origin.
            for (std::size_t c = 0; c < world.colliders.body.size(); ++c) {
                if (world.colliders.body[c].index == f.body &&
                    world.colliders.body[c] != kWorldBody) {
                    f.reach = std::max(f.reach, norm(world.colliders.poseInBody[c].position) +
                                                    boundingRadius(world.colliders.shape[c],
                                                                   world.colliders.size[c]));
                }
            }
            f.reach += margin;
            fingers_.push_back(f);
        }
        if (fingers_.empty()) {
            return fail("needs `fingers`, the finger joints");
        }
        motors_.clear();
        for (const std::string& motor : words(kindText(world, sensor_, "motors"))) {
            motors_.push_back(d.input(
                optionalKindSpec("motor/" + ownPrimName(world, sensor_, motor) + "/current_q",
                                 SignalType::Scalar, units::A)));
        }
        // Candidate objects: moving bodies of another robot (or none), or the one named.
        const std::uint32_t root = rootOf(world, fingers_.front().body);
        const std::string object = ownBodyName(world, sensor_, kindText(world, sensor_, "object"));
        objectBody_ = -1;
        if (!object.empty()) {
            const auto it = std::ranges::find(world.bodies.names, object);
            if (it == world.bodies.names.end()) {
                return fail("no body '" + object + "' (object)");
            }
            objectBody_ = static_cast<std::int64_t>(it - world.bodies.names.begin());
            poses_.watch(d, static_cast<std::uint32_t>(objectBody_), true);
        }
        scene_.declare(world, poses_, d, [&](std::uint32_t c) {
            const BodyId owner = world.colliders.body[c];
            if (owner == kWorldBody || !poses_.moving(owner.index)) {
                return false;
            }
            if (objectBody_ >= 0) {
                return owner.index == static_cast<std::uint32_t>(objectBody_);
            }
            return rootOf(world, owner.index) != root;
        });
        for (std::uint32_t i = 0; i < scene_.size(); ++i) {
            poses_.watch(d, world.colliders.body[scene_.collider(i)].index, true);
        }
        widthOut_ = d.output(kindSpec(base() + "/width", SignalType::Scalar, units::m));
        currentOut_ = d.output(kindSpec(base() + "/current", SignalType::Scalar, units::A));
        gripOut_ = d.output(kindSpec(base() + "/grip_force", SignalType::Scalar, units::N));
        detectedOut_ =
            d.output(kindSpec(base() + "/object_detected", SignalType::Bool, units::one));
        slipOut_ = d.output(kindSpec(base() + "/slip", SignalType::Bool, units::one));
        slipSpeedOut_ =
            d.output(kindSpec(base() + "/slip_speed", SignalType::Scalar, units::m_per_s));
        world_ = &world;
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        bus.write(widthOut_, kNaN);
        bus.write(currentOut_, kNaN);
        bus.write(gripOut_, 0.0);
        bus.write(detectedOut_, 0.0);
        bus.write(slipOut_, 0.0);
        bus.write(slipSpeedOut_, 0.0);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        random::Stream rng(context.seed, context.stepIndex, 0);
        double width = widthOffset_;
        double current = 0.0;
        bool proxy = motors_.empty();
        for (const SignalId m : motors_) {
            const double q = bus.read(m);
            if (!std::isfinite(q)) {
                proxy = true;  // electrical domain off
                break;
            }
            current += std::abs(q);
        }
        if (proxy) {
            current = 0.0;
        }
        bool all = true;
        double grip = std::numeric_limits<double>::infinity();
        double slipSpeed = 0.0;
        bool placed = false;
        for (const Finger& f : fingers_) {
            width += widthScale_ * bus.read(f.position);
            if (proxy) {
                const double tau = bus.read(f.torque);
                current += std::isfinite(tau) ? std::abs(tau) / torqueConstant_ : 0.0;
            }
            const std::span<const double> fv = bus.readN(f.contact);
            const Vec3 force{fv[0], fv[1], fv[2]};
            const double magnitude = norm(force);
            grip = std::min(grip, magnitude);
            if (!(magnitude > contactThreshold_)) {
                all = false;
                continue;
            }
            // The object at this finger and its velocity across the contact force.
            if (!placed) {
                scene_.place(bus, poses_);
                placed = true;
            }
            const Vec3 at = poses_.pose(bus, f.body).position;
            std::int64_t object = -1;
            for (const std::uint32_t i : scene_.overlap(at, f.reach)) {
                object = world_->colliders.body[scene_.collider(i)].index;
                break;
            }
            if (object < 0) {
                continue;
            }
            const Vec3 vo = poses_.pointVelocity(bus, static_cast<std::uint32_t>(object), at);
            const Vec3 vf = poses_.pointVelocity(bus, f.body, at);
            Vec3 rel{vo[0] - vf[0], vo[1] - vf[1], vo[2] - vf[2]};
            const double along =
                (rel[0] * force[0] + rel[1] * force[1] + rel[2] * force[2]) / magnitude;
            for (int k = 0; k < 3; ++k) {
                rel[k] -= along * force[k] / magnitude;
            }
            slipSpeed = std::max(slipSpeed, norm(rel));
        }
        const bool detected = all && width > closedWidth_;
        if (!groundTruth_) {
            width += widthNoise_ * rng.normal();
            current = std::max(0.0, current + currentNoise_ * rng.normal());
        }
        bus.write(widthOut_, width);
        bus.write(currentOut_, current);
        bus.write(gripOut_, all ? grip : 0.0);
        bus.write(detectedOut_, detected ? 1.0 : 0.0);
        bus.write(slipOut_, detected && slipSpeed > slipThreshold_ ? 1.0 : 0.0);
        bus.write(slipSpeedOut_, detected ? slipSpeed : 0.0);
        return {};
    }

private:
    /// The root of the body tree `body` is in (the gripper's robot, or a free object itself).
    static std::uint32_t rootOf(const World& world, std::uint32_t body) {
        for (std::size_t n = 0; n < world.bodies.parent.size(); ++n) {
            const BodyId parent = world.bodies.parent[body];
            if (parent == kWorldBody) {
                break;
            }
            body = parent.index;
        }
        return body;
    }

    struct Finger {
        std::uint32_t body{0};
        double reach{0.0};  // m from the body origin, contact_margin included
        SignalId position, torque, contact;
    };

    double widthScale_{1.0}, widthOffset_{0.0}, torqueConstant_{1.0};
    double contactThreshold_{kDefaultContactThresholdN}, slipThreshold_{kDefaultSlipThresholdMps};
    double closedWidth_{kDefaultClosedWidthM}, widthNoise_{0.0}, currentNoise_{0.0};
    std::int64_t objectBody_{-1};
    std::vector<Finger> fingers_;
    std::vector<SignalId> motors_;
    const World* world_{nullptr};
    BodyPoses poses_;
    ObstacleScene scene_;
    SignalId widthOut_, currentOut_, gripOut_, detectedOut_, slipOut_, slipSpeedOut_;
};

}  // namespace

std::unique_ptr<IModule> createGripperSensor(const World& world, std::uint32_t sensor,
                                             bool groundTruth, const KindContext&) {
    return std::make_unique<GripperSensor>(world, sensor, groundTruth);
}

}  // namespace lagrange::sensors::kinds
