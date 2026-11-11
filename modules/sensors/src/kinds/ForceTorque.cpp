// Kinds `force_torque` and `joint_torque` (sensors.md §9), both from what a joint carries:
// joint/<j>/reaction, what the parent does to the child at the child's origin, in the world frame
// (rigid_body.md "Contact force and force-torque"), which includes what its motor applies.
//
// `force_torque` is a six-axis wrist sensor in the manner of an ATI Mini45: the wrench moved to
// the sensor's origin and turned into its frame, then per-axis range (saturation), noise,
// resolution and a first-order low-pass. Preset `mini45_si_580_20` takes ATI's figures for the
// Mini45 with calibration SI-580-20 (ati-ia.com, F/T models, Mini45 "Standard Calibrations"):
// ranges Fx, Fy 580 N, Fz 1160 N, Tx, Ty, Tz 20 N·m; resolutions 1/4 N, Tx, Ty 1/188 N·m,
// Tz 1/376 N·m. ATI quotes those resolutions after dropping four counts of noise; the preset's
// noise of one resolution step per axis (1 sigma) is our assumption, illustrative.
//
// `joint_torque` is a link-side joint torque sensor as on a Franka Panda or a KUKA LBR iiwa: the
// reaction's moment about the joint's axis (its force along the axis for a slide), with range,
// noise, resolution, offset and low-pass. Its defaults are ideal; give a part's figures.
#include "KindScene.hpp"
#include "lagrange/random/Philox.hpp"

#include <cmath>
#include <limits>
#include <numbers>

namespace lagrange::sensors::kinds {

namespace {

using Vec3 = std::array<double, 3>;
constexpr double kInf = std::numeric_limits<double>::infinity();

// ATI Mini45, calibration SI-580-20 (ati-ia.com, Mini45 "Standard Calibrations", metric).
constexpr std::array<double, 6> kMini45Range{580.0, 580.0, 1160.0, 20.0, 20.0, 20.0};  // N, N·m
constexpr std::array<double, 6> kMini45Resolution{0.25,        0.25,        0.25,
                                                  1.0 / 188.0, 1.0 / 188.0, 1.0 / 376.0};

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

/// What both kinds share: the joint, its child's pose, its reaction, and the per-axis chain
/// range → noise → resolution → low-pass.
class JointLoadSensor : public KindModule {
public:
    using KindModule::KindModule;

protected:
    Status declareJoint(const WorldView& world, SignalDeclarer& d) {
        const World& raw = world.raw();
        joint_ = findJoint(raw, ownJointName(raw, sensor_, kindText(raw, sensor_, "joint")), body_);
        if (joint_ == KindScene::kNone) {
            return Status::error(StatusCode::InvalidConfig,
                                 kind_ + " " + name_ +
                                     ": no joint (name one with `joint`, or mount it on a joint's "
                                     "child link)");
        }
        const JointType type = raw.joints.type[joint_];
        if (type != JointType::Revolute && type != JointType::Continuous &&
            type != JointType::Prismatic) {
            return Status::error(StatusCode::InvalidConfig,
                                 kind_ + " " + name_ + ": joint " + raw.joints.names[joint_] +
                                     " reports no reaction (a hinge or slide does)");
        }
        child_ = raw.joints.child[joint_].index;
        poses_.watch(raw, d, child_);
        poses_.watch(raw, d, body_);
        SignalSpec reaction = kindSpec(
            "joint/" + std::string(world.signalInstanceName(raw.joints.prim[joint_])) + "/reaction",
            SignalType::Wrench, units::N, "world_local");
        reaction_ = d.input(reaction);
        cutoff_ = kindSetting(raw, sensor_, "cutoff_frequency", 0.0);
        return {};
    }

    [[nodiscard]] Vec3 reactionForce(const SignalBus& bus) const {
        const std::span<const double> r = bus.readN(reaction_);
        return {r[0], r[1], r[2]};
    }
    [[nodiscard]] Vec3 reactionMoment(const SignalBus& bus) const {
        const std::span<const double> r = bus.readN(reaction_);
        return {r[3], r[4], r[5]};
    }

    struct Axis {
        double range{kInf}, noise{0.0}, resolution{0.0}, offset{0.0};
    };

    /// One reading of one axis; `saturated` set when it is out of range.
    double measure(const Axis& a, double truth, std::uint32_t stream, const StepContext& context,
                   double& filtered, bool& saturated) const {
        if (groundTruth_) {
            filtered = truth;
            return truth;
        }
        double v = truth + a.offset;
        if (a.noise > 0.0) {
            random::Stream rng(context.seed, context.stepIndex, stream);
            v += a.noise * rng.normal();
        }
        // First-order low-pass (the sensor's own filter), exact for a step held over dt.
        if (cutoff_ > 0.0) {
            const double alpha =
                1.0 - std::exp(-2.0 * std::numbers::pi * cutoff_ * context.dt.seconds());
            filtered += alpha * (v - filtered);
            v = filtered;
        } else {
            filtered = v;
        }
        if (std::abs(v) > a.range) {
            saturated = true;
            v = std::copysign(a.range, v);
        }
        if (a.resolution > 0.0) {
            v = a.resolution * std::round(v / a.resolution);
        }
        return v;
    }

    BodyPoses poses_;
    std::uint32_t joint_{KindScene::kNone};
    std::uint32_t child_{0};
    SignalId reaction_;
    double cutoff_{0.0};
};

class ForceTorque final : public JointLoadSensor {
public:
    using JointLoadSensor::JointLoadSensor;

    Status declare(const WorldView& world, const ModuleConfig&, SignalDeclarer& d) override {
        if (Status s = declareJoint(world, d); !s.ok()) {
            return s;
        }
        const World& raw = world.raw();
        const std::string preset = kindText(raw, sensor_, "preset");
        const bool mini45 = preset == "mini45_si_580_20";
        if (!preset.empty() && !mini45) {
            return Status::error(StatusCode::InvalidConfig, "force_torque " + name_ +
                                                                ": unknown preset '" + preset +
                                                                "' (known: mini45_si_580_20)");
        }
        const std::string direction =
            kindText(raw, sensor_, "measure_direction", "parent_to_child");
        if (direction != "parent_to_child" && direction != "child_to_parent") {
            return Status::error(StatusCode::InvalidConfig,
                                 "force_torque " + name_ +
                                     ": measure_direction is parent_to_child or child_to_parent");
        }
        sign_ = direction == "child_to_parent" ? -1.0 : 1.0;
        static constexpr std::array<const char*, 6> kAxes{"fx", "fy", "fz", "tx", "ty", "tz"};
        for (std::size_t k = 0; k < 6; ++k) {
            const bool force = k < 3;
            const std::string group = force ? "force" : "torque";
            Axis& a = axes_[k];
            const double range = mini45 ? kMini45Range[k] : kInf;
            const double resolution = mini45 ? kMini45Resolution[k] : 0.0;
            // Per axis ("fz_range") over per kind ("force_range") over the preset.
            const auto value = [&](const char* what, double fallback) {
                return kindSetting(raw, sensor_, std::string(kAxes[k]) + "_" + what,
                                   kindSetting(raw, sensor_, group + "_" + what, fallback));
            };
            a.range = value("range", range);
            a.resolution = value("resolution", resolution);
            a.noise = value("noise", mini45 ? resolution : 0.0);
            a.offset = value("offset", 0.0);
        }
        const std::string frame = "sensor:" + name_;
        wrench_ = d.output(kindSpec(base() + "/wrench", SignalType::Wrench, units::N, frame));
        force_ = d.output(kindSpec(base() + "/force", SignalType::Vec3, units::N, frame));
        torque_ = d.output(kindSpec(base() + "/torque", SignalType::Vec3, units::N_m, frame));
        saturated_ = d.output(kindSpec(base() + "/saturated", SignalType::Bool, units::one));
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        filtered_.fill(0.0);
        out_.fill(0.0);
        write(bus, false);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        const Pose sensor = compose(poses_.at(bus, body_), mount_);
        const Vec3 at = poses_.at(bus, child_).position;  // where the reaction acts
        const Vec3 f = reactionForce(bus);
        Vec3 m = reactionMoment(bus);
        // The same wrench about the sensor's origin: τ_s = τ_j + (p_j − p_s) × F.
        const Vec3 arm{at[0] - sensor.position[0], at[1] - sensor.position[1],
                       at[2] - sensor.position[2]};
        const Vec3 extra = cross(arm, f);
        for (std::size_t k = 0; k < 3; ++k) {
            m[k] += extra[k];
        }
        const std::array<double, 4> inv = conjugate(sensor.orientation);
        const Vec3 fs = rotate(inv, f);
        const Vec3 ms = rotate(inv, m);
        bool saturated = false;
        for (std::size_t k = 0; k < 6; ++k) {
            const double truth = sign_ * (k < 3 ? fs[k] : ms[k - 3]);
            out_[k] = measure(axes_[k], truth, static_cast<std::uint32_t>(k), context, filtered_[k],
                              saturated);
        }
        write(bus, saturated);
        return {};
    }

    Status saveState(StateWriter& w) const override {
        w.putDoubles(filtered_);
        return {};
    }
    Status restoreState(StateReader& r) override {
        r.getDoubles(filtered_);
        return r.failed() ? Status::error(StatusCode::InvalidConfig, "short state") : Status{};
    }

private:
    void write(SignalBus& bus, bool saturated) {
        bus.writeN(wrench_, out_);
        bus.writeN(force_, std::span<const double>(out_.data(), 3));
        bus.writeN(torque_, std::span<const double>(out_.data() + 3, 3));
        bus.write(saturated_, saturated ? 1.0 : 0.0);
    }

    std::array<Axis, 6> axes_{};
    std::array<double, 6> filtered_{}, out_{};
    double sign_{1.0};
    SignalId wrench_, force_, torque_, saturated_;
};

class JointTorque final : public JointLoadSensor {
public:
    using JointLoadSensor::JointLoadSensor;

    Status declare(const WorldView& world, const ModuleConfig&, SignalDeclarer& d) override {
        if (Status s = declareJoint(world, d); !s.ok()) {
            return s;
        }
        const World& raw = world.raw();
        axisInChild_ = raw.joints.axis[joint_];
        slide_ = raw.joints.type[joint_] == JointType::Prismatic;
        axis_.range = kindSetting(raw, sensor_, "range", kInf);
        axis_.noise = kindSetting(raw, sensor_, "noise", 0.0);
        axis_.resolution = kindSetting(raw, sensor_, "resolution", 0.0);
        axis_.offset = kindSetting(raw, sensor_, "offset", 0.0);
        out_ = d.output(kindSpec(base() + (slide_ ? "/force" : "/torque"), SignalType::Scalar,
                                 slide_ ? units::N : units::N_m));
        saturated_ = d.output(kindSpec(base() + "/saturated", SignalType::Bool, units::one));
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        filtered_ = 0.0;
        bus.write(out_, 0.0);
        bus.write(saturated_, 0.0);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        // The axis is in the child's frame (World::addJoint); the reaction acts at its origin.
        const Vec3 a = rotate(poses_.at(bus, child_).orientation, axisInChild_);
        const Vec3 load = slide_ ? reactionForce(bus) : reactionMoment(bus);
        const double truth = a[0] * load[0] + a[1] * load[1] + a[2] * load[2];
        bool saturated = false;
        bus.write(out_, measure(axis_, truth, 0, context, filtered_, saturated));
        bus.write(saturated_, saturated ? 1.0 : 0.0);
        return {};
    }

    Status saveState(StateWriter& w) const override {
        w.put(filtered_);
        return {};
    }
    Status restoreState(StateReader& r) override {
        r.get(filtered_);
        return r.failed() ? Status::error(StatusCode::InvalidConfig, "short state") : Status{};
    }

private:
    Axis axis_;
    Vec3 axisInChild_{};
    bool slide_{false};
    double filtered_{0.0};
    SignalId out_, saturated_;
};

}  // namespace

std::unique_ptr<IModule> makeForceTorque(const World& world, std::uint32_t sensor, bool groundTruth,
                                         const KindContext&) {
    return std::make_unique<ForceTorque>(world, sensor, groundTruth);
}

std::unique_ptr<IModule> makeJointTorque(const World& world, std::uint32_t sensor, bool groundTruth,
                                         const KindContext&) {
    return std::make_unique<JointTorque>(world, sensor, groundTruth);
}

}  // namespace lagrange::sensors::kinds
