// clang-format off
#include <Jolt/Jolt.h>  // first: Jolt's headers expect it
// clang-format on

#include "lagrange/rigid_body/JoltBackend.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
#include <numbers>
#include <numeric>
#include <span>
#include <sstream>
#include <string>
#include <vector>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/EmptyShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/PlaneShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/StateRecorderImpl.h>
#include <Jolt/RegisterTypes.h>

namespace lagrange::rigid_body {

using namespace literals;

namespace {

using Vec3 = std::array<double, 3>;
using Mat3 = std::array<std::array<double, 3>, 3>;

// How stiffly a joint follows the one it copies (a Jolt motor's spring, under its 500 Hz limit at
// 1 ms; chosen by the test against the closed form, not measured).
constexpr float kMimicMotorHz = 100.0F;
// Explicit damping d is stable for d dt / I < 2; half of that leaves a margin.
constexpr double kMaxDampingShare = 1.0;

// Share of a step's new armature torque estimate taken (JoltBackend::Impl::updateArmatureTorques).
constexpr double kArmatureRelaxation = 0.5;

// A bound on any body's angular velocity that no robot reaches (rad/s); see its use.
constexpr float kMaxAngularVelocityRadPerS = 1.0e6F;

// Object layers: static scenery, and everything that moves.
constexpr JPH::ObjectLayer kStaticLayer = 0;
constexpr JPH::ObjectLayer kMovingLayer = 1;
constexpr JPH::uint kLayers = 2;

class BroadPhaseLayers final : public JPH::BroadPhaseLayerInterface {
public:
    [[nodiscard]] JPH::uint GetNumBroadPhaseLayers() const override { return kLayers; }
    [[nodiscard]] JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override {
        return JPH::BroadPhaseLayer(static_cast<JPH::BroadPhaseLayer::Type>(layer));
    }
};

class ObjectVsBroadPhase final : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
    [[nodiscard]] bool ShouldCollide(JPH::ObjectLayer layer,
                                     JPH::BroadPhaseLayer broad) const override {
        // Static does not collide with static.
        return layer == kMovingLayer || broad.GetValue() == kMovingLayer;
    }
};

class ObjectPairs final : public JPH::ObjectLayerPairFilter {
public:
    [[nodiscard]] bool ShouldCollide(JPH::ObjectLayer a, JPH::ObjectLayer b) const override {
        return a == kMovingLayer || b == kMovingLayer;
    }
};

/// Jolt's allocator, factory and type registry are process-wide; set them up once.
void initJolt() {
    static std::once_flag once;
    std::call_once(once, [] {
        JPH::RegisterDefaultAllocator();
        JPH::Factory::sInstance = new JPH::Factory();  // NOLINT: Jolt owns it for the process
        JPH::RegisterTypes();
    });
}

JPH::Vec3 jv(const Vec3& v) {
    return {static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2])};
}
JPH::Quat jq(const std::array<double, 4>& q) {  // ours (w, x, y, z), Jolt (x, y, z, w)
    return JPH::Quat(static_cast<float>(q[1]), static_cast<float>(q[2]), static_cast<float>(q[3]),
                     static_cast<float>(q[0]))
        .Normalized();
}
Vec3 dv(JPH::Vec3Arg v) {
    return {v.GetX(), v.GetY(), v.GetZ()};
}
std::array<double, 4> dq(JPH::QuatArg q) {
    return {q.GetW(), q.GetX(), q.GetY(), q.GetZ()};
}

Mat3 rotation(const std::array<double, 4>& q) {
    Mat3 r{};
    for (std::size_t c = 0; c < 3; ++c) {
        Vec3 e{};
        e.at(c) = 1.0;
        const Vec3 col = rotate(q, e);
        for (std::size_t i = 0; i < 3; ++i) {
            r.at(i).at(c) = col.at(i);
        }
    }
    return r;
}

/// Inertia (xx, yy, zz, xy, xz, yz) as a matrix.
Mat3 tensor(const std::array<double, 6>& i) {
    return {{{i[0], i[3], i[4]}, {i[3], i[1], i[5]}, {i[4], i[5], i[2]}}};
}

/// The inertia of a part about the group's centre of mass, in the group frame: rotated into
/// the group frame, then moved by the parallel-axis theorem.
Mat3 moved(const Mat3& local, const Mat3& r, double mass, const Vec3& offset) {
    Mat3 out{};
    for (std::size_t a = 0; a < 3; ++a) {
        for (std::size_t b = 0; b < 3; ++b) {
            double sum = 0.0;
            for (std::size_t k = 0; k < 3; ++k) {
                for (std::size_t l = 0; l < 3; ++l) {
                    sum += r.at(a).at(k) * local.at(k).at(l) * r.at(b).at(l);
                }
            }
            const double d2 = offset[0] * offset[0] + offset[1] * offset[1] + offset[2] * offset[2];
            out.at(a).at(b) = sum + mass * ((a == b ? d2 : 0.0) - offset.at(a) * offset.at(b));
        }
    }
    return out;
}

/// Rotation that turns Jolt's y-axis shapes (capsule, cylinder) to our z axis.
const JPH::Quat kYToZ = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.5F * JPH::JPH_PI);

JPH::ShapeSettings::ShapeResult shapeFor(ShapeType type, const std::array<double, 3>& size,
                                         const ConvexHull* hull,
                                         const HeightField* field = nullptr) {
    const auto f = [](double v) { return static_cast<float>(v); };
    switch (type) {
        case ShapeType::Sphere:
            return JPH::SphereShapeSettings(f(size[0])).Create();
        case ShapeType::Box: {
            const float smallest = f(std::min({size[0], size[1], size[2]}));
            return JPH::BoxShapeSettings(jv(size),
                                         std::min(JPH::cDefaultConvexRadius, 0.1F * smallest))
                .Create();
        }
        case ShapeType::Capsule:
            return JPH::CapsuleShapeSettings(f(size[1]), f(size[0])).Create();
        case ShapeType::Cylinder: {
            const float smallest = f(std::min(size[0], size[1]));
            return JPH::CylinderShapeSettings(f(size[1]), f(size[0]),
                                              std::min(JPH::cDefaultConvexRadius, 0.1F * smallest))
                .Create();
        }
        case ShapeType::Plane:
            return JPH::PlaneShapeSettings(JPH::Plane(JPH::Vec3::sAxisZ(), 0.0F)).Create();
        case ShapeType::ConvexHull: {
            JPH::Array<JPH::Vec3> points;
            points.reserve(hull->points.size());
            for (const auto& p : hull->points) {
                points.push_back(jv(p));
            }
            // A small convex radius: the hull's edges are rounded by it, inside the points.
            const float smallest = f(std::min({size[0], size[1], size[2]}));
            return JPH::ConvexHullShapeSettings(
                       points, std::min(JPH::cDefaultConvexRadius, 0.1F * smallest))
                .Create();
        }
        case ShapeType::HeightField: {
            // Jolt's height field lies in its x z plane with heights along y; shapeRotation
            // turns y up (kYToZ), which takes z to -y, so the rows go in reverse.
            JPH::ShapeSettings::ShapeResult result;
            if (field->columns != field->rows || field->columns < 4) {
                result.SetError("a height field for Jolt needs a square grid of 4 samples or more");
                return result;
            }
            const std::uint32_t n = field->columns;
            std::vector<float> samples(static_cast<std::size_t>(n) * n);
            for (std::uint32_t k = 0; k < n; ++k) {
                for (std::uint32_t x = 0; x < n; ++x) {
                    samples[static_cast<std::size_t>(k) * n + x] =
                        f(field->heights[static_cast<std::size_t>(n - 1 - k) * n + x]);
                }
            }
            const double half = static_cast<double>(n - 1) / 2.0;  // n is unsigned: no -(n - 1)
            const JPH::Vec3 offset(f(-half * field->cellX), 0.0F, f(-half * field->cellY));
            const JPH::Vec3 scale(f(field->cellX), 1.0F, f(field->cellY));
            return JPH::HeightFieldShapeSettings(samples.data(), offset, scale, n).Create();
        }
    }
    return {};
}

const HeightField* fieldOf(const World& world, std::size_t c) {
    return world.colliders.shape[c] == ShapeType::HeightField
               ? &world.heightfields.at(world.colliders.hull[c])
               : nullptr;
}

const ConvexHull* hullOf(const World& world, std::size_t c) {
    return world.colliders.shape[c] == ShapeType::ConvexHull
               ? &world.hulls.at(world.colliders.hull[c])
               : nullptr;
}

JPH::Quat shapeRotation(ShapeType type, const std::array<double, 4>& q) {
    const JPH::Quat r = jq(q);
    return type == ShapeType::Capsule || type == ShapeType::Cylinder ||
                   type == ShapeType::HeightField
               ? r * kYToZ
               : r;
}

SignalSpec spec(std::string name, SignalType type, Unit unit, std::string frame = {}) {
    SignalSpec out;
    out.name = std::move(name);
    out.type = type;
    out.unit = unit;
    out.frame = std::move(frame);
    return out;
}

/// Flows (torques, wrenches) are averaged over the physics step, so the impulse is conserved.
SignalSpec flowInput(std::string name, SignalType type, Unit unit, std::string frame = {}) {
    SignalSpec out = spec(std::move(name), type, unit, std::move(frame));
    out.hold = Hold::WindowAverage;
    out.optional = true;  // an actuated joint that nothing drives is free
    return out;
}

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

double dot(const Vec3& a, const Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

}  // namespace

struct JoltBackend::Impl {
    // Jolt objects, in destruction order (the system before its interfaces).
    BroadPhaseLayers broadPhase;
    ObjectVsBroadPhase objectVsBroadPhase;
    ObjectPairs objectPairs;
    std::unique_ptr<JPH::TempAllocatorImpl> temp;
    std::unique_ptr<JPH::JobSystemSingleThreaded> jobs;
    std::unique_ptr<JPH::PhysicsSystem> system;

    struct Group {  // one Jolt body: a moving body and the bodies fixed to it
        JPH::BodyID id;
        std::uint32_t root{0};
        double mass{0.0};
        Mat3 inertia{};  // about the centre of mass, in the root's frame
        Vec3 com{};      // in the root's frame
        float friction{0.5F};
        Vec3 lastVelocity{};  // of the centre of mass, at the end of the previous step
    };
    std::vector<Group> groups;
    std::vector<std::pair<JPH::BodyID, float>> staticFriction;  // scenery: body, base friction
    double appliedFrictionScale{1.0};
    std::vector<std::int32_t> groupOf;  // per world body; -1 for static ones
    std::vector<Pose> inGroup;          // per world body: its pose in its group root's frame

    struct Joint {
        std::uint32_t joint{0};
        bool prismatic{false};
        JPH::Ref<JPH::Constraint> constraint;
        JPH::BodyID parent;  // invalid for the world
        JPH::BodyID child;
        std::int32_t childGroup{-1};
        std::vector<std::int32_t> subtree;  // groups that move with the joint (child included)
        double angle{0.0};                  // rad, a hinge's position, counting turns
        double damping{0.0};                // N·m·s/rad or N·s/m
        double restInertia{0.0};            // kg·m² or kg moved by it, at rest
        std::int32_t mimic{-1};             // in joints: the one this one copies
        double mimicMultiplier{1.0}, mimicOffset{0.0};
        double armature{0.0};  // kg·m² or kg: the rotor inertia the joint drives
        SignalId position, velocity, torque, loadTorque;
        SignalId acceleration, effectiveInertia, constraintTorque;
        double lastVelocity{0.0};      // at the end of the previous step: for the acceleration
        double lastAcceleration{0.0};  // over the previous step
        double armatureTorque{0.0};    // applied over the previous step
        double applied{0.0};           // N·m or N: the effort put on the joint this step
        std::uint32_t childBody{0};    // world body
        std::int32_t parentGroup{-1};  // -1: the world or static scenery
        SignalId reaction;             // what the parent does to the child, at the child
    };
    std::vector<Joint> joints;

    struct Body {
        std::uint32_t body{0};
        SignalId pose, twist, acceleration, externalWrench;
        SignalId poseCommand;               // kinematic bodies only
        std::array<double, 6> lastTwist{};  // world, at the body origin: for the acceleration
        SignalId contactForce;
    };
    std::vector<Body> bodies;
    SignalId frictionScale;
    Vec3 gravity{};

    /// The joint's axis in world coordinates (on the child).
    [[nodiscard]] JPH::Vec3 axisOf(const Joint& j) const;

    /// The joint's anchor in world coordinates (on the child).
    [[nodiscard]] Vec3 anchorOf(const Joint& j) const;

    /// joint/<j>/reaction and body/<b>/contact_force after a step of `dt` s (see the definition).
    void writeReactionsAndContacts(SignalIo& io, double dt);

    void applyFrictionScale(double scale);

    /// Rotor inertia (armature, reflected through the gear) as joint torques: Jolt has no
    /// joint-space inertia, so the joints' inertia matrix M is built from the bodies (the
    /// composite-rigid-body formula, base held still). Over the last step the joints' other
    /// generalised forces were F = M q'' - tau_a; this step's armature torques are
    /// tau_a = -D (M + D)^-1 F, D the armatures, so that (M + D) q'' = F, one step late, for
    /// the whole chain at once. They go through Jolt's solver like any joint torque, so the
    /// constraints stay consistent. A floating base (an AMR's wheels) makes M approximate.
    void updateArmatureTorques();
    std::vector<double> armatureMatrix, armatureForces;  // its scratch, kept between steps
    bool hasArmature{false};

    /// Joint-space inertia M(a, b): what joint a feels when joint b accelerates (0 unless one's
    /// subtree holds the other's).
    [[nodiscard]] double jointInertia(const Joint& a, const Joint& b) const;

    /// The joint's relative velocity: rad/s about, or m/s along, its world axis `axis`.
    [[nodiscard]] double jointVelocity(const Joint& j, JPH::Vec3Arg axis) const;

    /// The inertia the joint moves with the rest of the tree held still (MuJoCo's joint-space
    /// diagonal): its subtree's inertia about the joint axis, or its mass for a slide, without
    /// the armature. `axis` and `anchor` are in world coordinates.
    [[nodiscard]] double subtreeInertia(const Joint& j, const Vec3& axis, const Vec3& anchor) const;

    [[nodiscard]] Vec3 axisWorld(const Joint& j, const World& world) const {
        const JPH::BodyLockInterfaceNoLock& lock = system->GetBodyLockInterfaceNoLock();
        const JPH::Body* child = lock.TryGetBody(j.child);
        const std::uint32_t c = world.joints.child[j.joint].index;
        const Pose& local = inGroup[c];
        const Vec3 inRoot = rotate(local.orientation, world.joints.axis[j.joint]);
        return rotate(dq(child->GetRotation()), inRoot);
    }
};

double JoltBackend::Impl::jointVelocity(const Joint& j, JPH::Vec3Arg axis) const {
    const JPH::BodyLockInterfaceNoLock& lock = system->GetBodyLockInterfaceNoLock();
    const JPH::Body* child = lock.TryGetBody(j.child);
    const JPH::Body* parent = j.parent.IsInvalid() ? nullptr : lock.TryGetBody(j.parent);
    if (j.prismatic) {
        JPH::Vec3 v = child->GetLinearVelocity();
        if (parent != nullptr) {
            v -= parent->GetLinearVelocity();
        }
        return v.Dot(axis);
    }
    JPH::Vec3 w = child->GetAngularVelocity();
    if (parent != nullptr) {
        w -= parent->GetAngularVelocity();
    }
    return w.Dot(axis);
}

JPH::Vec3 JoltBackend::Impl::axisOf(const Joint& j) const {
    const JPH::Body* child = system->GetBodyLockInterfaceNoLock().TryGetBody(j.child);
    if (j.prismatic) {
        const auto& slider = static_cast<const JPH::SliderConstraint&>(*j.constraint);
        return (child->GetRotation() * slider.GetConstraintToBody2Matrix().GetAxisX()).Normalized();
    }
    const auto& hinge = static_cast<const JPH::HingeConstraint&>(*j.constraint);
    return (child->GetRotation() * hinge.GetLocalSpaceHingeAxis2()).Normalized();
}

Vec3 JoltBackend::Impl::anchorOf(const Joint& j) const {
    const JPH::Body* child = system->GetBodyLockInterfaceNoLock().TryGetBody(j.child);
    const auto& constraint = static_cast<const JPH::TwoBodyConstraint&>(*j.constraint);
    return dv(JPH::Vec3(child->GetCenterOfMassTransform() *
                        constraint.GetConstraintToBody2Matrix().GetTranslation()));
}

void JoltBackend::Impl::applyFrictionScale(double scale) {
    JPH::BodyInterface& bi = system->GetBodyInterfaceNoLock();
    for (const Group& group : groups) {
        bi.SetFriction(group.id, static_cast<float>(group.friction * scale));
    }
    for (const auto& [id, friction] : staticFriction) {
        bi.SetFriction(id, static_cast<float>(friction * scale));
    }
    appliedFrictionScale = scale;
}

double JoltBackend::Impl::jointInertia(const Joint& a, const Joint& b) const {
    // b must move inside a's subtree (or the reverse): a's force is the rate of the momentum
    // b's motion gives b's subtree, taken about a's axis.
    const bool bInA =
        std::find(a.subtree.begin(), a.subtree.end(), b.childGroup) != a.subtree.end();
    const bool aInB =
        std::find(b.subtree.begin(), b.subtree.end(), a.childGroup) != b.subtree.end();
    if (!bInA && !aInB) {
        return 0.0;
    }
    const Joint& outer = bInA ? a : b;  // nearer the root
    const Joint& inner = bInA ? b : a;
    const JPH::BodyLockInterfaceNoLock& lock = system->GetBodyLockInterfaceNoLock();
    const Vec3 axisO = dv(axisOf(outer));
    const Vec3 axisI = dv(axisOf(inner));
    const Vec3 anchorO = anchorOf(outer);
    const Vec3 anchorI = anchorOf(inner);
    double sum = 0.0;
    for (const std::int32_t g : inner.subtree) {
        const Group& group = groups.at(static_cast<std::size_t>(g));
        const JPH::Body* body = lock.TryGetBody(group.id);
        const Vec3 com = dv(JPH::Vec3(body->GetCenterOfMassPosition()));
        // The group's motion for a unit rate of the inner joint.
        Vec3 omega{};
        Vec3 velocity = axisI;
        if (!inner.prismatic) {
            omega = axisI;
            velocity =
                cross(axisI, {com[0] - anchorI[0], com[1] - anchorI[1], com[2] - anchorI[2]});
        }
        const Vec3 momentum{group.mass * velocity[0], group.mass * velocity[1],
                            group.mass * velocity[2]};
        if (outer.prismatic) {
            sum += dot(axisO, momentum);
            continue;
        }
        // Angular momentum about the outer anchor: I omega (world) plus r x p.
        const std::array<double, 4> q = dq(body->GetRotation());
        const Vec3 local = rotate(conjugate(q), omega);
        Vec3 spun{};
        for (std::size_t r = 0; r < 3; ++r) {
            for (std::size_t c = 0; c < 3; ++c) {
                spun.at(r) += group.inertia.at(r).at(c) * local.at(c);
            }
        }
        const Vec3 spin = rotate(q, spun);
        const Vec3 orbit =
            cross({com[0] - anchorO[0], com[1] - anchorO[1], com[2] - anchorO[2]}, momentum);
        sum += dot(axisO, {spin[0] + orbit[0], spin[1] + orbit[1], spin[2] + orbit[2]});
    }
    return sum;
}

void JoltBackend::Impl::updateArmatureTorques() {
    const std::size_t n = joints.size();
    // A = M + D and the right-hand side D-weighted F, solved by Gaussian elimination with
    // partial pivoting (a few joints per robot).
    std::vector<double>& a = armatureMatrix;
    std::vector<double>& f = armatureForces;
    a.assign(n * n, 0.0);
    f.assign(n, 0.0);
    for (std::size_t r = 0; r < n; ++r) {
        for (std::size_t c = r; c < n; ++c) {
            const double m = jointInertia(joints[r], joints[c]);
            a[r * n + c] = m;
            a[c * n + r] = m;
        }
    }
    for (std::size_t r = 0; r < n; ++r) {
        double other = -joints[r].armatureTorque;
        for (std::size_t c = 0; c < n; ++c) {
            other += a[r * n + c] * joints[c].lastAcceleration;
        }
        f[r] = other;
        a[r * n + r] += joints[r].armature;
        if (a[r * n + r] <= 0.0) {
            a[r * n + r] = 1.0;  // a joint that moves nothing: decoupled, no armature torque
        }
    }
    for (std::size_t k = 0; k < n; ++k) {
        std::size_t pivot = k;
        for (std::size_t r = k + 1; r < n; ++r) {
            if (std::abs(a[r * n + k]) > std::abs(a[pivot * n + k])) {
                pivot = r;
            }
        }
        if (pivot != k) {
            for (std::size_t c = 0; c < n; ++c) {
                std::swap(a[k * n + c], a[pivot * n + c]);
            }
            std::swap(f[k], f[pivot]);
        }
        for (std::size_t r = k + 1; r < n; ++r) {
            const double factor = a[r * n + k] / a[k * n + k];
            for (std::size_t c = k; c < n; ++c) {
                a[r * n + c] -= factor * a[k * n + c];
            }
            f[r] -= factor * f[k];
        }
    }
    for (std::size_t k = n; k-- > 0;) {
        double value = f[k];
        for (std::size_t c = k + 1; c < n; ++c) {
            value -= a[k * n + c] * f[c];
        }
        f[k] = value / a[k * n + k];  // q'' with the armature
    }
    for (std::size_t r = 0; r < n; ++r) {
        // Half the step towards the new value: the one-step-late estimate otherwise rings at
        // half the step rate wherever Jolt's response differs from M (constraint stabilisation).
        joints[r].armatureTorque = kArmatureRelaxation * -joints[r].armature * f[r] +
                                   (1.0 - kArmatureRelaxation) * joints[r].armatureTorque;
    }
}

double JoltBackend::Impl::subtreeInertia(const Joint& j, const Vec3& axis,
                                         const Vec3& anchor) const {
    const JPH::BodyLockInterfaceNoLock& lock = system->GetBodyLockInterfaceNoLock();
    double sum = 0.0;
    for (const std::int32_t g : j.subtree) {
        const Group& group = groups.at(static_cast<std::size_t>(g));
        if (j.prismatic) {
            sum += group.mass;
            continue;
        }
        const JPH::Body* body = lock.TryGetBody(group.id);
        const std::array<double, 4> q = dq(body->GetRotation());
        // The axis in the group's frame, and the centre of mass's distance from the axis line.
        const Vec3 local = rotate(conjugate(q), axis);
        double about = 0.0;
        for (std::size_t a = 0; a < 3; ++a) {
            for (std::size_t b = 0; b < 3; ++b) {
                about += local.at(a) * group.inertia.at(a).at(b) * local.at(b);
            }
        }
        const Vec3 com = dv(JPH::Vec3(body->GetCenterOfMassPosition()));
        const Vec3 r{com[0] - anchor[0], com[1] - anchor[1], com[2] - anchor[2]};
        const Vec3 off = cross(axis, r);
        sum += about + group.mass * dot(off, off);
    }
    return sum;
}

JoltBackend::JoltBackend() : impl_(std::make_unique<Impl>()) {}
JoltBackend::~JoltBackend() = default;

ModuleInfo JoltBackend::info() const {
    return {"rigid_body", "jolt", 0, 1, false, true, CapabilityTier::Tier0};
}

Duration JoltBackend::preferredStep() const {
    return 1_ms;
}

namespace {

/// The scheduler's declarer and bus as the engine's SignalIo, inside the server.
class DeclarerIo final : public JoltBackend::SignalIo {
public:
    explicit DeclarerIo(SignalDeclarer& declarer) : declarer_(declarer) {}
    SignalId input(SignalSpec spec) override { return declarer_.input(std::move(spec)); }
    SignalId output(SignalSpec spec) override { return declarer_.output(std::move(spec)); }
    [[nodiscard]] double read(SignalId) const override { return 0.0; }
    [[nodiscard]] std::span<const double> readN(SignalId) const override { return {}; }
    void write(SignalId, double) override {}
    void writeN(SignalId, std::span<const double>) override {}

private:
    SignalDeclarer& declarer_;
};

class BusIo final : public JoltBackend::SignalIo {
public:
    explicit BusIo(SignalBus& bus) : bus_(bus) {}
    SignalId input(SignalSpec) override { return {}; }
    SignalId output(SignalSpec) override { return {}; }
    [[nodiscard]] double read(SignalId id) const override { return bus_.read(id); }
    [[nodiscard]] std::span<const double> readN(SignalId id) const override {
        return bus_.readN(id);
    }
    void write(SignalId id, double value) override { bus_.write(id, value); }
    void writeN(SignalId id, std::span<const double> values) override { bus_.writeN(id, values); }

private:
    SignalBus& bus_;
};

}  // namespace

Status JoltBackend::declare(const WorldView& world, const ModuleConfig& config,
                            SignalDeclarer& declarer) {
    DeclarerIo io(declarer);
    return declare(world, config, io);
}

Status JoltBackend::init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) {
    BusIo io(bus);
    return init(world, config, io);
}

Status JoltBackend::step(const StepContext& context, SignalBus& bus) {
    BusIo io(bus);
    return step(context, io);
}

Status JoltBackend::declare(const WorldView& world, const ModuleConfig& config, SignalIo& io) {
    Impl& s = *impl_;
    if (!config.fidelity().empty() && config.fidelity() != "articulated") {
        return Status::error(StatusCode::InvalidConfig,
                             "the Jolt backend supports fidelity \"articulated\", not \"" +
                                 std::string(config.fidelity()) + "\"");
    }
    if (world.batchCount() != 1) {
        return Status::error(StatusCode::Unsupported, "the Jolt backend runs one environment");
    }
    s.joints.clear();
    s.bodies.clear();
    const World::Joints& joints = world.joints();
    for (std::uint32_t j = 0; j < joints.names.size(); ++j) {
        const JointType type = joints.type[j];
        if (type == JointType::Spherical) {
            return Status::error(StatusCode::Unsupported,
                                 "the Jolt backend has no spherical joints yet (" +
                                     joints.names[j] + "); use mujoco");
        }
        if (type != JointType::Revolute && type != JointType::Continuous &&
            type != JointType::Prismatic) {
            continue;
        }
        const bool prismatic = type == JointType::Prismatic;
        const std::string base = "joint/" + std::string(world.signalInstanceName(joints.prim[j]));
        Impl::Joint out;
        out.joint = j;
        out.prismatic = prismatic;
        // A force-torque sensor in the joint, as MujocoBackend's.
        out.reaction =
            io.output(spec(base + "/reaction", SignalType::Wrench, units::N, "world_local"));
        out.position = io.output(
            spec(base + "/position", SignalType::Scalar, prismatic ? units::m : units::rad));
        out.velocity = io.output(spec(base + "/velocity", SignalType::Scalar,
                                      prismatic ? units::m_per_s : units::rad_per_s));
        if (joints.actuated[j]) {
            // As MujocoBackend: what a motor needs from the joint it drives.
            const Unit effort = prismatic ? units::N : units::N_m;
            out.torque = io.input(flowInput(base + "/torque", SignalType::Scalar, effort));
            out.loadTorque = io.input(flowInput(base + "/load_torque", SignalType::Scalar, effort));
            out.acceleration = io.output(spec(base + "/acceleration", SignalType::Scalar,
                                              prismatic ? units::m_per_s2 : units::rad_per_s2));
            out.effectiveInertia = io.output(spec(base + "/effective_inertia", SignalType::Scalar,
                                                  prismatic ? units::kg : units::kg_m2));
            out.constraintTorque =
                io.output(spec(base + "/constraint_torque", SignalType::Scalar, effort));
        }
        s.joints.push_back(out);
    }
    const std::vector<bool> moving = movingBodies(world.raw());
    const World::Bodies& bodies = world.bodies();
    for (std::uint32_t b = 0; b < bodies.names.size(); ++b) {
        if (!moving[b]) {
            continue;
        }
        const std::string base = "body/" + std::string(world.signalInstanceName(bodies.prim[b]));
        Impl::Body out;
        out.body = b;
        out.pose = io.output(spec(base + "/pose", SignalType::Pose, units::m, "world_local"));
        out.twist =
            io.output(spec(base + "/twist", SignalType::Wrench, units::m_per_s, "world_local"));
        out.acceleration =
            io.output(spec(base + "/acceleration", SignalType::Wrench, units::m_per_s2, "body"));
        out.externalWrench = io.input(
            flowInput(base + "/external_wrench", SignalType::Wrench, units::N, "world_local"));
        out.contactForce =
            io.output(spec(base + "/contact_force", SignalType::Vec3, units::N, "world_local"));
        if (b < bodies.kinematic.size() && bodies.kinematic[b]) {
            SignalSpec command =
                spec(base + "/pose_command", SignalType::Pose, units::m, "world_local");
            command.optional = true;  // without a command it stays where it is
            out.poseCommand = io.input(command);
        }
        s.bodies.push_back(out);
    }
    SignalSpec friction = spec("env/friction_scale", SignalType::Scalar, units::one);
    friction.optional = true;
    friction.defaultValue = 1.0;
    s.frictionScale = io.input(friction);
    return {};
}

Status JoltBackend::init(const WorldView& view, const ModuleConfig&, SignalIo& io) {
    initJolt();
    Impl& s = *impl_;
    const World& world = view.raw();
    const World::Bodies& bodies = world.bodies;
    const std::size_t n = bodies.names.size();
    const std::vector<Pose> rest = restPoses(world);
    const std::vector<bool> moving = movingBodies(world);

    // Groups: a moving body whose joint to its parent is not fixed starts one; bodies fixed to
    // it join it. Static bodies stay -1.
    std::vector<bool> jointed(n, false);
    for (std::size_t j = 0; j < world.joints.names.size(); ++j) {
        if (world.joints.type[j] != JointType::Fixed) {
            jointed.at(world.joints.child[j].index) = true;
        }
    }
    s.groupOf.assign(n, -1);
    s.inGroup.assign(n, Pose{});
    s.groups.clear();
    for (bool changed = true; changed;) {
        changed = false;
        for (std::uint32_t b = 0; b < n; ++b) {
            if (!moving[b] || s.groupOf[b] >= 0) {
                continue;
            }
            const BodyId parent = bodies.parent[b];
            if (jointed[b] || parent == kWorldBody) {
                s.groupOf[b] = static_cast<std::int32_t>(s.groups.size());
                s.groups.push_back({JPH::BodyID(), b});
                changed = true;
            } else if (s.groupOf[parent.index] >= 0) {
                s.groupOf[b] = s.groupOf[parent.index];
                s.inGroup[b] = compose(s.inGroup[parent.index], bodies.poseInParent[b]);
                changed = true;
            }
        }
    }

    s.temp = std::make_unique<JPH::TempAllocatorImpl>(16U * 1024U * 1024U);
    s.jobs = std::make_unique<JPH::JobSystemSingleThreaded>(JPH::cMaxPhysicsJobs);
    s.system = std::make_unique<JPH::PhysicsSystem>();
    s.system->Init(4096, 0, 8192, 8192, s.broadPhase, s.objectVsBroadPhase, s.objectPairs);
    s.system->SetGravity(jv(world.gravityLocal));
    // Contact friction is the larger of the two bodies', as MuJoCo combines its geoms' (Jolt's
    // default is the geometric mean).
    s.system->SetCombineFriction(
        [](const JPH::Body& a, const JPH::SubShapeID&, const JPH::Body& b, const JPH::SubShapeID&) {
            return std::max(a.GetFriction(), b.GetFriction());
        });
    s.gravity = world.gravityLocal;
    s.staticFriction.clear();
    s.appliedFrictionScale = 1.0;
    s.hasArmature = false;
    JPH::BodyInterface& bi = s.system->GetBodyInterfaceNoLock();

    // Bodies of one kinematic tree do not collide with each other (as MujocoBackend); separate
    // trees do, even when a world file's loose objects share one robot index.
    const auto treeOf = [&bodies](std::uint32_t b) {
        while (bodies.parent[b] != kWorldBody) {
            b = bodies.parent[b].index;
        }
        return b;
    };
    JPH::Ref<JPH::GroupFilterTable> filter = new JPH::GroupFilterTable(
        static_cast<JPH::uint>(std::max<std::size_t>(s.groups.size(), 1)));
    for (std::size_t a = 0; a < s.groups.size(); ++a) {
        for (std::size_t b = a + 1; b < s.groups.size(); ++b) {
            if (treeOf(s.groups[a].root) == treeOf(s.groups[b].root)) {
                filter->DisableCollision(static_cast<JPH::CollisionGroup::SubGroupID>(a),
                                         static_cast<JPH::CollisionGroup::SubGroupID>(b));
            }
        }
    }

    // Moving groups: compound of their colliders, with the masses combined.
    for (std::size_t g = 0; g < s.groups.size(); ++g) {
        double mass = 0.0;
        Vec3 com{};
        for (std::uint32_t b = 0; b < n; ++b) {
            if (s.groupOf[b] != static_cast<std::int32_t>(g)) {
                continue;
            }
            const Vec3 c = compose(s.inGroup[b], {bodies.comLocal[b], {1, 0, 0, 0}}).position;
            for (std::size_t i = 0; i < 3; ++i) {
                com.at(i) += bodies.mass[b] * c.at(i);
            }
            mass += bodies.mass[b];
        }
        if (mass <= 0.0) {
            return Status::error(StatusCode::InvalidConfig,
                                 "jolt: body " + bodies.names[s.groups[g].root] + " has no mass");
        }
        for (double& v : com) {
            v /= mass;
        }
        Mat3 inertia{};
        for (std::uint32_t b = 0; b < n; ++b) {
            if (s.groupOf[b] != static_cast<std::int32_t>(g)) {
                continue;
            }
            const Pose& p = s.inGroup[b];
            const Vec3 c = compose(p, {bodies.comLocal[b], {1, 0, 0, 0}}).position;
            const Mat3 part = moved(tensor(bodies.inertiaLocal[b]), rotation(p.orientation),
                                    bodies.mass[b], {c[0] - com[0], c[1] - com[1], c[2] - com[2]});
            for (std::size_t i = 0; i < 3; ++i) {
                for (std::size_t k = 0; k < 3; ++k) {
                    inertia.at(i).at(k) += part.at(i).at(k);
                }
            }
        }
        JPH::StaticCompoundShapeSettings compound;
        float friction = 0.0F;
        int parts = 0;
        for (std::size_t c = 0; c < world.colliders.body.size(); ++c) {
            const BodyId owner = world.colliders.body[c];
            if (owner == kWorldBody || s.groupOf[owner.index] != static_cast<std::int32_t>(g)) {
                continue;
            }
            const Pose p = compose(s.inGroup[owner.index], world.colliders.poseInBody[c]);
            const auto shape = shapeFor(world.colliders.shape[c], world.colliders.size[c],
                                        hullOf(world, c), fieldOf(world, c));
            if (shape.HasError()) {
                return Status::error(StatusCode::InvalidConfig,
                                     "jolt: collider of " + bodies.names[owner.index] + ": " +
                                         std::string(shape.GetError().c_str()));
            }
            compound.AddShape(jv(p.position),
                              shapeRotation(world.colliders.shape[c], p.orientation), shape.Get());
            friction = std::max(friction, static_cast<float>(world.colliders.friction[c]));
            ++parts;
        }
        JPH::RefConst<JPH::Shape> shape;
        if (parts == 0) {
            shape = JPH::EmptyShapeSettings(jv(com)).Create().Get();
        } else {
            const auto made = compound.Create();
            if (made.HasError()) {
                return Status::error(StatusCode::InvalidConfig,
                                     "jolt: " + std::string(made.GetError().c_str()));
            }
            // Put the centre of mass where the world says, not where the shapes' volume is.
            const JPH::Vec3 offset = jv(com) - made.Get()->GetCenterOfMass();
            shape = JPH::OffsetCenterOfMassShapeSettings(offset, made.Get()).Create().Get();
        }
        const Pose& at = rest[s.groups[g].root];
        const std::uint32_t root = s.groups[g].root;
        const bool kinematic = root < bodies.kinematic.size() && bodies.kinematic[root];
        JPH::BodyCreationSettings settings(
            shape, JPH::RVec3(jv(at.position)), jq(at.orientation),
            kinematic ? JPH::EMotionType::Kinematic : JPH::EMotionType::Dynamic, kMovingLayer);
        settings.mOverrideMassProperties = JPH::EOverrideMassProperties::MassAndInertiaProvided;
        settings.mMassPropertiesOverride.mMass = static_cast<float>(mass);
        JPH::Mat44 tensorJ = JPH::Mat44::sZero();
        for (std::size_t i = 0; i < 3; ++i) {
            for (std::size_t k = 0; k < 3; ++k) {
                tensorJ(static_cast<JPH::uint>(i), static_cast<JPH::uint>(k)) =
                    static_cast<float>(inertia.at(i).at(k));
            }
        }
        tensorJ(3, 3) = 1.0F;
        settings.mMassPropertiesOverride.mInertia = tensorJ;
        settings.mFriction = friction > 0.0F ? friction : 0.5F;
        s.groups[g].mass = mass;
        s.groups[g].inertia = inertia;
        s.groups[g].com = com;
        s.groups[g].friction = settings.mFriction;
        settings.mCollisionGroup = JPH::CollisionGroup(
            filter, static_cast<JPH::CollisionGroup::GroupID>(bodies.robot[s.groups[g].root]),
            static_cast<JPH::CollisionGroup::SubGroupID>(g));
        settings.mAllowSleeping = false;  // a robot waiting for a command must not doze off
        // Jolt damps every body by default (0.05 /s, a game's value); physics has no such drag.
        settings.mLinearDamping = 0.0F;
        settings.mAngularDamping = 0.0F;
        // Jolt clamps the angular velocity after adding forces and before solving constraints
        // (default 0.25 rev per frame at 60 Hz, a game's value): a joint torque on a link with
        // little inertia about its centre spins it far past that before the hinge takes over,
        // and the clamp would throw most of the torque away.
        settings.mMaxAngularVelocity = kMaxAngularVelocityRadPerS;
        s.groups[g].id = bi.CreateAndAddBody(settings, JPH::EActivation::Activate);
    }

    // Static scenery: one static body per collider of the world or of a body that never moves.
    for (std::size_t c = 0; c < world.colliders.body.size(); ++c) {
        const BodyId owner = world.colliders.body[c];
        if (owner != kWorldBody && moving[owner.index]) {
            continue;
        }
        const Pose p = owner == kWorldBody
                           ? world.colliders.poseInBody[c]
                           : compose(rest[owner.index], world.colliders.poseInBody[c]);
        const auto shape = shapeFor(world.colliders.shape[c], world.colliders.size[c],
                                    hullOf(world, c), fieldOf(world, c));
        if (shape.HasError()) {
            return Status::error(StatusCode::InvalidConfig,
                                 "jolt: " + std::string(shape.GetError().c_str()));
        }
        JPH::BodyCreationSettings settings(shape.Get(), JPH::RVec3(jv(p.position)),
                                           shapeRotation(world.colliders.shape[c], p.orientation),
                                           JPH::EMotionType::Static, kStaticLayer);
        settings.mFriction = static_cast<float>(world.colliders.friction[c]);
        s.staticFriction.emplace_back(bi.CreateAndAddBody(settings, JPH::EActivation::DontActivate),
                                      settings.mFriction);
    }

    // Joints between groups: the anchor is the child's origin, the axis in the child frame.
    for (Impl::Joint& j : s.joints) {
        const std::uint32_t child = world.joints.child[j.joint].index;
        const BodyId parent = world.joints.parent[j.joint];
        j.childGroup = s.groupOf[child];
        j.childBody = child;
        j.parentGroup = parent == kWorldBody ? -1 : s.groupOf[parent.index];
        j.child = s.groups.at(static_cast<std::size_t>(j.childGroup)).id;
        j.armature = world.joints.armature[j.joint];
        j.damping = world.joints.damping[j.joint];
        j.lastVelocity = 0.0;
        j.lastAcceleration = 0.0;
        j.armatureTorque = 0.0;
        j.angle = 0.0;
        s.hasArmature = s.hasArmature || j.armature > 0.0;
        // The groups that move with this joint: those whose root descends from its child.
        j.subtree.clear();
        for (std::size_t g = 0; g < s.groups.size(); ++g) {
            for (BodyId b{s.groups[g].root}; b != kWorldBody; b = bodies.parent[b.index]) {
                if (b.index == child) {
                    j.subtree.push_back(static_cast<std::int32_t>(g));
                    break;
                }
            }
        }
        const bool toWorld = parent == kWorldBody || s.groupOf[parent.index] < 0;
        j.parent = toWorld ? JPH::BodyID()
                           : s.groups.at(static_cast<std::size_t>(s.groupOf[parent.index])).id;
        const Pose& at = rest[child];
        const Vec3 axis = rotate(at.orientation, world.joints.axis[j.joint]);
        const JPH::Vec3 a = jv(axis).Normalized();
        const JPH::Vec3 normal = a.GetNormalizedPerpendicular();
        JPH::Body& childBody = *s.system->GetBodyLockInterfaceNoLock().TryGetBody(j.child);
        JPH::Body& parentBody = toWorld
                                    ? JPH::Body::sFixedToWorld
                                    : *s.system->GetBodyLockInterfaceNoLock().TryGetBody(j.parent);
        const double lower = world.joints.lowerLimit[j.joint];
        const double upper = world.joints.upperLimit[j.joint];
        const bool limited = world.joints.type[j.joint] != JointType::Continuous &&
                             std::isfinite(lower) && std::isfinite(upper) && lower < upper;
        if (j.prismatic) {
            JPH::SliderConstraintSettings slider;
            slider.mSpace = JPH::EConstraintSpace::WorldSpace;
            slider.mPoint1 = slider.mPoint2 = JPH::RVec3(jv(at.position));
            slider.mSliderAxis1 = slider.mSliderAxis2 = a;
            slider.mNormalAxis1 = slider.mNormalAxis2 = normal;
            if (limited) {
                slider.mLimitsMin = static_cast<float>(lower);
                slider.mLimitsMax = static_cast<float>(upper);
            }
            slider.mMaxFrictionForce = static_cast<float>(world.joints.friction[j.joint]);
            slider.mMotorSettings.mSpringSettings.mFrequency = kMimicMotorHz;
            j.constraint = slider.Create(parentBody, childBody);
        } else {
            JPH::HingeConstraintSettings hinge;
            hinge.mSpace = JPH::EConstraintSpace::WorldSpace;
            hinge.mPoint1 = hinge.mPoint2 = JPH::RVec3(jv(at.position));
            hinge.mHingeAxis1 = hinge.mHingeAxis2 = a;
            hinge.mNormalAxis1 = hinge.mNormalAxis2 = normal;
            if (limited) {
                hinge.mLimitsMin = static_cast<float>(lower);
                hinge.mLimitsMax = static_cast<float>(upper);
            }
            hinge.mMaxFrictionTorque = static_cast<float>(world.joints.friction[j.joint]);
            hinge.mMotorSettings.mSpringSettings.mFrequency = kMimicMotorHz;
            j.constraint = hinge.Create(parentBody, childBody);
        }
        s.system->AddConstraint(j.constraint);
    }
    // A joint that copies another: a position motor on its constraint, moved every step.
    for (Impl::Joint& j : s.joints) {
        j.mimic = -1;
        j.mimicMultiplier = world.joints.mimicMultiplier[j.joint];
        j.mimicOffset = world.joints.mimicOffset[j.joint];
        if (world.joints.mimic[j.joint] == kNoMimic) {
            continue;
        }
        for (std::size_t m = 0; m < s.joints.size(); ++m) {
            if (s.joints[m].joint == world.joints.mimic[j.joint]) {
                j.mimic = static_cast<std::int32_t>(m);
            }
        }
        if (j.mimic < 0) {
            continue;
        }
        if (j.prismatic) {
            static_cast<JPH::SliderConstraint&>(*j.constraint)
                .SetMotorState(JPH::EMotorState::Position);
        } else {
            static_cast<JPH::HingeConstraint&>(*j.constraint)
                .SetMotorState(JPH::EMotorState::Position);
        }
    }
    // Joints where a rebuilt run left them (World::Joints::initialPosition). The constraints above
    // measure from the rest poses, so the bodies move only now: deepest joint first, each by its
    // axis at rest, and the joints above then carry the result along.
    std::vector<std::size_t> deepestFirst(s.joints.size());
    std::iota(deepestFirst.begin(), deepestFirst.end(), std::size_t{0});
    const auto depth = [&](std::size_t index) {
        int d = 0;
        const std::uint32_t child = world.joints.child[s.joints[index].joint].index;
        for (BodyId b{child}; b != kWorldBody; b = bodies.parent[b.index]) {
            ++d;
        }
        return d;
    };
    std::stable_sort(deepestFirst.begin(), deepestFirst.end(),
                     [&](std::size_t a, std::size_t b) { return depth(a) > depth(b); });
    JPH::BodyInterface& placed = s.system->GetBodyInterfaceNoLock();
    for (const std::size_t index : deepestFirst) {
        Impl::Joint& j = s.joints[index];
        const double q = j.joint < world.joints.initialPosition.size()
                             ? world.joints.initialPosition[j.joint]
                             : 0.0;
        if (q == 0.0) {
            continue;
        }
        j.angle = q;
        const Pose& at = rest[world.joints.child[j.joint].index];
        const JPH::Vec3 axis = jv(rotate(at.orientation, world.joints.axis[j.joint])).Normalized();
        const JPH::RVec3 anchor(jv(at.position));
        const JPH::Quat turn = JPH::Quat::sRotation(axis, static_cast<float>(q));
        for (const std::int32_t g : j.subtree) {
            const JPH::BodyID id = s.groups.at(static_cast<std::size_t>(g)).id;
            JPH::RVec3 position = placed.GetPosition(id);
            JPH::Quat rotation = placed.GetRotation(id);
            if (j.prismatic) {
                position += static_cast<float>(q) * axis;
            } else {
                position = anchor + turn * JPH::Vec3(position - anchor);
                rotation = (turn * rotation).Normalized();
            }
            placed.SetPositionAndRotation(id, position, rotation, JPH::EActivation::DontActivate);
        }
    }
    s.system->OptimizeBroadPhase();

    // Initial outputs: the rest state, joints as placed.
    for (Impl::Joint& j : s.joints) {
        const std::uint32_t child = world.joints.child[j.joint].index;
        j.restInertia =
            j.armature +
            s.subtreeInertia(j, rotate(rest[child].orientation, world.joints.axis[j.joint]),
                             rest[child].position);
        io.write(j.position, j.angle);
        io.write(j.velocity, 0.0);
        if (j.acceleration.valid()) {
            io.write(j.acceleration, 0.0);
            io.write(j.constraintTorque, 0.0);
            io.write(j.effectiveInertia, j.restInertia);
        }
    }
    for (Impl::Body& b : s.bodies) {
        const JPH::BodyID root = s.groups.at(static_cast<std::size_t>(s.groupOf[b.body])).id;
        const Pose p =
            compose({dv(JPH::Vec3(placed.GetPosition(root))), dq(placed.GetRotation(root))},
                    s.inGroup[b.body]);
        const std::array<double, 7> pose{p.position[0],    p.position[1],    p.position[2],
                                         p.orientation[0], p.orientation[1], p.orientation[2],
                                         p.orientation[3]};
        io.writeN(b.pose, pose);
        io.writeN(b.twist, std::array<double, 6>{});
        b.lastTwist = {};
        // At rest: the proper acceleration holds the body up against gravity.
        const Vec3 up =
            rotate(conjugate(p.orientation), {-s.gravity[0], -s.gravity[1], -s.gravity[2]});
        io.writeN(b.acceleration, std::array<double, 6>{up[0], up[1], up[2], 0.0, 0.0, 0.0});
        io.writeN(b.contactForce, Vec3{});
    }
    for (Impl::Joint& j : s.joints) {
        io.writeN(j.reaction, std::array<double, 6>{});
        j.applied = 0.0;
    }
    for (Impl::Group& g : s.groups) {
        g.lastVelocity = {};
    }
    return {};
}

Status JoltBackend::step(const StepContext& context, SignalIo& io) {
    Impl& s = *impl_;
    JPH::BodyInterface& bi = s.system->GetBodyInterfaceNoLock();
    const JPH::BodyLockInterfaceNoLock& lock = s.system->GetBodyLockInterfaceNoLock();

    // Joint torques: equal and opposite on the two bodies, about the joint axis.
    if (s.hasArmature) {
        s.updateArmatureTorques();
    }
    for (Impl::Joint& j : s.joints) {
        if (j.mimic >= 0) {
            const Impl::Joint& leader = s.joints[static_cast<std::size_t>(j.mimic)];
            const double leaderAt =
                leader.prismatic
                    ? static_cast<JPH::SliderConstraint&>(*leader.constraint).GetCurrentPosition()
                    : leader.angle;
            const double target = j.mimicMultiplier * leaderAt + j.mimicOffset;
            if (j.prismatic) {
                static_cast<JPH::SliderConstraint&>(*j.constraint)
                    .SetTargetPosition(static_cast<float>(target));
            } else {
                static_cast<JPH::HingeConstraint&>(*j.constraint)
                    .SetTargetAngle(
                        static_cast<float>(std::remainder(target, 2.0 * std::numbers::pi)));
            }
        }
        const double rotor = j.armatureTorque;
        // Explicit damping: never more than a step can take without turning the joint round.
        const double damper =
            j.damping > 0.0
                ? -std::min(j.damping, kMaxDampingShare * j.restInertia / context.dt.seconds()) *
                      j.lastVelocity
                : 0.0;
        j.applied = 0.0;
        if (!j.torque.valid() && rotor == 0.0 && damper == 0.0) {
            continue;
        }
        const double tau =
            (j.torque.valid() ? io.read(j.torque) + io.read(j.loadTorque) : 0.0) + rotor + damper;
        j.applied = tau;
        JPH::Vec3 axis;
        if (j.prismatic) {
            axis = static_cast<JPH::SliderConstraint&>(*j.constraint)
                       .GetConstraintToBody2Matrix()
                       .GetAxisX();
            const JPH::Body* child = lock.TryGetBody(j.child);
            axis = (child->GetRotation() * axis).Normalized();
            bi.AddForce(j.child, axis * static_cast<float>(tau));
            if (!j.parent.IsInvalid()) {
                bi.AddForce(j.parent, -axis * static_cast<float>(tau));
            }
        } else {
            const auto& hinge = static_cast<JPH::HingeConstraint&>(*j.constraint);
            const JPH::Body* child = lock.TryGetBody(j.child);
            axis = (child->GetRotation() * hinge.GetLocalSpaceHingeAxis2()).Normalized();
            bi.AddTorque(j.child, axis * static_cast<float>(tau));
            if (!j.parent.IsInvalid()) {
                bi.AddTorque(j.parent, -axis * static_cast<float>(tau));
            }
        }
    }
    // External wrenches, at the centre of mass.
    for (const Impl::Body& b : s.bodies) {
        const std::span<const double> w = io.readN(b.externalWrench);
        if (w.size() == 6 && (w[0] != 0.0 || w[1] != 0.0 || w[2] != 0.0 || w[3] != 0.0 ||
                              w[4] != 0.0 || w[5] != 0.0)) {
            const JPH::BodyID id = s.groups.at(static_cast<std::size_t>(s.groupOf[b.body])).id;
            bi.AddForce(id, jv({w[0], w[1], w[2]}));
            bi.AddTorque(id, jv({w[3], w[4], w[5]}));
        }
    }

    // Kinematic bodies move to their pose command over the step.
    for (const Impl::Body& b : s.bodies) {
        if (!b.poseCommand.valid()) {
            continue;
        }
        const std::span<const double> p = io.readN(b.poseCommand);
        if (p[3] * p[3] + p[4] * p[4] + p[5] * p[5] + p[6] * p[6] < 0.5) {
            continue;  // unset: stay
        }
        const JPH::BodyID id = s.groups.at(static_cast<std::size_t>(s.groupOf[b.body])).id;
        // The body's origin is its group's root frame (a kinematic body hangs from the world).
        bi.MoveKinematic(id, JPH::RVec3(jv({p[0], p[1], p[2]})), jq({p[3], p[4], p[5], p[6]}),
                         static_cast<float>(context.dt.seconds()));
    }

    // env/friction_scale (wet floors, ...): every body's friction, as MuJoCo scales its geoms'.
    if (const double scale = io.read(s.frictionScale); scale != s.appliedFrictionScale) {
        s.applyFrictionScale(scale);
    }

    const double dt = context.dt.seconds();
    const JPH::EPhysicsUpdateError error =
        s.system->Update(static_cast<float>(dt), 1, s.temp.get(), s.jobs.get());
    if (error != JPH::EPhysicsUpdateError::None) {
        return Status::error(StatusCode::Diverged, "jolt: physics update error " +
                                                       std::to_string(static_cast<int>(error)));
    }

    for (Impl::Joint& j : s.joints) {
        const JPH::Vec3 axis = s.axisOf(j);
        double velocity = 0.0;
        double limitLambda = 0.0;
        if (j.prismatic) {
            const auto& slider = static_cast<JPH::SliderConstraint&>(*j.constraint);
            io.write(j.position, slider.GetCurrentPosition());
            limitLambda = slider.GetTotalLambdaPositionLimits();
        } else {
            const auto& hinge = static_cast<JPH::HingeConstraint&>(*j.constraint);
            // Jolt's angle wraps at ±π; a joint's position counts turns, as MuJoCo's does.
            j.angle += std::remainder(static_cast<double>(hinge.GetCurrentAngle()) - j.angle,
                                      2.0 * std::numbers::pi);
            io.write(j.position, j.angle);
            limitLambda = hinge.GetTotalLambdaRotationLimits();
        }
        velocity = s.jointVelocity(j, axis);
        io.write(j.velocity, velocity);
        j.lastAcceleration = (velocity - j.lastVelocity) / dt;
        if (j.acceleration.valid()) {
            // Jolt has no joint-space acceleration: the change of velocity over the step.
            io.write(j.acceleration, j.lastAcceleration);
            // The limit's impulse over the step, on the child along the axis. Contacts are not
            // projected onto the joint (MuJoCo's qfrc_constraint includes them).
            io.write(j.constraintTorque, limitLambda / dt);
            const Vec3 anchor = s.anchorOf(j);
            io.write(j.effectiveInertia, j.armature + s.subtreeInertia(j, dv(axis), anchor));
        }
        j.lastVelocity = velocity;
    }
    for (Impl::Body& b : s.bodies) {
        const JPH::Body* body =
            lock.TryGetBody(s.groups.at(static_cast<std::size_t>(s.groupOf[b.body])).id);
        const Pose root{dv(JPH::Vec3(body->GetPosition())), dq(body->GetRotation())};
        const Pose p = compose(root, s.inGroup[b.body]);
        const std::array<double, 7> pose{p.position[0],    p.position[1],    p.position[2],
                                         p.orientation[0], p.orientation[1], p.orientation[2],
                                         p.orientation[3]};
        io.writeN(b.pose, pose);
        const Vec3 v = dv(JPH::Vec3(body->GetPointVelocity(JPH::RVec3(jv(p.position)))));
        const Vec3 w = dv(body->GetAngularVelocity());
        const std::array<double, 6> twist{v[0], v[1], v[2], w[0], w[1], w[2]};
        io.writeN(b.twist, twist);
        // The proper acceleration at the body origin in the body frame, as an IMU feels it
        // (MuJoCo's cacc): the change of velocity over the step, less gravity.
        const std::array<double, 4> toBody = conjugate(p.orientation);
        const Vec3 linear = rotate(toBody, {(v[0] - b.lastTwist[0]) / dt - s.gravity[0],
                                            (v[1] - b.lastTwist[1]) / dt - s.gravity[1],
                                            (v[2] - b.lastTwist[2]) / dt - s.gravity[2]});
        const Vec3 angular =
            rotate(toBody, {(w[0] - b.lastTwist[3]) / dt, (w[1] - b.lastTwist[4]) / dt,
                            (w[2] - b.lastTwist[5]) / dt});
        io.writeN(b.acceleration, std::array<double, 6>{linear[0], linear[1], linear[2], angular[0],
                                                        angular[1], angular[2]});
        b.lastTwist = twist;
    }
    s.writeReactionsAndContacts(io, dt);
    return {};
}

// Jolt keeps its contact impulses to itself, but its constraints give theirs (GetTotalLambda*:
// the impulse over the step, + on body 2, the child, − on body 1, the parent; the hinge's
// rotation impulse in its basis b2 = a2⊥, c2 = a2 × b2, as HingeRotationConstraintPart builds
// it). So a joint's reaction is its constraint impulses over the step plus the effort applied
// through it, and what touches a group is what is left of its momentum change: m·Δv/Δt − m·g −
// its external forces − the joints' forces on it (Newton's second law).
void JoltBackend::Impl::writeReactionsAndContacts(SignalIo& io, double dt) {
    const JPH::BodyLockInterfaceNoLock& lock = system->GetBodyLockInterfaceNoLock();
    std::vector<Vec3> jointForce(groups.size(), Vec3{});  // on each group, from its joints
    const auto rotationOf = [&](JPH::BodyID id) {
        if (id.IsInvalid()) {
            return JPH::Quat::sIdentity();
        }
        return lock.TryGetBody(id)->GetRotation();
    };
    for (const Joint& j : joints) {
        const JPH::Quat r1 = rotationOf(j.parent);
        const JPH::Quat r2 = rotationOf(j.child);
        JPH::Vec3 force = JPH::Vec3::sZero();
        JPH::Vec3 torque = JPH::Vec3::sZero();
        if (j.prismatic) {
            const auto& slider = static_cast<const JPH::SliderConstraint&>(*j.constraint);
            const JPH::Mat44 frame = slider.GetConstraintToBody1Matrix();
            const JPH::Vec3 axis = (r1 * frame.GetAxisX()).Normalized();
            const JPH::Vec3 n1 = (r1 * frame.GetAxisY()).Normalized();
            const JPH::Vec3 n2 = (r1 * frame.GetAxisZ()).Normalized();
            const JPH::Vector<2> across = slider.GetTotalLambdaPosition();
            force = n1 * across[0] + n2 * across[1] +
                    axis * (slider.GetTotalLambdaPositionLimits() + slider.GetTotalLambdaMotor());
            torque = slider.GetTotalLambdaRotation();
            force = force / static_cast<float>(dt) + axis * static_cast<float>(j.applied);
            torque = torque / static_cast<float>(dt);
        } else {
            const auto& hinge = static_cast<const JPH::HingeConstraint&>(*j.constraint);
            const JPH::Vec3 a1 = (r1 * hinge.GetLocalSpaceHingeAxis1()).Normalized();
            const JPH::Vec3 a2 = (r2 * hinge.GetLocalSpaceHingeAxis2()).Normalized();
            const JPH::Vec3 b2 = a2.GetNormalizedPerpendicular();
            const JPH::Vec3 c2 = a2.Cross(b2);
            const JPH::Vector<2> bend = hinge.GetTotalLambdaRotation();
            force = hinge.GetTotalLambdaPosition() / static_cast<float>(dt);
            torque = (b2.Cross(a1) * bend[0] + c2.Cross(a1) * bend[1] +
                      a1 * (hinge.GetTotalLambdaRotationLimits() + hinge.GetTotalLambdaMotor())) /
                         static_cast<float>(dt) +
                     a2 * static_cast<float>(j.applied);
        }
        // The forces act at the anchor; the torque goes to the child body's origin.
        const Vec3 f = dv(force);
        const Vec3 anchor = anchorOf(j);
        const std::uint32_t childBody = j.childBody;
        const JPH::Body* child = lock.TryGetBody(j.child);
        const Pose root{dv(JPH::Vec3(child->GetPosition())), dq(child->GetRotation())};
        const Vec3 origin = compose(root, inGroup[childBody]).position;
        const Vec3 arm{anchor[0] - origin[0], anchor[1] - origin[1], anchor[2] - origin[2]};
        const Vec3 tau = dv(torque);
        io.writeN(j.reaction,
                  std::array<double, 6>{f[0], f[1], f[2], tau[0] + arm[1] * f[2] - arm[2] * f[1],
                                        tau[1] + arm[2] * f[0] - arm[0] * f[2],
                                        tau[2] + arm[0] * f[1] - arm[1] * f[0]});
        if (j.childGroup >= 0) {
            for (std::size_t k = 0; k < 3; ++k) {
                jointForce[static_cast<std::size_t>(j.childGroup)][k] += f[k];
            }
        }
        if (j.parentGroup >= 0) {
            for (std::size_t k = 0; k < 3; ++k) {
                jointForce[static_cast<std::size_t>(j.parentGroup)][k] -= f[k];
            }
        }
    }
    // The external forces on each group this step.
    std::vector<Vec3> external(groups.size(), Vec3{});
    for (const Body& b : bodies) {
        const std::span<const double> w = io.readN(b.externalWrench);
        if (w.size() == 6) {
            for (std::size_t k = 0; k < 3; ++k) {
                external[static_cast<std::size_t>(groupOf[b.body])][k] += w[k];
            }
        }
    }
    std::vector<Vec3> contact(groups.size(), Vec3{});
    for (std::size_t g = 0; g < groups.size(); ++g) {
        Group& group = groups[g];
        const JPH::Body* body = lock.TryGetBody(group.id);
        const Vec3 v = dv(body->GetLinearVelocity());
        if (body->IsDynamic()) {
            for (std::size_t k = 0; k < 3; ++k) {
                contact[g][k] = group.mass * ((v[k] - group.lastVelocity[k]) / dt - gravity[k]) -
                                external[g][k] - jointForce[g][k];
            }
        }
        group.lastVelocity = v;
    }
    // A group's contacts go to its root body (Jolt cannot tell which of its bodies they touch).
    for (const Body& b : bodies) {
        const auto g = static_cast<std::size_t>(groupOf[b.body]);
        io.writeN(b.contactForce, groups[g].root == b.body ? contact[g] : Vec3{});
    }
}

void JoltBackend::shutdown() {
    Impl& s = *impl_;
    if (s.system) {
        for (const Impl::Joint& j : s.joints) {
            s.system->RemoveConstraint(j.constraint);
        }
    }
    s.joints.clear();
    s.system.reset();
    s.jobs.reset();
    s.temp.reset();
}

// Jolt's own state recorder: positions, velocities, contacts and constraint state.
namespace {

/// The moving bodies and their contacts with each other: static scenery is rebuilt from the
/// world, so a state still fits after streamed tiles changed the scenery (huge-worlds.md §2).
/// Contacts with scenery are left out and found again on the next step.
/// The moving bodies' state; their contacts with each other, and with `ground` also those with
/// the static scenery (whose bodies are the same only while the scenery is).
class MovingBodies final : public JPH::StateRecorderFilter {
public:
    MovingBodies(const JPH::BodyLockInterface& locks, bool ground)
        : locks_(locks), ground_(ground) {}
    [[nodiscard]] bool ShouldSaveBody(const JPH::Body& body) const override {
        return !body.IsStatic();
    }
    [[nodiscard]] bool ShouldSaveContact(const JPH::BodyID& a,
                                         const JPH::BodyID& b) const override {
        return ground_ ? moving(a) || moving(b) : moving(a) && moving(b);
    }

private:
    [[nodiscard]] bool moving(const JPH::BodyID& id) const {
        const JPH::BodyLockRead lock(locks_, id);
        return lock.Succeeded() && !lock.GetBody().IsStatic();
    }
    const JPH::BodyLockInterface& locks_;
    bool ground_;
};

/// The static scenery as Jolt has it: its bodies' ids and positions (FNV-1a). A saved state's
/// contacts with static bodies are restored only into the same scenery.
std::uint64_t sceneryPrint(JPH::PhysicsSystem& system) {
    JPH::BodyIDVector ids;
    system.GetBodies(ids);
    std::uint64_t hash = 0xcbf29ce484222325ULL;
    const auto mix = [&hash](const void* data, std::size_t size) {
        const auto* bytes = static_cast<const unsigned char*>(data);
        for (std::size_t i = 0; i < size; ++i) {
            hash = (hash ^ bytes[i]) * 0x100000001b3ULL;
        }
    };
    const JPH::BodyLockInterface& locks = system.GetBodyLockInterfaceNoLock();
    for (const JPH::BodyID& id : ids) {
        const JPH::BodyLockRead lock(locks, id);
        if (!lock.Succeeded() || !lock.GetBody().IsStatic()) {
            continue;
        }
        const std::uint32_t value = id.GetIndexAndSequenceNumber();
        mix(&value, sizeof value);
        const JPH::RVec3 at = lock.GetBody().GetPosition();
        const std::array<double, 3> p{at.GetX(), at.GetY(), at.GetZ()};
        mix(p.data(), sizeof p);
    }
    return hash;
}

}  // namespace

Status JoltBackend::saveState(StateWriter& writer) const {
    // Twice: with the contacts on the static scenery, restored when the scenery is the same (a
    // checkpoint), and without, for a rebuilt run whose scenery changed (streamed tiles).
    for (const bool ground : {true, false}) {
        JPH::StateRecorderImpl recorder;
        const MovingBodies filter(impl_->system->GetBodyLockInterfaceNoLock(), ground);
        impl_->system->SaveState(recorder, JPH::EStateRecorderState::All, &filter);
        writer.putString(recorder.GetData());
    }
    writer.put(sceneryPrint(*impl_->system));
    // What the finite-difference accelerations and the friction scale need besides.
    writer.put(impl_->appliedFrictionScale);
    for (const Impl::Joint& j : impl_->joints) {
        writer.put(j.lastVelocity);
        writer.put(j.lastAcceleration);
        writer.put(j.armatureTorque);
        writer.put(j.angle);
    }
    for (const Impl::Body& b : impl_->bodies) {
        writer.putDoubles(b.lastTwist);
    }
    return {};
}

void JoltBackend::shiftOrigin(const std::array<double, 3>& shift) {
    // Every body, scenery included: a restored state puts the scenery back where it was in the
    // old frame, so all of it moves by -shift together (exact: whole metres).
    JPH::BodyIDVector ids;
    impl_->system->GetBodies(ids);
    JPH::BodyInterface& bi = impl_->system->GetBodyInterfaceNoLock();
    const JPH::RVec3 offset(jv(shift));
    for (const JPH::BodyID id : ids) {
        bi.SetPosition(id, bi.GetPosition(id) - offset, JPH::EActivation::DontActivate);
    }
}

Status JoltBackend::restoreState(StateReader& reader) {
    std::string withGround;
    std::string bytes;
    std::uint64_t scenery = 0;
    if (!reader.getString(withGround) || !reader.getString(bytes) || !reader.get(scenery)) {
        return Status::error(StatusCode::Internal, "jolt: short state");
    }
    if (scenery == sceneryPrint(*impl_->system)) {
        bytes = std::move(withGround);
    }
    JPH::StateRecorderImpl recorder;
    recorder.WriteBytes(bytes.data(), bytes.size());
    recorder.Rewind();
    if (!impl_->system->RestoreState(recorder)) {
        return Status::error(StatusCode::Internal, "jolt: state does not fit this world");
    }
    double scale = 1.0;
    bool ok = reader.get(scale);
    for (Impl::Joint& j : impl_->joints) {
        ok = ok && reader.get(j.lastVelocity) && reader.get(j.lastAcceleration) &&
             reader.get(j.armatureTorque) && reader.get(j.angle);
    }
    for (Impl::Body& b : impl_->bodies) {
        ok = ok && reader.getDoubles(b.lastTwist);
    }
    if (!ok) {
        return Status::error(StatusCode::Internal, "jolt: short state");
    }
    impl_->applyFrictionScale(scale);
    // The groups' last velocities (for their contact force) are the restored ones.
    for (Impl::Group& g : impl_->groups) {
        const JPH::Body* body = impl_->system->GetBodyLockInterfaceNoLock().TryGetBody(g.id);
        g.lastVelocity = dv(body->GetLinearVelocity());
    }
    // A saved state holds the gravity it ran under; the run's may have changed (setSite).
    impl_->system->SetGravity(jv(impl_->gravity));
    return {};
}

}  // namespace lagrange::rigid_body
