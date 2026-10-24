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
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/EmptyShape.h>
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

JPH::ShapeSettings::ShapeResult shapeFor(ShapeType type, const std::array<double, 3>& size) {
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
    }
    return {};
}

JPH::Quat shapeRotation(ShapeType type, const std::array<double, 4>& q) {
    const JPH::Quat r = jq(q);
    return type == ShapeType::Capsule || type == ShapeType::Cylinder ? r * kYToZ : r;
}

SignalSpec spec(std::string name, SignalType type, Unit unit, std::string frame = {}) {
    SignalSpec out;
    out.name = std::move(name);
    out.type = type;
    out.unit = unit;
    out.frame = std::move(frame);
    return out;
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
    };
    std::vector<Group> groups;
    std::vector<std::int32_t> groupOf;  // per world body; -1 for static ones
    std::vector<Pose> inGroup;          // per world body: its pose in its group root's frame

    struct Joint {
        std::uint32_t joint{0};
        bool prismatic{false};
        JPH::Ref<JPH::Constraint> constraint;
        JPH::BodyID parent;  // invalid for the world
        JPH::BodyID child;
        SignalId position, velocity, torque;
    };
    std::vector<Joint> joints;

    struct Body {
        std::uint32_t body{0};
        SignalId pose, twist, externalWrench;
    };
    std::vector<Body> bodies;

    [[nodiscard]] Vec3 axisWorld(const Joint& j, const World& world) const {
        const JPH::BodyLockInterfaceNoLock& lock = system->GetBodyLockInterfaceNoLock();
        const JPH::Body* child = lock.TryGetBody(j.child);
        const std::uint32_t c = world.joints.child[j.joint].index;
        const Pose& local = inGroup[c];
        const Vec3 inRoot = rotate(local.orientation, world.joints.axis[j.joint]);
        return rotate(dq(child->GetRotation()), inRoot);
    }
};

JoltBackend::JoltBackend() : impl_(std::make_unique<Impl>()) {}
JoltBackend::~JoltBackend() = default;

ModuleInfo JoltBackend::info() const {
    return {"rigid_body", "jolt", 0, 1, false, true, CapabilityTier::Tier0};
}

Duration JoltBackend::preferredStep() const {
    return 1_ms;
}

Status JoltBackend::declare(const WorldView& world, const ModuleConfig&, SignalDeclarer& declarer) {
    Impl& s = *impl_;
    s.joints.clear();
    s.bodies.clear();
    const World::Joints& joints = world.joints();
    for (std::uint32_t j = 0; j < joints.names.size(); ++j) {
        const JointType type = joints.type[j];
        if (type == JointType::Spherical) {
            return Status::error(StatusCode::Unsupported,
                                 "jolt: spherical joint " + joints.names[j] + " (spike)");
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
        out.position = declarer.output(
            spec(base + "/position", SignalType::Scalar, prismatic ? units::m : units::rad));
        out.velocity = declarer.output(spec(base + "/velocity", SignalType::Scalar,
                                            prismatic ? units::m_per_s : units::rad_per_s));
        if (joints.actuated[j]) {
            SignalSpec torque =
                spec(base + "/torque", SignalType::Scalar, prismatic ? units::N : units::N_m);
            torque.hold = Hold::WindowAverage;
            out.torque = declarer.input(torque);
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
        out.pose = declarer.output(spec(base + "/pose", SignalType::Pose, units::m, "world_local"));
        out.twist = declarer.output(
            spec(base + "/twist", SignalType::Wrench, units::m_per_s, "world_local"));
        SignalSpec wrench =
            spec(base + "/external_wrench", SignalType::Wrench, units::N, "world_local");
        wrench.hold = Hold::WindowAverage;
        wrench.optional = true;
        out.externalWrench = declarer.input(wrench);
        s.bodies.push_back(out);
    }
    return {};
}

Status JoltBackend::init(const WorldView& view, const ModuleConfig&, SignalBus& bus) {
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
    JPH::BodyInterface& bi = s.system->GetBodyInterfaceNoLock();

    // Bodies of one robot do not collide with each other (as MujocoBackend).
    JPH::Ref<JPH::GroupFilterTable> filter = new JPH::GroupFilterTable(
        static_cast<JPH::uint>(std::max<std::size_t>(s.groups.size(), 1)));
    for (std::size_t a = 0; a < s.groups.size(); ++a) {
        for (std::size_t b = a + 1; b < s.groups.size(); ++b) {
            if (bodies.robot[s.groups[a].root] == bodies.robot[s.groups[b].root]) {
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
            const auto shape = shapeFor(world.colliders.shape[c], world.colliders.size[c]);
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
        JPH::BodyCreationSettings settings(shape, JPH::RVec3(jv(at.position)), jq(at.orientation),
                                           JPH::EMotionType::Dynamic, kMovingLayer);
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
        settings.mCollisionGroup = JPH::CollisionGroup(
            filter, static_cast<JPH::CollisionGroup::GroupID>(bodies.robot[s.groups[g].root]),
            static_cast<JPH::CollisionGroup::SubGroupID>(g));
        settings.mAllowSleeping = false;  // a robot waiting for a command must not doze off
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
        const auto shape = shapeFor(world.colliders.shape[c], world.colliders.size[c]);
        if (shape.HasError()) {
            return Status::error(StatusCode::InvalidConfig,
                                 "jolt: " + std::string(shape.GetError().c_str()));
        }
        JPH::BodyCreationSettings settings(shape.Get(), JPH::RVec3(jv(p.position)),
                                           shapeRotation(world.colliders.shape[c], p.orientation),
                                           JPH::EMotionType::Static, kStaticLayer);
        settings.mFriction = static_cast<float>(world.colliders.friction[c]);
        bi.CreateAndAddBody(settings, JPH::EActivation::DontActivate);
    }

    // Joints between groups: the anchor is the child's origin, the axis in the child frame.
    for (Impl::Joint& j : s.joints) {
        const std::uint32_t child = world.joints.child[j.joint].index;
        const BodyId parent = world.joints.parent[j.joint];
        j.child = s.groups.at(static_cast<std::size_t>(s.groupOf[child])).id;
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
            j.constraint = hinge.Create(parentBody, childBody);
        }
        s.system->AddConstraint(j.constraint);
    }
    s.system->OptimizeBroadPhase();

    // Initial outputs: the rest state.
    for (const Impl::Joint& j : s.joints) {
        bus.write(j.position, 0.0);
        bus.write(j.velocity, 0.0);
    }
    for (const Impl::Body& b : s.bodies) {
        const Pose& p = rest[b.body];
        const std::array<double, 7> pose{p.position[0],    p.position[1],    p.position[2],
                                         p.orientation[0], p.orientation[1], p.orientation[2],
                                         p.orientation[3]};
        bus.writeN(b.pose, pose);
        bus.writeN(b.twist, std::array<double, 6>{});
    }
    return {};
}

Status JoltBackend::step(const StepContext& context, SignalBus& bus) {
    Impl& s = *impl_;
    JPH::BodyInterface& bi = s.system->GetBodyInterfaceNoLock();
    const JPH::BodyLockInterfaceNoLock& lock = s.system->GetBodyLockInterfaceNoLock();

    // Joint torques: equal and opposite on the two bodies, about the joint axis.
    for (const Impl::Joint& j : s.joints) {
        if (!j.torque.valid()) {
            continue;
        }
        const double tau = bus.read(j.torque);
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
        const std::span<const double> w = bus.readN(b.externalWrench);
        if (w.size() == 6 && (w[0] != 0.0 || w[1] != 0.0 || w[2] != 0.0 || w[3] != 0.0 ||
                              w[4] != 0.0 || w[5] != 0.0)) {
            const JPH::BodyID id = s.groups.at(static_cast<std::size_t>(s.groupOf[b.body])).id;
            bi.AddForce(id, jv({w[0], w[1], w[2]}));
            bi.AddTorque(id, jv({w[3], w[4], w[5]}));
        }
    }

    const JPH::EPhysicsUpdateError error =
        s.system->Update(static_cast<float>(context.dt.seconds()), 1, s.temp.get(), s.jobs.get());
    if (error != JPH::EPhysicsUpdateError::None) {
        return Status::error(StatusCode::Diverged, "jolt: physics update error " +
                                                       std::to_string(static_cast<int>(error)));
    }

    for (const Impl::Joint& j : s.joints) {
        const JPH::Body* child = lock.TryGetBody(j.child);
        const JPH::Body* parent = j.parent.IsInvalid() ? nullptr : lock.TryGetBody(j.parent);
        if (j.prismatic) {
            const auto& slider = static_cast<JPH::SliderConstraint&>(*j.constraint);
            bus.write(j.position, slider.GetCurrentPosition());
            const JPH::Vec3 axis =
                (child->GetRotation() * slider.GetConstraintToBody2Matrix().GetAxisX())
                    .Normalized();
            JPH::Vec3 v = child->GetLinearVelocity();
            if (parent != nullptr) {
                v -= parent->GetLinearVelocity();
            }
            bus.write(j.velocity, v.Dot(axis));
        } else {
            const auto& hinge = static_cast<JPH::HingeConstraint&>(*j.constraint);
            bus.write(j.position, hinge.GetCurrentAngle());
            const JPH::Vec3 axis =
                (child->GetRotation() * hinge.GetLocalSpaceHingeAxis2()).Normalized();
            JPH::Vec3 w = child->GetAngularVelocity();
            if (parent != nullptr) {
                w -= parent->GetAngularVelocity();
            }
            bus.write(j.velocity, w.Dot(axis));
        }
    }
    for (const Impl::Body& b : s.bodies) {
        const JPH::Body* body =
            lock.TryGetBody(s.groups.at(static_cast<std::size_t>(s.groupOf[b.body])).id);
        const Pose root{dv(JPH::Vec3(body->GetPosition())), dq(body->GetRotation())};
        const Pose p = compose(root, s.inGroup[b.body]);
        const std::array<double, 7> pose{p.position[0],    p.position[1],    p.position[2],
                                         p.orientation[0], p.orientation[1], p.orientation[2],
                                         p.orientation[3]};
        bus.writeN(b.pose, pose);
        const Vec3 v = dv(JPH::Vec3(body->GetPointVelocity(JPH::RVec3(jv(p.position)))));
        const Vec3 w = dv(body->GetAngularVelocity());
        bus.writeN(b.twist, std::array<double, 6>{v[0], v[1], v[2], w[0], w[1], w[2]});
    }
    return {};
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
Status JoltBackend::saveState(StateWriter& writer) const {
    JPH::StateRecorderImpl recorder;
    impl_->system->SaveState(recorder);
    const std::string bytes = recorder.GetData();
    writer.putString(bytes);
    return {};
}

Status JoltBackend::restoreState(StateReader& reader) {
    std::string bytes;
    if (!reader.getString(bytes)) {
        return Status::error(StatusCode::Internal, "jolt: short state");
    }
    JPH::StateRecorderImpl recorder;
    recorder.WriteBytes(bytes.data(), bytes.size());
    recorder.Rewind();
    if (!impl_->system->RestoreState(recorder)) {
        return Status::error(StatusCode::Internal, "jolt: state does not fit this world");
    }
    return {};
}

}  // namespace lagrange::rigid_body
