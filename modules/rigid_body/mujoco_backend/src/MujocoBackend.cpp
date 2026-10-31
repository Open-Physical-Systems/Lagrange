#include "lagrange/rigid_body/MujocoBackend.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

namespace lagrange::rigid_body {

namespace {

// A height field's base: the solid box under its lowest sample (MuJoCo hfield size[3]), and
// the least elevation range given to a flat field (MuJoCo needs a positive one).
constexpr double kHeightFieldBaseM = 1.0;
constexpr double kMinHeightFieldRangeM = 1e-3;

using namespace literals;

constexpr std::uint32_t kStateVersion = 1;

bool isActuatable(JointType type) {
    return type == JointType::Revolute || type == JointType::Continuous ||
           type == JointType::Prismatic;
}

SignalSpec spec(std::string name, SignalType type, Unit unit, std::string frame = {}) {
    SignalSpec out;
    out.name = std::move(name);
    out.type = type;
    out.unit = unit;
    out.frame = std::move(frame);
    return out;
}

SignalSpec flowInput(std::string name, SignalType type, Unit unit, bool optional,
                     std::string frame = {}) {
    SignalSpec out = spec(std::move(name), type, unit, std::move(frame));
    out.hold = Hold::WindowAverage;  // flows: the impulse over the physics step is conserved
    out.optional = optional;
    return out;
}

mjtGeom geomType(ShapeType shape) {
    switch (shape) {
        case ShapeType::Plane:
            return mjGEOM_PLANE;
        case ShapeType::Sphere:
            return mjGEOM_SPHERE;
        case ShapeType::Capsule:
            return mjGEOM_CAPSULE;
        case ShapeType::Cylinder:
            return mjGEOM_CYLINDER;
        case ShapeType::Box:
            return mjGEOM_BOX;
        case ShapeType::ConvexHull:
            return mjGEOM_MESH;  // MuJoCo collides with the convex hull of a mesh's vertices
        case ShapeType::HeightField:
            return mjGEOM_HFIELD;
    }
    return mjGEOM_SPHERE;
}

void setPose(double* position, double* orientation, const Pose& pose) {
    std::copy(pose.position.begin(), pose.position.end(), position);
    std::copy(pose.orientation.begin(), pose.orientation.end(), orientation);
}

}  // namespace

MujocoBackend::MujocoBackend() = default;

MujocoBackend::~MujocoBackend() {
    shutdown();
}

ModuleInfo MujocoBackend::info() const {
    return {"rigid_body", "mujoco", 0, 1, false, true, CapabilityTier::Tier0};
}

Duration MujocoBackend::preferredStep() const {
    return 1_ms;  // rigid_body.md §4
}

Status MujocoBackend::declare(const WorldView& world, const ModuleConfig& config,
                              SignalDeclarer& declarer) {
    if (!config.fidelity().empty() && config.fidelity() != "articulated") {
        return Status::error(StatusCode::InvalidConfig,
                             "the MuJoCo backend supports fidelity \"articulated\", not \"" +
                                 std::string(config.fidelity()) + "\"");
    }
    if (world.batchCount() != 1) {
        return Status::error(StatusCode::Unsupported,
                             "the MuJoCo backend runs one environment; batches need the GPU "
                             "backend (rigid_body.md §6)");
    }
    const World::Joints& joints = world.joints();
    for (std::uint32_t j = 0; j < joints.names.size(); ++j) {
        // Every hinge and slide reports where it is (robot_state_publisher needs the casters
        // too); only actuated ones take a torque and report what a motor needs.
        if (!isActuatable(joints.type[j])) {
            continue;
        }
        const bool actuated = joints.actuated[j];
        const bool prismatic = joints.type[j] == JointType::Prismatic;
        const Unit effort = prismatic ? units::N : units::N_m;
        const std::string base = "joint/" + std::string(world.signalInstanceName(joints.prim[j]));
        JointSignals s;
        s.joint = j;
        s.position = declarer.output(
            spec(base + "/position", SignalType::Scalar, prismatic ? units::m : units::rad));
        s.velocity = declarer.output(spec(base + "/velocity", SignalType::Scalar,
                                          prismatic ? units::m_per_s : units::rad_per_s));
        if (!actuated) {
            joints_.push_back(s);
            continue;
        }
        // Optional: an actuated joint that nothing drives (a robot without a controller) is free.
        s.torque = declarer.input(flowInput(base + "/torque", SignalType::Scalar, effort, true));
        s.loadTorque =
            declarer.input(flowInput(base + "/load_torque", SignalType::Scalar, effort, true));
        s.acceleration = declarer.output(spec(base + "/acceleration", SignalType::Scalar,
                                              prismatic ? units::m_per_s2 : units::rad_per_s2));
        s.effectiveInertia = declarer.output(spec(base + "/effective_inertia", SignalType::Scalar,
                                                  prismatic ? units::kg : units::kg_m2));
        s.constraintTorque =
            declarer.output(spec(base + "/constraint_torque", SignalType::Scalar, effort));
        joints_.push_back(s);
    }
    const std::vector<bool> dynamic = movingBodies(world.raw());
    const World::Bodies& bodies = world.bodies();
    for (std::uint32_t b = 0; b < bodies.names.size(); ++b) {
        if (!dynamic[b]) {
            continue;
        }
        const std::string base = "body/" + std::string(world.signalInstanceName(bodies.prim[b]));
        BodySignals s;
        s.body = b;
        s.externalWrench = declarer.input(flowInput(base + "/external_wrench", SignalType::Wrench,
                                                    units::N, true, "world_local"));
        s.pose = declarer.output(spec(base + "/pose", SignalType::Pose, units::m, "world_local"));
        s.twist = declarer.output(
            spec(base + "/twist", SignalType::Wrench, units::m_per_s, "world_local"));
        s.acceleration = declarer.output(
            spec(base + "/acceleration", SignalType::Wrench, units::m_per_s2, "body"));
        if (b < bodies.kinematic.size() && bodies.kinematic[b]) {
            // Optional: without a command it stays where it is.
            SignalSpec command =
                spec(base + "/pose_command", SignalType::Pose, units::m, "world_local");
            command.optional = true;
            s.poseCommand = declarer.input(command);
        }
        bodies_.push_back(s);
    }
    SignalSpec friction = spec("env/friction_scale", SignalType::Scalar, units::one);
    friction.optional = true;
    friction.defaultValue = 1.0;
    frictionScale_ = declarer.input(friction);
    return {};
}

Status MujocoBackend::init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) {
    mjSpec* spec = mj_makeSpec();
    // The world's angles (joint limits) are radians; MuJoCo's compiler reads degrees by default,
    // as MJCF does.
    spec->compiler.degree = 0;
    const Duration dt = config.dt().value_or(preferredStep());
    spec->option.timestep = dt.seconds();
    const std::array<double, 3> gravity = world.gravityLocal();
    std::copy(gravity.begin(), gravity.end(), spec->option.gravity);
    spec->option.cone = mjCONE_ELLIPTIC;           // spike 0.2-C1: pyramidal slips 1.5° early
    spec->option.integrator = mjINT_IMPLICITFAST;  // stable with joint damping; split stepping

    // Bodies, parents first.
    const World::Bodies& bodies = world.bodies();
    std::vector<mjsBody*> made(bodies.names.size(), nullptr);
    mjsBody* worldBody = mjs_findBody(spec, "world");
    for (std::size_t pass = 0; pass <= bodies.names.size(); ++pass) {
        bool progress = false;
        for (std::size_t b = 0; b < bodies.names.size(); ++b) {
            if (made[b] != nullptr) {
                continue;
            }
            const BodyId parent = bodies.parent[b];
            mjsBody* parentBody = parent == kWorldBody         ? worldBody
                                  : parent.index < made.size() ? made[parent.index]
                                                               : nullptr;
            if (parentBody == nullptr) {
                continue;
            }
            mjsBody* body = mjs_addBody(parentBody, nullptr);
            mjs_setName(body->element, bodies.names[b].c_str());
            setPose(body->pos, body->quat, bodies.poseInParent[b]);
            // Massless bodies (static scenery, frames) keep MuJoCo's defaults: an explicit zero
            // inertia is refused even when the body never moves.
            if (bodies.mass[b] > 0.0) {
                body->mass = bodies.mass[b];
                std::copy(bodies.comLocal[b].begin(), bodies.comLocal[b].end(), body->ipos);
                std::copy(bodies.inertiaLocal[b].begin(), bodies.inertiaLocal[b].end(),
                          body->fullinertia);
                body->explicitinertial = 1;
            }
            if (b < bodies.kinematic.size() && bodies.kinematic[b]) {
                if (parent != kWorldBody) {
                    mj_deleteSpec(spec);
                    return Status::error(
                        StatusCode::InvalidConfig,
                        "kinematic body \"" + bodies.names[b] + "\" must hang from the world");
                }
                body->mocap = 1;  // moved by its pose command
            }
            made[b] = body;
            progress = true;
        }
        if (!progress) {
            break;
        }
    }
    for (std::size_t b = 0; b < made.size(); ++b) {
        if (made[b] == nullptr) {
            mj_deleteSpec(spec);
            return Status::error(
                StatusCode::InvalidConfig,
                "body \"" + bodies.names[b] + "\" has no valid parent chain to the world");
        }
    }

    // Joints.
    const World::Joints& joints = world.joints();
    for (std::size_t j = 0; j < joints.names.size(); ++j) {
        if (joints.type[j] == JointType::Fixed) {
            continue;  // a body without a joint is welded to its parent
        }
        mjsBody* child = made.at(joints.child[j].index);
        mjsJoint* joint = joints.type[j] == JointType::Free ? mjs_addFreeJoint(child)
                                                            : mjs_addJoint(child, nullptr);
        mjs_setName(joint->element, joints.names[j].c_str());
        switch (joints.type[j]) {
            case JointType::Revolute:
            case JointType::Continuous:
                joint->type = mjJNT_HINGE;
                break;
            case JointType::Prismatic:
                joint->type = mjJNT_SLIDE;
                break;
            case JointType::Spherical:
                joint->type = mjJNT_BALL;
                break;
            case JointType::Free:
            case JointType::Fixed:
                break;
        }
        std::copy(joints.axis[j].begin(), joints.axis[j].end(), joint->axis);
        if (j < joints.armature.size()) {
            joint->armature = joints.armature[j];  // a motor's reflected rotor inertia
        }
        joint->damping[0] = joints.damping[j];
        joint->frictionloss = joints.friction[j];
        if (joints.type[j] != JointType::Continuous &&
            joints.lowerLimit[j] < joints.upperLimit[j]) {
            joint->limited = mjLIMITED_TRUE;
            joint->range[0] = joints.lowerLimit[j];
            joint->range[1] = joints.upperLimit[j];
        }
    }

    // A joint that copies another: y = data[0] + data[1] x, a gear both ways.
    for (std::size_t j = 0; j < joints.names.size(); ++j) {
        if (joints.mimic[j] != kNoMimic) {
            mjsEquality* copy = mjs_addEquality(spec, nullptr);
            copy->type = mjEQ_JOINT;
            copy->objtype = mjOBJ_JOINT;
            mjs_setString(copy->name1, joints.names[j].c_str());
            mjs_setString(copy->name2, joints.names[joints.mimic[j]].c_str());
            copy->data[0] = joints.mimicOffset[j];
            copy->data[1] = joints.mimicMultiplier[j];
        }
    }

    // Collision geometry, convex shapes only.
    const World::Colliders& colliders = world.colliders();
    for (std::size_t c = 0; c < colliders.body.size(); ++c) {
        const BodyId owner = colliders.body[c];
        mjsBody* body = owner == kWorldBody ? worldBody : made.at(owner.index);
        mjsGeom* geom = mjs_addGeom(body, nullptr);
        geom->type = geomType(colliders.shape[c]);
        std::copy(colliders.size[c].begin(), colliders.size[c].end(), geom->size);
        if (colliders.shape[c] == ShapeType::ConvexHull) {
            const ConvexHull& hull = world.raw().hulls.at(colliders.hull[c]);
            std::vector<float> vertices;
            vertices.reserve(hull.points.size() * 3);
            for (const auto& p : hull.points) {
                vertices.insert(vertices.end(), {static_cast<float>(p[0]), static_cast<float>(p[1]),
                                                 static_cast<float>(p[2])});
            }
            mjsMesh* mesh = mjs_addMesh(spec, nullptr);
            const std::string name = "hull_" + std::to_string(c);
            mjs_setName(mesh->element, name.c_str());
            mjs_setFloat(mesh->uservert, vertices.data(), static_cast<int>(vertices.size()));
            mjs_setString(geom->meshname, name.c_str());
        }
        setPose(geom->pos, geom->quat, colliders.poseInBody[c]);
        if (colliders.shape[c] == ShapeType::HeightField) {
            // MuJoCo's height field: elevations scaled to [0, size[2]] above the geom's origin,
            // on a base box size[3] deep, rows along y from -y, columns along x from -x.
            const HeightField& field = world.raw().heightfields.at(colliders.hull[c]);
            const auto [lowest, highest] =
                std::minmax_element(field.heights.begin(), field.heights.end());
            const double low = *lowest;
            const double range = std::max(*highest - low, kMinHeightFieldRangeM);
            std::vector<float> elevation;
            elevation.reserve(field.heights.size());
            for (const double h : field.heights) {
                elevation.push_back(static_cast<float>((h - low) / range));
            }
            mjsHField* hfield = mjs_addHField(spec);
            const std::string name = "hfield_" + std::to_string(c);
            mjs_setName(hfield->element, name.c_str());
            hfield->nrow = static_cast<int>(field.rows);
            hfield->ncol = static_cast<int>(field.columns);
            hfield->size[0] = colliders.size[c][0];
            hfield->size[1] = colliders.size[c][1];
            hfield->size[2] = range;
            hfield->size[3] = kHeightFieldBaseM;
            mjs_setFloat(hfield->userdata, elevation.data(), static_cast<int>(elevation.size()));
            mjs_setString(geom->hfieldname, name.c_str());
            // The field's origin is its lowest sample: move the geom down to it.
            const std::array<double, 3> up =
                rotate(colliders.poseInBody[c].orientation, {0.0, 0.0, low});
            for (std::size_t i = 0; i < 3; ++i) {
                geom->pos[i] += up.at(i);
            }
        }
        geom->friction[0] = colliders.friction[c];
        // Geoms carry collision only; mass and inertia come from the body tables.
        geom->mass = 0.0;
        geom->density = 0.0;
    }

    // No self-collision inside one articulated tree, as Gazebo's default (self_collide false):
    // links of one robot often overlap by design (a caster under the chassis), and MuJoCo only
    // filters a body against its direct parent. Separate trees still collide with each other.
    const auto rootOf = [&bodies](std::size_t b) {
        while (bodies.parent[b] != kWorldBody) {
            b = bodies.parent[b].index;
        }
        return b;
    };
    std::vector<std::size_t> withGeometry;
    for (std::size_t b = 0; b < bodies.names.size(); ++b) {
        if (std::find(colliders.body.begin(), colliders.body.end(),
                      BodyId{static_cast<std::uint32_t>(b)}) != colliders.body.end()) {
            withGeometry.push_back(b);
        }
    }
    for (std::size_t i = 0; i < withGeometry.size(); ++i) {
        for (std::size_t k = i + 1; k < withGeometry.size(); ++k) {
            const std::size_t a = withGeometry[i];
            const std::size_t b = withGeometry[k];
            if (rootOf(a) == rootOf(b)) {
                mjsExclude* exclude = mjs_addExclude(spec);
                mjs_setString(exclude->bodyname1, bodies.names[a].c_str());
                mjs_setString(exclude->bodyname2, bodies.names[b].c_str());
            }
        }
    }

    mjModel* model = mj_compile(spec, nullptr);
    if (model == nullptr) {
        const std::string error = mjs_getError(spec);
        mj_deleteSpec(spec);
        return Status::error(StatusCode::InvalidConfig, "MuJoCo refused the world: " + error);
    }
    mj_deleteSpec(spec);
    model_ = model;
    data_ = mj_makeData(model_);

    for (JointSignals& s : joints_) {
        const int id = mj_name2id(model_, mjOBJ_JOINT, joints.names[s.joint].c_str());
        s.qposAddress = model_->jnt_qposadr[id];
        s.dofAddress = model_->jnt_dofadr[id];
    }
    for (BodySignals& s : bodies_) {
        s.mujocoBody = mj_name2id(model_, mjOBJ_BODY, bodies.names[s.body].c_str());
        s.mocap = model_->body_mocapid[s.mujocoBody];
    }
    dt_ = dt.seconds();
    lastKinematicPose_.assign(bodies_.size(), {});
    baseFriction_.resize(static_cast<std::size_t>(model_->ngeom));
    for (int g = 0; g < model_->ngeom; ++g) {
        baseFriction_[static_cast<std::size_t>(g)] = model_->geom_friction[3 * g];
    }

    // Joints that start away from zero (a run rebuilt mid-way keeps its robots' poses).
    for (std::size_t j = 0; j < joints.names.size(); ++j) {
        const double start = j < joints.initialPosition.size() ? joints.initialPosition[j] : 0.0;
        const int id = start != 0.0 ? mj_name2id(model_, mjOBJ_JOINT, joints.names[j].c_str()) : -1;
        if (id >= 0 &&
            (model_->jnt_type[id] == mjJNT_HINGE || model_->jnt_type[id] == mjJNT_SLIDE)) {
            data_->qpos[model_->jnt_qposadr[id]] = start;
        }
    }
    // Kinematics of the initial state, so the first outputs are defined.
    mj_step1(model_, data_);
    writeOutputs(bus);
    return {};
}

Status MujocoBackend::step(const StepContext& /*context*/, SignalBus& bus) {
    // Inputs, held over this step by the scheduler.
    for (const JointSignals& s : joints_) {
        if (s.torque.valid()) {
            data_->qfrc_applied[s.dofAddress] = bus.read(s.torque) + bus.read(s.loadTorque);
        }
    }
    for (const BodySignals& s : bodies_) {
        const std::span<const double> wrench = bus.readN(s.externalWrench);
        std::copy(wrench.begin(), wrench.end(), data_->xfrc_applied + 6 * s.mujocoBody);
        if (s.mocap >= 0 && s.poseCommand.valid()) {
            const std::span<const double> p = bus.readN(s.poseCommand);
            const double norm = p[3] * p[3] + p[4] * p[4] + p[5] * p[5] + p[6] * p[6];
            if (norm > 0.5) {  // an unset (zero) command keeps the body where it is
                std::copy_n(p.begin(), 3, data_->mocap_pos + 3 * s.mocap);
                std::copy_n(p.begin() + 3, 4, data_->mocap_quat + 4 * s.mocap);
            }
        }
    }
    const double scale = bus.read(frictionScale_);
    if (scale != appliedFrictionScale_) {
        for (std::size_t g = 0; g < baseFriction_.size(); ++g) {
            model_->geom_friction[3 * g] = baseFriction_[g] * scale;
        }
        appliedFrictionScale_ = scale;
    }

    // mj_step1 ran at the end of the previous step (or in init); finish this one and prepare
    // the next, so the outputs are the state at the end of the step.
    mj_step2(model_, data_);
    if (data_->warning[mjWARN_BADQACC].number > 0) {
        data_->warning[mjWARN_BADQACC].number = 0;
        return Status::error(StatusCode::Diverged,
                             "MuJoCo found a non-finite acceleration and reset the state; reduce "
                             "the step or check masses and inertias");
    }
    mj_step1(model_, data_);
    mj_rnePostConstraint(model_, data_);  // body accelerations for the IMU
    writeOutputs(bus);
    return {};
}

void MujocoBackend::writeOutputs(SignalBus& bus) const {
    for (const JointSignals& s : joints_) {
        const auto dof = static_cast<std::size_t>(s.dofAddress);
        bus.write(s.position, data_->qpos[s.qposAddress]);
        bus.write(s.velocity, data_->qvel[dof]);
        if (!s.torque.valid()) {
            continue;
        }
        bus.write(s.acceleration, data_->qacc[dof]);
        // Diagonal of the joint-space inertia matrix, armature included (architecture.md §8).
        // MuJoCo 3.12 stores M as sparse lower-triangular rows; the diagonal ends each row
        // (checked against mj_fullM in test_mujoco_validation).
        const int last = model_->M_rowadr[dof] + model_->M_rownnz[dof] - 1;
        bus.write(s.effectiveInertia, data_->M[last]);
        bus.write(s.constraintTorque, data_->qfrc_constraint[dof]);
    }
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
        const BodySignals& s = bodies_[i];
        const int b = s.mujocoBody;
        std::array<double, 7> pose{};
        std::copy_n(data_->xpos + 3 * b, 3, pose.begin());
        std::copy_n(data_->xquat + 4 * b, 4, pose.begin() + 3);
        bus.writeN(s.pose, pose);
        if (s.mocap >= 0) {
            // A kinematic body's twist from its last two poses; no acceleration.
            std::array<double, 7>& last = lastKinematicPose_[i];
            std::array<double, 6> twist{};
            if (last[3] != 0.0 || last[4] != 0.0 || last[5] != 0.0 || last[6] != 0.0) {
                for (std::size_t k = 0; k < 3; ++k) {
                    twist[k] = (pose[k] - last[k]) / dt_;
                }
                // ω = 2·(q·conj(q_last)).vec / dt, small-angle.
                const std::array<double, 4> q{pose[3], pose[4], pose[5], pose[6]};
                const std::array<double, 4> r{last[3], -last[4], -last[5], -last[6]};
                const double w = q[0] * r[0] - q[1] * r[1] - q[2] * r[2] - q[3] * r[3];
                const double sign = w < 0.0 ? -1.0 : 1.0;
                twist[3] =
                    sign * 2.0 * (q[0] * r[1] + q[1] * r[0] + q[2] * r[3] - q[3] * r[2]) / dt_;
                twist[4] =
                    sign * 2.0 * (q[0] * r[2] - q[1] * r[3] + q[2] * r[0] + q[3] * r[1]) / dt_;
                twist[5] =
                    sign * 2.0 * (q[0] * r[3] + q[1] * r[2] - q[2] * r[1] + q[3] * r[0]) / dt_;
            }
            last = pose;
            bus.writeN(s.twist, twist);
            bus.writeN(s.acceleration, std::array<double, 6>{});
            continue;
        }

        // MuJoCo gives (angular, linear); the Wrench layout is (linear, angular).
        std::array<double, 6> motion{};
        mj_objectVelocity(model_, data_, mjOBJ_XBODY, b, motion.data(), 0);
        const std::array<double, 6> twist{motion[3], motion[4], motion[5],
                                          motion[0], motion[1], motion[2]};
        bus.writeN(s.twist, twist);
        mj_objectAcceleration(model_, data_, mjOBJ_XBODY, b, motion.data(), 1);
        const std::array<double, 6> acceleration{motion[3], motion[4], motion[5],
                                                 motion[0], motion[1], motion[2]};
        bus.writeN(s.acceleration, acceleration);
    }
}

void MujocoBackend::shutdown() {
    if (data_ != nullptr) {
        mj_deleteData(data_);
        data_ = nullptr;
    }
    if (model_ != nullptr) {
        mj_deleteModel(model_);
        model_ = nullptr;
    }
}

Status MujocoBackend::saveState(StateWriter& writer) const {
    if (model_ == nullptr) {
        return Status::error(StatusCode::Internal, "saveState before init");
    }
    const int size = mj_stateSize(model_, mjSTATE_INTEGRATION);
    std::vector<double> state(static_cast<std::size_t>(size));
    mj_getState(model_, data_, state.data(), mjSTATE_INTEGRATION);
    writer.put(kStateVersion);
    writer.put(appliedFrictionScale_);
    writer.putDoubles(state);
    return {};
}

void MujocoBackend::shiftOrigin(const std::array<double, 3>& shift) {
    if (model_ == nullptr) {
        return;
    }
    // Free bodies (floating bases) and mocap bodies (kinematic agents) are the only positions
    // in the world frame; everything else is relative to a parent. Fixed geometry was built in
    // the new frame already.
    for (int j = 0; j < model_->njnt; ++j) {
        if (model_->jnt_type[j] == mjJNT_FREE) {
            double* position = data_->qpos + model_->jnt_qposadr[j];
            for (std::size_t i = 0; i < 3; ++i) {
                position[i] -= shift.at(i);
            }
        }
    }
    for (int m = 0; m < model_->nmocap; ++m) {
        for (std::size_t i = 0; i < 3; ++i) {
            data_->mocap_pos[3 * m + i] -= shift.at(i);
        }
    }
    mj_step1(model_, data_);
    mj_rnePostConstraint(model_, data_);
}

Status MujocoBackend::restoreState(StateReader& reader) {
    if (model_ == nullptr) {
        return Status::error(StatusCode::Internal, "restoreState before init");
    }
    std::uint32_t version = 0;
    double scale = 1.0;
    std::vector<double> state(static_cast<std::size_t>(mj_stateSize(model_, mjSTATE_INTEGRATION)));
    if (!reader.get(version) || version != kStateVersion || !reader.get(scale) ||
        !reader.getDoubles(state)) {
        return Status::error(StatusCode::InvalidConfig, "MuJoCo state of another model or version");
    }
    mj_setState(model_, data_, state.data(), mjSTATE_INTEGRATION);
    for (std::size_t g = 0; g < baseFriction_.size(); ++g) {
        model_->geom_friction[3 * g] = baseFriction_[g] * scale;
    }
    appliedFrictionScale_ = scale;
    mj_step1(model_, data_);
    mj_rnePostConstraint(model_, data_);
    return {};
}

}  // namespace lagrange::rigid_body
