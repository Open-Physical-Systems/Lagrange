#include "lagrange/server/Robot.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <numbers>
#include <set>
#include <string>
#include <tinyxml2.h>
#include <urdf_parser/urdf_parser.h>
#include <utility>
#include <vector>

namespace lagrange::server {

namespace {

constexpr std::array<float, 4> kDefaultColor{0.70F, 0.70F, 0.72F, 1.0F};
constexpr std::array<float, 4> kPlaceholderColor{0.85F, 0.20F, 0.55F, 1.0F};  // "not loaded"
constexpr std::array<float, 4> kGroundColor{0.004F, 0.008F, 0.018F, 1.0F};    // the mock's floor
constexpr double kGroundHalfSizeM = 25.0;  // the visible ground; the collision plane is infinite
constexpr double kGroundThicknessM = 0.02;
// Moving bodies need positive mass and inertia in MuJoCo; links without an <inertial> get this.
constexpr double kPlaceholderMassKg = 1e-3;
constexpr double kPlaceholderInertia = 1e-6;  // kg·m²

Pose poseOf(const urdf::Pose& pose) {
    return {{pose.position.x, pose.position.y, pose.position.z},
            {pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z}};
}

std::array<double, 4> multiply(const std::array<double, 4>& a, const std::array<double, 4>& b) {
    return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}

/// Rotation matrix (row-major) of a unit quaternion.
std::array<double, 9> matrix(const std::array<double, 4>& q) {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    return {1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y),
            2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
            2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)};
}

/// Inertia tensor given in a frame rotated by q, expressed in the link frame: R I Rᵀ.
std::array<double, 6> rotateInertia(const urdf::Inertial& inertial) {
    const std::array<double, 9> i{inertial.ixx, inertial.ixy, inertial.ixz,
                                  inertial.ixy, inertial.iyy, inertial.iyz,
                                  inertial.ixz, inertial.iyz, inertial.izz};
    const std::array<double, 9> r = matrix(poseOf(inertial.origin).orientation);
    std::array<double, 9> ri{};
    std::array<double, 9> out{};
    for (int a = 0; a < 3; ++a) {
        for (int b = 0; b < 3; ++b) {
            for (int k = 0; k < 3; ++k) {
                ri[a * 3 + b] += r[a * 3 + k] * i[k * 3 + b];
            }
        }
    }
    for (int a = 0; a < 3; ++a) {
        for (int b = 0; b < 3; ++b) {
            for (int k = 0; k < 3; ++k) {
                out[a * 3 + b] += ri[a * 3 + k] * r[b * 3 + k];
            }
        }
    }
    return {out[0], out[4], out[8], out[1], out[2], out[5]};
}

/// Joints listed with a command interface in the <ros2_control> blocks of the file.
std::set<std::string> commandedJoints(const std::string& path) {
    std::set<std::string> joints;
    tinyxml2::XMLDocument document;
    if (document.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
        return joints;
    }
    const tinyxml2::XMLElement* robot = document.FirstChildElement("robot");
    for (const tinyxml2::XMLElement* control =
             robot != nullptr ? robot->FirstChildElement("ros2_control") : nullptr;
         control != nullptr; control = control->NextSiblingElement("ros2_control")) {
        for (const tinyxml2::XMLElement* joint = control->FirstChildElement("joint");
             joint != nullptr; joint = joint->NextSiblingElement("joint")) {
            const char* name = joint->Attribute("name");
            if (name != nullptr && joint->FirstChildElement("command_interface") != nullptr) {
                joints.insert(name);
            }
        }
    }
    return joints;
}

/// Lowest z of a collider in world coordinates, from its shape's support along -z.
double lowestPoint(ShapeType shape, const std::array<double, 3>& size, const Pose& world) {
    const std::array<double, 9> r = matrix(world.orientation);
    const double centre = world.position[2];
    // Third row of R: how much each local axis points along world z.
    const std::array<double, 3> up{r[6], r[7], r[8]};
    switch (shape) {
        case ShapeType::Sphere:
            return centre - size[0];
        case ShapeType::Box:
            return centre - (std::abs(up[0]) * size[0] + std::abs(up[1]) * size[1] +
                             std::abs(up[2]) * size[2]);
        case ShapeType::Cylinder: {
            const double along = std::abs(up[2]);
            return centre -
                   (along * size[1] + size[0] * std::sqrt(std::max(0.0, 1 - along * along)));
        }
        case ShapeType::Capsule:
            return centre - (std::abs(up[2]) * size[1] + size[0]);
        case ShapeType::Plane:
            return centre;
    }
    return centre;
}

class Builder {
public:
    Builder(const urdf::ModelInterface& model, const LoadOptions& options, Robot& robot)
        : model_(model), options_(options), robot_(robot) {}

    void note(std::string text) {
        if (std::find(robot_.notes.begin(), robot_.notes.end(), text) == robot_.notes.end()) {
            robot_.notes.push_back(std::move(text));
        }
    }

    std::uint32_t addPrim(ScenePrim prim) {
        if (prim.parent != kNoIndex) {
            prim.path = robot_.scene.prims[prim.parent].path + "/" + prim.name;
        } else {
            prim.path = "/" + prim.name;
        }
        robot_.scene.prims.push_back(std::move(prim));
        return static_cast<std::uint32_t>(robot_.scene.prims.size() - 1);
    }

    std::uint32_t mesh(const std::string& key, const std::function<TriMesh()>& build) {
        const auto it = meshes_.find(key);
        if (it != meshes_.end()) {
            return it->second;
        }
        robot_.scene.meshes.push_back(build());
        const auto id = static_cast<std::uint32_t>(robot_.scene.meshes.size() - 1);
        meshes_.emplace(key, id);
        return id;
    }

    std::uint32_t material(const std::string& name, const std::array<float, 4>& rgba) {
        for (std::uint32_t i = 0; i < robot_.scene.materials.size(); ++i) {
            if (robot_.scene.materials[i].rgba == rgba) {
                return i;
            }
        }
        robot_.scene.materials.push_back({name.empty() ? "default" : name, rgba});
        return static_cast<std::uint32_t>(robot_.scene.materials.size() - 1);
    }

    /// Adds a link, its joint to the parent, its colliders and visuals, then its children.
    void addLink(const urdf::Link& link, BodyId parentBody, std::uint32_t parentPrim,
                 const Pose& parentWorld, const urdf::Joint* joint) {
        World& world = robot_.world;
        const Pose inParent =
            joint != nullptr ? poseOf(joint->parent_to_joint_origin_transform) : Pose{};
        const Pose worldPose = compose(parentWorld, inParent);

        const bool moving = joint == nullptr ? parentBody == kWorldBody && !fixedBase_
                                             : joint->type != urdf::Joint::FIXED;
        double mass = 0.0;
        std::array<double, 3> com{};
        std::array<double, 6> inertia{};
        if (link.inertial && link.inertial->mass > 0.0) {
            mass = link.inertial->mass;
            const Pose origin = poseOf(link.inertial->origin);
            com = origin.position;
            inertia = rotateInertia(*link.inertial);
        } else if (moving) {
            note("link '" + link.name + "' moves but has no <inertial>; it was given 1 g");
            mass = kPlaceholderMassKg;
            inertia = {kPlaceholderInertia, kPlaceholderInertia, kPlaceholderInertia, 0, 0, 0};
        }
        const BodyId body = world.addBody(link.name, parentBody, inParent, mass, com, inertia);
        robot_.bodyRestPose.push_back(worldPose);

        if (joint != nullptr) {
            addJoint(*joint, body);
        }

        for (const urdf::CollisionSharedPtr& collision : link.collision_array) {
            addCollider(link.name, *collision, body);
        }

        ScenePrim prim;
        prim.name = link.name;
        prim.parent = parentPrim;
        prim.typeName = "Xform";
        prim.body = body.index;
        if (mass > 0.0) {
            prim.apiSchemas.emplace_back("PhysicsRigidBodyAPI");
        }
        if (!link.collision_array.empty()) {
            prim.apiSchemas.emplace_back("PhysicsCollisionAPI");
        }
        if (joint != nullptr && joint->type != urdf::Joint::FIXED) {
            prim.apiSchemas.push_back("joint:" + jointTypeName(joint->type));
        }
        const std::uint32_t linkPrim = addPrim(std::move(prim));
        robot_.bodyPrim.push_back(linkPrim);
        std::size_t index = 0;
        for (const urdf::VisualSharedPtr& visual : link.visual_array) {
            addVisual(link.name, *visual, index++, body, linkPrim);
        }
        for (const urdf::JointSharedPtr& child : link.child_joints) {
            const urdf::LinkConstSharedPtr childLink = model_.getLink(child->child_link_name);
            if (childLink) {
                addLink(*childLink, body, linkPrim, worldPose, child.get());
            }
        }
    }

    void setFixedBase(bool fixed) { fixedBase_ = fixed; }
    void setActuated(std::set<std::string> joints) { actuated_ = std::move(joints); }

private:
    static std::string jointTypeName(int type) {
        switch (type) {
            case urdf::Joint::REVOLUTE:
                return "revolute";
            case urdf::Joint::CONTINUOUS:
                return "continuous";
            case urdf::Joint::PRISMATIC:
                return "prismatic";
            case urdf::Joint::FLOATING:
                return "floating";
            case urdf::Joint::PLANAR:
                return "planar";
            default:
                return "fixed";
        }
    }

    void addJoint(const urdf::Joint& joint, BodyId child) {
        JointType type = JointType::Fixed;
        switch (joint.type) {
            case urdf::Joint::REVOLUTE:
                type = JointType::Revolute;
                break;
            case urdf::Joint::CONTINUOUS:
                type = JointType::Continuous;
                break;
            case urdf::Joint::PRISMATIC:
                type = JointType::Prismatic;
                break;
            case urdf::Joint::FLOATING:
                type = JointType::Free;
                break;
            case urdf::Joint::PLANAR:
                note("joint '" + joint.name + "' is planar, which is not supported; it is fixed");
                return;
            default:
                return;  // fixed: the child is welded to its parent
        }
        const bool actuated = actuated_.contains(joint.name) && type != JointType::Free;
        World& world = robot_.world;
        world.addJoint(joint.name, type, child, {joint.axis.x, joint.axis.y, joint.axis.z},
                       actuated);
        if (joint.limits) {
            if (type == JointType::Revolute || type == JointType::Prismatic) {
                world.joints.lowerLimit.back() = joint.limits->lower;
                world.joints.upperLimit.back() = joint.limits->upper;
            }
            world.joints.effortLimit.back() = joint.limits->effort;
            world.joints.velocityLimit.back() = joint.limits->velocity;
        }
        if (actuated) {
            robot_.actuatedJoints.push_back(joint.name);
        }
    }

    void addCollider(const std::string& link, const urdf::Collision& collision, BodyId body) {
        const urdf::Geometry* geometry = collision.geometry.get();
        if (geometry == nullptr) {
            return;
        }
        const Pose pose = poseOf(collision.origin);
        World& world = robot_.world;
        const double friction = options_.surfaceFriction;
        switch (geometry->type) {
            case urdf::Geometry::BOX: {
                const auto& box = static_cast<const urdf::Box&>(*geometry);
                world.addCollider(body, ShapeType::Box,
                                  {box.dim.x / 2.0, box.dim.y / 2.0, box.dim.z / 2.0}, pose,
                                  friction);
                break;
            }
            case urdf::Geometry::CYLINDER: {
                const auto& cylinder = static_cast<const urdf::Cylinder&>(*geometry);
                world.addCollider(body, ShapeType::Cylinder,
                                  {cylinder.radius, cylinder.length / 2.0, 0.0}, pose, friction);
                break;
            }
            case urdf::Geometry::SPHERE: {
                const auto& sphere = static_cast<const urdf::Sphere&>(*geometry);
                world.addCollider(body, ShapeType::Sphere, {sphere.radius, 0.0, 0.0}, pose,
                                  friction);
                break;
            }
            case urdf::Geometry::CAPSULE: {
                const auto& capsule = static_cast<const urdf::Capsule&>(*geometry);
                world.addCollider(body, ShapeType::Capsule,
                                  {capsule.radius, capsule.length / 2.0, 0.0}, pose, friction);
                break;
            }
            default:
                note("collision meshes are not used yet (link '" + link +
                     "'): convex decomposition comes with the collision bake tool");
                break;
        }
    }

    void addVisual(const std::string& link, const urdf::Visual& visual, std::size_t index,
                   BodyId body, std::uint32_t linkPrim) {
        const urdf::Geometry* geometry = visual.geometry.get();
        if (geometry == nullptr) {
            note("a visual of link '" + link + "' has no geometry");
            return;
        }
        std::array<float, 4> rgba = kDefaultColor;
        std::string materialName = visual.material_name;
        if (visual.material) {
            rgba = {visual.material->color.r, visual.material->color.g, visual.material->color.b,
                    visual.material->color.a};
            if (!visual.material->texture_filename.empty()) {
                note("textures are not loaded; textured materials show their colour");
            }
        }
        std::array<float, 3> scale{1.0F, 1.0F, 1.0F};
        std::uint32_t meshId = kNoIndex;
        switch (geometry->type) {
            case urdf::Geometry::BOX: {
                const auto& box = static_cast<const urdf::Box&>(*geometry);
                const double x = box.dim.x, y = box.dim.y, z = box.dim.z;
                meshId = mesh(
                    "box " + std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z),
                    [=] { return boxMesh(x, y, z); });
                break;
            }
            case urdf::Geometry::CYLINDER: {
                const auto& cylinder = static_cast<const urdf::Cylinder&>(*geometry);
                const double r = cylinder.radius, l = cylinder.length;
                meshId = mesh("cylinder " + std::to_string(r) + " " + std::to_string(l),
                              [=] { return cylinderMesh(r, l); });
                break;
            }
            case urdf::Geometry::SPHERE: {
                const double r = static_cast<const urdf::Sphere&>(*geometry).radius;
                meshId = mesh("sphere " + std::to_string(r), [=] { return sphereMesh(r); });
                break;
            }
            case urdf::Geometry::CAPSULE: {
                const auto& capsule = static_cast<const urdf::Capsule&>(*geometry);
                const double r = capsule.radius, l = capsule.length;
                note("capsule visuals are drawn as cylinders");
                meshId = mesh("cylinder " + std::to_string(r) + " " + std::to_string(l),
                              [=] { return cylinderMesh(r, l); });
                break;
            }
            default:
                note(
                    "visual mesh files are not loaded by lagrange_server yet; a magenta box is "
                    "shown (link '" +
                    link + "')");
                meshId = mesh("placeholder", [] { return boxMesh(0.1, 0.1, 0.1); });
                rgba = kPlaceholderColor;
                materialName = "missing-mesh";
                break;
        }
        ScenePrim prim;
        prim.name = index == 0 ? "visual" : "visual_" + std::to_string(index);
        prim.parent = linkPrim;
        prim.typeName = "Mesh";
        prim.mesh = meshId;
        prim.material = material(materialName, rgba);
        prim.body = body.index;
        prim.offset = poseOf(visual.origin);
        prim.scale = scale;
        addPrim(std::move(prim));
    }

    const urdf::ModelInterface& model_;
    const LoadOptions& options_;
    Robot& robot_;
    std::map<std::string, std::uint32_t> meshes_;
    std::set<std::string> actuated_;
    bool fixedBase_{false};
};

}  // namespace

Pose compose(const Pose& a, const Pose& b) {
    const std::array<double, 3> moved = rotate(a.orientation, b.position);
    return {{a.position[0] + moved[0], a.position[1] + moved[1], a.position[2] + moved[2]},
            multiply(a.orientation, b.orientation)};
}

std::array<double, 3> rotate(const std::array<double, 4>& q, const std::array<double, 3>& v) {
    const std::array<double, 9> r = matrix(q);
    return {r[0] * v[0] + r[1] * v[1] + r[2] * v[2], r[3] * v[0] + r[4] * v[1] + r[5] * v[2],
            r[6] * v[0] + r[7] * v[1] + r[8] * v[2]};
}

Expected<Robot, std::string> loadUrdf(const std::string& path, const LoadOptions& options) {
    const urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(path);
    if (!model) {
        return Unexpected{path + " is not a valid URDF (the parser's message is on the console)"};
    }
    const urdf::LinkConstSharedPtr root = model->getRoot();
    if (!root) {
        return Unexpected{path + " has no root link"};
    }

    Robot robot;
    robot.name = model->getName();
    robot.sourcePath = path;
    Builder builder(*model, options, robot);
    builder.setActuated(commandedJoints(path));

    robot.world.addCollider(kWorldBody, ShapeType::Plane, {0.0, 0.0, 1.0}, Pose{},
                            options.groundFriction);

    ScenePrim top;
    top.name = robot.name.empty() ? "robot" : robot.name;
    top.typeName = "Xform";
    const std::uint32_t robotPrim = builder.addPrim(std::move(top));

    // A root link named "world" is the ROS convention for a robot bolted to the ground.
    const bool fixedBase = root->name == "world";
    builder.setFixedBase(fixedBase);
    if (fixedBase) {
        for (const urdf::JointSharedPtr& joint : root->child_joints) {
            const urdf::LinkConstSharedPtr link = model->getLink(joint->child_link_name);
            if (link) {
                builder.addLink(*link, kWorldBody, robotPrim, Pose{}, joint.get());
            }
        }
    } else {
        builder.addLink(*root, kWorldBody, robotPrim, Pose{}, nullptr);
        robot.world.addJoint(root->name + "_floating", JointType::Free, BodyId{0}, {0.0, 0.0, 1.0},
                             false);
    }
    for (const BodyId child : robot.world.joints.child) {
        robot.jointPrim.push_back(robot.bodyPrim.at(child.index));
    }
    robot.scene.prims[robotPrim].body = robot.world.bodies.names.empty() ? kNoIndex : 0;

    // Static scenery after the robot: viewers take the first root prim as the robot.
    ScenePrim ground;
    ground.name = "ground";
    ground.typeName = "Mesh";
    ground.mesh = builder.mesh("ground", [] {
        return boxMesh(2 * kGroundHalfSizeM, 2 * kGroundHalfSizeM, kGroundThicknessM);
    });
    ground.material = builder.material("floor", kGroundColor);
    ground.offset.position = {0.0, 0.0, -kGroundThicknessM / 2.0};
    builder.addPrim(std::move(ground));

    // Place a free robot with its lowest collision shape just above the ground.
    if (!fixedBase && !robot.world.bodies.names.empty()) {
        double lowest = 0.0;
        bool any = false;
        const World::Colliders& colliders = robot.world.colliders;
        for (std::size_t c = 0; c < colliders.body.size(); ++c) {
            if (colliders.body[c] == kWorldBody) {
                continue;
            }
            const Pose shape =
                compose(robot.bodyRestPose[colliders.body[c].index], colliders.poseInBody[c]);
            const double z = lowestPoint(colliders.shape[c], colliders.size[c], shape);
            lowest = any ? std::min(lowest, z) : z;
            any = true;
        }
        if (!any) {
            robot.notes.emplace_back(
                "the robot has no collision shapes and will fall through the "
                "ground");
        }
        const double lift = (any ? -lowest : 0.0) + options.dropHeightM;
        robot.world.bodies.poseInParent[0].position[2] += lift;
        for (Pose& pose : robot.bodyRestPose) {
            pose.position[2] += lift;
        }
    }
    return robot;
}

}  // namespace lagrange::server
