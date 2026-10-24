#include "lagrange/importers/Importers.hpp"
#include "lagrange/meshes/MeshImport.hpp"
#include "lagrange/server/Robot.hpp"
#include "lagrange/stage/Stage.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <functional>
#include <map>
#include <numbers>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <tinyxml2.h>
#include <urdf_parser/urdf_parser.h>
#include <utility>
#include <vector>

namespace lagrange::server {

namespace {

constexpr std::array<float, 4> kDefaultColor{0.70F, 0.70F, 0.72F, 1.0F};
constexpr std::array<float, 4> kPlaceholderColor{0.85F, 0.20F, 0.55F, 1.0F};  // "not loaded"
// Moving bodies need positive mass and inertia in MuJoCo; links without an <inertial> get this.
constexpr double kPlaceholderMassKg = 1e-3;
constexpr double kPlaceholderInertia = 1e-6;  // kg·m²

Pose poseOf(const urdf::Pose& pose) {
    return {{pose.position.x, pose.position.y, pose.position.z},
            {pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z}};
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
    Builder(const urdf::ModelInterface& model, const LoadOptions& options, Robot& robot,
            std::string baseDirectory)
        : model_(model)
        , options_(options)
        , robot_(robot)
        , baseDirectory_(std::move(baseDirectory)) {}

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

    struct MeshFilePart {
        std::uint32_t mesh;
        std::string material;
        std::array<float, 4> rgba;
        bool hasColor;
    };

    /// The parts of a mesh file, imported once per file; empty (and a note) if it cannot be read.
    const std::vector<MeshFilePart>& meshFile(const std::string& uri, const std::string& link) {
        const auto cached = meshFiles_.find(uri);
        if (cached != meshFiles_.end()) {
            return cached->second;
        }
        std::vector<MeshFilePart>& parts = meshFiles_[uri];
        auto path = meshes::resolveMeshUri(uri, baseDirectory_);
        auto imported = path
                            ? meshes::importMesh(*path)
                            : Expected<meshes::ImportedMesh, std::string>(Unexpected{path.error()});
        if (!imported) {
            note(imported.error() + "; a magenta box is shown (link '" + link + "')");
            return parts;
        }
        for (const std::string& text : imported->notes) {
            note(uri + ": " + text);
        }
        for (std::size_t k = 0; k < imported->parts.size(); ++k) {
            meshes::MeshPart& part = imported->parts[k];
            TriMesh triangles;
            triangles.positions = std::move(part.positions);
            triangles.normals = std::move(part.normals);
            triangles.indices = std::move(part.indices);
            robot_.scene.meshes.push_back(std::move(triangles));
            parts.push_back({static_cast<std::uint32_t>(robot_.scene.meshes.size() - 1),
                             part.material, part.rgba, part.hasColor});
        }
        return parts;
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
            case urdf::Geometry::MESH: {
                const auto& file = static_cast<const urdf::Mesh&>(*geometry);
                const std::array<float, 3> fileScale{static_cast<float>(file.scale.x),
                                                     static_cast<float>(file.scale.y),
                                                     static_cast<float>(file.scale.z)};
                const std::vector<MeshFilePart>& parts = meshFile(file.filename, link);
                if (!parts.empty()) {
                    // One prim per material of the file; a URDF material colours them all.
                    for (std::size_t k = 0; k < parts.size(); ++k) {
                        ScenePrim prim;
                        prim.name = index == 0 ? "visual" : "visual_" + std::to_string(index);
                        if (parts.size() > 1) {
                            prim.name += "_" + std::to_string(k);
                        }
                        prim.parent = linkPrim;
                        prim.typeName = "Mesh";
                        prim.mesh = parts[k].mesh;
                        prim.material =
                            visual.material || !parts[k].hasColor
                                ? material(materialName, rgba)
                                : material(
                                      parts[k].material.empty() ? file.filename : parts[k].material,
                                      parts[k].rgba);
                        prim.body = body.index;
                        prim.offset = poseOf(visual.origin);
                        prim.scale = fileScale;
                        addPrim(std::move(prim));
                    }
                    return;
                }
                meshId = mesh("placeholder", [] { return boxMesh(0.1, 0.1, 0.1); });
                rgba = kPlaceholderColor;
                materialName = "missing-mesh";
                break;
            }
            default:
                note("an unknown visual geometry is drawn as a magenta box (link '" + link + "')");
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
    std::map<std::string, std::vector<MeshFilePart>> meshFiles_;  // by URI
    Robot& robot_;
    std::string baseDirectory_;  // of the URDF file: relative mesh paths start here
    std::map<std::string, std::uint32_t> meshes_;
    std::set<std::string> actuated_;
    bool fixedBase_{false};
};

}  // namespace

namespace {

/// Quaternion of a fixed-axis roll, pitch, yaw (R = Rz Ry Rx), as URDF and SDFormat use.
std::array<double, 4> fromRpy(double roll, double pitch, double yaw) {
    const double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
    const double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
    const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
    return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy};
}

double childNumber(const tinyxml2::XMLElement* parent, std::initializer_list<const char*> path,
                   double fallback) {
    const tinyxml2::XMLElement* element = parent;
    for (const char* name : path) {
        element = element != nullptr ? element->FirstChildElement(name) : nullptr;
    }
    double value = fallback;
    if (element != nullptr && element->QueryDoubleText(&value) != tinyxml2::XML_SUCCESS) {
        value = fallback;
    }
    return value;
}

/// The first <stddev> anywhere below `element`, or 0.
double firstStddev(const tinyxml2::XMLElement* element) {
    if (element == nullptr) {
        return 0.0;
    }
    for (const tinyxml2::XMLElement* child = element->FirstChildElement(); child != nullptr;
         child = child->NextSiblingElement()) {
        if (std::string_view(child->Name()) == "stddev") {
            double value = 0.0;
            return child->QueryDoubleText(&value) == tinyxml2::XML_SUCCESS ? value : 0.0;
        }
        if (const double found = firstStddev(child); found > 0.0) {
            return found;
        }
    }
    return 0.0;
}

const tinyxml2::XMLElement* firstNoise(const tinyxml2::XMLElement* element) {
    if (element == nullptr) {
        return nullptr;
    }
    for (const tinyxml2::XMLElement* child = element->FirstChildElement(); child != nullptr;
         child = child->NextSiblingElement()) {
        if (std::string_view(child->Name()) == "noise") {
            return child;
        }
        if (const tinyxml2::XMLElement* found = firstNoise(child); found != nullptr) {
            return found;
        }
    }
    return nullptr;
}

/// The SDFormat <noise> of an IMU channel (the first axis that has one) as the settings of
/// sensors::ImuNoise: "<prefix>_noise", "_bias_mean", ... Zeros are left out.
void imuNoise(const tinyxml2::XMLElement* channel, const std::string& prefix,
              std::map<std::string, double, std::less<>>& settings) {
    const tinyxml2::XMLElement* noise = firstNoise(channel);
    if (noise == nullptr) {
        return;
    }
    static constexpr std::array<std::pair<const char*, const char*>, 6> kFields{{
        {"stddev", "_noise"},
        {"bias_mean", "_bias_mean"},
        {"bias_stddev", "_bias_stddev"},
        {"dynamic_bias_stddev", "_dynamic_bias_stddev"},
        {"dynamic_bias_correlation_time", "_dynamic_bias_correlation_time"},
        {"precision", "_precision"},
    }};
    for (const auto& [element, key] : kFields) {
        double value = 0.0;
        const tinyxml2::XMLElement* field = noise->FirstChildElement(element);
        if (field != nullptr && field->QueryDoubleText(&value) == tinyxml2::XML_SUCCESS &&
            value != 0.0) {
            settings[prefix + key] = value;
        }
    }
}

const char* sensorSchema(SensorType kind) {
    switch (kind) {
        case SensorType::Lidar:
            return "LidarAPI_1";
        case SensorType::Camera:
            return "CameraSensorAPI_1";
        case SensorType::Imu:
            break;
    }
    return "ImuAPI_1";
}

/// Gazebo sensor blocks: <gazebo reference="link"><sensor name type>...</sensor></gazebo>.
void addSensors(Robot& robot, const std::string& path, std::size_t firstBody) {
    tinyxml2::XMLDocument document;
    if (document.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
        return;
    }
    const tinyxml2::XMLElement* top = document.FirstChildElement("robot");
    World& world = robot.world;
    for (const tinyxml2::XMLElement* gazebo = top != nullptr ? top->FirstChildElement("gazebo")
                                                             : nullptr;
         gazebo != nullptr; gazebo = gazebo->NextSiblingElement("gazebo")) {
        const char* reference = gazebo->Attribute("reference");
        for (const tinyxml2::XMLElement* sensor = gazebo->FirstChildElement("sensor");
             sensor != nullptr; sensor = sensor->NextSiblingElement("sensor")) {
            const std::string type =
                sensor->Attribute("type") != nullptr ? sensor->Attribute("type") : "";
            const char* name = sensor->Attribute("name");
            std::size_t body = world.bodies.names.size();
            for (std::size_t b = firstBody; b < world.bodies.names.size(); ++b) {
                if (reference != nullptr && world.bodies.names[b] == reference) {
                    body = b;
                }
            }
            if (name == nullptr || body == world.bodies.names.size()) {
                robot.notes.push_back("a <sensor> of type '" + type +
                                      "' has no name or no known link; it is skipped");
                continue;
            }
            Pose mount;
            if (const tinyxml2::XMLElement* pose = sensor->FirstChildElement("pose");
                pose != nullptr && pose->GetText() != nullptr) {
                std::array<double, 6> v{};
                std::istringstream text(pose->GetText());
                for (double& x : v) {
                    text >> x;
                }
                mount = {{v[0], v[1], v[2]}, fromRpy(v[3], v[4], v[5])};
            }
            const double rate = childNumber(sensor, {"update_rate"}, 10.0);
            std::map<std::string, double, std::less<>> settings;
            SensorType kind = SensorType::Imu;
            std::string typeName;
            if (type == "imu") {
                const tinyxml2::XMLElement* imu = sensor->FirstChildElement("imu");
                imuNoise(imu != nullptr ? imu->FirstChildElement("angular_velocity") : nullptr,
                         "gyro", settings);
                imuNoise(imu != nullptr ? imu->FirstChildElement("linear_acceleration") : nullptr,
                         "accel", settings);
                typeName = "Imu";
            } else if (type == "ray" || type == "gpu_ray" || type == "lidar" ||
                       type == "gpu_lidar") {
                kind = SensorType::Lidar;
                const tinyxml2::XMLElement* ray = sensor->FirstChildElement("ray");
                if (ray == nullptr) {
                    ray = sensor->FirstChildElement("lidar");
                }
                settings["samples"] = childNumber(ray, {"scan", "horizontal", "samples"}, 360.0);
                settings["min_angle"] =
                    childNumber(ray, {"scan", "horizontal", "min_angle"}, -3.14159265358979);
                settings["max_angle"] =
                    childNumber(ray, {"scan", "horizontal", "max_angle"}, 3.14159265358979);
                settings["min_range"] = childNumber(ray, {"range", "min"}, 0.1);
                settings["max_range"] = childNumber(ray, {"range", "max"}, 30.0);
                settings["range_noise"] =
                    firstStddev(ray != nullptr ? ray->FirstChildElement("noise") : nullptr);
                settings["vertical_samples"] =
                    childNumber(ray, {"scan", "vertical", "samples"}, 1.0);
                settings["vertical_min_angle"] =
                    childNumber(ray, {"scan", "vertical", "min_angle"}, 0.0);
                settings["vertical_max_angle"] =
                    childNumber(ray, {"scan", "vertical", "max_angle"}, 0.0);
                typeName = "Lidar";
            } else if (type == "camera" || type == "depth" || type == "depth_camera" ||
                       type == "rgbd_camera") {
                // One raster camera gives colour, depth and instance ids (sensors.md).
                kind = SensorType::Camera;
                const tinyxml2::XMLElement* camera = sensor->FirstChildElement("camera");
                settings["width"] = childNumber(camera, {"image", "width"}, 640.0);
                settings["height"] = childNumber(camera, {"image", "height"}, 480.0);
                settings["horizontal_fov"] = childNumber(camera, {"horizontal_fov"}, 1.047);
                settings["near_clip"] = childNumber(camera, {"clip", "near"}, 0.05);
                settings["far_clip"] = childNumber(camera, {"clip", "far"}, 100.0);
                typeName = "Camera";
            } else {
                robot.notes.push_back("sensor '" + std::string(name) + "' of type '" + type +
                                      "' is not simulated yet");
                continue;
            }
            const auto bodyId = BodyId{static_cast<std::uint32_t>(body)};
            world.addSensor(name, kind, bodyId, mount, rate, settings);
            ScenePrim prim;
            prim.name = name;
            prim.parent = robot.bodyPrim[body];
            prim.path = robot.scene.prims[prim.parent].path + "/" + name;
            prim.typeName = typeName;
            prim.apiSchemas = {"SensorAPI_1", sensorSchema(kind)};
            prim.body = static_cast<std::uint32_t>(body);
            prim.offset = mount;
            robot.scene.prims.push_back(std::move(prim));
            robot.sensorPrim.push_back(static_cast<std::uint32_t>(robot.scene.prims.size() - 1));
        }
    }
}

}  // namespace

Expected<void, std::string> addUrdf(Robot& robot, const std::string& path,
                                    const LoadOptions& options) {
    const urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(path);
    if (!model) {
        return Unexpected{path + " is not a valid URDF (the parser's message is on the console)"};
    }
    const urdf::LinkConstSharedPtr root = model->getRoot();
    if (!root) {
        return Unexpected{path + " has no root link"};
    }
    const std::size_t firstBody = robot.world.bodies.names.size();
    const std::size_t firstJoint = robot.world.joints.names.size();
    Builder builder(*model, options, robot, std::filesystem::path(path).parent_path().string());
    builder.setActuated(commandedJoints(path));

    ScenePrim top;
    top.name = model->getName().empty() ? "robot" : model->getName();
    top.typeName = "Xform";
    const std::uint32_t topPrim = builder.addPrim(std::move(top));

    // A root link named "world" is the ROS convention for something bolted to the ground.
    const bool fixedBase = root->name == "world";
    builder.setFixedBase(fixedBase);
    if (fixedBase) {
        for (const urdf::JointSharedPtr& joint : root->child_joints) {
            const urdf::LinkConstSharedPtr link = model->getLink(joint->child_link_name);
            if (link) {
                builder.addLink(*link, kWorldBody, topPrim, Pose{}, joint.get());
            }
        }
    } else {
        builder.addLink(*root, kWorldBody, topPrim, options.spawn, nullptr);
        robot.world.bodies.poseInParent[firstBody] = options.spawn;
        robot.world.addJoint(root->name + "_floating", JointType::Free,
                             BodyId{static_cast<std::uint32_t>(firstBody)}, {0.0, 0.0, 1.0}, false);
        robot.scene.prims[topPrim].body = static_cast<std::uint32_t>(firstBody);
    }
    for (std::size_t j = firstJoint; j < robot.world.joints.names.size(); ++j) {
        robot.jointPrim.push_back(robot.bodyPrim.at(robot.world.joints.child[j].index));
    }

    // Place a free robot with its lowest collision shape just above the ground.
    if (!fixedBase && robot.world.bodies.names.size() > firstBody) {
        double lowest = 0.0;
        bool any = false;
        const World::Colliders& colliders = robot.world.colliders;
        for (std::size_t c = 0; c < colliders.body.size(); ++c) {
            if (colliders.body[c] == kWorldBody || colliders.body[c].index < firstBody) {
                continue;
            }
            const Pose shape =
                compose(robot.bodyRestPose[colliders.body[c].index], colliders.poseInBody[c]);
            const double z = lowestPoint(colliders.shape[c], colliders.size[c], shape);
            lowest = any ? std::min(lowest, z) : z;
            any = true;
        }
        if (!any) {
            robot.notes.emplace_back("'" + model->getName() +
                                     "' has no collision shapes and will fall through the ground");
        }
        const double lift = (any ? options.spawn.position[2] - lowest : 0.0) + options.dropHeightM;
        robot.world.bodies.poseInParent[firstBody].position[2] += lift;
        for (std::size_t b = firstBody; b < robot.bodyRestPose.size(); ++b) {
            robot.bodyRestPose[b].position[2] += lift;
        }
    }
    addSensors(robot, path, firstBody);
    return {};
}

Expected<Robot, std::string> loadUrdf(const std::string& path, const LoadOptions& options) {
    Robot robot;
    robot.sourcePath = path;
    robot.world.addCollider(kWorldBody, ShapeType::Plane, {0.0, 0.0, 1.0}, Pose{},
                            options.groundFriction);
    if (auto added = addUrdf(robot, path, options); !added) {
        return Unexpected{added.error()};
    }
    robot.name = robot.scene.prims.front().name;

    addGroundPrim(robot);  // static scenery after the robot
    return robot;
}

std::array<double, 3> recentre(Robot& robot, double rebaseDistanceM) {
    World& world = robot.world;
    if (world.bodies.names.empty()) {
        return {0.0, 0.0, 0.0};
    }
    const std::array<double, 3> shift =
        coords::rebaseShift(robot.bodyRestPose.front().position, rebaseDistanceM);
    if (shift == std::array<double, 3>{0.0, 0.0, 0.0}) {
        return shift;
    }
    const auto move = [&shift](Pose& pose) {
        for (std::size_t i = 0; i < 3; ++i) {
            pose.position.at(i) -= shift.at(i);
        }
    };
    for (std::size_t b = 0; b < world.bodies.names.size(); ++b) {
        if (world.bodies.parent[b] == kWorldBody) {
            move(world.bodies.poseInParent[b]);
        }
        move(robot.bodyRestPose[b]);
    }
    for (std::size_t c = 0; c < world.colliders.body.size(); ++c) {
        if (world.colliders.body[c] != kWorldBody) {
            continue;
        }
        Pose& pose = world.colliders.poseInBody[c];
        if (world.colliders.shape[c] == ShapeType::Plane && world.colliders.size[c][0] == 0.0) {
            // An unbounded plane only moves along its normal (exact for a level ground).
            const std::array<double, 3> normal = rotate(pose.orientation, {0.0, 0.0, 1.0});
            const double along = shift[0] * normal[0] + shift[1] * normal[1] + shift[2] * normal[2];
            for (std::size_t i = 0; i < 3; ++i) {
                pose.position.at(i) -= along * normal.at(i);
            }
        } else {
            move(pose);
        }
    }
    for (ScenePrim& prim : robot.scene.prims) {
        if (prim.body != kNoIndex) {
            continue;
        }
        if (prim.path == "/ground") {
            // It draws the infinite ground plane, so it stays under the region.
            prim.offset.position[2] -= shift[2];
        } else {
            move(prim.offset);
        }
    }
    coords::applyRebase(world.region, shift);
    return shift;
}

bool isUsd(const std::string& path) {
    const std::string extension = std::filesystem::path(path).extension().string();
    return extension == ".usd" || extension == ".usda" || extension == ".usdc";
}

namespace {

/// A file that holds its bodies where they were authored (USD, SDFormat, MJCF): --spawn moves
/// the robot from there, as it moves a URDF robot from its origin.
Expected<Robot, std::string> placed(Expected<Robot, std::string> model, const Pose& spawn) {
    if (!model || (spawn.position == std::array<double, 3>{0.0, 0.0, 0.0} &&
                   spawn.orientation == std::array<double, 4>{1.0, 0.0, 0.0, 0.0})) {
        return model;
    }
    World& world = model->world;
    for (std::size_t b = 0; b < world.bodies.names.size(); ++b) {
        if (world.bodies.parent[b] == kWorldBody) {
            world.bodies.poseInParent[b] = compose(spawn, world.bodies.poseInParent[b]);
        }
        model->bodyRestPose[b] = compose(spawn, model->bodyRestPose[b]);
    }
    return model;
}

}  // namespace

Expected<Robot, std::string> loadModel(const std::string& path, const LoadOptions& options) {
    importers::ImportOptions import;
    import.groundFriction = options.groundFriction;
    switch (importers::formatOf(path)) {
        case importers::Format::Usd: {
            stage::ReadOptions read;
            read.defaultFriction = options.surfaceFriction;
            read.groundFriction = options.groundFriction;
            return placed(stage::loadStage(path, read), options.spawn);
        }
        case importers::Format::Sdf:
            return placed(importers::importSdf(path, import), options.spawn);
        case importers::Format::Mjcf:
            return placed(importers::importMjcf(path, import), options.spawn);
        case importers::Format::Urdf:
        case importers::Format::Unknown:
            break;
    }
    return loadUrdf(path, options);
}

Expected<void, std::string> addModel(Robot& robot, const std::string& path,
                                     const LoadOptions& options) {
    importers::ImportOptions import;
    import.addGround = false;
    Expected<Robot, std::string> other = Unexpected{std::string()};
    switch (importers::formatOf(path)) {
        case importers::Format::Usd: {
            stage::ReadOptions read;
            read.defaultFriction = options.surfaceFriction;
            read.addGround = false;
            other = stage::loadStage(path, read);
            break;
        }
        case importers::Format::Sdf:
            other = importers::importSdf(path, import);
            break;
        case importers::Format::Mjcf:
            other = importers::importMjcf(path, import);
            break;
        case importers::Format::Urdf:
        case importers::Format::Unknown:
            return addUrdf(robot, path, options);
    }
    if (!other) {
        return Unexpected{other.error()};
    }
    appendModel(robot, *other);
    return {};
}

}  // namespace lagrange::server
