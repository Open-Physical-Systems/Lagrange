// SDFormat -> world tables and scene, through libsdformat's DOM (roadmap 0.2-G3).
#include "Description.hpp"
#include "lagrange/importers/Importers.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <gz/math/Inertial.hh>
#include <gz/math/Pose3.hh>
#include <limits>
#include <sdf/Actor.hh>
#include <sdf/Camera.hh>
#include <sdf/Collision.hh>
#include <sdf/Element.hh>
#include <sdf/Error.hh>
#include <sdf/Geometry.hh>
#include <sdf/Imu.hh>
#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Lidar.hh>
#include <sdf/Link.hh>
#include <sdf/Material.hh>
#include <sdf/Model.hh>
#include <sdf/ParserConfig.hh>
#include <sdf/Pbr.hh>
#include <sdf/Plugin.hh>
#include <sdf/Root.hh>
#include <sdf/Sensor.hh>
#include <sdf/Surface.hh>
#include <sdf/Visual.hh>
#include <sdf/World.hh>
#include <sdf/sdf.hh>
#include <set>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

namespace lagrange::importers {

namespace {

using detail::CollisionDesc;
using detail::Description;
using detail::Geom;
using detail::GeomType;
using detail::JointDesc;
using detail::LinkDesc;
using detail::SensorDesc;
using detail::VisualDesc;

// libsdformat's stand-ins for "no limit": axis limits of ±1e16 and effort/velocity of -1.
constexpr double kUnlimited = 1e16;

Pose poseOf(const gz::math::Pose3d& pose) {
    return {{pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z()},
            {pose.Rot().W(), pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z()}};
}

std::string errorText(const sdf::Errors& errors) {
    std::ostringstream out;
    for (const sdf::Error& error : errors) {
        out << "\n  " << error.Message();
    }
    return out.str();
}

/// "::" (SDFormat's scope separator) as "_", for body, joint and sensor names.
std::string flat(std::string name) {
    for (std::size_t at = name.find("::"); at != std::string::npos; at = name.find("::", at)) {
        name.replace(at, 2, "_");
    }
    return name;
}

/// Every leaf value under a plugin's elements: the joint names a plugin refers to.
void leafValues(const sdf::ElementPtr& element, std::set<std::string>& out) {
    if (!element) {
        return;
    }
    if (!element->GetFirstElement() && element->GetValue()) {
        out.insert(element->GetValue()->GetAsString());
    }
    for (sdf::ElementPtr child = element->GetFirstElement(); child;
         child = child->GetNextElement()) {
        leafValues(child, out);
    }
}

class Reader {
public:
    explicit Reader(Model& model) : model_(model) {}

    /// The links and joints of `model` (and its nested models) into `d`, scoped with `scope`.
    void readModel(const sdf::Model& model, const Pose& modelWorld, const std::string& scope,
                   bool isStatic, Description& d) {
        const bool fixed = isStatic || model.Static();
        for (const sdf::Plugin& plugin : model.Plugins()) {
            std::set<std::string> values;
            for (const sdf::ElementPtr& element : plugin.Contents()) {
                leafValues(element, values);
            }
            for (const std::string& value : values) {
                pluginValues_.insert(scope + value);
            }
        }
        for (std::uint64_t l = 0; l < model.LinkCount(); ++l) {
            const sdf::Link& link = *model.LinkByIndex(l);
            gz::math::Pose3d pose;
            if (const sdf::Errors errors = link.SemanticPose().Resolve(pose, "__model__");
                !errors.empty()) {
                detail::note(model_, "link '" + link.Name() + "': " + errorText(errors));
            }
            LinkDesc out;
            out.name = flat(scope + link.Name());
            out.pose = compose(modelWorld, poseOf(pose));
            if (!fixed) {
                const gz::math::Inertiald& inertial = link.Inertial();
                out.mass = inertial.MassMatrix().Mass();
                const gz::math::Vector3d& com = inertial.Pose().Pos();
                out.com = {com.X(), com.Y(), com.Z()};
                const gz::math::Matrix3d moi = inertial.Moi();  // about the CoM, link axes
                out.inertia = {moi(0, 0), moi(1, 1), moi(2, 2), moi(0, 1), moi(0, 2), moi(1, 2)};
            }
            for (std::uint64_t c = 0; c < link.CollisionCount(); ++c) {
                readCollision(*link.CollisionByIndex(c), out);
            }
            for (std::uint64_t v = 0; v < link.VisualCount(); ++v) {
                readVisual(*link.VisualByIndex(v), out);
            }
            for (std::uint64_t s = 0; s < link.SensorCount(); ++s) {
                readSensor(*link.SensorByIndex(s), out);
            }
            d.links.push_back(std::move(out));
            if (!fixed) {
                floating_.insert(d.links.back().name);
            }
        }
        for (std::uint64_t j = 0; j < model.JointCount(); ++j) {
            readJoint(*model.JointByIndex(j), scope, d);
        }
        for (std::uint64_t m = 0; m < model.ModelCount(); ++m) {
            const sdf::Model& nested = *model.ModelByIndex(m);
            gz::math::Pose3d pose;
            if (const sdf::Errors errors = nested.SemanticPose().Resolve(pose, "__model__");
                !errors.empty()) {
                detail::note(model_, "model '" + nested.Name() + "': " + errorText(errors));
            }
            readModel(nested, compose(modelWorld, poseOf(pose)),
                      scope + nested.Name() + "::", fixed, d);
        }
    }

    /// After readModel: links that are no joint's child float, and plugin joints are actuated.
    void finish(Description& d) {
        std::set<std::string> children;
        for (const JointDesc& joint : d.joints) {
            children.insert(joint.child);
        }
        for (const LinkDesc& link : d.links) {
            if (floating_.contains(link.name) && !children.contains(link.name)) {
                JointDesc joint;
                joint.name = link.name + "_floating";
                joint.type = JointType::Free;
                joint.child = link.name;
                joint.axis = {0.0, 0.0, 1.0};
                d.joints.push_back(std::move(joint));
            }
        }
        for (JointDesc& joint : d.joints) {
            if (joint.type != JointType::Free && joint.type != JointType::Fixed &&
                actuatedScoped_.contains(joint.name)) {
                joint.actuated = true;
            }
        }
    }

    /// A joint of the world, between links named `model::link` (the descriptions' links are
    /// already named so, flattened).
    void readWorldJoint(const sdf::Joint& joint, Description& d) { readJoint(joint, "", d); }

private:
    void readJoint(const sdf::Joint& joint, const std::string& scope, Description& d) {
        JointDesc out;
        out.name = flat(scope + joint.Name());
        std::string parent;
        std::string child;
        if (sdf::Errors errors = joint.ResolveParentLink(parent); !errors.empty()) {
            detail::note(model_, "joint '" + joint.Name() + "': " + errorText(errors));
            return;
        }
        if (sdf::Errors errors = joint.ResolveChildLink(child); !errors.empty()) {
            detail::note(model_, "joint '" + joint.Name() + "': " + errorText(errors));
            return;
        }
        out.parent = parent == "world" ? std::string() : flat(scope + parent);
        out.child = flat(scope + child);
        gz::math::Pose3d anchor;
        if (sdf::Errors errors = joint.SemanticPose().Resolve(anchor, child); !errors.empty()) {
            detail::note(model_, "joint '" + joint.Name() + "': " + errorText(errors));
        }
        out.anchor = poseOf(anchor);
        const sdf::JointAxis* axis = joint.Axis(0);
        switch (joint.Type()) {
            case sdf::JointType::REVOLUTE:
                out.type = JointType::Revolute;
                break;
            case sdf::JointType::CONTINUOUS:
                out.type = JointType::Continuous;
                break;
            case sdf::JointType::PRISMATIC:
                out.type = JointType::Prismatic;
                break;
            case sdf::JointType::BALL:
                out.type = JointType::Spherical;
                break;
            case sdf::JointType::FIXED:
                out.type = JointType::Fixed;
                break;
            default:
                detail::note(model_, "joint '" + joint.Name() +
                                         "' is of a type that is not supported; it is fixed");
                out.type = JointType::Fixed;
                break;
        }
        if (axis != nullptr) {
            out.damping = axis->Damping();
            out.friction = axis->Friction();
            if (const std::optional<sdf::MimicConstraint> mimic = axis->Mimic(); mimic) {
                out.mimic = flat(scope + mimic->Joint());
                out.mimicMultiplier = mimic->Multiplier();
                out.mimicOffset = mimic->Offset() - mimic->Multiplier() * mimic->Reference();
            }
            gz::math::Vector3d xyz = axis->Xyz();
            if (sdf::Errors errors = axis->ResolveXyz(xyz); !errors.empty()) {
                detail::note(model_, "joint '" + joint.Name() + "': " + errorText(errors));
            }
            out.axis = {xyz.X(), xyz.Y(), xyz.Z()};
            const bool limited = axis->Lower() > -kUnlimited && axis->Upper() < kUnlimited &&
                                 axis->Lower() < axis->Upper();
            if (out.type == JointType::Revolute && !limited) {
                out.type = JointType::Continuous;
            } else if (limited) {
                out.lower = axis->Lower();
                out.upper = axis->Upper();
            }
            out.effort =
                axis->Effort() > 0.0 && std::isfinite(axis->Effort()) ? axis->Effort() : 0.0;
            out.velocity = axis->MaxVelocity() > 0.0 && std::isfinite(axis->MaxVelocity())
                               ? axis->MaxVelocity()
                               : 0.0;
        }
        if (pluginValues_.contains(scope + joint.Name())) {
            actuatedScoped_.insert(out.name);
        }
        d.joints.push_back(std::move(out));
    }

    /// The geometry in `geom`; false (with a note) when it has none that can be used.
    bool readGeometry(const sdf::Geometry* geometry, const std::string& owner, Geom& geom) {
        if (geometry == nullptr) {
            return false;
        }
        switch (geometry->Type()) {
            case sdf::GeometryType::BOX: {
                const gz::math::Vector3d size = geometry->BoxShape()->Size();
                geom.type = GeomType::Box;
                geom.size = {size.X() / 2.0, size.Y() / 2.0, size.Z() / 2.0};
                return true;
            }
            case sdf::GeometryType::SPHERE:
                geom.type = GeomType::Sphere;
                geom.size = {geometry->SphereShape()->Radius(), 0.0, 0.0};
                return true;
            case sdf::GeometryType::CYLINDER:
                geom.type = GeomType::Cylinder;
                geom.size = {geometry->CylinderShape()->Radius(),
                             geometry->CylinderShape()->Length() / 2.0, 0.0};
                return true;
            case sdf::GeometryType::CAPSULE:
                geom.type = GeomType::Capsule;
                geom.size = {geometry->CapsuleShape()->Radius(),
                             geometry->CapsuleShape()->Length() / 2.0, 0.0};
                return true;
            case sdf::GeometryType::ELLIPSOID: {
                const gz::math::Vector3d radii = geometry->EllipsoidShape()->Radii();
                geom.type = GeomType::Ellipsoid;
                geom.size = {radii.X(), radii.Y(), radii.Z()};
                return true;
            }
            case sdf::GeometryType::PLANE: {
                const sdf::Plane& plane = *geometry->PlaneShape();
                const gz::math::Vector3d normal = plane.Normal().Normalized();
                if (std::abs(normal.Z() - 1.0) > 1e-9) {
                    detail::note(model_, "the plane of '" + owner +
                                             "' does not face +z; only ground planes are used");
                    return false;
                }
                geom.type = GeomType::Plane;
                geom.size = {plane.Size().X() / 2.0, plane.Size().Y() / 2.0, 0.0};
                return true;
            }
            case sdf::GeometryType::MESH: {
                const gz::math::Vector3d scale = geometry->MeshShape()->Scale();
                geom.type = GeomType::Mesh;
                geom.size = {scale.X(), scale.Y(), scale.Z()};
                return true;  // build() notes that mesh files are not loaded yet
            }
            default:
                detail::note(model_, "'" + owner + "' has a geometry type that is not supported");
                return false;
        }
    }

    void readCollision(const sdf::Collision& collision, LinkDesc& link) {
        CollisionDesc out;
        if (!readGeometry(collision.Geom(), collision.Name(), out.geom)) {
            return;
        }
        gz::math::Pose3d pose;
        if (sdf::Errors errors = collision.SemanticPose().Resolve(pose); !errors.empty()) {
            detail::note(model_, "collision '" + collision.Name() + "': " + errorText(errors));
        }
        out.geom.pose = poseOf(pose);
        const sdf::Surface* surface = collision.Surface();
        if (surface != nullptr && surface->Friction() != nullptr &&
            surface->Friction()->ODE() != nullptr) {
            out.friction = surface->Friction()->ODE()->Mu();
        }
        link.collisions.push_back(out);
    }

    void readVisual(const sdf::Visual& visual, LinkDesc& link) {
        VisualDesc out;
        if (!readGeometry(visual.Geom(), visual.Name(), out.geom)) {
            return;
        }
        gz::math::Pose3d pose;
        if (sdf::Errors errors = visual.SemanticPose().Resolve(pose); !errors.empty()) {
            detail::note(model_, "visual '" + visual.Name() + "': " + errorText(errors));
        }
        out.geom.pose = poseOf(pose);
        if (const sdf::Material* material = visual.Material(); material != nullptr) {
            const gz::math::Color color = material->Diffuse();
            out.rgba = {color.R(), color.G(), color.B(),
                        color.A() * (1.0F - visual.Transparency())};
            const gz::math::Color glow = material->Emissive();
            SurfaceLook look;
            look.emissive = {glow.R(), glow.G(), glow.B()};
            bool told = glow.R() > 0.0F || glow.G() > 0.0F || glow.B() > 0.0F;
            if (const sdf::Pbr* pbr = material->PbrMaterial(); pbr != nullptr) {
                if (const sdf::PbrWorkflow* metal = pbr->Workflow(sdf::PbrWorkflowType::METAL);
                    metal != nullptr) {
                    look.metallic = static_cast<float>(metal->Metalness());
                    look.roughness = static_cast<float>(metal->Roughness());
                    told = true;
                }
            }
            if (told) {
                out.look = look;
            }
        }
        link.visuals.push_back(out);
    }

    /// A custom sensor's parameters: every leaf element under <sensor> by its path, as the URDF
    /// reader does (numbers into settings, the rest into text; a first element named as the type
    /// is left out of the path).
    static void customParameters(const sdf::ElementPtr& element, const std::string& prefix,
                                 const std::string& type, SensorDesc& out) {
        static const std::set<std::string, std::less<>> kNotParameters{
            "pose",  "update_rate", "always_on",      "visualize",
            "topic", "plugin",      "enable_metrics", "frame_id"};
        for (sdf::ElementPtr child = element->GetFirstElement(); child != nullptr;
             child = child->GetNextElement()) {
            const std::string name = child->GetName();
            if (prefix.empty() && kNotParameters.contains(name)) {
                continue;
            }
            const std::string path =
                prefix.empty() && name == type ? "" : (prefix.empty() ? name : prefix + "/" + name);
            if (child->GetFirstElement() != nullptr) {
                customParameters(child, path, type, out);
                continue;
            }
            if (path.empty() || !child->GetValue()) {
                continue;
            }
            const std::string value = child->GetValue()->GetAsString();
            char* end = nullptr;
            const double number = std::strtod(value.c_str(), &end);
            if (!value.empty() && end != value.c_str() && *end == '\0') {
                out.settings[path] = number;
            } else {
                out.text[path] = value;
            }
        }
    }

    void readSensor(const sdf::Sensor& sensor, LinkDesc& link) {
        SensorDesc out;
        out.name = sensor.Name();
        out.rateHz = sensor.UpdateRate();
        gz::math::Pose3d pose;
        if (sdf::Errors errors = sensor.SemanticPose().Resolve(pose); !errors.empty()) {
            detail::note(model_, "sensor '" + sensor.Name() + "': " + errorText(errors));
        }
        out.pose = poseOf(pose);
        if (const sdf::Imu* imu = sensor.ImuSensor();
            sensor.Type() == sdf::SensorType::IMU && imu != nullptr) {
            out.type = SensorType::Imu;
            out.settings["gyro_noise"] = std::max({imu->AngularVelocityXNoise().StdDev(),
                                                   imu->AngularVelocityYNoise().StdDev(),
                                                   imu->AngularVelocityZNoise().StdDev()});
            out.settings["accel_noise"] = std::max({imu->LinearAccelerationXNoise().StdDev(),
                                                    imu->LinearAccelerationYNoise().StdDev(),
                                                    imu->LinearAccelerationZNoise().StdDev()});
        } else if (const sdf::Lidar* lidar = sensor.LidarSensor();
                   (sensor.Type() == sdf::SensorType::LIDAR ||
                    sensor.Type() == sdf::SensorType::GPU_LIDAR) &&
                   lidar != nullptr) {
            out.type = SensorType::Lidar;
            out.settings["samples"] = lidar->HorizontalScanSamples();
            out.settings["min_angle"] = lidar->HorizontalScanMinAngle().Radian();
            out.settings["max_angle"] = lidar->HorizontalScanMaxAngle().Radian();
            out.settings["min_range"] = lidar->RangeMin();
            out.settings["max_range"] = lidar->RangeMax();
            out.settings["range_noise"] = lidar->LidarNoise().StdDev();
            out.settings["vertical_samples"] = lidar->VerticalScanSamples();
            out.settings["vertical_min_angle"] = lidar->VerticalScanMinAngle().Radian();
            out.settings["vertical_max_angle"] = lidar->VerticalScanMaxAngle().Radian();
        } else if (const sdf::Camera* camera = sensor.CameraSensor();
                   (sensor.Type() == sdf::SensorType::CAMERA ||
                    sensor.Type() == sdf::SensorType::DEPTH_CAMERA ||
                    sensor.Type() == sdf::SensorType::RGBD_CAMERA ||
                    sensor.Type() == sdf::SensorType::THERMAL_CAMERA) &&
                   camera != nullptr) {
            // One raster camera gives colour, depth and instance ids; it looks along x.
            out.type = SensorType::Camera;
            out.settings["width"] = camera->ImageWidth();
            out.settings["height"] = camera->ImageHeight();
            out.settings["horizontal_fov"] = camera->HorizontalFov().Radian();
            out.settings["near_clip"] = camera->NearClip();
            out.settings["far_clip"] = camera->FarClip();
            if (sensor.Type() == sdf::SensorType::THERMAL_CAMERA) {
                out.settings["thermal"] = 1.0;  // ThermalCameraAPI_1's defaults
            }
        } else if (!sensor.TypeStr().empty() && sensor.Element() != nullptr) {
            // Any other type is a custom sensor that a plugin implements (RFC 0008 §3).
            out.type = SensorType::Custom;
            out.kind = sensor.TypeStr();
            customParameters(sensor.Element(), "", out.kind, out);
        } else {
            detail::note(model_, "sensor '" + sensor.Name() + "' has no type; skipped");
            return;
        }
        link.sensors.push_back(std::move(out));
    }

    Model& model_;
    std::set<std::string> floating_;        // links of models that are not static
    std::set<std::string> pluginValues_;    // scoped values from model plugins
    std::set<std::string> actuatedScoped_;  // flat joint names a plugin names
};

sdf::ParserConfig parserConfig(const std::string& path) {
    sdf::ParserConfig config;
    config.AddURIPath("model://", std::filesystem::path(path).parent_path().string());
    if (const char* resources = std::getenv("GZ_SIM_RESOURCE_PATH"); resources != nullptr) {
        std::istringstream list(resources);
        for (std::string dir; std::getline(list, dir, ':');) {
            if (!dir.empty()) {
                config.AddURIPath("model://", dir);
            }
        }
    }
    return config;
}

/// Joints between a world's models: the models they join become one description, their links and
/// joints named `model_link`, so that the joined robot is one tree. A joint to the world holds its
/// model where the joint says, in place of floating.
void joinModels(const sdf::World& world, Reader& reader, std::vector<Description>& descriptions,
                Model& model) {
    const auto modelOf = [](const std::string& link) {
        const std::size_t at = link.find("::");
        return at == std::string::npos ? std::string() : link.substr(0, at);
    };
    const auto prefixed = [](Description& d) {
        const std::string prefix = d.name + "_";
        for (detail::LinkDesc& link : d.links) {
            link.name = prefix + link.name;
        }
        for (detail::JointDesc& joint : d.joints) {
            joint.name = prefix + joint.name;
            joint.child = prefix + joint.child;
            joint.parent = joint.parent.empty() ? joint.parent : prefix + joint.parent;
            joint.mimic = joint.mimic.empty() ? joint.mimic : prefix + joint.mimic;
        }
    };
    const auto find = [&descriptions](const std::string& name) {
        return std::find_if(descriptions.begin(), descriptions.end(),
                            [&name](const Description& d) { return d.name == name; });
    };
    std::map<std::string, std::string> holder;  // each joined model: the description now holding it
    const auto holderOf = [&](const std::string& name) {
        const auto it = holder.find(name);
        if (it != holder.end()) {
            return it->second;
        }
        const auto d = find(name);
        if (d != descriptions.end()) {
            prefixed(*d);
            holder[name] = name;
            return name;
        }
        return std::string();
    };
    for (std::uint64_t j = 0; j < world.JointCount(); ++j) {
        const sdf::Joint& joint = *world.JointByIndex(j);
        std::string parent;
        std::string child;
        if (sdf::Errors errors = joint.ResolveParentLink(parent); !errors.empty()) {
            detail::note(model, "joint '" + joint.Name() + "': " + errorText(errors));
            continue;
        }
        if (sdf::Errors errors = joint.ResolveChildLink(child); !errors.empty()) {
            detail::note(model, "joint '" + joint.Name() + "': " + errorText(errors));
            continue;
        }
        const std::string to = holderOf(modelOf(child));
        const std::string from = parent == "world" ? to : holderOf(modelOf(parent));
        if (to.empty() || from.empty()) {
            detail::note(model,
                         "joint '" + joint.Name() + "' joins no links of this world's models");
            continue;
        }
        const auto target = find(from);
        if (from != to) {  // the child's whole model moves in with the parent's
            const auto moved = find(to);
            const auto firstMesh = static_cast<std::uint32_t>(target->meshes.size());
            for (detail::LinkDesc& link : moved->links) {
                for (detail::VisualDesc& visual : link.visuals) {
                    if (visual.geom.mesh != kNoIndex) {
                        visual.geom.mesh += firstMesh;
                    }
                }
                for (detail::CollisionDesc& collision : link.collisions) {
                    if (collision.geom.mesh != kNoIndex) {
                        collision.geom.mesh += firstMesh;
                    }
                }
                target->links.push_back(std::move(link));
            }
            std::move(moved->joints.begin(), moved->joints.end(),
                      std::back_inserter(target->joints));
            std::move(moved->meshes.begin(), moved->meshes.end(),
                      std::back_inserter(target->meshes));
            for (auto& [name, now] : holder) {
                now = now == to ? from : now;
            }
            descriptions.erase(moved);
        }
        const auto into = find(from);
        const std::string childLink = flat(child);
        std::erase_if(into->joints, [&childLink](const detail::JointDesc& existing) {
            return existing.type == JointType::Free && existing.child == childLink;
        });
        reader.readWorldJoint(joint, *into);
    }
}

}  // namespace

Expected<Model, std::string> importSdf(const std::string& path, const ImportOptions& options) {
    if (std::error_code ec; !std::filesystem::is_regular_file(path, ec)) {
        return Unexpected{"cannot read " + path};
    }
    sdf::Root root;
    sdf::Errors errors = root.Load(path, parserConfig(path));
    std::erase_if(errors, [](const sdf::Error& e) { return e.Code() == sdf::ErrorCode::WARNING; });
    if (!errors.empty()) {
        return Unexpected{path + " is not a valid SDFormat file:" + errorText(errors)};
    }
    Model model;
    model.sourcePath = path;
    Reader reader(model);
    std::vector<Description> descriptions;
    if (const sdf::Model* top = root.Model(); top != nullptr) {
        gz::math::Pose3d pose = top->RawPose();
        Description d;
        d.name = top->Name();
        reader.readModel(*top, poseOf(pose), "", false, d);
        reader.finish(d);
        descriptions.push_back(std::move(d));
    } else if (root.WorldCount() > 0) {
        const sdf::World& world = *root.WorldByIndex(0);
        const gz::math::Vector3d gravity = world.Gravity();
        model.world.gravityLocal = {gravity.X(), gravity.Y(), gravity.Z()};
        for (std::uint64_t m = 0; m < world.ModelCount(); ++m) {
            const sdf::Model& entry = *world.ModelByIndex(m);
            gz::math::Pose3d pose;
            if (sdf::Errors resolve = entry.SemanticPose().Resolve(pose, "world");
                !resolve.empty()) {
                return Unexpected{"model '" + entry.Name() + "': " + errorText(resolve)};
            }
            Description d;
            d.name = entry.Name();
            reader.readModel(entry, poseOf(pose), "", false, d);
            reader.finish(d);
            descriptions.push_back(std::move(d));
        }
        joinModels(world, reader, descriptions, model);
    } else {
        return Unexpected{path + " has no <model> and no <world>"};
    }
    const std::uint64_t actors = root.WorldCount() > 0 ? root.WorldByIndex(0)->ActorCount() : 0;
    if (descriptions.empty() && actors == 0) {
        return Unexpected{path + " has no models"};
    }

    bool ownGround = false;
    for (const Description& d : descriptions) {
        ownGround = ownGround || detail::hasGroundPlane(d);
    }
    const bool addGround = options.addGround && !ownGround;
    if (addGround) {
        model.world.addCollider(kWorldBody, ShapeType::Plane, {0.0, 0.0, 1.0}, Pose{},
                                options.groundFriction);
    }
    for (const Description& d : descriptions) {
        if (auto built = detail::build(d, model); !built) {
            return Unexpected{path + ": " + built.error()};
        }
    }
    model.name = model.scene.prims.empty() ? std::string("actors") : model.scene.prims.front().name;
    // Actors (Gazebo's people) become HumanAPI_1 prims, which the server expands (RFC 0009 §2):
    // their script's waypoints, walked at its mean speed; skin and animation files are ignored.
    for (std::uint64_t a = 0; a < actors; ++a) {
        const sdf::Actor& actor = *root.WorldByIndex(0)->ActorByIndex(a);
        const Pose start = poseOf(actor.RawPose());
        SchemaInstance human;
        human.schema = "HumanAPI_1";
        human.prim = model.world.addPrim("/" + actor.Name(), actor.Name());
        model.world.prims.poseInBody.at(human.prim.index) = start;
        human.integers["human:loop"] = actor.ScriptLoop() ? 1 : 0;
        std::vector<double> waypoints;
        double distance = 0.0;
        double seconds = 0.0;
        std::array<double, 2> last{start.position[0], start.position[1]};
        for (std::uint64_t t = 0; t < actor.TrajectoryCount(); ++t) {
            const sdf::Trajectory& trajectory = *actor.TrajectoryByIndex(t);
            double firstTime = 0.0;
            double lastTime = 0.0;
            for (std::uint64_t w = 0; w < trajectory.WaypointCount(); ++w) {
                const sdf::Waypoint& waypoint = *trajectory.WaypointByIndex(w);
                const std::array<double, 3> at = compose(start, poseOf(waypoint.Pose())).position;
                distance += std::hypot(at[0] - last[0], at[1] - last[1]);
                last = {at[0], at[1]};
                waypoints.insert(waypoints.end(), {at[0], at[1]});
                firstTime = w == 0 ? waypoint.Time() : firstTime;
                lastTime = waypoint.Time();
            }
            seconds += lastTime - firstTime;
        }
        human.tokens["human:behavior"] = waypoints.empty() ? "stand" : "waypoints";
        human.realArrays["human:waypoints"] = std::move(waypoints);
        if (seconds > 0.0 && distance > 0.0) {
            human.reals["human:walkSpeed"] = distance / seconds;
        }
        model.world.schemaInstances.push_back(std::move(human));
    }
    if (addGround) {
        addGroundPrim(model);
    }
    return model;
}

}  // namespace lagrange::importers
