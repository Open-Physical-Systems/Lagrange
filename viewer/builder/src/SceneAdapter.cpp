// The document as a SceneFrame for the viewer's own SceneRenderer (plan decisions 1 and 2).
//
// One prim per link (no mesh), a child prim per visual shape, a prim per sensor (a small marker
// mesh), one collider per collision shape. A preview change only re-poses: poseScene keeps the
// statics pointer, so the renderer keeps the meshes on the GPU (R15).
#include "lagrange/viewer/builder/Primitives.hpp"
#include "lagrange/viewer/builder/SceneAdapter.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <utility>

namespace lagrange::viewer::builder {

namespace {

/// Colour of the sensor markers: the sensors domain colour (docs/design/ui tokens, #3FB68B),
/// so a marker reads as a sensor and not as a part of the robot.
constexpr std::array<float, 4> kSensorMarkerColor{0.247F, 0.714F, 0.545F, 1.0F};
/// The lidar marker: a puck (radius, height in m), as lidars are drawn in the reference image.
constexpr double kLidarMarkerRadius = 0.035;
constexpr double kLidarMarkerHeight = 0.04;

bool hasColor(const std::array<float, 4>& rgba) {
    return std::ranges::any_of(rgba, [](float v) { return v != 0.0F; });
}

std::optional<MeshData> shapeMesh(const Shape& shape) {
    switch (shape.kind) {
        case Shape::Kind::Box:
            return boxMesh(shape.size);
        case Shape::Kind::Cylinder:
            return cylinderMesh(shape.size[0], shape.size[1]);
        case Shape::Kind::Sphere:
            return sphereMesh(shape.size[0]);
        case Shape::Kind::Mesh:
            // Mesh files are not loaded by the builder yet (no mesh reader in the viewer):
            // the shape is listed but not drawn. TODO(RFC 0016 phase 2): mesh shapes.
            return std::nullopt;
    }
    return std::nullopt;
}

using Quat = std::array<double, 4>;  // w x y z

Frame jointMotion(const Joint& joint, double q) {
    Frame motion;
    std::array<double, 3> axis = joint.axis;
    const double length = std::hypot(axis[0], axis[1], axis[2]);
    if (length <= 0.0) {
        return motion;  // a zero axis is a Problem; it does not move
    }
    for (double& v : axis) {
        v /= length;
    }
    switch (joint.type) {
        case Joint::Type::Revolute:
        case Joint::Type::Continuous: {
            const double s = std::sin(q * 0.5);
            motion.rotation = Quat{std::cos(q * 0.5), axis[0] * s, axis[1] * s, axis[2] * s};
            break;
        }
        case Joint::Type::Prismatic:
            motion.position = {axis[0] * q, axis[1] * q, axis[2] * q};
            break;
        case Joint::Type::Fixed:
            break;
    }
    return motion;
}

/// The preview position of `joint`: its leader's through the mimic, else the map's, else 0.
double jointPosition(const Document& document, const Joint& joint, const Preview& preview,
                     int depth = 0) {
    if (joint.mimic && depth < static_cast<int>(document.joints.size())) {
        const auto& [leaderName, multiplier, offset] = *joint.mimic;
        if (const Joint* leader = findJoint(document, leaderName); leader != nullptr) {
            return multiplier * jointPosition(document, *leader, preview, depth + 1) + offset;
        }
    }
    const auto found = preview.find(joint.name);
    return found != preview.end() ? found->second : 0.0;
}

Transform toTransform(const Frame& frame) {
    Transform out;
    out.position = frame.position;
    for (std::size_t k = 0; k < 4; ++k) {
        out.rotation.at(k) = static_cast<float>(frame.rotation.at(k));
    }
    return out;
}

}  // namespace

BuiltScene buildStatics(const Document& document, std::uint64_t generation) {
    auto statics = std::make_shared<SceneStatics>();
    statics->generation = generation;
    statics->stagePath = "builder:" + document.robot;
    BuiltScene scene;
    const auto addPrim = [&](PrimData prim, std::string item, int link, const Frame& offset,
                             int sensor) {
        statics->prims.push_back(std::move(prim));
        scene.primItem.push_back(std::move(item));
        scene.primLink.push_back(link);
        scene.primOffset.push_back(offset);
        scene.primSensor.push_back(sensor);
        return static_cast<std::uint32_t>(statics->prims.size() - 1);
    };
    const auto addMaterial = [&](std::string name, const std::array<float, 4>& color) {
        MaterialData material;
        material.name = std::move(name);
        material.baseColor = color;
        statics->materials.push_back(std::move(material));
        return static_cast<std::uint32_t>(statics->materials.size() - 1);
    };

    std::vector<std::uint32_t> linkPrims;  // per link: its own prim
    const std::string root = "/" + (document.robot.empty() ? std::string("robot") : document.robot);
    for (std::size_t l = 0; l < document.links.size(); ++l) {
        const Link& link = document.links[l];
        const int linkIndex = static_cast<int>(l);
        PrimData linkPrim;
        linkPrim.name = link.name;
        linkPrim.path = root + "/" + link.name;
        linkPrim.typeName = "Xform";
        const std::uint32_t linkPrimId = addPrim(linkPrim, link.name, linkIndex, Frame{}, -1);
        linkPrims.push_back(linkPrimId);
        int visualCount = 0;
        for (const Shape& shape : link.shapes) {
            std::optional<MeshData> mesh = shapeMesh(shape);
            if (!mesh || mesh->positions.empty()) {
                continue;
            }
            const Frame offset = toFrame(shape.origin);
            statics->meshes.push_back(std::move(*mesh));
            const auto meshId = static_cast<std::uint32_t>(statics->meshes.size() - 1);
            if (shape.visual) {
                PrimData prim;
                prim.parent = linkPrimId;
                prim.name = "visual_" + std::to_string(visualCount++);
                prim.path = linkPrim.path + "/" + prim.name;
                prim.typeName = "Mesh";
                prim.mesh = meshId;
                prim.material = addMaterial(link.name + "/" + prim.name,
                                            hasColor(shape.rgba) ? shape.rgba : kDefaultShapeColor);
                addPrim(std::move(prim), link.name, linkIndex, offset, -1);
            }
            if (shape.collision) {
                ColliderData collider;
                collider.prim = linkPrimId;
                collider.mesh = meshId;
                const Transform pose = toTransform(offset);
                collider.position = pose.position;
                collider.rotation = pose.rotation;
                statics->colliders.push_back(collider);
            }
        }
    }

    // Sensors: a marker each, on its link (a sensor on a missing link sits at the origin; that
    // is a Problem the validation names).
    std::optional<std::uint32_t> cubeMesh;
    std::optional<std::uint32_t> puckMesh;
    std::optional<std::uint32_t> sensorMaterial;
    for (std::size_t s = 0; s < document.sensors.size(); ++s) {
        const Sensor& sensor = document.sensors[s];
        const bool lidar = sensor.type == "lidar" || sensor.type == "gpu_lidar" ||
                           sensor.type == "ray" || sensor.type == "gpu_ray";
        std::optional<std::uint32_t>& mesh = lidar ? puckMesh : cubeMesh;
        if (!mesh) {
            statics->meshes.push_back(
                lidar ? cylinderMesh(kLidarMarkerRadius, kLidarMarkerHeight)
                      : boxMesh({kSensorMarkerSize, kSensorMarkerSize, kSensorMarkerSize}));
            mesh = static_cast<std::uint32_t>(statics->meshes.size() - 1);
        }
        if (!sensorMaterial) {
            sensorMaterial = addMaterial("sensor", kSensorMarkerColor);
        }
        int linkIndex = -1;
        std::uint32_t parent = kNone;
        for (std::size_t l = 0; l < document.links.size(); ++l) {
            if (document.links[l].name == sensor.link) {
                linkIndex = static_cast<int>(l);
                break;
            }
        }
        if (linkIndex >= 0) {
            parent = linkPrims[static_cast<std::size_t>(linkIndex)];
        }
        PrimData prim;
        prim.parent = parent;
        prim.name = sensor.name;
        prim.path = (parent != kNone ? statics->prims[parent].path : root) + "/" + sensor.name;
        prim.typeName = "Sensor";
        prim.mesh = *mesh;
        prim.material = *sensorMaterial;
        addPrim(std::move(prim), sensor.name, linkIndex, toFrame(sensor.origin),
                static_cast<int>(s));
    }
    scene.statics = std::move(statics);
    return scene;
}

std::shared_ptr<const SceneFrame> poseScene(const BuiltScene& scene,
                                            const std::vector<Frame>& links) {
    auto frame = std::make_shared<SceneFrame>();
    frame->statics = scene.statics;
    const std::size_t count = scene.statics ? scene.statics->prims.size() : 0;
    frame->transforms.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
        const int link = i < scene.primLink.size() ? scene.primLink[i] : -1;
        const Frame offset = i < scene.primOffset.size() ? scene.primOffset[i] : Frame{};
        const Frame base = link >= 0 && static_cast<std::size_t>(link) < links.size()
                               ? links[static_cast<std::size_t>(link)]
                               : Frame{};
        frame->transforms[i] = toTransform(compose(base, offset));
    }
    return frame;
}

std::vector<Frame> previewFrames(const Document& document, const Preview& preview) {
    const Tree tree = buildTree(document);
    if (tree.ok) {
        return linkFrames(document, tree, preview);
    }
    // Local forward kinematics until WP-A's buildTree lands (it is a stub that is never ok).
    const std::size_t n = document.links.size();
    std::vector<int> parentJoint(n, -1);
    for (std::size_t j = 0; j < document.joints.size(); ++j) {
        for (std::size_t l = 0; l < n; ++l) {
            if (document.links[l].name == document.joints[j].child && parentJoint[l] < 0) {
                parentJoint[l] = static_cast<int>(j);
                break;
            }
        }
    }
    std::vector<Frame> frames(n);
    std::vector<int> state(n, 0);  // 0 not done, 1 in progress (a loop), 2 done
    const auto solve = [&](auto&& self, std::size_t l) -> void {
        if (state[l] != 0) {
            return;  // done, or a loop: left where it is (the identity)
        }
        state[l] = 1;
        if (const int j = parentJoint[l]; j >= 0) {
            const Joint& joint = document.joints[static_cast<std::size_t>(j)];
            for (std::size_t p = 0; p < n; ++p) {
                if (document.links[p].name == joint.parent && p != l) {
                    self(self, p);
                    if (state[p] == 2) {
                        frames[l] =
                            compose(compose(frames[p], toFrame(joint.origin)),
                                    jointMotion(joint, jointPosition(document, joint, preview)));
                    }
                    break;
                }
            }
        }
        state[l] = 2;
    };
    for (std::size_t l = 0; l < n; ++l) {
        solve(solve, l);
    }
    return frames;
}

std::uint32_t primOf(const BuiltScene& scene, std::string_view item) {
    for (std::size_t i = 0; i < scene.primItem.size(); ++i) {
        // The first prim of an item is its own (the link's Xform, or the sensor's marker).
        if (scene.primItem[i] == item) {
            return static_cast<std::uint32_t>(i);
        }
    }
    return kNone;
}

}  // namespace lagrange::viewer::builder
