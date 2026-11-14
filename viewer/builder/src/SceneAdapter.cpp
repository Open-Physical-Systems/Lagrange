// The document as a SceneFrame for the viewer's own SceneRenderer (plan decisions 1 and 2).
//
// One prim per link (no mesh), a child prim per visual shape, a prim per sensor (a small marker
// mesh), one collider per collision shape. A preview change only re-poses: poseScene keeps the
// statics pointer, so the renderer keeps the meshes on the GPU (R15).
#include "lagrange/viewer/builder/MeshShapes.hpp"
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

/// One drawn piece of a shape: a primitive is one, a mesh file one per material.
struct Piece {
    MeshData mesh;
    std::array<float, 4> color{};  // the file's colour; all zero: none (the shape's or default)
};

/// A mesh file's parts at the shape's scale (PrimData has no scale: it is baked in).
std::vector<Piece> meshPieces(const Shape& shape) {
    std::vector<Piece> pieces;
    const std::shared_ptr<const MeshFile> file = loadMesh(shape.mesh);
    for (std::size_t p = 0; p < file->parts.size(); ++p) {
        Piece piece{file->parts[p], file->colors[p]};
        MeshData& mesh = piece.mesh;
        for (std::size_t i = 0; i + 2 < mesh.positions.size(); i += 3) {
            for (std::size_t k = 0; k < 3; ++k) {
                mesh.positions[i + k] *= static_cast<float>(shape.scale.at(k));
            }
        }
        // A mirroring or uneven scale turns the normals: n' ∝ n / scale.
        for (std::size_t i = 0; i + 2 < mesh.normals.size(); i += 3) {
            std::array<double, 3> n{};
            for (std::size_t k = 0; k < 3; ++k) {
                n.at(k) =
                    mesh.normals[i + k] / (shape.scale.at(k) != 0.0 ? shape.scale.at(k) : 1.0);
            }
            const double length = std::hypot(n[0], n[1], n[2]);
            for (std::size_t k = 0; k < 3 && length > 0.0; ++k) {
                mesh.normals[i + k] = static_cast<float>(n.at(k) / length);
            }
        }
        if (!mesh.positions.empty()) {
            std::array<float, 6> box{mesh.positions[0], mesh.positions[1], mesh.positions[2],
                                     mesh.positions[0], mesh.positions[1], mesh.positions[2]};
            for (std::size_t i = 0; i + 2 < mesh.positions.size(); i += 3) {
                for (std::size_t k = 0; k < 3; ++k) {
                    box.at(k) = std::min(box.at(k), mesh.positions[i + k]);
                    box.at(k + 3) = std::max(box.at(k + 3), mesh.positions[i + k]);
                }
            }
            mesh.aabb = box;
        }
        pieces.push_back(std::move(piece));
    }
    return pieces;
}

std::vector<Piece> shapePieces(const Shape& shape) {
    switch (shape.kind) {
        case Shape::Kind::Box:
            return {Piece{boxMesh(shape.size)}};
        case Shape::Kind::Cylinder:
            return {Piece{cylinderMesh(shape.size[0], shape.size[1])}};
        case Shape::Kind::Sphere:
            return {Piece{sphereMesh(shape.size[0])}};
        case Shape::Kind::Capsule:
            return {Piece{capsuleMesh(shape.size[0], shape.size[1])}};
        case Shape::Kind::Mesh:
            return meshPieces(shape);  // Addendum A: read by lagrange_meshes
    }
    return {};
}

/// What the shape collides as, to draw: a primitive itself, a mesh its server hull.
std::optional<MeshData> collisionMesh(const Shape& shape, std::optional<std::uint32_t> drawn,
                                      const SceneStatics& statics) {
    if (shape.kind != Shape::Kind::Mesh) {
        if (drawn) {
            return statics.meshes[*drawn];
        }
        return std::nullopt;
    }
    const std::shared_ptr<const MeshFile> file = loadMesh(shape.mesh);
    if (!file->error.empty()) {
        return std::nullopt;
    }
    const MeshSolid& solid = file->solid(shape.scale);
    if (!solid.hull) {
        return std::nullopt;
    }
    return surfaceMesh(*solid.hull);
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

BuiltScene buildStatics(const Document& document, std::uint64_t generation,
                        const BuildOptions& options) {
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
        scene.primShape.push_back(-1);
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
    struct CollisionPrim {
        std::uint32_t mesh;
        std::uint32_t parent;
        int link;
        Frame offset;
        int shape;
        std::string item;
    };
    std::vector<CollisionPrim> collisionPrims;  // drawn last, translucent
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
        for (std::size_t shapeIndex = 0; shapeIndex < link.shapes.size(); ++shapeIndex) {
            const Shape& shape = link.shapes[shapeIndex];
            const Frame offset = toFrame(shape.origin);
            std::optional<std::uint32_t> firstMesh;
            // Only what is drawn or collides as itself needs its own meshes; a collision-only
            // mesh (a split part) is drawn by its hull below.
            const bool drawn = shape.visual || shape.kind != Shape::Kind::Mesh;
            for (Piece& piece : drawn ? shapePieces(shape) : std::vector<Piece>{}) {
                if (piece.mesh.positions.empty()) {
                    continue;
                }
                statics->meshes.push_back(std::move(piece.mesh));
                const auto meshId = static_cast<std::uint32_t>(statics->meshes.size() - 1);
                firstMesh = firstMesh.value_or(meshId);
                if (shape.visual) {
                    PrimData prim;
                    prim.parent = linkPrimId;
                    prim.name = "visual_" + std::to_string(visualCount++);
                    prim.path = linkPrim.path + "/" + prim.name;
                    prim.typeName = "Mesh";
                    prim.mesh = meshId;
                    const bool own = hasColor(shape.rgba);
                    prim.material = addMaterial(
                        link.name + "/" + prim.name,
                        own ? shape.rgba
                            : (hasColor(piece.color) ? piece.color : kDefaultShapeColor));
                    const std::uint32_t id =
                        addPrim(std::move(prim), link.name, linkIndex, offset, -1);
                    scene.primShape[id] = static_cast<int>(shapeIndex);
                }
                if (shape.collision && shape.kind != Shape::Kind::Mesh) {
                    ColliderData collider;
                    collider.prim = linkPrimId;
                    collider.mesh = meshId;
                    const Transform pose = toTransform(offset);
                    collider.position = pose.position;
                    collider.rotation = pose.rotation;
                    statics->colliders.push_back(collider);
                }
            }
            if (!shape.collision) {
                continue;
            }
            // A mesh collides as its convex hull (the server's convexHullOf): that is the
            // collider. Drawn later, translucent, so the visuals stay seen through it.
            std::optional<MeshData> hull;
            if (shape.kind == Shape::Kind::Mesh || options.collision || !shape.visual) {
                hull = collisionMesh(shape, firstMesh, *statics);
            }
            if (!hull || hull->positions.empty()) {
                continue;
            }
            std::uint32_t meshId = firstMesh.value_or(kNone);
            if (shape.kind == Shape::Kind::Mesh) {
                statics->meshes.push_back(std::move(*hull));
                meshId = static_cast<std::uint32_t>(statics->meshes.size() - 1);
                ColliderData collider;
                collider.prim = linkPrimId;
                collider.mesh = meshId;
                const Transform pose = toTransform(offset);
                collider.position = pose.position;
                collider.rotation = pose.rotation;
                statics->colliders.push_back(collider);
            }
            // Collision-only shapes are always drawn (they are the user's: templates, fitted,
            // drawn, parts); every collision shape when the view's collision toggle is on.
            if (options.collision || !shape.visual) {
                collisionPrims.push_back({meshId, linkPrimId, linkIndex, offset,
                                          static_cast<int>(shapeIndex), link.name});
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
    // Collision geometry, translucent, after everything opaque (the Lit pipeline blends by the
    // material's alpha; drawn last, what is inside shows through).
    if (!collisionPrims.empty()) {
        const std::uint32_t material = addMaterial("collision", kCollisionColor);
        int count = 0;
        for (const CollisionPrim& c : collisionPrims) {
            PrimData prim;
            prim.parent = c.parent;
            prim.name = "collision_" + std::to_string(count++);
            prim.path = statics->prims[c.parent].path + "/" + prim.name;
            prim.typeName = "Collision";
            prim.mesh = c.mesh;
            prim.material = material;
            const std::uint32_t id = addPrim(std::move(prim), c.item, c.link, c.offset, -1);
            scene.primShape[id] = c.shape;
        }
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
