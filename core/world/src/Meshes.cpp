// Primitive meshes for the viewer, the same shapes and vertex layout as the mock server's
// meshes.py, so a robot looks the same whichever server shows it.
#include "lagrange/world/Model.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <optional>
#include <utility>

namespace lagrange {

namespace {

void addVertex(TriMesh& mesh, const std::array<double, 3>& p, const std::array<double, 3>& n) {
    for (int i = 0; i < 3; ++i) {
        mesh.positions.push_back(static_cast<float>(p.at(static_cast<std::size_t>(i))));
        mesh.normals.push_back(static_cast<float>(n.at(static_cast<std::size_t>(i))));
    }
}

std::uint32_t vertexCount(const TriMesh& mesh) {
    return static_cast<std::uint32_t>(mesh.positions.size() / 3);
}

}  // namespace

ConvexHull boxHull(const std::array<double, 3>& half) {
    ConvexHull hull;
    for (int k = 0; k < 8; ++k) {
        hull.points.push_back({(k & 1) != 0 ? half[0] : -half[0], (k & 2) != 0 ? half[1] : -half[1],
                               (k & 4) != 0 ? half[2] : -half[2]});
    }
    // Two triangles per face, wound outward (corner k has bit 0 x, bit 1 y, bit 2 z).
    hull.triangles = {{0, 2, 3}, {0, 3, 1}, {4, 5, 7}, {4, 7, 6}, {0, 1, 5}, {0, 5, 4},
                      {2, 6, 7}, {2, 7, 3}, {0, 4, 6}, {0, 6, 2}, {1, 3, 7}, {1, 7, 5}};
    return hull;
}

TriMesh boxMesh(double sizeX, double sizeY, double sizeZ) {
    const std::array<double, 3> half{sizeX / 2.0, sizeY / 2.0, sizeZ / 2.0};
    TriMesh mesh;
    for (std::size_t axis = 0; axis < 3; ++axis) {
        const std::size_t u = (axis + 1) % 3;
        const std::size_t v = (axis + 2) % 3;
        for (const double sign : {-1.0, 1.0}) {
            std::array<std::array<double, 3>, 4> corners{};
            const std::array<std::array<double, 2>, 4> steps{{{-1, -1}, {1, -1}, {1, 1}, {-1, 1}}};
            for (std::size_t c = 0; c < 4; ++c) {
                corners[c][axis] = sign * half[axis];
                corners[c][u] = steps[c][0] * half[u];
                corners[c][v] = steps[c][1] * half[v];
            }
            if (sign < 0) {
                std::swap(corners[0], corners[3]);
                std::swap(corners[1], corners[2]);
            }
            std::array<double, 3> normal{};
            normal[axis] = sign;
            // Two triangles with flat normals, one vertex per corner of each (meshes.py).
            for (const auto& triangle :
                 {std::array<std::size_t, 3>{0, 1, 2}, std::array<std::size_t, 3>{0, 2, 3}}) {
                for (const std::size_t corner : triangle) {
                    mesh.indices.push_back(vertexCount(mesh));
                    addVertex(mesh, corners[corner], normal);
                }
            }
        }
    }
    return mesh;
}

TriMesh cylinderMesh(double radius, double length, int segments) {
    TriMesh mesh;
    const double top = length / 2.0;
    const double bottom = -length / 2.0;
    const auto ring = [&](int i) {
        const double angle = 2.0 * std::numbers::pi * i / segments;
        return std::array<double, 2>{radius * std::cos(angle), radius * std::sin(angle)};
    };
    for (int i = 0; i < segments; ++i) {  // smooth side wall
        const auto [x, y] = ring(i);
        const std::array<double, 3> outward{x / radius, y / radius, 0.0};
        addVertex(mesh, {x, y, bottom}, outward);
        addVertex(mesh, {x, y, top}, outward);
        const auto a = static_cast<std::uint32_t>(2 * i);
        const auto b = static_cast<std::uint32_t>(2 * ((i + 1) % segments));
        for (const std::uint32_t index : {a, b, a + 1, a + 1, b, b + 1}) {
            mesh.indices.push_back(index);
        }
    }
    for (const auto& [z, direction] : {std::array<double, 2>{top, 1.0}, {bottom, -1.0}}) {
        const std::uint32_t centre = vertexCount(mesh);
        addVertex(mesh, {0.0, 0.0, z}, {0.0, 0.0, direction});
        for (int i = 0; i < segments; ++i) {
            const auto [x, y] = ring(i);
            addVertex(mesh, {x, y, z}, {0.0, 0.0, direction});
        }
        for (int i = 0; i < segments; ++i) {
            const std::uint32_t first = centre + 1 + static_cast<std::uint32_t>(i);
            const std::uint32_t second =
                centre + 1 + static_cast<std::uint32_t>((i + 1) % segments);
            mesh.indices.push_back(centre);
            mesh.indices.push_back(direction > 0 ? first : second);
            mesh.indices.push_back(direction > 0 ? second : first);
        }
    }
    return mesh;
}

TriMesh sphereMesh(double radius, int segments, int rings) {
    TriMesh mesh;
    for (int r = 0; r <= rings; ++r) {
        const double polar = std::numbers::pi * r / rings;
        for (int s = 0; s <= segments; ++s) {
            const double azimuth = 2.0 * std::numbers::pi * s / segments;
            const std::array<double, 3> n{std::sin(polar) * std::cos(azimuth),
                                          std::sin(polar) * std::sin(azimuth), std::cos(polar)};
            addVertex(mesh, {radius * n[0], radius * n[1], radius * n[2]}, n);
        }
    }
    for (int r = 0; r < rings; ++r) {
        for (int s = 0; s < segments; ++s) {
            const auto a = static_cast<std::uint32_t>(r * (segments + 1) + s);
            const auto b = a + static_cast<std::uint32_t>(segments + 1);
            for (const std::uint32_t index : {a, b, a + 1, a + 1, b, b + 1}) {
                mesh.indices.push_back(index);
            }
        }
    }
    return mesh;
}

void addGroundPrim(Model& model) {
    constexpr std::array<float, 4> kGroundColor{0.004F, 0.008F, 0.018F, 1.0F};  // the mock's floor
    constexpr double kGroundHalfSizeM =
        25.0;  // the visible ground; the collision plane is infinite
    constexpr double kGroundThicknessM = 0.02;
    ScenePrim ground;
    ground.name = "ground";
    ground.path = "/ground";
    ground.typeName = "Mesh";
    model.scene.meshes.push_back(
        boxMesh(2 * kGroundHalfSizeM, 2 * kGroundHalfSizeM, kGroundThicknessM));
    ground.mesh = static_cast<std::uint32_t>(model.scene.meshes.size() - 1);
    model.scene.materials.push_back({"floor", kGroundColor});
    ground.material = static_cast<std::uint32_t>(model.scene.materials.size() - 1);
    ground.offset.position = {0.0, 0.0, -kGroundThicknessM / 2.0};
    model.scene.prims.push_back(std::move(ground));
}

void appendModel(Model& model, const Model& other) {
    World& w = model.world;
    const World& o = other.world;
    const auto bodyOffset = static_cast<std::uint32_t>(w.bodies.names.size());
    // Robots: the other model's after ours (a model with bodies but no names is one robot).
    if (w.robots.empty() && !w.bodies.names.empty()) {
        w.robots.emplace_back("robot");
    }
    const auto robotOffset = static_cast<std::uint32_t>(w.robots.size());
    if (o.robots.empty() && !o.bodies.names.empty()) {
        w.robots.emplace_back("scenery");
    }
    w.robots.insert(w.robots.end(), o.robots.begin(), o.robots.end());
    const auto map = [bodyOffset](BodyId b) {
        return b == kWorldBody ? b : BodyId{b.index + bodyOffset};
    };
    for (std::size_t b = 0; b < o.bodies.names.size(); ++b) {
        w.addBody(o.bodies.names[b], map(o.bodies.parent[b]), o.bodies.poseInParent[b],
                  o.bodies.mass[b], o.bodies.comLocal[b], o.bodies.inertiaLocal[b]);
        w.bodies.robot.back() = robotOffset + o.bodies.robot[b];
        w.bodies.kinematic.back() = o.bodies.kinematic[b];
        model.bodyRestPose.push_back(other.bodyRestPose[b]);
    }
    for (std::size_t j = 0; j < o.joints.names.size(); ++j) {
        w.addJoint(o.joints.names[j], o.joints.type[j], map(o.joints.child[j]), o.joints.axis[j],
                   o.joints.actuated[j]);
        w.joints.lowerLimit.back() = o.joints.lowerLimit[j];
        w.joints.upperLimit.back() = o.joints.upperLimit[j];
        w.joints.effortLimit.back() = o.joints.effortLimit[j];
        w.joints.velocityLimit.back() = o.joints.velocityLimit[j];
        w.joints.armature.back() = o.joints.armature[j];
        w.joints.initialPosition.back() = o.joints.initialPosition[j];
    }
    for (std::size_t c = 0; c < o.colliders.body.size(); ++c) {
        if (o.colliders.body[c] == kWorldBody && o.colliders.shape[c] == ShapeType::Plane) {
            continue;
        }
        if (o.colliders.shape[c] == ShapeType::HeightField) {
            w.addHeightFieldCollider(o.heightfields.at(o.colliders.hull[c]),
                                     o.colliders.poseInBody[c], o.colliders.friction[c]);
            continue;
        }
        if (o.colliders.shape[c] == ShapeType::ConvexHull) {
            w.addHullCollider(map(o.colliders.body[c]), o.hulls.at(o.colliders.hull[c]),
                              o.colliders.poseInBody[c], o.colliders.friction[c]);
            continue;
        }
        w.addCollider(map(o.colliders.body[c]), o.colliders.shape[c], o.colliders.size[c],
                      o.colliders.poseInBody[c], o.colliders.friction[c]);
    }
    for (std::size_t s = 0; s < o.sensors.names.size(); ++s) {
        w.addSensor(o.sensors.names[s], o.sensors.type[s], map(o.sensors.body[s]),
                    o.sensors.poseInBody[s], o.sensors.rateHz[s], o.sensors.settings[s]);
    }
    // Schema instances: bodies, joints and sensors got new prims above; any other prim (a bus,
    // a battery) is added as it was.
    std::vector<PrimId> worldPrimMap(o.prims.paths.size(), PrimId{kNoIndex});
    const auto mapPrims = [&worldPrimMap](const std::vector<PrimId>& from,
                                          const std::vector<PrimId>& to, std::size_t offset) {
        for (std::size_t i = 0; i < from.size(); ++i) {
            worldPrimMap[from[i].index] = to[offset + i];
        }
    };
    mapPrims(o.bodies.prim, w.bodies.prim, w.bodies.prim.size() - o.bodies.prim.size());
    mapPrims(o.joints.prim, w.joints.prim, w.joints.prim.size() - o.joints.prim.size());
    mapPrims(o.sensors.prim, w.sensors.prim, w.sensors.prim.size() - o.sensors.prim.size());
    const auto mapPrim = [&](PrimId p) {
        if (worldPrimMap[p.index].index == kNoIndex) {
            worldPrimMap[p.index] =
                w.addPrim(o.prims.paths[p.index], o.prims.instanceNames[p.index]);
        }
        return worldPrimMap[p.index];
    };
    for (SchemaInstance instance : o.schemaInstances) {
        instance.prim = mapPrim(instance.prim);
        for (auto& [name, targets] : instance.relationships) {
            for (PrimId& target : targets) {
                target = mapPrim(target);
            }
        }
        w.schemaInstances.push_back(std::move(instance));
    }
    // Where the prims are (after the schema instances added theirs), with their bodies renumbered.
    for (std::size_t p = 0; p < worldPrimMap.size(); ++p) {
        if (worldPrimMap[p].index != kNoIndex) {
            w.prims.body[worldPrimMap[p].index] = map(o.prims.body[p]);
            w.prims.poseInBody[worldPrimMap[p].index] = o.prims.poseInBody[p];
        }
    }
    model.actuatedJoints.insert(model.actuatedJoints.end(), other.actuatedJoints.begin(),
                                other.actuatedJoints.end());
    model.notes.insert(model.notes.end(), other.notes.begin(), other.notes.end());
    model.tiles.insert(model.tiles.end(), other.tiles.begin(), other.tiles.end());

    // The scene: keep `model`'s ground slab last.
    std::vector<ScenePrim>& prims = model.scene.prims;
    std::optional<ScenePrim> ground;
    if (!prims.empty() && prims.back().name == "ground" && prims.back().parent == kNoIndex) {
        ground = prims.back();
        prims.pop_back();
    }
    const auto meshOffset = static_cast<std::uint32_t>(model.scene.meshes.size());
    const auto materialOffset = static_cast<std::uint32_t>(model.scene.materials.size());
    model.scene.meshes.insert(model.scene.meshes.end(), other.scene.meshes.begin(),
                              other.scene.meshes.end());
    model.scene.materials.insert(model.scene.materials.end(), other.scene.materials.begin(),
                                 other.scene.materials.end());
    std::vector<std::uint32_t> primMap(other.scene.prims.size(), kNoIndex);
    for (std::size_t p = 0; p < other.scene.prims.size(); ++p) {
        ScenePrim prim = other.scene.prims[p];
        if (prim.parent == kNoIndex && prim.name == "ground" && prim.body == kNoIndex) {
            continue;
        }
        if (prim.parent != kNoIndex) {
            prim.parent = primMap[prim.parent];
        }
        prim.body = prim.body == kNoIndex ? kNoIndex : prim.body + bodyOffset;
        prim.mesh = prim.mesh == kNoIndex ? kNoIndex : prim.mesh + meshOffset;
        prim.material = prim.material == kNoIndex ? kNoIndex : prim.material + materialOffset;
        prims.push_back(std::move(prim));
        primMap[p] = static_cast<std::uint32_t>(prims.size() - 1);
    }
    for (const std::uint32_t p : other.bodyPrim) {
        model.bodyPrim.push_back(p == kNoIndex ? kNoIndex : primMap[p]);
    }
    for (const std::uint32_t p : other.jointPrim) {
        model.jointPrim.push_back(p == kNoIndex ? kNoIndex : primMap[p]);
    }
    for (const std::uint32_t p : other.sensorPrim) {
        model.sensorPrim.push_back(p == kNoIndex ? kNoIndex : primMap[p]);
    }
    if (ground && other.groundPlane) {
        prims.push_back(std::move(*ground));
    }
    if (!other.groundPlane) {
        // The joining world brings its own ground: no infinite plane under its terrain.
        World::Colliders& c = w.colliders;
        for (std::size_t i = c.body.size(); i-- > 0;) {
            if (c.body[i] == kWorldBody && c.shape[i] == ShapeType::Plane) {
                const auto at = static_cast<std::ptrdiff_t>(i);
                c.body.erase(c.body.begin() + at);
                c.shape.erase(c.shape.begin() + at);
                c.size.erase(c.size.begin() + at);
                c.poseInBody.erase(c.poseInBody.begin() + at);
                c.friction.erase(c.friction.begin() + at);
                c.hull.erase(c.hull.begin() + at);
            }
        }
        model.groundPlane = false;
    }
}

void prefixModel(Model& model, const std::string& prefix, bool passive) {
    World& w = model.world;
    const std::string front = prefix + "_";
    for (std::string& name : w.bodies.names) {
        name = front + name;
    }
    for (std::string& name : w.joints.names) {
        name = front + name;
    }
    for (std::string& name : w.sensors.names) {
        name = front + name;
    }
    for (std::string& name : model.actuatedJoints) {
        name = front + name;
    }
    if (passive) {
        std::fill(w.joints.actuated.begin(), w.joints.actuated.end(), false);
        model.actuatedJoints.clear();
    }
    // Paths: the top prim of each tree is renamed, the rest follows.
    const auto rename = [&prefix](std::string& path) {
        const std::size_t second = path.find('/', 1);
        path = "/" + prefix + (second == std::string::npos ? "" : path.substr(second));
    };
    for (ScenePrim& prim : model.scene.prims) {
        if (prim.name == "ground" && prim.parent == kNoIndex) {
            continue;
        }
        rename(prim.path);
        if (prim.parent == kNoIndex) {
            prim.name = prefix;
        }
    }
    for (std::size_t p = 0; p < w.prims.paths.size(); ++p) {
        rename(w.prims.paths[p]);
    }
    for (std::size_t b = 0; b < w.bodies.prim.size(); ++b) {
        if (w.bodies.prim[b].index < w.prims.instanceNames.size()) {
            w.prims.instanceNames[w.bodies.prim[b].index] = w.bodies.names[b];
        }
    }
    for (std::size_t j = 0; j < w.joints.prim.size(); ++j) {
        if (w.joints.prim[j].index < w.prims.instanceNames.size()) {
            w.prims.instanceNames[w.joints.prim[j].index] = w.joints.names[j];
        }
    }
    for (std::size_t s = 0; s < w.sensors.prim.size(); ++s) {
        if (w.sensors.prim[s].index < w.prims.instanceNames.size()) {
            w.prims.instanceNames[w.sensors.prim[s].index] = w.sensors.names[s];
        }
    }
    if (w.robots.size() <= 1) {
        w.robots = {prefix};
    }
    model.name = prefix;
}

ModelExtent extentOf(const Model& model) {
    return {model.world.colliders.body.size(), model.world.hulls.size(),
            model.world.heightfields.size(),   model.scene.prims.size(),
            model.scene.meshes.size(),         model.scene.materials.size()};
}

void truncateModel(Model& model, const ModelExtent& extent) {
    World::Colliders& c = model.world.colliders;
    c.body.resize(extent.colliders);
    c.shape.resize(extent.colliders);
    c.size.resize(extent.colliders);
    c.poseInBody.resize(extent.colliders);
    c.friction.resize(extent.colliders);
    c.hull.resize(extent.colliders);
    model.world.hulls.resize(extent.hulls);
    model.world.heightfields.resize(extent.heightfields);
    model.scene.prims.resize(extent.prims);
    model.scene.meshes.resize(extent.meshes);
    model.scene.materials.resize(extent.materials);
}

void appendTile(Model& model, const Model& tile, const std::string& tilePath,
                const std::array<double, 3>& regionOrigin,
                std::span<const std::array<double, 4>> covered) {
    const auto isCovered = [covered](const Pose& world) {
        return std::any_of(covered.begin(), covered.end(), [&world](const auto& box) {
            return world.position[0] >= box[0] && world.position[1] >= box[1] &&
                   world.position[0] <= box[2] && world.position[1] <= box[3];
        });
    };
    const auto local = [&regionOrigin](Pose pose) {
        for (std::size_t i = 0; i < 3; ++i) {
            pose.position.at(i) -= regionOrigin.at(i);
        }
        return pose;
    };
    World& w = model.world;
    const World& o = tile.world;
    for (std::size_t c = 0; c < o.colliders.body.size(); ++c) {
        // A tile's own ground plane is the world's, which is there already.
        if (o.colliders.body[c] != kWorldBody || o.colliders.shape[c] == ShapeType::Plane) {
            continue;
        }
        if (o.colliders.shape[c] == ShapeType::HeightField) {
            w.addHeightFieldCollider(o.heightfields.at(o.colliders.hull[c]),
                                     local(o.colliders.poseInBody[c]), o.colliders.friction[c]);
            continue;
        }
        if (o.colliders.shape[c] == ShapeType::ConvexHull) {
            w.addHullCollider(kWorldBody, o.hulls.at(o.colliders.hull[c]),
                              local(o.colliders.poseInBody[c]), o.colliders.friction[c]);
        } else {
            w.addCollider(kWorldBody, o.colliders.shape[c], o.colliders.size[c],
                          local(o.colliders.poseInBody[c]), o.colliders.friction[c]);
        }
    }

    std::uint32_t anchor = kNoIndex;  // the tile's prim in `model`
    for (std::uint32_t p = 0; p < model.scene.prims.size() && anchor == kNoIndex; ++p) {
        if (model.scene.prims[p].path == tilePath) {
            anchor = p;
        }
    }
    // The tile prim itself is not in the scene while its payload is unloaded: it comes with
    // the tile, under its parent prim.
    if (anchor == kNoIndex) {
        for (const ScenePrim& from : tile.scene.prims) {
            if (from.path != tilePath) {
                continue;
            }
            ScenePrim prim = from;
            prim.mesh = kNoIndex;
            prim.material = kNoIndex;
            prim.offset = local(from.offset);
            prim.parent = kNoIndex;
            const std::string parentPath = tilePath.substr(0, tilePath.rfind('/'));
            for (std::uint32_t p = 0; p < model.scene.prims.size(); ++p) {
                if (model.scene.prims[p].path == parentPath) {
                    prim.parent = p;
                }
            }
            model.scene.prims.push_back(std::move(prim));
            anchor = static_cast<std::uint32_t>(model.scene.prims.size() - 1);
        }
    }
    const auto meshOffset = static_cast<std::uint32_t>(model.scene.meshes.size());
    const auto materialOffset = static_cast<std::uint32_t>(model.scene.materials.size());
    model.scene.meshes.insert(model.scene.meshes.end(), tile.scene.meshes.begin(),
                              tile.scene.meshes.end());
    model.scene.materials.insert(model.scene.materials.end(), tile.scene.materials.begin(),
                                 tile.scene.materials.end());
    const std::string prefix = tilePath + "/";
    std::vector<std::uint32_t> primMap(tile.scene.prims.size(), kNoIndex);
    for (std::size_t p = 0; p < tile.scene.prims.size(); ++p) {
        const ScenePrim& from = tile.scene.prims[p];
        if (from.path == tilePath) {
            primMap[p] = anchor;  // a mesh on the tile prim itself is left out
            continue;
        }
        if (from.path.rfind(prefix, 0) != 0 || from.body != kNoIndex) {
            continue;  // the tile's ancestors (in `model` already), or not scenery
        }
        // Under a loaded finer tile; its children go with it (their parent is not mapped).
        if ((from.parent != kNoIndex && primMap[from.parent] == kNoIndex) ||
            isCovered(from.offset)) {
            continue;
        }
        ScenePrim prim = from;
        prim.parent = from.parent == kNoIndex ? anchor : primMap[from.parent];
        prim.offset = local(from.offset);
        prim.mesh = from.mesh == kNoIndex ? kNoIndex : from.mesh + meshOffset;
        prim.material = from.material == kNoIndex ? kNoIndex : from.material + materialOffset;
        model.scene.prims.push_back(std::move(prim));
        primMap[p] = static_cast<std::uint32_t>(model.scene.prims.size() - 1);
    }
}

}  // namespace lagrange
