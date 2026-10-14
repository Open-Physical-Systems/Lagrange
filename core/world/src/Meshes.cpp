// Primitive meshes for the viewer, the same shapes and vertex layout as the mock server's
// meshes.py, so a robot looks the same whichever server shows it.
#include "lagrange/world/Model.hpp"

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
    const auto map = [bodyOffset](BodyId b) {
        return b == kWorldBody ? b : BodyId{b.index + bodyOffset};
    };
    for (std::size_t b = 0; b < o.bodies.names.size(); ++b) {
        w.addBody(o.bodies.names[b], map(o.bodies.parent[b]), o.bodies.poseInParent[b],
                  o.bodies.mass[b], o.bodies.comLocal[b], o.bodies.inertiaLocal[b]);
        model.bodyRestPose.push_back(other.bodyRestPose[b]);
    }
    for (std::size_t j = 0; j < o.joints.names.size(); ++j) {
        w.addJoint(o.joints.names[j], o.joints.type[j], map(o.joints.child[j]), o.joints.axis[j],
                   o.joints.actuated[j]);
        w.joints.lowerLimit.back() = o.joints.lowerLimit[j];
        w.joints.upperLimit.back() = o.joints.upperLimit[j];
        w.joints.effortLimit.back() = o.joints.effortLimit[j];
        w.joints.velocityLimit.back() = o.joints.velocityLimit[j];
    }
    for (std::size_t c = 0; c < o.colliders.body.size(); ++c) {
        if (o.colliders.body[c] == kWorldBody && o.colliders.shape[c] == ShapeType::Plane) {
            continue;
        }
        w.addCollider(map(o.colliders.body[c]), o.colliders.shape[c], o.colliders.size[c],
                      o.colliders.poseInBody[c], o.colliders.friction[c]);
    }
    for (std::size_t s = 0; s < o.sensors.names.size(); ++s) {
        w.addSensor(o.sensors.names[s], o.sensors.type[s], map(o.sensors.body[s]),
                    o.sensors.poseInBody[s], o.sensors.rateHz[s], o.sensors.settings[s]);
    }
    model.actuatedJoints.insert(model.actuatedJoints.end(), other.actuatedJoints.begin(),
                                other.actuatedJoints.end());
    model.notes.insert(model.notes.end(), other.notes.begin(), other.notes.end());

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
    if (ground) {
        prims.push_back(std::move(*ground));
    }
}

}  // namespace lagrange
