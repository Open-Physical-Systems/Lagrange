// The primitive shapes as triangle meshes for the Builder view (plan decision 1, R14).
//
// The tessellation is core/world/src/Meshes.cpp's (boxMesh, cylinderMesh, sphereMesh), copied
// because the viewer may not link lagrange_core (invariant 14): the robot looks the same in the
// builder and in the run it is tested in. Primitives.SameVertexCountsAsCore pins the counts.
#include "lagrange/viewer/builder/Primitives.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <utility>

namespace lagrange::viewer::builder {

namespace {

std::uint32_t vertexCount(const MeshData& mesh) {
    return static_cast<std::uint32_t>(mesh.positions.size() / 3);
}

void addVertex(MeshData& mesh, const std::array<double, 3>& p, const std::array<double, 3>& n) {
    for (std::size_t k = 0; k < 3; ++k) {
        mesh.positions.push_back(static_cast<float>(p.at(k)));
        mesh.normals.push_back(static_cast<float>(n.at(k)));
    }
}

/// Fills mesh.aabb (min x y z, max x y z) from its positions.
void fillAabb(MeshData& mesh) {
    if (mesh.positions.empty()) {
        mesh.aabb = {};
        return;
    }
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

}  // namespace

MeshData boxMesh(const std::array<double, 3>& size) {
    const std::array<double, 3> half{size[0] / 2.0, size[1] / 2.0, size[2] / 2.0};
    MeshData mesh;
    for (std::size_t axis = 0; axis < 3; ++axis) {
        const std::size_t u = (axis + 1) % 3;
        const std::size_t v = (axis + 2) % 3;
        for (const double sign : {-1.0, 1.0}) {
            std::array<std::array<double, 3>, 4> corners{};
            const std::array<std::array<double, 2>, 4> steps{{{-1, -1}, {1, -1}, {1, 1}, {-1, 1}}};
            for (std::size_t c = 0; c < 4; ++c) {
                corners.at(c).at(axis) = sign * half.at(axis);
                corners.at(c).at(u) = steps.at(c)[0] * half.at(u);
                corners.at(c).at(v) = steps.at(c)[1] * half.at(v);
            }
            if (sign < 0) {
                std::swap(corners[0], corners[3]);
                std::swap(corners[1], corners[2]);
            }
            std::array<double, 3> normal{};
            normal.at(axis) = sign;
            // Two triangles with flat normals, one vertex per corner of each.
            for (const auto& triangle :
                 {std::array<std::size_t, 3>{0, 1, 2}, std::array<std::size_t, 3>{0, 2, 3}}) {
                for (const std::size_t corner : triangle) {
                    mesh.indices.push_back(vertexCount(mesh));
                    addVertex(mesh, corners.at(corner), normal);
                }
            }
        }
    }
    fillAabb(mesh);
    return mesh;
}

MeshData cylinderMesh(double radius, double length, int segments) {
    MeshData mesh;
    if (segments < 3 || radius <= 0.0) {
        return mesh;
    }
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
    fillAabb(mesh);
    return mesh;
}

MeshData sphereMesh(double radius, int segments, int rings) {
    MeshData mesh;
    if (segments < 3 || rings < 2 || radius <= 0.0) {
        return mesh;
    }
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
    fillAabb(mesh);
    return mesh;
}

}  // namespace lagrange::viewer::builder
