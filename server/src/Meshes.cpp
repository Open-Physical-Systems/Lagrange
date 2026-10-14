// Primitive meshes for the viewer, the same shapes and vertex layout as the mock server's
// meshes.py, so a robot looks the same whichever server shows it.
#include "lagrange/server/Robot.hpp"

#include <array>
#include <cmath>
#include <numbers>

namespace lagrange::server {

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

}  // namespace lagrange::server
