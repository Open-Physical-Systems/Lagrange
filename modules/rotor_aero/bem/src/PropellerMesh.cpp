// Propellers drawn from their files (Propeller.hpp): blade meshes, and on a loaded model.
#include "Stations.hpp"
#include "lagrange/rotor_aero/Propeller.hpp"
#include "lagrange/world/Model.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

namespace lagrange::rotor_aero {

namespace {

constexpr int kRadialSections = 24;  // blade sections from root to tip
constexpr int kSectionPoints = 12;   // points per surface, leading to trailing edge
// Hub: a cylinder as wide as the blade root and as thick as a fraction of its chord.
constexpr double kHubThicknessOfRootChord = 0.6;

using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>;  // w, x, y, z

Vec3 sub(const Vec3& a, const Vec3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
double dot(const Vec3& a, const Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/// NACA four-digit section at chord fraction x (0 leading edge, 1 trailing edge): half
/// thickness and mean line height, as fractions of the chord.
std::array<double, 2> naca4(double x, double thickness, double camber, double position) {
    const double half = 5.0 * thickness *
                        (0.2969 * std::sqrt(x) - 0.1260 * x - 0.3516 * x * x + 0.2843 * x * x * x -
                         0.1015 * x * x * x * x);
    double mean = 0.0;
    if (camber > 0.0 && position > 0.0 && position < 1.0) {
        mean = x < position ? camber / (position * position) * (2.0 * position * x - x * x)
                            : camber / ((1.0 - position) * (1.0 - position)) *
                                  (1.0 - 2.0 * position + 2.0 * position * x - x * x);
    }
    return {half, mean};
}

void append(TriMesh& to, const TriMesh& from) {
    const auto base = static_cast<std::uint32_t>(to.positions.size() / 3);
    to.positions.insert(to.positions.end(), from.positions.begin(), from.positions.end());
    to.normals.insert(to.normals.end(), from.normals.begin(), from.normals.end());
    for (const std::uint32_t i : from.indices) {
        to.indices.push_back(base + i);
    }
}

/// One blade along the direction at angle `psi` about +z, from radius fraction `from` to `to`,
/// moved `shift` (m) towards the axis.
TriMesh blade(const Propeller& p, double psi, double from, double to, double shift = 0.0) {
    const double hand = p.hand == "cw" ? -1.0 : 1.0;
    const Vec3 radial{std::cos(psi), std::sin(psi), 0.0};
    // The direction the blade moves in: about +z in the propeller's hand.
    const Vec3 ahead{-hand * std::sin(psi), hand * std::cos(psi), 0.0};
    std::vector<Vec3> points;
    const int ring = 2 * kSectionPoints;
    for (int i = 0; i <= kRadialSections; ++i) {
        const double x = from + (to - from) * i / kRadialSections;
        const BladeStation s = stationAt(p.stations, x);
        const double r = x * p.radius - shift;
        const double c = s.chord * p.radius;
        // Chord towards the leading edge, and the side the camber faces (the front).
        const double cb = std::cos(s.twist);
        const double sb = std::sin(s.twist);
        for (int k = 0; k < ring; ++k) {
            // Around the section: upper surface trailing to leading edge, then lower back.
            const bool upper = k < kSectionPoints;
            const int j = upper ? kSectionPoints - 1 - k : k - kSectionPoints;
            const double u = 0.5 * (1.0 - std::cos(std::numbers::pi * j / (kSectionPoints - 1)));
            const auto [half, mean] =
                naca4(u, s.thickness, s.aerofoil.camber, s.aerofoil.camberPosition);
            const double along = (0.25 - u) * c;  // from the quarter chord towards the edge
            const double normal = (mean + (upper ? half : -half)) * c;
            const double e = along * cb - normal * sb;
            const double t = along * sb + normal * cb;
            points.push_back({r * radial[0] + e * ahead[0], r * radial[1] + e * ahead[1], t});
        }
    }
    std::vector<std::array<std::uint32_t, 3>> triangles;
    for (int i = 0; i < kRadialSections; ++i) {
        for (int k = 0; k < ring; ++k) {
            const auto a = static_cast<std::uint32_t>(i * ring + k);
            const auto b = static_cast<std::uint32_t>(i * ring + (k + 1) % ring);
            triangles.push_back({a, b, a + ring});
            triangles.push_back({b, b + ring, a + ring});
        }
    }
    // Root and tip caps, fanned from their centres.
    for (const int i : {0, kRadialSections}) {
        Vec3 centre{};
        for (int k = 0; k < ring; ++k) {
            for (int d = 0; d < 3; ++d) {
                centre[d] += points[i * ring + k][d] / ring;
            }
        }
        const auto c = static_cast<std::uint32_t>(points.size());
        points.push_back(centre);
        for (int k = 0; k < ring; ++k) {
            const auto a = static_cast<std::uint32_t>(i * ring + k);
            const auto b = static_cast<std::uint32_t>(i * ring + (k + 1) % ring);
            triangles.push_back(i == 0 ? std::array{c, b, a} : std::array{c, a, b});
        }
    }
    // Outward winding whatever the hand (a mirrored blade turns it inside out): positive volume.
    double volume = 0.0;
    for (const auto& t : triangles) {
        volume += dot(points[t[0]], cross(points[t[1]], points[t[2]]));
    }
    if (volume < 0.0) {
        for (auto& t : triangles) {
            std::swap(t[1], t[2]);
        }
    }
    TriMesh m;
    std::vector<Vec3> normals(points.size(), Vec3{});
    for (const auto& t : triangles) {
        const Vec3 n = cross(sub(points[t[1]], points[t[0]]), sub(points[t[2]], points[t[0]]));
        for (const std::uint32_t v : t) {
            for (int d = 0; d < 3; ++d) {
                normals[v][d] += n[d];
            }
        }
        m.indices.insert(m.indices.end(), t.begin(), t.end());
    }
    for (std::size_t v = 0; v < points.size(); ++v) {
        const double len = std::sqrt(dot(normals[v], normals[v]));
        for (int d = 0; d < 3; ++d) {
            m.positions.push_back(static_cast<float>(points[v][d]));
            m.normals.push_back(static_cast<float>(len > 0.0 ? normals[v][d] / len : 0.0));
        }
    }
    return m;
}

/// The rotation taking +z to the unit vector `to`.
Quat fromZ(const Vec3& to) {
    if (to[2] < -1.0 + 1e-12) {
        return {0.0, 1.0, 0.0, 0.0};  // half a turn about x
    }
    // Half-way quaternion: (1 + z·t, z × t), normalised.
    Quat q{1.0 + to[2], -to[1], to[0], 0.0};
    const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2]);
    return {q[0] / n, q[1] / n, q[2] / n, 0.0};
}

}  // namespace

TriMesh propellerMesh(const Propeller& p) {
    TriMesh mesh;
    if (p.stations.empty() || p.blades < 1) {
        return mesh;
    }
    for (int b = 0; b < p.blades; ++b) {
        append(mesh,
               blade(p, 2.0 * std::numbers::pi * b / p.blades, p.stations.front().radius, 1.0));
    }
    const BladeStation& root = p.stations.front();
    append(mesh, cylinderMesh(root.radius * p.radius,
                              kHubThicknessOfRootChord * root.chord * p.radius, 24));
    return mesh;
}

TriMesh foldingHubMesh(const Propeller& p) {
    TriMesh mesh;
    if (p.stations.empty() || !p.folding) {
        return mesh;
    }
    const double root = p.stations.front().radius;
    for (int b = 0; b < p.blades; ++b) {
        append(mesh, blade(p, 2.0 * std::numbers::pi * b / p.blades, root,
                           std::max(p.folding->hinge, root)));
    }
    const BladeStation& s = p.stations.front();
    append(mesh, cylinderMesh(root * p.radius, kHubThicknessOfRootChord * s.chord * p.radius, 24));
    return mesh;
}

TriMesh foldingBladeMesh(const Propeller& p) {
    if (p.stations.empty() || !p.folding) {
        return {};
    }
    const double hinge = std::max(p.folding->hinge, p.stations.front().radius);
    return blade(p, 0.0, hinge, 1.0, hinge * p.radius);
}

void addPropellerBlades(Model& model) {
    const WorldView world(model.world);
    const World::Joints& joints = model.world.joints;
    for (const SchemaInstance* s : world.instancesOf("RotorAPI_1")) {
        const auto file = s->assetPath("aero:rotor:propeller");
        const auto j =
            static_cast<std::size_t>(std::ranges::find(joints.prim, s->prim) - joints.prim.begin());
        if (!file || j >= joints.prim.size()) {
            continue;
        }
        std::string error;
        const Propeller p = loadPropeller(std::string(*file), error);
        if (!error.empty()) {
            model.notes.push_back("propeller not drawn: " + error);
            continue;
        }
        const std::uint32_t body = joints.child[j].index;
        // The link's own visuals give way to the propeller its file describes.
        std::uint32_t material = kNoIndex;
        for (ScenePrim& prim : model.scene.prims) {
            if (prim.body == body && prim.mesh != kNoIndex) {
                material = prim.material;
                prim.mesh = kNoIndex;
                prim.typeName = "Xform";
            }
        }
        const double mount =
            s->token("aero:rotor:mount").value_or("puller") == "pusher" ? -1.0 : 1.0;
        const Vec3& a = joints.axis[j];
        const double n = std::sqrt(dot(a, a));
        ScenePrim prim;
        prim.name = "propeller";
        prim.typeName = "Mesh";
        prim.body = body;
        prim.parent = body < model.bodyPrim.size() ? model.bodyPrim[body] : kNoIndex;
        prim.path =
            (prim.parent != kNoIndex ? model.scene.prims[prim.parent].path : std::string()) +
            "/propeller";
        prim.offset.orientation = fromZ({mount * a[0] / n, mount * a[1] / n, mount * a[2] / n});
        prim.material = material;
        prim.mesh = static_cast<std::uint32_t>(model.scene.meshes.size());
        if (!p.folding) {
            model.scene.meshes.push_back(propellerMesh(p));
            model.scene.prims.push_back(std::move(prim));
            continue;
        }
        // Folding: the hub, and each blade on its hinge, turned by the rotor's blade angle.
        const std::string joint(world.signalInstanceName(s->prim));
        const Quat mounted = prim.offset.orientation;
        model.scene.meshes.push_back(foldingHubMesh(p));
        const auto bladeMesh = static_cast<std::uint32_t>(model.scene.meshes.size());
        model.scene.meshes.push_back(foldingBladeMesh(p));
        const std::string path = prim.path;
        model.scene.prims.push_back(prim);
        for (int b = 0; b < p.blades; ++b) {
            const double psi = 2.0 * std::numbers::pi * b / p.blades;
            const double hinge = std::max(p.folding->hinge, p.stations.front().radius) * p.radius;
            ScenePrim part = prim;
            part.name = "blade_" + std::to_string(b + 1);
            part.path = path + "_" + part.name;
            part.mesh = bladeMesh;
            // At its hinge, along its azimuth, in the propeller's frame; it folds about the
            // tangent: Ry(−β) lifts its tip towards the thrust direction.
            const Quat azimuth{std::cos(0.5 * psi), 0.0, 0.0, std::sin(0.5 * psi)};
            part.offset.orientation = multiply(mounted, azimuth);
            part.offset.position =
                rotate(mounted, {hinge * std::cos(psi), hinge * std::sin(psi), 0.0});
            part.hinge = "rotor/" + joint + "/blade_angle";
            part.hingeAxis = {0.0, -1.0, 0.0};
            model.scene.prims.push_back(std::move(part));
        }
    }
}

}  // namespace lagrange::rotor_aero
