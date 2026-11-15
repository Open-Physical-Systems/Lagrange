// Mesh shapes of the builder (RFC 0016 Addendum A). See MeshShapes.hpp.
//
// lagrange_meshes reads the file exactly as the server does (server/src/Robot.cpp,
// addMeshCollider: every part's vertices into one TriMesh, then convexHullOf and hullFill), so
// the hull drawn here is the hull the server collides.
#include "Math.hpp"
#include "lagrange/meshes/MeshImport.hpp"
#include "lagrange/viewer/builder/Kinematics.hpp"
#include "lagrange/viewer/builder/MeshShapes.hpp"
#include "lagrange/world/Model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <mutex>
#include <numbers>
#include <unordered_map>
#include <utility>

namespace lagrange::viewer::builder {

namespace {

using detail::Mat3;
using detail::Vec3;

Vec3 sub(const Vec3& a, const Vec3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
double dot(const Vec3& a, const Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

// --- the cache of files ---------------------------------------------------------------------

struct Entry {
    std::filesystem::file_time_type time;
    std::shared_ptr<const MeshFile> mesh;
};

std::mutex& cacheMutex() {
    static std::mutex mutex;
    return mutex;
}
std::map<std::string, Entry>& cache() {
    static std::map<std::string, Entry> entries;
    return entries;
}
std::filesystem::path& baseDirectory() {
    static std::filesystem::path base;
    return base;
}

/// Vertices closer than this share of the mesh's size are one (float32 files: a seam's
/// cos(2π) and cos(0) differ in the last bits). About twice float32's epsilon.
constexpr double kWeldShare = 2.5e-7;

/// Welds equal vertices and tells whether the surface is closed: each directed edge appears
/// once and its reverse once (a closed, consistently wound 2-manifold).
void weld(MeshFile& mesh) {
    double size = 0.0;
    for (const float v : mesh.positions) {
        size = std::max(size, std::abs(static_cast<double>(v)));
    }
    const double cell = size > 0.0 ? size * kWeldShare : 1.0;
    std::map<std::array<long long, 3>, std::uint32_t> index;
    std::vector<std::uint32_t> remap(mesh.positions.size() / 3);
    for (std::size_t v = 0; v < remap.size(); ++v) {
        const std::array<float, 3> p{mesh.positions[3 * v], mesh.positions[3 * v + 1],
                                     mesh.positions[3 * v + 2]};
        const std::array<long long, 3> key{std::llround(p[0] / cell), std::llround(p[1] / cell),
                                           std::llround(p[2] / cell)};
        const auto [it, added] =
            index.emplace(key, static_cast<std::uint32_t>(mesh.welded.points.size()));
        if (added) {
            mesh.welded.points.push_back({p[0], p[1], p[2]});
        }
        remap[v] = it->second;
    }
    std::unordered_map<std::uint64_t, int> edges;
    const auto key = [](std::uint32_t a, std::uint32_t b) {
        return (static_cast<std::uint64_t>(a) << 32U) | b;
    };
    for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        const std::array<std::uint32_t, 3> t{remap[mesh.indices[i]], remap[mesh.indices[i + 1]],
                                             remap[mesh.indices[i + 2]]};
        if (t[0] == t[1] || t[1] == t[2] || t[0] == t[2]) {
            continue;  // degenerate after welding: no area, no edge
        }
        mesh.welded.triangles.push_back(t);
        for (std::size_t e = 0; e < 3; ++e) {
            ++edges[key(t.at(e), t.at((e + 1) % 3))];
        }
    }
    bool closed = !mesh.welded.triangles.empty();
    for (const auto& [edge, count] : edges) {
        const auto a = static_cast<std::uint32_t>(edge >> 32U);
        const auto b = static_cast<std::uint32_t>(edge & 0xFFFF'FFFFU);
        const auto reverse = edges.find(key(b, a));
        if (count != 1 || reverse == edges.end() || reverse->second != 1) {
            closed = false;
            break;
        }
    }
    mesh.closed = closed;
}

std::shared_ptr<const MeshFile> failed(std::string path, std::string error) {
    auto mesh = std::make_shared<MeshFile>();
    mesh->path = std::move(path);
    mesh->error = std::move(error);
    return mesh;
}

std::shared_ptr<const MeshFile> read(const std::string& path) {
    auto imported = meshes::importMesh(path);
    if (!imported) {
        return failed(path, imported.error());
    }
    auto mesh = std::make_shared<MeshFile>();
    mesh->path = path;
    mesh->notes = imported->notes;
    for (const meshes::MeshPart& part : imported->parts) {
        // The server's merge (Robot.cpp addMeshCollider), for convexHullOf and hullFill.
        const auto first = static_cast<std::uint32_t>(mesh->positions.size() / 3);
        mesh->positions.insert(mesh->positions.end(), part.positions.begin(), part.positions.end());
        for (const std::uint32_t index : part.indices) {
            mesh->indices.push_back(first + index);
        }
        MeshData data;
        data.positions = part.positions;
        data.normals = part.normals;
        data.indices = part.indices;
        mesh->parts.push_back(std::move(data));
        mesh->colors.push_back(part.hasColor ? part.rgba : std::array<float, 4>{});
    }
    weld(*mesh);
    return mesh;
}

// --- Mirtich 1996 ----------------------------------------------------------------------------

/// The integrals of 1, x, y, z, x², y², z², xy, yz, zx over the solid (Mirtich's T0, T1, T2, TP),
/// summed face by face from the projection integrals over each face's edges.
struct VolumeIntegrals {
    double t0{0};
    Vec3 t1{};
    Vec3 t2{};
    Vec3 tp{};  // xy, yz, zx
};

VolumeIntegrals integrate(const Surface& surface) {
    VolumeIntegrals out;
    for (const auto& tri : surface.triangles) {
        const std::array<Vec3, 3> v{surface.points[tri[0]], surface.points[tri[1]],
                                    surface.points[tri[2]]};
        Vec3 n = cross(sub(v[1], v[0]), sub(v[2], v[0]));
        const double length = std::sqrt(dot(n, n));
        if (length <= 0.0) {
            continue;
        }
        n = {n[0] / length, n[1] / length, n[2] / length};
        // C: the axis the face is most nearly perpendicular to; A, B the projection plane.
        std::size_t c = 0;
        if (std::abs(n[1]) > std::abs(n[c])) {
            c = 1;
        }
        if (std::abs(n[2]) > std::abs(n[c])) {
            c = 2;
        }
        const std::size_t a = (c + 1) % 3;
        const std::size_t b = (a + 1) % 3;
        // Projection integrals (Mirtich §4.3): over the face's projection onto the A-B plane.
        double p1 = 0, pa = 0, pb = 0, paa = 0, pab = 0, pbb = 0, paaa = 0, paab = 0, pabb = 0,
               pbbb = 0;
        for (std::size_t e = 0; e < 3; ++e) {
            const double a0 = v.at(e).at(a);
            const double b0 = v.at(e).at(b);
            const double a1 = v.at((e + 1) % 3).at(a);
            const double b1 = v.at((e + 1) % 3).at(b);
            const double da = a1 - a0;
            const double db = b1 - b0;
            const double a02 = a0 * a0, a03 = a02 * a0, a04 = a03 * a0;
            const double b02 = b0 * b0, b03 = b02 * b0, b04 = b03 * b0;
            const double a12 = a1 * a1, a13 = a12 * a1;
            const double b12 = b1 * b1, b13 = b12 * b1;
            const double c1 = a1 + a0;
            const double ca = a1 * c1 + a02;
            const double caa = a1 * ca + a03;
            const double caaa = a1 * caa + a04;
            const double cb = b1 * (b1 + b0) + b02;
            const double cbb = b1 * cb + b03;
            const double cbbb = b1 * cbb + b04;
            const double cab = 3 * a12 + 2 * a1 * a0 + a02;
            const double kab = a12 + 2 * a1 * a0 + 3 * a02;
            const double caab = a0 * cab + 4 * a13;
            const double kaab = a1 * kab + 4 * a03;
            const double cabb = 4 * b13 + 3 * b12 * b0 + 2 * b1 * b02 + b03;
            const double kabb = b13 + 2 * b12 * b0 + 3 * b1 * b02 + 4 * b03;
            p1 += db * c1;
            pa += db * ca;
            paa += db * caa;
            paaa += db * caaa;
            pb += da * cb;
            pbb += da * cbb;
            pbbb += da * cbbb;
            pab += db * (b1 * cab + b0 * kab);
            paab += db * (b1 * caab + b0 * kaab);
            pabb += da * (a1 * cabb + a0 * kabb);
        }
        p1 /= 2.0;
        pa /= 6.0;
        paa /= 12.0;
        paaa /= 20.0;
        pb /= -6.0;
        pbb /= -12.0;
        pbbb /= -20.0;
        pab /= 24.0;
        paab /= 60.0;
        pabb /= -60.0;
        // Face integrals (§4.2): the projection integrals lifted onto the face's plane.
        const double w = -dot(n, v[0]);
        const double na = n.at(a);
        const double nb = n.at(b);
        const double k1 = 1.0 / n.at(c);
        const double k2 = k1 * k1;
        const double k3 = k2 * k1;
        const double k4 = k3 * k1;
        const double fa = k1 * pa;
        const double fb = k1 * pb;
        const double fc = -k2 * (na * pa + nb * pb + w * p1);
        const double faa = k1 * paa;
        const double fbb = k1 * pbb;
        const double fcc = k3 * (na * na * paa + 2 * na * nb * pab + nb * nb * pbb +
                                 w * (2 * (na * pa + nb * pb) + w * p1));
        const double faaa = k1 * paaa;
        const double fbbb = k1 * pbbb;
        const double fccc = -k4 * (na * na * na * paaa + 3 * na * na * nb * paab +
                                   3 * na * nb * nb * pabb + nb * nb * nb * pbbb +
                                   3 * w * (na * na * paa + 2 * na * nb * pab + nb * nb * pbb) +
                                   w * w * (3 * (na * pa + nb * pb) + w * p1));
        const double faab = k1 * paab;
        const double fbbc = -k2 * (na * pabb + nb * pbbb + w * pbb);
        const double fcca = k3 * (na * na * paaa + 2 * na * nb * paab + nb * nb * pabb +
                                  w * (2 * (na * paa + nb * pab) + w * pa));
        // Volume integrals (§4.1), by the divergence theorem.
        out.t0 += n[0] * (a == 0 ? fa : (b == 0 ? fb : fc));
        out.t1.at(a) += na * faa;
        out.t1.at(b) += nb * fbb;
        out.t1.at(c) += n.at(c) * fcc;
        out.t2.at(a) += na * faaa;
        out.t2.at(b) += nb * fbbb;
        out.t2.at(c) += n.at(c) * fccc;
        out.tp.at(a) += na * faab;
        out.tp.at(b) += nb * fbbc;
        out.tp.at(c) += n.at(c) * fcca;
    }
    for (std::size_t k = 0; k < 3; ++k) {
        out.t1.at(k) /= 2.0;
        out.t2.at(k) /= 3.0;
        out.tp.at(k) /= 2.0;
    }
    return out;
}

TriMesh wholeOf(const MeshFile& mesh) {
    TriMesh whole;
    whole.positions = mesh.positions;
    whole.indices = mesh.indices;
    return whole;
}

Surface scaled(const Surface& surface, const std::array<double, 3>& scale) {
    Surface out = surface;
    for (Vec3& p : out.points) {
        for (std::size_t k = 0; k < 3; ++k) {
            p.at(k) *= scale.at(k);
        }
    }
    return out;
}

// --- fitting ---------------------------------------------------------------------------------

/// Eigenvectors of a symmetric 3×3 matrix by cyclic Jacobi rotations: the columns of the
/// returned matrix, turned to lie closest to x, y and z in that order, right-handed.
Mat3 principalAxes(const std::array<double, 6>& tensor) {
    Mat3 m = detail::toMatrix(tensor);
    Mat3 v{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    for (int sweep = 0; sweep < 50; ++sweep) {
        const double off = m[0][1] * m[0][1] + m[0][2] * m[0][2] + m[1][2] * m[1][2];
        const double scale = m[0][0] * m[0][0] + m[1][1] * m[1][1] + m[2][2] * m[2][2];
        if (off <= 1e-30 * std::max(scale, 1e-300)) {
            break;
        }
        for (std::size_t p = 0; p < 2; ++p) {
            for (std::size_t q = p + 1; q < 3; ++q) {
                if (m.at(p).at(q) == 0.0) {
                    continue;
                }
                const double theta = (m.at(q).at(q) - m.at(p).at(p)) / (2.0 * m.at(p).at(q));
                const double t =
                    (theta >= 0 ? 1.0 : -1.0) / (std::abs(theta) + std::sqrt(theta * theta + 1.0));
                const double cs = 1.0 / std::sqrt(t * t + 1.0);
                const double sn = t * cs;
                for (std::size_t k = 0; k < 3; ++k) {  // m = Jᵀ m J
                    const double mkp = m.at(k).at(p);
                    const double mkq = m.at(k).at(q);
                    m.at(k).at(p) = cs * mkp - sn * mkq;
                    m.at(k).at(q) = sn * mkp + cs * mkq;
                }
                for (std::size_t k = 0; k < 3; ++k) {
                    const double mpk = m.at(p).at(k);
                    const double mqk = m.at(q).at(k);
                    m.at(p).at(k) = cs * mpk - sn * mqk;
                    m.at(q).at(k) = sn * mpk + cs * mqk;
                }
                for (std::size_t k = 0; k < 3; ++k) {
                    const double vkp = v.at(k).at(p);
                    const double vkq = v.at(k).at(q);
                    v.at(k).at(p) = cs * vkp - sn * vkq;
                    v.at(k).at(q) = sn * vkp + cs * vkq;
                }
            }
        }
    }
    // Columns as axes; pick for x, y the column most along it, z = x × y.
    std::array<Vec3, 3> columns{};
    for (std::size_t k = 0; k < 3; ++k) {
        columns.at(k) = {v[0].at(k), v[1].at(k), v[2].at(k)};
    }
    std::array<bool, 3> used{};
    std::array<Vec3, 3> axes{};
    for (std::size_t axis = 0; axis < 2; ++axis) {
        std::size_t best = 3;
        for (std::size_t k = 0; k < 3; ++k) {
            if (!used.at(k) && (best == 3 || std::abs(columns.at(k).at(axis)) >
                                                 std::abs(columns.at(best).at(axis)))) {
                best = k;
            }
        }
        used.at(best) = true;
        axes.at(axis) = columns.at(best);
        if (axes.at(axis).at(axis) < 0.0) {
            for (double& value : axes.at(axis)) {
                value = -value;
            }
        }
    }
    axes[2] = cross(axes[0], axes[1]);
    Mat3 out{};
    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t k = 0; k < 3; ++k) {
            out.at(r).at(k) = axes.at(k).at(r);
        }
    }
    return out;
}

struct Ball {
    Vec3 centre{};
    double radius{-1.0};  // < 0: empty
    [[nodiscard]] bool holds(const Vec3& p) const {
        const Vec3 d = sub(p, centre);
        return radius >= 0.0 && dot(d, d) <= radius * radius * (1.0 + 1e-12) + 1e-24;
    }
};

Ball ballOf(const Vec3& a, const Vec3& b) {
    const Vec3 c{(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2};
    const Vec3 d = sub(a, c);
    return {c, std::sqrt(dot(d, d))};
}

/// The smallest ball with a, b, c on its sphere (their circumcircle's), or of the farthest
/// pair when they are in a line.
Ball ballOf(const Vec3& a, const Vec3& b, const Vec3& c) {
    const Vec3 ab = sub(b, a);
    const Vec3 ac = sub(c, a);
    const Vec3 n = cross(ab, ac);
    const double nn = dot(n, n);
    if (nn <= 1e-30 * dot(ab, ab) * dot(ac, ac)) {
        Ball best = ballOf(a, b);
        for (const Ball& other : {ballOf(a, c), ballOf(b, c)}) {
            if (other.radius > best.radius) {
                best = other;
            }
        }
        return best;
    }
    const Vec3 t1 = cross(n, ab);
    const Vec3 t2 = cross(ac, n);
    const double ac2 = dot(ac, ac);
    const double ab2 = dot(ab, ab);
    const Vec3 offset{(ac2 * t1[0] + ab2 * t2[0]) / (2 * nn),
                      (ac2 * t1[1] + ab2 * t2[1]) / (2 * nn),
                      (ac2 * t1[2] + ab2 * t2[2]) / (2 * nn)};
    return {{a[0] + offset[0], a[1] + offset[1], a[2] + offset[2]}, std::sqrt(dot(offset, offset))};
}

Ball ballOf(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d) {
    const Vec3 u = sub(b, a);
    const Vec3 v = sub(c, a);
    const Vec3 w = sub(d, a);
    const double det = dot(u, cross(v, w));
    const double scale = std::sqrt(dot(u, u) * dot(v, v) * dot(w, w));
    if (std::abs(det) <= 1e-12 * scale) {
        // In a plane: the smallest of the four triples' balls that holds all four.
        Ball best;
        for (const Ball& ball :
             {ballOf(a, b, c), ballOf(a, b, d), ballOf(a, c, d), ballOf(b, c, d)}) {
            if (ball.holds(a) && ball.holds(b) && ball.holds(c) && ball.holds(d) &&
                (best.radius < 0.0 || ball.radius < best.radius)) {
                best = ball;
            }
        }
        return best.radius >= 0.0 ? best : ballOf(a, b, c);
    }
    // 2 (p_i - a)·x = |p_i - a|², x = centre - a.
    const double uu = dot(u, u);
    const double vv = dot(v, v);
    const double ww = dot(w, w);
    const Vec3 vw = cross(v, w);
    const Vec3 wu = cross(w, u);
    const Vec3 uv = cross(u, v);
    const Vec3 x{(uu * vw[0] + vv * wu[0] + ww * uv[0]) / (2 * det),
                 (uu * vw[1] + vv * wu[1] + ww * uv[1]) / (2 * det),
                 (uu * vw[2] + vv * wu[2] + ww * uv[2]) / (2 * det)};
    return {{a[0] + x[0], a[1] + x[1], a[2] + x[2]}, std::sqrt(dot(x, x))};
}

/// The smallest enclosing ball (Welzl, "Smallest enclosing disks (balls and ellipsoids)",
/// 1991), in its iterative form: each point outside the ball so far is on the new one's sphere.
/// Expected linear time in a random order. `dimensions` 2: the points' z is 0 (a disc).
Ball smallestBall(std::vector<Vec3> points, std::size_t dimensions) {
    // A fixed shuffle (the expected time needs a random order; the fit must repeat).
    std::uint64_t state = 0x9E37'79B9'7F4A'7C15ULL;
    for (std::size_t i = points.size(); i > 1; --i) {
        state = state * 6364136223846793005ULL + 1442695040888963407ULL;
        std::swap(points[i - 1], points[(state >> 33U) % i]);
    }
    if (points.empty()) {
        return {};
    }
    Ball ball{points[0], 0.0};
    for (std::size_t i = 1; i < points.size(); ++i) {
        if (ball.holds(points[i])) {
            continue;
        }
        ball = {points[i], 0.0};
        for (std::size_t j = 0; j < i; ++j) {
            if (ball.holds(points[j])) {
                continue;
            }
            ball = ballOf(points[i], points[j]);
            for (std::size_t k = 0; k < j; ++k) {
                if (ball.holds(points[k])) {
                    continue;
                }
                ball = ballOf(points[i], points[j], points[k]);
                if (dimensions < 3) {
                    continue;
                }
                for (std::size_t l = 0; l < k; ++l) {
                    if (!ball.holds(points[l])) {
                        ball = ballOf(points[i], points[j], points[k], points[l]);
                    }
                }
            }
        }
    }
    return ball;
}

/// Matrix → URDF rpy (R = Rz(yaw) Ry(pitch) Rx(roll)).
std::array<double, 3> rpyOf(const Mat3& r) {
    const double pitch = std::atan2(-r[2][0], std::hypot(r[0][0], r[1][0]));
    if (std::hypot(r[0][0], r[1][0]) < 1e-12) {  // gimbal lock: roll folded into yaw
        return {0.0, pitch, std::atan2(-r[0][1], r[1][1])};
    }
    return {std::atan2(r[2][1], r[2][2]), pitch, std::atan2(r[1][0], r[0][0])};
}

/// The mesh's pose in the link (its origin) composed with a local pose (centre, axes).
Pose placed(const Pose& meshPose, const Vec3& centre, const Mat3& axes) {
    const Frame mesh = toFrame(meshPose);
    const Mat3 m = detail::matrixOf(mesh.rotation);
    Mat3 r{};
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            for (std::size_t k = 0; k < 3; ++k) {
                r.at(i).at(j) += m.at(i).at(k) * axes.at(k).at(j);
            }
        }
    }
    const Vec3 at = detail::apply(m, centre);
    Pose out;
    out.xyz = {mesh.position[0] + at[0], mesh.position[1] + at[1], mesh.position[2] + at[2]};
    out.rpy = rpyOf(r);
    for (double& angle : out.rpy) {
        if (std::abs(angle) < 1e-12) {
            angle = 0.0;  // no -0 in a written file
        }
    }
    return out;
}

}  // namespace

// --- MeshFile ------------------------------------------------------------------------------

struct MeshFile::Cache {
    std::mutex mutex;
    std::map<std::array<double, 3>, std::unique_ptr<MeshSolid>> solids;
};

MeshFile::MeshFile() : cache_(std::make_unique<Cache>()) {}
MeshFile::~MeshFile() = default;

const MeshSolid& MeshFile::solid(const std::array<double, 3>& scale) const {
    const std::lock_guard lock(cache_->mutex);
    auto& slot = cache_->solids[scale];
    if (slot) {
        return *slot;
    }
    slot = std::make_unique<MeshSolid>();
    MeshSolid& solid = *slot;
    solid.hull = serverHull(*this, scale);
    solid.closed = closed;
    if (closed) {
        solid.unit = volumeIntegrals(scaled(welded, scale), 1.0);
        if (solid.hull) {
            ConvexHull hull;
            hull.points = solid.hull->points;
            hull.triangles = solid.hull->triangles;
            solid.fill = hullFill(wholeOf(*this), scale, hull);
            solid.concave = solid.fill < kConcaveFill;
        }
    } else if (solid.hull) {
        solid.unit = volumeIntegrals(*solid.hull, 1.0);
        solid.fromHull = true;
    }
    return solid;
}

void setMeshBase(const std::filesystem::path& base) {
    const std::lock_guard lock(cacheMutex());
    baseDirectory() = base;
}

std::filesystem::path meshBase() {
    const std::lock_guard lock(cacheMutex());
    return baseDirectory();
}

std::shared_ptr<const MeshFile> loadMesh(const std::string& uri,
                                         const std::filesystem::path& base) {
    auto resolved = meshes::resolveMeshUri(uri, base.string());
    if (!resolved) {
        return failed(uri, resolved.error());
    }
    std::error_code error;
    const std::filesystem::path path(*resolved);
    const auto time = std::filesystem::last_write_time(path, error);
    if (error) {
        return failed(*resolved, "not found: " + path.string());
    }
    {
        const std::lock_guard lock(cacheMutex());
        if (const auto found = cache().find(*resolved);
            found != cache().end() && found->second.time == time) {
            return found->second.mesh;
        }
    }
    std::shared_ptr<const MeshFile> mesh = read(*resolved);  // outside the lock: may be slow
    const std::lock_guard lock(cacheMutex());
    cache()[*resolved] = Entry{time, mesh};
    return mesh;
}

std::shared_ptr<const MeshFile> loadMesh(const std::string& uri) {
    return loadMesh(uri, meshBase());
}

MassProperties volumeIntegrals(const Surface& surface, double density) {
    VolumeIntegrals t = integrate(surface);
    if (t.t0 < 0.0) {  // wound inwards: every integral has the opposite sign
        t.t0 = -t.t0;
        for (std::size_t k = 0; k < 3; ++k) {
            t.t1.at(k) = -t.t1.at(k);
            t.t2.at(k) = -t.t2.at(k);
            t.tp.at(k) = -t.tp.at(k);
        }
    }
    MassProperties out;
    if (!(t.t0 > 0.0) || !(density > 0.0)) {
        return out;
    }
    const double mass = density * t.t0;
    const Vec3 com{t.t1[0] / t.t0, t.t1[1] / t.t0, t.t1[2] / t.t0};
    // About the origin, then moved to the COM (the parallel axis theorem backwards).
    const double ixx = density * (t.t2[1] + t.t2[2]) - mass * (com[1] * com[1] + com[2] * com[2]);
    const double iyy = density * (t.t2[2] + t.t2[0]) - mass * (com[2] * com[2] + com[0] * com[0]);
    const double izz = density * (t.t2[0] + t.t2[1]) - mass * (com[0] * com[0] + com[1] * com[1]);
    const double ixy = -density * t.tp[0] + mass * com[0] * com[1];
    const double iyz = -density * t.tp[1] + mass * com[1] * com[2];
    const double ixz = -density * t.tp[2] + mass * com[2] * com[0];
    out.mass = mass;
    out.com = com;
    out.inertia = {ixx, ixy, ixz, iyy, iyz, izz};
    return out;
}

std::optional<Surface> serverHull(const MeshFile& mesh, const std::array<double, 3>& scale) {
    const std::optional<ConvexHull> hull = convexHullOf(wholeOf(mesh), scale);
    if (!hull) {
        return std::nullopt;
    }
    return Surface{hull->points, hull->triangles};
}

MeshData surfaceMesh(const Surface& surface) {
    MeshData out;
    std::array<float, 6> box{1e30F, 1e30F, 1e30F, -1e30F, -1e30F, -1e30F};
    for (const auto& t : surface.triangles) {
        const Vec3& p = surface.points[t[0]];
        Vec3 n = cross(sub(surface.points[t[1]], p), sub(surface.points[t[2]], p));
        const double length = std::sqrt(dot(n, n));
        if (length > 0.0) {
            n = {n[0] / length, n[1] / length, n[2] / length};
        }
        for (const std::uint32_t index : t) {
            const Vec3& v = surface.points[index];
            out.indices.push_back(static_cast<std::uint32_t>(out.positions.size() / 3));
            for (std::size_t k = 0; k < 3; ++k) {
                out.positions.push_back(static_cast<float>(v.at(k)));
                out.normals.push_back(static_cast<float>(n.at(k)));
                box.at(k) = std::min(box.at(k), static_cast<float>(v.at(k)));
                box.at(k + 3) = std::max(box.at(k + 3), static_cast<float>(v.at(k)));
            }
        }
    }
    out.aabb = out.positions.empty() ? std::array<float, 6>{} : box;
    return out;
}

std::optional<Shape> fitToMesh(const Shape& mesh, Shape::Kind kind) {
    if (mesh.kind != Shape::Kind::Mesh || kind == Shape::Kind::Mesh) {
        return std::nullopt;
    }
    const std::shared_ptr<const MeshFile> file = loadMesh(mesh.mesh);
    if (!file->error.empty()) {
        return std::nullopt;
    }
    const MeshSolid& solid = file->solid(mesh.scale);
    if (!solid.hull) {
        return std::nullopt;  // flat
    }
    // Every vertex, scaled (the server's hull keeps at most 256 of them, which can cut a
    // round end short).
    std::vector<Vec3> points = file->welded.points;
    for (Vec3& p : points) {
        for (std::size_t k = 0; k < 3; ++k) {
            p.at(k) *= mesh.scale.at(k);
        }
    }
    Shape out;
    out.kind = kind;
    out.visual = false;
    out.collision = true;
    out.solid = false;
    if (kind == Shape::Kind::Sphere) {
        const Ball ball = smallestBall(points, 3);
        out.size = {ball.radius, 0.0, 0.0};
        out.origin = placed(mesh.origin, ball.centre, Mat3{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}});
        return out;
    }
    // Two frames are tried, the mesh's principal axes of inertia and the mesh's own axes (a
    // part modelled square to its frame, whose inertia turns its axes); the smaller shape wins.
    const Mat3 identity{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    struct Fit {
        double volume{std::numeric_limits<double>::infinity()};
        std::array<double, 3> size{};
        Pose origin;
    };
    Fit best;
    for (const Mat3& axes : {principalAxes(solid.unit.inertia), identity}) {
        // Coordinates of every point along the axes (columns of `axes`).
        std::vector<Vec3> local;
        local.reserve(points.size());
        Vec3 low{1e300, 1e300, 1e300};
        Vec3 high{-1e300, -1e300, -1e300};
        for (const Vec3& p : points) {
            Vec3 q{};
            for (std::size_t k = 0; k < 3; ++k) {
                q.at(k) = axes[0].at(k) * p[0] + axes[1].at(k) * p[1] + axes[2].at(k) * p[2];
                low.at(k) = std::min(low.at(k), q.at(k));
                high.at(k) = std::max(high.at(k), q.at(k));
            }
            local.push_back(q);
        }
        const auto toMesh = [&](const Vec3& q) {  // axes coordinates → mesh frame
            return Vec3{axes[0][0] * q[0] + axes[0][1] * q[1] + axes[0][2] * q[2],
                        axes[1][0] * q[0] + axes[1][1] * q[1] + axes[1][2] * q[2],
                        axes[2][0] * q[0] + axes[2][1] * q[1] + axes[2][2] * q[2]};
        };
        if (kind == Shape::Kind::Box) {
            const std::array<double, 3> size{high[0] - low[0], high[1] - low[1], high[2] - low[2]};
            const double volume = size[0] * size[1] * size[2];
            if (volume < best.volume * (1.0 - 1e-9)) {
                const Vec3 mid{(high[0] + low[0]) / 2, (high[1] + low[1]) / 2,
                               (high[2] + low[2]) / 2};
                best = {volume, size, placed(mesh.origin, toMesh(mid), axes)};
            }
            continue;
        }
        // A cylinder or capsule along axis k: the smallest disc of the points across it.
        for (std::size_t k = 0; k < 3; ++k) {
            const std::size_t u = (k + 1) % 3;
            const std::size_t v = (k + 2) % 3;
            std::vector<Vec3> flat;
            flat.reserve(local.size());
            for (const Vec3& q : local) {
                flat.push_back({q.at(u), q.at(v), 0.0});
            }
            const Ball disc = smallestBall(flat, 2);
            const double r = disc.radius;
            double from = low.at(k);
            double to = high.at(k);
            double volume = std::numbers::pi * r * r * (to - from);
            if (kind == Shape::Kind::Capsule) {
                // The straight part [from, to] must reach each point's height less the cap there.
                from = 1e300;
                to = -1e300;
                for (const Vec3& q : local) {
                    const double du = q.at(u) - disc.centre[0];
                    const double dv = q.at(v) - disc.centre[1];
                    const double cap = std::sqrt(std::max(0.0, r * r - du * du - dv * dv));
                    from = std::min(from, q.at(k) + cap);
                    to = std::max(to, q.at(k) - cap);
                }
                if (to < from) {  // shorter than its caps: a sphere-like capsule
                    from = to = (from + to) / 2;
                }
                volume = std::numbers::pi * r * r * (to - from) +
                         4.0 / 3.0 * std::numbers::pi * r * r * r;
            }
            if (!(volume < best.volume * (1.0 - 1e-9))) {
                continue;
            }
            Vec3 mid{};
            mid.at(k) = (from + to) / 2;
            mid.at(u) = disc.centre[0];
            mid.at(v) = disc.centre[1];
            // The shape's z along axis k: columns (u, v, k) of `axes`, right-handed (cyclic).
            Mat3 turned{};
            for (std::size_t row = 0; row < 3; ++row) {
                turned.at(row) = {axes.at(row).at(u), axes.at(row).at(v), axes.at(row).at(k)};
            }
            best = {volume, {r, to - from, 0.0}, placed(mesh.origin, toMesh(mid), turned)};
        }
    }
    out.size = best.size;
    out.origin = best.origin;
    return out;
}

}  // namespace lagrange::viewer::builder
