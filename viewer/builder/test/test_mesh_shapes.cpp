// Mesh shapes and the collision choice of the builder (RFC 0016 Addendum A, its "Tests"):
// mass of closed meshes against closed forms (a tessellated box and sphere), Fit to mesh
// against known boxes, cylinders and spheres, the hull the builder draws equal to the server's
// (the same convexHullOf on the same merge), each collision choice through the URDF and back,
// and a split written by CoACD that the server's loader reads with no notes.
#include "lagrange/meshes/MeshImport.hpp"
#include "lagrange/viewer/builder/Collision.hpp"
#include "lagrange/viewer/builder/CollisionSplit.hpp"
#include "lagrange/viewer/builder/Gizmo.hpp"
#include "lagrange/viewer/builder/MassProperties.hpp"
#include "lagrange/viewer/builder/MeshShapes.hpp"
#include "lagrange/viewer/builder/Primitives.hpp"
#include "lagrange/viewer/builder/SceneAdapter.hpp"
#include "lagrange/viewer/builder/Urdf.hpp"
#include "lagrange/viewer/builder/Validation.hpp"
#include "lagrange/world/Model.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <numbers>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

namespace builder = lagrange::viewer::builder;
using builder::CollisionChoice;
using builder::Document;
using builder::Link;
using builder::Shape;
using builder::Surface;
using Vec3 = std::array<double, 3>;

namespace {

constexpr double kPi = std::numbers::pi;

/// A fresh folder for one test's files.
std::filesystem::path scratch(const std::string& name) {
    const std::filesystem::path dir =
        std::filesystem::temp_directory_path() / ("lagrange_mesh_shapes_" + name);
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    return dir;
}

/// Rotation matrix of URDF rpy (R = Rz Ry Rx).
std::array<Vec3, 3> rotation(const Vec3& rpy) {
    const double cr = std::cos(rpy[0]), sr = std::sin(rpy[0]);
    const double cp = std::cos(rpy[1]), sp = std::sin(rpy[1]);
    const double cy = std::cos(rpy[2]), sy = std::sin(rpy[2]);
    return {{{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
             {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
             {-sp, cp * sr, cp * cr}}};
}

Vec3 apply(const std::array<Vec3, 3>& r, const Vec3& v) {
    return {r[0][0] * v[0] + r[0][1] * v[1] + r[0][2] * v[2],
            r[1][0] * v[0] + r[1][1] * v[1] + r[1][2] * v[2],
            r[2][0] * v[0] + r[2][1] * v[1] + r[2][2] * v[2]};
}

/// A primitive's MeshData as a surface (seam vertices stay duplicated: the integrals do not
/// need them welded), turned by `rpy` and moved by `offset`.
Surface surfaceOf(const lagrange::viewer::MeshData& mesh, const Vec3& rpy = {},
                  const Vec3& offset = {}) {
    Surface out;
    const auto r = rotation(rpy);
    for (std::size_t i = 0; i + 2 < mesh.positions.size(); i += 3) {
        const Vec3 p = apply(r, {mesh.positions[i], mesh.positions[i + 1], mesh.positions[i + 2]});
        out.points.push_back({p[0] + offset[0], p[1] + offset[1], p[2] + offset[2]});
    }
    for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        out.triangles.push_back({mesh.indices[i], mesh.indices[i + 1], mesh.indices[i + 2]});
    }
    return out;
}

/// An icosphere of radius r, `levels` subdivisions, welded (a closed, outward surface).
Surface icosphere(double r, int levels) {
    const double t = (1.0 + std::sqrt(5.0)) / 2.0;
    Surface s;
    for (const Vec3& p : std::vector<Vec3>{{-1, t, 0},
                                           {1, t, 0},
                                           {-1, -t, 0},
                                           {1, -t, 0},
                                           {0, -1, t},
                                           {0, 1, t},
                                           {0, -1, -t},
                                           {0, 1, -t},
                                           {t, 0, -1},
                                           {t, 0, 1},
                                           {-t, 0, -1},
                                           {-t, 0, 1}}) {
        const double l = std::hypot(p[0], p[1], p[2]);
        s.points.push_back({p[0] / l, p[1] / l, p[2] / l});
    }
    s.triangles = {{0, 11, 5}, {0, 5, 1},  {0, 1, 7},   {0, 7, 10}, {0, 10, 11},
                   {1, 5, 9},  {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
                   {3, 9, 4},  {3, 4, 2},  {3, 2, 6},   {3, 6, 8},  {3, 8, 9},
                   {4, 9, 5},  {2, 4, 11}, {6, 2, 10},  {8, 6, 7},  {9, 8, 1}};
    for (int level = 0; level < levels; ++level) {
        std::map<std::pair<std::uint32_t, std::uint32_t>, std::uint32_t> middle;
        const auto mid = [&](std::uint32_t a, std::uint32_t b) {
            const auto key = std::minmax(a, b);
            if (const auto found = middle.find(key); found != middle.end()) {
                return found->second;
            }
            Vec3 m{(s.points[a][0] + s.points[b][0]) / 2, (s.points[a][1] + s.points[b][1]) / 2,
                   (s.points[a][2] + s.points[b][2]) / 2};
            const double l = std::hypot(m[0], m[1], m[2]);
            s.points.push_back({m[0] / l, m[1] / l, m[2] / l});
            const auto index = static_cast<std::uint32_t>(s.points.size() - 1);
            middle.emplace(key, index);
            return index;
        };
        std::vector<std::array<std::uint32_t, 3>> next;
        for (const auto& [a, b, c] : s.triangles) {
            const std::uint32_t ab = mid(a, b), bc = mid(b, c), ca = mid(c, a);
            next.insert(next.end(), {{a, ab, ca}, {b, bc, ab}, {c, ca, bc}, {ab, bc, ca}});
        }
        s.triangles = std::move(next);
    }
    for (Vec3& p : s.points) {
        p = {p[0] * r, p[1] * r, p[2] * r};
    }
    return s;
}

/// The volume of a closed surface by signed tetrahedra (an independent check of Mirtich's T0).
double tetraVolume(const Surface& s) {
    double v = 0.0;
    for (const auto& [a, b, c] : s.triangles) {
        const Vec3& p = s.points[a];
        const Vec3& q = s.points[b];
        const Vec3& r = s.points[c];
        v += (p[0] * (q[1] * r[2] - q[2] * r[1]) - p[1] * (q[0] * r[2] - q[2] * r[0]) +
              p[2] * (q[0] * r[1] - q[1] * r[0])) /
             6.0;
    }
    return std::abs(v);
}

/// Writes a surface as an OBJ file, each triangle with vertices of its own (as an STL repeats
/// them), so the builder must weld them to see it closed.
void writeObj(const std::filesystem::path& path, const Surface& s, bool split = true) {
    std::ofstream out(path);
    out.precision(17);
    if (split) {
        std::size_t n = 0;
        for (const auto& t : s.triangles) {
            for (const std::uint32_t i : t) {
                out << "v " << s.points[i][0] << ' ' << s.points[i][1] << ' ' << s.points[i][2]
                    << '\n';
            }
            out << "f " << n + 1 << ' ' << n + 2 << ' ' << n + 3 << '\n';
            n += 3;
        }
        return;
    }
    for (const Vec3& p : s.points) {
        out << "v " << p[0] << ' ' << p[1] << ' ' << p[2] << '\n';
    }
    for (const auto& t : s.triangles) {
        out << "f " << t[0] + 1 << ' ' << t[1] + 1 << ' ' << t[2] + 1 << '\n';
    }
}

void expectTensor(const std::array<double, 6>& got, const std::array<double, 6>& want,
                  double tolerance) {
    for (std::size_t k = 0; k < 6; ++k) {
        EXPECT_NEAR(got.at(k), want.at(k), tolerance) << "component " << k;
    }
}

/// The inertia of a box a × b × c, mass m, turned by r: R diag Rᵀ.
std::array<double, 6> turnedBoxInertia(double m, const Vec3& size, const Vec3& rpy) {
    const double a = size[0], b = size[1], c = size[2];
    const Vec3 d{m * (b * b + c * c) / 12, m * (a * a + c * c) / 12, m * (a * a + b * b) / 12};
    const auto r = rotation(rpy);
    std::array<Vec3, 3> out{};
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            for (std::size_t k = 0; k < 3; ++k) {
                out.at(i).at(j) += r.at(i).at(k) * d.at(k) * r.at(j).at(k);
            }
        }
    }
    return {out[0][0], out[0][1], out[0][2], out[1][1], out[1][2], out[2][2]};
}

Shape meshShape(const std::filesystem::path& file) {
    Shape shape;
    shape.kind = Shape::Kind::Mesh;
    shape.mesh = file.string();
    return shape;
}

}  // namespace

// --- mass (Mirtich 1996) against closed forms ------------------------------------------------

TEST(MeshMass, TessellatedBoxIsExact) {
    // A box is exactly its 12 triangles: Mirtich's integrals give the closed form, turned and
    // moved off the origin, to the float32 rounding of the mesh's vertices (MeshData's).
    const Vec3 size{0.3, 0.2, 0.1};
    const Vec3 rpy{0.3, -0.5, 1.1};
    const Vec3 at{0.4, -0.2, 0.7};
    const double density = 2700.0;
    const builder::MassProperties p =
        builder::volumeIntegrals(surfaceOf(builder::boxMesh(size), rpy, at), density);
    const double m = density * size[0] * size[1] * size[2];
    EXPECT_NEAR(p.mass, m, 1e-6 * m);
    for (std::size_t k = 0; k < 3; ++k) {
        EXPECT_NEAR(p.com.at(k), at.at(k), 1e-7);
    }
    expectTensor(p.inertia, turnedBoxInertia(m, size, rpy), 1e-6 * 0.15);
    // In double precision the same solid is exact to rounding.
    Surface exact;
    for (int corner = 0; corner < 8; ++corner) {
        exact.points.push_back({(corner & 1) != 0 ? 0.5 : -0.5, (corner & 2) != 0 ? 1.0 : -1.0,
                                (corner & 4) != 0 ? 1.5 : -1.5});
    }
    exact.triangles = {{0, 2, 3}, {0, 3, 1}, {4, 5, 7}, {4, 7, 6}, {0, 1, 5}, {0, 5, 4},
                       {2, 6, 7}, {2, 7, 3}, {0, 4, 6}, {0, 6, 2}, {1, 3, 7}, {1, 7, 5}};
    const builder::MassProperties q = builder::volumeIntegrals(exact, 2.0);
    EXPECT_NEAR(q.mass, 2.0 * 6.0, 1e-12);
    expectTensor(q.inertia, turnedBoxInertia(12.0, {1.0, 2.0, 3.0}, {}), 1e-12);
}

TEST(MeshMass, InwardWindingGivesTheSame) {
    Surface s = surfaceOf(builder::boxMesh({0.1, 0.2, 0.3}));
    for (auto& t : s.triangles) {
        std::swap(t[1], t[2]);
    }
    const builder::MassProperties p = builder::volumeIntegrals(s, 1000.0);
    EXPECT_NEAR(p.mass, 1000.0 * 0.006, 1e-6);
    EXPECT_GT(p.inertia[0], 0.0);
}

TEST(MeshMass, TessellatedSphereTendsToTheClosedForm) {
    const double r = 0.05;
    const double density = 7850.0;
    const Surface s = icosphere(r, 4);  // 5120 faces
    const builder::MassProperties p = builder::volumeIntegrals(s, density);
    // T0 against the polyhedron's own volume by tetrahedra: the same solid, two methods.
    EXPECT_NEAR(p.mass, density * tetraVolume(s), 1e-12 * p.mass);
    // Against the sphere's closed forms: the polyhedron is inside it, within 0.2 % at this
    // tessellation (inscribed in the sphere: volume short by about the chord's sagitta).
    const double m = density * 4.0 / 3.0 * kPi * r * r * r;
    EXPECT_NEAR(p.mass, m, 3e-3 * m);
    EXPECT_LT(p.mass, m);
    const double i = 2.0 / 5.0 * p.mass * r * r;  // with the polyhedron's mass
    for (const std::size_t k : {0U, 3U, 5U}) {
        EXPECT_NEAR(p.inertia.at(k), i, 3e-3 * i);
    }
    for (const std::size_t k : {1U, 2U, 4U}) {
        EXPECT_NEAR(p.inertia.at(k), 0.0, 1e-9 * i);
    }
    for (std::size_t k = 0; k < 3; ++k) {
        EXPECT_NEAR(p.com.at(k), 0.0, 1e-12);
    }
}

TEST(MeshMass, CapsuleClosedFormMatchesItsTessellation) {
    const double r = 0.04;
    const double h = 0.12;
    Shape capsule;
    capsule.kind = Shape::Kind::Capsule;
    capsule.size = {r, h, 0.0};
    const builder::MassProperties closed = builder::shapeMass(capsule, 1000.0);
    const builder::MassProperties fine =
        builder::volumeIntegrals(surfaceOf(builder::capsuleMesh(r, h, 256, 128)), 1000.0);
    EXPECT_NEAR(closed.mass, 1000.0 * (kPi * r * r * h + 4.0 / 3.0 * kPi * r * r * r), 1e-12);
    EXPECT_NEAR(fine.mass, closed.mass, 1e-3 * closed.mass);
    for (const std::size_t k : {0U, 3U, 5U}) {
        EXPECT_NEAR(fine.inertia.at(k), closed.inertia.at(k), 2e-3 * closed.inertia.at(k));
    }
}

TEST(MeshMass, ClosedFileWeldsAndScales) {
    const auto dir = scratch("closed");
    writeObj(dir / "box.obj", surfaceOf(builder::boxMesh({1.0, 2.0, 3.0})));
    const auto file = builder::loadMesh((dir / "box.obj").string(), {});
    ASSERT_TRUE(file->error.empty()) << file->error;
    EXPECT_TRUE(file->closed);
    EXPECT_EQ(file->welded.points.size(), 8U);
    Shape shape = meshShape(dir / "box.obj");
    shape.scale = {0.1, 0.1, 0.1};  // 0.1 × 0.2 × 0.3 m
    shape.origin.xyz = {0.0, 0.0, 1.0};
    const builder::MassProperties p = builder::shapeMass(shape, 1000.0);
    EXPECT_NEAR(p.mass, 6.0, 1e-5);
    EXPECT_NEAR(p.com[2], 1.0, 1e-7);
    EXPECT_FALSE(file->solid(shape.scale).fromHull);
    // Through a link: the link's mass is the mesh's.
    Link link;
    link.name = "part";
    link.shapes.push_back(shape);
    link.inertial.density = 1000.0;
    ASSERT_TRUE(builder::linkMass(link).has_value());
    EXPECT_NEAR(builder::linkMass(link)->mass, 6.0, 1e-5);
}

TEST(MeshMass, OpenMeshTakesItsHull) {
    const auto dir = scratch("open");
    Surface s = surfaceOf(builder::boxMesh({1.0, 1.0, 1.0}));
    s.triangles.resize(10);  // one face of the cube gone: not closed
    writeObj(dir / "open.obj", s);
    const auto file = builder::loadMesh((dir / "open.obj").string(), {});
    ASSERT_TRUE(file->error.empty());
    EXPECT_FALSE(file->closed);
    const builder::MeshSolid& solid = file->solid({1, 1, 1});
    EXPECT_TRUE(solid.fromHull);
    EXPECT_NEAR(solid.unit.mass, 1.0, 1e-6);  // the hull is the whole cube
    // Validation says so (a warning) once the file's folder is known.
    Document document = builder::newDocument("open");
    Shape shape = meshShape(dir / "open.obj");
    document.links[0].shapes = {shape};
    document.links[0].inertial.density = 1000.0;
    bool warned = false;
    for (const builder::Problem& p : builder::validate(document, nullptr, dir)) {
        warned = warned || (p.level == builder::Level::Warning &&
                            p.message.find("not closed") != std::string::npos);
    }
    EXPECT_TRUE(warned);
}

// --- the hull: the server's -------------------------------------------------------------------

TEST(MeshHull, EqualsTheServersHull) {
    // Robot.cpp's addMeshCollider: importMesh, every part into one TriMesh, convexHullOf at the
    // URDF's scale. The builder's MeshFile must give the same points and triangles.
    const std::filesystem::path dae =
        std::filesystem::path(LAGRANGE_BUILDER_ASSETS_DIR) / "amr_fms" / "meshes" / "battery.dae";
    const std::array<double, 3> scale{0.5, 1.0, 2.0};
    auto imported = lagrange::meshes::importMesh(dae.string());
    ASSERT_TRUE(imported.has_value());
    lagrange::TriMesh whole;
    for (const auto& part : imported->parts) {
        const auto first = static_cast<std::uint32_t>(whole.positions.size() / 3);
        whole.positions.insert(whole.positions.end(), part.positions.begin(), part.positions.end());
        for (const std::uint32_t index : part.indices) {
            whole.indices.push_back(first + index);
        }
    }
    const auto server = lagrange::convexHullOf(whole, scale);
    const auto file = builder::loadMesh(dae.string(), {});
    const auto mine = builder::serverHull(*file, scale);
    ASSERT_TRUE(server.has_value());
    ASSERT_TRUE(mine.has_value());
    EXPECT_EQ(mine->points, server->points);
    EXPECT_EQ(mine->triangles, server->triangles);
}

TEST(MeshHull, TwoPartsInOneFileAreConcaveWithAHint) {
    const auto dir = scratch("concave");
    Surface two = surfaceOf(builder::boxMesh({1, 1, 1}), {}, {-1.5, 0, 0});
    const Surface other = surfaceOf(builder::boxMesh({1, 1, 1}), {}, {1.5, 0, 0});
    const auto base = static_cast<std::uint32_t>(two.points.size());
    two.points.insert(two.points.end(), other.points.begin(), other.points.end());
    for (auto t : other.triangles) {
        two.triangles.push_back({t[0] + base, t[1] + base, t[2] + base});
    }
    writeObj(dir / "two.obj", two);
    const auto file = builder::loadMesh((dir / "two.obj").string(), {});
    const builder::MeshSolid& solid = file->solid({1, 1, 1});
    EXPECT_TRUE(solid.closed);
    EXPECT_NEAR(solid.fill, 2.0 / 4.0, 1e-9);  // two cubes in a 4 × 1 × 1 hull
    EXPECT_TRUE(solid.concave);
    Document document = builder::newDocument("two");
    document.links[0].shapes = {meshShape(dir / "two.obj")};
    document.links[0].inertial.density = 1000.0;
    bool hinted = false;
    for (const builder::Problem& p : builder::validate(document, nullptr, dir)) {
        hinted = hinted || (p.level == builder::Level::Hint &&
                            p.message.find("concave") != std::string::npos);
    }
    EXPECT_TRUE(hinted);
}

// --- Fit to mesh -------------------------------------------------------------------------------

TEST(FitToMesh, BoxFindsATurnedBox) {
    const auto dir = scratch("fit_box");
    const Vec3 size{0.4, 0.2, 0.1};
    const Vec3 rpy{0.0, 0.0, 0.5};
    const Vec3 at{0.1, 0.2, 0.3};
    writeObj(dir / "box.obj", surfaceOf(builder::boxMesh(size), rpy, at));
    Shape mesh = meshShape(dir / "box.obj");
    mesh.origin.xyz = {0.0, 0.0, 1.0};
    const auto fitted = builder::fitToMesh(mesh, Shape::Kind::Box);
    ASSERT_TRUE(fitted.has_value());
    EXPECT_EQ(fitted->kind, Shape::Kind::Box);
    EXPECT_FALSE(fitted->visual);
    EXPECT_TRUE(fitted->collision);
    EXPECT_FALSE(fitted->solid);
    for (std::size_t k = 0; k < 3; ++k) {
        EXPECT_NEAR(fitted->size.at(k), size.at(k), 1e-6);
    }
    EXPECT_NEAR(fitted->origin.xyz[0], at[0], 1e-6);
    EXPECT_NEAR(fitted->origin.xyz[1], at[1], 1e-6);
    EXPECT_NEAR(fitted->origin.xyz[2], at[2] + 1.0, 1e-6);
    // The same box: its corners where the mesh's are (a box's axes are only up to sign).
    EXPECT_NEAR(std::abs(std::sin(fitted->origin.rpy[2] - rpy[2])), 0.0, 1e-6);
    EXPECT_NEAR(fitted->origin.rpy[0], 0.0, 1e-6);
    EXPECT_NEAR(fitted->origin.rpy[1], 0.0, 1e-6);
}

TEST(FitToMesh, CylinderFindsItsAxisAndRadius) {
    const auto dir = scratch("fit_cylinder");
    const double r = 0.05;
    const double h = 0.3;
    const Vec3 rpy{kPi / 2, 0.0, 0.0};  // axis along -y
    writeObj(dir / "cyl.obj", surfaceOf(builder::cylinderMesh(r, h), rpy, {0, 0, 0.2}));
    const auto fitted = builder::fitToMesh(meshShape(dir / "cyl.obj"), Shape::Kind::Cylinder);
    ASSERT_TRUE(fitted.has_value());
    EXPECT_NEAR(fitted->size[0], r, 1e-6);
    EXPECT_NEAR(fitted->size[1], h, 1e-6);
    EXPECT_NEAR(fitted->origin.xyz[2], 0.2, 1e-6);
    // Its z along the mesh's axis (y): R's third column is ±y.
    const auto rot = rotation(fitted->origin.rpy);
    EXPECT_NEAR(std::abs(rot[1][2]), 1.0, 1e-6);
}

TEST(FitToMesh, SphereEnclosesAndCapsuleHolds) {
    const auto dir = scratch("fit_round");
    const double r = 0.07;
    writeObj(dir / "ball.obj", icosphere(r, 2), false);
    const auto sphere = builder::fitToMesh(meshShape(dir / "ball.obj"), Shape::Kind::Sphere);
    ASSERT_TRUE(sphere.has_value());
    EXPECT_NEAR(sphere->size[0], r, 1e-7);  // every vertex is on the sphere (float32 file)
    for (const double v : sphere->origin.xyz) {
        EXPECT_NEAR(v, 0.0, 1e-7);
    }
    // A capsule mesh: the fit gives its radius and straight length back.
    writeObj(dir / "capsule.obj", surfaceOf(builder::capsuleMesh(0.03, 0.2, 64, 32)), true);
    const auto capsule = builder::fitToMesh(meshShape(dir / "capsule.obj"), Shape::Kind::Capsule);
    ASSERT_TRUE(capsule.has_value());
    EXPECT_NEAR(capsule->size[0], 0.03, 1e-6);
    EXPECT_NEAR(capsule->size[1], 0.2, 1e-6);
    // And the box of the capsule mesh is 0.06 × 0.06 × 0.26.
    const auto box = builder::fitToMesh(meshShape(dir / "capsule.obj"), Shape::Kind::Box);
    ASSERT_TRUE(box.has_value());
    EXPECT_NEAR(box->size[2], 0.26, 1e-6);
}

// --- the collision choice in the document and the URDF ------------------------------------------

namespace {

Document meshRobot(const std::filesystem::path& file) {
    Document document = builder::newDocument("meshbot");
    const builder::AddedLink added =
        builder::addMeshLink(document, "base_link", file.string(),
                             lagrange::viewer::builder::Joint::Type::Fixed, 1000.0);
    EXPECT_EQ(added.link, "part");
    return document;
}

Document roundTrip(const Document& document, const std::filesystem::path& at) {
    const std::string text = builder::writeUrdf(document, {.savedAt = at});
    const builder::ReadResult read = builder::readUrdf(text);
    EXPECT_TRUE(read.document.has_value()) << read.error;
    return read.document.value_or(Document{});
}

int count(const std::string& text, const std::string& what) {
    int n = 0;
    for (std::size_t at = text.find(what); at != std::string::npos; at = text.find(what, at + 1)) {
        ++n;
    }
    return n;
}

}  // namespace

TEST(CollisionChoice, EachChoiceRoundTrips) {
    const auto dir = scratch("choices");
    writeObj(dir / "part.obj", surfaceOf(builder::boxMesh({0.2, 0.1, 0.05})));
    const std::filesystem::path urdf = dir / "meshbot.urdf";
    builder::setMeshBase(dir);  // what BuilderHost does: relative meshes against the file's folder
    Document document = meshRobot(dir / "part.obj");
    Link* link = builder::findLink(document, "part");
    ASSERT_NE(link, nullptr);
    EXPECT_EQ(link->collision, CollisionChoice::Mesh);

    // 1: the mesh collides; one <visual> and one <collision> naming it, merged on reading.
    std::string text = builder::writeUrdf(document, {.savedAt = urdf});
    EXPECT_NE(text.find("lagrange_collision=\"mesh\""), std::string::npos);
    EXPECT_NE(text.find("filename=\"part.obj\""), std::string::npos);  // relative on save
    Document back = roundTrip(document, urdf);
    const Link* read = builder::findLink(back, "part");
    ASSERT_NE(read, nullptr);
    EXPECT_EQ(read->collision, CollisionChoice::Mesh);
    ASSERT_EQ(read->shapes.size(), 1U);
    EXPECT_TRUE(read->shapes[0].visual && read->shapes[0].collision);

    // 3: shapes: a fitted box; the mesh is visual and solid only, the box collision only.
    builder::setCollisionChoice(*link, CollisionChoice::Shapes);
    const auto fitted = builder::fitToMesh(link->shapes[0], Shape::Kind::Box);
    ASSERT_TRUE(fitted.has_value());
    builder::addCollisionShape(*link, *fitted);
    text = builder::writeUrdf(document, {.savedAt = urdf});
    EXPECT_EQ(count(text, "<visual"), 2);     // base_link's box and the mesh
    EXPECT_EQ(count(text, "<collision"), 2);  // base_link's box and the fitted box
    back = roundTrip(document, urdf);
    read = builder::findLink(back, "part");
    EXPECT_EQ(read->collision, CollisionChoice::Shapes);
    ASSERT_EQ(read->shapes.size(), 2U);
    EXPECT_TRUE(read->shapes[0].visual && !read->shapes[0].collision && read->shapes[0].solid);
    EXPECT_TRUE(!read->shapes[1].visual && read->shapes[1].collision && !read->shapes[1].solid);
    EXPECT_EQ(read->shapes[1].kind, Shape::Kind::Box);
    // Mass from the mesh only, the same after reading.
    ASSERT_TRUE(builder::linkMass(*read).has_value());
    EXPECT_NEAR(builder::linkMass(*read)->mass, 1000.0 * 0.2 * 0.1 * 0.05, 1e-6);

    // 4: drawn: a capsule placed by hand stays; the choice reads back.
    builder::setCollisionChoice(*link, CollisionChoice::Drawn);
    Shape capsule = builder::defaultShape(Shape::Kind::Capsule);
    capsule.origin.xyz = {0.01, 0.02, 0.03};
    builder::addCollisionShape(*link, capsule);
    // A capsule is URDF 1.1 (urdfdom ignores it in 1.0): the version is written with it.
    EXPECT_NE(builder::writeUrdf(document, {.savedAt = urdf})
                  .find("<robot name=\"meshbot\" version=\"1.1\">"),
              std::string::npos);
    back = roundTrip(document, urdf);
    read = builder::findLink(back, "part");
    EXPECT_EQ(read->collision, CollisionChoice::Drawn);
    ASSERT_EQ(read->shapes.size(), 3U);  // the mesh, the fitted box, the capsule
    EXPECT_EQ(read->shapes[2].kind, Shape::Kind::Capsule);
    EXPECT_EQ(read->shapes[2].size, capsule.size);
    EXPECT_EQ(read->shapes[2].origin.xyz, capsule.origin.xyz);

    // 2: split: the parts replace the primitives; the choice and the parts read back.
    builder::setSplitParts(*link, {(dir / "meshbot_collision" / "part_0_000.obj").string(),
                                   (dir / "meshbot_collision" / "part_0_001.obj").string()});
    text = builder::writeUrdf(document, {.savedAt = urdf});
    EXPECT_NE(text.find("meshbot_collision/part_0_001.obj"), std::string::npos);
    back = roundTrip(document, urdf);
    read = builder::findLink(back, "part");
    EXPECT_EQ(read->collision, CollisionChoice::Split);
    ASSERT_EQ(read->shapes.size(), 3U);
    EXPECT_EQ(read->shapes[1].mesh, "meshbot_collision/part_0_000.obj");
    EXPECT_TRUE(builder::collisionOnly(read->shapes[2]));

    // Back to 1 removes every collision-only shape.
    builder::setCollisionChoice(*link, CollisionChoice::Mesh);
    EXPECT_EQ(link->shapes.size(), 1U);
}

TEST(CollisionChoice, SceneDrawsMeshesAndCollisionTranslucent) {
    const auto dir = scratch("scene");
    writeObj(dir / "part.obj", surfaceOf(builder::boxMesh({0.2, 0.1, 0.05})));
    Document document = meshRobot(dir / "part.obj");
    Link* link = builder::findLink(document, "part");
    builder::setCollisionChoice(*link, CollisionChoice::Drawn);
    builder::addCollisionShape(*link, builder::defaultShape(Shape::Kind::Sphere));
    const builder::BuiltScene off = builder::buildStatics(document, 1);
    const builder::BuiltScene on = builder::buildStatics(document, 2, {.collision = true});
    const auto collisionPrims = [](const builder::BuiltScene& scene) {
        int n = 0;
        for (const auto& prim : scene.statics->prims) {
            n += prim.typeName == "Collision" ? 1 : 0;
        }
        return n;
    };
    // The mesh is drawn (a prim with a mesh on link "part"), the sphere always (collision only).
    bool meshDrawn = false;
    for (std::size_t i = 0; i < on.statics->prims.size(); ++i) {
        meshDrawn =
            meshDrawn || (on.primItem[i] == "part" && on.statics->prims[i].typeName == "Mesh");
    }
    EXPECT_TRUE(meshDrawn);
    EXPECT_EQ(collisionPrims(off), 1);
    EXPECT_EQ(collisionPrims(on), 2);  // + base_link's box
    const auto& last = on.statics->prims.back();
    EXPECT_EQ(last.typeName, "Collision");
    EXPECT_LT(on.statics->materials[last.material].baseColor[3], 1.0F);
    EXPECT_EQ(on.primShape.back(), 1);  // the sphere is shape 1 of "part"
    EXPECT_EQ(on.primShape.size(), on.statics->prims.size());
}

// --- a split, written and loaded by the server's loader -----------------------------------------

namespace {

std::optional<std::filesystem::path> program(const char* package, const char* name) {
    const char* prefixes = std::getenv("AMENT_PREFIX_PATH");
    std::istringstream list(prefixes != nullptr ? prefixes : "");
    for (std::string prefix; std::getline(list, prefix, ':');) {
        const auto path = std::filesystem::path(prefix) / "lib" / package / name;
        if (std::filesystem::is_regular_file(path)) {
            return path;
        }
    }
    return std::nullopt;
}

}  // namespace

TEST(CollisionSplit, PartsAreWrittenAndLoadWithNoNotes) {
    const auto bake = builder::findBakeProgram();
    const auto import = program("lagrange_server", "lagrange_import");
    if (!bake || !import) {
        GTEST_SKIP() << "lagrange_collision_bake or lagrange_server is not installed";
    }
    const auto dir = scratch("split");
    // An L of two boxes: concave, so CoACD splits it.
    Surface l = surfaceOf(builder::boxMesh({0.3, 0.1, 0.1}), {}, {0.1, 0, 0});
    const Surface leg = surfaceOf(builder::boxMesh({0.1, 0.1, 0.3}), {}, {-0.1, 0, 0.2});
    const auto base = static_cast<std::uint32_t>(l.points.size());
    l.points.insert(l.points.end(), leg.points.begin(), leg.points.end());
    for (auto t : leg.triangles) {
        l.triangles.push_back({t[0] + base, t[1] + base, t[2] + base});
    }
    writeObj(dir / "bracket.obj", l, false);
    Document document = builder::newDocument("splitbot");
    document.links[0].inertial.density = 1000.0;
    const builder::AddedLink added =
        builder::addMeshLink(document, "base_link", (dir / "bracket.obj").string(),
                             lagrange::viewer::builder::Joint::Type::Fixed, 1000.0);
    ASSERT_EQ(added.link, "bracket");
    const std::filesystem::path urdf = dir / "splitbot.urdf";
    ASSERT_EQ(builder::saveAtomically(urdf, builder::writeUrdf(document, {.savedAt = urdf})), "");

    // CoACD is found for the account, not through $HOME (a user-site install must still be
    // found when HOME points elsewhere, as here).
    const char* home = std::getenv("HOME");
    const std::string savedHome = home != nullptr ? home : "";
    ::setenv("HOME", dir.c_str(), 1);
    ::unsetenv("PYTHONUSERBASE");
    builder::CollisionSplit split;
    builder::SplitRequest request;
    request.robotFile = urdf;
    request.robot = "splitbot";
    request.link = "bracket";
    request.mesh = "bracket.obj";
    request.meshBase = dir;
    ASSERT_EQ(split.start(request), "");
    EXPECT_EQ(split.state(), builder::CollisionSplit::State::Running);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(240);
    while (split.poll() == builder::CollisionSplit::State::Running &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ::setenv("HOME", savedHome.c_str(), 1);
    ASSERT_EQ(split.state(), builder::CollisionSplit::State::Done) << split.error();
    ASSERT_GE(split.parts().size(), 2U);
    for (const std::string& part : split.parts()) {
        EXPECT_EQ(std::filesystem::path(part).parent_path(), dir / "splitbot_collision");
    }
    EXPECT_FALSE(std::filesystem::exists(dir / ".splitbot_split_bracket"));  // work folder gone

    builder::setSplitParts(*builder::findLink(document, "bracket"), split.parts());
    ASSERT_EQ(builder::saveAtomically(urdf, builder::writeUrdf(document, {.savedAt = urdf})), "");
    const std::string command = import->string() + " " + urdf.string() + " " +
                                (dir / "out.usda").string() + " 2>" + (dir / "err.txt").string();
    ASSERT_EQ(std::system(command.c_str()), 0);
    std::ifstream errors(dir / "err.txt");
    std::string line;
    while (std::getline(errors, line)) {
        EXPECT_NE(line.rfind("note:", 0), 0U) << line;
    }
    // Each part is a collider of the bracket's body.
    std::ifstream usda(dir / "out.usda");
    const std::string text{std::istreambuf_iterator<char>(usda), std::istreambuf_iterator<char>()};
    EXPECT_NE(text.find("bracket"), std::string::npos);
}

// --- the gizmos on Addendum A's shapes ---------------------------------------------------------

TEST(MeshGizmo, CapsuleHasSizeHandles) {
    namespace g = builder::gizmo;
    Shape capsule = builder::defaultShape(Shape::Kind::Capsule);  // r 0.05, length 0.1
    EXPECT_DOUBLE_EQ(*g::extentOf(capsule, 0), 0.1);
    EXPECT_DOUBLE_EQ(*g::extentOf(capsule, 2), 0.2);  // tip to tip
    // The +z tip out by 0.05: the straight part grows, the radius stays, the -z tip stays.
    ASSERT_TRUE(g::resize(capsule, 2, 1, 0.05, g::kMoveStep));
    EXPECT_NEAR(capsule.size[1], 0.15, 1e-12);
    EXPECT_NEAR(capsule.size[0], 0.05, 1e-12);
    EXPECT_NEAR(capsule.origin.xyz[2], 0.025, 1e-12);
    // Across: the radius.
    ASSERT_TRUE(g::resize(capsule, 0, -1, 0.02, g::kMoveStep));
    EXPECT_NEAR(capsule.size[0], 0.06, 1e-12);
    // Pulled shorter than its caps: a straight part of 0, never negative.
    ASSERT_TRUE(g::resize(capsule, 2, 1, -1.0, g::kMoveStep));
    EXPECT_EQ(capsule.size[1], 0.0);
    // A mesh has none (its scale is in Properties).
    EXPECT_FALSE(g::extentOf(meshShape("x.obj"), 0).has_value());
}

TEST(MeshGizmo, PicksCapsulesMeshesAndCollisionOnlyShapes) {
    namespace g = builder::gizmo;
    const auto dir = scratch("gizmo_pick");
    writeObj(dir / "part.obj", surfaceOf(builder::boxMesh({0.2, 0.2, 0.2})));
    Document document = builder::newDocument("pick");
    document.links[0].shapes.clear();
    Shape mesh = meshShape(dir / "part.obj");
    mesh.origin.xyz = {1.0, 0.0, 0.0};
    document.links[0].shapes.push_back(mesh);
    Shape capsule = builder::defaultShape(Shape::Kind::Capsule);
    capsule.origin.xyz = {-1.0, 0.0, 0.0};
    builder::addCollisionShape(document.links[0], capsule);
    const std::vector<builder::Frame> links(1);
    // Down onto the mesh: its top face at z 0.1.
    auto hit = g::pickShape(document, links, {{1.0, 0.0, 5.0}, {0, 0, -1}}, false);
    ASSERT_TRUE(hit.has_value());
    EXPECT_EQ(hit->shape, 0);
    EXPECT_NEAR(hit->point[2], 0.1, 1e-6);
    // Down onto the capsule's top cap (z 0.05 + 0.05), collision only, with the toggle off.
    hit = g::pickShape(document, links, {{-1.0, 0.0, 5.0}, {0, 0, -1}}, false);
    ASSERT_TRUE(hit.has_value());
    EXPECT_EQ(hit->shape, 1);
    EXPECT_NEAR(hit->point[2], 0.1, 1e-9);
    // Move on the mesh's origin: the pose a Move gizmo edits.
    const g::Target target{g::Target::Kind::Shape, "base_link", 0};
    builder::Pose* pose = g::poseOf(document, target);
    ASSERT_NE(pose, nullptr);
    *pose = g::moved(*pose, 1, 0.123, g::kMoveStep);
    EXPECT_NEAR(document.links[0].shapes[0].origin.xyz[1], 0.125, 1e-12);
}
