// convexHullOf on nearly degenerate point sets: every hull closed, wound outward, convex within
// its tolerance, of positive volume, a sphere (V - E + F = 2), and the same bits every time.
#include "lagrange/world/Model.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange {
namespace {

using Point = std::array<double, 3>;

TriMesh meshOf(const std::vector<std::array<float, 3>>& points) {
    TriMesh mesh;
    for (const auto& p : points) {
        mesh.positions.insert(mesh.positions.end(), p.begin(), p.end());
    }
    return mesh;
}

std::vector<Point> scaledPoints(const TriMesh& mesh, const Point& scale) {
    std::vector<Point> out;
    for (std::size_t i = 0; i + 2 < mesh.positions.size(); i += 3) {
        out.push_back({mesh.positions[i] * scale[0], mesh.positions[i + 1] * scale[1],
                       mesh.positions[i + 2] * scale[2]});
    }
    return out;
}

/// Checks a hull against the points it was made of (all of them when there are at most
/// kMaxHullPoints; else its own points only, since the rest were dropped on purpose).
void expectGoodHull(const ConvexHull& hull, const std::vector<Point>& input,
                    const std::string& what) {
    SCOPED_TRACE(what);
    ASSERT_GE(hull.points.size(), 4U);
    ASSERT_GE(hull.triangles.size(), 4U);
    // Every hull point is one of the input points, used by a triangle.
    std::vector<int> used(hull.points.size(), 0);
    for (const auto& t : hull.triangles) {
        for (const std::uint32_t v : t) {
            ASSERT_LT(v, hull.points.size());
            used[v] = 1;
        }
        ASSERT_NE(t[0], t[1]);
        ASSERT_NE(t[1], t[2]);
        ASSERT_NE(t[0], t[2]);
    }
    for (std::size_t i = 0; i < hull.points.size(); ++i) {
        EXPECT_EQ(used[i], 1) << "hull point " << i << " is on no triangle";
        EXPECT_NE(std::find(input.begin(), input.end(), hull.points[i]), input.end());
    }
    // Closed and consistently wound: each directed edge once, and its twin once.
    std::vector<std::pair<std::uint32_t, std::uint32_t>> edges;
    for (const auto& t : hull.triangles) {
        for (std::size_t e = 0; e < 3; ++e) {
            edges.emplace_back(t.at(e), t.at((e + 1) % 3));
        }
    }
    std::sort(edges.begin(), edges.end());
    EXPECT_EQ(std::adjacent_find(edges.begin(), edges.end()), edges.end()) << "a repeated edge";
    for (const auto& [from, to] : edges) {
        EXPECT_TRUE(std::binary_search(edges.begin(), edges.end(), std::pair{to, from}))
            << "edge " << from << "-" << to << " has no twin";
    }
    const auto vertices = static_cast<long>(hull.points.size());
    const auto edgeCount = static_cast<long>(edges.size() / 2);
    const auto faceCount = static_cast<long>(hull.triangles.size());
    EXPECT_EQ(vertices - edgeCount + faceCount, 2) << "not a sphere";
    // Positive volume, about the centre (so that a far offset loses no digits).
    Point centre{};
    for (const Point& p : hull.points) {
        for (std::size_t k = 0; k < 3; ++k) {
            centre.at(k) += p.at(k) / static_cast<double>(hull.points.size());
        }
    }
    const auto from = [&](const Point& p) {
        return Point{p[0] - centre[0], p[1] - centre[1], p[2] - centre[2]};
    };
    double volume = 0.0;
    for (const auto& t : hull.triangles) {
        const Point a = from(hull.points[t[0]]);
        const Point b = from(hull.points[t[1]]);
        const Point c = from(hull.points[t[2]]);
        volume += (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) +
                   a[2] * (b[0] * c[1] - b[1] * c[0])) /
                  6.0;
    }
    EXPECT_GT(volume, 0.0);
    // Convex: every point behind every face's plane, within a few times the hull's tolerance
    // (1e-6 of its size).
    Point lo = input.front();
    Point hi = input.front();
    for (const Point& p : input) {
        for (std::size_t k = 0; k < 3; ++k) {
            lo.at(k) = std::min(lo.at(k), p.at(k));
            hi.at(k) = std::max(hi.at(k), p.at(k));
        }
    }
    const double size = std::hypot(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]);
    const double allowed = 4e-6 * size;
    const std::vector<Point>& check = input.size() <= kMaxHullPoints ? input : hull.points;
    double worst = -1.0;
    for (const auto& t : hull.triangles) {
        const Point& a = hull.points[t[0]];
        const Point& b = hull.points[t[1]];
        const Point& c = hull.points[t[2]];
        const Point u{b[0] - a[0], b[1] - a[1], b[2] - a[2]};
        const Point v{c[0] - a[0], c[1] - a[1], c[2] - a[2]};
        Point n{u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]};
        const double length = std::hypot(n[0], n[1], n[2]);
        ASSERT_GT(length, 0.0) << "a degenerate triangle";
        for (double& x : n) {
            x /= length;
        }
        for (const Point& p : check) {
            const double out = n[0] * (p[0] - a[0]) + n[1] * (p[1] - a[1]) + n[2] * (p[2] - a[2]);
            worst = std::max(worst, out);
        }
    }
    EXPECT_LE(worst, allowed) << "a point is outside the hull, size " << size;
}

/// The hull, checked, and checked to come out the same twice.
std::optional<ConvexHull> checkedHull(const TriMesh& mesh, const Point& scale,
                                      const std::string& what) {
    const std::optional<ConvexHull> hull = convexHullOf(mesh, scale);
    const std::optional<ConvexHull> again = convexHullOf(mesh, scale);
    EXPECT_EQ(hull.has_value(), again.has_value()) << what;
    if (hull && again) {
        EXPECT_EQ(hull->points, again->points) << what;
        EXPECT_EQ(hull->triangles, again->triangles) << what;
        expectGoodHull(*hull, scaledPoints(mesh, scale), what);
    }
    return hull;
}

// A CoACD part of the AMR_FMS swerve module (swerve_module_fixed.dae, part 27): a 2.8 mm slab
// whose top face has one corner 0.3 µm low. Its hull came out open: edges without their twin and
// faces wound inward, folded back over a face the point was level with.
const std::vector<std::array<float, 3>> kSwervePart{
    {-0.037340602f, -0.027172994f, -0.00125425732f},
    {-0.0404191035f, -0.027172994f, -0.0040913645f},
    {-0.074166429f, -0.027172994f, -0.00125425732f},
    {7.98881054e-06f, -0.0462419569f, -0.00125425732f},
    {-0.0712694478f, -0.027172994f, -0.0040913645f},
    {7.98881054e-06f, -0.0486916465f, -0.0040913645f},
    {-0.073338722f, -0.0297040039f, -0.00125425732f},
    {7.98881054e-06f, -0.047100873f, -0.00125425732f},
    {-0.070332824f, -0.0297040039f, -0.0040913645f},
    {7.98881054e-06f, -0.0763091129f, -0.0040913645f},
    {-0.0724819696f, -0.0318894103f, -0.00125425732f},
    {-0.0696575886f, -0.0314726588f, -0.0040913645f},
    {-0.00206854671f, -0.0790840775f, -0.00125425732f},
    {7.98881054e-06f, -0.0790942401f, -0.00125453269f},
    {-0.00210485011f, -0.0790840775f, -0.00125425732f},
    {-0.00430481523f, -0.0790637457f, -0.00125425732f},
    {-0.00210485011f, -0.0762837064f, -0.0040913645f},
    {-0.0703255594f, -0.0363364655f, -0.00125425732f},
    {-0.0674866688f, -0.0358739692f, -0.0040913645f},
    {-0.00650478035f, -0.0790180075f, -0.00125425732f},
    {-0.00430481523f, -0.0762074673f, -0.0040913645f},
    {-0.0680457294f, -0.0406513742f, -0.00125425732f},
    {-0.0651560128f, -0.0400109997f, -0.0040913645f},
    {-0.00870474875f, -0.0789316058f, -0.00125425732f},
    {-0.00650478035f, -0.0760651648f, -0.0040913645f},
    {-0.0636603218f, -0.0472635072f, -0.00125425732f},
    {-0.0633190751f, -0.0427402195f, -0.0040913645f},
    {-0.0109047122f, -0.0787130642f, -0.00125425732f},
    {-0.00870474875f, -0.0758720362f, -0.0040913645f},
    {-0.0606979918f, -0.0465062368f, -0.0040913645f},
    {-0.0587085825f, -0.0489559296f, -0.0040913645f},
    {-0.0566465664f, -0.0512938076f, -0.0040913645f},
    {-0.0615910476f, -0.0496013853f, -0.00125425732f},
    {-0.0145422792f, -0.0750334454f, -0.0040913645f},
    {-0.0140921207f, -0.0751249284f, -0.0040913645f},
    {-0.013104679f, -0.0753028131f, -0.0040913645f},
    {-0.0109047122f, -0.075617919f, -0.0040913645f},
    {-0.0197045743f, -0.0767716092f, -0.00125425732f},
    {-0.0545192146f, -0.0535300401f, -0.0040913645f},
    {-0.0594419086f, -0.0518477836f, -0.00125425732f},
    {-0.0197045743f, -0.0737679404f, -0.0040913645f},
    {-0.0220424917f, -0.0760448331f, -0.00125425732f},
    {-0.0232550135f, -0.0728073788f, -0.0040913645f},
    {-0.0219335848f, -0.0732139677f, -0.0040913645f},
    {-0.052319243f, -0.05569004f, -0.0040913645f},
    {-0.0528492713f, -0.0584497476f, -0.00125425732f},
    {-0.0550637591f, -0.0562643445f, -0.00125425732f},
    {-0.0572564596f, -0.054058603f, -0.00125425732f},
    {-0.0263044697f, -0.07460145f, -0.00125425732f},
    {-0.0283664858f, -0.0709574068f, -0.0040913645f},
    {-0.0500248903f, -0.0577788788f, -0.0040913645f},
    {-0.0483404309f, -0.0627443343f, -0.00125425732f},
    {-0.0506202608f, -0.0606199133f, -0.00125425732f},
    {-0.028584303f, -0.0737577778f, -0.00125425732f},
    {-0.0330858821f, -0.0718722236f, -0.00125425732f},
    {-0.0329769751f, -0.0689092183f, -0.0040913645f},
    {-0.0476361501f, -0.0598067355f, -0.0040913645f},
    {-0.0458137351f, -0.0646146381f, -0.00125425732f},
    {-0.0374785542f, -0.0695089358f, -0.00125425732f},
    {-0.0372462144f, -0.0666984022f, -0.0040913645f},
    {-0.0412685946f, -0.0643300265f, -0.0040913645f},
    {-0.0417768356f, -0.0671761358f, -0.00125425732f}};

TEST(ConvexHull, SwerveModulePartIsClosedAndConvex) {
    for (const double scale : {1.0, 0.001, 1000.0}) {
        const std::optional<ConvexHull> hull = checkedHull(
            meshOf(kSwervePart), {scale, scale, scale}, "scale " + std::to_string(scale));
        ASSERT_TRUE(hull);
    }
}

/// A small deterministic generator (SplitMix64), so the sets are the same on every platform.
class Random {
public:
    explicit Random(std::uint64_t seed) : state_(seed) {}
    std::uint64_t next() {
        std::uint64_t z = (state_ += 0x9E37'79B9'7F4A'7C15ULL);
        z = (z ^ (z >> 30U)) * 0xBF58'476D'1CE4'E5B9ULL;
        z = (z ^ (z >> 27U)) * 0x94D0'49BB'1331'11EBULL;
        return z ^ (z >> 31U);
    }
    /// Uniform in [lo, hi).
    double uniform(double lo, double hi) {
        return lo + (hi - lo) * static_cast<double>(next() >> 11U) * 0x1.0p-53;
    }
    std::size_t below(std::size_t n) { return static_cast<std::size_t>(next() % n); }

private:
    std::uint64_t state_;
};

using Cloud = std::vector<std::array<float, 3>>;

std::array<float, 3> at(double x, double y, double z) {
    return {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
}

/// Points on the faces, edges and corners of a box: many exactly coplanar and collinear.
Cloud boxSurface(Random& random, std::size_t count) {
    Cloud out;
    for (int k = 0; k < 8; ++k) {
        out.push_back(
            at((k & 1) != 0 ? 0.5 : -0.5, (k & 2) != 0 ? 0.3 : -0.3, (k & 4) != 0 ? 0.2 : -0.2));
    }
    const std::array<double, 3> half{0.5, 0.3, 0.2};
    while (out.size() < count) {
        std::array<double, 3> p{random.uniform(-half[0], half[0]),
                                random.uniform(-half[1], half[1]),
                                random.uniform(-half[2], half[2])};
        const std::size_t axis = random.below(3);
        p.at(axis) = random.below(2) == 0 ? -half.at(axis) : half.at(axis);
        if (random.below(3) == 0) {  // on an edge
            const std::size_t other = (axis + 1 + random.below(2)) % 3;
            p.at(other) = random.below(2) == 0 ? -half.at(other) : half.at(other);
        }
        out.push_back(at(p[0], p[1], p[2]));
    }
    return out;
}

/// A slab `thickness` thick whose faces are flat to within `ripple` (a CoACD part's look).
Cloud slab(Random& random, std::size_t count, double thickness, double ripple) {
    Cloud out;
    while (out.size() < count) {
        const double x = random.uniform(-0.05, 0.05);
        const double y = random.uniform(-0.03, 0.03);
        const double side = random.below(2) == 0 ? -thickness / 2.0 : thickness / 2.0;
        const double bump = random.below(4) == 0 ? random.uniform(-ripple, ripple) : 0.0;
        out.push_back(at(x, y, side + bump));
        if (random.below(5) == 0) {  // a rim point, straight above or below another
            out.push_back(at(x, y, -side));
        }
    }
    return out;
}

/// Points on a sphere, some of them twice or a hair apart.
Cloud sphere(Random& random, std::size_t count, double radius) {
    Cloud out;
    while (out.size() < count) {
        const double z = random.uniform(-1.0, 1.0);
        const double a = random.uniform(0.0, 2.0 * std::numbers::pi);
        const double r = std::sqrt(1.0 - z * z);
        out.push_back(at(radius * r * std::cos(a), radius * r * std::sin(a), radius * z));
        if (random.below(6) == 0) {
            out.push_back(out.back());
        }
        if (random.below(6) == 0) {
            auto p = out.back();
            p[0] = std::nextafter(p[0], 1.0F);
            out.push_back(p);
        }
    }
    return out;
}

/// A convex part cut from a grid: points on a coarse lattice inside a random set of planes,
/// so many points are coplanar and collinear, with a few nudged off their plane.
Cloud latticePart(Random& random, std::size_t count) {
    std::vector<std::array<double, 4>> planes;
    for (int i = 0; i < 8; ++i) {
        const double z = random.uniform(-1.0, 1.0);
        const double a = random.uniform(0.0, 2.0 * std::numbers::pi);
        const double r = std::sqrt(1.0 - z * z);
        planes.push_back({r * std::cos(a), r * std::sin(a), z, random.uniform(0.3, 0.6)});
    }
    Cloud out;
    std::size_t tries = 0;
    while (out.size() < count && tries++ < 100000) {
        constexpr double kStep = 0.05;
        std::array<double, 3> p{std::round(random.uniform(-1.0, 1.0) / kStep) * kStep,
                                std::round(random.uniform(-1.0, 1.0) / kStep) * kStep,
                                std::round(random.uniform(-1.0, 1.0) / kStep) * kStep};
        bool inside = true;
        for (const auto& plane : planes) {
            inside = inside && plane[0] * p[0] + plane[1] * p[1] + plane[2] * p[2] <= plane[3];
        }
        if (!inside) {
            continue;
        }
        if (random.below(8) == 0) {
            p.at(random.below(3)) += random.uniform(-1e-6, 1e-6);
        }
        out.push_back(at(p[0], p[1], p[2]));
    }
    return out;
}

/// Shuffles in place (so the same shape is met in another order) and moves it far off.
Cloud shuffledAndMoved(Random& random, Cloud cloud, double offset) {
    for (std::size_t i = cloud.size(); i > 1; --i) {
        std::swap(cloud[i - 1], cloud[random.below(i)]);
    }
    for (auto& p : cloud) {
        for (float& x : p) {
            x = static_cast<float>(x + offset);
        }
    }
    return cloud;
}

TEST(ConvexHull, NearlyDegenerateSetsAreClosedConvexSpheres) {
    std::size_t made = 0;
    for (std::uint64_t seed = 1; seed <= 400; ++seed) {
        Random random(seed);
        const std::string what = "seed " + std::to_string(seed);
        std::vector<std::pair<std::string, Cloud>> clouds;
        clouds.emplace_back("box", boxSurface(random, 8 + random.below(120)));
        clouds.emplace_back(
            "slab", slab(random, 8 + random.below(120), std::pow(10.0, random.uniform(-4.5, -2.0)),
                         std::pow(10.0, random.uniform(-9.0, -6.0))));
        clouds.emplace_back("sphere", sphere(random, 6 + random.below(200),
                                             std::pow(10.0, random.uniform(-2.0, 1.0))));
        clouds.emplace_back("lattice", latticePart(random, 6 + random.below(150)));
        for (const auto& [name, cloud] : clouds) {
            for (const double offset : {0.0, 3.0}) {
                const Cloud moved = shuffledAndMoved(random, cloud, offset);
                const double scale = std::pow(10.0, random.uniform(-3.0, 3.0));
                const auto hull =
                    checkedHull(meshOf(moved), {scale, scale, scale},
                                what + " " + name + " offset " + std::to_string(offset));
                made += hull ? 1U : 0U;
                if (::testing::Test::HasFailure()) {
                    return;  // one failing set is enough to read
                }
            }
        }
    }
    EXPECT_GT(made, 2800U);  // nearly all of the 3200 sets have a volume
}

TEST(ConvexHull, ManyPointsAreCutDownAndStayClosed) {
    for (std::uint64_t seed = 1; seed <= 20; ++seed) {
        Random random(seed);
        const auto hull = checkedHull(meshOf(sphere(random, 600 + random.below(600), 0.2)),
                                      {1.0, 2.0, 0.5}, "seed " + std::to_string(seed));
        ASSERT_TRUE(hull);
        EXPECT_LE(hull->points.size(), kMaxHullPoints);
    }
}

TEST(ConvexHull, FlatOrTooFewPointsHaveNone) {
    EXPECT_FALSE(convexHullOf(meshOf({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}})));
    EXPECT_FALSE(convexHullOf(meshOf({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 0}})));
    EXPECT_FALSE(convexHullOf(meshOf({{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}})));
}

}  // namespace
}  // namespace lagrange
