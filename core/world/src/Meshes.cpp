// Primitive meshes for the viewer, the same shapes and vertex layout as the mock server's
// meshes.py, so a robot looks the same whichever server shows it.
#include "lagrange/world/Model.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <numbers>
#include <optional>
#include <utility>
#include <vector>

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

namespace {

using Point = std::array<double, 3>;

Point minus(const Point& a, const Point& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
Point crossOf(const Point& a, const Point& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
double dotOf(const Point& a, const Point& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

struct HullFace {
    std::array<std::uint32_t, 3> v;
    Point normal;  // unit, outward
    double offset;
};

/// An edge of the rim of the faces a new point sees, wound as in the seen face, and the face
/// beyond it that stays.
struct RimEdge {
    std::uint32_t from;
    std::uint32_t to;
    std::size_t beyond;
};

/// Grows the faces point `p` sees (`seen`: at first those it is over the tolerance above) until
/// replacing them by a fan from `p` keeps the hull a closed, convex surface: they are one piece
/// with one rim loop, and no fan face is thinner than the tolerance or has a hull vertex more
/// than the tolerance in front of it. The faces it adds are ones `p` is level with (within the
/// tolerance of their planes), next to those seen: a point that sees one face of a nearly flat
/// pair and lies over the other must replace both, or its fan folds back over the other. Fills
/// `rim` in the order of the faces and their edges. False when no such region exists: `p` is
/// then left out.
bool settleSeen(const std::vector<HullFace>& faces, const std::vector<Point>& points,
                std::uint32_t p, double tolerance, std::vector<char>& seen,
                std::vector<RimEdge>& rim) {
    // Each directed edge and its face, to find the face beyond an edge.
    std::vector<std::array<std::size_t, 3>> owners;
    owners.reserve(3 * faces.size());
    for (std::size_t i = 0; i < faces.size(); ++i) {
        for (std::size_t e = 0; e < 3; ++e) {
            owners.push_back({faces[i].v.at(e), faces[i].v.at((e + 1) % 3), i});
        }
    }
    std::sort(owners.begin(), owners.end());
    const auto beyondOf = [&](std::size_t i, std::size_t e) -> std::size_t {
        const std::array<std::size_t, 3> key{faces[i].v.at((e + 1) % 3), faces[i].v.at(e), 0};
        const auto it = std::lower_bound(owners.begin(), owners.end(), key);
        if (it == owners.end() || (*it)[0] != key[0] || (*it)[1] != key[1]) {
            return faces.size();
        }
        return (*it)[2];
    };
    std::vector<double> height(faces.size());
    for (std::size_t i = 0; i < faces.size(); ++i) {
        height[i] = dotOf(faces[i].normal, points[p]) - faces[i].offset;
    }
    // The faces joined by edges to `start` through faces that pass `through`.
    const auto region = [&](std::size_t start, const auto& through) {
        std::vector<char> reached(faces.size(), 0);
        std::vector<std::size_t> queue{start};
        reached[start] = 1;
        while (!queue.empty()) {
            const std::size_t i = queue.back();
            queue.pop_back();
            for (std::size_t e = 0; e < 3; ++e) {
                const std::size_t j = beyondOf(i, e);
                if (j < faces.size() && reached[j] == 0 && through(j)) {
                    reached[j] = 1;
                    queue.push_back(j);
                }
            }
        }
        return reached;
    };
    std::size_t top = faces.size();
    for (std::size_t i = 0; i < faces.size(); ++i) {
        if (seen[i] != 0 && (top == faces.size() || height[i] > height[top])) {
            top = i;
        }
    }
    if (top == faces.size()) {
        return false;
    }
    // One piece: the seen faces joined to the one it is farthest above, through faces it sees
    // or is level with. Every later step adds faces next to these, so it stays one piece.
    {
        const std::vector<char> joined =
            region(top, [&](std::size_t j) { return seen[j] != 0 || height[j] > -tolerance; });
        bool apart = false;
        for (std::size_t i = 0; i < faces.size(); ++i) {
            apart = apart || (seen[i] != 0 && joined[i] == 0);
        }
        if (apart) {
            for (std::size_t i = 0; i < faces.size(); ++i) {
                seen[i] = joined[i];
            }
        }
    }
    // Takes in the faces next to the seen ones that it is level with; false when there are none.
    const auto takeLevelNeighbours = [&]() {
        std::vector<std::size_t> add;
        for (std::size_t i = 0; i < faces.size(); ++i) {
            if (seen[i] == 0) {
                continue;
            }
            for (std::size_t e = 0; e < 3; ++e) {
                const std::size_t j = beyondOf(i, e);
                if (j < faces.size() && seen[j] == 0 && height[j] > -tolerance) {
                    add.push_back(j);
                }
            }
        }
        for (const std::size_t j : add) {
            seen[j] = 1;
        }
        return !add.empty();
    };
    constexpr double kFoldSlack = 2.0;
    std::vector<char> onHull(points.size(), 0);
    for (std::size_t round = 0; round <= faces.size(); ++round) {
        rim.clear();
        for (std::size_t i = 0; i < faces.size(); ++i) {
            if (seen[i] == 0) {
                continue;
            }
            for (std::size_t e = 0; e < 3; ++e) {
                const std::size_t beyond = beyondOf(i, e);
                if (beyond == faces.size()) {
                    return false;  // not a closed surface
                }
                if (seen[beyond] == 0) {
                    rim.push_back({faces[i].v.at(e), faces[i].v.at((e + 1) % 3), beyond});
                }
            }
        }
        if (rim.empty()) {
            return false;  // it sees every face
        }
        // One rim loop: each rim vertex starts one edge, and following them goes round all.
        std::vector<std::pair<std::uint32_t, std::size_t>> starts;
        starts.reserve(rim.size());
        for (std::size_t k = 0; k < rim.size(); ++k) {
            starts.emplace_back(rim[k].from, k);
        }
        std::sort(starts.begin(), starts.end());
        bool pinched = false;
        for (std::size_t k = 1; k < starts.size(); ++k) {
            pinched = pinched || starts[k].first == starts[k - 1].first;
        }
        std::size_t length = 0;
        if (!pinched) {
            std::size_t k = 0;
            do {
                const auto it =
                    std::lower_bound(starts.begin(), starts.end(),
                                     std::pair<std::uint32_t, std::size_t>{rim[k].to, 0});
                if (it == starts.end() || it->first != rim[k].to) {
                    break;
                }
                k = it->second;
                ++length;
            } while (k != 0 && length <= rim.size());
        }
        if (pinched || length != rim.size()) {
            // Fill the holes: the faces cut off from the one it is farthest below.
            std::size_t bottom = faces.size();
            for (std::size_t i = 0; i < faces.size(); ++i) {
                if (seen[i] == 0 && (bottom == faces.size() || height[i] < height[bottom])) {
                    bottom = i;
                }
            }
            const std::vector<char> outside =
                region(bottom, [&](std::size_t j) { return seen[j] == 0; });
            bool changed = false;
            for (std::size_t i = 0; i < faces.size(); ++i) {
                if (seen[i] == 0 && outside[i] == 0) {
                    seen[i] = 1;
                    changed = true;
                }
            }
            if (!changed && !takeLevelNeighbours()) {
                return false;
            }
            continue;
        }
        // No fan face thinner than the tolerance or folded over the face beyond its edge.
        bool changed = false;
        for (const RimEdge& edge : rim) {
            const Point& a = points[edge.from];
            const Point along = minus(points[edge.to], a);
            const Point normal = crossOf(along, minus(points[p], a));
            const double area = std::sqrt(dotOf(normal, normal));
            bool grow = area <= tolerance * std::sqrt(dotOf(along, along));
            for (const std::uint32_t w : faces[edge.beyond].v) {
                grow = grow || dotOf(normal, minus(points[w], a)) > tolerance * area;
            }
            if (grow) {
                seen[edge.beyond] = 1;
                changed = true;
            }
        }
        if (changed) {
            continue;
        }
        // Nor one with any hull vertex in front of it: p sees a face only through the tilt of its
        // plane far from it (a flat side with one corner a hair off), and its fan would cover
        // the faces beside. Take in the faces next to the seen ones that it is level with.
        std::fill(onHull.begin(), onHull.end(), 0);
        for (std::size_t i = 0; i < faces.size(); ++i) {
            if (seen[i] == 0) {
                for (const std::uint32_t v : faces[i].v) {
                    onHull[v] = 1;
                }
            }
        }
        double fold = 0.0;  // the most a hull vertex is in front of a fan face, m
        for (const RimEdge& edge : rim) {
            const Point& a = points[edge.from];
            const Point normal = crossOf(minus(points[edge.to], a), minus(points[p], a));
            const double area = std::sqrt(dotOf(normal, normal));
            for (std::uint32_t w = 0; w < points.size(); ++w) {
                if (onHull[w] != 0) {
                    fold = std::max(fold, dotOf(normal, minus(points[w], a)) / area);
                }
            }
        }
        if (fold <= tolerance) {
            return true;
        }
        if (!takeLevelNeighbours()) {
            // Nothing level left to take in: a fold of a hair over the tolerance (four nearly
            // coplanar points) is kept rather than leaving out a point that is far outside.
            return fold <= kFoldSlack * tolerance;
        }
    }
    return false;
}

/// A fan face from a rim edge to a new point, wound as the rim edge.
HullFace fanFace(const std::vector<Point>& points, std::uint32_t p, std::uint32_t q,
                 std::uint32_t r) {
    Point n = crossOf(minus(points[q], points[p]), minus(points[r], points[p]));
    const double length = std::sqrt(dotOf(n, n));
    if (length > 0.0) {
        n = {n[0] / length, n[1] / length, n[2] / length};
    }
    return HullFace{{p, q, r}, n, dotOf(n, points[p])};
}

/// The hull grown from `start` by each point in turn: the faces it is over the tolerance above
/// are replaced by a fan from their rim, nothing checked. Empty once it has too many faces.
std::vector<HullFace> growInOrder(const std::vector<Point>& points,
                                  const std::vector<HullFace>& start, double tolerance) {
    std::vector<HullFace> faces = start;
    for (std::uint32_t p = 0; p < points.size(); ++p) {
        std::vector<std::pair<std::uint32_t, std::uint32_t>> edges;
        std::vector<HullFace> kept;
        kept.reserve(faces.size());
        for (const HullFace& f : faces) {
            if (dotOf(f.normal, points[p]) - f.offset > tolerance) {
                for (std::size_t e = 0; e < 3; ++e) {
                    edges.emplace_back(f.v.at(e), f.v.at((e + 1) % 3));
                }
            } else {
                kept.push_back(f);
            }
        }
        if (edges.empty()) {
            continue;
        }
        for (const auto& [from, to] : edges) {
            if (std::find(edges.begin(), edges.end(), std::pair{to, from}) == edges.end()) {
                kept.push_back(fanFace(points, from, to, p));
            }
        }
        faces = std::move(kept);
        if (faces.size() > 2 * points.size()) {
            return {};  // more than a hull of these points has (2 V - 4): it went wrong
        }
    }
    return faces;
}

/// The hull grown from `start` farthest point first (quickhull's order): the point farthest
/// above a face replaces the faces it sees, grown by settleSeen into a region whose fan keeps
/// the hull closed and convex, until no point is more than the tolerance above a face. A point
/// no region fits is left out.
std::vector<HullFace> growFarthestFirst(const std::vector<Point>& points,
                                        const std::vector<HullFace>& start, double tolerance) {
    std::vector<HullFace> faces = start;
    std::vector<char> done(points.size(), 0);
    for (const HullFace& f : faces) {
        for (const std::uint32_t v : f.v) {
            done[v] = 1;
        }
    }
    std::vector<RimEdge> rim;
    for (;;) {
        std::uint32_t farthest = 0;
        double most = tolerance;
        for (std::uint32_t q = 0; q < points.size(); ++q) {
            if (done[q] != 0) {
                continue;
            }
            for (const HullFace& f : faces) {
                const double height = dotOf(f.normal, points[q]) - f.offset;
                if (height > most) {
                    most = height;
                    farthest = q;
                }
            }
        }
        if (most <= tolerance) {
            return faces;
        }
        const std::uint32_t p = farthest;
        done[p] = 1;
        std::vector<char> sees(faces.size(), 0);
        for (std::size_t i = 0; i < faces.size(); ++i) {
            sees[i] = dotOf(faces[i].normal, points[p]) - faces[i].offset > tolerance ? 1 : 0;
        }
        if (!settleSeen(faces, points, p, tolerance, sees, rim)) {
            continue;
        }
        std::vector<HullFace> kept;
        kept.reserve(faces.size() + rim.size());
        for (std::size_t i = 0; i < faces.size(); ++i) {
            if (sees[i] == 0) {
                kept.push_back(faces[i]);
            }
        }
        for (const RimEdge& edge : rim) {
            kept.push_back(fanFace(points, edge.from, edge.to, p));
        }
        faces = std::move(kept);
    }
}

/// Whether `faces` close round the points: each edge once each way, a sphere (V - E + F = 2),
/// every face a plane with every point behind it within a few times the tolerance, and a
/// volume.
bool isClosedConvexHull(const std::vector<HullFace>& faces, const std::vector<Point>& points,
                        double tolerance) {
    constexpr double kSlack = 3.0;  // a point skipped at the tolerance, under a later face
    if (faces.size() < 4) {
        return false;
    }
    std::vector<std::pair<std::uint32_t, std::uint32_t>> edges;
    edges.reserve(3 * faces.size());
    std::vector<char> used(points.size(), 0);
    for (const HullFace& f : faces) {
        if (std::abs(dotOf(f.normal, f.normal) - 1.0) > 1e-9) {
            return false;  // a face without a plane
        }
        for (std::size_t e = 0; e < 3; ++e) {
            edges.emplace_back(f.v.at(e), f.v.at((e + 1) % 3));
            used[f.v.at(e)] = 1;
        }
    }
    std::sort(edges.begin(), edges.end());
    if (std::adjacent_find(edges.begin(), edges.end()) != edges.end()) {
        return false;
    }
    for (const auto& [from, to] : edges) {
        if (!std::binary_search(edges.begin(), edges.end(), std::pair{to, from})) {
            return false;
        }
    }
    const auto vertices = static_cast<std::ptrdiff_t>(std::count(used.begin(), used.end(), 1));
    const auto edgeCount = static_cast<std::ptrdiff_t>(edges.size() / 2);
    if (vertices - edgeCount + static_cast<std::ptrdiff_t>(faces.size()) != 2) {
        return false;
    }
    for (const HullFace& f : faces) {
        for (const Point& p : points) {
            if (dotOf(f.normal, p) - f.offset > kSlack * tolerance) {
                return false;
            }
        }
    }
    // Closed, outward and convex: the volume about any vertex is positive.
    const Point& o = points[faces.front().v[0]];
    double volume = 0.0;
    for (const HullFace& f : faces) {
        volume += dotOf(minus(points[f.v[0]], o),
                        crossOf(minus(points[f.v[1]], o), minus(points[f.v[2]], o)));
    }
    return volume > 0.0;
}

/// The farthest vertex in each of kMaxHullPoints directions spread over the sphere (a Fibonacci
/// lattice), in the order of the vertices. Measured in the points' bounding box scaled to a cube,
/// so that a flat shape keeps its faces as well as its rim.
std::vector<Point> farthestPoints(const std::vector<Point>& points) {
    Point lo = points.front();
    Point hi = points.front();
    for (const Point& p : points) {
        for (std::size_t k = 0; k < 3; ++k) {
            lo.at(k) = std::min(lo.at(k), p.at(k));
            hi.at(k) = std::max(hi.at(k), p.at(k));
        }
    }
    Point inverse{};
    for (std::size_t k = 0; k < 3; ++k) {
        inverse.at(k) = hi.at(k) > lo.at(k) ? 1.0 / (hi.at(k) - lo.at(k)) : 0.0;
    }
    const auto reach = [&](const Point& p, const Point& direction) {
        return (p[0] - lo[0]) * inverse[0] * direction[0] +
               (p[1] - lo[1]) * inverse[1] * direction[1] +
               (p[2] - lo[2]) * inverse[2] * direction[2];
    };
    const double golden = std::numbers::pi * (3.0 - std::sqrt(5.0));
    std::vector<bool> keep(points.size(), false);
    for (std::size_t k = 0; k < kMaxHullPoints; ++k) {
        const double z = 1.0 - 2.0 * (static_cast<double>(k) + 0.5) / kMaxHullPoints;
        const double r = std::sqrt(1.0 - z * z);
        const double a = golden * static_cast<double>(k);
        const Point direction{r * std::cos(a), r * std::sin(a), z};
        std::size_t best = 0;
        for (std::size_t i = 1; i < points.size(); ++i) {
            if (reach(points[i], direction) > reach(points[best], direction)) {
                best = i;
            }
        }
        keep[best] = true;
    }
    std::vector<Point> out;
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (keep[i]) {
            out.push_back(points[i]);
        }
    }
    return out;
}

}  // namespace

std::optional<ConvexHull> convexHullOf(const TriMesh& mesh, const std::array<double, 3>& scale) {
    std::vector<Point> points;
    points.reserve(mesh.positions.size() / 3);
    for (std::size_t i = 0; i + 2 < mesh.positions.size(); i += 3) {
        points.push_back({mesh.positions[i] * scale[0], mesh.positions[i + 1] * scale[1],
                          mesh.positions[i + 2] * scale[2]});
    }
    // A mesh repeats its vertices per face: once each, keeping the first's place.
    std::vector<Point> sorted = points;
    std::sort(sorted.begin(), sorted.end());
    sorted.erase(std::unique(sorted.begin(), sorted.end()), sorted.end());
    if (sorted.size() < 4) {
        return std::nullopt;
    }
    std::vector<bool> seen(sorted.size(), false);
    std::vector<Point> unique;
    unique.reserve(sorted.size());
    for (const Point& p : points) {
        const auto at = static_cast<std::size_t>(std::lower_bound(sorted.begin(), sorted.end(), p) -
                                                 sorted.begin());
        if (!seen[at]) {
            seen[at] = true;
            unique.push_back(p);
        }
    }
    points = std::move(unique);
    if (points.size() > kMaxHullPoints) {
        points = farthestPoints(points);
    }
    // A first tetrahedron, of the farthest points: along x, from that line, from that plane.
    std::size_t a = 0;
    std::size_t b = 0;
    for (std::size_t i = 1; i < points.size(); ++i) {
        a = points[i][0] < points[a][0] ? i : a;
        b = points[i][0] > points[b][0] ? i : b;
    }
    double extent = 0.0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        const Point d = minus(points[i], points[a]);
        const double length = dotOf(d, d);
        if (length > extent) {
            extent = length;
            b = i;
        }
    }
    extent = std::sqrt(extent);
    const double tolerance = 1e-6 * extent;
    if (extent <= 0.0) {
        return std::nullopt;
    }
    std::size_t c = a;
    double farthest = 0.0;
    const Point ab = minus(points[b], points[a]);
    for (std::size_t i = 0; i < points.size(); ++i) {
        const Point across = crossOf(ab, minus(points[i], points[a]));
        const double area = std::sqrt(dotOf(across, across)) / extent;
        if (area > farthest) {
            farthest = area;
            c = i;
        }
    }
    if (farthest <= tolerance) {
        return std::nullopt;
    }
    Point up = crossOf(ab, minus(points[c], points[a]));
    const double upLength = std::sqrt(dotOf(up, up));
    up = {up[0] / upLength, up[1] / upLength, up[2] / upLength};
    std::size_t d = a;
    farthest = 0.0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        const double height = std::abs(dotOf(up, minus(points[i], points[a])));
        if (height > farthest) {
            farthest = height;
            d = i;
        }
    }
    if (farthest <= tolerance) {
        return std::nullopt;
    }
    const Point inside{(points[a][0] + points[b][0] + points[c][0] + points[d][0]) / 4.0,
                       (points[a][1] + points[b][1] + points[c][1] + points[d][1]) / 4.0,
                       (points[a][2] + points[b][2] + points[c][2] + points[d][2]) / 4.0};
    const auto face = [&](std::uint32_t p, std::uint32_t q, std::uint32_t r) {
        Point n = crossOf(minus(points[q], points[p]), minus(points[r], points[p]));
        const double length = std::sqrt(dotOf(n, n));
        if (length > 0.0) {
            n = {n[0] / length, n[1] / length, n[2] / length};
        }
        HullFace f{{p, q, r}, n, dotOf(n, points[p])};
        if (dotOf(n, inside) > f.offset) {  // wound inward: turn it round
            f.v = {p, r, q};
            f.normal = {-n[0], -n[1], -n[2]};
            f.offset = -f.offset;
        }
        return f;
    };
    const auto ia = static_cast<std::uint32_t>(a);
    const auto ib = static_cast<std::uint32_t>(b);
    const auto ic = static_cast<std::uint32_t>(c);
    const auto id = static_cast<std::uint32_t>(d);
    const std::vector<HullFace> start{face(ia, ib, ic), face(ia, ib, id), face(ia, ic, id),
                                      face(ib, ic, id)};
    // First as convexHullOf always has: each point in turn, outside the hull so far, replaces
    // the faces it sees by a fan from their rim. Kept for the hulls it makes well, so that they
    // stay the same to the bit; on a nearly flat set its fan can fold back over a face the point
    // is level with, and the result is no hull at all (a CoACD part of the AMR_FMS swerve module
    // came out open, with faces wound inward). Then the careful way: farthest point first, each fan
    // checked (settleSeen), with a coarser tolerance while that still fails.
    std::vector<HullFace> faces = growInOrder(points, start, tolerance);
    if (!isClosedConvexHull(faces, points, tolerance)) {
        constexpr std::array<double, 4> kCoarser{1.0, 4.0, 16.0, 64.0};
        bool closed = false;
        for (const double coarser : kCoarser) {
            faces = growFarthestFirst(points, start, coarser * tolerance);
            closed = isClosedConvexHull(faces, points, coarser * tolerance);
            if (closed) {
                break;
            }
        }
        if (!closed) {
            return std::nullopt;
        }
    }
    // Only the points on the hull, in their order.
    std::vector<std::uint32_t> index(points.size(), kNoIndex);
    for (const HullFace& f : faces) {
        for (const std::uint32_t v : f.v) {
            index[v] = 0;
        }
    }
    ConvexHull hull;
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (index[i] == 0) {
            index[i] = static_cast<std::uint32_t>(hull.points.size());
            hull.points.push_back(points[i]);
        }
    }
    for (const HullFace& f : faces) {
        hull.triangles.push_back({index[f.v[0]], index[f.v[1]], index[f.v[2]]});
    }
    return hull;
}

ConvexHull ellipsoidHull(const std::array<double, 3>& radii) {
    constexpr int kSegments = 24;  // 218 points: under kMaxHullPoints
    constexpr int kRings = 10;
    return convexHullOf(sphereMesh(1.0, kSegments, kRings), radii).value_or(ConvexHull{});
}

double hullFill(const TriMesh& mesh, const std::array<double, 3>& scale, const ConvexHull& hull) {
    double hullVolume = 0.0;
    for (const auto& t : hull.triangles) {
        hullVolume += dotOf(hull.points[t[0]], crossOf(hull.points[t[1]], hull.points[t[2]])) / 6.0;
    }
    double meshVolume = 0.0;
    const auto at = [&](std::uint32_t i) -> Point {
        return {mesh.positions[3 * i] * scale[0], mesh.positions[3 * i + 1] * scale[1],
                mesh.positions[3 * i + 2] * scale[2]};
    };
    for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        meshVolume +=
            dotOf(at(mesh.indices[i]), crossOf(at(mesh.indices[i + 1]), at(mesh.indices[i + 2]))) /
            6.0;
    }
    return hullVolume > 0.0 ? std::abs(meshVolume) / hullVolume : 0.0;
}

TriMesh flatMesh(const ConvexHull& surface) {
    TriMesh out;
    for (const auto& t : surface.triangles) {
        const Point& p = surface.points[t[0]];
        const Point& q = surface.points[t[1]];
        const Point& r = surface.points[t[2]];
        Point n = crossOf(minus(q, p), minus(r, p));
        const double length = std::sqrt(dotOf(n, n));
        if (length > 0.0) {
            n = {n[0] / length, n[1] / length, n[2] / length};
        }
        for (const Point* v : {&p, &q, &r}) {
            out.indices.push_back(static_cast<std::uint32_t>(out.positions.size() / 3));
            for (std::size_t k = 0; k < 3; ++k) {
                out.positions.push_back(static_cast<float>(v->at(k)));
                out.normals.push_back(static_cast<float>(n.at(k)));
            }
        }
    }
    return out;
}

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
    const auto firstJoint = static_cast<std::uint32_t>(w.joints.names.size());
    for (std::size_t j = 0; j < o.joints.names.size(); ++j) {
        w.addJoint(o.joints.names[j], o.joints.type[j], map(o.joints.child[j]), o.joints.axis[j],
                   o.joints.actuated[j]);
        w.joints.lowerLimit.back() = o.joints.lowerLimit[j];
        w.joints.upperLimit.back() = o.joints.upperLimit[j];
        w.joints.effortLimit.back() = o.joints.effortLimit[j];
        w.joints.velocityLimit.back() = o.joints.velocityLimit[j];
        w.joints.armature.back() = o.joints.armature[j];
        w.joints.damping.back() = o.joints.damping[j];
        w.joints.friction.back() = o.joints.friction[j];
        w.joints.mimic.back() =
            o.joints.mimic[j] == kNoMimic ? kNoMimic : firstJoint + o.joints.mimic[j];
        w.joints.mimicMultiplier.back() = o.joints.mimicMultiplier[j];
        w.joints.mimicOffset.back() = o.joints.mimicOffset[j];
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
    // Their surfaces: the colliders above in order, each with its material appended.
    {
        const auto materialOffset = static_cast<std::uint32_t>(w.surfaceMaterials.size());
        w.surfaceMaterials.insert(w.surfaceMaterials.end(), o.surfaceMaterials.begin(),
                                  o.surfaceMaterials.end());
        std::size_t added = w.colliders.material.size();
        for (std::size_t c = o.colliders.body.size(); c-- > 0;) {
            if (o.colliders.body[c] == kWorldBody && o.colliders.shape[c] == ShapeType::Plane) {
                continue;
            }
            --added;
            if (o.colliders.material[c] != kNoMaterial) {
                w.colliders.material[added] = o.colliders.material[c] + materialOffset;
            }
        }
    }
    for (std::size_t s = 0; s < o.sensors.names.size(); ++s) {
        w.addSensor(o.sensors.names[s], o.sensors.type[s], map(o.sensors.body[s]),
                    o.sensors.poseInBody[s], o.sensors.rateHz[s], o.sensors.settings[s]);
        if (s < o.sensors.kind.size()) {
            w.sensors.kind.back() = o.sensors.kind[s];
            w.sensors.text.back() = o.sensors.text[s];
        }
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
            // Where it is (EMI sources, radios, people): its body and pose come with it.
            w.prims.body.at(worldPrimMap[p.index].index) = map(o.prims.body.at(p.index));
            w.prims.poseInBody.at(worldPrimMap[p.index].index) = o.prims.poseInBody.at(p.index);
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
                c.material.erase(c.material.begin() + at);
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
        // Nothing commands a passive robot: its motors make no torque, in any mode.
        for (SchemaInstance& instance : w.schemaInstances) {
            if (instance.schema == "ElectricalMotorAPI_1") {
                instance.tokens["elec:controlMode"] = "torque";
            }
        }
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
    // Components (a motor, a bus, a battery): their signals must not meet another robot's.
    for (const SchemaInstance& instance : w.schemaInstances) {
        if (instance.prim.index < w.prims.instanceNames.size()) {
            std::string& name = w.prims.instanceNames[instance.prim.index];
            if (!name.starts_with(front)) {
                name.insert(0, front);
            }
        }
    }
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
    c.material.resize(extent.colliders);
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
        // A tile's surface: one table entry per collider that has one (tiles are truncated away
        // and the table only grows by what they name; a run has few tiles with materials).
        if (o.colliders.material[c] != kNoMaterial) {
            w.surfaceMaterials.push_back(o.surfaceMaterials.at(o.colliders.material[c]));
            w.colliders.material.back() = static_cast<std::uint32_t>(w.surfaceMaterials.size() - 1);
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

bool surfaceLookPreset(std::string_view name, SurfaceLook& out) {
    struct Row {
        std::string_view name;
        float metallic, roughness;
    };
    static constexpr std::array<Row, 34> kLooks{{
        {"chrome", 1.0F, 0.05F},    {"mirror", 1.0F, 0.02F},   {"gold", 1.0F, 0.2F},
        {"copper", 1.0F, 0.3F},     {"brass", 1.0F, 0.3F},     {"steel", 1.0F, 0.35F},
        {"stainless", 1.0F, 0.25F}, {"metal", 1.0F, 0.4F},     {"aluminium", 1.0F, 0.4F},
        {"aluminum", 1.0F, 0.4F},   {"iron", 1.0F, 0.55F},     {"glass", 0.0F, 0.03F},
        {"paint", 0.0F, 0.3F},      {"carpaint", 0.0F, 0.12F}, {"lacquer", 0.0F, 0.1F},
        {"ceramic", 0.0F, 0.12F},   {"glossy", 0.0F, 0.12F},   {"plastic", 0.0F, 0.42F},
        {"matte", 0.0F, 0.8F},      {"wood", 0.0F, 0.6F},      {"rubber", 0.0F, 0.88F},
        {"tyre", 0.0F, 0.88F},      {"tire", 0.0F, 0.88F},     {"wheel", 0.0F, 0.85F},
        {"concrete", 0.0F, 0.92F},  {"floor", 0.0F, 0.85F},    {"ground", 0.0F, 0.92F},
        {"asphalt", 0.0F, 0.95F},   {"road", 0.0F, 0.95F},     {"brick", 0.0F, 0.9F},
        {"fabric", 0.0F, 0.95F},    {"cloth", 0.0F, 0.95F},    {"skin", 0.0F, 0.55F},
        {"grass", 0.0F, 0.95F},
    }};
    for (const Row& row : kLooks) {
        if (row.name == name) {
            out.metallic = row.metallic;
            out.roughness = row.roughness;
            return true;
        }
    }
    return false;
}

SurfaceLook lookOf(const SceneMaterial& material) {
    if (material.look) {
        return *material.look;
    }
    SurfaceLook look;
    std::string word;
    for (std::size_t i = 0; i <= material.name.size(); ++i) {
        const char c = i < material.name.size() ? material.name[i] : ' ';
        if (std::isalnum(static_cast<unsigned char>(c)) != 0) {
            word += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        } else {
            if (surfaceLookPreset(word, look)) {
                return look;
            }
            word.clear();
        }
    }
    return look;
}

}  // namespace lagrange
