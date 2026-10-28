// Tiled worlds from open data (WorldBuilder.hpp, huge-worlds.md §4).
#include "lagrange/geospatial/Tiles3d.hpp"
#include "lagrange/geospatial/WorldBuilder.hpp"
#include "lagrange/stage/Stage.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <numbers>
#include <sstream>

namespace lagrange::geospatial {

namespace {

constexpr double kPrismDepthM = 2.0;  // terrain prisms reach this far under their lowest corner
constexpr double kRoadLiftM = 0.05;   // roads drawn just above the terrain, never in it
constexpr double kFoundationM = 0.5;  // buildings reach this far into the ground
constexpr std::array<float, 4> kTerrainColor{0.42F, 0.47F, 0.36F, 1.0F};
constexpr std::array<float, 4> kBuildingColor{0.68F, 0.67F, 0.65F, 1.0F};
constexpr std::array<float, 4> kRoadColor{0.24F, 0.24F, 0.26F, 1.0F};
constexpr std::array<float, 4> kTiles3dColor{0.75F, 0.75F, 0.78F, 1.0F};

using Vec3 = std::array<double, 3>;
using Point2 = std::array<double, 2>;

/// The terrain height (world ENU up, m) under an east/north position: the geodetic point under
/// it is found by fixed-point iteration, since up in ENU is not the ellipsoid's normal away
/// from the origin. Zero without an elevation model; empty outside it.
std::optional<double> groundAt(const WorldOptions& options, const coords::GeoOrigin& origin,
                               double e, double n) {
    if (!options.dem) {
        return 0.0;
    }
    constexpr int kIterations = 3;  // converges to well under a millimetre within 50 km
    double u = 0.0;
    for (int k = 0; k < kIterations; ++k) {
        const coords::Geodetic g = origin.toGeodetic({e, n, u});
        const auto h = options.dem->ellipsoidalHeight(g.latitudeRad, g.longitudeRad);
        if (!h) {
            return std::nullopt;
        }
        u = origin.fromGeodetic({g.latitudeRad, g.longitudeRad, *h}).u;
    }
    return u;
}

Point2 enuOf(const coords::GeoOrigin& origin, const std::array<double, 2>& latLon) {
    const coords::Enu p = origin.fromGeodetic({latLon[0], latLon[1], 0.0});
    return {p.e, p.n};
}

/// Builds one tile's model: a top prim "tile" with mesh children and world colliders.
class TileModel {
public:
    TileModel(double x0, double y0) : x0_(x0), y0_(y0) {
        ScenePrim top;
        top.name = "tile";
        top.path = "/tile";
        top.typeName = "Xform";
        m_.scene.prims.push_back(top);
        m_.name = "tile";
    }

    /// Adds a mesh (positions in world ENU) as a child of the tile, in the tile's frame.
    void mesh(const std::string& name, const std::vector<Vec3>& points,
              const std::vector<std::uint32_t>& indices, const std::array<float, 4>& rgba) {
        if (indices.empty()) {
            return;
        }
        TriMesh tri;
        tri.indices = indices;
        std::vector<Vec3> normals(points.size(), Vec3{});
        for (std::size_t t = 0; t + 2 < indices.size(); t += 3) {
            const Vec3& a = points[indices[t]];
            const Vec3& b = points[indices[t + 1]];
            const Vec3& c = points[indices[t + 2]];
            const Vec3 ab{b[0] - a[0], b[1] - a[1], b[2] - a[2]};
            const Vec3 ac{c[0] - a[0], c[1] - a[1], c[2] - a[2]};
            const Vec3 nrm{ab[1] * ac[2] - ab[2] * ac[1], ab[2] * ac[0] - ab[0] * ac[2],
                           ab[0] * ac[1] - ab[1] * ac[0]};
            for (std::size_t k = 0; k < 3; ++k) {
                for (std::size_t i = 0; i < 3; ++i) {
                    normals[indices[t + k]].at(i) += nrm.at(i);
                }
            }
        }
        for (std::size_t v = 0; v < points.size(); ++v) {
            tri.positions.push_back(static_cast<float>(points[v][0] - x0_));
            tri.positions.push_back(static_cast<float>(points[v][1] - y0_));
            tri.positions.push_back(static_cast<float>(points[v][2]));
            const Vec3& nv = normals[v];
            const double len = std::sqrt(nv[0] * nv[0] + nv[1] * nv[1] + nv[2] * nv[2]);
            for (std::size_t i = 0; i < 3; ++i) {
                tri.normals.push_back(len > 0.0 ? static_cast<float>(nv.at(i) / len)
                                                : (i == 2 ? 1.0F : 0.0F));
            }
            low_ = std::min(low_, points[v][2]);
            high_ = std::max(high_, points[v][2]);
        }
        m_.scene.meshes.push_back(std::move(tri));
        m_.scene.materials.push_back({name + "_material", rgba});
        ScenePrim prim;
        prim.name = name;
        prim.path = "/tile/" + name;
        prim.parent = 0;
        prim.typeName = "Mesh";
        prim.mesh = static_cast<std::uint32_t>(m_.scene.meshes.size() - 1);
        prim.material = static_cast<std::uint32_t>(m_.scene.materials.size() - 1);
        m_.scene.prims.push_back(std::move(prim));
    }

    /// A convex prism over a triangle (world ENU, any winding) from `bottom` up to the
    /// triangle's own heights, as a hull collider on the world.
    void prism(Vec3 a, Vec3 b, Vec3 c, double bottom) {
        const double cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
        if (std::abs(cross) < 1e-9) {
            return;  // degenerate
        }
        if (cross < 0.0) {
            std::swap(b, c);  // counter-clockwise seen from above
        }
        const Vec3 centre{(a[0] + b[0] + c[0]) / 3.0, (a[1] + b[1] + c[1]) / 3.0,
                          (a[2] + b[2] + c[2] + 3.0 * bottom) / 6.0};
        ConvexHull hull;
        for (const Vec3& p : {a, b, c}) {
            hull.points.push_back({p[0] - centre[0], p[1] - centre[1], p[2] - centre[2]});
        }
        for (const Vec3& p : {a, b, c}) {
            hull.points.push_back({p[0] - centre[0], p[1] - centre[1], bottom - centre[2]});
        }
        // Top 0 1 2 (up), bottom 3 5 4 (down), each side (i, i', j) and (j, i', j') outward.
        hull.triangles = {{0, 1, 2}, {3, 5, 4}, {0, 3, 1}, {1, 3, 4},
                          {1, 4, 2}, {2, 4, 5}, {2, 5, 0}, {0, 5, 3}};
        Pose pose;
        pose.position = {centre[0] - x0_, centre[1] - y0_, centre[2]};
        m_.world.addHullCollider(kWorldBody, std::move(hull), pose, 1.0);
        low_ = std::min(low_, bottom);
        high_ = std::max({high_, a[2], b[2], c[2]});
    }

    /// A convex prism over a convex polygon (world ENU x y, counter-clockwise) from `bottom`
    /// to `top`, as one hull collider.
    void prism(const std::vector<Point2>& polygon, double bottom, double top) {
        const std::size_t n = polygon.size();
        Point2 c{};
        for (const Point2& p : polygon) {
            c[0] += p[0] / static_cast<double>(n);
            c[1] += p[1] / static_cast<double>(n);
        }
        const double mid = (bottom + top) / 2.0;
        ConvexHull hull;
        for (const Point2& p : polygon) {
            hull.points.push_back({p[0] - c[0], p[1] - c[1], top - mid});
        }
        for (const Point2& p : polygon) {
            hull.points.push_back({p[0] - c[0], p[1] - c[1], bottom - mid});
        }
        const auto un = static_cast<std::uint32_t>(n);
        for (std::uint32_t i = 1; i + 1 < un; ++i) {
            hull.triangles.push_back({0, i, i + 1});             // top, up
            hull.triangles.push_back({un, un + i + 1, un + i});  // bottom, down
        }
        for (std::uint32_t i = 0; i < un; ++i) {
            const std::uint32_t j = (i + 1) % un;
            hull.triangles.push_back({i, un + i, j});  // sides, outward
            hull.triangles.push_back({j, un + i, un + j});
        }
        Pose pose;
        pose.position = {c[0] - x0_, c[1] - y0_, mid};
        m_.world.addHullCollider(kWorldBody, std::move(hull), pose, 1.0);
        low_ = std::min(low_, bottom);
        high_ = std::max(high_, top);
    }

    /// Terrain collision: a height field centred at (cx, cy) in world ENU.
    void heightField(HeightField field, double cx, double cy) {
        for (const double h : field.heights) {
            low_ = std::min(low_, h);
            high_ = std::max(high_, h);
        }
        Pose pose;
        pose.position = {cx - x0_, cy - y0_, 0.0};
        m_.world.addHeightFieldCollider(std::move(field), pose, 1.0);
    }

    Model& model() { return m_; }
    [[nodiscard]] double low() const { return std::isfinite(low_) ? low_ : 0.0; }
    [[nodiscard]] double high() const { return std::isfinite(high_) ? high_ : 0.0; }

private:
    Model m_;
    double x0_;
    double y0_;
    double low_{std::numeric_limits<double>::infinity()};
    double high_{-std::numeric_limits<double>::infinity()};
};

Point2 centroid(const std::vector<Point2>& ring) {
    Point2 c{};
    for (const Point2& p : ring) {
        c[0] += p[0] / static_cast<double>(ring.size());
        c[1] += p[1] / static_cast<double>(ring.size());
    }
    return c;
}

/// True for a convex polygon, either winding (collinear corners allowed).
bool convex(const std::vector<Point2>& ring) {
    int sign = 0;
    for (std::size_t i = 0; i < ring.size(); ++i) {
        const Point2& a = ring[i];
        const Point2& b = ring[(i + 1) % ring.size()];
        const Point2& c = ring[(i + 2) % ring.size()];
        const double cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]);
        if (std::abs(cross) < 1e-9) {
            continue;
        }
        const int s = cross > 0.0 ? 1 : -1;
        if (sign != 0 && s != sign) {
            return false;
        }
        sign = s;
    }
    return sign != 0;
}

bool inside(const Point2& p, const TileRequest& t) {
    return p[0] >= t.x0 && p[0] < t.x0 + t.size && p[1] >= t.y0 && p[1] < t.y0 + t.size;
}

}  // namespace

std::vector<std::array<std::uint32_t, 3>> triangulate(const std::vector<Point2>& polygon) {
    std::vector<std::array<std::uint32_t, 3>> out;
    const std::size_t n = polygon.size();
    if (n < 3) {
        return out;
    }
    double area = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const Point2& a = polygon[i];
        const Point2& b = polygon[(i + 1) % n];
        area += a[0] * b[1] - b[0] * a[1];
    }
    if (std::abs(area) < 1e-12) {
        return out;
    }
    std::vector<std::uint32_t> ring(n);
    for (std::uint32_t i = 0; i < n; ++i) {
        ring[i] = area > 0.0 ? i : static_cast<std::uint32_t>(n - 1 - i);  // counter-clockwise
    }
    const auto cross = [&](std::uint32_t a, std::uint32_t b, std::uint32_t c) {
        const Point2& p = polygon[a];
        const Point2& q = polygon[b];
        const Point2& r = polygon[c];
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
    };
    std::size_t guard = 0;
    while (ring.size() > 3 && guard++ < n * n) {
        bool clipped = false;
        for (std::size_t i = 0; i < ring.size(); ++i) {
            const std::uint32_t a = ring[(i + ring.size() - 1) % ring.size()];
            const std::uint32_t b = ring[i];
            const std::uint32_t c = ring[(i + 1) % ring.size()];
            if (cross(a, b, c) <= 0.0) {
                continue;  // reflex
            }
            bool empty = true;
            for (const std::uint32_t p : ring) {
                if (p != a && p != b && p != c && cross(a, b, p) >= 0.0 && cross(b, c, p) >= 0.0 &&
                    cross(c, a, p) >= 0.0) {
                    empty = false;
                    break;
                }
            }
            if (empty) {
                out.push_back({a, b, c});
                ring.erase(ring.begin() + static_cast<std::ptrdiff_t>(i));
                clipped = true;
                break;
            }
        }
        if (!clipped) {
            break;  // not simple: what is left is dropped
        }
    }
    if (ring.size() == 3) {
        out.push_back({ring[0], ring[1], ring[2]});
    }
    return out;
}

Expected<Model, std::string> buildTile(const WorldOptions& options, const coords::GeoOrigin& origin,
                                       const TileRequest& t) {
    TileModel tile(t.x0, t.y0);
    // Terrain: a grid of cell spacing, two triangles a cell.
    const int cells = std::max(1, static_cast<int>(std::lround(t.size / t.cell)));
    const double step = t.size / cells;
    std::vector<Vec3> grid;
    grid.reserve(static_cast<std::size_t>((cells + 1) * (cells + 1)));
    for (int j = 0; j <= cells; ++j) {
        for (int i = 0; i <= cells; ++i) {
            const double e = t.x0 + i * step;
            const double n = t.y0 + j * step;
            const auto u = groundAt(options, origin, e, n);
            if (!u) {
                char text[160];
                std::snprintf(text, sizeof text,
                              "the elevation model does not cover (%.0f, %.0f) m of the world", e,
                              n);
                return Unexpected{std::string(text)};
            }
            grid.push_back({e, n, *u});
        }
    }
    const auto at = [cells](int i, int j) {
        return static_cast<std::uint32_t>(j * (cells + 1) + i);
    };
    std::vector<std::uint32_t> indices;
    for (int j = 0; j < cells; ++j) {
        for (int i = 0; i < cells; ++i) {
            indices.insert(indices.end(), {at(i, j), at(i + 1, j), at(i + 1, j + 1), at(i, j),
                                           at(i + 1, j + 1), at(i, j + 1)});
        }
    }
    tile.mesh("terrain", grid, indices, kTerrainColor);
    if (t.collisionCell > 0.0) {
        // Terrain collision: one height field on the grid's every n-th sample.
        const int every = std::max(1, static_cast<int>(std::lround(t.collisionCell / step)));
        HeightField field;
        field.columns = static_cast<std::uint32_t>(cells / every + 1);
        field.rows = field.columns;
        field.cellX = step * every;
        field.cellY = step * every;
        for (int j = 0; j <= cells; j += every) {
            for (int i = 0; i <= cells; i += every) {
                field.heights.push_back(grid[at(i, j)][2]);
            }
        }
        tile.heightField(std::move(field), t.x0 + t.size / 2.0, t.y0 + t.size / 2.0);
    }

    // Buildings whose footprint's centre is in the tile.
    if (options.osm) {
        std::size_t k = 0;
        for (const OsmBuilding& b : options.osm->buildings) {
            std::vector<Point2> ring;
            for (const auto& p : b.footprint) {
                ring.push_back(enuOf(origin, p));
            }
            if (!inside(centroid(ring), t)) {
                continue;
            }
            double base = std::numeric_limits<double>::infinity();
            for (const Point2& p : ring) {
                base = std::min(base, groundAt(options, origin, p[0], p[1]).value_or(0.0));
            }
            const double top = base + b.height;
            const auto triangles = triangulate(ring);
            if (triangles.empty()) {
                continue;
            }
            std::vector<Vec3> points;
            std::vector<std::uint32_t> faces;
            for (const Point2& p : ring) {
                points.push_back({p[0], p[1], top});
            }
            for (const auto& tri : triangles) {
                faces.insert(faces.end(), {tri[0], tri[1], tri[2]});
            }
            // Walls: one quad per edge, outward for a counter-clockwise ring.
            double area = 0.0;
            for (std::size_t i = 0; i < ring.size(); ++i) {
                const Point2& p = ring[i];
                const Point2& q = ring[(i + 1) % ring.size()];
                area += p[0] * q[1] - q[0] * p[1];
            }
            for (std::size_t i = 0; i < ring.size(); ++i) {
                std::size_t a = i;
                std::size_t c = (i + 1) % ring.size();
                if (area < 0.0) {
                    std::swap(a, c);
                }
                const auto first = static_cast<std::uint32_t>(points.size());
                points.push_back({ring[a][0], ring[a][1], base - kFoundationM});
                points.push_back({ring[c][0], ring[c][1], base - kFoundationM});
                points.push_back({ring[c][0], ring[c][1], top});
                points.push_back({ring[a][0], ring[a][1], top});
                faces.insert(faces.end(),
                             {first, first + 1, first + 2, first, first + 2, first + 3});
            }
            tile.mesh("building_" + std::to_string(k++), points, faces, kBuildingColor);
            if (t.collisionCell > 0.0 && convex(ring)) {
                // Most buildings: one hull (fewer colliders load faster).
                std::vector<Point2> ccw = ring;
                if (area < 0.0) {
                    std::reverse(ccw.begin(), ccw.end());
                }
                tile.prism(ccw, base - kFoundationM, top);
            } else if (t.collisionCell > 0.0) {
                for (const auto& tri : triangles) {
                    tile.prism({ring[tri[0]][0], ring[tri[0]][1], top},
                               {ring[tri[1]][0], ring[tri[1]][1], top},
                               {ring[tri[2]][0], ring[tri[2]][1], top}, base - kFoundationM);
                }
            }
        }
        // Roads: ribbons draped on the terrain, pieces no longer than a cell, visual only.
        if (t.collisionCell > 0.0) {
            std::vector<Vec3> points;
            std::vector<std::uint32_t> faces;
            for (const OsmRoad& road : options.osm->roads) {
                for (std::size_t s = 0; s + 1 < road.line.size(); ++s) {
                    const Point2 p = enuOf(origin, road.line[s]);
                    const Point2 q = enuOf(origin, road.line[s + 1]);
                    const double length = std::hypot(q[0] - p[0], q[1] - p[1]);
                    if (length <= 0.0 || !inside({(p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0}, t)) {
                        continue;
                    }
                    const Point2 side{-(q[1] - p[1]) / length * road.width / 2.0,
                                      (q[0] - p[0]) / length * road.width / 2.0};
                    const int pieces = std::max(1, static_cast<int>(std::ceil(length / t.cell)));
                    for (int k2 = 0; k2 < pieces; ++k2) {
                        const double f0 = static_cast<double>(k2) / pieces;
                        const double f1 = static_cast<double>(k2 + 1) / pieces;
                        const Point2 a{p[0] + (q[0] - p[0]) * f0, p[1] + (q[1] - p[1]) * f0};
                        const Point2 b{p[0] + (q[0] - p[0]) * f1, p[1] + (q[1] - p[1]) * f1};
                        const double za = groundAt(options, origin, a[0], a[1]).value_or(0.0);
                        const double zb = groundAt(options, origin, b[0], b[1]).value_or(0.0);
                        const auto first = static_cast<std::uint32_t>(points.size());
                        points.push_back({a[0] - side[0], a[1] - side[1], za + kRoadLiftM});
                        points.push_back({b[0] - side[0], b[1] - side[1], zb + kRoadLiftM});
                        points.push_back({b[0] + side[0], b[1] + side[1], zb + kRoadLiftM});
                        points.push_back({a[0] + side[0], a[1] + side[1], za + kRoadLiftM});
                        faces.insert(faces.end(),
                                     {first, first + 1, first + 2, first, first + 2, first + 3});
                    }
                }
            }
            tile.mesh("roads", points, faces, kRoadColor);
        }
    }
    return std::move(tile.model());
}

Expected<WorldReport, std::string> buildWorld(const WorldOptions& options,
                                              const std::string& directory) {
    if (!(options.size > 0.0) || !(options.fineTile > 0.0) || !(options.coarseTile > 0.0) ||
        !(options.fineCell > 0.0) || !(options.coarseCell > 0.0)) {
        return Unexpected{std::string("the world's size, tiles and cells must be positive")};
    }
    namespace fs = std::filesystem;
    std::error_code error;
    fs::create_directories(fs::path(directory) / "tiles", error);
    if (error) {
        return Unexpected{"cannot create " + directory + ": " + error.message()};
    }
    const coords::GeoOrigin origin(options.origin);
    WorldReport report;
    std::ostringstream prims;
    const double half = options.size / 2.0;

    // 3D Tiles: their meshes in world ENU, put in the fine tile their centre falls in.
    std::vector<Tiles3dMesh> extra;
    for (const std::string& tileset : options.tiles3d) {
        auto meshes = readTiles3d(tileset, origin);
        if (!meshes) {
            return Unexpected{meshes.error()};
        }
        report.notes.insert(report.notes.end(), meshes->notes.begin(), meshes->notes.end());
        extra.insert(extra.end(), meshes->meshes.begin(), meshes->meshes.end());
    }
    report.meshes3d = extra.size();

    const auto writeTile = [&](int level, int x, int y, const TileRequest& request,
                               double geometricError, Model model) -> Expected<void, std::string> {
        double low = 0.0;
        double high = 0.0;
        for (const TriMesh& mesh : model.scene.meshes) {
            for (std::size_t i = 2; i < mesh.positions.size(); i += 3) {
                low = std::min(low, static_cast<double>(mesh.positions[i]));
                high = std::max(high, static_cast<double>(mesh.positions[i]));
            }
        }
        const std::string stem =
            "L" + std::to_string(level) + "_" + std::to_string(x) + "_" + std::to_string(y);
        stage::WriteOptions write;
        write.doc = "Tile " + stem + " of " + options.name + " (lagrange_build_world)";
        write.staticColliders = "/tile/colliders";
        const std::string file = (fs::path(directory) / "tiles" / (stem + ".usdc")).string();
        if (auto written = stage::writeStage(model, file, write); !written) {
            return Unexpected{written.error()};
        }
        char text[1024];
        std::snprintf(text, sizeof text,
                      "    def Xform \"%s\" (\n"
                      "        prepend apiSchemas = [\"WorldTileAPI_1\"]\n"
                      "        prepend payload = @./tiles/%s.usdc@\n"
                      "    )\n    {\n"
                      "        int world:tile:level = %d\n"
                      "        int2 world:tile:index = (%d, %d)\n"
                      "        double3 world:tile:boundsMin = (%.3f, %.3f, %.3f)\n"
                      "        double3 world:tile:boundsMax = (%.3f, %.3f, %.3f)\n"
                      "        double world:tile:geometricError = %g\n"
                      "        bool world:tile:hasCollision = %s\n"
                      "        double3 xformOp:translate = (%.3f, %.3f, 0)\n"
                      "        uniform token[] xformOpOrder = [\"xformOp:translate\"]\n"
                      "    }\n",
                      stem.c_str(), stem.c_str(), level, x, y, request.x0, request.y0, low,
                      request.x0 + request.size, request.y0 + request.size, high, geometricError,
                      request.collisionCell > 0.0 ? "true" : "false", request.x0, request.y0);
        prims << text;
        return {};
    };

    // A tile's terrain is at most about half a cell's curvature from the true surface: the
    // cell size stands for the geometric error.
    const int coarseCount = static_cast<int>(std::ceil(options.size / options.coarseTile));
    for (int x = 0; x < coarseCount; ++x) {
        for (int y = 0; y < coarseCount; ++y) {
            const TileRequest request{-half + x * options.coarseTile,
                                      -half + y * options.coarseTile, options.coarseTile,
                                      options.coarseCell, 0.0};
            WorldOptions coarse = options;
            if (coarse.osm) {
                coarse.osm->roads.clear();  // roads only in fine tiles
            }
            auto model = buildTile(coarse, origin, request);
            if (!model) {
                return Unexpected{model.error()};
            }
            if (auto ok = writeTile(0, x, y, request, options.coarseCell / 2.0, std::move(*model));
                !ok) {
                return Unexpected{ok.error()};
            }
            ++report.coarseTiles;
        }
    }
    const int fineCount = static_cast<int>(std::ceil(options.size / options.fineTile));
    for (int x = 0; x < fineCount; ++x) {
        for (int y = 0; y < fineCount; ++y) {
            const TileRequest request{-half + x * options.fineTile, -half + y * options.fineTile,
                                      options.fineTile, options.fineCell, options.collisionCell};
            auto model = buildTile(options, origin, request);
            if (!model) {
                return Unexpected{model.error()};
            }
            // 3D Tiles meshes: visual, under the fine tile their centre is in.
            std::size_t k = 0;
            for (const Tiles3dMesh& m : extra) {
                if (!inside({m.centre[0], m.centre[1]}, request)) {
                    continue;
                }
                TileModel part(request.x0, request.y0);
                part.mesh("mesh3d_" + std::to_string(k++), m.points, m.indices, kTiles3dColor);
                Model& into = *model;
                const Model& add = part.model();
                const auto meshOffset = static_cast<std::uint32_t>(into.scene.meshes.size());
                const auto materialOffset = static_cast<std::uint32_t>(into.scene.materials.size());
                into.scene.meshes.push_back(add.scene.meshes.back());
                into.scene.materials.push_back(add.scene.materials.back());
                ScenePrim prim = add.scene.prims.back();
                prim.mesh = meshOffset;
                prim.material = materialOffset;
                into.scene.prims.push_back(std::move(prim));
            }
            if (auto ok = writeTile(1, x, y, request, options.fineCell / 2.0, std::move(*model));
                !ok) {
                return Unexpected{ok.error()};
            }
            ++report.fineTiles;
        }
    }
    if (options.osm) {
        report.buildings = options.osm->buildings.size();
        report.roads = options.osm->roads.size();
        report.notes.insert(report.notes.end(), options.osm->notes.begin(),
                            options.osm->notes.end());
    }

    const double latDeg = options.origin.latitudeRad * 180.0 / std::numbers::pi;
    const double lonDeg = options.origin.longitudeRad * 180.0 / std::numbers::pi;
    char header[2048];
    std::snprintf(header, sizeof header,
                  "#usda 1.0\n(\n    defaultPrim = \"%s\"\n"
                  "    doc = \"\"\"%s: a %.0f m square world from open data, written by "
                  "lagrange_build_world.\n    Its frame is the ENU frame at latitude %.7f, "
                  "longitude %.7f, height %.3f m (WGS84):\n    run it with --geo-origin "
                  "%.7f,%.7f,%.3f. Sources and licences: ATTRIBUTION.md.\"\"\"\n"
                  "%s"
                  "    kilogramsPerUnit = 1\n    metersPerUnit = 1\n    upAxis = \"Z\"\n)\n\n"
                  "def Xform \"%s\"\n{\n",
                  options.name.c_str(), options.name.c_str(), options.size, latDeg, lonDeg,
                  options.origin.heightM, latDeg, lonDeg, options.origin.heightM,
                  // Terrain from an elevation model is the ground: no infinite plane under it.
                  options.dem ? "    customLayerData = {\n        bool lagrangeGroundPlane = 0\n"
                                "    }\n"
                              : "",
                  options.name.c_str());
    report.rootPath = (fs::path(directory) / (options.name + ".usda")).string();
    std::ofstream(report.rootPath) << header << prims.str() << "}\n";

    std::ofstream attribution(fs::path(directory) / "ATTRIBUTION.md");
    attribution << "# " << options.name << ": sources and licences\n\n"
                << "Written by lagrange_build_world. The world's geometry is derived from the "
                   "data below; its licences apply to the tiles.\n\n";
    if (options.dem) {
        attribution << "## Elevation\n\n";
        for (const std::string& s : options.dem->sources()) {
            attribution << "- " << s << "\n";
        }
        attribution << "\n";
    }
    if (options.osm) {
        attribution << "## OpenStreetMap\n\n"
                       "Buildings and roads: © OpenStreetMap contributors, available under the "
                       "Open Database License (ODbL) 1.0, https://www.openstreetmap.org/copyright."
                       " This world is a produced work of that data.\n\n";
    }
    if (!options.tiles3d.empty()) {
        attribution << "## 3D Tiles\n\n";
        for (const std::string& s : options.tiles3d) {
            attribution << "- " << s << "\n";
        }
        attribution << "\n";
    }
    if (!options.attribution.empty()) {
        attribution << "## Notes\n\n";
        for (const std::string& line : options.attribution) {
            attribution << "- " << line << "\n";
        }
    }
    return report;
}

}  // namespace lagrange::geospatial
