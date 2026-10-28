#pragma once

#include "lagrange/geospatial/Dem.hpp"
#include "lagrange/geospatial/Osm.hpp"
#include "lagrange/module/Expected.hpp"
#include "lagrange/world/Model.hpp"
#include "lagrange/world/coords/Frames.hpp"

#include <optional>
#include <string>
#include <vector>

namespace lagrange::geospatial {

/// A tiled world from open data (huge-worlds.md §4): terrain from an elevation model, buildings
/// and roads from OpenStreetMap, and 3D Tiles (glTF content) for show, written as the streamed
/// layout of §2: a root layer with one WorldTileAPI_1 prim per tile, the tiles' content in
/// tiles/*.usdc, and ATTRIBUTION.md.
///
/// The world is a square centred on the geo origin, in its ENU frame (world ENU: the stage's
/// frame; the server's --geo-origin must be the same point). Two levels:
/// - level 0, coarse tiles: terrain on a coarse grid and buildings as blocks, visual only;
/// - level 1, fine tiles: terrain on a fine grid and buildings and roads, with colliders: the
///   terrain as a height field on the collision grid, each building as one convex prism (or,
///   for a concave footprint, prisms over its triangles; convex pieces, CLAUDE.md §4).
struct WorldOptions {
    coords::Geodetic origin;     ///< The world's geo origin (rad, rad, m). Required.
    double size{0.0};            ///< m: the square's side. Required.
    double fineTile{256.0};      ///< m
    double fineCell{8.0};        ///< m: terrain mesh spacing in fine tiles
    double collisionCell{16.0};  ///< m: terrain collision spacing (a multiple of fineCell)
    double coarseTile{2048.0};   ///< m
    double coarseCell{64.0};     ///< m
    std::optional<Dem> dem;      ///< None: flat ground at height 0.
    std::optional<OsmData> osm;
    std::vector<std::string> tiles3d;      ///< 3D Tiles tileset.json files, visual only
    std::vector<std::string> attribution;  ///< lines for ATTRIBUTION.md: sources and licences
    std::string name{"world"};             ///< the root prim's name and the root file's stem
};

struct WorldReport {
    std::string rootPath;
    std::size_t coarseTiles{0};
    std::size_t fineTiles{0};
    std::size_t buildings{0};
    std::size_t roads{0};
    std::size_t meshes3d{0};
    std::vector<std::string> notes;
};

/// Writes the world into `directory` (created if needed).
[[nodiscard]] Expected<WorldReport, std::string> buildWorld(const WorldOptions& options,
                                                            const std::string& directory);

/// One fine or coarse tile's content as a model in the tile's frame (corner at the origin):
/// for tests and for tools that write their own layout.
struct TileRequest {
    double x0{0.0};  ///< world ENU of the tile's south-west corner, m
    double y0{0.0};
    double size{0.0};
    double cell{0.0};           ///< terrain mesh spacing
    double collisionCell{0.0};  ///< 0: no colliders
};
[[nodiscard]] Expected<Model, std::string> buildTile(const WorldOptions& options,
                                                     const coords::GeoOrigin& origin,
                                                     const TileRequest& tile);

/// Ear clipping of a simple polygon (counter-clockwise or clockwise, x y pairs): triangles as
/// index triples, counter-clockwise. Empty for a degenerate polygon.
[[nodiscard]] std::vector<std::array<std::uint32_t, 3>> triangulate(
    const std::vector<std::array<double, 2>>& polygon);

}  // namespace lagrange::geospatial
