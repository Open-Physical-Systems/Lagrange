// lagrange_build_world: a tiled world from open data (huge-worlds.md §4).
#include "lagrange/geospatial/WorldBuilder.hpp"

#include <cstdio>
#include <cstdlib>
#include <numbers>
#include <optional>
#include <string>
#include <string_view>

namespace {

void usage() {
    std::puts(
        "Usage: lagrange_build_world OUT_DIR --origin LAT,LON,HEIGHT --size M [options]\n"
        "\n"
        "Writes a streamed world (WorldTileAPI_1 tiles, huge-worlds.md section 2) centred on\n"
        "the geo origin (degrees, metres above the WGS84 ellipsoid, or `ground`: on the\n"
        "elevation model's ground): OUT_DIR/NAME.usda, its\n"
        "tiles in OUT_DIR/tiles/ and OUT_DIR/ATTRIBUTION.md.\n"
        "\n"
        "  --name NAME            root prim and file name (default world)\n"
        "  --dem FILE             elevation raster (GeoTIFF, ...); repeat for a mosaic\n"
        "  --geoid FILE           geoid undulation raster: the DEM is orthometric\n"
        "                         (e.g. /usr/share/proj/egm96_15.gtx)\n"
        "  --assume-ellipsoidal   the DEM's heights are above the ellipsoid already\n"
        "  --osm FILE.osm         OpenStreetMap XML: buildings and roads\n"
        "  --tiles3d TILESET.json 3D Tiles to show (visual only); repeatable\n"
        "  --attribution TEXT     a line for ATTRIBUTION.md (a source and its licence)\n"
        "  --fine-tile M --fine-cell M --collision-cell M     (defaults 256, 8, 16)\n"
        "  --coarse-tile M --coarse-cell M                    (defaults 2048, 64)\n");
}

/// LAT,LON,HEIGHT or LAT,LON,ground (the origin on the elevation model's ground).
bool parseOrigin(const char* text, lagrange::coords::Geodetic& out, bool& onGround) {
    double lat = 0.0;
    double lon = 0.0;
    double h = 0.0;
    char word[16] = {};
    constexpr double kRadian = std::numbers::pi / 180.0;
    if (text != nullptr && std::sscanf(text, "%lf,%lf,%lf", &lat, &lon, &h) == 3) {
        out = {lat * kRadian, lon * kRadian, h};
        return true;
    }
    if (text != nullptr && std::sscanf(text, "%lf,%lf,%15s", &lat, &lon, word) == 3 &&
        std::string_view(word) == "ground") {
        out = {lat * kRadian, lon * kRadian, 0.0};
        onGround = true;
        return true;
    }
    return false;
}

}  // namespace

int main(int argc, char** argv) {
    using namespace lagrange::geospatial;
    WorldOptions options;
    Dem::Options dem;
    std::string out;
    bool haveOrigin = false;
    bool originOnGround = false;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        const auto value = [&]() -> const char* { return i + 1 < argc ? argv[++i] : nullptr; };
        const auto number = [&](double& into) {
            const char* v = value();
            if (v == nullptr) {
                std::fprintf(stderr, "error: %s needs a value\n", argv[i - 1]);
                std::exit(2);
            }
            into = std::strtod(v, nullptr);
        };
        if (arg == "--help" || arg == "-h") {
            usage();
            return 0;
        }
        if (arg == "--origin") {
            haveOrigin = parseOrigin(value(), options.origin, originOnGround);
        } else if (arg == "--size") {
            number(options.size);
        } else if (arg == "--name") {
            const char* v = value();
            options.name = v != nullptr ? v : options.name;
        } else if (arg == "--dem") {
            if (const char* v = value()) {
                dem.paths.emplace_back(v);
            }
        } else if (arg == "--geoid") {
            if (const char* v = value()) {
                dem.geoidPath = v;
            }
        } else if (arg == "--assume-ellipsoidal") {
            dem.assumeEllipsoidal = true;
        } else if (arg == "--osm") {
            const char* v = value();
            auto osm = v != nullptr ? readOsm(v) : lagrange::Unexpected{std::string("--osm FILE")};
            if (!osm) {
                std::fprintf(stderr, "error: %s\n", osm.error().c_str());
                return 1;
            }
            if (options.osm) {
                options.osm->buildings.insert(options.osm->buildings.end(), osm->buildings.begin(),
                                              osm->buildings.end());
                options.osm->roads.insert(options.osm->roads.end(), osm->roads.begin(),
                                          osm->roads.end());
            } else {
                options.osm = std::move(*osm);
            }
        } else if (arg == "--tiles3d") {
            if (const char* v = value()) {
                options.tiles3d.emplace_back(v);
            }
        } else if (arg == "--attribution") {
            if (const char* v = value()) {
                options.attribution.emplace_back(v);
            }
        } else if (arg == "--fine-tile") {
            number(options.fineTile);
        } else if (arg == "--fine-cell") {
            number(options.fineCell);
        } else if (arg == "--collision-cell") {
            number(options.collisionCell);
        } else if (arg == "--coarse-tile") {
            number(options.coarseTile);
        } else if (arg == "--coarse-cell") {
            number(options.coarseCell);
        } else if (!arg.starts_with("--") && out.empty()) {
            out = std::string(arg);
        } else {
            std::fprintf(stderr, "error: unknown argument %s (see --help)\n", argv[i]);
            return 2;
        }
    }
    if (out.empty() || !haveOrigin || !(options.size > 0.0)) {
        usage();
        return 2;
    }
    if (!dem.paths.empty()) {
        auto opened = Dem::open(dem);
        if (!opened) {
            std::fprintf(stderr, "error: %s\n", opened.error().c_str());
            return 1;
        }
        options.dem = std::move(*opened);
    }
    if (originOnGround) {
        const auto h = options.dem ? options.dem->ellipsoidalHeight(options.origin.latitudeRad,
                                                                    options.origin.longitudeRad)
                                   : std::optional<double>(0.0);
        if (!h) {
            std::fprintf(stderr, "error: the elevation model does not cover the origin\n");
            return 1;
        }
        options.origin.heightM = *h;
    }
    auto report = buildWorld(options, out);
    if (!report) {
        std::fprintf(stderr, "error: %s\n", report.error().c_str());
        return 1;
    }
    std::printf(
        "wrote %s: %zu coarse and %zu fine tiles, %zu buildings, %zu roads, %zu 3D Tiles "
        "meshes\n",
        report->rootPath.c_str(), report->coarseTiles, report->fineTiles, report->buildings,
        report->roads, report->meshes3d);
    for (const std::string& note : report->notes) {
        std::printf("note: %s\n", note.c_str());
    }
    return 0;
}
