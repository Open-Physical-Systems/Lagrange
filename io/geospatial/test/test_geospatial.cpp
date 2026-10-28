#include "lagrange/geospatial/Dem.hpp"
#include "lagrange/geospatial/Osm.hpp"
#include "lagrange/geospatial/Tiles3d.hpp"
#include "lagrange/geospatial/WorldBuilder.hpp"
#include "lagrange/stage/Stage.hpp"

#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <gdal_priv.h>
#include <numbers>
#include <ogr_spatialref.h>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::geospatial {
namespace {

constexpr double kRadian = std::numbers::pi / 180.0;
// The test world's origin: an arbitrary point (central Kyushu), 20 m above the ellipsoid.
constexpr double kLat0 = 33.0;
constexpr double kLon0 = 130.5;

std::filesystem::path scratch(const std::string& name) {
    const auto dir = std::filesystem::path(::testing::TempDir()) / "lagrange_geospatial";
    std::filesystem::create_directories(dir);
    return dir / name;
}

/// A GeoTIFF in WGS84 lon/lat over the origin +-0.05 degrees, 0.001 degree pixels, whose
/// height is a plane: `base` + `perDegreeLat` x (latitude - kLat0) at the pixel centres.
std::string writeDem(const std::string& name, double base, double perDegreeLat) {
    GDALAllRegister();
    const std::string path = scratch(name).string();
    constexpr int kSize = 100;
    constexpr double kPixel = 0.001;
    GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("GTiff");
    std::unique_ptr<GDALDataset> ds(
        driver->Create(path.c_str(), kSize, kSize, 1, GDT_Float32, nullptr));
    const double west = kLon0 - kSize * kPixel / 2;
    const double north = kLat0 + kSize * kPixel / 2;
    std::array<double, 6> geo{west, kPixel, 0.0, north, 0.0, -kPixel};
    ds->SetGeoTransform(geo.data());
    OGRSpatialReference wgs84;
    wgs84.SetWellKnownGeogCS("WGS84");
    ds->SetSpatialRef(&wgs84);
    std::vector<float> values(kSize * kSize);
    for (int r = 0; r < kSize; ++r) {
        const double lat = north - (r + 0.5) * kPixel;
        for (int c = 0; c < kSize; ++c) {
            values[r * kSize + c] = static_cast<float>(base + perDegreeLat * (lat - kLat0));
        }
    }
    EXPECT_EQ(ds->GetRasterBand(1)->RasterIO(GF_Write, 0, 0, kSize, kSize, values.data(), kSize,
                                             kSize, GDT_Float32, 0, 0),
              CE_None);
    return path;
}

Expected<Dem, std::string> openDem(const std::string& path, const std::string& geoid = {},
                                   bool ellipsoidal = false) {
    Dem::Options options;
    options.paths = {path};
    options.geoidPath = geoid;
    options.assumeEllipsoidal = ellipsoidal;
    return Dem::open(options);
}

TEST(Dem, ItNeedsToKnowTheVerticalDatum) {
    const std::string path = writeDem("flat.tif", 50.0, 0.0);
    EXPECT_FALSE(openDem(path)) << "orthometric or ellipsoidal must be said";
    const auto ellipsoidal = openDem(path, {}, true);
    ASSERT_TRUE(ellipsoidal) << ellipsoidal.error();
    EXPECT_NEAR(*ellipsoidal->ellipsoidalHeight(kLat0 * kRadian, kLon0 * kRadian), 50.0, 1e-4);
    // A geoid 30 m above the ellipsoid: h = H + N.
    const std::string geoid = writeDem("geoid.tif", 30.0, 0.0);
    const auto orthometric = openDem(path, geoid);
    ASSERT_TRUE(orthometric) << orthometric.error();
    EXPECT_NEAR(*orthometric->ellipsoidalHeight(kLat0 * kRadian, kLon0 * kRadian), 80.0, 1e-4);
    EXPECT_FALSE(ellipsoidal->ellipsoidalHeight((kLat0 + 1.0) * kRadian, kLon0 * kRadian))
        << "outside the raster";
}

TEST(Dem, SamplesBilinearlyBetweenPixelCentres) {
    const std::string path = writeDem("slope.tif", 100.0, 1000.0);  // 1 m per 0.001 degree
    const auto dem = openDem(path, {}, true);
    ASSERT_TRUE(dem);
    for (const double dLat : {-0.0213, 0.0, 0.0071, 0.0332}) {
        EXPECT_NEAR(*dem->ellipsoidalHeight((kLat0 + dLat) * kRadian, (kLon0 + 0.01) * kRadian),
                    100.0 + 1000.0 * dLat, 1e-3);
    }
}

TEST(WorldBuilder, TerrainFollowsTheElevationModelInTheEnuFrame) {
    const std::string path = writeDem("terrain.tif", 100.0, 1000.0);
    WorldOptions options;
    options.origin = {kLat0 * kRadian, kLon0 * kRadian, 20.0};
    options.dem = *openDem(path, {}, true);
    const coords::GeoOrigin origin(options.origin);
    const auto tile = buildTile(
        options, origin, {.x0 = 0.0, .y0 = 0.0, .size = 128.0, .cell = 8.0, .collisionCell = 16.0});
    ASSERT_TRUE(tile) << tile.error();
    ASSERT_EQ(tile->scene.meshes.size(), 1U);
    // 2 prisms per 16 m collision cell of a 128 m tile.
    EXPECT_EQ(tile->world.colliders.body.size(), 2U * 8U * 8U);
    // Each vertex's height: the geodetic point under it, on the model, in ENU.
    const TriMesh& mesh = tile->scene.meshes[0];
    for (std::size_t v = 0; v < mesh.positions.size() / 3; v += 37) {
        const double e = mesh.positions[3 * v];
        const double n = mesh.positions[3 * v + 1];
        const double u = mesh.positions[3 * v + 2];
        const coords::Geodetic g = origin.toGeodetic({e, n, u});
        const double model = 100.0 + 1000.0 * (g.latitudeRad / kRadian - kLat0);
        EXPECT_NEAR(g.heightM, model, 1e-3) << "vertex " << v;
    }
    // About 80 m below the origin (100 - 20), less the 128 m tile's tilt and curvature.
    EXPECT_NEAR(mesh.positions[2], 80.0, 0.01);
}

TEST(WorldBuilder, EarClippingCoversTheFootprint) {
    // An L: 3 x 3 square with the 2 x 2 upper-right quarter cut, 5 square metres.
    const std::vector<std::array<double, 2>> l{{0, 0}, {3, 0}, {3, 1}, {1, 1}, {1, 3}, {0, 3}};
    const auto triangles = triangulate(l);
    ASSERT_EQ(triangles.size(), 4U);
    double area = 0.0;
    for (const auto& t : triangles) {
        const auto& a = l[t[0]];
        const auto& b = l[t[1]];
        const auto& c = l[t[2]];
        const double cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
        EXPECT_GT(cross, 0.0) << "counter-clockwise";
        area += cross / 2.0;
    }
    EXPECT_NEAR(area, 5.0, 1e-12);
}

std::string writeOsm() {
    // A 20 m square building, 15 m high, 40 m east of the origin, and a residential road.
    const coords::GeoOrigin origin({kLat0 * kRadian, kLon0 * kRadian, 0.0});
    std::ostringstream xml;
    xml << "<?xml version='1.0'?><osm version='0.6'>";
    const std::array<std::array<double, 2>, 6> corners{
        {{30, -10}, {50, -10}, {50, 10}, {30, 10}, {-100, -2}, {100, -2}}};
    xml.precision(12);
    for (std::size_t i = 0; i < corners.size(); ++i) {
        const coords::Geodetic g = origin.toGeodetic({corners[i][0], corners[i][1], 0.0});
        xml << "<node id='" << i + 1 << "' lat='" << g.latitudeRad / kRadian << "' lon='"
            << g.longitudeRad / kRadian << "'/>";
    }
    xml << "<way id='10'><nd ref='1'/><nd ref='2'/><nd ref='3'/><nd ref='4'/><nd ref='1'/>"
           "<tag k='building' v='yes'/><tag k='height' v='15 m'/></way>"
           "<way id='11'><nd ref='5'/><nd ref='6'/><tag k='highway' v='residential'/></way>"
           "<relation id='12'/></osm>";
    const std::string path = scratch("town.osm").string();
    std::ofstream(path) << xml.str();
    return path;
}

TEST(WorldBuilder, BuildingsAndRoadsFromOpenStreetMap) {
    const auto osm = readOsm(writeOsm());
    ASSERT_TRUE(osm) << osm.error();
    ASSERT_EQ(osm->buildings.size(), 1U);
    EXPECT_DOUBLE_EQ(osm->buildings[0].height, 15.0);
    EXPECT_EQ(osm->buildings[0].footprint.size(), 4U);
    ASSERT_EQ(osm->roads.size(), 1U);
    EXPECT_DOUBLE_EQ(osm->roads[0].width, 6.0);
    EXPECT_EQ(osm->notes.size(), 1U) << "the relation is noted";

    WorldOptions options;
    options.origin = {kLat0 * kRadian, kLon0 * kRadian, 0.0};
    options.osm = *osm;
    const coords::GeoOrigin origin(options.origin);
    const auto tile =
        buildTile(options, origin,
                  {.x0 = -64.0, .y0 = -64.0, .size = 128.0, .cell = 16.0, .collisionCell = 16.0});
    ASSERT_TRUE(tile) << tile.error();
    // Terrain 2 x 8 x 8 prisms, and the building (convex) as one hull of 8 corners.
    ASSERT_EQ(tile->world.colliders.body.size(), 2U * 64U + 1U);
    EXPECT_EQ(tile->world.hulls.back().points.size(), 8U);
    bool building = false;
    bool roads = false;
    for (const ScenePrim& prim : tile->scene.prims) {
        if (prim.name == "building_0") {
            building = true;
            const TriMesh& mesh = tile->scene.meshes.at(prim.mesh);
            float top = -1e9F;
            for (std::size_t i = 2; i < mesh.positions.size(); i += 3) {
                top = std::max(top, mesh.positions[i]);
            }
            EXPECT_NEAR(top, 15.0, 0.01);
        }
        roads = roads || prim.name == "roads";
    }
    EXPECT_TRUE(building);
    EXPECT_TRUE(roads);
}

TEST(WorldBuilder, WritesAStreamedWorldTheServerReads) {
    WorldOptions options;
    options.origin = {kLat0 * kRadian, kLon0 * kRadian, 0.0};
    options.size = 512.0;
    options.fineTile = 256.0;
    options.fineCell = 32.0;
    options.collisionCell = 32.0;
    options.coarseTile = 512.0;
    options.coarseCell = 128.0;
    options.osm = *readOsm(writeOsm());
    options.name = "test_world";
    options.attribution = {"test data"};
    const auto dir = scratch("world");
    std::filesystem::remove_all(dir);
    const auto report = buildWorld(options, dir.string());
    ASSERT_TRUE(report) << report.error();
    EXPECT_EQ(report->coarseTiles, 1U);
    EXPECT_EQ(report->fineTiles, 4U);

    stage::ReadOptions read;
    read.addGround = false;
    const auto root = stage::loadStage(report->rootPath, read);
    ASSERT_TRUE(root) << root.error();
    ASSERT_EQ(root->tiles.size(), 5U);
    std::size_t collisionTiles = 0;
    for (const streaming::TileInfo& t : root->tiles) {
        collisionTiles += t.hasCollision ? 1 : 0;
        if (!t.hasCollision) {
            continue;
        }
        read.tile = t.path;
        const auto content = stage::loadStage(report->rootPath, read);
        ASSERT_TRUE(content) << content.error();
        EXPECT_GE(content->world.colliders.body.size(), 2U * 8U * 8U) << t.path;
        // Colliders where the tile is.
        for (const Pose& pose : content->world.colliders.poseInBody) {
            EXPECT_GE(pose.position[0], t.boundsMin[0] - 1e-6);
            EXPECT_LE(pose.position[0], t.boundsMax[0] + 1e-6);
        }
    }
    EXPECT_EQ(collisionTiles, 4U);
    std::ifstream attribution(dir / "ATTRIBUTION.md");
    std::stringstream text;
    text << attribution.rdbuf();
    EXPECT_NE(text.str().find("OpenStreetMap contributors"), std::string::npos);
    EXPECT_NE(text.str().find("test data"), std::string::npos);
}

std::string base64(const std::string& bytes) {
    static constexpr char kAlphabet[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    for (std::size_t i = 0; i < bytes.size(); i += 3) {
        const auto b = [&](std::size_t k) {
            return k < bytes.size() ? static_cast<unsigned char>(bytes[k]) : 0U;
        };
        const std::uint32_t v = (b(i) << 16) | (b(i + 1) << 8) | b(i + 2);
        out += kAlphabet[(v >> 18) & 63];
        out += kAlphabet[(v >> 12) & 63];
        out += i + 1 < bytes.size() ? kAlphabet[(v >> 6) & 63] : '=';
        out += i + 2 < bytes.size() ? kAlphabet[v & 63] : '=';
    }
    return out;
}

TEST(Tiles3d, ContentIsPlacedThroughItsTransformInTheEnuFrame) {
    // One triangle in glTF axes (y up), in a tile whose transform is the ENU frame at a point
    // 100 m east of the world's origin: glTF x is east, -z north, y up.
    const coords::GeoOrigin origin({kLat0 * kRadian, kLon0 * kRadian, 0.0});
    const std::array<float, 9> points{1, 0, 0, 0, 2, 0, 0, 0, -3};
    std::string bytes(sizeof points, '\0');
    std::memcpy(bytes.data(), points.data(), sizeof points);
    std::ostringstream gltf;
    gltf << R"({"asset":{"version":"2.0"},"scenes":[{"nodes":[0]}],"scene":0,)"
         << R"("nodes":[{"mesh":0}],"meshes":[{"primitives":[{"attributes":{"POSITION":0}}]}],)"
         << R"("accessors":[{"bufferView":0,"componentType":5126,"count":3,"type":"VEC3",)"
         << R"("min":[0,0,-3],"max":[1,2,0]}],"bufferViews":[{"buffer":0,"byteLength":36}],)"
         << R"("buffers":[{"byteLength":36,"uri":"data:application/octet-stream;base64,)"
         << base64(bytes) << R"("}]})";
    std::ofstream(scratch("triangle.gltf")) << gltf.str();
    const coords::Ecef at = origin.toEcef({100.0, 0.0, 0.0});
    const std::array<double, 9> r = origin.enuFromEcefRotation();  // rows: east, north, up
    std::ostringstream tileset;
    tileset.precision(17);
    tileset << R"({"asset":{"version":"1.1"},"geometricError":10,"root":{"geometricError":0,)"
            << R"("boundingVolume":{"sphere":[0,0,0,10]},"transform":[)" << r[0] << ',' << r[1]
            << ',' << r[2] << ",0," << r[3] << ',' << r[4] << ',' << r[5] << ",0," << r[6] << ','
            << r[7] << ',' << r[8] << ",0," << at.x << ',' << at.y << ',' << at.z
            << R"(,1],"content":{"uri":"triangle.gltf"}}})";
    std::ofstream(scratch("tileset.json")) << tileset.str();
    const auto content = readTiles3d(scratch("tileset.json").string(), origin);
    ASSERT_TRUE(content) << content.error();
    ASSERT_EQ(content->meshes.size(), 1U);
    const auto& p = content->meshes[0].points;
    ASSERT_EQ(p.size(), 3U);
    const std::array<std::array<double, 3>, 3> expected{
        {{101.0, 0.0, 0.0}, {100.0, 0.0, 2.0}, {100.0, 3.0, 0.0}}};
    for (std::size_t v = 0; v < 3; ++v) {
        for (std::size_t i = 0; i < 3; ++i) {
            EXPECT_NEAR(p[v].at(i), expected.at(v).at(i), 1e-3) << "vertex " << v;
        }
    }
}

}  // namespace
}  // namespace lagrange::geospatial
