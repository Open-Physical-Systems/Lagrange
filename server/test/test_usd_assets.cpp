// The sample assets in USD (made by lagrange_import from the URDFs) simulate as the URDFs do:
// same bodies, joints, colliders and sensors; the SDFormat and MJCF versions convert the same way.
// If this fails after a URDF change, convert again:
//   lagrange_import assets/robots/amr_diffdrive/amr_diffdrive.urdf
//                   assets/robots/amr_diffdrive/amr_diffdrive.usda
#include "lagrange/server/MapExport.hpp"
#include "lagrange/server/Robot.hpp"
#include "lagrange/stage/Stage.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

std::size_t named(const std::vector<std::string>& names, const std::string& name) {
    const auto it = std::find(names.begin(), names.end(), name);
    EXPECT_NE(it, names.end()) << name;
    return static_cast<std::size_t>(it - names.begin());
}

/// `b` (read back from USD) has the bodies, joints, colliders and sensors of `a`.
void expectSameModel(const Robot& a0, const Robot& b0) {
    const World& a = a0.world;
    const World& b = b0.world;
    EXPECT_EQ(b0.name, a0.name);
    EXPECT_TRUE(b0.notes.empty()) << b0.notes.front();
    constexpr double kFloat = 2e-5;  // UsdPhysics stores masses and joint frames as float

    ASSERT_EQ(b.bodies.names.size(), a.bodies.names.size());
    for (std::size_t i = 0; i < a.bodies.names.size(); ++i) {
        const std::size_t j = named(b.bodies.names, a.bodies.names[i]);
        EXPECT_NEAR(b.bodies.mass[j], a.bodies.mass[i], kFloat) << a.bodies.names[i];
        for (std::size_t k = 0; k < 6; ++k) {
            EXPECT_NEAR(b.bodies.inertiaLocal[j][k], a.bodies.inertiaLocal[i][k], kFloat);
        }
        for (std::size_t k = 0; k < 3; ++k) {
            EXPECT_NEAR(b0.bodyRestPose[j].position[k], a0.bodyRestPose[i].position[k], 1e-9)
                << a.bodies.names[i];
        }
    }
    ASSERT_EQ(b.joints.names.size(), a.joints.names.size());
    for (std::size_t i = 0; i < a.joints.names.size(); ++i) {
        const std::size_t j = named(b.joints.names, a.joints.names[i]);
        EXPECT_EQ(b.joints.type[j], a.joints.type[i]) << a.joints.names[i];
        EXPECT_EQ(b.joints.actuated[j], a.joints.actuated[i]) << a.joints.names[i];
        if (a.joints.type[i] != JointType::Free) {
            for (std::size_t k = 0; k < 3; ++k) {
                EXPECT_NEAR(b.joints.axis[j][k], a.joints.axis[i][k], kFloat) << a.joints.names[i];
            }
        }
    }
    EXPECT_EQ(b0.actuatedJoints, a0.actuatedJoints);
    ASSERT_EQ(b.colliders.body.size(), a.colliders.body.size());
    for (const ShapeType shape :
         {ShapeType::Plane, ShapeType::Box, ShapeType::Cylinder, ShapeType::Sphere}) {
        EXPECT_EQ(std::count(b.colliders.shape.begin(), b.colliders.shape.end(), shape),
                  std::count(a.colliders.shape.begin(), a.colliders.shape.end(), shape));
    }
    ASSERT_EQ(b.sensors.names, a.sensors.names);
    for (std::size_t s = 0; s < a.sensors.names.size(); ++s) {
        EXPECT_EQ(b.sensors.settings[s], a.sensors.settings[s]) << a.sensors.names[s];
        EXPECT_EQ(b.bodies.names[b.sensors.body[s].index], a.bodies.names[a.sensors.body[s].index]);
    }
}

// --spawn moves a USD, SDFormat or MJCF robot as it moves a URDF one: the same bodies end up
// in the same places.
TEST(UsdAssets, SpawnMovesEveryFormatLikeUrdf) {
    LoadOptions options;
    options.spawn.position = {4.0, -2.0, 0.0};
    options.spawn.orientation = {std::cos(0.35), 0.0, 0.0, std::sin(0.35)};  // 0.7 rad about z
    auto urdf = loadModel(LAGRANGE_TEST_AMR_URDF, options);
    ASSERT_TRUE(urdf) << urdf.error();
    for (const char* path :
         {LAGRANGE_TEST_AMR_USDA, LAGRANGE_TEST_AMR_SDF, LAGRANGE_TEST_AMR_MJCF}) {
        auto other = loadModel(path, options);
        ASSERT_TRUE(other) << other.error();
        // The files list the bodies in different orders: match them by name.
        const auto& names = other->world.bodies.names;
        for (std::size_t b = 0; b < urdf->bodyRestPose.size(); ++b) {
            const std::string& name = urdf->world.bodies.names[b];
            const auto it = std::find(names.begin(), names.end(), name);
            ASSERT_NE(it, names.end()) << path << ": " << name;
            const Pose& got = other->bodyRestPose[static_cast<std::size_t>(it - names.begin())];
            const Pose& want = urdf->bodyRestPose[b];
            for (std::size_t i = 0; i < 3; ++i) {
                EXPECT_NEAR(got.position[i], want.position[i], 1e-5) << path << ": " << name;
            }
            // Same rotation: q and -q are the same.
            double dot = 0.0;
            for (std::size_t i = 0; i < 4; ++i) {
                dot += got.orientation[i] * want.orientation[i];
            }
            EXPECT_NEAR(std::abs(dot), 1.0, 1e-6) << path << ": " << name;
        }
    }
}

TEST(UsdAssets, TheSampleAmrInUsdMatchesItsUrdf) {
    auto urdf = loadModel(LAGRANGE_TEST_AMR_URDF);
    auto usd = loadModel(LAGRANGE_TEST_AMR_USDA);
    ASSERT_TRUE(urdf) << urdf.error();
    ASSERT_TRUE(usd) << usd.error();
    expectSameModel(*urdf, *usd);
}

TEST(UsdAssets, TheSdfAndMjcfVersionsConvertToUsd) {
    // Acceptance criterion 7: SDFormat and MJCF import to USD that loads back the same robot
    // (test_robot.cpp drives the three versions and compares them).
    const std::filesystem::path dir =
        std::filesystem::path(::testing::TempDir()) / "lagrange_usd_assets";
    std::filesystem::create_directories(dir);
    for (const char* path : {LAGRANGE_TEST_AMR_SDF, LAGRANGE_TEST_AMR_MJCF}) {
        auto source = loadModel(path);
        ASSERT_TRUE(source) << source.error();
        const std::string usda =
            (dir / (std::filesystem::path(path).extension().string().substr(1) + ".usda")).string();
        ASSERT_TRUE(stage::writeStage(*source, usda)) << path;
        auto usd = loadModel(usda);
        ASSERT_TRUE(usd) << usd.error();
        expectSameModel(*source, *usd);
    }
}

TEST(UsdAssets, TheWarehouseInUsdMapsLikeItsUrdf) {
    // The scenery becomes world colliders in USD (static, massless links need no body); the
    // occupancy map, which only sees fixed geometry, must come out the same.
    auto urdf = loadModel(LAGRANGE_TEST_AMR_URDF);
    auto usd = loadModel(LAGRANGE_TEST_AMR_USDA);
    ASSERT_TRUE(urdf && usd);
    ASSERT_TRUE(addModel(*urdf, LAGRANGE_TEST_WAREHOUSE_URDF));
    ASSERT_TRUE(addModel(*usd, LAGRANGE_TEST_WAREHOUSE_USDA));
    const std::filesystem::path dir =
        std::filesystem::path(::testing::TempDir()) / "lagrange_usd_assets";
    std::filesystem::create_directories(dir);
    ASSERT_TRUE(exportOccupancyMap(urdf->world, (dir / "urdf.yaml").string()));
    ASSERT_TRUE(exportOccupancyMap(usd->world, (dir / "usd.yaml").string()));
    // Box sizes pass through float scales in USD, so a bound may round to the next cell:
    // compare the image sizes to one cell and the occupied area to 1 %.
    struct Pgm {
        int width{0}, height{0};
        long occupied{0};
    };
    const auto read = [](const std::filesystem::path& path) {
        std::ifstream in(path, std::ios::binary);
        std::string magic, comment;
        Pgm pgm;
        int maxValue = 0;
        in >> magic;
        in.ignore();
        std::getline(in, comment);
        in >> pgm.width >> pgm.height >> maxValue;
        in.ignore();
        std::string pixels(static_cast<std::size_t>(pgm.width) * pgm.height, '\0');
        in.read(pixels.data(), static_cast<std::streamsize>(pixels.size()));
        pgm.occupied = std::count(pixels.begin(), pixels.end(), '\0');
        return pgm;
    };
    const Pgm a = read(dir / "urdf.pgm");
    const Pgm b = read(dir / "usd.pgm");
    EXPECT_NEAR(b.width, a.width, 1);
    EXPECT_NEAR(b.height, a.height, 1);
    ASSERT_GT(a.occupied, 1000);
    EXPECT_NEAR(static_cast<double>(b.occupied), static_cast<double>(a.occupied),
                0.01 * static_cast<double>(a.occupied));
}

/// The electrical schema instances of a robot as text, by schema and prim path, relationship
/// targets by path: models loaded different ways compare equal when their data agree.
std::map<std::string, std::string> electrical(const Robot& robot) {
    const WorldView view(robot.world);
    std::map<std::string, std::string> out;
    for (const SchemaInstance& s : robot.world.schemaInstances) {
        if (s.schema.rfind("Sensor", 0) == 0 || s.schema.rfind("Lidar", 0) == 0 ||
            s.schema.rfind("Imu", 0) == 0) {
            continue;
        }
        std::ostringstream text;
        text.precision(17);
        for (const auto& [k, v] : s.reals) {
            text << k << '=' << v << '\n';
        }
        for (const auto& [k, v] : s.integers) {
            text << k << '=' << v << '\n';
        }
        for (const auto& [k, v] : s.tokens) {
            text << k << '=' << v << '\n';
        }
        for (const auto& [k, v] : s.realArrays) {
            text << k << '=';
            for (const double x : v) {
                text << x << ' ';
            }
            text << '\n';
        }
        for (const auto& [k, targets] : s.relationships) {
            text << k << "->";
            for (const PrimId t : targets) {
                text << view.primPath(t) << ' ';
            }
            text << '\n';
        }
        const std::string key = s.schema + (s.instance.empty() ? "" : ":" + s.instance) + " " +
                                std::string(view.primPath(s.prim));
        EXPECT_TRUE(out.emplace(key, text.str()).second) << "twice: " << key;
    }
    return out;
}

TEST(Overlay, TheImportedAmrGainsThePowerSystemOfTheHandWrittenSample) {
    // Roadmap 0.2-G2: the URDF with the overlay applied, the hand-written USD sample, and the
    // URDF converted with lagrange_import --overlay carry the same motors, battery, bus and load.
    auto robot = loadModel(LAGRANGE_TEST_AMR_URDF);
    ASSERT_TRUE(robot) << robot.error();
    ASSERT_TRUE(stage::applyOverlay(*robot, LAGRANGE_TEST_AMR_OVERLAY));
    auto handWritten = loadModel(LAGRANGE_TEST_AMR_POWERED_USDA);
    ASSERT_TRUE(handWritten) << handWritten.error();

    const std::map<std::string, std::string> overlaid = electrical(*robot);
    // Two motors, the battery and its port, the bus, the load; four thermal nodes (motors,
    // battery, chassis) and three links to the chassis.
    EXPECT_EQ(overlaid.size(), 13U);
    EXPECT_TRUE(overlaid.contains("ThermalLinkAPI_1:to_chassis /amr_diffdrive/power/battery"));
    EXPECT_TRUE(
        overlaid.contains("ElectricalMotorAPI_1 /amr_diffdrive/base_link/left_drive_motor"));
    EXPECT_TRUE(overlaid.contains("PortAPI_1:dc_out /amr_diffdrive/power/battery"));
    EXPECT_EQ(overlaid, electrical(*handWritten));

    const std::filesystem::path converted =
        std::filesystem::path(::testing::TempDir()) / "amr_diffdrive_powered_import.usda";
    ASSERT_TRUE(stage::writeStage(*robot, converted.string()));
    auto readBack = loadModel(converted.string());
    ASSERT_TRUE(readBack) << readBack.error();
    EXPECT_EQ(overlaid, electrical(*readBack));

    // The motors are in the scene for the viewer, on the chassis, with their schema listed.
    const auto motor =
        std::find_if(robot->scene.prims.begin(), robot->scene.prims.end(),
                     [](const ScenePrim& p) { return p.name == "left_drive_motor"; });
    ASSERT_NE(motor, robot->scene.prims.end());
    EXPECT_EQ(motor->path, "/amr_diffdrive/base_link/left_drive_motor");
    EXPECT_EQ(motor->body, robot->scene.prims[robot->bodyPrim[0]].body);
    EXPECT_NE(std::find(motor->apiSchemas.begin(), motor->apiSchemas.end(), "ElectricalMotorAPI_1"),
              motor->apiSchemas.end());
}

}  // namespace
}  // namespace lagrange::server
