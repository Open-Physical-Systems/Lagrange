// The occupancy map: geometry and bounds of a small known scene.
#include "lagrange/server/MapExport.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

struct Pgm {
    std::size_t width{0}, height{0};
    std::vector<unsigned char> pixels;
    [[nodiscard]] unsigned char at(std::size_t col, std::size_t row) const {
        return pixels[row * width + col];
    }
};

Pgm readPgm(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    std::string magic, comment;
    int maxValue = 0;
    Pgm pgm;
    in >> magic;
    in.ignore();
    std::getline(in, comment);
    in >> pgm.width >> pgm.height >> maxValue;
    in.ignore();
    pgm.pixels.resize(pgm.width * pgm.height);
    in.read(reinterpret_cast<char*>(pgm.pixels.data()),
            static_cast<std::streamsize>(pgm.pixels.size()));
    EXPECT_EQ(magic, "P5");
    EXPECT_EQ(maxValue, 255);
    return pgm;
}

class MapExportTest : public ::testing::Test {
protected:
    void SetUp() override {
        dir_ = std::filesystem::path(::testing::TempDir()) / "lagrange_map_export";
        std::filesystem::create_directories(dir_);
    }
    std::filesystem::path dir_;
};

TEST_F(MapExportTest, MapsFixedShapesAtTheSliceAndLeavesTheRobotOut) {
    World world;
    world.addCollider(kWorldBody, ShapeType::Plane, {0, 0, 0}, Pose{}, 1.0);
    // A 2 m x 1 m wall from x = 0 to 2, y = 0 to 1, 1 m tall.
    world.addCollider(kWorldBody, ShapeType::Box, {1.0, 0.5, 0.5}, Pose{{1.0, 0.5, 0.5}}, 1.0);
    // A cylinder post of radius 0.5 at (4, 0.5).
    world.addCollider(kWorldBody, ShapeType::Cylinder, {0.5, 0.5, 0}, Pose{{4.0, 0.5, 0.5}}, 1.0);
    // A shelf above the slice: inside the bounds but not in the map.
    world.addCollider(kWorldBody, ShapeType::Box, {0.5, 0.5, 0.1}, Pose{{4.0, 0.5, 2.0}}, 1.0);
    // The robot, on a free joint: never mapped.
    const BodyId robot = world.addBody("robot", kWorldBody, Pose{{2.5, 0.5, 0.2}}, 1.0, {}, {});
    world.addJoint("robot_free", JointType::Free, robot, {0, 0, 1}, false);
    world.addCollider(robot, ShapeType::Sphere, {0.2, 0, 0}, Pose{}, 1.0);

    MapSettings settings;
    settings.heightM = 0.2;
    settings.resolutionM = 0.1;
    settings.marginM = 1.0;
    const std::filesystem::path yaml = dir_ / "scene.yaml";
    ASSERT_TRUE(exportOccupancyMap(world, yaml.string(), settings));

    std::ifstream text(yaml);
    std::stringstream contents;
    contents << text.rdbuf();
    EXPECT_NE(contents.str().find("image: scene.pgm"), std::string::npos);
    EXPECT_NE(contents.str().find("origin: [-1, -1, 0.0]"), std::string::npos) << contents.str();

    // Bounds: x from -1 to 5.5, y from -1 to 2 (shapes plus the margin).
    const Pgm pgm = readPgm(dir_ / "scene.pgm");
    ASSERT_EQ(pgm.width, 65U);
    ASSERT_EQ(pgm.height, 30U);
    const auto cell = [&](double x, double y) {
        const auto col = static_cast<std::size_t>(std::floor((x + 1.0) / 0.1));
        const auto row = pgm.height - 1 - static_cast<std::size_t>(std::floor((y + 1.0) / 0.1));
        return pgm.at(col, row);
    };
    EXPECT_EQ(cell(1.0, 0.5), 0) << "inside the wall";
    EXPECT_EQ(cell(1.95, 0.95), 0) << "wall corner";
    EXPECT_EQ(cell(4.0, 0.5), 0) << "post centre";
    EXPECT_EQ(cell(4.0, 0.05), 0) << "post edge";
    EXPECT_EQ(cell(4.45, 0.95), 254) << "outside the post's circle";
    EXPECT_EQ(cell(2.5, 0.5), 254) << "the robot is not mapped";
    EXPECT_EQ(cell(-0.5, -0.5), 254) << "margin";
}

TEST_F(MapExportTest, RejectsAWorldWithoutFixedGeometry) {
    World world;
    world.addCollider(kWorldBody, ShapeType::Plane, {0, 0, 0}, Pose{}, 1.0);
    EXPECT_FALSE(exportOccupancyMap(world, (dir_ / "empty.yaml").string()));
}

}  // namespace
}  // namespace lagrange::server
