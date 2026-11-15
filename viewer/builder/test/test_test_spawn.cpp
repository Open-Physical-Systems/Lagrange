// Test ▾ → Start at…: defaults per world, --spawn's value, the per-world memory.
#include "lagrange/viewer/builder/TestServer.hpp"
#include "lagrange/viewer/builder/TestSpawn.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <unistd.h>

#include <gtest/gtest.h>

using namespace lagrange::viewer::builder;

TEST(TestSpawn, SampleWorldsStartWhereTheirSampleRunsStart) {
    EXPECT_TRUE(defaultSpawn("none").isOrigin());
    EXPECT_TRUE(defaultSpawn("").isOrigin());
    EXPECT_TRUE(defaultSpawn("/a/worlds/warehouse_small/warehouse_small.usda").isOrigin());
    EXPECT_EQ(defaultSpawn("/a/worlds/tiled_town/tiled_town.usda"),
              (SpawnPose{1036.0, 1036.0, 0.0, 0.0}));
    EXPECT_EQ(defaultSpawn("/a/worlds/amr_fms_warehouse/warehouse.sdf"),
              (SpawnPose{26.0, 8.0, 0.0, 180.0}));
    EXPECT_EQ(defaultSpawn("/a/worlds/town/town.usda"), (SpawnPose{30.0, -1.75, 0.0, 0.0}));
}

TEST(TestSpawn, ArgumentIsTheServersSpawnWithTheHeadingInRadians) {
    EXPECT_EQ(spawnArgument({}), "");  // the origin: no --spawn at all
    EXPECT_EQ(spawnArgument({2.0, -1.5, 0.0, 0.0}), "2,-1.5,0,0");
    EXPECT_EQ(spawnArgument({26.0, 8.0, 0.0, 180.0}), "26,8,0,3.141592653589793");
    EXPECT_EQ(spawnArgument({0.0, 0.0, 0.0, 90.0}), "0,0,0,1.5707963267948966");
    EXPECT_EQ(spawnLabel({26.0, 8.0, 0.0, 180.0}), "26, 8, 0 · 180°");
}

// Review L6: far from the origin six significant digits are not enough (12345.67 was passed
// as 12345.7, 7 cm off; 100 km out, 100000.25 as 100000, 25 cm off). The label may round.
TEST(TestSpawn, FarPosesReachTheServerAndTheFileExactly) {
    EXPECT_EQ(spawnArgument({12345.67, -100000.25, 0.0, 0.0}), "12345.67,-100000.25,0,0");
    EXPECT_EQ(spawnArgument({100000.0, 99999.875, 1.5, 0.0}), "100000,99999.875,1.5,0");
    EXPECT_EQ(spawnLabel({12345.67, -100000.25, 0.0, 0.0}), "12345.7, -100000, 0 · 0°");

    const auto file = std::filesystem::temp_directory_path() /
                      ("lagrange_far_spawns_" + std::to_string(::getpid())) / "builder_spawns.tsv";
    std::filesystem::remove_all(file.parent_path());
    const SpawnPose far{12345.67, -100000.25, 0.125, 33.3333};
    {
        SpawnMemory memory;
        memory.load(file);
        memory.set("/w/huge.usda", far);
    }
    SpawnMemory memory;
    memory.load(file);
    EXPECT_EQ(memory.at("/w/huge.usda"), far);  // bit for bit
    std::filesystem::remove_all(file.parent_path());
}

TEST(TestSpawn, ServerGetsSpawnOnlyWhenOneIsChosen) {
    TestRequest request;
    request.program = "/opt/x/lagrange_server";
    request.urdf = "/r/rover.urdf";
    request.world = "/w/warehouse_small.usda";
    request.endpoint = "unix:/r/s.sock";
    std::vector<std::string> args = TestServer::arguments(request);
    EXPECT_EQ(std::count(args.begin(), args.end(), "--spawn"), 0);
    request.spawn = "2,-1.5,0,0.785398";
    args = TestServer::arguments(request);
    const auto at = std::find(args.begin(), args.end(), "--spawn");
    ASSERT_NE(at, args.end());
    ASSERT_NE(at + 1, args.end());
    EXPECT_EQ(*(at + 1), "2,-1.5,0,0.785398");
}

TEST(TestSpawn, MemoryRemembersPerWorldAndSurvivesARestart) {
    const auto file = std::filesystem::temp_directory_path() /
                      ("lagrange_spawns_" + std::to_string(::getpid())) / "builder_spawns.tsv";
    std::filesystem::remove_all(file.parent_path());
    const std::string warehouse = "/a/worlds/warehouse_small/warehouse_small.usda";
    const std::string town = "/a/worlds/town/town.usda";
    {
        SpawnMemory memory;
        memory.load(file);  // missing: nothing remembered
        EXPECT_TRUE(memory.at(warehouse).isOrigin());
        EXPECT_EQ(memory.at(town), defaultSpawn(town));
        memory.set(warehouse, {2.0, -1.5, 0.1, 45.0});
        memory.set("none", {1.0, 1.0, 0.0, 0.0});
        memory.set("/odd path/with spaces/world.sdf", {3.0, 0.0, 0.0, -90.0});
    }
    SpawnMemory memory;
    memory.load(file);
    EXPECT_EQ(memory.at(warehouse), (SpawnPose{2.0, -1.5, 0.1, 45.0}));
    EXPECT_EQ(memory.at(""), (SpawnPose{1.0, 1.0, 0.0, 0.0}));  // the empty world is "none"
    EXPECT_EQ(memory.at("/odd path/with spaces/world.sdf"), (SpawnPose{3.0, 0.0, 0.0, -90.0}));
    memory.reset(warehouse);
    EXPECT_TRUE(memory.at(warehouse).isOrigin());
    // A broken line is skipped, the rest still read.
    {
        std::ofstream out(file, std::ios::app);
        out << "nonsense\n1\t2\tx\t4\t/w.usda\n5\t6\t7\t8\t/v.usda\n";
    }
    memory.load(file);
    EXPECT_TRUE(memory.at("/w.usda").isOrigin());
    EXPECT_EQ(memory.at("/v.usda"), (SpawnPose{5.0, 6.0, 7.0, 8.0}));
    std::filesystem::remove_all(file.parent_path());
}
