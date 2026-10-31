// The humans domain in a run (RFC 0009 validation), on MuJoCo and Jolt where it matters.
#include "lagrange/server/Server.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

namespace fs = std::filesystem;

std::string write(const std::string& name, const std::string& text) {
    const fs::path dir =
        fs::temp_directory_path() / ("lagrange_humans_" + std::to_string(::getpid()));
    fs::create_directories(dir);
    std::ofstream(dir / name) << text;
    return (dir / name).string();
}

std::string person(const std::string& attributes) {
    return write("person.usda",
                 "#usda 1.0\n(\n    metersPerUnit = 1\n    upAxis = \"Z\"\n)\n"
                 "def Xform \"p\" (\n    prepend apiSchemas = [\"HumanAPI_1\"]\n)\n{\n" +
                     attributes + "\n}\n");
}

ServerOptions run(const std::string& robot = "") {
    ServerOptions o;
    o.robotPath = robot;
    o.realTimeFactor = 0.0;
    o.ros = false;
    return o;
}

Pose at(double x, double y) {
    Pose pose;
    pose.position = {x, y, 0.0};
    return pose;
}

double value(const Server& server, const std::string& name) {
    const std::vector<double> v = server.read(name);
    return v.empty() ? -1e9 : v[0];
}

// RFC 0009 validation "Speed": a straight walk averages walkSpeed within 2 %.
TEST(Humans, WalksAtItsSpeedAlongItsWaypoints) {
    const std::string file = person(
        "uniform token human:behavior = \"waypoints\"\n"
        "double[] human:waypoints = [40, 0]\nint human:loop = 0");
    for (const char* physics : {"mujoco", "jolt"}) {
        ServerOptions o = run();
        o.physics = physics;
        Server server(o);
        ASSERT_TRUE(server.load().ok());
        ASSERT_TRUE(server.spawnRobot(file, "ana", at(0.0, 0.0)));
        ASSERT_TRUE(server.stepUntil(SimTime{5'000'000'000}).ok());
        const double x5 = server.read("human/ana/pose").at(0);
        ASSERT_TRUE(server.stepUntil(SimTime{15'000'000'000}).ok());
        const double x15 = server.read("human/ana/pose").at(0);
        EXPECT_NEAR((x15 - x5) / 10.0, 1.34, 0.02 * 1.34) << physics;
        // The body follows the commands through physics.
        const std::vector<double> foot = server.read("body/human_ana_foot_l/pose");
        ASSERT_EQ(foot.size(), 7U);
        EXPECT_NEAR(foot[0], x15, 1.0) << physics;
        EXPECT_LT(foot[2], 0.5) << physics << ": a foot is near the ground";
        EXPECT_NEAR(server.read("human/ana/pose").at(1), 0.0, 0.01) << physics;
    }
}

// Waypoints in order, looping; a goal overrides them.
TEST(Humans, FollowsWaypointsInALoopAndTakesAGoal) {
    Server server(run());
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(server.spawnRobot(person("uniform token human:behavior = \"waypoints\"\n"
                                         "double[] human:waypoints = [3, 0, 3, 3, 0, 3, 0, 0]"),
                                  "ana", at(0.0, 0.0)));
    double farthestX = 0.0;
    double farthestY = 0.0;
    int backHome = 0;
    bool away = false;
    for (std::int64_t t = 1; t <= 300; ++t) {
        ASSERT_TRUE(server.stepUntil(SimTime{t * 100'000'000}).ok());
        const std::vector<double> p = server.read("human/ana/pose");
        farthestX = std::max(farthestX, p[0]);
        farthestY = std::max(farthestY, p[1]);
        away = away || std::hypot(p[0], p[1]) > 2.0;
        if (away && std::hypot(p[0], p[1]) < 0.5) {
            ++backHome;
            away = false;
        }
    }
    EXPECT_GT(farthestX, 2.5);
    EXPECT_GT(farthestY, 2.5);
    EXPECT_GE(backHome, 1) << "round the square and back";
}

TEST(Humans, WanderStaysInItsCircleAndIsRepeatable) {
    const std::string file =
        person("uniform token human:behavior = \"wander\"\ndouble human:wanderRadius = 3");
    const auto once = [&] {
        Server server(run());
        EXPECT_TRUE(server.load().ok());
        EXPECT_TRUE(server.spawnRobot(file, "ana", at(10.0, 10.0)));
        double farthest = 0.0;
        double moved = 0.0;
        for (std::int64_t t = 1; t <= 300; ++t) {
            EXPECT_TRUE(server.stepUntil(SimTime{t * 100'000'000}).ok());
            const std::vector<double> p = server.read("human/ana/pose");
            farthest = std::max(farthest, std::hypot(p[0] - 10.0, p[1] - 10.0));
            moved = std::max(moved, value(server, "human/ana/speed"));
        }
        EXPECT_LT(farthest, 3.5) << "inside the circle";
        EXPECT_GT(moved, 1.0) << "walking";
        return server.stateHash().value();
    };
    EXPECT_EQ(once(), once()) << "the same seed, the same walk";
}

// RFC 0009 validation "Avoidance": a person walking at a stopped robot passes it, keeping clear.
TEST(Humans, WalksRoundARobot) {
    ServerOptions o = run(LAGRANGE_TEST_AMR_URDF);
    o.overlayPath.clear();
    o.demoDrive = false;
    Server server(o);
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(
        server.spawnRobot(person("uniform token human:behavior = \"waypoints\"\n"
                                 "double[] human:waypoints = [6, 0.05]\nint human:loop = 0"),
                          "ana", at(-6.0, 0.0)));
    double closest = 1e9;
    for (std::int64_t t = 1; t <= 200; ++t) {
        ASSERT_TRUE(server.stepUntil(SimTime{t * 100'000'000}).ok());
        closest = std::min(closest, value(server, "human/ana/nearest_robot"));
    }
    std::printf("closest %.2f m\n", closest);
    EXPECT_GT(closest, 0.6) << "kept clear of the robot";
    EXPECT_GT(server.read("human/ana/pose").at(0), 5.0) << "and got past it";
}

// Walls: a person heading through a wall stops at it rather than walking through.
TEST(Humans, DoesNotWalkThroughWalls) {
    const std::string world =
        write("wall.sdf",
              "<?xml version=\"1.0\"?><sdf version=\"1.9\"><world name=\"w\"><model name=\"wall\">"
              "<static>true</static><pose>5 0 1 0 0 0</pose><link name=\"l\"><collision name=\"c\">"
              "<geometry><box><size>0.2 40 2</size></box></geometry></collision></link></model>"
              "</world></sdf>\n");
    ServerOptions o = run();
    o.worldPath = world;
    Server server(o);
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(server.spawnRobot(person("uniform token human:behavior = \"waypoints\"\n"
                                         "double[] human:waypoints = [10, 0]\nint human:loop = 0"),
                                  "ana", at(0.0, 0.0)));
    double farthest = -1e9;
    for (std::int64_t t = 1; t <= 150; ++t) {
        ASSERT_TRUE(server.stepUntil(SimTime{t * 100'000'000}).ok());
        farthest = std::max(farthest, server.read("human/ana/pose").at(0));
    }
    EXPECT_GT(farthest, 3.5) << "it walked up to the wall";
    EXPECT_LT(farthest, 4.9) << "and not through it";
}

// RFC 0009 validation "Sensors": a lidar at knee height sees the legs swap places: a person's
// silhouette changes with the gait, which a capsule's would not. Measured through a robot's lidar
// in the world; here the scene query behind it: the person is seen by the sample AMR's lidar.
TEST(Humans, TheLidarSeesThePerson) {
    ServerOptions o = run(LAGRANGE_TEST_AMR_URDF);
    o.overlayPath.clear();
    o.demoDrive = false;
    Server server(o);
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(server.stepUntil(SimTime{200'000'000}).ok());
    const std::vector<double> empty = server.read("sensor/front_lidar/ranges");
    ASSERT_FALSE(empty.empty());
    std::size_t before = 0;
    for (const double r : empty) {
        before += std::isfinite(r) ? 1U : 0U;
    }
    ASSERT_TRUE(server.spawnRobot(LAGRANGE_TEST_ADULT_USDA, "ana", at(2.0, 0.0)));
    ASSERT_TRUE(server.stepUntil(SimTime{800'000'000}).ok());
    std::size_t after = 0;
    for (const double r : server.read("sensor/front_lidar/ranges")) {
        after += std::isfinite(r) ? 1U : 0U;
    }
    EXPECT_GT(after, before) << "returns from the person";
}

// RFC 0009 validation "Contact": contact is one-way. A robot driven into a person is stopped or
// turned by them, and the person does not move.
TEST(Humans, ARobotCannotPushAPerson) {
    const auto drive = [](bool withPerson) {
        ServerOptions o = run(LAGRANGE_TEST_AMR_URDF);
        o.overlayPath.clear();
        Server server(o);
        EXPECT_TRUE(server.load().ok());
        if (withPerson) {
            EXPECT_TRUE(server.spawnRobot(LAGRANGE_TEST_ADULT_USDA, "ana", at(0.9, 0.0)));
        }
        EXPECT_TRUE(server.stepUntil(SimTime{4'000'000'000}).ok());
        if (withPerson) {
            EXPECT_NEAR(server.read("body/human_ana_pelvis/pose").at(0), 0.9, 1e-9);
            EXPECT_NEAR(server.read("body/human_ana_pelvis/pose").at(1), 0.0, 1e-9);
        }
        return server.read("body/base_link/pose");
    };
    const std::vector<double> free = drive(false);
    const std::vector<double> blocked = drive(true);
    EXPECT_GT(std::hypot(free[0] - blocked[0], free[1] - blocked[1]), 0.1)
        << "the person got in the way";
}

// RFC 0009 validation "Sensors": a lidar scanning at shin height sees two legs apart, then
// passing each other, as a person crosses in front of it. A capsule would stay one blob.
TEST(Humans, ALidarSeesTwoLegsThatScissor) {
    ServerOptions o = run(LAGRANGE_TEST_AMR_URDF);
    o.overlayPath.clear();
    o.demoDrive = false;
    Server server(o);
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(
        server.spawnRobot(person("uniform token human:behavior = \"waypoints\"\n"
                                 "double[] human:waypoints = [2.5, 20]\nint human:loop = 0\n"
                                 "double human:walkSpeed = 1.0"),
                          "ana", at(2.5, -3.0)));
    std::size_t fewest = 99;
    std::size_t most = 0;
    for (std::int64_t t = 20; t <= 60; ++t) {  // crossing in front
        ASSERT_TRUE(server.stepUntil(SimTime{t * 100'000'000}).ok());
        const std::vector<double> ranges = server.read("sensor/front_lidar/ranges");
        ASSERT_FALSE(ranges.empty());
        std::size_t clusters = 0;
        bool inside = false;
        double last = 0.0;
        for (const double r : ranges) {
            const bool hit = std::isfinite(r);
            if (hit && (!inside || std::abs(r - last) > 0.3)) {
                ++clusters;
            }
            inside = hit;
            last = r;
        }
        if (clusters > 0) {
            fewest = std::min(fewest, clusters);
            most = std::max(most, clusters);
        }
    }
    EXPECT_EQ(most, 2U) << "two legs apart";
    EXPECT_EQ(fewest, 1U) << "one blob as they pass, side on";
}

// SDFormat <actor> (RFC 0009 §2): Gazebo's people walk their trajectory here, at its speed.
TEST(Humans, AGazeboActorWalksItsTrajectory) {
    const std::string world = write(
        "actor.sdf",
        "<?xml version=\"1.0\"?><sdf version=\"1.9\"><world name=\"w\"><actor name=\"bob\">"
        "<pose>1 2 0 0 0 0</pose><script><loop>false</loop><trajectory id=\"0\" type=\"walk\">"
        "<waypoint><time>0</time><pose>0 0 0 0 0 0</pose></waypoint>"
        "<waypoint><time>10</time><pose>10 0 0 0 0 0</pose></waypoint>"
        "</trajectory></script></actor></world></sdf>\n");
    ServerOptions o = run();
    o.worldPath = world;
    Server server(o);
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    EXPECT_NEAR(server.read("human/bob/pose").at(0), 1.0, 1e-6);
    EXPECT_NEAR(server.read("human/bob/pose").at(1), 2.0, 1e-6);
    ASSERT_TRUE(server.stepUntil(SimTime{6'000'000'000}).ok());
    EXPECT_NEAR(server.read("human/bob/pose").at(0), 1.0 + 1.0 * 5.5, 1.0) << "1 m/s, as scripted";
    ASSERT_TRUE(server.stepUntil(SimTime{15'000'000'000}).ok());
    EXPECT_NEAR(server.read("human/bob/pose").at(0), 11.0, 0.4) << "stops at the end";
}

// --humans: a crowd placed clear of the walls and each other.
TEST(Humans, ACrowdIsPlacedClear) {
    ServerOptions o = run(LAGRANGE_TEST_AMR_URDF);
    o.overlayPath.clear();
    o.worldPath = LAGRANGE_TEST_WAREHOUSE_URDF;
    o.humanCount = 6;
    o.humanRadius = 7.0;
    Server server(o);
    ASSERT_TRUE(server.load().ok());
    EXPECT_EQ(server.read("human/person_5/speed").size(), 1U);
    ASSERT_TRUE(server.stepUntil(SimTime{10'000'000'000}).ok());
    EXPECT_GT(server.read("human/person_0/nearest_robot").at(0), 0.0);
}

// RFC 0009 §7: off, nobody is there.
TEST(Humans, OffThereIsNobody) {
    ServerOptions o = run();
    o.substituted.insert("humans");
    Server server(o);
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(server.spawnRobot(LAGRANGE_TEST_ADULT_USDA, "ana", at(0.0, 0.0)));
    ASSERT_TRUE(server.stepUntil(SimTime{100'000'000}).ok());
    EXPECT_LT(server.read("human/ana/pose").at(2), -50.0);
}

// What a crowd costs (RFC 0009 §9): prints seconds of simulation per second of wall clock.
TEST(Humans, CostOfACrowd) {
    std::ostringstream usda;
    usda << "#usda 1.0\n(\n    metersPerUnit = 1\n    upAxis = \"Z\"\n)\n";
    constexpr int kPeople = 50;
    for (int i = 0; i < kPeople; ++i) {
        usda << "def Xform \"p" << i << "\" (\n    prepend apiSchemas = [\"HumanAPI_1\"]\n)\n{\n"
             << "    double3 xformOp:translate = (" << (i % 10) * 3 << ", " << (i / 10) * 3
             << ", 0)\n"
             << "    uniform token[] xformOpOrder = [\"xformOp:translate\"]\n"
             << "    uniform token human:behavior = \"wander\"\n    int human:seed = " << i
             << "\n}\n";
    }
    ServerOptions o = run(LAGRANGE_TEST_AMR_URDF);
    o.overlayPath.clear();
    o.worldPath = write("crowd.usda", usda.str());
    Server server(o);
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_EQ(server.read("human/p49/speed").size(), 1U);
    EXPECT_NEAR(server.read("body/human_p49_pelvis/pose").at(0), 27.0, 1e-9)
        << "where the file says";
    const auto start = std::chrono::steady_clock::now();
    ASSERT_TRUE(server.stepUntil(SimTime{5'000'000'000}).ok());
    const double wall =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    std::printf("%d people and the sample AMR: %.2f s simulated per s of wall clock\n", kPeople,
                5.0 / wall);
    EXPECT_GT(5.0 / wall, 1.0) << "faster than real time";
}

}  // namespace
}  // namespace lagrange::server
