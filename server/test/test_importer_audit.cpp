// What robot files say and the importers once dropped: mesh and ellipsoid colliders, a height
// field, Gazebo's friction.
#include "lagrange/server/Robot.hpp"
#include "lagrange/server/Server.hpp"

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <unistd.h>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

std::filesystem::path scratch(const std::string& name) {
    const std::filesystem::path dir =
        std::filesystem::temp_directory_path() / ("lagrange_audit_" + std::to_string(::getpid()));
    std::filesystem::create_directories(dir);
    return dir / name;
}

using Triangle = std::array<std::array<float, 3>, 3>;

/// The twelve triangles of a box, outward.
void addBox(std::vector<Triangle>& out, std::array<float, 3> lo, std::array<float, 3> hi) {
    const auto p = [&](int x, int y, int z) {
        return std::array<float, 3>{x != 0 ? hi[0] : lo[0], y != 0 ? hi[1] : lo[1],
                                    z != 0 ? hi[2] : lo[2]};
    };
    const auto quad = [&](auto a, auto b, auto c, auto d) {
        out.push_back({a, b, c});
        out.push_back({a, c, d});
    };
    quad(p(0, 0, 0), p(0, 1, 0), p(1, 1, 0), p(1, 0, 0));
    quad(p(0, 0, 1), p(1, 0, 1), p(1, 1, 1), p(0, 1, 1));
    quad(p(0, 0, 0), p(1, 0, 0), p(1, 0, 1), p(0, 0, 1));
    quad(p(0, 1, 0), p(0, 1, 1), p(1, 1, 1), p(1, 1, 0));
    quad(p(0, 0, 0), p(0, 0, 1), p(0, 1, 1), p(0, 1, 0));
    quad(p(1, 0, 0), p(1, 1, 0), p(1, 1, 1), p(1, 0, 1));
}

std::string writeStl(const std::string& name, const std::vector<Triangle>& triangles) {
    const std::filesystem::path path = scratch(name);
    std::ofstream file(path, std::ios::binary);
    const std::array<char, 80> header{};
    file.write(header.data(), header.size());
    const auto count = static_cast<std::uint32_t>(triangles.size());
    file.write(reinterpret_cast<const char*>(&count), sizeof(count));
    for (const Triangle& t : triangles) {
        const std::array<float, 3> normal{};
        file.write(reinterpret_cast<const char*>(normal.data()), sizeof(normal));
        file.write(reinterpret_cast<const char*>(t.data()), sizeof(t));
        const std::uint16_t attribute = 0;
        file.write(reinterpret_cast<const char*>(&attribute), sizeof(attribute));
    }
    return path.string();
}

TriMesh meshOf(const std::vector<std::array<float, 3>>& points) {
    TriMesh mesh;
    for (const auto& p : points) {
        mesh.positions.insert(mesh.positions.end(), p.begin(), p.end());
    }
    return mesh;
}

void expectConvexAround(const ConvexHull& hull, const TriMesh& mesh, double scale = 1.0) {
    for (const auto& t : hull.triangles) {
        const auto& a = hull.points.at(t[0]);
        const auto& b = hull.points.at(t[1]);
        const auto& c = hull.points.at(t[2]);
        const std::array<double, 3> u{b[0] - a[0], b[1] - a[1], b[2] - a[2]};
        const std::array<double, 3> v{c[0] - a[0], c[1] - a[1], c[2] - a[2]};
        const std::array<double, 3> n{u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
                                      u[0] * v[1] - u[1] * v[0]};
        const double length = std::hypot(n[0], n[1], n[2]);
        for (std::size_t i = 0; i + 2 < mesh.positions.size(); i += 3) {
            const double out = (n[0] * (mesh.positions[i] * scale - a[0]) +
                                n[1] * (mesh.positions[i + 1] * scale - a[1]) +
                                n[2] * (mesh.positions[i + 2] * scale - a[2])) /
                               length;
            EXPECT_LE(out, 1e-3) << "a point outside a face";
        }
    }
}

TEST(ConvexHullOf, ACubeWithPointsInsideIsItsCorners) {
    std::vector<std::array<float, 3>> points;
    for (int k = 0; k < 8; ++k) {
        points.push_back({(k & 1) != 0 ? 1.0F : -1.0F, (k & 2) != 0 ? 1.0F : -1.0F,
                          (k & 4) != 0 ? 1.0F : -1.0F});
    }
    points.push_back({0.0F, 0.0F, 0.0F});
    points.push_back({0.5F, -0.2F, 0.9F});
    const TriMesh mesh = meshOf(points);
    const std::optional<ConvexHull> hull = convexHullOf(mesh);
    ASSERT_TRUE(hull);
    EXPECT_EQ(hull->points.size(), 8U);
    EXPECT_EQ(hull->triangles.size(), 12U);
    expectConvexAround(*hull, mesh);
}

TEST(ConvexHullOf, ManyPointsAreCutDownAndFlatOnesRefused) {
    const TriMesh sphere = sphereMesh(1.0, 64, 32);
    const std::optional<ConvexHull> hull = convexHullOf(sphere, {2.0, 2.0, 2.0});
    ASSERT_TRUE(hull);
    EXPECT_LE(hull->points.size(), kMaxHullPoints);
    EXPECT_GT(hull->points.size(), 100U);
    for (const auto& p : hull->points) {
        EXPECT_NEAR(std::hypot(p[0], p[1], p[2]), 2.0, 1e-5);
    }
    // A flat shape keeps points on its faces, not only on its rim.
    const std::optional<ConvexHull> disc = convexHullOf(sphere, {1.0, 1.0, 0.05});
    ASSERT_TRUE(disc);
    int nearPole = 0;
    for (const auto& p : disc->points) {
        nearPole += std::hypot(p[0], p[1]) < 0.3 ? 1 : 0;
    }
    EXPECT_GT(nearPole, 8);
    EXPECT_FALSE(convexHullOf(meshOf({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}})));
    EXPECT_FALSE(convexHullOf(meshOf({{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}})));
    EXPECT_FALSE(convexHullOf(meshOf({{0, 0, 0}, {1, 0, 0}})));
    const ConvexHull ball = ellipsoidHull({1.0, 2.0, 3.0});
    double top = 0.0;
    for (const auto& p : ball.points) {
        top = std::max(top, p[2]);
    }
    EXPECT_NEAR(top, 3.0, 1e-5);
}

std::string urdfWithMesh(const std::string& mesh, const std::string& extra) {
    const std::filesystem::path path = scratch("mesh_robot.urdf");
    std::ofstream(path) << R"(<?xml version="1.0"?><robot name="block"><link name="body">
<inertial><mass value="2"/><inertia ixx="0.02" iyy="0.02" izz="0.02" ixy="0" ixz="0" iyz="0"/>
</inertial><collision><geometry><mesh filename="file://)"
                        << mesh << R"("/></geometry></collision></link>)" << extra << "</robot>";
    return path.string();
}

ServerOptions dropped(const std::string& robot, const std::string& physics) {
    ServerOptions o;
    o.robotPath = robot;
    o.worldPath.clear();
    o.physics = physics;
    o.ros = false;
    o.realTimeFactor = 0.0;
    return o;
}

// A robot whose only collision is a mesh once fell through the ground.
TEST(MeshColliders, AUrdfMeshCollidesOnBothEngines) {
    std::vector<Triangle> box;
    addBox(box, {-0.2F, -0.2F, -0.1F}, {0.2F, 0.2F, 0.1F});
    const std::string robot = urdfWithMesh(writeStl("box.stl", box), "");
    for (const char* physics : {"mujoco", "jolt"}) {
        Server server(dropped(robot, physics));
        const Status loaded = server.load();
        ASSERT_TRUE(loaded.ok()) << loaded.message;
        ASSERT_TRUE(server.stepUntil(SimTime{2'000'000'000}).ok());
        EXPECT_NEAR(server.read("body/body/pose").at(2), 0.1, 0.005) << physics;
    }
}

TEST(MeshColliders, ConcaveIsAHullWhenItMovesAndSkippedWhenFixed) {
    std::vector<Triangle> two;  // two blocks in one file, far apart
    addBox(two, {-1.0F, -0.1F, -0.1F}, {-0.8F, 0.1F, 0.1F});
    addBox(two, {0.8F, -0.1F, -0.1F}, {1.0F, 0.1F, 0.1F});
    const std::string stl = writeStl("two.stl", two);
    auto moving = loadUrdf(urdfWithMesh(stl, ""), {});
    ASSERT_TRUE(moving) << moving.error();
    EXPECT_EQ(moving->world.hulls.size(), 1U);
    bool told = false;
    for (const std::string& note : moving->notes) {
        told = told || note.find("concave") != std::string::npos;
    }
    EXPECT_TRUE(told);
    // A link welded to the world, as a world's scenery is.
    const std::filesystem::path scene = scratch("room.urdf");
    std::ofstream(scene) << R"(<?xml version="1.0"?><robot name="room"><link name="world"/>
<link name="walls"><collision><geometry><mesh filename="file://)"
                         << stl << R"("/></geometry></collision></link>
<joint name="fix" type="fixed"><parent link="world"/><child link="walls"/></joint></robot>)";
    auto still = loadUrdf(scene.string(), {});
    ASSERT_TRUE(still) << still.error();
    EXPECT_TRUE(still->world.hulls.empty()) << "its hull would fill the room";
}

TEST(MeshColliders, GazeboFrictionReachesTheLink) {
    std::vector<Triangle> box;
    addBox(box, {-0.2F, -0.2F, -0.1F}, {0.2F, 0.2F, 0.1F});
    const std::string robot = urdfWithMesh(writeStl("box2.stl", box),
                                           R"(<gazebo reference="body"><mu1>0.3</mu1></gazebo>)");
    auto model = loadUrdf(robot, {});
    ASSERT_TRUE(model) << model.error();
    bool found = false;
    for (std::size_t c = 0; c < model->world.colliders.body.size(); ++c) {
        if (model->world.colliders.body[c] != kWorldBody) {
            EXPECT_DOUBLE_EQ(model->world.colliders.friction[c], 0.3);
            found = true;
        }
    }
    EXPECT_TRUE(found);
}

TEST(SdfColliders, AnEllipsoidRestsOnItsRadius) {
    const std::filesystem::path path = scratch("egg.sdf");
    std::ofstream(path) << R"(<?xml version="1.0"?><sdf version="1.9"><world name="w">
<model name="egg"><pose>0 0 0.5 0 0 0</pose><link name="egg_link"><inertial><mass>1</mass><inertia>
<ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial><collision name="c"><geometry>
<ellipsoid><radii>0.3 0.3 0.1</radii></ellipsoid></geometry></collision></link></model>
</world></sdf>)";
    for (const char* physics : {"mujoco", "jolt"}) {
        ServerOptions o;
        o.robotPath.clear();
        o.worldPath = path.string();
        o.ros = false;
        o.realTimeFactor = 0.0;
        o.physics = physics;
        Server server(o);
        const Status loaded = server.load();
        ASSERT_TRUE(loaded.ok()) << loaded.message;
        ASSERT_TRUE(server.stepUntil(SimTime{15'000'000'000}).ok());  // it rocks a while
        const std::string base = "body/" + server.world().bodies.names.at(0);
        EXPECT_NEAR(server.read(base + "/pose").at(2), 0.1, 0.003) << physics;
    }
}

// MuJoCo scales elevation data to 0..1 of the field's height. The field is 1 m up: the ball stops
// on it, not on the ground.
TEST(MjcfColliders, AHeightFieldHoldsABall) {
    const std::filesystem::path path = scratch("terrain.xml");
    std::ofstream(path) << R"(<mujoco><asset><hfield name="h" nrow="3" ncol="3" size="5 5 2 0.1"
elevation="0 0 0 0 0 0 0 0 1"/></asset><worldbody>
<geom type="hfield" hfield="h" pos="0 0 1"/><body name="ball" pos="-1 -1 2"><freejoint/>
<geom type="sphere" size="0.2" mass="1"/></body></worldbody></mujoco>)";
    auto model = loadModel(path.string(), {});
    ASSERT_TRUE(model) << model.error();
    ASSERT_EQ(model->world.heightfields.size(), 1U);
    const HeightField& field = model->world.heightfields[0];
    EXPECT_EQ(field.columns, 3U);
    EXPECT_NEAR(field.cellX, 5.0, 1e-9);
    // The XML's rows are flipped on the way into mjModel, whose data this is read from: its last
    // value is at +x, -y.
    EXPECT_NEAR(field.heightAt(5.0, -5.0).value_or(-1.0), 2.0, 1e-5) << "the high corner";
    EXPECT_NEAR(field.heightAt(-1.0, -1.0).value_or(-1.0), 0.0, 1e-5);
    ServerOptions o;
    o.robotPath.clear();
    o.worldPath = path.string();
    o.ros = false;
    o.realTimeFactor = 0.0;
    Server server(o);
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.stepUntil(SimTime{2'000'000'000}).ok());
    EXPECT_NEAR(server.read("body/ball/pose").at(2), 1.2, 0.01);
}

}  // namespace
}  // namespace lagrange::server
