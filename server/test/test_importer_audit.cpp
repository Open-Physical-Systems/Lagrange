// What robot files say and the importers once dropped: mesh and ellipsoid colliders, a height
// field, Gazebo's friction.
#include "lagrange/server/Robot.hpp"
#include "lagrange/server/Server.hpp"
#include "lagrange/stage/Stage.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <tuple>
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
// A material's name is enough to look like it; the file's own values win.
TEST(SurfaceLooks, FromAWordOfTheNameUnlessTheFileSays) {
    EXPECT_GT(lookOf({"front_Steel_plate", {}}).metallic, 0.5F);
    EXPECT_GT(lookOf({"rubber", {}}).roughness, 0.8F);
    EXPECT_LT(lookOf({"glass_door", {}}).roughness, 0.1F);
    EXPECT_EQ(lookOf({"gusteel", {}}), SurfaceLook{}) << "whole words only";
    SceneMaterial told{"steel", {}};
    told.look = SurfaceLook{0.0F, 0.9F, {0.0F, 0.0F, 0.0F}};
    EXPECT_EQ(lookOf(told).metallic, 0.0F);
}

TEST(SurfaceLooks, AUrdfLinkSaysHowItLooks) {
    const std::filesystem::path path = std::filesystem::temp_directory_path() /
                                       ("lagrange_look_" + std::to_string(::getpid()) + ".urdf");
    std::ofstream(path) << R"(<?xml version="1.0"?><robot name="r"><link name="a">
<visual><geometry><box size="1 1 1"/></geometry></visual></link>
<gazebo reference="a"><visual_material preset="chrome" roughness="0.2" emissive="1 0 0"/></gazebo>
</robot>)";
    const auto model = loadModel(path.string());
    ASSERT_TRUE(model) << model.error();
    bool seen = false;
    for (const ScenePrim& prim : model->scene.prims) {
        if (prim.mesh != kNoIndex && prim.material != kNoIndex && prim.name != "ground") {
            const SurfaceLook look = lookOf(model->scene.materials[prim.material]);
            EXPECT_EQ(look.metallic, 1.0F);
            EXPECT_EQ(look.roughness, 0.2F);
            EXPECT_EQ(look.emissive[0], 1.0F);
            seen = true;
        }
    }
    EXPECT_TRUE(seen);
}

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

/// A joint about z (so gravity does nothing to it) with a known inertia, or along x.
std::string spinner(const std::string& type, const std::string& axis, const std::string& dynamics) {
    const std::filesystem::path path = scratch(type + "_spinner.urdf");
    std::ofstream(path) << R"(<?xml version="1.0"?><robot name="rig"><link name="world"/>
<link name="head"><inertial><mass value="2"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0"
iyz="0"/></inertial></link><joint name="move" type=")"
                        << type << R"("><parent link="world"/><child link="head"/>
<origin xyz="0 0 1"/><axis xyz=")"
                        << axis
                        << R"("/><limit lower="-1000" upper="1000" effort="100" velocity="1000"/>)"
                        << dynamics << "</joint></robot>";
    return path.string();
}

ServerOptions driven(const std::string& robot, const std::string& physics) {
    ServerOptions o = dropped(robot, physics);
    o.jointControl = true;
    o.demoDrive = false;
    if (physics == "jolt_abi") {
        o.plugins = {"lagrange_jolt"};
    }
    return o;
}

// Against the closed form under a constant effort: v(t) = F/d (1 - exp(-d t / I)), for a hinge
// (I = 1 kg m2) and a slide (I = 2 kg, its mass); dry friction takes f off F, and holds below it.
TEST(JointDynamics, DampingAndFrictionMatchTheClosedForm) {
    for (const char* physics : {"mujoco", "jolt", "jolt_abi"}) {
        for (const auto& [type, axis, inertia] :
             {std::tuple<const char*, const char*, double>{"continuous", "0 0 1", 1.0},
              std::tuple<const char*, const char*, double>{"prismatic", "1 0 0", 2.0}}) {
            const std::string where = std::string(physics) + " " + type;
            const double d = 0.5;
            const double tau = 2.0;
            const double t = 2.0;
            Server damped(driven(spinner(type, axis, R"(<dynamics damping="0.5"/>)"), physics));
            const Status loaded = damped.load();
            ASSERT_TRUE(loaded.ok()) << where << ": " << loaded.message;
            ASSERT_TRUE(damped.commandJoint("move", "effort", tau));
            ASSERT_TRUE(damped.stepUntil(SimTime{2'000'000'000}).ok());
            EXPECT_NEAR(damped.read("joint/move/velocity").at(0),
                        tau / d * (1.0 - std::exp(-d * t / inertia)), 0.03)
                << where;

            Server rubbing(driven(spinner(type, axis, R"(<dynamics friction="0.5"/>)"), physics));
            ASSERT_TRUE(rubbing.load().ok()) << where;
            ASSERT_TRUE(rubbing.commandJoint("move", "effort", tau));
            ASSERT_TRUE(rubbing.stepUntil(SimTime{1'000'000'000}).ok());
            EXPECT_NEAR(rubbing.read("joint/move/velocity").at(0), (tau - 0.5) / inertia, 0.05)
                << where;

            Server held(driven(spinner(type, axis, R"(<dynamics friction="0.5"/>)"), physics));
            ASSERT_TRUE(held.load().ok()) << where;
            ASSERT_TRUE(held.commandJoint("move", "effort", 0.3));
            ASSERT_TRUE(held.stepUntil(SimTime{1'000'000'000}).ok());
            EXPECT_NEAR(held.read("joint/move/velocity").at(0), 0.0, 0.02) << where;
        }
    }
}

std::string gears() {
    const std::filesystem::path path = scratch("gears.urdf");
    const auto link = [](const char* name) {
        return std::string("<link name=\"") + name +
               R"("><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0"
ixz="0" iyz="0"/></inertial></link>)";
    };
    std::ofstream(path) << R"(<?xml version="1.0"?><robot name="gears"><link name="world"/>)"
                        << link("a") << link("b") << R"(
<joint name="leader" type="continuous"><parent link="world"/><child link="a"/>
<origin xyz="0 0 1"/><axis xyz="0 0 1"/><limit effort="100" velocity="100"/></joint>
<joint name="follower" type="continuous"><parent link="world"/><child link="b"/>
<origin xyz="1 0 1"/><axis xyz="0 0 1"/><limit effort="100" velocity="100"/>
<mimic joint="leader" multiplier="2" offset="0.1"/></joint></robot>)";
    return path.string();
}

TEST(JointDynamics, AMimicJointCopiesItsLeaderAndTakesNoCommands) {
    for (const char* physics : {"mujoco", "jolt", "jolt_abi"}) {
        Server server(driven(gears(), physics));
        const Status loaded = server.load();
        ASSERT_TRUE(loaded.ok()) << physics << ": " << loaded.message;
        EXPECT_FALSE(server.commandJoint("follower", "velocity", 1.0)) << physics;
        ASSERT_TRUE(server.commandJoint("leader", "velocity", 1.0));
        ASSERT_TRUE(server.stepUntil(SimTime{3'000'000'000}).ok());
        const double leader = server.read("joint/leader/position").at(0);
        EXPECT_GT(leader, 2.0) << physics;
        EXPECT_NEAR(server.read("joint/follower/position").at(0), 2.0 * leader + 0.1, 0.02)
            << physics;
    }
}

TEST(JointDynamics, SurviveTheTripThroughUsd) {
    auto source =
        loadUrdf(spinner("prismatic", "1 0 0", R"(<dynamics damping="0.5" friction="0.2"/>)"), {});
    ASSERT_TRUE(source) << source.error();
    const std::string usda = scratch("spinner.usda").string();
    ASSERT_TRUE(stage::writeStage(*source, usda));
    auto back = loadModel(usda);
    ASSERT_TRUE(back) << back.error();
    ASSERT_EQ(back->world.joints.names.size(), 1U);
    EXPECT_DOUBLE_EQ(back->world.joints.damping[0], 0.5);
    EXPECT_DOUBLE_EQ(back->world.joints.friction[0], 0.2);

    auto geared = loadUrdf(gears(), {});
    ASSERT_TRUE(geared) << geared.error();
    const std::string usdGears = scratch("gears.usda").string();
    ASSERT_TRUE(stage::writeStage(*geared, usdGears));
    auto gearsBack = loadModel(usdGears);
    ASSERT_TRUE(gearsBack) << gearsBack.error();
    const World::Joints& joints = gearsBack->world.joints;
    const auto follower = static_cast<std::size_t>(
        std::find(joints.names.begin(), joints.names.end(), "follower") - joints.names.begin());
    ASSERT_LT(follower, joints.names.size());
    ASSERT_NE(joints.mimic[follower], kNoMimic);
    EXPECT_EQ(joints.names[joints.mimic[follower]], "leader");
    EXPECT_DOUBLE_EQ(joints.mimicMultiplier[follower], 2.0);
    EXPECT_DOUBLE_EQ(joints.mimicOffset[follower], 0.1);
}

TEST(JointDynamics, SdfAndMjcfCarryThem) {
    const std::filesystem::path sdfPath = scratch("axis.sdf");
    std::ofstream(sdfPath) << R"(<?xml version="1.0"?><sdf version="1.11"><model name="m">
<link name="a"><inertial><mass>1</mass></inertial></link>
<link name="b"><inertial><mass>1</mass></inertial></link>
<joint name="ja" type="revolute"><parent>world</parent><child>a</child><axis><xyz>0 0 1</xyz>
<dynamics><damping>0.7</damping><friction>0.3</friction></dynamics>
<limit><lower>-1</lower><upper>1</upper></limit></axis></joint>
<joint name="jb" type="revolute"><parent>world</parent><child>b</child><axis><xyz>0 0 1</xyz>
<limit><lower>-1</lower><upper>1</upper></limit>
<mimic joint="ja"><multiplier>3</multiplier><offset>0.2</offset><reference>0</reference></mimic>
</axis></joint></model></sdf>)";
    auto sdf = loadModel(sdfPath.string());
    ASSERT_TRUE(sdf) << sdf.error();
    ASSERT_EQ(sdf->world.joints.names.size(), 2U);
    EXPECT_DOUBLE_EQ(sdf->world.joints.damping[0], 0.7);
    EXPECT_DOUBLE_EQ(sdf->world.joints.friction[0], 0.3);
    EXPECT_EQ(sdf->world.joints.mimic[1], 0U);
    EXPECT_DOUBLE_EQ(sdf->world.joints.mimicMultiplier[1], 3.0);
    EXPECT_DOUBLE_EQ(sdf->world.joints.mimicOffset[1], 0.2);

    const std::filesystem::path xml = scratch("gear.xml");
    std::ofstream(xml) << R"(<mujoco><worldbody><body name="a"><joint name="ja" axis="0 0 1"
damping="0.7" frictionloss="0.3"/><geom type="sphere" size="0.1"/></body><body name="b" pos="1 0 0">
<joint name="jb" axis="0 0 1"/><geom type="sphere" size="0.1"/></body></worldbody><equality>
<joint joint1="jb" joint2="ja" polycoef="0.2 3 0 0 0"/></equality></mujoco>)";
    auto mjcf = loadModel(xml.string());
    ASSERT_TRUE(mjcf) << mjcf.error();
    ASSERT_EQ(mjcf->world.joints.names.size(), 2U);
    EXPECT_DOUBLE_EQ(mjcf->world.joints.damping[0], 0.7);
    EXPECT_DOUBLE_EQ(mjcf->world.joints.friction[0], 0.3);
    EXPECT_EQ(mjcf->world.joints.mimic[1], 0U);
    EXPECT_DOUBLE_EQ(mjcf->world.joints.mimicMultiplier[1], 3.0);
    EXPECT_DOUBLE_EQ(mjcf->world.joints.mimicOffset[1], 0.2);
}

// Gazebo's world joint: a pendulum hung from a static model swings about the joint; its own model
// is no longer floating.
TEST(SdfWorldJoints, JoinModelsIntoOneTree) {
    const std::filesystem::path path = scratch("hung.sdf");
    std::ofstream(path) << R"(<?xml version="1.0"?><sdf version="1.11"><world name="w">
<model name="post"><static>true</static><pose>0 0 2 0 0 0</pose><link name="top"><collision name="c">
<geometry><box><size>0.1 0.1 0.1</size></box></geometry></collision></link></model>
<model name="bob"><pose>0.5 0 2 0 0 0</pose><link name="ball"><inertial><mass>1</mass><inertia>
<ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia></inertial></link></model>
<joint name="hang" type="revolute"><pose relative_to="post::top">0 0 0 0 0 0</pose>
<parent>post::top</parent><child>bob::ball</child><axis><xyz>0 1 0</xyz>
<limit><lower>-10</lower><upper>10</upper></limit></axis></joint>
</world></sdf>)";
    auto model = loadModel(path.string());
    ASSERT_TRUE(model) << model.error();
    ASSERT_EQ(model->world.joints.names.size(), 1U);
    ServerOptions o;
    o.robotPath.clear();
    o.worldPath = path.string();
    o.ros = false;
    o.realTimeFactor = 0.0;
    Server server(o);
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    std::string ball;
    for (const std::string& name : server.world().bodies.names) {
        ball = name.find("ball") != std::string::npos ? name : ball;
    }
    ASSERT_FALSE(ball.empty());
    const std::string joint = "joint/" + server.world().joints.names.at(0) + "/position";
    double swing = 0.0;
    for (std::int64_t ms = 50; ms <= 400; ms += 50) {
        ASSERT_TRUE(server.stepUntil(SimTime{ms * 1'000'000}).ok());
        // A joint's body has its frame at the joint: it stays at the top of the post.
        const std::vector<double> pose = server.read("body/" + ball + "/pose");
        EXPECT_NEAR(pose.at(0), 0.0, 1e-3) << ms;
        EXPECT_NEAR(pose.at(2), 2.0, 1e-3) << ms;
        swing = std::max(swing, std::abs(server.read(joint).at(0)));
    }
    EXPECT_GT(swing, 0.5) << "it swings";
}

}  // namespace
}  // namespace lagrange::server
