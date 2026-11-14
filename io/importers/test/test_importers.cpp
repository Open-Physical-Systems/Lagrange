// The SDFormat and MJCF importers: the sample AMR in both formats loads to the bodies, joints,
// colliders and sensors of the URDF version; joint anchors, multi-joint bodies, ground planes
// and broken files behave as Importers.hpp says. (test_robot.cpp in lagrange_server drives all
// three versions and compares them.)
#include "lagrange/importers/Importers.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>

#include <gtest/gtest.h>

namespace lagrange::importers {
namespace {

std::filesystem::path writeText(const std::string& name, const std::string& text) {
    const std::filesystem::path dir =
        std::filesystem::path(::testing::TempDir()) / "lagrange_importers";
    std::filesystem::create_directories(dir);
    const std::filesystem::path path = dir / name;
    std::ofstream(path) << text;
    return path;
}

std::size_t body(const Model& model, const std::string& name) {
    const auto& names = model.world.bodies.names;
    const auto it = std::find(names.begin(), names.end(), name);
    EXPECT_NE(it, names.end()) << "no body " << name;
    return static_cast<std::size_t>(it - names.begin());
}

std::size_t joint(const Model& model, const std::string& name) {
    const auto& names = model.world.joints.names;
    const auto it = std::find(names.begin(), names.end(), name);
    EXPECT_NE(it, names.end()) << "no joint " << name;
    return static_cast<std::size_t>(it - names.begin());
}

std::size_t count(const World::Colliders& colliders, ShapeType shape) {
    return static_cast<std::size_t>(
        std::count(colliders.shape.begin(), colliders.shape.end(), shape));
}

/// What both sample files share with amr_diffdrive.urdf.
void expectSampleAmr(const Model& amr) {
    const World& world = amr.world;
    EXPECT_EQ(amr.name, "amr_diffdrive");
    EXPECT_EQ(world.bodies.names.size(), 10U);
    EXPECT_DOUBLE_EQ(world.bodies.mass[body(amr, "base_link")], 20.0);
    EXPECT_NEAR(world.bodies.comLocal[body(amr, "base_link")][2], 0.03, 1e-9);
    EXPECT_NEAR(world.bodies.inertiaLocal[body(amr, "base_link")][2], 0.86667, 1e-9);
    // The wheel sits 0.23 m to the left of the chassis, and the chassis 0.101 m up.
    EXPECT_NEAR(world.bodies.poseInParent[body(amr, "left_wheel")].position[1], 0.23, 1e-9);
    EXPECT_NEAR(amr.bodyRestPose[body(amr, "base_link")].position[2], 0.101, 1e-9);
    EXPECT_NEAR(amr.bodyRestPose[body(amr, "rear_caster_wheel")].position[0], -0.25, 1e-9);
    EXPECT_NEAR(amr.bodyRestPose[body(amr, "rear_caster_wheel")].position[2], 0.031, 1e-9);

    // Six movable joints and the floating base; the welded lidar links have none.
    ASSERT_EQ(world.joints.names.size(), 7U);
    EXPECT_EQ(world.joints.type[joint(amr, "base_link_floating")], JointType::Free);
    const std::size_t left = joint(amr, "left_wheel_joint");
    EXPECT_EQ(world.joints.type[left], JointType::Continuous);
    EXPECT_NEAR(world.joints.axis[left][1], 1.0, 1e-9);
    EXPECT_DOUBLE_EQ(world.joints.effortLimit[left], 10.0);
    EXPECT_EQ(world.joints.type[joint(amr, "front_caster_swivel")], JointType::Continuous);
    EXPECT_EQ(amr.actuatedJoints,
              (std::vector<std::string>{"left_wheel_joint", "right_wheel_joint"}));

    // The ground, the chassis box, two wheel cylinders and two caster spheres.
    const World::Colliders& colliders = world.colliders;
    ASSERT_EQ(colliders.body.size(), 6U);
    EXPECT_EQ(count(colliders, ShapeType::Plane), 1U);
    EXPECT_EQ(count(colliders, ShapeType::Box), 1U);
    EXPECT_EQ(count(colliders, ShapeType::Cylinder), 2U);
    EXPECT_EQ(count(colliders, ShapeType::Sphere), 2U);
    for (std::size_t c = 0; c < colliders.body.size(); ++c) {
        if (colliders.shape[c] == ShapeType::Box) {
            EXPECT_NEAR(colliders.size[c][0], 0.30, 1e-9);
        }
        if (colliders.shape[c] == ShapeType::Cylinder) {
            EXPECT_NEAR(colliders.size[c][1], 0.025, 1e-9);
            EXPECT_DOUBLE_EQ(colliders.friction[c], 1.0);
        }
    }

    // Scene: the robot first, its links nested as the body tree, the ground slab last.
    const Scene& scene = amr.scene;
    EXPECT_EQ(scene.prims.front().path, "/amr_diffdrive");
    EXPECT_EQ(scene.prims.front().body, body(amr, "base_link"));  // the articulation
    EXPECT_EQ(scene.prims[amr.bodyPrim[body(amr, "front_caster_wheel")]].path,
              "/amr_diffdrive/base_link/front_caster_fork/front_caster_wheel");
    EXPECT_EQ(scene.prims.back().name, "ground");
    EXPECT_EQ(amr.jointPrim.size(), world.joints.names.size());
    EXPECT_EQ(scene.prims[amr.jointPrim[left]].name, "left_wheel");
}

TEST(ImportSdf, SampleAmrMatchesTheUrdf) {
    auto amr = importSdf(LAGRANGE_TEST_AMR_SDF);
    ASSERT_TRUE(amr) << amr.error();
    expectSampleAmr(*amr);
    EXPECT_TRUE(amr->notes.empty()) << amr->notes.front();

    const World::Sensors& sensors = amr->world.sensors;
    ASSERT_EQ(sensors.names.size(), 3U);
    EXPECT_EQ(sensors.names[0], "imu");
    EXPECT_EQ(sensors.type[0], SensorType::Imu);
    EXPECT_DOUBLE_EQ(sensors.rateHz[0], 200.0);
    EXPECT_DOUBLE_EQ(sensors.settings[0].at("gyro_noise"), 0.002);
    EXPECT_EQ(sensors.names[1], "front_lidar");
    EXPECT_EQ(sensors.type[1], SensorType::Lidar);
    EXPECT_DOUBLE_EQ(sensors.settings[1].at("samples"), 720.0);
    EXPECT_DOUBLE_EQ(sensors.settings[1].at("max_range"), 20.0);
    EXPECT_DOUBLE_EQ(sensors.settings[1].at("range_noise"), 0.01);
    EXPECT_EQ(sensors.names[2], "top_lidar");
    EXPECT_DOUBLE_EQ(sensors.settings[2].at("vertical_samples"), 16.0);
    EXPECT_NEAR(sensors.settings[2].at("vertical_min_angle"), -0.261799, 1e-12);
    EXPECT_EQ(amr->scene.prims[amr->sensorPrim[1]].path,
              "/amr_diffdrive/base_link/lidar_link/front_lidar");
    EXPECT_EQ(amr->scene.materials.size(), 5U);  // body, accent, rubber, steel and the floor
}

/// An SDFormat rig with the lidar patterns and fisheye of RFC 0015 §4: Lagrange's custom elements
/// in `prefix` ("lagrange:" or "" for the plain form URDF uses).
std::string opticsSdf(const std::string& prefix) {
    const std::string p = prefix;
    return R"(<?xml version="1.0"?>
<sdf version="1.12" xmlns:lagrange="https://open-physical-systems.github.io/Lagrange">
  <model name="rig"><static>true</static><link name="head">
    <sensor name="rosette" type="lidar"><update_rate>10</update_rate><lidar>
      <)" + p +
           R"(pattern>rosette</)" + p + R"(pattern>
      <scan><horizontal><samples>500</samples></horizontal></scan>
      <range><min>0.1</min><max>60</max></range>
      <)" + p +
           R"(rosette><fov>0.8</fov><point_rate>5000</point_rate><speed_1>700</speed_1>
      <speed_2>-400</speed_2><prism_ratio>0.5</prism_ratio></)" +
           p + R"(rosette>
    </lidar></sensor>
    <sensor name="rosette_default" type="lidar"><update_rate>10</update_rate><lidar>
      <)" + p +
           R"(pattern>rosette</)" + p + R"(pattern>
      <range><min>0.1</min><max>60</max></range></lidar></sensor>
    <sensor name="flash" type="lidar"><update_rate>10</update_rate><lidar>
      <)" + p +
           R"(pattern>flash</)" + p + R"(pattern>
      <scan><horizontal><samples>16</samples><min_angle>-0.5</min_angle><max_angle>0.5</max_angle>
      </horizontal><vertical><samples>4</samples><min_angle>-0.2</min_angle>
      <max_angle>0.2</max_angle></vertical></scan>
      <range><min>0.1</min><max>30</max></range></lidar></sensor>
    <sensor name="vlp" type="gpu_lidar"><update_rate>10</update_rate><lidar><scan><horizontal>
      <samples>8</samples></horizontal><vertical><samples>2</samples>
      <)" + p +
           R"(angles>0.1 -0.3 0.0</)" + p + R"(angles></vertical></scan>
      <range><min>0.1</min><max>30</max></range></lidar></sensor>
    <sensor name="fisheye" type="camera"><update_rate>10</update_rate><camera>
      <horizontal_fov>3.4907</horizontal_fov><image><width>64</width><height>64</height></image>
      <lens><type>equidistant</type><cutoff_angle>1.8</cutoff_angle>
      <)" + p +
           R"(k1>0.01</)" + p + R"(k1><)" + p + R"(k4>0.0001</)" + p + R"(k4></lens>
    </camera></sensor>
    <sensor name="fisheye_default" type="camera"><update_rate>10</update_rate><camera>
      <horizontal_fov>3.4907</horizontal_fov><image><width>64</width><height>64</height></image>
      <lens><type>equidistant</type></lens></camera></sensor>
    <sensor name="odd" type="camera"><update_rate>10</update_rate><camera>
      <image><width>64</width><height>64</height></image>
      <lens><type>stereographic</type></lens></camera></sensor>
    <sensor name="plain" type="camera"><update_rate>10</update_rate><camera>
      <image><width>64</width><height>64</height></image></camera></sensor>
  </link></model>
</sdf>)";
}

TEST(ImportSdf, LidarPatternsAndFisheyeLenses) {
    for (const std::string prefix : {"lagrange:", ""}) {
        auto rig = importSdf(
            writeText("optics_" + std::to_string(prefix.size()) + ".sdf", opticsSdf(prefix))
                .string());
        ASSERT_TRUE(rig) << rig.error();
        const World::Sensors& s = rig->world.sensors;
        const auto find = [&](const std::string& name) {
            for (std::size_t i = 0; i < s.names.size(); ++i) {
                if (s.names[i] == name) {
                    return s.settings[i];
                }
            }
            ADD_FAILURE() << name;
            return std::map<std::string, double, std::less<>>{};
        };
        const auto rosette = find("rosette");
        EXPECT_EQ(rosette.at("pattern"), 2.0) << prefix;
        EXPECT_EQ(rosette.at("samples"), 500.0);
        EXPECT_EQ(rosette.at("rosette_fov"), 0.8);
        EXPECT_EQ(rosette.at("rosette_point_rate"), 5000.0);
        EXPECT_EQ(rosette.at("rosette_speed_1"), 700.0);
        EXPECT_EQ(rosette.at("rosette_speed_2"), -400.0);
        EXPECT_EQ(rosette.at("rosette_prism_ratio"), 0.5);
        EXPECT_FALSE(find("rosette_default").contains("samples"));  // the rosette's own

        const auto flash = find("flash");
        EXPECT_EQ(flash.at("pattern"), 1.0);
        EXPECT_EQ(flash.at("motion_distortion"), 0.0);

        const auto vlp = find("vlp");
        EXPECT_EQ(vlp.at("vertical_angle_0"), 0.1);
        EXPECT_EQ(vlp.at("vertical_angle_1"), -0.3);
        EXPECT_EQ(vlp.at("vertical_angle_2"), 0.0);
        EXPECT_FALSE(vlp.contains("vertical_angle_3"));
        EXPECT_FALSE(vlp.contains("pattern"));

        const auto fisheye = find("fisheye");
        EXPECT_EQ(fisheye.at("projection"), 1.0);
        EXPECT_EQ(fisheye.at("distortion_model"), 2.0);
        EXPECT_EQ(fisheye.at("fisheye_cutoff"), 1.8);
        EXPECT_EQ(fisheye.at("distortion_k1"), 0.01);
        EXPECT_EQ(fisheye.at("distortion_k4"), 0.0001);
        EXPECT_FALSE(fisheye.contains("distortion_k2"));
        EXPECT_EQ(fisheye.at("horizontal_fov"), 3.4907);
        const auto unset = find("fisheye_default");
        EXPECT_EQ(unset.at("projection"), 1.0);
        EXPECT_FALSE(unset.contains("fisheye_cutoff"));  // not SDFormat's 1.5707 default

        EXPECT_FALSE(find("odd").contains("projection"));
        EXPECT_FALSE(find("plain").contains("projection"));  // no <lens>: a pinhole
        bool noted = false;
        for (const std::string& note : rig->notes) {
            noted = noted || (note.find("'odd'") != std::string::npos &&
                              note.find("stereographic") != std::string::npos);
        }
        EXPECT_TRUE(noted) << prefix;
    }
}

TEST(ImportMjcf, SampleAmrMatchesTheUrdf) {
    auto amr = importMjcf(LAGRANGE_TEST_AMR_MJCF);
    ASSERT_TRUE(amr) << amr.error();
    expectSampleAmr(*amr);
    EXPECT_TRUE(amr->notes.empty()) << amr->notes.front();

    // No lidars in MJCF; the gyro and the accelerometer on one site are one IMU at 200 Hz.
    const World::Sensors& sensors = amr->world.sensors;
    ASSERT_EQ(sensors.names.size(), 1U);
    EXPECT_EQ(sensors.names[0], "imu");
    EXPECT_NEAR(sensors.rateHz[0], 200.0, 1e-9);
    EXPECT_DOUBLE_EQ(sensors.settings[0].at("gyro_noise"), 0.002);
    EXPECT_DOUBLE_EQ(sensors.settings[0].at("accel_noise"), 0.02);
    EXPECT_EQ(amr->scene.materials.size(), 5U);
}

TEST(ImportSdf, AnOffsetJointMovesTheBodyFrameToTheAnchor) {
    // A pendulum hinged 0.5 m above its link's origin, the link's mass at the origin.
    const auto path = writeText("pendulum.sdf", R"(<?xml version="1.0"?>
<sdf version="1.11">
  <model name="pendulum">
    <link name="support">
      <pose>0 0 2 0 0 0</pose>
    </link>
    <link name="bob">
      <pose>0 0 1 0 0 0</pose>
      <inertial><mass>2</mass>
        <inertia><ixx>0.1</ixx><iyy>0.2</iyy><izz>0.3</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name="ball"><geometry><sphere><radius>0.1</radius></sphere></geometry></collision>
    </link>
    <joint name="world_weld" type="fixed"><parent>world</parent><child>support</child></joint>
    <joint name="hinge" type="revolute">
      <parent>support</parent><child>bob</child>
      <pose>0 0 0.5 0 0 1.5707963267948966</pose>
      <axis><xyz>1 0 0</xyz><limit><lower>-1</lower><upper>1</upper></limit></axis>
    </joint>
  </model>
</sdf>)");
    auto model = importSdf(path.string());
    ASSERT_TRUE(model) << model.error();
    const World& world = model->world;
    const std::size_t bob = body(*model, "bob");
    // The body frame is the joint frame: 1.5 m up, turned 90° about z.
    EXPECT_NEAR(model->bodyRestPose[bob].position[2], 1.5, 1e-9);
    EXPECT_NEAR(world.bodies.poseInParent[bob].position[2], -0.5, 1e-9);
    // The axis stays x of the joint frame; the mass and the ball are 0.5 m below it.
    const std::size_t hinge = joint(*model, "hinge");
    EXPECT_EQ(world.joints.type[hinge], JointType::Revolute);
    EXPECT_NEAR(world.joints.axis[hinge][0], 1.0, 1e-9);
    EXPECT_DOUBLE_EQ(world.joints.upperLimit[hinge], 1.0);
    EXPECT_NEAR(world.bodies.comLocal[bob][2], -0.5, 1e-9);
    // Inertia in the turned frame: xx and yy swap.
    EXPECT_NEAR(world.bodies.inertiaLocal[bob][0], 0.2, 1e-9);
    EXPECT_NEAR(world.bodies.inertiaLocal[bob][1], 0.1, 1e-9);
    const auto ball = static_cast<std::size_t>(
        std::find(world.colliders.shape.begin(), world.colliders.shape.end(), ShapeType::Sphere) -
        world.colliders.shape.begin());
    EXPECT_NEAR(world.colliders.poseInBody[ball].position[2], -0.5, 1e-9);
    // The support is welded to the world: no joint of its own.
    EXPECT_EQ(world.bodies.parent[body(*model, "support")], kWorldBody);
    EXPECT_EQ(world.joints.names.size(), 1U);
}

TEST(ImportSdf, AWorldGivesOnePrimPerModelAndUsesItsGroundPlane) {
    const auto path = writeText("world.sdf", R"(<?xml version="1.0"?>
<sdf version="1.11">
  <world name="yard">
    <gravity>0 0 -9.8</gravity>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <surface><friction><ode><mu>0.8</mu></ode></friction></surface></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></visual>
      </link>
    </model>
    <model name="crate">
      <pose>1 2 0.25 0 0 0</pose>
      <link name="link">
        <inertial><mass>5</mass></inertial>
        <collision name="c"><geometry><box><size>0.5 0.5 0.5</size></box></geometry></collision>
      </link>
    </model>
  </world>
</sdf>)");
    auto model = importSdf(path.string());
    ASSERT_TRUE(model) << model.error();
    EXPECT_DOUBLE_EQ(model->world.gravityLocal[2], -9.8);
    // The file's plane is the ground, with its friction; no second ground is added.
    const World::Colliders& colliders = model->world.colliders;
    ASSERT_EQ(count(colliders, ShapeType::Plane), 1U);
    const auto plane = static_cast<std::size_t>(
        std::find(colliders.shape.begin(), colliders.shape.end(), ShapeType::Plane) -
        colliders.shape.begin());
    EXPECT_EQ(colliders.body[plane], kWorldBody);
    EXPECT_DOUBLE_EQ(colliders.friction[plane], 0.8);
    EXPECT_NE(model->scene.prims.back().name, "ground");
    // Both models have a link called "link": the second is renamed, with a note.
    EXPECT_EQ(model->world.bodies.names[1], "crate_link");
    EXPECT_FALSE(model->notes.empty());
    EXPECT_EQ(model->scene.prims[model->bodyPrim[1]].path, "/crate/crate_link");
    EXPECT_NEAR(model->bodyRestPose[1].position[1], 2.0, 1e-9);
    // The crate is not static, so it floats.
    EXPECT_EQ(model->world.joints.type.at(0), JointType::Free);
}

TEST(ImportMjcf, SeveralJointsOnABodyGetHelperFrames) {
    // A point mass on x and y slides and a hinge about z, offset from the body origin.
    const auto path = writeText("slider.xml", R"(<mujoco model="slider">
  <compiler angle="radian"/>
  <worldbody>
    <geom name="floor" type="plane" size="0 0 1"/>
    <body name="puck" pos="0 0 0.1">
      <joint name="x" type="slide" axis="1 0 0"/>
      <joint name="y" type="slide" axis="0 1 0"/>
      <joint name="spin" type="hinge" axis="0 0 1" pos="0.2 0 0" range="-1 1"/>
      <geom type="cylinder" size="0.1 0.05" mass="1"/>
    </body>
  </worldbody>
</mujoco>)");
    auto model = importMjcf(path.string());
    ASSERT_TRUE(model) << model.error();
    const World& world = model->world;
    // x_frame and y_frame carry the first two joints; the puck carries the hinge at its anchor.
    ASSERT_EQ(world.bodies.names.size(), 4U);  // two frames, the puck, worldbody (the floor)
    const std::size_t xFrame = body(*model, "x_frame");
    const std::size_t yFrame = body(*model, "y_frame");
    const std::size_t puck = body(*model, "puck");
    EXPECT_EQ(world.bodies.parent[xFrame], kWorldBody);
    EXPECT_EQ(world.bodies.parent[yFrame].index, xFrame);
    EXPECT_EQ(world.bodies.parent[puck].index, yFrame);
    EXPECT_NEAR(world.bodies.poseInParent[puck].position[0], 0.2, 1e-9);
    EXPECT_NEAR(world.bodies.comLocal[puck][0], -0.2, 1e-9);
    EXPECT_EQ(world.joints.type[joint(*model, "x")], JointType::Prismatic);
    EXPECT_EQ(world.joints.type[joint(*model, "spin")], JointType::Revolute);
    EXPECT_DOUBLE_EQ(world.joints.lowerLimit[joint(*model, "spin")], -1.0);
    EXPECT_FALSE(model->notes.empty());  // the helper frames are reported
    // The floor plane is the ground, on the world.
    ASSERT_EQ(count(world.colliders, ShapeType::Plane), 1U);
    for (std::size_t c = 0; c < world.colliders.body.size(); ++c) {
        if (world.colliders.shape[c] == ShapeType::Plane) {
            EXPECT_EQ(world.colliders.body[c], kWorldBody);
        }
    }
}

TEST(ImportMjcf, MeshGeomsAreDrawnWithTheirTriangles) {
    const auto path = writeText("tetra.xml", R"(<mujoco model="tetra">
  <asset>
    <mesh name="tetra" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/>
  </asset>
  <worldbody>
    <body name="piece" pos="0 0 1">
      <freejoint/>
      <geom type="mesh" mesh="tetra" contype="0" conaffinity="0" mass="1"/>
      <geom type="box" size="0.1 0.1 0.1" group="3"/>
    </body>
  </worldbody>
</mujoco>)");
    auto model = importMjcf(path.string());
    ASSERT_TRUE(model) << model.error();
    // The tetrahedron is drawn; the group 3 box only collides.
    std::size_t meshes = 0;
    for (const ScenePrim& prim : model->scene.prims) {
        if (prim.typeName == "Mesh" && prim.name.starts_with("visual")) {
            ++meshes;
            EXPECT_EQ(model->scene.meshes[prim.mesh].indices.size(), 12U);  // 4 faces
        }
    }
    EXPECT_EQ(meshes, 1U);
    EXPECT_EQ(count(model->world.colliders, ShapeType::Box), 1U);
    EXPECT_EQ(model->world.joints.names.front(), "piece_floating");
}

TEST(Importers, BrokenFilesAreRefusedWithTheReason) {
    auto missing = importSdf("/nonexistent/robot.sdf");
    ASSERT_FALSE(missing);
    EXPECT_NE(missing.error().find("cannot read"), std::string::npos);
    auto invalid = importSdf(writeText("bad.sdf", "<sdf version=\"1.11\"><model/></sdf>").string());
    ASSERT_FALSE(invalid);
    EXPECT_NE(invalid.error().find("not a valid SDFormat file"), std::string::npos);

    auto mjcf = importMjcf(writeText("bad.xml",
                                     "<mujoco><worldbody><body><joint type=\"nope\"/>"
                                     "</body></worldbody></mujoco>")
                               .string());
    ASSERT_FALSE(mjcf);
    EXPECT_NE(mjcf.error().find("not a valid MJCF file"), std::string::npos);

    const auto loop = writeText("loop.sdf", R"(<?xml version="1.0"?>
<sdf version="1.11">
  <model name="loop">
    <link name="a"/><link name="b"/>
    <joint name="there" type="revolute"><parent>a</parent><child>b</child><axis><xyz>0 0 1</xyz></axis></joint>
    <joint name="back" type="revolute"><parent>b</parent><child>a</child><axis><xyz>0 0 1</xyz></axis></joint>
  </model>
</sdf>)");
    auto looped = importSdf(loop.string());
    EXPECT_FALSE(looped);
}

TEST(Importers, FormatIsReadFromTheRootElement) {
    EXPECT_EQ(formatOf(LAGRANGE_TEST_AMR_SDF), Format::Sdf);
    EXPECT_EQ(formatOf(LAGRANGE_TEST_AMR_MJCF), Format::Mjcf);
    EXPECT_EQ(formatOf(LAGRANGE_TEST_AMR_URDF), Format::Urdf);
    EXPECT_EQ(formatOf("robot.usda"), Format::Usd);
    EXPECT_EQ(formatOf("/nonexistent/robot.xml"), Format::Unknown);
}

}  // namespace
}  // namespace lagrange::importers
