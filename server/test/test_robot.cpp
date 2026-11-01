#include "lagrange/electrical/IdealTorqueSource.hpp"
#include "lagrange/rigid_body/JoltBackend.hpp"
#include "lagrange/rigid_body/MujocoBackend.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/sensors/Sensors.hpp"
#include "lagrange/server/DiffDrive.hpp"
#include "lagrange/server/Robot.hpp"
#include "lagrange/server/SwerveDrive.hpp"
#include "lagrange/stage/Stage.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <tinyxml2.h>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

std::size_t bodyIndex(const Robot& robot, const std::string& name) {
    const auto& names = robot.world.bodies.names;
    return static_cast<std::size_t>(std::find(names.begin(), names.end(), name) - names.begin());
}

Robot amr() {
    auto robot = loadUrdf(LAGRANGE_TEST_AMR_URDF);
    EXPECT_TRUE(robot) << (robot ? "" : robot.error());
    return robot ? std::move(robot.value()) : Robot{};
}

// An arm's URDF as CAD exporters write it: no "world" link, every axis z in its own frame, and
// ros2_control naming the joints. It must load as an arm, not as a swerve base or a loose body.
TEST(LagrangeBlock, AUrdfNamesItsSchematicAndMotors) {
    const auto dir = std::filesystem::temp_directory_path() / "lagrange_block_test";
    std::filesystem::create_directories(dir / "power");
    std::ofstream(dir / "power" / "p.cir")
        << "title\nV_pack in 0 DC 24\nR_pack in dc 0.1\nC_bus dc 0 1m\nR_lamp dc 0 12\n.end\n";
    const std::string links =
        "<link name='world'/><joint name='m' type='fixed'><parent link='world'/><child "
        "link='a'/></joint><link name='a'><inertial><mass value='1'/><inertia ixx='1' iyy='1' "
        "izz='1' ixy='0' ixz='0' iyz='0'/></inertial></link><joint name='spin' "
        "type='continuous'><parent link='a'/><child link='b'/><axis xyz='0 0 1'/></joint><link "
        "name='b'><inertial><mass value='1'/><inertia ixx='1' iyy='1' izz='1' ixy='0' ixz='0' "
        "iyz='0'/></inertial></link>";
    const auto write = [&](const std::string& name, const std::string& block) {
        const auto file = dir / name;
        std::ofstream(file) << "<robot name='r'>" << links << block << "</robot>";
        return file.string();
    };
    auto robot = loadUrdf(write("good.urdf", R"(<lagrange>
        <schematic file="power/p.cir"/>
        <battery source="V_pack" capacity="3600" cutoff="18" max_charge="5"/>
        <bus node="dc" undervoltage="16" overvoltage="30"/>
        <motor joint="spin" node="dc" r="0.2" l="0.001" kt="0.05" imax="10"/>
    </lagrange>)"));
    ASSERT_TRUE(robot) << robot.error();
    const WorldView view(robot->world);
    EXPECT_EQ(view.instancesOf("BatteryAPI_1").size(), 1U);
    EXPECT_EQ(view.instancesOf("ElectricalLoadAPI_1").size(), 1U);
    ASSERT_EQ(view.instancesOf("ElectricalMotorAPI_1").size(), 1U);
    EXPECT_EQ(view.signalInstanceName(view.instancesOf("ElectricalMotorAPI_1")[0]->prim),
              "spin_motor");
    // Without a block a URDF has no power system, and mistakes in one stop the load.
    auto plain = loadUrdf(write("plain.urdf", ""));
    ASSERT_TRUE(plain) << plain.error();
    EXPECT_TRUE(WorldView(plain->world).instancesOf("BatteryAPI_1").empty());
    for (const auto& [block, expected] : std::vector<std::pair<std::string, std::string>>{
             {"<lagrange/>", "needs <schematic"},
             {"<lagrange><schematic file='missing.cir'/></lagrange>", "cannot read"},
             {"<lagrange><schematic file='power/p.cir'/></lagrange>", "*lagrange battery V_pack"},
             {"<lagrange><schematic file='power/p.cir'/><relay/></lagrange>", "<relay>"},
             {"<lagrange><schematic file='power/p.cir'/><motor node='dc'/></lagrange>", "joint="},
         }) {
        auto refused = loadUrdf(write("bad.urdf", block));
        ASSERT_FALSE(refused) << block;
        EXPECT_NE(refused.error().find(expected), std::string::npos) << refused.error();
    }
    std::filesystem::remove_all(dir);
}

TEST(ArmUrdf, IsFixedToTheGroundAndNoSwerveBase) {
    std::string text =
        "<robot name='cad_arm'><link name='base_link'><inertial><mass value='5'/>"
        "<inertia ixx='1' iyy='1' izz='1' ixy='0' ixz='0' iyz='0'/></inertial></link>";
    std::string control = "<ros2_control name='s' type='system'>";
    std::string parent = "base_link";
    for (int i = 1; i <= 6; ++i) {
        const std::string link = "link_" + std::to_string(i);
        const std::string joint = "joint_" + std::to_string(i);
        text += "<link name='" + link +
                "'><inertial><mass value='1'/><inertia ixx='0.1' iyy='0.1' izz='0.1' ixy='0' "
                "ixz='0' iyz='0'/></inertial></link><joint name='" +
                joint + "' type='continuous'><parent link='" + parent + "'/><child link='" + link +
                "'/><origin xyz='0.1 0 0.1' rpy='1.57 0 0'/><axis xyz='0 0 1'/><limit effort='50' "
                "velocity='1'/></joint>";
        control += "<joint name='" + joint + "'><command_interface name='position'/></joint>";
        parent = link;
    }
    text += control + "</ros2_control></robot>";
    const std::string path = testing::TempDir() + "/cad_arm.urdf";
    std::FILE* file = std::fopen(path.c_str(), "w");
    ASSERT_NE(file, nullptr);
    std::fputs(text.c_str(), file);
    std::fclose(file);
    auto robot = loadUrdf(path);
    ASSERT_TRUE(robot) << robot.error();
    EXPECT_FALSE(findSwerveModules(*robot));
    for (const JointType type : robot->world.joints.type) {
        EXPECT_NE(type, JointType::Free);
    }
    // Something with wheels stays free.
    const Robot wheels = amr();
    EXPECT_TRUE(std::ranges::count(wheels.world.joints.type, JointType::Free) == 1);
}

TEST(Meshes, PrimitivesHaveTheMockServersLayout) {
    const TriMesh box = boxMesh(1.0, 2.0, 3.0);
    EXPECT_EQ(box.positions.size(), 36U * 3U);  // 6 faces x 2 triangles x 3 corners
    EXPECT_EQ(box.indices.size(), 36U);
    const TriMesh cylinder = cylinderMesh(0.1, 0.05);
    EXPECT_EQ(cylinder.positions.size() / 3, 32U * 2U + 2U * 33U);
    EXPECT_EQ(cylinder.indices.size(), 32U * 6U + 2U * 32U * 3U);
    const TriMesh sphere = sphereMesh(1.0);
    EXPECT_EQ(sphere.positions.size() / 3, 17U * 33U);
}

TEST(LoadUrdf, SampleAmrBecomesBodiesJointsAndColliders) {
    const Robot robot = amr();
    const World& world = robot.world;
    EXPECT_EQ(robot.name, "amr_diffdrive");
    // Chassis, 2 wheels, 2 forks, 2 balls, front lidar, roof mast and roof lidar.
    EXPECT_EQ(world.bodies.names.size(), 10U);
    EXPECT_EQ(world.bodies.names[0], "base_link");
    EXPECT_DOUBLE_EQ(world.bodies.mass[0], 20.0);
    EXPECT_DOUBLE_EQ(world.bodies.comLocal[0][2], 0.03);
    // Six joints from the file, and the free joint of the floating base.
    ASSERT_EQ(world.joints.names.size(), 7U);
    EXPECT_EQ(world.joints.type.back(), JointType::Free);
    EXPECT_EQ(robot.actuatedJoints,
              (std::vector<std::string>{"left_wheel_joint", "right_wheel_joint"}));
    // Ground, chassis box, two wheel cylinders, two caster spheres.
    ASSERT_EQ(world.colliders.body.size(), 6U);
    EXPECT_EQ(world.colliders.body[0], kWorldBody);
    EXPECT_EQ(
        std::count(world.colliders.shape.begin(), world.colliders.shape.end(), ShapeType::Cylinder),
        2);
    EXPECT_EQ(
        std::count(world.colliders.shape.begin(), world.colliders.shape.end(), ShapeType::Sphere),
        2);
    for (std::size_t c = 0; c < world.colliders.body.size(); ++c) {
        if (world.colliders.shape[c] == ShapeType::Box) {
            EXPECT_DOUBLE_EQ(world.colliders.size[c][0], 0.30);  // half of 0.60
        }
        if (world.colliders.shape[c] == ShapeType::Cylinder) {
            EXPECT_DOUBLE_EQ(world.colliders.size[c][1], 0.025);  // half of the 0.05 width
        }
    }
    EXPECT_TRUE(robot.notes.empty()) << robot.notes.front();
}

TEST(LoadUrdf, GazeboSensorBlocksBecomeSensors) {
    const Robot robot = amr();
    const World::Sensors& sensors = robot.world.sensors;
    ASSERT_EQ(sensors.names.size(), 3U);
    EXPECT_EQ(sensors.names[0], "front_lidar");
    EXPECT_EQ(sensors.type[0], SensorType::Lidar);
    EXPECT_EQ(robot.world.bodies.names[sensors.body[0].index], "lidar_link");
    EXPECT_DOUBLE_EQ(sensors.rateHz[0], 10.0);
    EXPECT_DOUBLE_EQ(sensors.settings[0].at("samples"), 720.0);
    EXPECT_DOUBLE_EQ(sensors.settings[0].at("max_range"), 20.0);
    EXPECT_DOUBLE_EQ(sensors.settings[0].at("range_noise"), 0.01);
    EXPECT_DOUBLE_EQ(sensors.settings[0].at("vertical_samples"), 1.0);  // a 2D scanner
    // The roof lidar: 16 rings from -15 to +15 degrees.
    EXPECT_EQ(sensors.names[1], "top_lidar");
    EXPECT_EQ(sensors.type[1], SensorType::Lidar);
    EXPECT_DOUBLE_EQ(sensors.settings[1].at("samples"), 900.0);
    EXPECT_DOUBLE_EQ(sensors.settings[1].at("vertical_samples"), 16.0);
    EXPECT_NEAR(sensors.settings[1].at("vertical_min_angle"), -0.261799, 1e-12);
    EXPECT_NEAR(sensors.settings[1].at("vertical_max_angle"), 0.261799, 1e-12);
    EXPECT_EQ(sensors.type[2], SensorType::Imu);
    EXPECT_DOUBLE_EQ(sensors.rateHz[2], 200.0);
    EXPECT_DOUBLE_EQ(sensors.settings[2].at("gyro_noise"), 0.002);
    EXPECT_EQ(robot.scene.prims[robot.sensorPrim[0]].path,
              "/amr_diffdrive/base_link/lidar_link/front_lidar");
    EXPECT_EQ(robot.scene.prims[robot.sensorPrim[0]].typeName, "Lidar");
}

TEST(LoadUrdf, CameraSensorsLoadAndSurviveAUsdRoundTrip) {
    Robot robot;
    ASSERT_TRUE(addUrdf(robot, LAGRANGE_TEST_ARM6_URDF));
    const World::Sensors& sensors = robot.world.sensors;
    ASSERT_EQ(sensors.names.size(), 1U);
    EXPECT_EQ(sensors.names[0], "wrist_camera");
    EXPECT_EQ(sensors.type[0], SensorType::Camera);
    EXPECT_EQ(robot.world.bodies.names[sensors.body[0].index], "palm_link");
    EXPECT_DOUBLE_EQ(sensors.rateHz[0], 15.0);
    EXPECT_DOUBLE_EQ(sensors.settings[0].at("width"), 640.0);
    EXPECT_DOUBLE_EQ(sensors.settings[0].at("height"), 480.0);
    EXPECT_DOUBLE_EQ(sensors.settings[0].at("horizontal_fov"), 1.2);
    EXPECT_DOUBLE_EQ(sensors.settings[0].at("near_clip"), 0.02);
    EXPECT_EQ(robot.scene.prims[robot.sensorPrim[0]].apiSchemas.back(), "CameraSensorAPI_1");

    // Written as a UsdGeomCamera (looking along its -z) and read back as the same sensor.
    const std::string usda = ::testing::TempDir() + "/arm6_camera.usda";
    ASSERT_TRUE(stage::writeStage(robot, usda));
    const auto back = stage::loadStage(usda);
    ASSERT_TRUE(back) << back.error();
    const World::Sensors& read = back->world.sensors;
    ASSERT_EQ(read.names.size(), 1U);
    EXPECT_EQ(read.type[0], SensorType::Camera);
    EXPECT_EQ(back->scene.prims[back->sensorPrim[0]].typeName, "Camera");
    EXPECT_NEAR(read.settings[0].at("horizontal_fov"), 1.2, 1e-6);
    EXPECT_DOUBLE_EQ(read.settings[0].at("width"), 640.0);
    EXPECT_NEAR(read.settings[0].at("far_clip"), 5.0, 1e-6);
    for (std::size_t i = 0; i < 3; ++i) {
        EXPECT_NEAR(read.poseInBody[0].position[i], sensors.poseInBody[0].position[i], 1e-9);
    }
    // The same rotation (q or -q).
    double dot = 0.0;
    for (std::size_t i = 0; i < 4; ++i) {
        dot += read.poseInBody[0].orientation[i] * sensors.poseInBody[0].orientation[i];
    }
    EXPECT_NEAR(std::abs(dot), 1.0, 1e-9);
}

TEST(LoadUrdf, AWorldFileAddsFixedScenery) {
    Robot robot = amr();
    const std::size_t bodies = robot.world.bodies.names.size();
    ASSERT_TRUE(addUrdf(robot, LAGRANGE_TEST_WAREHOUSE_URDF));
    EXPECT_EQ(robot.world.bodies.names.size(), bodies + 7U);
    EXPECT_EQ(robot.world.colliders.body.size(), 6U + 7U);
    // Fixed to the world: no joints added, nothing lifted.
    EXPECT_EQ(robot.world.joints.names.size(), 7U);
    EXPECT_DOUBLE_EQ(robot.bodyRestPose[bodyIndex(robot, "wall_south")].position[2], 1.0);
    EXPECT_EQ(robot.scene.prims.front().path, "/amr_diffdrive");  // still the robot first
}

TEST(LoadUrdf, FreeRobotIsLiftedOntoTheGround) {
    const Robot robot = amr();
    // The wheels (radius 0.1) and the casters both reach 0.1 m below the axle.
    EXPECT_NEAR(robot.world.bodies.poseInParent[0].position[2], 0.101, 1e-9);
    EXPECT_NEAR(robot.bodyRestPose[bodyIndex(robot, "left_wheel")].position[1], 0.23, 1e-12);
}

TEST(LoadUrdf, SceneHasGroundRobotLinksAndVisuals) {
    const Robot robot = amr();
    const Scene& scene = robot.scene;
    ASSERT_GE(scene.prims.size(), 2U + 7U + 8U);
    EXPECT_EQ(scene.prims[0].path, "/amr_diffdrive");
    EXPECT_EQ(scene.prims.back().name, "ground");
    EXPECT_EQ(scene.prims[robot.bodyPrim[0]].path, "/amr_diffdrive/base_link");
    const std::uint32_t wheel = robot.bodyPrim[bodyIndex(robot, "left_wheel")];
    EXPECT_EQ(scene.prims[wheel].path, "/amr_diffdrive/base_link/left_wheel");
    EXPECT_EQ(scene.prims[wheel].apiSchemas,
              (std::vector<std::string>{"PhysicsRigidBodyAPI", "PhysicsCollisionAPI",
                                        "joint:continuous"}));
    // Wheels share one mesh; materials are deduplicated by colour.
    EXPECT_EQ(scene.materials.size(), 5U);  // ground + body, accent, rubber, steel
}

TEST(LoadUrdf, FixedBaseRobotAndMissingFiles) {
    auto missing = loadUrdf("/nonexistent/robot.urdf");
    ASSERT_FALSE(missing);
    EXPECT_NE(missing.error().find("not a valid URDF"), std::string::npos);

    auto arm = loadUrdf(LAGRANGE_TEST_ARM_URDF);
    ASSERT_TRUE(arm) << arm.error();
    // The arm has visuals only: no collisions, so it would fall.
    EXPECT_FALSE(arm->notes.empty());
}

TEST(LoadUrdf, MeshVisualsAreImportedWithTheirScaleAndMaterial) {
    auto arm = loadUrdf(LAGRANGE_TEST_ARM_URDF);
    ASSERT_TRUE(arm) << arm.error();
    for (const std::string& note : arm->notes) {
        EXPECT_EQ(note.find("magenta"), std::string::npos) << note;
    }
    const auto tool =
        std::find_if(arm->scene.prims.begin(), arm->scene.prims.end(),
                     [](const ScenePrim& p) { return p.path.ends_with("tool/visual"); });
    ASSERT_NE(tool, arm->scene.prims.end());
    // tool.stl: a tetrahedron in millimetres, scaled by the URDF to metres.
    ASSERT_NE(tool->mesh, kNoIndex);
    EXPECT_EQ(arm->scene.meshes[tool->mesh].indices.size(), 4U * 3U);
    EXPECT_EQ(tool->scale, (std::array<float, 3>{0.001F, 0.001F, 0.001F}));
    // The URDF's material wins over the file's.
    EXPECT_EQ(arm->scene.materials[tool->material].name, "steel");
}

TEST(AmrSimulation, TheDriverMovesTheRobotAndItStaysUpright) {
    const Robot robot = amr();
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<rigid_body::MujocoBackend>());
    scheduler.add(std::make_unique<DriveCommand>(DriveCommand::Settings{}, nullptr));
    DiffDriveController::Settings settings;
    settings.leftJoint = robot.actuatedJoints[0];
    settings.rightJoint = robot.actuatedJoints[1];
    scheduler.add(std::make_unique<DiffDriveController>(settings));
    scheduler.add(std::make_unique<electrical::IdealTorqueSource>());  // torque_command -> torque
    auto wired = scheduler.wire(WorldView(robot.world));
    ASSERT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    ASSERT_TRUE(scheduler.stepUntil(SimTime{6'000'000'000}).ok());

    const SignalBus& bus = scheduler.bus();
    const std::span<const double> pose = bus.readN(bus.find("body/base_link/pose"));
    const double travelled = std::hypot(pose[0], pose[1]);
    // 3 m along a 1.2 m circle after the 2 s ramp: a chord of 2 R sin(1.25 rad) = 2.28 m.
    EXPECT_NEAR(travelled, 2.28, 0.2);
    // It turns as commanded: 0.5 rad/s about z.
    EXPECT_NEAR(bus.readN(bus.find("body/base_link/twist"))[5], 0.5, 0.025);
    EXPECT_NEAR(pose[2], 0.1, 0.01) << "the chassis stays at axle height";
    // Upright: the body's z axis stays within 5° of vertical (quaternion w, x, y, z).
    const double tiltCos = 1.0 - 2.0 * (pose[4] * pose[4] + pose[5] * pose[5]);
    EXPECT_GT(tiltCos, std::cos(5.0 * 3.14159265358979 / 180.0));
    // The wheel follows its commanded speed: v/r at the end of the ramp, within 10 %.
    const double command = bus.read(bus.find("base/command/linear_velocity"));
    EXPECT_NEAR(command, 0.6, 1e-12);
    const double left = bus.read(bus.find("joint/left_wheel_joint/velocity"));
    const double right = bus.read(bus.find("joint/right_wheel_joint/velocity"));
    EXPECT_NEAR((left + right) / 2.0 * 0.1, 0.6, 0.06);
}

// The AMR_FMS swerve base: its four modules are found, and the built-in swerve controller moves
// it sideways without turning, then turns it on the spot without moving.
TEST(SwerveSimulation, TheBaseMovesSidewaysAndTurnsOnTheSpot) {
    auto loaded = loadUrdf(LAGRANGE_TEST_AMR_FMS_URDF);
    ASSERT_TRUE(loaded) << loaded.error();
    const Robot& robot = *loaded;
    const auto swerve = findSwerveModules(robot);
    ASSERT_TRUE(swerve);
    ASSERT_EQ(swerve->modules.size(), 4U);
    EXPECT_NEAR(swerve->wheelRadiusM, 0.0625, 1e-9);
    EXPECT_FALSE(findSwerveModules(amr())) << "a differential drive is not a swerve base";

    const auto run = [&](VelocityCommand command, double seconds) {
        // An external command makes the run non-deterministic by declaration only.
        Scheduler scheduler(SchedulerOptions{.deterministic = false});
        scheduler.add(std::make_unique<rigid_body::MujocoBackend>());
        DriveCommand::Settings settings;
        settings.lateral = true;
        scheduler.add(std::make_unique<DriveCommand>(
            settings, [command]() -> std::optional<VelocityCommand> { return command; }));
        scheduler.add(std::make_unique<SwerveController>(*swerve));
        scheduler.add(std::make_unique<electrical::IdealTorqueSource>());
        auto wired = scheduler.wire(WorldView(robot.world));
        EXPECT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
        EXPECT_TRUE(scheduler.stepUntil(SimTime{static_cast<std::int64_t>(seconds * 1e9)}).ok());
        const SignalBus& bus = scheduler.bus();
        const std::span<const double> pose = bus.readN(bus.find("body/base_link/pose"));
        const double yaw = 2.0 * std::atan2(pose[6], pose[3]);  // upright: w, z only
        return std::array<double, 3>{pose[0], pose[1], yaw};
    };
    // 0.3 m/s to the left for 4 s: about 1.2 m, less the time the modules take to steer.
    const auto sideways = run({0.0, 0.0, 0.3}, 4.0);
    EXPECT_NEAR(sideways[1], 1.15, 0.1);
    EXPECT_NEAR(sideways[0], 0.0, 0.05);
    // A little yaw while the modules steer, against the damping its URDF gives their joints.
    EXPECT_NEAR(sideways[2], 0.0, 0.06);
    // 0.5 rad/s for 3 s about the modules' centre: 1.5 rad less the first few tenths of a second
    // in which each module steers 45° to its tangent, and the centre stays.
    const auto turned = run({0.0, 0.5, 0.0}, 3.0);
    EXPECT_NEAR(turned[2], 1.35, 0.08);
    EXPECT_LT(std::hypot(turned[0], turned[1]), 0.1);
}

// --no-demo (the Nav2 launch): the robot waits at its start pose for the first /cmd_vel.
TEST(AmrSimulation, WithoutTheDemoTheRobotStaysPut) {
    const Robot robot = amr();
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<rigid_body::MujocoBackend>());
    DriveCommand::Settings still;
    still.demo = false;
    scheduler.add(std::make_unique<DriveCommand>(still, nullptr));
    DiffDriveController::Settings settings;
    settings.leftJoint = robot.actuatedJoints[0];
    settings.rightJoint = robot.actuatedJoints[1];
    scheduler.add(std::make_unique<DiffDriveController>(settings));
    scheduler.add(std::make_unique<electrical::IdealTorqueSource>());  // torque_command -> torque
    ASSERT_TRUE(scheduler.wire(WorldView(robot.world)));
    ASSERT_TRUE(scheduler.stepUntil(SimTime{3'000'000'000}).ok());
    const SignalBus& bus = scheduler.bus();
    const std::span<const double> pose = bus.readN(bus.find("body/base_link/pose"));
    EXPECT_LT(std::hypot(pose[0], pose[1]), 0.01);
    EXPECT_EQ(bus.read(bus.find("base/command/linear_velocity")), 0.0);
}

/// The sample warehouse authored `offset` away: every fixed link (a joint from "world") moved,
/// written to `file` the way a hand-made file would hold it, so the large coordinates go through
/// the parser and its rounding.
std::string movedWarehouse(const std::array<double, 3>& offset, const std::string& name) {
    tinyxml2::XMLDocument document;
    EXPECT_EQ(document.LoadFile(LAGRANGE_TEST_WAREHOUSE_URDF), tinyxml2::XML_SUCCESS);
    for (tinyxml2::XMLElement* joint = document.RootElement()->FirstChildElement("joint");
         joint != nullptr; joint = joint->NextSiblingElement("joint")) {
        const tinyxml2::XMLElement* parent = joint->FirstChildElement("parent");
        tinyxml2::XMLElement* origin = joint->FirstChildElement("origin");
        if (parent == nullptr || std::string(parent->Attribute("link")) != "world" ||
            origin == nullptr) {
            continue;
        }
        std::array<double, 3> xyz{};
        EXPECT_EQ(std::sscanf(origin->Attribute("xyz"), "%lf %lf %lf", &xyz[0], &xyz[1], &xyz[2]),
                  3);
        char text[96];
        std::snprintf(text, sizeof text, "%.17g %.17g %.17g", xyz[0] + offset[0],
                      xyz[1] + offset[1], xyz[2] + offset[2]);
        origin->SetAttribute("xyz", text);
    }
    const std::string path = (std::filesystem::path(::testing::TempDir()) / name).string();
    EXPECT_EQ(document.SaveFile(path.c_str()), tinyxml2::XML_SUCCESS);
    return path;
}

Robot amrInWarehouse(const std::array<double, 3>& offset, const std::string& name) {
    LoadOptions options;
    options.spawn.position = offset;
    auto robot = loadUrdf(LAGRANGE_TEST_AMR_URDF, options);
    EXPECT_TRUE(robot) << (robot ? "" : robot.error());
    if (!robot) {
        return Robot{};
    }
    auto added = addUrdf(*robot, movedWarehouse(offset, name));
    EXPECT_TRUE(added) << (added ? "" : added.error());
    recentre(*robot);
    return std::move(*robot);
}

struct DemoState {
    std::vector<double> pose;    // base_link, region-local
    std::vector<double> ranges;  // the 2D lidar's last scan
    std::vector<double> wheels;  // joint positions, rad
};

/// The demo drive with the sensors for `seconds`.
DemoState runDemo(const Robot& robot, std::int64_t seconds) {
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<rigid_body::MujocoBackend>());
    for (auto& module : sensors::createSensorModules(robot.world)) {
        scheduler.add(std::move(module));
    }
    scheduler.add(std::make_unique<DriveCommand>(DriveCommand::Settings{}, nullptr));
    DiffDriveController::Settings settings;
    settings.leftJoint = robot.actuatedJoints[0];
    settings.rightJoint = robot.actuatedJoints[1];
    scheduler.add(std::make_unique<DiffDriveController>(settings));
    scheduler.add(std::make_unique<electrical::IdealTorqueSource>());  // torque_command -> torque
    auto wired = scheduler.wire(WorldView(robot.world));
    EXPECT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    EXPECT_TRUE(scheduler.stepUntil(SimTime{seconds * 1'000'000'000}).ok());
    const SignalBus& bus = scheduler.bus();
    const auto read = [&bus](const std::string& name) {
        const SignalId id = bus.find(name);
        EXPECT_TRUE(id.valid()) << name;
        const std::span<const double> v = id.valid() ? bus.readN(id) : std::span<const double>{};
        return std::vector<double>(v.begin(), v.end());
    };
    return {
        read("body/base_link/pose"),
        read("sensor/front_lidar/ranges"),
        {read("joint/left_wheel_joint/position")[0], read("joint/right_wheel_joint/position")[0]}};
}

// v0.2 acceptance criterion 8 (roadmap 0.2-H4): the warehouse and the AMR authored 500 km from
// the origin simulate as they do at the origin. The floating origin takes the robot's whole-metre
// position out at load, so physics and sensors see the same small coordinates; what is left is
// the rounding of the authored coordinates themselves (an ulp of 5e5 m is 5.8e-11 m). Measured:
// poses bit-identical, ranges within 1.2e-10 m; without the floating origin the same run is off
// by 5e-7 m in pose and 8e-5 m in range after 10 s, which these 1e-9 m bounds catch.
TEST(FloatingOrigin, AWorld500KmOutSimulatesLikeOneAtTheOrigin) {
    const std::array<double, 3> far{400'000.0, 300'000.0, 0.0};  // 500 km from the origin
    const Robot here = amrInWarehouse({0.0, 0.0, 0.0}, "warehouse_0km.urdf");
    const Robot there = amrInWarehouse(far, "warehouse_500km.urdf");
    EXPECT_EQ(here.world.region.rebaseCount, 0U);
    EXPECT_EQ(there.world.region.rebaseCount, 1U);
    EXPECT_EQ(there.world.region.worldEnu, far);
    // Same region-local world: rest poses, scenery and colliders.
    ASSERT_EQ(here.bodyRestPose.size(), there.bodyRestPose.size());
    for (std::size_t b = 0; b < here.bodyRestPose.size(); ++b) {
        for (std::size_t i = 0; i < 3; ++i) {
            EXPECT_NEAR(there.bodyRestPose[b].position[i], here.bodyRestPose[b].position[i], 1e-9)
                << here.world.bodies.names[b];
        }
    }
    ASSERT_EQ(here.world.colliders.body.size(), there.world.colliders.body.size());
    for (std::size_t c = 0; c < here.world.colliders.body.size(); ++c) {
        for (std::size_t i = 0; i < 3; ++i) {
            EXPECT_NEAR(there.world.colliders.poseInBody[c].position[i],
                        here.world.colliders.poseInBody[c].position[i], 1e-9);
        }
    }

    const DemoState a = runDemo(here, 10);
    const DemoState b = runDemo(there, 10);
    ASSERT_EQ(a.pose.size(), 7U);
    ASSERT_EQ(b.pose.size(), 7U);
    EXPECT_GT(std::hypot(a.pose[0], a.pose[1]), 0.5) << "the robot drove";
    for (std::size_t i = 0; i < 7; ++i) {
        EXPECT_NEAR(b.pose[i], a.pose[i], 1e-9) << "pose component " << i;
    }
    for (std::size_t i = 0; i < 2; ++i) {
        EXPECT_NEAR(b.wheels[i], a.wheels[i], 1e-9);
    }
    ASSERT_EQ(a.ranges.size(), b.ranges.size());
    std::size_t returns = 0;
    for (std::size_t i = 0; i < a.ranges.size(); ++i) {
        if (std::isinf(a.ranges[i])) {
            EXPECT_TRUE(std::isinf(b.ranges[i])) << "beam " << i;
            continue;
        }
        ++returns;
        EXPECT_NEAR(b.ranges[i], a.ranges[i], 1e-9) << "beam " << i;
    }
    EXPECT_GT(returns, a.ranges.size() / 2) << "the lidar sees the warehouse";
}

/// The demo's modules (physics on `backend`, sensors, the figure-eight drive) wired on `robot`.
std::unique_ptr<Scheduler> demoScheduler(const Robot& robot, const std::string& backend) {
    auto scheduler = std::make_unique<Scheduler>(SchedulerOptions{.deterministic = true});
    if (backend == "jolt") {
        scheduler->add(std::make_unique<rigid_body::JoltBackend>());
    } else {
        scheduler->add(std::make_unique<rigid_body::MujocoBackend>());
    }
    for (auto& module : sensors::createSensorModules(robot.world)) {
        scheduler->add(std::move(module));
    }
    scheduler->add(std::make_unique<DriveCommand>(DriveCommand::Settings{}, nullptr));
    DiffDriveController::Settings settings;
    settings.leftJoint = robot.actuatedJoints[0];
    settings.rightJoint = robot.actuatedJoints[1];
    scheduler->add(std::make_unique<DiffDriveController>(settings));
    scheduler->add(std::make_unique<electrical::IdealTorqueSource>());
    auto wired = scheduler->wire(WorldView(robot.world));
    EXPECT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    return scheduler;
}

std::vector<double> busValues(const SignalBus& bus, const std::string& name) {
    const SignalId id = bus.find(name);
    EXPECT_TRUE(id.valid()) << name;
    const std::span<const double> v = id.valid() ? bus.readN(id) : std::span<const double>{};
    return {v.begin(), v.end()};
}

// huge-worlds.md §6 (v0.2 row, done in v0.7 with the rebase during a run): moving the region
// origin halfway through a run does not change what happens. At 5 s the world is shifted by a
// whole-metre offset, the run rebuilt on it and its state carried over and shifted
// (Scheduler::shiftOrigin), as the server does when the robot drives far; at 10 s the pose,
// moved back by the offset, and the lidar ranges match a run that was never rebased.
// MuJoCo computes in double: within 1e-9 m (rounding at a few metres), ranges too. Jolt
// computes in float: the pose within 1e-4 m, and the ranges within 5 mm (measured 2 mm: a heading
// off by 1e-4 rad moves a grazing beam's hit by millimetres along the wall).
// `strayBeams`: beams allowed past the range tolerance. On Jolt's single-precision poses, a pose
// within `tolerance` still moves a grazing beam by millimetres and one at an object's edge by
// metres; the pose check above is what says the trajectory is kept.
void expectRebaseKeepsTheRun(const std::string& backend, double tolerance, double rangeTolerance,
                             std::size_t strayBeams = 0) {
    const std::array<double, 3> offset{37.0, -12.0, 0.0};
    const Robot start = amrInWarehouse({0.0, 0.0, 0.0}, "warehouse_rebase_" + backend + ".urdf");
    const auto plain = demoScheduler(start, backend);
    ASSERT_TRUE(plain->stepUntil(SimTime{10'000'000'000}).ok());

    Robot robot = start;
    auto first = demoScheduler(robot, backend);
    ASSERT_TRUE(first->stepUntil(SimTime{5'000'000'000}).ok());
    StateWriter saved;
    ASSERT_TRUE(first->saveState(saved).ok());
    shiftRegion(robot, offset);
    EXPECT_EQ(robot.world.region.worldEnu, offset);
    EXPECT_EQ(robot.world.region.rebaseCount, 1U);
    auto second = demoScheduler(robot, backend);
    StateReader reader(saved.bytes());
    ASSERT_TRUE(second->restoreState(reader).ok());
    second->shiftOrigin(offset);
    const std::vector<double> shifted = busValues(second->bus(), "body/base_link/pose");
    const std::vector<double> before = busValues(first->bus(), "body/base_link/pose");
    for (std::size_t i = 0; i < 3; ++i) {
        EXPECT_NEAR(shifted[i] + offset.at(i), before[i], 1e-12) << "the bus pose moved with it";
    }
    ASSERT_TRUE(second->stepUntil(SimTime{10'000'000'000}).ok());

    const std::vector<double> a = busValues(plain->bus(), "body/base_link/pose");
    const std::vector<double> b = busValues(second->bus(), "body/base_link/pose");
    EXPECT_GT(std::hypot(a[0], a[1]), 0.5) << "the robot drove";
    for (std::size_t i = 0; i < 7; ++i) {
        EXPECT_NEAR(b[i] + (i < 3 ? offset.at(i) : 0.0), a[i], tolerance) << "pose " << i;
    }
    const std::vector<double> ra = busValues(plain->bus(), "sensor/front_lidar/ranges");
    const std::vector<double> rb = busValues(second->bus(), "sensor/front_lidar/ranges");
    ASSERT_EQ(ra.size(), rb.size());
    std::size_t stray = 0;
    for (std::size_t i = 0; i < ra.size(); ++i) {
        if (std::isinf(ra[i]) != std::isinf(rb[i]) ||
            (!std::isinf(ra[i]) && std::abs(rb[i] - ra[i]) > rangeTolerance)) {
            ++stray;
        }
    }
    EXPECT_LE(stray, strayBeams) << "of " << ra.size() << " beams";
}

TEST(FloatingOrigin, ARebaseDuringTheRunKeepsTheTrajectoryOnMujoco) {
    expectRebaseKeepsTheRun("mujoco", 1e-9, 1e-9);
}

TEST(FloatingOrigin, ARebaseDuringTheRunKeepsTheTrajectoryOnJolt) {
    expectRebaseKeepsTheRun("jolt", 1e-4, 5e-3, 10);
}

TEST(FloatingOrigin, ARobotNearTheOriginStaysWhereItWas) {
    Robot robot = amr();
    const std::vector<Pose> before = robot.bodyRestPose;
    EXPECT_EQ(recentre(robot), (std::array<double, 3>{0.0, 0.0, 0.0}));
    EXPECT_EQ(robot.world.region.rebaseCount, 0U);
    EXPECT_EQ(robot.bodyRestPose[0].position, before[0].position);
}

/// The base pose after 6 s of the drive test, with the deterministic scheduler.
std::array<double, 7> drivenPose(const Robot& robot) {
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<rigid_body::MujocoBackend>());
    scheduler.add(std::make_unique<DriveCommand>(DriveCommand::Settings{}, nullptr));
    DiffDriveController::Settings settings;
    settings.leftJoint = robot.actuatedJoints.at(0);
    settings.rightJoint = robot.actuatedJoints.at(1);
    scheduler.add(std::make_unique<DiffDriveController>(settings));
    scheduler.add(std::make_unique<electrical::IdealTorqueSource>());  // torque_command -> torque
    auto wired = scheduler.wire(WorldView(robot.world));
    EXPECT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    EXPECT_TRUE(scheduler.stepUntil(SimTime{6'000'000'000}).ok());
    const SignalBus& bus = scheduler.bus();
    const std::span<const double> pose = bus.readN(bus.find("body/base_link/pose"));
    std::array<double, 7> out{};
    std::copy(pose.begin(), pose.end(), out.begin());
    return out;
}

TEST(AmrSimulation, TheSdfAndMjcfVersionsDriveLikeTheUrdf) {
    // Acceptance criterion 7 (with test_usd_assets for the USD side): the three descriptions of
    // the sample AMR are the same robot. Body and joint order differ between the loaders, and
    // MuJoCo's contacts depend on it, so the paths agree to a few millimetres, not bit for bit.
    const std::array<double, 7> urdf = drivenPose(amr());
    for (const char* path : {LAGRANGE_TEST_AMR_SDF, LAGRANGE_TEST_AMR_MJCF}) {
        auto robot = loadModel(path);
        ASSERT_TRUE(robot) << robot.error();
        const std::array<double, 7> pose = drivenPose(*robot);
        EXPECT_NEAR(pose[0], urdf[0], 0.01) << path;
        EXPECT_NEAR(pose[1], urdf[1], 0.01) << path;
        EXPECT_NEAR(pose[2], urdf[2], 0.001) << path;
        // Same heading: |<q_urdf, q>| close to 1.
        double dot = 0.0;
        for (std::size_t k = 3; k < 7; ++k) {
            dot += pose[k] * urdf[k];
        }
        EXPECT_GT(std::abs(dot), std::cos(0.01)) << path;
    }
}

}  // namespace
}  // namespace lagrange::server
