#include "lagrange/rigid_body/MujocoBackend.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/server/DiffDrive.hpp"
#include "lagrange/server/Robot.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

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
    // The arm has visuals only: no collisions, so it would fall, and a mesh visual.
    EXPECT_FALSE(arm->notes.empty());
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

}  // namespace
}  // namespace lagrange::server
