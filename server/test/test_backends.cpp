// Backend agreement (roadmap v0.3 "Second rigid-body backend"): the same scenarios on MuJoCo
// and Jolt, through the whole server, compared with each other. MuJoCo is the reference: it has
// the closed-form validation suite of spike 0.2-C1; Jolt's own closed-form checks are in
// modules/rigid_body/jolt_backend/test/test_jolt.cpp.
#include "lagrange/rigid_body/JoltBackend.hpp"
#include "lagrange/rigid_body/MujocoBackend.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/sensors/ShapeCaster.hpp"
#include "lagrange/server/Robot.hpp"
#include "lagrange/server/Server.hpp"
#include "lagrange/stage/Stage.hpp"

#include <cmath>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

std::unique_ptr<Server> load(const std::string& physics, const std::string& robot,
                             const std::string& world, const std::string& overlay) {
    const std::filesystem::path dir =
        std::filesystem::path(::testing::TempDir()) / "lagrange_backends";
    std::filesystem::create_directories(dir);
    ServerOptions options;
    options.endpoint = proto::Endpoint::unixSocket((dir / (physics + ".sock")).string());
    options.robotPath = robot;
    options.worldPath = world;
    options.overlayPath = overlay;
    options.ambientTemperatureK = 298.15;
    options.realTimeFactor = 0.0;
    options.ros = false;
    options.cameras = false;
    options.physics = physics;
    auto server = std::make_unique<Server>(options);
    const Status loaded = server->load();
    EXPECT_TRUE(loaded.ok()) << physics << ": " << loaded.message;
    return loaded.ok() ? std::move(server) : nullptr;
}

TEST(Backends, TheSampleAmrDrivesTheSameOnBothWithEveryDomainOn) {
    // The demo drive for 6 s with electrical, thermal and sensors live: Jolt wires everything
    // MuJoCo does (the IMU's body acceleration, the motors' joint signals).
    std::vector<std::vector<double>> poses;
    std::vector<double> soc, force, speed, yawRate;
    for (const char* physics : {"mujoco", "jolt"}) {
        SCOPED_TRACE(physics);
        auto server = load(physics, LAGRANGE_TEST_AMR_URDF, LAGRANGE_TEST_WAREHOUSE_URDF,
                           LAGRANGE_TEST_AMR_OVERLAY);
        ASSERT_TRUE(server);
        EXPECT_NE(server->wiringDot().find(std::string("rigid_body/") + physics),
                  std::string::npos);
        ASSERT_TRUE(server->stepUntil(SimTime{6'000'000'000}).ok());
        poses.push_back(server->read("body/base_link/pose"));
        const std::vector<double> twist = server->read("body/base_link/twist");
        speed.push_back(std::hypot(twist[0], twist[1]));
        yawRate.push_back(twist[5]);
        soc.push_back(server->read("battery/battery/soc").at(0));
        force.push_back(server->read("sensor/imu/specific_force").at(2));
    }
    const std::vector<double>& m = poses[0];
    const std::vector<double>& j = poses[1];
    // Where they differ: on Jolt's rigid contacts the load between the drive wheels and the
    // casters is statically indeterminate, and the wheels slip while the drive ramps up, so Jolt
    // falls behind by about 0.4 m in the first 2 s. Recorded, not asserted (known issue).
    RecordProperty("position_difference_m", std::to_string(std::hypot(m[0] - j[0], m[1] - j[1])));
    // Once at speed, both follow the command: 0.6 m/s on a 1.2 m circle.
    EXPECT_NEAR(speed[0], 0.6, 0.02);
    EXPECT_NEAR(speed[1], 0.6, 0.02);
    EXPECT_NEAR(yawRate[0], 0.5, 0.02);
    EXPECT_NEAR(yawRate[1], 0.5, 0.02);
    EXPECT_NEAR(m[2], j[2], 0.005) << "both at axle height";
    // Both draw from the battery (the slip costs Jolt some energy in the ramp).
    EXPECT_NEAR(1.0 - soc[1], 1.0 - soc[0], 0.25 * (1.0 - soc[0]));
    // The IMU feels gravity's support on both (it is noisy: a loose bound).
    EXPECT_NEAR(force[0], 9.81, 0.5);
    EXPECT_NEAR(force[1], 9.81, 0.5);
}

const std::vector<std::string> kArmJoints{"shoulder_pan_joint", "shoulder_lift_joint",
                                          "elbow_joint",        "wrist_1_joint",
                                          "wrist_2_joint",      "wrist_3_joint"};

/// The arm's joint signals after `seconds` on `backend`, its joints free, under gravity tilted
/// 0.3 rad off vertical (the upright arm is an equilibrium otherwise). `armature` is added to
/// the shoulder lift and elbow.
struct ArmRun {
    std::vector<double> inertia, acceleration, position;
};
ArmRun runArm(bool jolt, double seconds, double armature) {
    auto robot = loadUrdf(LAGRANGE_TEST_ARM6_URDF);
    EXPECT_TRUE(robot) << robot.error();
    World& world = robot->world;
    constexpr double kTilt = 0.3;  // rad
    world.gravityLocal = {9.80665 * std::sin(kTilt), 0.0, -9.80665 * std::cos(kTilt)};
    for (std::size_t j = 0; j < world.joints.names.size(); ++j) {
        if (world.joints.names[j] == "shoulder_lift_joint" ||
            world.joints.names[j] == "elbow_joint") {
            world.joints.armature[j] = armature;
        }
    }
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    if (jolt) {
        scheduler.add(std::make_unique<rigid_body::JoltBackend>());
    } else {
        scheduler.add(std::make_unique<rigid_body::MujocoBackend>());
    }
    auto wired = scheduler.wire(WorldView(world));
    EXPECT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    ArmRun out;
    const SignalBus& bus = scheduler.bus();
    for (const std::string& joint : kArmJoints) {
        out.inertia.push_back(bus.read(bus.find("joint/" + joint + "/effective_inertia")));
    }
    EXPECT_TRUE(scheduler.stepUntil(SimTime{static_cast<std::int64_t>(seconds * 1e9)}).ok());
    for (const std::string& joint : kArmJoints) {
        out.acceleration.push_back(bus.read(bus.find("joint/" + joint + "/acceleration")));
        out.position.push_back(bus.read(bus.find("joint/" + joint + "/position")));
    }
    return out;
}

TEST(Backends, TheArmsJointSpaceInertiaAndFallAgree) {
    // What each joint's motor turns (MuJoCo's joint-space inertia diagonal, Jolt's subtree
    // inertia about the axis), and the arm's fall over 0.3 s: the whole multibody's dynamics.
    // (Jolt's accelerations are differences of velocity, so its first steps are not compared.)
    const ArmRun m = runArm(false, 0.3, 0.0);
    const ArmRun j = runArm(true, 0.3, 0.0);
    for (std::size_t i = 0; i < kArmJoints.size(); ++i) {
        SCOPED_TRACE(kArmJoints[i]);
        EXPECT_NEAR(j.inertia[i], m.inertia[i], 0.02 * m.inertia[i] + 1e-6);
        EXPECT_NEAR(j.position[i], m.position[i], 0.02 * std::abs(m.position[i]) + 0.005);
        EXPECT_NEAR(j.acceleration[i], m.acceleration[i], 0.05 * std::abs(m.acceleration[i]) + 0.5);
    }
}

TEST(Backends, ArmatureSlowsTheFallAlike) {
    // Rotor inertia on the shoulder lift and elbow: MuJoCo has it in its joint-space inertia,
    // Jolt applies it as joint torques from its own joint-space inertia (JoltBackend). The
    // fall over 0.3 s agrees, wrist included (the coupling).
    constexpr double kArmature = 0.5;  // kg·m², a geared motor's reflected rotor inertia
    const ArmRun m = runArm(false, 0.3, kArmature);
    const ArmRun j = runArm(true, 0.3, kArmature);
    const ArmRun free = runArm(false, 0.3, 0.0);
    for (const std::size_t i : {1U, 2U, 4U}) {
        SCOPED_TRACE(kArmJoints[i]);
        EXPECT_NEAR(j.inertia[i], m.inertia[i], 0.02 * m.inertia[i]);
        RecordProperty(kArmJoints[i] + "_mujoco_rad", std::to_string(m.position[i]));
        RecordProperty(kArmJoints[i] + "_jolt_rad", std::to_string(j.position[i]));
        EXPECT_NEAR(j.position[i], m.position[i], 0.05 * std::abs(m.position[i]) + 0.015);
    }
    for (const std::size_t i : {1U, 2U}) {
        EXPECT_LT(std::abs(m.position[i]), std::abs(free.position[i])) << "the armature slows it";
    }
}

TEST(Backends, AConvexHullBoxRestsLikeABoxOnBoth) {
    // Collision geometry as a convex hull (the collision bake's output): a 0.2 m cube's hull
    // dropped from 0.15 m comes to rest at its half height.
    for (const bool jolt : {false, true}) {
        SCOPED_TRACE(jolt ? "jolt" : "mujoco");
        World world;
        world.addCollider(kWorldBody, ShapeType::Plane, {0, 0, 1}, Pose{}, 1.0);
        const BodyId box = world.addBody("box", kWorldBody, Pose{{0.0, 0.0, 0.15}}, 1.0, {},
                                         {0.0067, 0.0067, 0.0067, 0, 0, 0});
        world.addJoint("box_free", JointType::Free, box, {0, 0, 1}, false);
        world.addHullCollider(box, boxHull({0.1, 0.1, 0.1}), Pose{}, 1.0);
        Scheduler scheduler(SchedulerOptions{.deterministic = true});
        if (jolt) {
            scheduler.add(std::make_unique<rigid_body::JoltBackend>());
        } else {
            scheduler.add(std::make_unique<rigid_body::MujocoBackend>());
        }
        auto wired = scheduler.wire(WorldView(world));
        ASSERT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
        ASSERT_TRUE(scheduler.stepUntil(SimTime{1'500'000'000}).ok());
        const std::span<const double> pose =
            scheduler.bus().readN(scheduler.bus().find("body/box/pose"));
        EXPECT_NEAR(pose[2], 0.1, 0.002);
        EXPECT_NEAR(pose[3], 1.0, 1e-3) << "flat on a face";
    }
}

// Terrain as a height field (huge-worlds.md §4.1): both backends must read the grid the same
// way round. A 13 x 13 field, 1 m cells, flat but for the (+x, +y) quarter raised by 1 m; a
// ball dropped over each quarter rests on what is under it, so a flipped row or column order
// shows as the wrong ball sitting high.
TEST(Backends, AHeightFieldIsTheSameWayRoundOnBoth) {
    for (const bool jolt : {false, true}) {
        SCOPED_TRACE(jolt ? "jolt" : "mujoco");
        World world;
        HeightField field;
        field.columns = 13;
        field.rows = 13;
        field.cellX = 1.0;
        field.cellY = 1.0;
        for (std::uint32_t r = 0; r < 13; ++r) {
            for (std::uint32_t c = 0; c < 13; ++c) {
                field.heights.push_back(c > 6 && r > 6 ? 1.0 : 0.0);
            }
        }
        world.addHeightFieldCollider(field, Pose{{10.0, -5.0, 0.0}}, 1.0);
        const std::array<std::array<double, 2>, 4> spots{{{3, 3}, {-3, 3}, {3, -3}, {-3, -3}}};
        for (std::size_t k = 0; k < spots.size(); ++k) {
            const std::string name = "ball_" + std::to_string(k);
            const BodyId ball =
                world.addBody(name, kWorldBody, Pose{{10.0 + spots[k][0], -5.0 + spots[k][1], 1.6}},
                              1.0, {}, {0.004, 0.004, 0.004, 0, 0, 0});
            world.addJoint(name + "_free", JointType::Free, ball, {0, 0, 1}, false);
            world.addCollider(ball, ShapeType::Sphere, {0.1, 0, 0}, Pose{}, 1.0);
        }
        Scheduler scheduler(SchedulerOptions{.deterministic = true});
        if (jolt) {
            scheduler.add(std::make_unique<rigid_body::JoltBackend>());
        } else {
            scheduler.add(std::make_unique<rigid_body::MujocoBackend>());
        }
        auto wired = scheduler.wire(WorldView(world));
        ASSERT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
        ASSERT_TRUE(scheduler.stepUntil(SimTime{2'000'000'000}).ok());
        for (std::size_t k = 0; k < spots.size(); ++k) {
            const std::span<const double> pose = scheduler.bus().readN(
                scheduler.bus().find("body/ball_" + std::to_string(k) + "/pose"));
            const double ground = k == 0 ? 1.0 : 0.0;
            EXPECT_NEAR(pose[2], ground + 0.1, 0.01) << "ball " << k;
        }
    }
}

TEST(CollisionBake, TheBakedRackKeepsItsGapsForTheLidarAndHoldsABox) {
    // assets/worlds/rack_sample/rack_baked.usda: the sample rack (one non-convex mesh, x = 3 m)
    // after collision_bake. The lidar sees between its boards and stops at a board's edge; a
    // box rests on its middle board.
    auto rack = stage::loadStage(LAGRANGE_TEST_RACK_BAKED_USDA);
    ASSERT_TRUE(rack) << rack.error();
    const World& world = rack->world;
    const auto hullCount = std::count(world.colliders.shape.begin(), world.colliders.shape.end(),
                                      ShapeType::ConvexHull);
    RecordProperty("hulls", std::to_string(hullCount));
    EXPECT_GE(hullCount, 7);
    std::vector<sensors::ShapeCaster::Shape> shapes;
    for (std::size_t c = 0; c < world.colliders.body.size(); ++c) {
        shapes.push_back({world.colliders.shape[c], world.colliders.size[c],
                          world.colliders.poseInBody[c],
                          world.colliders.shape[c] == ShapeType::ConvexHull
                              ? &world.hulls.at(world.colliders.hull[c])
                              : nullptr});
    }
    sensors::ShapeCaster caster;
    caster.update(shapes);
    // Between the first two boards (0.3 m and 1.0 m), between the uprights: nothing.
    EXPECT_TRUE(std::isinf(caster.cast({0.0, 0.0, 0.65}, {1.0, 0.0, 0.0})));
    // At a board's height: its near edge, 3 - 0.6 = 2.4 m away.
    EXPECT_NEAR(caster.cast({0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}), 2.4, 0.003);

    // A 0.2 m box dropped on the middle board (top at 1.015 m) rests at 1.115 m.
    Model scene = *rack;
    World& w = scene.world;
    const BodyId box = w.addBody("box", kWorldBody, Pose{{3.0, 0.0, 1.25}}, 1.0, {},
                                 {0.0067, 0.0067, 0.0067, 0, 0, 0});
    w.addJoint("box_free", JointType::Free, box, {0, 0, 1}, false);
    w.addCollider(box, ShapeType::Box, {0.1, 0.1, 0.1}, Pose{}, 1.0);
    for (const bool jolt : {false, true}) {
        SCOPED_TRACE(jolt ? "jolt" : "mujoco");
        Scheduler scheduler(SchedulerOptions{.deterministic = true});
        if (jolt) {
            scheduler.add(std::make_unique<rigid_body::JoltBackend>());
        } else {
            scheduler.add(std::make_unique<rigid_body::MujocoBackend>());
        }
        auto wired = scheduler.wire(WorldView(w));
        ASSERT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
        ASSERT_TRUE(scheduler.stepUntil(SimTime{1'500'000'000}).ok());
        EXPECT_NEAR(scheduler.bus().readN(scheduler.bus().find("body/box/pose"))[2], 1.115, 0.003);
    }
}

TEST(Backends, AProfileChoosesTheBackend) {
    auto server = load("mujoco", LAGRANGE_TEST_AMR_URDF, LAGRANGE_TEST_WAREHOUSE_URDF,
                       LAGRANGE_TEST_AMR_OVERLAY);
    ASSERT_TRUE(server);
    const auto profile = [](const char* backend) {
        return std::string("schema_version: 1\nstage: ") + LAGRANGE_TEST_AMR_URDF +
               "\ndomains:\n  rigid_body: {enabled: true, backend: " + backend +
               "}\n  electrical: {enabled: true}\n  thermal: {enabled: true, "
               "ambient_temperature: 298.15}\n  sensors: {enabled: true}\n";
    };
    const Status jolt = server->applyProfile(profile("jolt"));
    ASSERT_TRUE(jolt.ok()) << jolt.message;
    EXPECT_NE(server->wiringDot().find("rigid_body/jolt"), std::string::npos);
    ASSERT_TRUE(server->stepUntil(SimTime{1'000'000'000}).ok());
    const Status physx = server->applyProfile(profile("physx"));
    ASSERT_FALSE(physx.ok());
    EXPECT_NE(physx.message.find("jolt, mujoco"), std::string::npos) << physx.message;
}

TEST(Backends, JoltRunsAreDeterministicThroughTheServer) {
    std::vector<std::uint64_t> hashes;
    for (int run = 0; run < 2; ++run) {
        auto server = load("jolt", LAGRANGE_TEST_AMR_URDF, LAGRANGE_TEST_WAREHOUSE_URDF,
                           LAGRANGE_TEST_AMR_OVERLAY);
        ASSERT_TRUE(server);
        ASSERT_TRUE(server->stepUntil(SimTime{1'000'000'000}).ok());
        const auto hash = server->stateHash();
        ASSERT_TRUE(hash) << hash.error().message;
        hashes.push_back(*hash);
    }
    EXPECT_EQ(hashes[0], hashes[1]);
}

}  // namespace
}  // namespace lagrange::server
