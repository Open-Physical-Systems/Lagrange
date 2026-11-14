// Kind `wheel_odometry` (sensors.md §9): a two-wheeled robot on MuJoCo, driven by wheel torques.
// - On a grippy floor the wheels roll: odometry follows the ground truth closely, straight and
//   turning on the spot.
// - A radius 5 % too large reads every distance 5 % long; a 1024-count encoder reads in whole
//   counts.
// - On a slippery floor with too much torque the wheels spin: the odometry runs far ahead of the
//   body, the slip a real robot's odometry suffers, which nothing in the sensor models.
// - The ground-truth substitute reads the body's own pose exactly.
#include "amr_kinds_test_util.hpp"
#include "lagrange/sensors/Kinds.hpp"
#include "lagrange/world/Model.hpp"

#include <cstdio>
#include <numbers>

namespace lagrange::sensors {
namespace {

using namespace kindtest;

constexpr double kRadius = 0.1;  // m, the wheels
constexpr double kTrack = 0.4;   // m, between the wheels' mid-planes
constexpr double kCounts = 1024.0;

/// Base box with two driven wheels (axis +y) and two caster balls on ball joints.
World robot(double friction) {
    World w;
    w.addCollider(kWorldBody, ShapeType::Plane, {0, 0, 1}, Pose{}, friction);
    const BodyId base =
        w.addBody("base", kWorldBody, at(0, 0, kRadius), 10.0, {}, {0.1, 0.15, 0.2, 0, 0, 0});
    w.addJoint("base_free", JointType::Free, base, {0, 0, 1}, false);
    w.addCollider(base, ShapeType::Box, {0.2, 0.15, 0.03}, Pose{}, friction);
    // Cylinders stand along z: turned 90° about x, their axis is the wheel's y.
    const Pose upright = turned(std::numbers::pi / 2.0, 1, 0, 0);
    for (const auto& [name, y] :
         {std::pair{"left_wheel", kTrack / 2.0}, std::pair{"right_wheel", -kTrack / 2.0}}) {
        const BodyId wheel =
            w.addBody(name, base, at(0, y, 0), 1.0, {}, {0.0026, 0.005, 0.0026, 0, 0, 0});
        w.addJoint(std::string(name) + "_joint", JointType::Continuous, wheel, {0, 1, 0}, true);
        w.addCollider(wheel, ShapeType::Cylinder, {kRadius, 0.02, 0}, upright, friction);
    }
    for (const auto& [name, x] :
         {std::pair{"caster_front", 0.15}, std::pair{"caster_rear", -0.15}}) {
        const BodyId ball = w.addBody(name, base, at(x, 0, -0.05), 0.1, {}, diagonal(1e-4));
        w.addJoint(std::string(name) + "_joint", JointType::Spherical, ball, {0, 0, 1}, false);
        w.addCollider(ball, ShapeType::Sphere, {0.05, 0, 0}, Pose{}, friction);
    }
    // Found on its own: the two driven wheels under its body.
    w.addCustomSensor("odom", "wheel_odometry", base, Pose{}, 1000.0, {}, {});
    w.addCustomSensor("odom_long", "wheel_odometry", base, Pose{}, 1000.0, {{"radius_error", 0.05}},
                      {{"left_joint", "left_wheel_joint"}, {"right_joint", "right_wheel_joint"}});
    w.addCustomSensor("odom_encoder", "wheel_odometry", base, Pose{}, 1000.0,
                      {{"encoder_resolution", kCounts}}, {});
    return w;
}

std::vector<Writer::Output> torques(double left, double right) {
    return {{specOf("joint/left_wheel_joint/torque", SignalType::Scalar, units::N_m),
             [left](double) { return std::vector<double>{left}; }},
            {specOf("joint/right_wheel_joint/torque", SignalType::Scalar, units::N_m),
             [right](double) { return std::vector<double>{right}; }}};
}

double yawOf(const std::vector<double>& p) {
    return std::atan2(2.0 * (p[3] * p[6] + p[4] * p[5]), 1.0 - 2.0 * (p[5] * p[5] + p[6] * p[6]));
}

TEST(KindWheelOdometry, IsBuiltIn) {
    EXPECT_TRUE(isBuiltinKind("wheel_odometry"));
}

TEST(KindWheelOdometry, RollingStraightItFollowsTheGroundTruth) {
    const World w = robot(1.0);
    KindRun r = start(w, true, torques(0.3, 0.3));
    ASSERT_TRUE(r.scheduler);
    ASSERT_TRUE(r.until(3.0));
    const std::vector<double> body = r.readN("body/base/pose");
    const std::vector<double> odom = r.readN("sensor/odom/pose");
    ASSERT_GT(body[0], 1.0) << "it drove forward";
    std::printf("straight 3 s: truth x %.4f y %.4f, odometry x %.4f y %.4f, drift %.2f %%\n",
                body[0], body[1], odom[0], odom[1], 100.0 * (odom[0] - body[0]) / body[0]);
    EXPECT_NEAR(odom[0], body[0], 0.01 * body[0]);
    EXPECT_NEAR(odom[1], body[1], 0.01);
    EXPECT_NEAR(r.readN("sensor/odom/twist")[0], r.readN("body/base/twist")[0], 0.02);
    // A radius 5 % large reads 5 % long.
    EXPECT_NEAR(r.readN("sensor/odom_long/pose")[0], 1.05 * odom[0], 1e-6 * odom[0]);
    // The encoder reads whole counts: within a count's travel of the exact angle.
    EXPECT_NEAR(r.readN("sensor/odom_encoder/pose")[0], odom[0],
                2.0 * std::numbers::pi / kCounts * kRadius);
}

TEST(KindWheelOdometry, TurningOnTheSpotItFollowsTheYaw) {
    const World w = robot(1.0);
    KindRun r = start(w, true, torques(-0.3, 0.3));
    ASSERT_TRUE(r.scheduler);
    ASSERT_TRUE(r.until(2.0));
    const double truth = yawOf(r.readN("body/base/pose"));
    const double odom = r.read("sensor/odom/yaw");
    ASSERT_GT(truth, 0.2) << "it turned left";
    std::printf("turn 2 s: truth yaw %.4f rad, odometry %.4f rad\n", truth, odom);
    EXPECT_NEAR(odom, truth, 0.03 * truth);
}

TEST(KindWheelOdometry, SpinningWheelsOnASlipperyFloorOverstateTheTravel) {
    // μ = 0.05: about 6 N of traction against the 40 N the torques ask for.
    const World w = robot(0.05);
    KindRun r = start(w, true, torques(2.0, 2.0));
    ASSERT_TRUE(r.scheduler);
    ASSERT_TRUE(r.until(2.0));
    const double truth = r.readN("body/base/pose")[0];
    const double odom = r.readN("sensor/odom/pose")[0];
    std::printf("slip 2 s: truth x %.3f m, odometry %.3f m\n", truth, odom);
    EXPECT_GT(odom, 3.0 * truth);
    EXPECT_LT(truth, 0.5 * 0.05 * kGravity * 4.0 + 0.1) << "no faster than friction allows";
}

TEST(KindWheelOdometry, GroundTruthIsTheBodysPose) {
    const World w = robot(0.05);
    KindRun r = start(w, true, torques(2.0, 1.0), true);
    ASSERT_TRUE(r.scheduler);
    ASSERT_TRUE(r.until(2.0));
    const std::vector<double> body = r.readN("body/base/pose");
    const std::vector<double> odom = r.readN("sensor/odom/pose");
    EXPECT_NEAR(odom[0], body[0], 1e-3);
    EXPECT_NEAR(odom[1], body[1], 1e-3);
    EXPECT_NEAR(r.read("sensor/odom/yaw"), yawOf(body), 1e-3);
}

TEST(KindWheelOdometry, AnErrorOfMinusOneIsRefused) {
    // separation_error -1 believes a track of 0 m: every turn divided by zero and the pose read
    // NaN for ever. It is refused at start, as a radius_error of -1 is.
    for (const char* key : {"separation_error", "radius_error"}) {
        World w;
        const BodyId base =
            w.addBody("base", kWorldBody, at(0, 0, kRadius), 10.0, {}, {0.1, 0.15, 0.2, 0, 0, 0});
        w.addJoint("base_free", JointType::Free, base, {0, 0, 1}, false);
        for (const auto& [name, y] :
             {std::pair{"left_wheel", kTrack / 2.0}, std::pair{"right_wheel", -kTrack / 2.0}}) {
            const BodyId wheel =
                w.addBody(name, base, at(0, y, 0), 1.0, {}, {0.0026, 0.005, 0.0026, 0, 0, 0});
            w.addJoint(std::string(name) + "_joint", JointType::Continuous, wheel, {0, 1, 0}, true);
            w.addCollider(wheel, ShapeType::Cylinder, {kRadius, 0.02, 0}, Pose{}, 1.0);
        }
        w.addCustomSensor("odom", "wheel_odometry", base, Pose{}, 100.0, {{key, -1.0}}, {});
        Scheduler scheduler{SchedulerOptions{.deterministic = true}};
        scheduler.add(createBuiltinKind(w, 0, false, nullptr, nullptr));
        const auto wired = scheduler.wire(WorldView(w));
        ASSERT_FALSE(wired) << key;
        EXPECT_NE(to_string(wired.error().front()).find("above -1"), std::string::npos) << key;
    }
}

TEST(KindWheelOdometry, FindsTheWheelsOfAPassiveRobot) {
    // A robot added to a run (spawned) is passive: none of its joints is driven. Its odometry
    // still finds the two wheels under its body, so the spawn is not refused.
    World w = robot(1.0);
    std::fill(w.joints.actuated.begin(), w.joints.actuated.end(), false);
    Scheduler scheduler{SchedulerOptions{.deterministic = true}};
    scheduler.add(createBuiltinKind(w, 0, false, nullptr, nullptr));
    const auto wired = scheduler.wire(WorldView(w));
    // Alone on the scheduler its inputs have no writers; what matters is that it declared.
    if (!wired) {
        for (const auto& error : wired.error()) {
            EXPECT_NE(error.kind, WiringError::Kind::ModuleFailed) << to_string(error);
        }
    }
}

TEST(KindWheelOdometry, ASpawnedCopyReadsItsOwnWheels) {
    // A robot added to a run has its names prefixed ("copy_left_wheel_joint", prefixModel) while
    // its sensors' text still says "left_wheel_joint": the copy's odometry read the first
    // robot's wheels. Only the first robot drives here; the copy, 3 m to the side, must read 0.
    Model model;
    model.world = robot(1.0);
    model.world.robots = {"first"};
    Model copy;
    copy.world = robot(1.0);
    copy.world.bodies.poseInParent[0].position[1] += 3.0;
    model.bodyRestPose = restPoses(model.world);
    copy.bodyRestPose = restPoses(copy.world);
    prefixModel(copy, "copy", true);
    appendModel(model, copy);
    KindRun r = start(model.world, true, torques(0.3, 0.3));
    ASSERT_TRUE(r.scheduler);
    ASSERT_TRUE(r.until(2.0));
    ASSERT_GT(r.readN("sensor/odom_long/pose")[0], 0.5) << "the first robot drove";
    for (const char* sensor : {"sensor/copy_odom/pose", "sensor/copy_odom_long/pose"}) {
        EXPECT_NEAR(r.readN(sensor)[0], 0.0, 1e-3) << sensor;
    }
}

}  // namespace
}  // namespace lagrange::sensors
