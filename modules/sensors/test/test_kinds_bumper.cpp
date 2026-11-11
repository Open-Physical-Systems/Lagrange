// Kind `bumper` (sensors.md §9): a box robot pushed by a known force into a wall. At rest against
// the wall everything that touches it (the wall and the floor) pushes back with exactly the
// push, so the front bumper reads it and closes; the rear one, whose sector looks back, stays
// open; and standing on the floor alone, the weight is vertical and closes neither.
#include "amr_kinds_test_util.hpp"
#include "lagrange/sensors/Kinds.hpp"

#include <numbers>

namespace lagrange::sensors {
namespace {

using namespace kindtest;

constexpr double kPush = 30.0;     // N, along +x, from t = 1 s
constexpr double kFriction = 0.1;  // floor and box: the push overcomes it
constexpr double kMass = 10.0;     // kg

World pushedBox() {
    World w;
    w.addCollider(kWorldBody, ShapeType::Plane, {0, 0, 1}, Pose{}, kFriction);
    // The wall's face at x = 0.45; the robot's front at x = 0.2.
    w.addCollider(kWorldBody, ShapeType::Box, {0.05, 1.0, 0.5}, at(0.5, 0.0, 0.5), kFriction);
    const BodyId robot = w.addBody("robot", kWorldBody, at(0.0, 0.0, 0.1), kMass, {},
                                   {0.11, 0.17, 0.24, 0.0, 0.0, 0.0});
    w.addJoint("robot_free", JointType::Free, robot, {0, 0, 1}, false);
    w.addCollider(robot, ShapeType::Box, {0.2, 0.15, 0.1}, Pose{}, kFriction);
    // Front: pressed by what is ahead within ±60°; rear: within ±60° of straight behind.
    w.addCustomSensor("front_bumper", "bumper", robot, at(0.2, 0.0, 0.0), 100.0,
                      {{"threshold", 5.0}, {"sector_min", -1.05}, {"sector_max", 1.05}}, {});
    w.addCustomSensor("rear_bumper", "bumper", robot, turned(std::numbers::pi, 0, 0, 1, -0.2),
                      100.0, {{"sector_min", -1.05}, {"sector_max", 1.05}}, {});
    // Every direction, the vertical part too: it reads the weight.
    w.addCustomSensor("skin", "bumper", robot, Pose{}, 100.0, {{"vertical", 1.0}}, {});
    return w;
}

std::vector<Writer::Output> push() {
    return {{specOf("body/robot/external_wrench", SignalType::Wrench, units::N, "world_local"),
             [](double t) { return std::vector<double>{t >= 1.0 ? kPush : 0.0, 0, 0, 0, 0, 0}; }}};
}

TEST(KindBumper, IsBuiltIn) {
    EXPECT_TRUE(isBuiltinKind("bumper"));
}

TEST(KindBumper, OnTheFloorAloneNothingIsPressed) {
    const World w = pushedBox();
    KindRun r = start(w, true, push());
    ASSERT_TRUE(r.scheduler);
    ASSERT_TRUE(r.until(0.9));
    EXPECT_EQ(r.read("sensor/front_bumper/pressed"), 0.0);
    EXPECT_EQ(r.read("sensor/rear_bumper/pressed"), 0.0);
    EXPECT_NEAR(r.read("sensor/front_bumper/force"), 0.0, 0.5);
    // The floor holds up the weight: the vertical skin reads m·g and is pressed.
    EXPECT_NEAR(r.read("sensor/skin/force"), kMass * kGravity, 0.01 * kMass * kGravity);
    EXPECT_EQ(r.read("sensor/skin/pressed"), 1.0);
}

TEST(KindBumper, AgainstTheWallTheFrontReadsThePush) {
    const World w = pushedBox();
    KindRun r = start(w, true, push());
    ASSERT_TRUE(r.scheduler);
    ASSERT_TRUE(r.until(3.0));
    // At rest: the wall and the floor's friction together push back with exactly the push.
    EXPECT_NEAR(r.readN("body/robot/pose")[0], 0.25, 0.01) << "against the wall";
    EXPECT_EQ(r.read("sensor/front_bumper/pressed"), 1.0);
    EXPECT_NEAR(r.read("sensor/front_bumper/force"), kPush, 0.02 * kPush);
    EXPECT_NEAR(r.read("sensor/front_bumper/direction"), 0.0, 0.02) << "straight ahead";
    EXPECT_EQ(r.read("sensor/rear_bumper/pressed"), 0.0) << "the rear looks the other way";
    EXPECT_NEAR(r.read("sensor/rear_bumper/force"), kPush, 0.02 * kPush);
    EXPECT_NEAR(std::abs(r.read("sensor/rear_bumper/direction")), std::numbers::pi, 0.02);
}

TEST(KindBumper, GroundTruthReadsTheSame) {
    const World w = pushedBox();
    KindRun a = start(w, true, push());
    KindRun b = start(w, true, push(), true);
    ASSERT_TRUE(a.scheduler && b.scheduler);
    ASSERT_TRUE(a.until(3.0) && b.until(3.0));
    EXPECT_EQ(a.read("sensor/front_bumper/force"), b.read("sensor/front_bumper/force"));
    EXPECT_EQ(b.read("sensor/front_bumper/pressed"), 1.0);
}

}  // namespace
}  // namespace lagrange::sensors
