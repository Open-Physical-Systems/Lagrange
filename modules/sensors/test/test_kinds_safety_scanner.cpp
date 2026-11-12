// Kind `safety_scanner` (sensors.md §9): a plate drives straight at a scanner at 1 m/s. Its face
// crosses the 2 m warning field (a polygon) at t = 1 s and the 1 m protective field (a rectangle)
// at t = 2 s; each output switches one response time (70 ms, rounded up to whole 40 ms scans)
// after the first scan that sees it, and the ground-truth substitute switches with no delay.
// min_distance is the face's distance.
#include "amr_kinds_test_util.hpp"
#include "lagrange/sensors/Kinds.hpp"

namespace lagrange::sensors {
namespace {

using namespace kindtest;

constexpr double kStart = 3.05;  // m, the plate's centre; its face is 5 cm nearer
constexpr double kSpeed = 1.0;   // m/s, towards the scanner
constexpr double kScan = 0.04;   // s, 25 Hz

World approach() {
    World w;
    const BodyId mount = w.addBody("mount", kWorldBody, at(0, 0, 0.2), 1.0, {}, diagonal(0.01));
    w.addCollider(mount, ShapeType::Cylinder, {0.06, 0.08, 0}, Pose{}, 1.0);  // its housing
    const BodyId plate = w.addBody("plate", kWorldBody, at(kStart, 0, 0.2), 10.0, {}, diagonal(1));
    w.bodies.kinematic.back() = true;
    w.addCollider(plate, ShapeType::Box, {0.05, 0.5, 0.3}, Pose{}, 1.0);
    w.addCustomSensor(
        "scanner", "safety_scanner", mount, Pose{}, 1.0 / kScan,
        {{"protective/x_max", 1.0}, {"protective/y_min", -0.5}, {"protective/y_max", 0.5}},
        {{"warning/polygon", "0,-0.6 2,-0.6 2,0.6 0,0.6"}});
    return w;
}

std::vector<Writer::Output> drive() {
    return {
        {specOf("body/plate/pose_command", SignalType::Pose, units::m, "world_local"),
         [](double t) { return std::vector<double>{kStart - kSpeed * t, 0, 0.2, 1, 0, 0, 0}; }}};
}

TEST(KindSafetyScanner, IsBuiltIn) {
    EXPECT_TRUE(isBuiltinKind("safety_scanner"));
}

TEST(KindSafetyScanner, FieldsSwitchOneResponseTimeLate) {
    const World w = approach();
    KindRun r = start(w, true, drive());
    ASSERT_TRUE(r.scheduler);
    ASSERT_TRUE(r.until(0.5));
    EXPECT_NEAR(r.read("sensor/scanner/min_distance"), 2.5, 0.07);  // the face, one scan ago
    EXPECT_EQ(r.read("sensor/scanner/warning"), 0.0);
    EXPECT_EQ(r.readN("sensor/scanner/ranges").size(), 1618U) << "275° at 0.17°";
    // In the warning field from t = 1 s; seen by the scan at 1.0x s, shown 2 scans later.
    ASSERT_TRUE(r.until(1.0 + kScan));
    EXPECT_EQ(r.read("sensor/scanner/warning"), 0.0) << "still within the response time";
    ASSERT_TRUE(r.until(1.0 + 4.0 * kScan + 0.001));
    EXPECT_EQ(r.read("sensor/scanner/warning"), 1.0);
    EXPECT_EQ(r.read("sensor/scanner/protective"), 0.0);
    ASSERT_TRUE(r.until(2.0 + kScan));
    EXPECT_EQ(r.read("sensor/scanner/protective"), 0.0) << "still within the response time";
    ASSERT_TRUE(r.until(2.0 + 4.0 * kScan + 0.001));
    EXPECT_EQ(r.read("sensor/scanner/protective"), 1.0);
    EXPECT_NEAR(r.read("sensor/scanner/min_distance"), 3.0 - (2.0 + 3.0 * kScan), 0.05);
}

TEST(KindSafetyScanner, GroundTruthHasNoDelay) {
    const World w = approach();
    KindRun r = start(w, true, drive(), true);
    ASSERT_TRUE(r.scheduler);
    ASSERT_TRUE(r.until(2.0 + 2.0 * kScan + 0.001));
    EXPECT_EQ(r.read("sensor/scanner/protective"), 1.0);
}

// Speed-dependent field switching: a robot drives at 1 m/s towards a wall 4 m ahead. The
// switching scanner, fast above 0.5 m/s, watches 1.5 m ahead and trips with the wall 1.5 m away;
// the fixed one watches 0.5 m and trips a metre later.
World driving() {
    World w;
    const BodyId robot = w.addBody("robot", kWorldBody, at(0, 0, 0.2), 10.0, {}, diagonal(1));
    w.bodies.kinematic.back() = true;
    w.addCollider(kWorldBody, ShapeType::Box, {0.05, 2.0, 0.5}, at(4.05, 0, 0.5), 1.0);
    const std::map<std::string, double, std::less<>> slow{
        {"protective/x_max", 0.5}, {"protective/y_min", -0.4}, {"protective/y_max", 0.4}};
    auto switching = slow;
    switching.insert({{"switch_speed", 0.5},
                      {"protective_fast/x_max", 1.5},
                      {"protective_fast/y_min", -0.4},
                      {"protective_fast/y_max", 0.4}});
    w.addCustomSensor("switching", "safety_scanner", robot, Pose{}, 1.0 / kScan, switching, {});
    w.addCustomSensor("fixed", "safety_scanner", robot, Pose{}, 1.0 / kScan, slow, {});
    return w;
}

TEST(KindSafetyScanner, FieldsSwitchWithSpeed) {
    const World w = driving();
    KindRun r =
        start(w, true,
              {{specOf("body/robot/pose_command", SignalType::Pose, units::m, "world_local"),
                [](double t) { return std::vector<double>{kSpeed * t, 0, 0.2, 1, 0, 0, 0}; }}});
    ASSERT_TRUE(r.scheduler);
    // The wall is 1.5 m away at t = 2.5 s, 0.5 m at 3.5 s; outputs come 2 scans after a scan.
    ASSERT_TRUE(r.until(2.4));
    EXPECT_EQ(r.read("sensor/switching/case"), 1.0) << "fast";
    EXPECT_EQ(r.read("sensor/switching/protective"), 0.0);
    ASSERT_TRUE(r.until(2.5 + 4.0 * kScan + 0.001));
    EXPECT_EQ(r.read("sensor/switching/protective"), 1.0);
    EXPECT_EQ(r.read("sensor/fixed/protective"), 0.0);
    ASSERT_TRUE(r.until(3.5 + 4.0 * kScan + 0.001));
    EXPECT_EQ(r.read("sensor/fixed/protective"), 1.0);
}

TEST(KindSafetyScanner, ItNeedsAProtectiveField) {
    World w;
    const BodyId mount = w.addBody("mount", kWorldBody, Pose{}, 1.0, {}, diagonal(0.01));
    w.addCustomSensor("scanner", "safety_scanner", mount, Pose{}, 25.0, {}, {});
    Scheduler scheduler;
    for (auto& m : createSensorModules(w)) {
        scheduler.add(std::move(m));
    }
    auto wired = scheduler.wire(WorldView(w));
    ASSERT_FALSE(wired);
    EXPECT_NE(to_string(wired.error().front()).find("protective"), std::string::npos);
}

TEST(KindSafetyScanner, AnAbsurdResponseTimeIsRefused) {
    // Its delay is one state per scan: 1e9 s asked for 3e10 bytes (std::bad_alloc at start).
    World w;
    const BodyId mount = w.addBody("mount", kWorldBody, Pose{}, 1.0, {}, diagonal(0.01));
    w.addCustomSensor("scanner", "safety_scanner", mount, Pose{}, 25.0,
                      {{"protective/radius", 0.5}, {"response_time", 1e9}}, {});
    Scheduler scheduler;
    for (auto& m : createSensorModules(w)) {
        scheduler.add(std::move(m));
    }
    auto wired = scheduler.wire(WorldView(w));
    ASSERT_FALSE(wired);
    EXPECT_NE(to_string(wired.error().front()).find("response_time"), std::string::npos);
}

}  // namespace
}  // namespace lagrange::sensors
