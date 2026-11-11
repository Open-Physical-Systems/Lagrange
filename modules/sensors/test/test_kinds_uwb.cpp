// uwb (sensors.md §9): exact ranges in ground truth, the ±10 cm spread with line of sight, the
// excess delay through a concrete wall in closed form, dropouts beyond range, anchors as prims.
#include "more_kinds_util.hpp"

#include <cmath>
#include <memory>

namespace lagrange::sensors::test {
namespace {

/// A tag on a robot at (1, 2, 1); anchors at the origin (z = 3) and at (11, 2, 1).
KindRun uwbRun(std::map<std::string, double, std::less<>> settings = {}, double rate = 10.0) {
    KindRun r;
    const BodyId robot = movingBody(r.world, "robot");
    r.world.addCustomSensor("tag", "uwb", robot, Pose{}, rate, std::move(settings),
                            {{"anchors", "0 0 3; 11 2 1"}});
    return r;
}

std::unique_ptr<Writer> robotAt(std::array<double, 3> p = {1.0, 2.0, 1.0}) {
    auto w = std::make_unique<Writer>();
    w->body("robot", [p](double) { return p; });
    return w;
}

TEST(Uwb, GroundTruthIsTheDistance) {
    KindRun r = uwbRun();
    ASSERT_TRUE(r.wire(robotAt(), true));
    ASSERT_TRUE(r.runTo(0.5));
    const auto ranges = r.readN("sensor/tag/ranges");
    ASSERT_EQ(ranges.size(), 2U);
    EXPECT_DOUBLE_EQ(ranges[0], 3.0);  // √(1 + 4 + 4)
    EXPECT_DOUBLE_EQ(ranges[1], 10.0);
    EXPECT_EQ(r.readN("sensor/tag/los"), (std::vector<double>{1.0, 1.0}));
    EXPECT_EQ(r.read("sensor/tag/count"), 2.0);
}

TEST(Uwb, TenCentimetresWithLineOfSight) {
    KindRun r = uwbRun({}, 100.0);
    ASSERT_TRUE(r.wire(robotAt()));
    double sum = 0.0, squares = 0.0;
    constexpr int kSamples = 2000;
    for (int k = 1; k <= kSamples; ++k) {
        ASSERT_TRUE(r.runTo(0.01 * k));
        const double e = r.readN("sensor/tag/ranges")[1] - 10.0;
        sum += e;
        squares += e * e;
    }
    const double mean = sum / kSamples;
    const double sigma = std::sqrt(squares / kSamples - mean * mean);
    EXPECT_NEAR(mean, 0.0, 4.0 * 0.1 / std::sqrt(kSamples));
    EXPECT_NEAR(sigma, 0.1, 0.01);
}

TEST(Uwb, AConcreteWallDelaysThePulse) {
    // 0.2 m of concrete across the line to the second anchor: (√5.24 − 1)·0.2 m longer.
    KindRun r = uwbRun({{"range_noise", 0.0}, {"nlos_noise", 0.0}});
    const BodyId wall = fixedBody(r.world, "wall", {6.0, 2.0, 1.0});
    r.world.addCollider(wall, ShapeType::Box, {0.1, 1.0, 2.0}, Pose{}, 0.8);
    ASSERT_TRUE(r.wire(robotAt()));
    ASSERT_TRUE(r.runTo(0.5));
    const auto ranges = r.readN("sensor/tag/ranges");
    EXPECT_NEAR(ranges[1], 10.0 + (std::sqrt(5.24) - 1.0) * 0.2, 1e-5);
    EXPECT_NEAR(ranges[0], 3.0, 1e-12);
    EXPECT_EQ(r.readN("sensor/tag/los"), (std::vector<double>{1.0, 0.0}));
}

TEST(Uwb, NothingBeyondItsRange) {
    KindRun r = uwbRun({{"max_range", 5.0}});
    ASSERT_TRUE(r.wire(robotAt()));
    ASSERT_TRUE(r.runTo(0.5));
    const auto ranges = r.readN("sensor/tag/ranges");
    EXPECT_FALSE(std::isnan(ranges[0]));
    EXPECT_TRUE(std::isnan(ranges[1]));
    EXPECT_TRUE(std::isnan(r.readN("sensor/tag/los")[1]));
    EXPECT_EQ(r.read("sensor/tag/count"), 1.0);
}

TEST(Uwb, AnchorsNamedInTheWorld) {
    KindRun r;
    const BodyId robot = movingBody(r.world, "robot");
    const PrimId a = r.world.addPrim("/world/uwb_anchor_1", "uwb_anchor_1");
    r.world.prims.poseInBody[a.index].position = {1.0, 6.0, 4.0};
    fixedBody(r.world, "uwb_anchor_2", {1.0, 2.0, 7.0});
    r.world.addCustomSensor("tag", "uwb", robot, Pose{}, 10.0, {}, {});
    ASSERT_TRUE(r.wire(robotAt(), true));
    ASSERT_TRUE(r.runTo(0.5));
    const auto ranges = r.readN("sensor/tag/ranges");
    ASSERT_EQ(ranges.size(), 2U);
    EXPECT_DOUBLE_EQ(ranges[0], 5.0);  // (0, 4, 3)
    EXPECT_DOUBLE_EQ(ranges[1], 6.0);
}

// QA 2026-11-12 #17: after a floating-origin rebase (here 100 m along x) the robot's pose is in
// the new region frame; the anchors given as text must move with it, or every range is off by
// the rebase.
TEST(Uwb, TextAnchorsFollowAFloatingOriginRebase) {
    KindRun r = uwbRun();
    auto shift = std::make_shared<double>(0.0);
    auto w = std::make_unique<Writer>();
    w->body("robot", [shift](double) { return std::array<double, 3>{1.0 - *shift, 2.0, 1.0}; });
    ASSERT_TRUE(r.wire(std::move(w), true));
    ASSERT_TRUE(r.runTo(0.5));
    *shift = 100.0;
    r.scheduler->shiftOrigin({100.0, 0.0, 0.0});
    ASSERT_TRUE(r.runTo(1.0));
    const auto ranges = r.readN("sensor/tag/ranges");
    ASSERT_EQ(ranges.size(), 2U);
    EXPECT_NEAR(ranges[0], 3.0, 1e-9);
    EXPECT_NEAR(ranges[1], 10.0, 1e-9);
}

}  // namespace
}  // namespace lagrange::sensors::test
