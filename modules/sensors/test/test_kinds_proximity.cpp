// proximity (sensors.md §9): per-element distances to a wall as d / (cos el · cos az), the
// nearest of them, its own body unseen, the range cut and the noise.
#include "more_kinds_util.hpp"

#include <cmath>

namespace lagrange::sensors::test {
namespace {

/// A 2 × 3 skin on a robot at (0, 0, 1) facing +x; a wall whose face is at x = `face`.
KindRun skin(double face, std::map<std::string, double, std::less<>> settings, bool groundTruth) {
    KindRun r;
    const BodyId robot = movingBody(r.world, "robot");
    r.world.addCollider(robot, ShapeType::Box, {0.1, 0.1, 0.1}, Pose{}, 0.8);  // its own body
    settings.emplace("rows", 2.0);
    settings.emplace("columns", 3.0);
    settings.emplace("fov", 0.6);
    settings.emplace("vertical_fov", 0.2);
    r.world.addCustomSensor("skin", "proximity", robot, Pose{}, 50.0, std::move(settings), {});
    const BodyId wall = fixedBody(r.world, "wall", {face + 0.05, 0.0, 1.0});
    r.world.addCollider(wall, ShapeType::Box, {0.05, 2.0, 2.0}, Pose{}, 0.8);
    auto w = std::make_unique<Writer>();
    w->body("robot", [](double) { return std::array<double, 3>{0.0, 0.0, 1.0}; });
    r.wire(std::move(w), groundTruth);
    return r;
}

TEST(Proximity, EachElementSeesTheWallAlongItsRay) {
    KindRun r = skin(0.2, {}, true);
    ASSERT_TRUE(r.runTo(0.1));
    const auto d = r.readN("sensor/skin/distances");
    ASSERT_EQ(d.size(), 6U);
    for (int row = 0; row < 2; ++row) {
        const double el = -0.1 + (row + 0.5) * 0.1;
        for (int c = 0; c < 3; ++c) {
            const double az = -0.3 + (c + 0.5) * 0.2;
            EXPECT_NEAR(d[row * 3 + c], 0.2 / (std::cos(el) * std::cos(az)), 1e-9);
        }
    }
    EXPECT_NEAR(r.read("sensor/skin/min_distance"), 0.2 / std::cos(0.05), 1e-9);
    EXPECT_EQ(r.read("sensor/skin/detected"), 1.0);
}

TEST(Proximity, NothingBeyondItsReach) {
    KindRun r = skin(0.5, {}, false);
    ASSERT_TRUE(r.runTo(0.1));
    EXPECT_TRUE(std::isinf(r.read("sensor/skin/min_distance")));
    EXPECT_EQ(r.read("sensor/skin/detected"), 0.0);
    // Reach 0.202 m: the middle column (0.2003 m), not the sides (0.2045 m).
    KindRun s = skin(0.2, {{"max_range", 0.202}}, true);
    ASSERT_TRUE(s.runTo(0.1));
    const auto d = s.readN("sensor/skin/distances");
    EXPECT_TRUE(std::isinf(d[0]));
    EXPECT_TRUE(std::isfinite(d[1]));
    EXPECT_TRUE(std::isinf(d[2]));
}

TEST(Proximity, MillimetreNoise) {
    KindRun r = skin(0.2, {{"rows", 1.0}, {"columns", 1.0}}, false);
    double sum = 0.0, squares = 0.0;
    constexpr int kSamples = 1000;
    for (int k = 1; k <= kSamples; ++k) {
        ASSERT_TRUE(r.runTo(0.02 * k));
        const double e = r.read("sensor/skin/min_distance") - 0.2;
        sum += e;
        squares += e * e;
    }
    EXPECT_NEAR(sum / kSamples, 0.0, 2e-4);
    EXPECT_NEAR(std::sqrt(squares / kSamples), 0.001, 1e-4);
}

}  // namespace
}  // namespace lagrange::sensors::test
