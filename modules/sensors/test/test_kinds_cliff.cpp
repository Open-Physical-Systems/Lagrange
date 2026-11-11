// Kind `cliff` (sensors.md §9): cliff sensors over a table whose edge is at x = 1 m, with the
// floor half a metre below. Over the table they read their height exactly; past the edge the
// floor is out of their range and they report a cliff; held too high over the table they see it
// but farther than the threshold, a drop too. Noise has the sigma asked for, and the ground-truth
// substitute reads exactly.
#include "amr_kinds_test_util.hpp"
#include "lagrange/sensors/Kinds.hpp"

#include <limits>

namespace lagrange::sensors {
namespace {

using namespace kindtest;

constexpr double kNoise = 0.002;  // m, 1 sigma

World table() {
    World w;
    w.addCollider(kWorldBody, ShapeType::Plane, {0, 0, 1}, at(0, 0, -0.5), 1.0);
    // The table top at z = 0 over x in [-1, 1].
    w.addCollider(kWorldBody, ShapeType::Box, {1.0, 1.0, 0.05}, at(0.0, 0.0, -0.05), 1.0);
    // A robot standing still (no joint: its rest pose) with a skirt the rays must pass.
    const BodyId robot = w.addBody("robot", kWorldBody, at(0.0, 0.0, 0.1), 5.0, {}, diagonal(0.1));
    w.addCollider(robot, ShapeType::Box, {1.5, 0.2, 0.08}, Pose{}, 1.0);
    const std::map<std::string, double, std::less<>> threshold{{"threshold", 0.06}};
    w.addCustomSensor("over_table", "cliff", robot, at(0.5, 0.0, -0.06), 100.0, threshold, {});
    w.addCustomSensor("past_edge", "cliff", robot, at(1.2, 0.0, -0.06), 100.0, threshold, {});
    w.addCustomSensor("too_high", "cliff", robot, at(-0.5, 0.0, -0.01), 100.0, threshold, {});
    w.addCustomSensor("noisy", "cliff", robot, at(0.0, 0.0, -0.06), 100.0,
                      {{"range_noise", kNoise}}, {});
    // A cone of five rays at the edge: the nearest return (the table) wins.
    w.addCustomSensor("cone", "cliff", robot, at(1.01, 0.0, -0.06), 100.0,
                      {{"rays", 5.0}, {"beam_angle", 0.3}}, {});
    return w;
}

TEST(KindCliff, IsBuiltIn) {
    EXPECT_TRUE(isBuiltinKind("cliff"));
}

TEST(KindCliff, ReadsTheTableAndTheDrop) {
    const World w = table();
    KindRun r = start(w, false, {});
    ASSERT_TRUE(r.scheduler);
    ASSERT_TRUE(r.until(0.05));
    EXPECT_NEAR(r.read("sensor/over_table/range"), 0.04, 1e-12);
    EXPECT_EQ(r.read("sensor/over_table/cliff"), 0.0);
    EXPECT_EQ(r.read("sensor/past_edge/range"), std::numeric_limits<double>::infinity());
    EXPECT_EQ(r.read("sensor/past_edge/cliff"), 1.0);
    EXPECT_NEAR(r.read("sensor/too_high/range"), 0.09, 1e-12);
    EXPECT_EQ(r.read("sensor/too_high/cliff"), 1.0) << "0.09 m is beyond the 0.06 m threshold";
    // Past the edge by 1 cm, the slanted rays still find the table 4 cm down.
    EXPECT_NEAR(r.read("sensor/cone/range"), 0.04 / std::cos(0.3), 1e-9);
    EXPECT_EQ(r.read("sensor/cone/cliff"), 0.0);
}

TEST(KindCliff, NoiseHasItsSigmaAndGroundTruthIsExact) {
    const World w = table();
    KindRun r = start(w, false, {});
    KindRun truth = start(w, false, {}, true);
    ASSERT_TRUE(r.scheduler && truth.scheduler);
    double sum = 0.0;
    double squares = 0.0;
    constexpr int kSamples = 2000;
    for (int k = 1; k <= kSamples; ++k) {
        ASSERT_TRUE(r.until(0.01 * k));
        const double e = r.read("sensor/noisy/range") - 0.04;
        sum += e;
        squares += e * e;
    }
    EXPECT_NEAR(sum / kSamples, 0.0, 4.0 * kNoise / std::sqrt(kSamples));
    EXPECT_NEAR(std::sqrt(squares / kSamples), kNoise, 0.05 * kNoise);
    ASSERT_TRUE(truth.until(0.5));
    EXPECT_NEAR(truth.read("sensor/noisy/range"), 0.04, 1e-12);
}

}  // namespace
}  // namespace lagrange::sensors
