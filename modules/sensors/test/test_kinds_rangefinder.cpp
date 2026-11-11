// Kind "rangefinder" (sensors.md §9): one beam along +x to the nearest collider other than its
// own body, its range limits, its noise, and fog shortening its reach.
#include "kinds_test_util.hpp"
#include "lagrange/sensors/WeatherEffects.hpp"

#include <cmath>
#include <limits>

namespace lagrange::sensors {
namespace {

using namespace test;

/// The sensor on a body with its own 0.2 m sphere, a 0.1 m thick wall whose face is at x = `wall`.
void scene(KindRun& r, double wall, std::map<std::string, double, std::less<>> settings = {}) {
    const BodyId frame = rig(r.world, "alt", "rangefinder", Pose{}, 100.0, std::move(settings));
    r.world.addCollider(frame, ShapeType::Sphere, {0.2, 0, 0}, Pose{}, 1.0);
    Pose at;
    at.position = {wall + 0.05, 0.0, 0.0};
    r.world.addCollider(kWorldBody, ShapeType::Box, {0.05, 2.0, 2.0}, at, 1.0);
}

TEST(Rangefinder, GroundTruthIsTheDistanceToTheWall) {
    KindRun r;
    scene(r, 5.0);
    ASSERT_TRUE(start(r, bodyFeeds(Pose{}, {}), true));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{50'000'000}).ok());
    EXPECT_NEAR(r.read("sensor/alt/range"), 5.0, 1e-9);  // past its own sphere
    EXPECT_EQ(r.read("sensor/alt/valid"), 1.0);
}

TEST(Rangefinder, BeyondItsRangeIsInfinity) {
    KindRun r;
    scene(r, 15.0);  // the TFmini-S reaches 12 m
    ASSERT_TRUE(start(r, bodyFeeds(Pose{}, {}), true));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{50'000'000}).ok());
    EXPECT_EQ(r.read("sensor/alt/range"), std::numeric_limits<double>::infinity());
    EXPECT_EQ(r.read("sensor/alt/valid"), 0.0);
}

TEST(Rangefinder, NoiseOfTheTFminiS) {
    KindRun r;
    scene(r, 8.0);
    ASSERT_TRUE(start(r, bodyFeeds(Pose{}, {})));
    double sum = 0.0;
    for (int k = 1; k <= 200; ++k) {
        ASSERT_TRUE(r.scheduler->stepUntil(SimTime{k * 10'000'000LL}).ok());
        const double d = r.read("sensor/alt/range");
        EXPECT_NEAR(d, 8.0, 4.0 * 0.04 + 0.005);  // σ = max(3 cm, 0.5 %) = 4 cm at 8 m
        EXPECT_NEAR(std::remainder(d, 0.01), 0.0, 1e-9);
        sum += d;
    }
    EXPECT_NEAR(sum / 200.0, 8.0, 0.01);
}

TEST(Rangefinder, FogShortensItsReach) {
    // The lidar equation's reach in an extinction of 0.5/m: W(σ·R)/σ, under 3 m for R = 12 m.
    const double reach = lidarDetectionRange(12.0, 0.5, 0.0);
    ASSERT_LT(reach, 3.0);
    for (const auto& [wall, seen] : {std::pair{reach - 0.3, true}, std::pair{reach + 0.3, false}}) {
        KindRun r;
        scene(r, wall, {{"noise/stddev", 0.0}, {"noise/fraction", 0.0}});
        std::vector<Feed> f = bodyFeeds(Pose{}, {});
        f.push_back(constant("env/extinction/nir_905", SignalType::Scalar, units::per_m, {0.5}));
        ASSERT_TRUE(start(r, std::move(f)));
        ASSERT_TRUE(r.scheduler->stepUntil(SimTime{50'000'000}).ok());
        EXPECT_EQ(r.read("sensor/alt/valid"), seen ? 1.0 : 0.0) << wall;
    }
}

TEST(Rangefinder, ThirtyHertzRunsOnWholeMilliseconds) {
    // 1e9/30 ns shares no tick with a 1 ms module: the run refused to start (base tick 1 ns).
    // Every kind's period is on whole milliseconds, as the cameras' are: 33 ms.
    KindRun r;
    const BodyId frame = rig(r.world, "alt", "rangefinder", Pose{}, 30.0);
    r.world.addCollider(frame, ShapeType::Sphere, {0.2, 0, 0}, Pose{}, 1.0);
    auto module = createBuiltinKind(r.world, 0, true, nullptr, nullptr);
    ASSERT_TRUE(module);
    EXPECT_EQ(module->preferredStep().ns, 33'000'000);
    // A period on whole 10 µs stays exact: 400 Hz is 2.5 ms, not 3.
    KindRun fast;
    rig(fast.world, "alt", "rangefinder", Pose{}, 400.0);
    EXPECT_EQ(createBuiltinKind(fast.world, 0, true, nullptr, nullptr)->preferredStep().ns,
              2'500'000);
    ASSERT_TRUE(start(r, bodyFeeds(Pose{}, {}), true));  // its feeds step at 1 ms
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{100'000'000}).ok());
}

TEST(Rangefinder, NotANumberIsRefused) {
    // "nan" parses as a number and passes every range check (comparisons with NaN are false):
    // a built-in kind refuses it by name instead of reading NaN.
    KindRun r;
    scene(r, 5.0, {{"range/max", std::numeric_limits<double>::quiet_NaN()}});
    r.scheduler = std::make_unique<Scheduler>(SchedulerOptions{.deterministic = true});
    r.scheduler->add(createBuiltinKind(r.world, 0, false, nullptr, nullptr));
    auto wired = r.scheduler->wire(WorldView(r.world));
    ASSERT_FALSE(wired);
    EXPECT_NE(to_string(wired.error().front()).find("range/max is not a number"),
              std::string::npos);
}

TEST(Rangefinder, AnUpdateRateOfZeroIsRefusedByName) {
    // Gazebo's <update_rate>0</update_rate> (as fast as possible) made a 1000 s period and a
    // refusal that blamed the profile's dt values.
    for (const double rate : {0.0, -5.0}) {
        KindRun r;
        rig(r.world, "alt", "rangefinder", Pose{}, rate);
        r.scheduler = std::make_unique<Scheduler>(SchedulerOptions{.deterministic = true});
        r.scheduler->add(createBuiltinKind(r.world, 0, false, nullptr, nullptr));
        auto wired = r.scheduler->wire(WorldView(r.world));
        ASSERT_FALSE(wired);
        EXPECT_NE(to_string(wired.error().front()).find("update_rate"), std::string::npos);
    }
}

TEST(Rangefinder, TooCloseIsMinusInfinity) {
    // REP 117: −inf when something is closer than the minimum range (a landed drone's
    // altimeter over the ground), +inf only when nothing is in range. It read +inf for both.
    KindRun r;
    scene(r, 0.05);  // the wall 5 cm ahead, the TFmini-S measures from 10 cm
    ASSERT_TRUE(start(r, bodyFeeds(Pose{}, {}), false));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{50'000'000}).ok());
    EXPECT_EQ(r.read("sensor/alt/range"), -std::numeric_limits<double>::infinity());
    EXPECT_EQ(r.read("sensor/alt/valid"), 0.0);
}

}  // namespace
}  // namespace lagrange::sensors
