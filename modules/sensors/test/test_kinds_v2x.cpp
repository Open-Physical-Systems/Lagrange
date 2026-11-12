// Kind "v2x" (sensors.md §9): BSMs from traffic/agents and SPaT from traffic/lights, laid out as
// the traffic domain lays them out, against closed forms: exact copies in ground truth, the
// signal heads found from the lamps, the delivery ratio's ½ at loss_distance, range, and nothing
// without the traffic domain.
#include "kinds_test_util.hpp"

#include <cmath>

namespace lagrange::sensors::kinds {
inline namespace extra2 {
double v2xDelivery(double distance, double d50, double width);
}  // namespace extra2
}  // namespace lagrange::sensors::kinds

namespace lagrange::sensors {
namespace {

using namespace test;

void kinematic(World& w, const std::string& name, std::array<double, 3> at) {
    Pose p;
    p.position = at;
    w.addBody(name, kWorldBody, p, 0.1, {}, {1, 1, 1, 0, 0, 0});
    w.bodies.kinematic.back() = true;
}

/// The receiver at the origin; a car and a pedestrian; signal heads of lanes 7 and 3 (and lamps
/// that are not red vehicle lamps, which do not count).
void scene(KindRun& r, std::map<std::string, double, std::less<>> settings = {}) {
    World& w = r.world;
    rig(w, "obu", "v2x", Pose{}, 10.0, std::move(settings));
    kinematic(w, "traffic_vehicle_0", {50, 0, 0});
    kinematic(w, "traffic_pedestrian_1", {0, 300, 0});
    kinematic(w, "traffic_lamp_7_red", {100, 5, 3.4});
    kinematic(w, "traffic_lamp_7_green", {100, 5, 3.0});
    kinematic(w, "traffic_lamp_3_red", {20, -5, 3.4});
    kinematic(w, "traffic_lamp_crossing_0_0_dont_walk", {10, 10, 2.5});
}

std::vector<Feed> traffic() {
    std::vector<Feed> f = bodyFeeds(Pose{}, {});
    // class, x, y, z, yaw, speed, length, width
    f.push_back(constant("traffic/agents", SignalType::ScalarArray, units::one,
                         {0, 50, 0, 0, 1.5, 8.0, 4.5, 1.8, 1, 0, 300, 0, -1.0, 1.34, 0.5, 0.5}));
    f.back().spec.arrayLength = 16;
    // per lane with a light, in lane order: road id, state (lane 3 green, lane 7 red)
    f.push_back(constant("traffic/lights", SignalType::ScalarArray, units::one, {12, 2, 31, 0}));
    f.back().spec.arrayLength = 4;
    return f;
}

TEST(V2x, IsBuiltin) {
    EXPECT_TRUE(isBuiltinKind("v2x"));
}

TEST(V2x, DeliveryCurve) {
    EXPECT_DOUBLE_EQ(kinds::v2xDelivery(300.0, 300.0, 50.0), 0.5);
    EXPECT_NEAR(kinds::v2xDelivery(100.0, 300.0, 50.0), 1.0 / (1.0 + std::exp(-4.0)), 1e-15);
}

TEST(V2x, GroundTruthCopiesTheMessages) {
    KindRun r;
    scene(r);
    ASSERT_TRUE(start(r, traffic(), true));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{500'000'000}).ok());
    const std::vector<double> bsm = r.readN("sensor/obu/bsm");
    ASSERT_EQ(bsm.size(), 20U);
    const std::vector<double> car{0, 1, 0, 50, 0, 1.5, 8.0, 4.5, 1.8, 0};
    for (std::size_t k = 0; k < car.size(); ++k) {
        EXPECT_DOUBLE_EQ(bsm[k], car[k]) << k;
    }
    EXPECT_EQ(bsm[10 + 1], 1.0);
    EXPECT_EQ(bsm[10 + 2], 1.0);  // a pedestrian
    const std::vector<double> spat = r.readN("sensor/obu/spat");
    ASSERT_EQ(spat.size(), 14U);
    const std::vector<double> lane3{0, 1, 12, 2, 20, -5, 0};
    const std::vector<double> lane7{1, 1, 31, 0, 100, 5, 0};
    for (std::size_t k = 0; k < 7; ++k) {
        EXPECT_DOUBLE_EQ(spat[k], lane3[k]) << k;
        EXPECT_DOUBLE_EQ(spat[7 + k], lane7[k]) << k;
    }
    EXPECT_EQ(r.read("sensor/obu/bsm_count"), 2.0);
    EXPECT_EQ(r.read("sensor/obu/spat_count"), 2.0);
    EXPECT_EQ(r.read("sensor/obu/valid"), 1.0);
}

TEST(V2x, HalfLostAtTheLossDistance) {
    KindRun r;
    scene(r, {{"timeout", 0.0}});
    ASSERT_TRUE(start(r, traffic(), false));
    int car = 0, pedestrian = 0;
    const int n = 2000;
    for (int k = 1; k <= n; ++k) {
        ASSERT_TRUE(r.scheduler->stepUntil(SimTime{k * 100'000'000LL}).ok());
        const std::vector<double> bsm = r.readN("sensor/obu/bsm");
        car += bsm[1] == 1.0 ? 1 : 0;
        pedestrian += bsm[11] == 1.0 ? 1 : 0;
    }
    // 50 m: 1/(1 + e^−5) = 0.993; 300 m: ½ (binomial σ = 1.1 % over 2000).
    EXPECT_NEAR(car / double(n), kinds::v2xDelivery(50.0, 300.0, 50.0), 0.01);
    EXPECT_NEAR(pedestrian / double(n), 0.5, 0.045);
}

TEST(V2x, BeyondRange) {
    KindRun r;
    scene(r, {{"range/max", 200.0}});
    ASSERT_TRUE(start(r, traffic(), true));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{500'000'000}).ok());
    const std::vector<double> bsm = r.readN("sensor/obu/bsm");
    EXPECT_EQ(bsm[1], 1.0);
    EXPECT_EQ(bsm[11], 0.0);  // the pedestrian, 300 m away
    EXPECT_TRUE(std::isnan(bsm[19]));
    EXPECT_EQ(r.read("sensor/obu/bsm_count"), 1.0);
}

TEST(V2x, NothingWithoutTraffic) {
    KindRun r;
    scene(r);
    ASSERT_TRUE(start(r, bodyFeeds(Pose{}, {}), true));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{500'000'000}).ok());
    EXPECT_EQ(r.read("sensor/obu/valid"), 0.0);
    EXPECT_EQ(r.read("sensor/obu/bsm_count"), 0.0);
    EXPECT_EQ(r.readN("sensor/obu/bsm")[1], 0.0);
}

TEST(V2x, RestoredRunRepeats) {
    // What it last heard and when is state: a restored run keeps its rows and ages.
    KindRun r;
    scene(r, {{"timeout", 0.5}});
    ASSERT_TRUE(start(r, traffic(), false));
    expectRestoredRunRepeats(r, {"sensor/obu/bsm", "sensor/obu/spat", "sensor/obu/bsm_count"},
                             2'000'000'000, 12'000'000'000, 100'000'000);
}

}  // namespace
}  // namespace lagrange::sensors
