// Kind "adsb_receiver" (sensors.md §9): other robots heard by their bodies' signals, against
// closed forms: which robots are aircraft, their positions (local and geodetic), range and the
// radio horizon, DO-260B's squitter rate and the message's quantisation.
#include "kinds_test_util.hpp"
#include "lagrange/world/coords/Frames.hpp"

#include <cmath>
#include <set>

namespace lagrange::sensors::kinds {
inline namespace extra2 {
double radioHorizon(double h1, double h2);
}  // namespace extra2
}  // namespace lagrange::sensors::kinds

namespace lagrange::sensors {
namespace {

using namespace test;
constexpr std::size_t kStride = 10;

/// The receiver on robot 0 ("frame" at the origin), a quadcopter (robot 1, a RotorAPI_1 joint)
/// and a car (robot 2, no rotors).
void scene(KindRun& r, std::map<std::string, double, std::less<>> settings = {},
           std::map<std::string, std::string, std::less<>> text = {}) {
    World& w = r.world;
    rig(w, "adsb", "adsb_receiver", Pose{}, 100.0, std::move(settings), std::move(text));
    w.robots = {"station", "quad", "car"};
    const BodyId drone =
        w.addBody("quad_base", kWorldBody, Pose{}, 1.0, {}, {0.1, 0.1, 0.1, 0, 0, 0});
    w.addJoint("quad_free", JointType::Free, drone, {0, 0, 1}, false);
    w.bodies.robot.back() = 1;
    const BodyId prop =
        w.addBody("quad_prop", drone, Pose{}, 0.01, {}, {1e-4, 1e-4, 1e-4, 0, 0, 0});
    w.addJoint("rotor_0", JointType::Revolute, prop, {0, 0, 1}, true);
    w.bodies.robot.back() = 1;
    SchemaInstance rotor;
    rotor.schema = "RotorAPI_1";
    rotor.prim = w.joints.prim.back();
    w.schemaInstances.push_back(rotor);
    const BodyId car = w.addBody("car_base", kWorldBody, Pose{}, 1.0, {}, {0.1, 0.1, 0.1, 0, 0, 0});
    w.addJoint("car_free", JointType::Free, car, {0, 0, 1}, false);
    w.bodies.robot.back() = 2;
}

/// Feeds: the receiver still at the origin, the quad at `quad` moving at `v`, the car still.
std::vector<Feed> feeds(std::array<double, 3> quad, std::array<double, 3> v) {
    std::vector<Feed> f = bodyFeeds(Pose{}, {});
    f.push_back({makeSpec("body/quad_base/pose", SignalType::Pose, units::m, "world_local"),
                 [quad, v](double t) {
                     return std::vector<double>{
                         quad[0] + v[0] * t, quad[1] + v[1] * t, quad[2] + v[2] * t, 1, 0, 0, 0};
                 }});
    f.push_back(constant("body/quad_base/twist", SignalType::Wrench, units::m_per_s,
                         {v[0], v[1], v[2], 0, 0, 0}, "world_local"));
    f.push_back(constant("body/car_base/pose", SignalType::Pose, units::m, {30, 40, 0, 1, 0, 0, 0},
                         "world_local"));
    f.push_back(constant("body/car_base/twist", SignalType::Wrench, units::m_per_s,
                         {0, 0, 0, 0, 0, 0}, "world_local"));
    return f;
}

TEST(AdsbReceiver, IsBuiltin) {
    EXPECT_TRUE(isBuiltinKind("adsb_receiver"));
}

TEST(AdsbReceiver, RadioHorizon) {
    // The 4/3 Earth rule of thumb: d ≈ 4.12·√h km, h in metres.
    EXPECT_NEAR(kinds::radioHorizon(1.0, 0.0), 4121.6, 0.5);
    EXPECT_NEAR(kinds::radioHorizon(100.0, 100.0), 2.0 * 41216.0, 5.0);
}

TEST(AdsbReceiver, HearsTheQuadExactly) {
    KindRun r;
    scene(r);
    ASSERT_TRUE(start(r, feeds({300, -400, 120}, {5, 0, 1}), true));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{1'000'000'000}).ok());
    const std::vector<double> a = r.readN("sensor/adsb/aircraft");
    ASSERT_EQ(a.size(), kStride);  // the car has no rotors
    EXPECT_EQ(a[0], 1.0);          // its robot index
    EXPECT_EQ(a[1], 1.0);
    EXPECT_NEAR(a[2], 305.0, 1e-9);
    EXPECT_NEAR(a[3], -400.0, 1e-9);
    EXPECT_NEAR(a[4], 121.0, 1e-9);
    EXPECT_NEAR(a[5], 5.0, 1e-12);
    EXPECT_NEAR(a[7], 1.0, 1e-12);
    EXPECT_NEAR(a[9], std::hypot(305.0, 400.0, 121.0), 1e-9);
    EXPECT_EQ(r.read("sensor/adsb/count"), 1.0);
    EXPECT_EQ(r.read("sensor/adsb/geodetic"), 0.0);
}

TEST(AdsbReceiver, EveryOtherRobotWhenAsked) {
    KindRun r;
    scene(r, {}, {{"aircraft", "all"}});
    ASSERT_TRUE(start(r, feeds({300, -400, 120}, {}), true));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{100'000'000}).ok());
    const std::vector<double> a = r.readN("sensor/adsb/aircraft");
    ASSERT_EQ(a.size(), 2 * kStride);
    EXPECT_EQ(a[kStride], 2.0);
    EXPECT_NEAR(a[kStride + 9], 50.0, 1e-9);  // the car at (30, 40)
}

TEST(AdsbReceiver, GeodeticWithTheSiteOrigin) {
    KindRun r;
    scene(r);
    r.world.geoOrigin = coords::Geodetic{0.8278, 0.1492, 488.0};
    ASSERT_TRUE(start(r, feeds({300, -400, 120}, {}), true));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{100'000'000}).ok());
    const std::vector<double> a = r.readN("sensor/adsb/aircraft");
    const coords::Geodetic g =
        coords::GeoOrigin(*r.world.geoOrigin).toGeodetic({300.0, -400.0, 120.0});
    EXPECT_NEAR(a[2], g.latitudeRad, 1e-12);
    EXPECT_NEAR(a[3], g.longitudeRad, 1e-12);
    EXPECT_NEAR(a[4], g.heightM, 1e-6);
    EXPECT_EQ(r.read("sensor/adsb/geodetic"), 1.0);
}

TEST(AdsbReceiver, RangeAndHorizon) {
    // Beyond range/max; then within range but below the horizon: 100 m up and 50 km away is
    // past √(2·(4/3)·R·100 m) + √(2·(4/3)·R·2 m) = 47.0 km for a receiver's antenna 2 m up.
    for (const auto& [quad, range, heard] :
         {std::tuple{std::array<double, 3>{2000, 0, 100}, 1000.0, false},
          std::tuple{std::array<double, 3>{30e3, 0, 100}, 200e3, true},
          std::tuple{std::array<double, 3>{50e3, 0, 100}, 200e3, false}}) {
        KindRun r;
        scene(r, {{"range/max", range}});
        ASSERT_TRUE(start(r, feeds(quad, {}), true));
        ASSERT_TRUE(r.scheduler->stepUntil(SimTime{100'000'000}).ok());
        const std::vector<double> a = r.readN("sensor/adsb/aircraft");
        EXPECT_EQ(a[1], heard ? 1.0 : 0.0) << quad[0];
        EXPECT_EQ(std::isnan(a[2]), !heard);
    }
}

TEST(AdsbReceiver, TwiceASecondQuantised) {
    KindRun r;
    scene(r);
    ASSERT_TRUE(start(r, feeds({300, -400, 120}, {10, 3, 0.5}), false));
    int updates = 0;
    double last = std::nan("");
    double maxAge = 0.0;
    for (int k = 1; k <= 1000; ++k) {  // 10 s at 100 Hz
        ASSERT_TRUE(r.scheduler->stepUntil(SimTime{k * 10'000'000LL}).ok());
        const std::vector<double> a = r.readN("sensor/adsb/aircraft");
        if (a[1] != 1.0) {
            continue;
        }
        maxAge = std::max(maxAge, a[8]);
        if (a[2] != last) {
            ++updates;
            last = a[2];
            EXPECT_NEAR(std::remainder(a[4], 25.0 * 0.3048), 0.0, 1e-9);    // 25 ft
            EXPECT_NEAR(std::remainder(a[5], 1852.0 / 3600.0), 0.0, 1e-9);  // 1 kt
            EXPECT_NEAR(a[5], 10.0, 0.26);
            EXPECT_NEAR(a[4], 120.0 + 0.5 * k * 0.01, 15.0);
        }
    }
    // 0.4 to 0.6 s apart: 17 to 25 in 10 s.
    EXPECT_GE(updates, 16);
    EXPECT_LE(updates, 26);
    EXPECT_LE(maxAge, 0.6 + 0.01 + 1e-9);  // one step late at most
}

TEST(AdsbReceiver, RestoredRunRepeats) {
    // Its squitter clocks and last messages are state: a restored run hears the same messages at
    // the same moments (no negative age, no stale row).
    KindRun r;
    scene(r);
    ASSERT_TRUE(start(r, feeds({300, -400, 120}, {5, 0, 1}), false));
    expectRestoredRunRepeats(r, {"sensor/adsb/aircraft", "sensor/adsb/count"}, 1'000'000'000,
                             4'000'000'000, 10'000'000);
}

}  // namespace
}  // namespace lagrange::sensors
