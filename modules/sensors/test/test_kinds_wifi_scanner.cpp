// wifi_scanner (sensors.md §9): RSSI from the log-distance model in closed form, a wall's loss,
// the sensitivity cut, the comms domain's received power, the scan interval and ground truth.
#include "more_kinds_util.hpp"

#include <cmath>
#include <memory>
#include <numbers>

namespace lagrange::sensors::test {
namespace {

/// The model by hand: Friis to 1 m, then exponent 3, 20 dBm at 2.437 GHz, walls of 10 dB.
double expectedDbm(double distance, int walls = 0) {
    const double lambda = 299792458.0 / 2.437e9;
    const double fspl1 = 20.0 * std::log10(4.0 * std::numbers::pi * 1.0 / lambda);
    return 20.0 - fspl1 - 30.0 * std::log10(distance) - 10.0 * walls;
}

/// A scanner on a robot; APs at the origin and at
/// (0, 30, 1).
KindRun scannerRun(std::map<std::string, double, std::less<>> settings = {},
                   std::map<std::string, std::string, std::less<>> text = {}) {
    KindRun r;
    const BodyId robot = movingBody(r.world, "robot");
    if (!text.contains("access_points")) {
        text["access_points"] = "0 0 1; 0 30 1";
    }
    r.world.addCustomSensor("wifi", "wifi_scanner", robot, Pose{}, 10.0, std::move(settings),
                            std::move(text));
    return r;
}

/// The robot at x = `x(t)`, y = 0, z = 1.
std::unique_ptr<Writer> mover(std::function<double(double)> x = [](double) { return 10.0; }) {
    auto w = std::make_unique<Writer>();
    w->body("robot", [x](double t) { return std::array<double, 3>{x(t), 0.0, 1.0}; });
    return w;
}

TEST(WifiScanner, LogDistanceRssiRoundedToWholeDbm) {
    KindRun r = scannerRun({{"scan_time", 0.0}});
    ASSERT_TRUE(r.wire(mover()));
    ASSERT_TRUE(r.runTo(1.0));
    const auto rssi = r.readN("sensor/wifi/rssi");
    ASSERT_EQ(rssi.size(), 2U);
    EXPECT_EQ(rssi[0], std::round(expectedDbm(10.0)));
    EXPECT_EQ(rssi[0], -50.0);  // 20 − 40.2 − 30 dB
    EXPECT_EQ(rssi[1], std::round(expectedDbm(std::hypot(10.0, 30.0))));
    EXPECT_EQ(r.read("sensor/wifi/count"), 2.0);
    EXPECT_NEAR(r.readN("sensor/wifi/power")[0], 1e-3 * std::pow(10.0, -5.0), 1e-15);
}

TEST(WifiScanner, AWallCostsItsLoss) {
    KindRun r = scannerRun({{"scan_time", 0.0}});
    const BodyId wall = fixedBody(r.world, "wall", {5.0, 0.0, 1.0});
    r.world.addCollider(wall, ShapeType::Box, {0.1, 2.0, 2.0}, Pose{}, 0.8);
    ASSERT_TRUE(r.wire(mover()));
    ASSERT_TRUE(r.runTo(1.0));
    EXPECT_EQ(r.readN("sensor/wifi/rssi")[0], std::round(expectedDbm(10.0, 1)));
    EXPECT_EQ(r.readN("sensor/wifi/rssi")[1], std::round(expectedDbm(std::hypot(10.0, 30.0))));
}

TEST(WifiScanner, WeakerThanTheSensitivityIsNotListed) {
    // At 10 m the first AP reads −50 dBm; the second, 31.6 m away, −65 dBm.
    KindRun r = scannerRun({{"scan_time", 0.0}, {"sensitivity", -60.0}});
    ASSERT_TRUE(r.wire(mover()));
    ASSERT_TRUE(r.runTo(1.0));
    const auto rssi = r.readN("sensor/wifi/rssi");
    EXPECT_EQ(rssi[0], -50.0);
    EXPECT_TRUE(std::isnan(rssi[1]));
    EXPECT_EQ(r.read("sensor/wifi/count"), 1.0);
}

TEST(WifiScanner, ReadsTheCommsDomainsReceivedPower) {
    KindRun r;
    const BodyId robot = movingBody(r.world, "robot");
    const PrimId ap = r.world.addPrim("/access_point", "access_point");
    r.world.prims.poseInBody[ap.index].position = {0.0, 0.0, 1.0};
    const PrimId radio = r.world.addPrim("/robot/wifi", "wifi");
    r.world.prims.body[radio.index] = robot;
    SchemaInstance link;
    link.schema = "RadioLinkAPI_1";
    link.prim = radio;
    link.reals = {{"comms:frequency", 2.437e9}, {"comms:bandwidth", 2e7}, {"comms:txPower", 0.1}};
    link.relationships["comms:transmitter"] = {ap};
    r.world.schemaInstances.push_back(link);
    r.world.addCustomSensor("scan", "wifi_scanner", robot, Pose{}, 10.0, {{"scan_time", 0.0}}, {});
    auto w = mover([](double) { return 4.0; });
    // −40 dBm from the comms domain for 2 s, then its ideal substitute's 1 W sentinel.
    w->add("link/wifi/rx_power", SignalType::Scalar, units::W,
           [](double t) { return std::vector<double>{t < 2.0 ? 1e-7 : 1.0}; });
    ASSERT_TRUE(r.wire(std::move(w)));
    ASSERT_TRUE(r.runTo(1.0));
    EXPECT_EQ(r.readN("sensor/scan/rssi")[0], -40.0);
    // Ideal comms: the link's own attributes, exponent 2 by default: free space to 4 m.
    ASSERT_TRUE(r.runTo(3.0));
    const double lambda = 299792458.0 / 2.437e9;
    EXPECT_EQ(r.readN("sensor/scan/rssi")[0],
              std::round(20.0 - 20.0 * std::log10(4.0 * std::numbers::pi * 4.0 / lambda)));
}

TEST(WifiScanner, NewResultsOncePerScan) {
    KindRun r = scannerRun({{"scan_time", 2.0}});
    // At x = 1 until t = 1.5, then at x = 3.
    ASSERT_TRUE(r.wire(mover([](double t) { return t < 1.5 ? 1.0 : 3.0; })));
    ASSERT_TRUE(r.runTo(0.15));  // the first scan at the first step
    EXPECT_EQ(r.readN("sensor/wifi/rssi")[0], std::round(expectedDbm(1.0)));
    ASSERT_TRUE(r.runTo(1.9));  // moved, but no new scan yet
    EXPECT_EQ(r.readN("sensor/wifi/rssi")[0], std::round(expectedDbm(1.0)));
    ASSERT_TRUE(r.runTo(2.2));  // the second scan, 2 s after the first
    EXPECT_EQ(r.readN("sensor/wifi/rssi")[0], std::round(expectedDbm(3.0)));
}

TEST(WifiScanner, GroundTruthIsExact) {
    KindRun r = scannerRun({{"scan_time", 0.0}, {"noise", 4.0}});
    ASSERT_TRUE(r.wire(mover(), true));
    ASSERT_TRUE(r.runTo(1.0));
    EXPECT_NEAR(r.readN("sensor/wifi/rssi")[0], expectedDbm(10.0), 1e-9);
}

// QA 2026-11-12 #17: after a floating-origin rebase the access points given as text move with
// the region frame, as the robot's pose does.
TEST(WifiScanner, TextAccessPointsFollowAFloatingOriginRebase) {
    KindRun r = scannerRun({{"scan_time", 0.0}});
    auto shift = std::make_shared<double>(0.0);
    ASSERT_TRUE(r.wire(mover([shift](double) { return 10.0 - *shift; }), true));
    ASSERT_TRUE(r.runTo(0.5));
    const auto before = r.readN("sensor/wifi/rssi");
    *shift = 500.0;
    r.scheduler->shiftOrigin({500.0, 0.0, 0.0});
    ASSERT_TRUE(r.runTo(1.0));
    EXPECT_EQ(r.readN("sensor/wifi/rssi"), before);
}

}  // namespace
}  // namespace lagrange::sensors::test
