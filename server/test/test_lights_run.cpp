// RFC 0021 in a whole run: the sample AMR with a headlamp and a beacon (an overlay). The lamps
// draw their power from the 48 V bus (P = V·I of the load within 0.5 %), the headlamp's heat
// brings its thermal node to T_amb + P_heat/(h·A) (within 1 %), `--domain lights=off` is dark,
// and the same run twice has the same state. (ROS: test_ros_topics.)
#include "lagrange/server/Server.hpp"

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

constexpr double kAmbient = 293.15;     // K
constexpr double kHeadlampHa = 0.2;     // W/K: thermal:convectionCoeff 10 (fallback) × 0.02 m²
constexpr double kHeadlampFull = 1500;  // cd

ServerOptions litAmr() {
    ServerOptions o;
    o.robotPath = LAGRANGE_TEST_AMR_URDF;
    o.overlayPath = LAGRANGE_TEST_AMR_LIGHTS_OVERLAY;
    o.worldPath = "";
    o.ambientTemperatureK = kAmbient;
    o.demoDrive = false;  // at rest: natural convection only
    o.ros = false;
    o.realTimeFactor = 0.0;
    return o;
}

double at(const Server& server, const std::string& name) {
    return server.read(name).at(0);
}

TEST(LightsRun, TheLampsDrawFromTheBusAndHeatTheirNode) {
    Server server(litAmr());
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.stepUntil(SimTime{1'000'000'000}).ok());
    EXPECT_DOUBLE_EQ(at(server, "lamp/headlamp/intensity"), kHeadlampFull);
    EXPECT_EQ(at(server, "lamp/headlamp/on"), 1.0);
    for (const char* lamp : {"headlamp", "beacon"}) {
        const std::string load = std::string("load/") + lamp + "/";
        const double demand = at(server, load + "power_demand");
        const double drawn = at(server, load + "current") * at(server, "bus/dc_48v/voltage");
        ASSERT_GT(demand, 0.0) << lamp;
        EXPECT_NEAR(drawn / demand, 1.0, 0.005) << lamp << ": " << drawn << " W of " << demand;
    }
    // Five time constants of the node (2 J/K over 0.2 W/K: 10 s) and more.
    ASSERT_TRUE(server.stepUntil(SimTime{80'000'000'000}).ok());
    const double heat = at(server, "lamp/headlamp/loss_power");
    const double expected = kAmbient + heat / kHeadlampHa;
    const double temperature = at(server, "thermal/headlamp/temperature");
    EXPECT_NEAR((temperature - kAmbient) / (expected - kAmbient), 1.0, 0.01)
        << temperature << " K against " << expected << " K";
}

TEST(LightsRun, OffTheLampsAreDarkAndDrawWhatTheLoadSays) {
    ServerOptions o = litAmr();
    o.substituted.insert("lights");
    Server server(o);
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.stepUntil(SimTime{500'000'000}).ok());
    EXPECT_EQ(at(server, "lamp/headlamp/intensity"), 0.0);
    EXPECT_EQ(at(server, "lamp/beacon/on"), 0.0);
    EXPECT_EQ(at(server, "load/headlamp/power_demand"), 0.0);
}

TEST(LightsRun, TheSameRunTwiceHasTheSameState) {
    std::optional<std::uint64_t> first;
    for (int trial = 0; trial < 2; ++trial) {
        Server server(litAmr());
        ASSERT_TRUE(server.load().ok());
        ASSERT_TRUE(server.stepUntil(SimTime{2'000'000'000}).ok());
        auto hash = server.stateHash();
        ASSERT_TRUE(hash);
        if (first) {
            EXPECT_EQ(*hash, *first);
        }
        first = *hash;
    }
}

}  // namespace
}  // namespace lagrange::server
