// RFC 0011 in a whole run: the sample quadcopter hovers on a tether from --tether. The cable
// hangs between the anchor and the drone; its two ends hold its weight up between them, and the
// drone feels the pull at its end. With `--domain tether=off` there is no cable and no pull.
#include "lagrange/server/Server.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

constexpr double kG = 9.80665;
constexpr double kDensity = 0.02;  // kg/m, TetherAPI_1's fallback

ServerOptions tethered() {
    ServerOptions o;
    o.robotPath = LAGRANGE_TEST_QUAD_USDA;
    o.worldPath = "";
    o.airDensityKgM3 = 1.225;
    o.ros = false;
    o.realTimeFactor = 0.0;
    // A ground station 2 m off, 1 m up; the drone takes off from the origin to hover 2 m up.
    o.tethers.push_back({{2.0, 0.0, 1.0}, "base_link", 3.0});
    return o;
}

TEST(TetherRun, AQuadcopterHoversOnItsTether) {
    Server server(tethered());
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    const double start = server.read("body/base_link/pose").at(2);
    ASSERT_TRUE(server.stepUntil(SimTime{12'000'000'000}).ok());
    // Averaged over 2 s of hovering.
    double up = 0.0;
    double weight = 0.0;
    double pull = 0.0;
    double tension = 0.0;
    double height = 0.0;
    double contacts = 0.0;
    constexpr int kSamples = 200;
    for (int k = 1; k <= kSamples; ++k) {
        ASSERT_TRUE(server.stepUntil(SimTime{12'000'000'000 + k * 10'000'000LL}).ok());
        const std::vector<double> nodes = server.read("tether/tether_0/nodes");
        std::size_t count = 0;
        while (3 * count + 2 < nodes.size() && std::isfinite(nodes[3 * count])) {
            ++count;
        }
        ASSERT_GE(count, 3U);
        const auto at = [&](std::size_t i) {
            return std::array<double, 3>{nodes[3 * i], nodes[3 * i + 1], nodes[3 * i + 2]};
        };
        const auto rise = [](const std::array<double, 3>& from, const std::array<double, 3>& to) {
            return (to[2] - from[2]) /
                   std::hypot(to[0] - from[0], to[1] - from[1], to[2] - from[2]);
        };
        // The station holds the cable up by its tension times the slope there, and the half
        // segment on it; the drone holds the rest, which is the pull it feels.
        const double anchor = server.read("tether/tether_0/tension_anchor").at(0);
        const double attach = server.read("tether/tether_0/tension_attach").at(0);
        const double piece =
            server.read("tether/tether_0/length").at(0) / static_cast<double>(count - 1);
        const std::vector<double> w = server.read("body/base_link/wrench/tether");
        up += (anchor * rise(at(1), at(0)) + 0.5 * piece * kDensity * kG - w.at(2)) / kSamples;
        weight += kDensity * server.read("tether/tether_0/length").at(0) * kG / kSamples;
        pull += std::hypot(w.at(0), w.at(1), w.at(2)) / kSamples;
        tension += attach / kSamples;
        height += (server.read("body/base_link/pose").at(2) - start) / kSamples;
        contacts += server.read("tether/tether_0/contacts").at(0) / kSamples;
    }
    std::printf(
        "tethered hover: %.3f m up; ends hold %.4f N of %.4f N; pull on the drone "
        "%.4f N, tension there %.4f N; contacts %.1f\n",
        height, up, weight, pull, tension, contacts);
    EXPECT_NEAR(height, 2.0, 0.1);
    EXPECT_EQ(contacts, 0.0);  // hanging free
    EXPECT_NEAR(up, weight, 0.02 * weight);
    EXPECT_NEAR(pull, tension, 0.15 * tension);  // the tension, plus the end's half segment
    EXPECT_GT(tension, 0.1);
}

TEST(TetherRun, OffThereIsNoCableAndNoPull) {
    ServerOptions o = tethered();
    o.substituted.insert("tether");
    Server server(o);
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.stepUntil(SimTime{3'000'000'000}).ok());
    EXPECT_EQ(server.read("tether/tether_0/tension_attach").at(0), 0.0);
    EXPECT_TRUE(server.read("tether/tether_0/nodes").empty());
    EXPECT_TRUE(server.read("body/base_link/wrench/tether").empty());
}

// From an overlay on a URDF robot: `tether:attach: base_link` names the link's own prim, which
// must resolve to the link's body (the cable's far end at the drone, not at the world origin).
TEST(TetherRun, AnOverlayTiesItToAUrdfLink) {
    // Only the tether: the drone sits where it starts, the cable from the station to it.
    std::stringstream text;
    text << R"(schema_version: 1
apply:
  - create: tether/station
    schemas:
      TetherAPI_1:
        tether:anchorPoint: [2.0, 0.0, 1.0]
        tether:attach: base_link
        tether:attachPoint: [0.0, 0.0, -0.05]
        tether:length: 3.0
)";
    const std::filesystem::path overlay =
        std::filesystem::temp_directory_path() /
        ("lagrange_tether_" + std::to_string(::getpid()) + ".overlay.yaml");
    std::ofstream(overlay) << text.str();
    ServerOptions o;
    o.robotPath = LAGRANGE_TEST_QUAD_URDF;
    o.overlayPath = overlay.string();
    o.airDensityKgM3 = 1.225;
    o.ros = false;
    o.realTimeFactor = 0.0;
    Server server(o);
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.stepUntil(SimTime{500'000'000}).ok());
    const std::vector<double> drone = server.read("body/base_link/pose");
    const std::vector<double> nodes = server.read("tether/station/nodes");
    std::size_t count = 0;
    while (3 * count + 2 < nodes.size() && std::isfinite(nodes[3 * count])) {
        ++count;
    }
    ASSERT_GE(count, 2U);
    const double* end = &nodes[3 * (count - 1)];
    EXPECT_NEAR(end[0], drone.at(0), 0.02);
    EXPECT_NEAR(end[1], drone.at(1), 0.02);
    EXPECT_NEAR(end[2], drone.at(2) - 0.05, 0.02);
    EXPECT_NEAR(nodes[0], 2.0, 1e-9);  // the anchor, a world point
    EXPECT_NEAR(nodes[2], 1.0, 1e-9);
    std::filesystem::remove(overlay);
}

}  // namespace
}  // namespace lagrange::server
