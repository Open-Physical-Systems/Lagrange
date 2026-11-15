// RFC 0011 in a whole run: the sample quadcopter hovers on a tether from --tether. The cable
// hangs between the anchor and the drone; its two ends hold its weight up between them, and the
// drone feels the pull at its end. With `--domain tether=off` there is no cable and no pull.
#include "lagrange/server/Server.hpp"

#include <array>
#include <cmath>
#include <cstdint>
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

// A mast 4.2 m off along the drone's front-left diagonal, about as high as the drone hovers: the
// cable leaves the frame sloping down a little and passes under rotor 3, across its near wake.
// Long enough to let the drone take off from the ground beside the mast's foot.
constexpr std::array<double, 3> kWakeAnchor{3.0, 3.0, 2.0};
constexpr double kWakeLength = 4.75;  // m

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

// The rotors' downwash on the cable (RFC 0011 step 6). The tether runs from the hovering drone to
// a mast, under rotor 3, across its near wake. Over 2 s of hover:
// - the ends hold the cable's weight plus the downwash's drag, which is what the drag law gives
//   for the downwash the aerodynamics domain writes at the nodes (tether/<n>/downwash);
// - that drag is the closed form for a cylinder across the actuator disc's slipstream: the wake
//   speed w = v_i·(1 + s/√(s² + R²)) at s below the disc, v_i = √(T/2ρπR²) from the run's own
//   rotor thrust, over its width 2·r_w, r_w = R·√(v_i/w): F = ½·ρ·Cd·d·w²·2r_w·cos²θ for a
//   cable sloping at θ;
// - with the aerodynamics domain off (no downwash) the ends hold only the weight, as before.
struct WakeLoad {
    double extra{0.0};         // N, held up by the ends beyond the weight
    double predicted{0.0};     // N, Σ ½ρCd·d·L·|u_n|·u_n (down) from the published downwash
    double inWake{0.0};        // m of cable where the downwash is over half U_H
    double peak{0.0};          // m/s, the strongest downwash on the cable
    double hoverInduced{0.0};  // U_H, m/s, from the rotors' thrust
    double closed{0.0};        // N, the slipstream closed form at rotor 3
    double below{0.0};         // m, s: how far under rotor 3's disc the cable crosses its axis
    double height{0.0};
    double contacts{0.0};
    bool hasDownwash{false};
};

WakeLoad hoverInWake(bool aero) {
    // From an overlay, for masses 5 cm apart: rotor 3's slipstream is 18 cm across there.
    std::stringstream text;
    text << "schema_version: 1\napply:\n  - create: tether/tether_0\n    schemas:\n"
         << "      TetherAPI_1:\n        tether:anchorPoint: [" << kWakeAnchor[0] << ", "
         << kWakeAnchor[1] << ", " << kWakeAnchor[2] << "]\n"
         << "        tether:attach: base_link\n        tether:length: " << kWakeLength << "\n"
         << "        tether:segmentLength: 0.05\n";
    const std::filesystem::path overlay =
        std::filesystem::temp_directory_path() /
        ("lagrange_tether_wake_" + std::to_string(::getpid()) + ".overlay.yaml");
    std::ofstream(overlay) << text.str();
    ServerOptions o = tethered();
    o.tethers.clear();
    o.overlayPath = overlay.string();
    if (!aero) {
        o.substituted.insert("aerodynamics");
    }
    Server server(o);
    const Status loaded = server.load();
    EXPECT_TRUE(loaded.ok()) << loaded.message;
    WakeLoad out;
    std::filesystem::remove(overlay);
    if (!loaded.ok() || !server.stepUntil(SimTime{12'000'000'000}).ok()) {
        return out;
    }
    constexpr double kRho = 1.225;
    constexpr double kCd = 1.2;        // TetherAPI_1's fallbacks
    constexpr double kD = 0.005;       // m
    constexpr double kRadius = 0.127;  // m, the sample's rotors
    const double start = 0.0;
    constexpr int kSamples = 200;
    for (int k = 1; k <= kSamples; ++k) {
        if (!server.stepUntil(SimTime{12'000'000'000 + k * 10'000'000LL}).ok()) {
            ADD_FAILURE();
            return out;
        }
        const std::vector<double> nodes = server.read("tether/tether_0/nodes");
        const std::vector<double> wake = server.read("tether/tether_0/downwash");
        out.hasDownwash = !wake.empty();
        std::size_t count = 0;
        while (3 * count + 2 < nodes.size() && std::isfinite(nodes[3 * count])) {
            ++count;
        }
        if (count < 3) {
            ADD_FAILURE() << "no cable";
            return out;
        }
        const auto at = [&](const std::vector<double>& v, std::size_t i) {
            return std::array<double, 3>{v[3 * i], v[3 * i + 1], v[3 * i + 2]};
        };
        const double length = server.read("tether/tether_0/length").at(0);
        const double piece = length / static_cast<double>(count - 1);
        const double anchor = server.read("tether/tether_0/tension_anchor").at(0);
        const std::array<double, 3> a0 = at(nodes, 0);
        const std::array<double, 3> a1 = at(nodes, 1);
        const double rise =
            (a0[2] - a1[2]) / std::hypot(a0[0] - a1[0], a0[1] - a1[1], a0[2] - a1[2]);
        const std::vector<double> w = server.read("body/base_link/wrench/tether");
        const double up = anchor * rise + 0.5 * piece * kDensity * kG - w.at(2);
        out.extra += (up - kDensity * length * kG) / kSamples;
        double thrust = 0.0;
        for (int r = 1; r <= 4; ++r) {
            thrust += server.read("rotor/rotor_" + std::to_string(r) + "_joint/thrust").at(0);
        }
        const double uh =
            std::sqrt(thrust / 4.0 / (2.0 * kRho * std::numbers::pi * kRadius * kRadius));
        out.hoverInduced += uh / kSamples;
        out.height += (server.read("body/base_link/pose").at(2) - start) / kSamples;
        out.contacts += server.read("tether/tether_0/contacts").at(0) / kSamples;
        // Where the cable passes under rotor 3's hub: the point of it nearest the axis.
        const double thrust3 = server.read("rotor/rotor_3_joint/thrust").at(0);
        const double vi = std::sqrt(thrust3 / (2.0 * kRho * std::numbers::pi * kRadius * kRadius));
        const std::vector<double> hub = server.read("body/rotor_3_link/pose");
        double nearest = 1e9;
        double s = 0.0;
        double level = 1.0;  // cos θ of the cable there, θ its slope
        for (std::size_t i = 0; i + 1 < count; ++i) {
            const auto p = at(nodes, i);
            const auto q = at(nodes, i + 1);
            const double r = std::hypot(p[0] - hub.at(0), p[1] - hub.at(1));
            if (r < nearest) {
                nearest = r;
                s = hub.at(2) - p[2];
                level = std::hypot(q[0] - p[0], q[1] - p[1]) /
                        std::hypot(q[0] - p[0], q[1] - p[1], q[2] - p[2]);
            }
        }
        const double ws = vi * (1.0 + s / std::sqrt(s * s + kRadius * kRadius));
        const double rw = kRadius * std::sqrt(vi / ws);
        // Across the column at a slope θ: the normal speed w·cos θ over 2r_w / cos θ of cable,
        // and the vertical share cos θ of that push.
        out.closed += 0.5 * kRho * kCd * kD * ws * ws * 2.0 * rw * level * level / kSamples;
        out.below += s / kSamples;
        if (wake.size() < 3 * count) {
            continue;
        }
        double inWake = 0.0;
        for (std::size_t j = 0; j + 1 < count; ++j) {
            const auto p = at(nodes, j);
            const auto q = at(nodes, j + 1);
            const std::array<double, 3> d{q[0] - p[0], q[1] - p[1], q[2] - p[2]};
            const double l = std::hypot(d[0], d[1], d[2]);
            // Each half of the segment in its own mass's air, as the cable takes it.
            for (const std::size_t m : {j, j + 1}) {
                const auto u = at(wake, m);
                const double along = (u[0] * d[0] + u[1] * d[1] + u[2] * d[2]) / l;
                const std::array<double, 3> n{u[0] - along * d[0] / l, u[1] - along * d[1] / l,
                                              u[2] - along * d[2] / l};
                const double un = std::hypot(n[0], n[1], n[2]);
                const double tangential =
                    0.5 * kRho * kCd / 50.0 * kD * 0.5 * l * std::abs(along) * along * d[2] / l;
                out.predicted +=
                    -(0.5 * kRho * kCd * kD * 0.5 * l * un * n[2] + tangential) / kSamples;
            }
            const auto u = at(wake, j);
            const auto v = at(wake, j + 1);
            const std::array<double, 3> air{0.5 * (u[0] + v[0]), 0.5 * (u[1] + v[1]),
                                            0.5 * (u[2] + v[2])};
            const double speed = std::hypot(air[0], air[1], air[2]);
            out.peak = std::max(out.peak, speed);
            if (speed > 0.5 * uh) {
                inWake += l;
            }
        }
        out.inWake += inWake / kSamples;
    }
    return out;
}

TEST(TetherRun, TheDronesDownwashPushesItsTetherDown) {
    const WakeLoad on = hoverInWake(true);
    const WakeLoad off = hoverInWake(false);
    std::printf(
        "downwash on the tether: U_H %.2f m/s, peak on the cable %.2f m/s, %.2f m in the wake, "
        "%.3f m under rotor 3; ends hold %.4f N over the weight; the drag law on the published "
        "downwash %.4f N; slipstream closed form %.4f N; aerodynamics off: %.4f N over the "
        "weight; %.2f m up, %.1f contacts\n",
        on.hoverInduced, on.peak, on.inWake, on.below, on.extra, on.predicted, on.closed, off.extra,
        on.height, on.contacts);
    EXPECT_TRUE(on.hasDownwash);
    EXPECT_FALSE(off.hasDownwash);
    EXPECT_NEAR(on.height, 2.0, 0.15);
    EXPECT_GT(on.below, 0.0);  // under the disc
    // Pushed down, by what the drag law gives for the air the aerodynamics domain wrote, and
    // that air is the actuator disc's slipstream.
    EXPECT_GT(on.extra, 0.0);
    EXPECT_NEAR(on.extra, on.predicted, 0.1 * on.predicted);
    EXPECT_NEAR(on.extra, on.closed, 0.2 * on.closed);
    EXPECT_GT(on.peak, on.hoverInduced);
    EXPECT_LT(on.peak, 2.0 * on.hoverInduced);
    // Without the downwash the ends hold the weight alone.
    EXPECT_NEAR(off.extra, 0.0, 0.02 * kDensity * kWakeLength * kG);
}

// The cable in the drone's wake is part of the CPU reference (invariant 7): two runs, the same
// state every 100 ms.
TEST(TetherRun, InTheWakeTheSameRunGivesTheSameState) {
    const auto hashes = [] {
        ServerOptions o = tethered();
        o.tethers.clear();
        o.tethers.push_back({{1.0, 1.0, 1.5}, "base_link", 2.5});
        Server server(o);
        EXPECT_TRUE(server.load().ok());
        std::vector<std::uint64_t> out;
        for (int k = 1; k <= 40; ++k) {
            EXPECT_TRUE(server.stepUntil(SimTime{k * 100'000'000LL}).ok());
            out.push_back(server.stateHash().value());
        }
        EXPECT_FALSE(server.read("tether/tether_0/downwash").empty());
        return out;
    };
    EXPECT_EQ(hashes(), hashes());
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
