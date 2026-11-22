// Trees (docs/physics/ground.md "Trees") in a whole run, on MuJoCo: the sample quadcopter takes
// off from under the crown of the open field's tree_1 (TreeAPI_1, crown from 2.5 m to 6.1 m) and
// hovers 2 m up, its rotors in the crown: they cut the leaves they spin through (the cut volume and
// leaves grow, the rotors pay for it), and the rotors' air reaches the crown (tree/<n>/downwash).
// The leaves they cut off fly (vegetation/leaves), in the rotors' wake, and settle on the
// ground. The same run twice is bit for bit.
#include "lagrange/server/Server.hpp"
#include "lagrange/world/Leaves.hpp"

#include <cmath>
#include <cstdio>
#include <numbers>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

ServerOptions underTheTree() {
    ServerOptions o;
    o.robotPath = LAGRANGE_TEST_QUAD_USDA;
    o.worldPath = LAGRANGE_TEST_OPEN_FIELD;
    o.airDensityKgM3 = 1.225;
    o.ros = false;
    o.realTimeFactor = 0.0;
    o.spawn.position = {19.0, 8.0, 1.2};  // a metre from tree_1's trunk, under its crown
    return o;
}

double at(Server& server, const std::string& signal) {
    return server.read(signal).at(0);
}

TEST(TreesRun, RotorsInTheCrownCutItsLeaves) {
    Server server(underTheTree());
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    double power = 0.0;
    double downwash = 0.0;
    for (int k = 0; k < 80; ++k) {  // 8 s
        ASSERT_TRUE(server.stepUntil(SimTime{server.simTime().ns + 100'000'000}).ok());
        for (const char* rotor :
             {"rotor_1_joint", "rotor_2_joint", "rotor_3_joint", "rotor_4_joint"}) {
            power = std::max(power, at(server, std::string("rotor/") + rotor + "/cutting_power"));
        }
        const std::vector<double> w = server.read("tree/tree_1/downwash");
        downwash = std::max(downwash, std::hypot(w[0], w[1], w[2]));
    }
    const double cut = at(server, "tree/tree_1/cut_volume");
    const double leaves = at(server, "tree/tree_1/leaves_shed");
    const std::vector<double> sway = server.read("tree/tree_1/sway");
    std::printf(
        "tree_1: %.4f m³ cut, %.0f leaves shed; most cutting power %.1f W; rotors' air at "
        "the crown up to %.2f m/s; sway %.4f m\n",
        cut, leaves, power, downwash, std::hypot(sway[0], sway[1]));
    EXPECT_GT(cut, 0.0) << "the rotors cut into the crown";
    EXPECT_NEAR(leaves, std::floor(cut * 1.0 / 0.003), 1.0);  // LAD 1 m²/m³, 30 cm² leaves
    EXPECT_GT(power, 0.0) << "and paid for it";
    // The rotors are inside the crown, under its centre where the air is sampled: what reaches
    // it there is their inflow, small, not their wake.
    EXPECT_GT(downwash, 0.0) << "their air reaches the crown";
    // The other trees, far away, are untouched.
    EXPECT_EQ(at(server, "tree/tree_2/cut_volume"), 0.0);
}

TEST(TreesRun, TheCutLeavesFlyAndSettle) {
    Server server(underTheTree());
    ASSERT_TRUE(server.load().ok());
    double most = 0.0;  // the most in the air at once
    double version = 0.0;
    std::size_t count = 0;
    double highest = 0.0;
    for (int k = 0; k < 120; ++k) {  // 12 s
        ASSERT_TRUE(server.stepUntil(SimTime{server.simTime().ns + 100'000'000}).ok());
        most = std::max(most, at(server, "tree/tree_1/leaves_flying"));
        const std::vector<double> leaves = server.read(std::string(kLeavesSignal));
        ASSERT_EQ(leaves.size(), kLeavesLength);
        EXPECT_GE(leaves[0], version) << "the version only grows";
        version = leaves[0];
        count = static_cast<std::size_t>(leaves[1]);
        for (std::size_t n = 0; n < count; ++n) {
            highest = std::max(highest, leaves[kLeavesHeader + (kLeavesFields * n) + 2]);
        }
    }
    const std::vector<double> leaves = server.read(std::string(kLeavesSignal));
    std::size_t grounded = 0;
    std::size_t flat = 0;
    for (std::size_t n = 0; n < count; ++n) {
        const std::size_t base = kLeavesHeader + (kLeavesFields * n);
        if (leaves[base + 2] < 0.01) {
            ++grounded;
            flat += std::abs(leaves[base + 4] - (std::numbers::pi / 2.0)) < 1e-12 ? 1U : 0U;
        }
        EXPECT_GT(leaves[base + 5], 0.0) << "every leaf has a size";
    }
    const double shed = at(server, "tree/tree_1/leaves_shed");
    std::printf(
        "tree_1: %.0f leaves cut off, %zu particles (%zu on the ground, %zu flat), up to %.0f in "
        "the air at once, highest %.2f m\n",
        shed, count, grounded, flat, most, highest);
    EXPECT_GT(count, 0U) << "the cut leaves fly";
    EXPECT_NEAR(static_cast<double>(count), shed, std::max(2.0, 0.05 * shed))
        << "one particle per leaf cut off (the air at the crown is under the stripping speed)";
    EXPECT_GT(most, 0.0) << "some were in the air";
    EXPECT_GT(grounded, 0U) << "and some reached the ground";
    EXPECT_EQ(flat, grounded) << "lying flat";
    EXPECT_GT(highest, 2.0) << "they start in the crown, over 2.5 m";
}

TEST(TreesRun, SameRunTwiceIsBitForBit) {
    std::vector<double> traces[2];
    for (auto& trace : traces) {
        Server server(underTheTree());
        ASSERT_TRUE(server.load().ok());
        for (int k = 0; k < 30; ++k) {
            ASSERT_TRUE(server.stepUntil(SimTime{server.simTime().ns + 100'000'000}).ok());
            trace.push_back(at(server, "tree/tree_1/cut_volume"));
            for (const double v : server.read("tree/tree_1/sway")) {
                trace.push_back(v);
            }
            const std::vector<double> leaves = server.read(std::string(kLeavesSignal));
            const auto used =
                kLeavesHeader + (kLeavesFields * static_cast<std::size_t>(leaves.at(1)));
            trace.insert(trace.end(), leaves.begin(),
                         leaves.begin() + static_cast<std::ptrdiff_t>(used));
        }
    }
    EXPECT_EQ(traces[0], traces[1]);
}

// A steady 12 m/s wind bends tree_1's crown by F/(m ω²): its sway mode (0.5 Hz, ζ 0.1, 2 kg per
// m³ of crown) under ½ ρ Cd A U², A = π R c with R = c = 1.8 m, Cd 0.3. The quadcopter sits on
// the ground far from the tree (its rotors stopped), so the aerodynamics domain runs and blows.
TEST(TreesRun, TheWindBendsTheCrown) {
    ServerOptions o = underTheTree();
    o.spawn.position = {0.0, 0.0, 0.0};
    o.site["wind_speed"] = 12.0;
    o.site["wind_from"] = 270.0;  // from the west: blowing towards +x
    Server server(o);
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.stepUntil(SimTime{20'000'000'000}).ok());
    // Gusts ride on the mean wind: average the pressure and the crown's lean over 10 s.
    double speedSquared = 0.0;
    double lean = 0.0;
    constexpr int kSamples = 100;
    for (int k = 0; k < kSamples; ++k) {
        ASSERT_TRUE(server.stepUntil(SimTime{server.simTime().ns + 100'000'000}).ok());
        const std::vector<double> wind = server.read("env/wind/velocity");
        speedSquared += (wind[0] * wind[0] + wind[1] * wind[1]) / kSamples;
        lean += server.read("tree/tree_1/sway").at(0) / kSamples;
    }
    constexpr double kPi = 3.14159265358979323846;
    const double area = kPi * 1.8 * 1.8;
    const double mass = 2.0 * 4.0 / 3.0 * kPi * 1.8 * 1.8 * 1.8;
    const double omega = 2.0 * kPi * 0.5;
    const double expected = 0.5 * 1.225 * 0.3 * area * speedSquared / (mass * omega * omega);
    std::printf("mean wind² %.1f m²/s²: the crown leans %.3f m downwind on average, F/k %.3f m\n",
                speedSquared, lean, expected);
    EXPECT_GT(speedSquared, 25.0);
    EXPECT_NEAR(lean, expected, 0.1 * expected);
}

}  // namespace
}  // namespace lagrange::server
