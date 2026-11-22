// Trees (docs/physics/ground.md "Trees") in a whole run, on MuJoCo: the sample quadcopter takes
// off from under the crown of the open field's tree_1 (TreeAPI_1, crown from 2.5 m to 6.1 m) and
// hovers 2 m up, its rotors in the crown: they cut the leaves they spin through (the cut volume and
// leaves grow, the rotors pay for it), and the rotors' air reaches the crown (tree/<n>/downwash).
// The same run twice is bit for bit.
#include "lagrange/server/Server.hpp"

#include <cmath>
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
        }
    }
    EXPECT_EQ(traces[0], traces[1]);
}

}  // namespace
}  // namespace lagrange::server
