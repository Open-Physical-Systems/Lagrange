// Branches (docs/physics/ground.md "Branches") in a whole run, on MuJoCo: the sample quadcopter
// takes off from under the open field's tree_1 (as test_trees_run) and climbs into its crown,
// where a blade meets a limb: the rotor stops, the drone is kicked sideways and falls out of the
// tree. The same run twice, and a run restored from a checkpoint before the strike, are bit for
// bit. A ram on a rail (test/data/tree_ram.urdf) driven into the crown stops on its wood, the
// wood's push balancing the joint's force and bending the crown back; its lidar sees the wood;
// the same run twice is bit for bit.
#include "lagrange/server/Server.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
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

const char* const kRotors[] = {"rotor_1_joint", "rotor_2_joint", "rotor_3_joint", "rotor_4_joint"};

struct Strike {
    double time{-1.0};
    int rotor{-1};
    double before{0.0};  // rad/s, 2 ms earlier
    double after{0.0};   // rad/s, at the strike's report
    double height{0.0};  // m, of the base link
    double sideSpeed{0.0};
};

// Steps in 2 ms until the first strike, then on to `seconds`.
Strike flyInto(Server& server, double seconds) {
    Strike first;
    double speeds[4] = {};
    while (server.simTime().seconds() < seconds) {
        EXPECT_TRUE(server.stepUntil(SimTime{server.simTime().ns + 2'000'000}).ok());
        for (int r = 0; r < 4; ++r) {
            const double w = at(server, std::string("joint/") + kRotors[r] + "/velocity");
            if (first.rotor < 0 &&
                at(server, std::string("rotor/") + kRotors[r] + "/branch_strikes") > 0.0) {
                const std::vector<double> p = server.read("body/base_link/pose");
                first = {server.simTime().seconds(), r, speeds[r], w, p[2], 0.0};
            }
            speeds[r] = w;
        }
        if (first.rotor >= 0 && server.simTime().seconds() < first.time + 0.1) {
            const std::vector<double> v = server.read("body/base_link/twist");
            first.sideSpeed = std::max(first.sideSpeed, std::hypot(v[0], v[1]));
        }
    }
    return first;
}

// The quadcopter climbs into tree_1's crown (base 2.5 m) until a blade meets a limb several
// centimetres thick: that rotor is stopped within the step, the strike kicks the drone sideways,
// and with a rotor stalling on the wood it falls out of the tree to the ground.
TEST(BranchesRun, ARotorMeetsALimbStallsAndTheDroneFalls) {
    Server server(underTheTree());
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    const Strike s = flyInto(server, 6.0);
    ASSERT_GE(s.rotor, 0) << "a blade meets a branch";
    double strikes = 0.0;
    double damage = 0.0;
    for (const char* r : kRotors) {
        strikes += at(server, std::string("rotor/") + r + "/branch_strikes");
        damage = std::max(damage, at(server, std::string("rotor/") + r + "/damage"));
    }
    const std::vector<double> p = server.read("body/base_link/pose");
    std::printf(
        "first strike at %.3f s, %s, base link %.2f m up: %.0f -> %.1f rad/s; side speed "
        "up to %.2f m/s; %.0f strikes, damage up to %.3f; the drone at %.2f m after 6 s; "
        "%.0f twigs cut\n",
        s.time, kRotors[s.rotor], s.height, s.before, s.after, s.sideSpeed, strikes, damage, p[2],
        at(server, "tree/tree_1/twigs_cut"));
    EXPECT_GT(s.height, 2.5 - 0.2) << "in the crown, over its base";
    EXPECT_GT(std::abs(s.before), 100.0) << "spinning when it met the wood";
    EXPECT_LT(std::abs(s.after), 0.05 * std::abs(s.before)) << "and stopped by it";
    EXPECT_GT(s.sideSpeed, 0.1) << "the strike kicks the drone sideways";
    EXPECT_GT(damage, 0.0);
    EXPECT_LE(damage, 1.0);
    EXPECT_LT(p[2], 0.5) << "a stalled rotor brings it down out of the tree";
    EXPECT_EQ(at(server, "tree/tree_2/twigs_cut"), 0.0);
}

TEST(BranchesRun, SameRunTwiceAndARestoredRunAreBitForBit) {
    std::vector<double> traces[3];
    for (int k = 0; k < 2; ++k) {
        Server server(underTheTree());
        ASSERT_TRUE(server.load().ok());
        ASSERT_TRUE(server.stepUntil(SimTime{2'200'000'000}).ok());
        const auto saved = server.saveState();
        ASSERT_TRUE(saved);
        for (int run = 0; run < 2; ++run) {
            if (run == 1 && k == 1) {  // the second server goes back and runs it again
                ASSERT_TRUE(server.restoreState(*saved).ok());
            } else if (run == 1) {
                break;
            }
            std::vector<double>& trace = traces[k + run];
            while (server.simTime().ns < 2'800'000'000) {
                ASSERT_TRUE(server.stepUntil(SimTime{server.simTime().ns + 10'000'000}).ok());
                for (const char* r : kRotors) {
                    trace.push_back(at(server, std::string("rotor/") + r + "/branch_strikes"));
                    trace.push_back(at(server, std::string("rotor/") + r + "/damage"));
                    trace.push_back(at(server, std::string("joint/") + r + "/velocity"));
                }
                for (const double v : server.read("body/base_link/pose")) {
                    trace.push_back(v);
                }
            }
        }
    }
    ASSERT_FALSE(traces[0].empty());
    EXPECT_GT(traces[0][traces[0].size() - 7 - 12], 0.0) << "a strike happened in the window";
    EXPECT_EQ(traces[0], traces[1]);
    EXPECT_EQ(traces[0], traces[2]);
}

ServerOptions ramAtTheTree() {
    ServerOptions o;
    o.robotPath = LAGRANGE_TEST_TREE_RAM;
    o.worldPath = LAGRANGE_TEST_OPEN_FIELD;
    o.ros = false;
    o.realTimeFactor = 0.0;
    o.jointControl = true;
    o.lidar = ServerOptions::Lidar::Cpu;
    return o;
}

// The ram is driven at 0.3 m/s along -y with up to 40 N (its effort limit) into tree_1's crown.
// It stops on the wood at the crown's edge: the wood pushes it back with the 40 N the joint
// pushes along its rail, and the crown's sway mode takes the same push the other way, bending to
// F/(m ω²) on average.
TEST(BranchesRun, ARamStopsOnTheWoodAndBendsTheCrown) {
    Server server(ramAtTheTree());
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.commandJoint("ram_joint", "velocity", 0.3));
    ASSERT_TRUE(server.stepUntil(SimTime{4'000'000'000}).ok());
    double along = 0.0;
    double sway = 0.0;
    double speed = 0.0;
    constexpr int kSamples = 40;
    for (int k = 0; k < kSamples; ++k) {  // 4 s, two sway periods
        ASSERT_TRUE(server.stepUntil(SimTime{server.simTime().ns + 100'000'000}).ok());
        along += server.read("body/ram/wrench/branches").at(1) / kSamples;
        sway += server.read("tree/tree_1/sway").at(1) / kSamples;
        speed = std::max(speed, std::abs(server.read("body/ram/twist").at(1)));
    }
    const std::vector<double> p = server.read("body/ram/pose");
    constexpr double kPi = 3.14159265358979323846;
    const double mass = 2.0 * 4.0 / 3.0 * kPi * 1.8 * 1.8 * 1.8;  // tree_1's crown, 2 kg/m³
    const double omega = 2.0 * kPi * 0.5;
    std::printf(
        "the ram stops at y %.3f m (the crown's edge at %.1f m): the wood pushes back %.1f N "
        "along the rail, the crown leans %.3f m (F/k %.3f m), the ram moving at most %.3f m/s; "
        "%.0f twigs broken\n",
        p[1], 8.0 + 1.8, along, sway, -along / (mass * omega * omega), speed,
        at(server, "tree/tree_1/twigs_cut"));
    EXPECT_GT(p[1], 8.0) << "it stopped short of the trunk";
    EXPECT_LT(p[1], 9.9 - 0.1) << "and inside the crown";
    EXPECT_NEAR(along, 40.0, 2.0) << "the wood balances the joint's 40 N along the rail";
    EXPECT_LT(sway, 0.0) << "the crown is pushed the other way";
    EXPECT_NEAR(sway, -along / (mass * omega * omega), 0.25 * along / (mass * omega * omega));
    // The lidar on the ram sees the wood ahead, nearer than the trunk.
    const std::vector<double> ranges = server.read("sensor/front_lidar/ranges");
    const double nearest = *std::ranges::min_element(ranges);
    EXPECT_LT(nearest, p[1] - 8.0 - 0.2);
}

TEST(BranchesRun, ARamRunTwiceIsBitForBit) {
    std::vector<double> traces[2];
    for (auto& trace : traces) {
        Server server(ramAtTheTree());
        ASSERT_TRUE(server.load().ok());
        ASSERT_TRUE(server.commandJoint("ram_joint", "velocity", 0.3));
        for (int k = 0; k < 30; ++k) {
            ASSERT_TRUE(server.stepUntil(SimTime{server.simTime().ns + 100'000'000}).ok());
            for (const char* signal :
                 {"body/ram/pose", "body/ram/wrench/branches", "tree/tree_1/sway",
                  "tree/tree_1/twigs_cut", "sensor/front_lidar/ranges"}) {
                const std::vector<double> v = server.read(signal);
                trace.insert(trace.end(), v.begin(), v.end());
            }
        }
    }
    EXPECT_EQ(traces[0], traces[1]);
}

}  // namespace
}  // namespace lagrange::server
