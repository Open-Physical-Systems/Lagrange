// Holding brakes (RFC 0020, "Also worth adding") in whole runs, on MuJoCo, with the sample arm
// (1 kg 0.3 m out on a horizontal shoulder, 2.94 N·m of gravity at the horizontal) and its
// 5 N·m brake:
//   - applied, it holds the arm against gravity without creeping;
//   - released, it lets go after the coil's −τ·ln(1 − I_r/I_0); the arm falls; powered off it is
//     applied again after −τ·ln(I_d/I_0) and its 4 N·m dynamic torque stops the arm, where an
//     independent integration of the same arm, brake and coil stops it;
//   - the domain off (zero) never holds; the same run twice is bit for bit.
#include "lagrange/server/Server.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

// The sample's numbers (brake_arm.urdf, brake_arm.overlay.yaml).
constexpr double kMass = 1.0, kArm = 0.3, kInertia = 1.0 * 0.3 * 0.3 + 0.001;
constexpr double kG = 9.80665;
constexpr double kHolding = 5.0, kDynamic = 4.0;
constexpr double kTau = 0.5 / 48.0, kFull = 24.0 / 48.0, kRelease = 0.35, kDropout = 0.15;

ServerOptions arm() {
    ServerOptions o;
    o.robotPath = LAGRANGE_TEST_BRAKE_URDF;
    o.overlayPath = LAGRANGE_TEST_BRAKE_OVERLAY;
    o.ros = false;
    o.realTimeFactor = 0.0;
    return o;
}

double at(Server& server, const std::string& signal) {
    return server.read(signal).at(0);
}

bool step(Server& server, std::int64_t ns) {
    return server.stepUntil(SimTime{server.simTime().ns + ns}).ok();
}

/// The arm at 10 µs: released from rest at the horizontal for `free` s, then the brake applied;
/// the angle it stops at (θ grows as the arm falls: gravity's torque m·g·r·cos θ).
double referenceStop(double free) {
    const double dt = 1e-5;
    double theta = 0.0, omega = 0.0, t = 0.0;
    while (t < free) {
        omega += dt * kMass * kG * kArm * std::cos(theta) / kInertia;
        theta += dt * omega;
        t += dt;
    }
    while (omega > 0.0) {
        omega += dt * (kMass * kG * kArm * std::cos(theta) - kDynamic) / kInertia;
        theta += dt * std::max(omega, 0.0);
    }
    return theta;
}

TEST(BrakesRun, AppliedItHoldsTheArmAgainstGravity) {
    Server server(arm());
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(step(server, 100'000'000));
    const double start = at(server, "joint/shoulder/position");
    ASSERT_TRUE(step(server, 1'000'000'000));
    const double drift = at(server, "joint/shoulder/position") - start;
    std::printf("held arm: %.2e rad in 1 s\n", drift);
    EXPECT_LT(std::abs(drift), 1e-4);
    EXPECT_LT(std::abs(start), 1e-3);
    EXPECT_EQ(at(server, "act/shoulder/engaged"), 1.0);
    EXPECT_EQ(at(server, "act/shoulder/torque"), kHolding);
}

TEST(BrakesRun, ReleasedTheArmFallsAndAppliedAgainItStopsIt) {
    Server server(arm());
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(step(server, 200'000'000));
    ASSERT_TRUE(server.commandActuator("shoulder", 1.0));
    const SimTime on = server.simTime();
    double released = -1.0;
    for (int k = 0; k < 100 && released < 0.0; ++k) {
        ASSERT_TRUE(step(server, 1'000'000));
        if (at(server, "act/shoulder/engaged") == 0.0) {
            released = static_cast<double>(server.simTime().ns - on.ns) * 1e-9;
        }
    }
    const double releaseTime = -kTau * std::log(1.0 - kRelease / kFull);
    ASSERT_GT(released, 0.0);
    EXPECT_GE(released, releaseTime - 1e-9);
    EXPECT_LT(released, releaseTime + 0.0011);
    // Falling for 0.1 s after the command, then off.
    const double releasedAt = static_cast<double>(server.simTime().ns) * 1e-9;
    ASSERT_TRUE(step(server, 100'000'000 - static_cast<std::int64_t>(released * 1e9)));
    ASSERT_TRUE(server.commandActuator("shoulder", 0.0));
    const SimTime off = server.simTime();
    double engaged = -1.0;
    for (int k = 0; k < 100 && engaged < 0.0; ++k) {
        ASSERT_TRUE(step(server, 1'000'000));
        if (at(server, "act/shoulder/engaged") == 1.0) {
            engaged = static_cast<double>(server.simTime().ns - off.ns) * 1e-9;
        }
    }
    const double engageTime = -kTau * std::log(kDropout / kFull);
    ASSERT_GT(engaged, 0.0);
    EXPECT_GE(engaged, engageTime - 1e-9);
    EXPECT_LT(engaged, engageTime + 0.0011);
    const double free = static_cast<double>(server.simTime().ns) * 1e-9 - releasedAt;
    // It stops and stays.
    ASSERT_TRUE(step(server, 800'000'000));
    const double stopped = at(server, "joint/shoulder/position");
    ASSERT_TRUE(step(server, 500'000'000));
    const double later = at(server, "joint/shoulder/position");
    const double expected = referenceStop(free);
    std::printf(
        "brake: released %.1f ms (%.2f), applied %.1f ms (%.2f), free %.4f s, stopped at "
        "%.4f rad (reference %.4f), then %.2e rad in 0.5 s\n",
        released * 1e3, releaseTime * 1e3, engaged * 1e3, engageTime * 1e3, free, stopped, expected,
        later - stopped);
    EXPECT_NEAR(stopped, expected, 0.03 * expected);
    EXPECT_LT(std::abs(later - stopped), 1e-4);
    EXPECT_NEAR(at(server, "joint/shoulder/velocity"), 0.0, 1e-3);
}

TEST(BrakesRun, TheDomainOffNeverHolds) {
    ServerOptions o = arm();
    o.substituted.insert("actuators");
    Server server(o);
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(step(server, 300'000'000));
    EXPECT_GT(at(server, "joint/shoulder/position"), 0.3);  // fell
    EXPECT_EQ(at(server, "act/shoulder/engaged"), 0.0);
}

TEST(BrakesRun, SameRunTwiceIsBitForBit) {
    std::vector<double> traces[2];
    for (auto& trace : traces) {
        Server server(arm());
        ASSERT_TRUE(server.load().ok());
        ASSERT_TRUE(step(server, 50'000'000));
        ASSERT_TRUE(server.commandActuator("shoulder", 1.0));
        for (int k = 0; k < 150; ++k) {
            if (k == 80) {
                ASSERT_TRUE(server.commandActuator("shoulder", 0.0));
            }
            ASSERT_TRUE(step(server, 1'000'000));
            trace.push_back(at(server, "joint/shoulder/position"));
            trace.push_back(at(server, "act/shoulder/current"));
        }
    }
    EXPECT_EQ(traces[0], traces[1]);
}

}  // namespace
}  // namespace lagrange::server
