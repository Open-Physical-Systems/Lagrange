// RFC 0020 §4.6 in whole runs, on MuJoCo (whose joint limits are soft at its default solref: the
// stops let the plunger 0.2 mm past rest under the spring's preload and about 2.6 mm past the
// end under the 2.5 N pull, so positions are checked to those), with the sample bench solenoid (a
// 20 g plunger, 10 mm of stroke, 12 V on 24 Ω):
//   - energised, the plunger pulls in, and its pull-in time and the current's dip agree with an
//     independent integration of the same coil, spring and mass at 1 µs (m x'' = ½ i² dL/dx −
//     F0 − k x, dλ/dt = V − R λ/L(x));
//   - at the stop the current settles at V/R and the pull at ½ (V/R)² dL/dx of the last segment;
//   - off, the spring brings it back to rest;
//   - the domain off leaves the coil unpowered; the same run twice is bit for bit.
#include "lagrange/server/Server.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

// The sample's numbers (solenoid_bench.overlay.yaml, solenoid_bench.urdf).
const std::vector<double> kX{0.0, 0.002, 0.004, 0.006, 0.008, 0.009, 0.010};
const std::vector<double> kL{0.030, 0.033, 0.037, 0.044, 0.057, 0.070, 0.090};
constexpr double kR = 24.0, kV = 12.0, kRate = 5.0, kPreload = 0.1, kStroke = 0.010;
constexpr double kMass = 0.02;
// MuJoCo's soft stops: how far past them the plunger may sit, m.
constexpr double kSoftStop = 3e-4;
constexpr double kSoftStopPulled = 3e-3;

ServerOptions bench() {
    ServerOptions o;
    o.robotPath = LAGRANGE_TEST_SOLENOID_URDF;
    o.overlayPath = LAGRANGE_TEST_SOLENOID_OVERLAY;
    o.ros = false;
    o.realTimeFactor = 0.0;
    return o;
}

double at(Server& server, const std::string& signal) {
    return server.read(signal).at(0);
}

std::size_t seg(double x) {
    std::size_t k = 0;
    while (k + 2 < kX.size() && x >= kX[k + 1]) {
        ++k;
    }
    return k;
}
double inductance(double x) {
    x = std::clamp(x, 0.0, kStroke);
    const std::size_t k = seg(x);
    return kL[k] + (x - kX[k]) / (kX[k + 1] - kX[k]) * (kL[k + 1] - kL[k]);
}
double slope(double x) {
    const std::size_t k = seg(std::clamp(x, 0.0, kStroke));
    return (kL[k + 1] - kL[k]) / (kX[k + 1] - kX[k]);
}

/// The reference: the coil, spring and plunger at 1 µs (semi-implicit Euler), from rest with the
/// coil switched on at t = 0; the time the plunger reaches `reach` and the lowest current after
/// the first peak before then plus 5 ms.
struct Reference {
    double pullIn{-1.0};
    double peak{0.0};
    double dip{0.0};
};
Reference reference(double reach) {
    Reference r;
    const double dt = 1e-6;
    double flux = 0.0, x = 0.0, v = 0.0, t = 0.0;
    double lowest = 1e9;
    bool peaked = false;
    double previous = 0.0;
    while (t < 0.2) {
        const double L = inductance(x);
        const double i = flux / L;
        flux += dt * (kV - kR * i);
        double force = 0.5 * i * i * slope(x) - (kPreload + kRate * x);
        v += dt * force / kMass;
        x += dt * v;
        if (x < 0.0) {
            x = 0.0;
            v = std::max(v, 0.0);
        }
        if (x > kStroke) {
            x = kStroke;
            v = std::min(v, 0.0);
        }
        t += dt;
        if (!peaked && i < previous && x > 0.0) {
            peaked = true;
            r.peak = previous;
        }
        if (peaked && r.pullIn < 0.0) {
            lowest = std::min(lowest, i);
        }
        previous = i;
        if (r.pullIn < 0.0 && x >= reach) {
            r.pullIn = t;
        }
        if (r.pullIn > 0.0 && t > r.pullIn + 0.005) {
            break;
        }
        if (r.pullIn > 0.0) {
            lowest = std::min(lowest, i);
        }
    }
    r.dip = lowest;
    return r;
}

TEST(SolenoidsRun, PullInFollowsTheCoilAndPlungersOwnEquations) {
    Server server(bench());
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.stepUntil(SimTime{100'000'000}).ok());
    // The spring holds it out, against the soft stop.
    EXPECT_LT(std::abs(at(server, "joint/plunger/position")), kSoftStop);
    EXPECT_EQ(at(server, "act/plunger/current"), 0.0);
    ASSERT_TRUE(server.commandActuator("plunger", 1.0));
    const SimTime start = server.simTime();
    const double reach = 0.99 * kStroke;
    double pullIn = -1.0, peak = 0.0, dip = 1e9, previous = 0.0;
    bool peaked = false;
    for (int k = 0; k < 200; ++k) {  // the run's 1 ms steps, 0.2 s
        ASSERT_TRUE(server.stepUntil(SimTime{server.simTime().ns + 1'000'000}).ok());
        const double i = at(server, "act/plunger/current");
        const double x = at(server, "joint/plunger/position");
        const double t = static_cast<double>(server.simTime().ns - start.ns) * 1e-9;
        if (!peaked && i < previous && x > 1e-5) {
            peaked = true;
            peak = previous;
        }
        if (peaked) {
            dip = std::min(dip, i);
        }
        previous = i;
        if (pullIn < 0.0 && x >= reach) {
            pullIn = t;
        }
        if (pullIn > 0.0 && t > pullIn + 0.005) {
            break;
        }
    }
    const Reference ref = reference(reach);
    std::printf(
        "solenoid pull-in: %.2f ms (reference %.2f ms); current peak %.4f A (%.4f), dip %.4f A "
        "(%.4f)\n",
        pullIn * 1e3, ref.pullIn * 1e3, peak, ref.peak, dip, ref.dip);
    ASSERT_GT(pullIn, 0.0) << "the plunger never pulled in";
    EXPECT_NEAR(pullIn, ref.pullIn, 0.05 * ref.pullIn);
    EXPECT_NEAR(peak, ref.peak, 0.05 * ref.peak);
    // The dip: the current falls while the plunger moves in, as a valve driver sees it.
    EXPECT_LT(dip, 0.9 * peak);
    EXPECT_NEAR(dip, ref.dip, 0.1 * ref.dip);
    // At the stop: V/R, and the last segment's pull.
    ASSERT_TRUE(server.stepUntil(SimTime{server.simTime().ns + 100'000'000}).ok());
    EXPECT_NEAR(at(server, "act/plunger/current"), kV / kR, 0.01 * kV / kR);
    const double pull = 0.5 * (kV / kR) * (kV / kR) * (0.090 - 0.070) / 0.001;
    EXPECT_NEAR(at(server, "act/plunger/force"), pull, 0.01 * pull);
    EXPECT_GE(at(server, "joint/plunger/position"), kStroke);  // fully in, on the soft stop
    EXPECT_LT(at(server, "joint/plunger/position"), kStroke + kSoftStopPulled);
    EXPECT_NEAR(at(server, "act/plunger/power"), kV * kV / kR, 0.01 * kV * kV / kR);
    // Off: the spring returns it.
    ASSERT_TRUE(server.commandActuator("plunger", 0.0));
    ASSERT_TRUE(server.stepUntil(SimTime{server.simTime().ns + 300'000'000}).ok());
    EXPECT_LT(at(server, "joint/plunger/position"), kSoftStop);
    EXPECT_NEAR(at(server, "act/plunger/current"), 0.0, 1e-6);
}

TEST(SolenoidsRun, TheDomainOffLeavesTheCoilUnpowered) {
    ServerOptions o = bench();
    o.substituted.insert("actuators");
    Server server(o);
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.commandActuator("plunger", 1.0));
    ASSERT_TRUE(server.stepUntil(SimTime{200'000'000}).ok());
    EXPECT_EQ(at(server, "act/plunger/current"), 0.0);
    EXPECT_LT(std::abs(at(server, "joint/plunger/position")), kSoftStop);
}

TEST(SolenoidsRun, SameRunTwiceIsBitForBit) {
    std::vector<double> traces[2];
    for (auto& trace : traces) {
        Server server(bench());
        ASSERT_TRUE(server.load().ok());
        ASSERT_TRUE(server.stepUntil(SimTime{50'000'000}).ok());
        ASSERT_TRUE(server.commandActuator("plunger", 1.0));
        for (int k = 0; k < 100; ++k) {
            ASSERT_TRUE(server.stepUntil(SimTime{server.simTime().ns + 1'000'000}).ok());
            trace.push_back(at(server, "joint/plunger/position"));
            trace.push_back(at(server, "act/plunger/current"));
        }
    }
    EXPECT_EQ(traces[0], traces[1]);
}

}  // namespace
}  // namespace lagrange::server
