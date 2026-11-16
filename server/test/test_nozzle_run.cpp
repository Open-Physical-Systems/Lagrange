// RFC 0022 §8 in a whole run: the firefighting sample AMR in the burning sample warehouse. Its
// monitor is aimed by joint commands at the elevation the drag-free jet needs to land on the
// pallet's middle; with the valve open the jet lands there through the robot's own links, all of
// it wets the fire (flow over the burning area), the heat release rate decays with Evans' time
// constant for that rate and the fire is out at τ·ln 100 after the water arrived; the tank loses
// the flow every second and its weight with it; the jet pushes the barrel back with ṁ v0. With
// `--domain actuators=off` nothing sprays.
#include "lagrange/server/Server.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <numbers>
#include <string>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

constexpr double kG = 9.80665;
constexpr double kWater = 998.2;                  // kg/m³, TankAPI_1's fallback
constexpr double kBarrel = 0.12;                  // m, tilt pivot to the nozzle's exit
constexpr double kPalletX = 5.6, kPalletY = 1.3;  // the pallet's middle (warehouse_small)
constexpr double kPalletTop = 0.6;                // m
constexpr double kBurningArea = 0.8 * 1.2;        // m², the pallet's footprint

ServerOptions firefighter() {
    ServerOptions o;
    o.robotPath = LAGRANGE_TEST_FIREFIGHTER_URDF;
    o.worldPath = LAGRANGE_TEST_WAREHOUSE_FIRE_USDA;
    o.ros = false;
    o.realTimeFactor = 0.0;
    o.jointControl = true;
    o.spawn.position = {1.6, kPalletY, 0.0};
    return o;
}

/// The nozzle's exit velocity, the sample's 0.5 bar through C_d 0.95.
double exitSpeed() {
    return 0.95 * std::sqrt(2.0 * 50'000.0 / kWater);
}

/// The tilt at which a drag-free jet from a barrel pivoting at `pivot` (world, m) lands at
/// horizontal distance `reach` on a surface at `top` (m): the lower of the two, by bisection.
double aim(const std::vector<double>& pivot, double reach, double top) {
    const double v = exitSpeed();
    const auto miss = [&](double e) {
        const double s = reach - kBarrel * std::cos(e);  // horizontal flight
        const double z0 = pivot[2] + kBarrel * std::sin(e);
        const double c = std::cos(e);
        return z0 + s * std::tan(e) - kG * s * s / (2.0 * v * v * c * c) - top;
    };
    double lo = -0.3;
    double hi = 0.7;  // below 45°: the rising side of the miss
    for (int i = 0; i < 80; ++i) {
        const double mid = 0.5 * (lo + hi);
        (miss(mid) > 0.0 ? hi : lo) = mid;
    }
    return 0.5 * (lo + hi);
}

/// The barrel's elevation above the horizontal in the world, from its link's pose (x forward).
double elevation(const std::vector<double>& pose) {
    const double w = pose[3], x = pose[4], y = pose[5], z = pose[6];
    const double up = 2.0 * (x * z - w * y);  // the rotated x axis's z component
    return std::asin(std::clamp(up, -1.0, 1.0));
}

TEST(NozzleRun, TheAmrPutsOutThePalletFire) {
    Server server(firefighter());
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.stepUntil(SimTime{2'000'000'000}).ok());
    const double full = 0.04 * kWater;
    EXPECT_NEAR(server.read("act/nozzle/tank_mass").at(0), full, 1e-9);
    // The tank's 39.9 kg pull down at the tank.
    EXPECT_NEAR(server.read("body/tank_link/wrench/tank").at(2), -full * kG, 1e-6);
    // The wheels held where they are (the jet would push the robot back) and the monitor aimed
    // at the elevation, in the world, that lands the jet on the pallet's middle from where its
    // pivot is. The joint's position loop sags under the barrel and the robot leans on its
    // casters under the tank, so the command is corrected by the barrel's elevation as it is,
    // as an operator corrects by eye.
    for (const char* wheel : {"left_wheel_joint", "right_wheel_joint"}) {
        const double at = server.read(std::string("joint/") + wheel + "/position").at(0);
        ASSERT_TRUE(server.commandJoint(wheel, "position", at));
    }
    ASSERT_TRUE(server.commandJoint("pan_joint", "position", 0.0));
    double command = 0.0;
    double tilt = 0.0;
    std::vector<double> pivot;
    for (int i = 0; i < 4; ++i) {
        ASSERT_TRUE(server.commandJoint("tilt_joint", "position", command));
        ASSERT_TRUE(server.stepUntil(SimTime{3'000'000'000 + i * 750'000'000LL}).ok());
        pivot = server.read("body/tilt_link/pose");
        ASSERT_EQ(pivot.size(), 7U);
        tilt = aim(pivot, kPalletX - pivot[0], kPalletTop);
        command += tilt - elevation(pivot);
    }
    ASSERT_TRUE(server.commandJoint("tilt_joint", "position", command));
    ASSERT_TRUE(server.stepUntil(SimTime{6'000'000'000}).ok());
    std::printf("pivot at %.3f, %.3f, %.3f: elevation %.4f rad for %.4f wanted (joint %.4f)\n",
                pivot[0], pivot[1], pivot[2], elevation(server.read("body/tilt_link/pose")), tilt,
                server.read("joint/tilt_joint/position").at(0));
    EXPECT_EQ(server.read("act/nozzle/flow").at(0), 0.0);  // the valve is shut
    const double hrrBefore = server.read("fire/pallet/hrr").at(0);
    EXPECT_GT(hrrBefore, 1000.0);
    // Open the valve.
    ASSERT_TRUE(server.commandActuator("nozzle", 1.0));
    ASSERT_TRUE(server.stepUntil(SimTime{6'500'000'000}).ok());
    const double v0 = exitSpeed();
    const double flow = kWater * std::numbers::pi * 0.004 * 0.004 * v0;
    EXPECT_NEAR(server.read("act/nozzle/flow").at(0), flow, 1e-9);
    const std::vector<double> impact = server.read("act/nozzle/impact");
    ASSERT_EQ(impact.size(), 3U);
    std::printf("lands at %.3f, %.3f, %.3f (the pallet's middle %.1f, %.1f, %.1f)\n", impact[0],
                impact[1], impact[2], kPalletX, kPalletY, kPalletTop);
    EXPECT_NEAR(impact[0], kPalletX, 0.15);
    EXPECT_NEAR(impact[1], kPalletY, 0.05);
    EXPECT_NEAR(impact[2], kPalletTop, 1e-3);
    // The reaction on the barrel: ṁ v0 against the jet.
    const std::vector<double> push = server.read("body/tilt_link/wrench/nozzle");
    ASSERT_EQ(push.size(), 6U);
    EXPECT_NEAR(std::hypot(push[0], push[1], push[2]), flow * v0, 1e-6 * flow * v0);
    EXPECT_LT(push[0], 0.0);
    // All of it on the fuel: w'' = ṁ / A.
    ASSERT_TRUE(server.stepUntil(SimTime{6'600'000'000}).ok());
    const double rate = flow / kBurningArea;
    EXPECT_NEAR(server.read("fire/pallet/water_rate").at(0), rate, 1e-6 * rate);
    // Find when the water arrived and when the fire went out, every 0.1 s.
    const double tau = 3.0 * std::pow(rate, -1.85);
    double arrived = -1.0;
    double out = -1.0;
    double atArrival = 0.0;
    for (int k = 61; k <= 900 && out < 0.0; ++k) {
        ASSERT_TRUE(server.stepUntil(SimTime{k * 100'000'000LL}).ok());
        const double t = k * 0.1;
        if (arrived < 0.0 && server.read("fire/pallet/water_rate").at(0) > 0.0) {
            arrived = t;
            atArrival = server.read("fire/pallet/hrr").at(0);
        }
        if (server.read("fire/pallet/state").at(0) == 4.0) {
            out = t;
        }
        if (arrived > 0.0 && k % 50 == 0 && out < 0.0) {
            const double expected = atArrival * std::exp(-(t - arrived) / tau);
            EXPECT_NEAR(server.read("fire/pallet/hrr").at(0), expected, 0.05 * expected) << t;
        }
    }
    ASSERT_GT(arrived, 0.0);
    ASSERT_GT(out, 0.0);
    std::printf(
        "w'' %.4f kg/(m2 s): tau %.2f s; water at %.1f s, out at %.1f s (%.1f s, "
        "tau ln 100 = %.1f s)\n",
        rate, tau, arrived, out, out - arrived, tau * std::log(100.0));
    EXPECT_NEAR(out - arrived, tau * std::log(100.0), 0.5);
    // The tank: what the nozzle let out since it opened at 6.0 s (10 ms steps).
    const double spent = out - 6.0;
    EXPECT_NEAR(server.read("act/nozzle/tank_mass").at(0), full - flow * spent, 0.002 * full);
    EXPECT_NEAR(server.read("body/tank_link/wrench/tank").at(2),
                -server.read("act/nozzle/tank_mass").at(0) * kG, 0.01);
}

// RFC 0022 §Validation: a fire with smoke and a nozzle, run twice: the same state hash.
TEST(NozzleRun, TheSameRunTwiceHasTheSameState) {
    const auto once = [] {
        ServerOptions o = firefighter();
        o.ambientTemperatureK = 293.15;  // so the smoke rises
        o.airDensityKgM3 = 1.2;
        Server server(o);
        EXPECT_TRUE(server.load().ok());
        EXPECT_TRUE(server.commandJoint("tilt_joint", "position", 0.25));
        EXPECT_TRUE(server.stepUntil(SimTime{2'000'000'000}).ok());
        EXPECT_TRUE(server.commandActuator("nozzle", 1.0));
        EXPECT_TRUE(server.stepUntil(SimTime{6'000'000'000}).ok());
        EXPECT_GT(server.read("act/nozzle/flow").at(0), 0.0);
        EXPECT_GT(server.read("env/smoke/mass").at(0), 0.0);
        const auto hash = server.stateHash();
        EXPECT_TRUE(hash.has_value());
        return hash.has_value() ? *hash : 0;
    };
    const std::uint64_t a = once();
    EXPECT_EQ(a, once());
}

TEST(NozzleRun, OffNothingSprays) {
    ServerOptions o = firefighter();
    o.substituted.insert("actuators");
    Server server(o);
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(server.commandActuator("nozzle", 1.0));
    ASSERT_TRUE(server.stepUntil(SimTime{1'000'000'000}).ok());
    EXPECT_EQ(server.read("act/nozzle/flow").at(0), 0.0);
    EXPECT_NEAR(server.read("act/nozzle/tank_mass").at(0), 0.04 * kWater, 1e-9);
    EXPECT_FALSE(server.commandActuator("no_such_nozzle", 1.0));
}

}  // namespace
}  // namespace lagrange::server
