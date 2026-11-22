// Blade damage (rotor_aero.md §10) in a whole run, on MuJoCo: the sample quadcopter hovers 2 m up
// and at 3 s rotor_1 loses 10 % of one blade (the blade_cutter test plugin writes
// rotor/rotor_1_joint/damage, as a branch strike does). Its thrust at a given speed drops by the
// blade-element share, the controller spins it faster to make up for it, and the mass imbalance
// shakes the frame: the IMU's spectrum peaks at that rotor's speed of rotation. The same run twice
// is bit for bit.
#include "lagrange/rotor_aero/RotorAero.hpp"
#include "lagrange/server/Server.hpp"

#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <numbers>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

constexpr double kDamage = 0.1;     // the plugin's default
constexpr double kCutAt = 3.0;      // s, the plugin's default
constexpr double kImuRate = 250.0;  // Hz, the sample quadcopter's IMU (sensor:rate)

ServerOptions hover(bool cut) {
    ServerOptions o;
    o.robotPath = LAGRANGE_TEST_QUAD_USDA;
    o.worldPath = "";
    o.airDensityKgM3 = 1.225;
    o.ros = false;
    o.realTimeFactor = 0.0;
    if (cut) {
        ::setenv("LAGRANGE_PLUGIN_PATH", LAGRANGE_TEST_PLUGIN_DIR, 1);
        o.plugins = {"blade_cutter"};
    }
    return o;
}

double at(Server& server, const std::string& signal) {
    return server.read(signal).at(0);
}

struct Flight {
    std::vector<double> accel[2];       // m/s², the IMU's specific force x and y, 4 to 6 s
    std::vector<double> gyro[2];        // rad/s, its angular rate x and y
    double ratioBefore{0.0};            // rotor_1's thrust / ω², N·s², 2.5 to 3 s
    double ratioAfter{0.0};             // the same, 5 to 6 s
    double speed{0.0};                  // rad/s, rotor_1's mean |ω|, 4 to 6 s
    double lowest{1e9}, highest{-1e9};  // m, the base link's height after the cut
    std::vector<double> trace;          // for bit-for-bit
};

Flight fly(bool cut) {
    Server server(hover(cut));
    const Status loaded = server.load();
    EXPECT_TRUE(loaded.ok()) << loaded.message;
    Flight f;
    int before = 0;
    int after = 0;
    int spun = 0;
    // One IMU sample per 4 ms.
    const std::int64_t step = static_cast<std::int64_t>(1e9 / kImuRate);
    while (server.simTime().seconds() < 6.0 - 1e-9) {
        EXPECT_TRUE(server.stepUntil(SimTime{server.simTime().ns + step}).ok());
        const double t = server.simTime().seconds();
        const double omega = at(server, "joint/rotor_1_joint/velocity");
        const double thrust = at(server, "rotor/rotor_1_joint/thrust");
        if (t > 2.5 && t <= kCutAt) {
            f.ratioBefore += thrust / (omega * omega);
            ++before;
        }
        if (t > 5.0) {
            f.ratioAfter += thrust / (omega * omega);
            ++after;
        }
        const double z = server.read("body/base_link/pose").at(2);
        if (t > kCutAt) {
            f.lowest = std::min(f.lowest, z);
            f.highest = std::max(f.highest, z);
        }
        if (t > 4.0) {
            const std::vector<double> a = server.read("sensor/imu/specific_force");
            const std::vector<double> g = server.read("sensor/imu/angular_rate");
            for (int k = 0; k < 2; ++k) {
                f.accel[k].push_back(a.at(k));
                f.gyro[k].push_back(g.at(k));
            }
            f.speed += std::abs(omega);
            ++spun;
        }
        f.trace.push_back(omega);
        f.trace.push_back(z);
    }
    f.ratioBefore /= before;
    f.ratioAfter /= after;
    f.speed /= spun;
    return f;
}

// The amplitude of `x` (sampled at kImuRate) at `hz`, Hann-windowed, mean removed.
double amplitude(const std::vector<double>& x, double hz) {
    double mean = 0.0;
    for (const double v : x) {
        mean += v;
    }
    mean /= static_cast<double>(x.size());
    std::complex<double> sum;
    double weights = 0.0;
    const auto n = static_cast<double>(x.size());
    for (std::size_t i = 0; i < x.size(); ++i) {
        const double w = 0.5 - 0.5 * std::cos(2.0 * std::numbers::pi * static_cast<double>(i) / n);
        sum += w * (x[i] - mean) *
               std::polar(1.0, -2.0 * std::numbers::pi * hz * static_cast<double>(i) / kImuRate);
        weights += w;
    }
    return 2.0 * std::abs(sum) / weights;
}

// The strongest frequency of `x` from 20 Hz to Nyquist, by 0.25 Hz.
double peak(const std::vector<double>& x) {
    double best = 0.0;
    double where = 0.0;
    for (double hz = 20.0; hz < kImuRate / 2.0; hz += 0.25) {
        if (const double a = amplitude(x, hz); a > best) {
            best = a;
            where = hz;
        }
    }
    return where;
}

TEST(BladeDamageRun, ADamagedRotorLosesThrustAndShakesTheImuAtItsSpeed) {
    const Flight whole = fly(false);
    const Flight cut = fly(true);
    // Its thrust at a given speed falls by the blade-element share; the whole run's does not.
    const double share = rotor_aero::damagedThrustShare(kDamage, 2);
    EXPECT_NEAR(cut.ratioAfter / cut.ratioBefore, share, 0.01 * share);
    EXPECT_NEAR(whole.ratioAfter / whole.ratioBefore, 1.0, 0.01);
    // The controller spins it faster to make up, about 1/√share (the other rotors share a little).
    EXPECT_GT(cut.speed, 1.04 * whole.speed) << "the controller spins it faster to make up";
    // The rotor's frequency, folded about Nyquist when the IMU samples it too slowly.
    double rotor = cut.speed / (2.0 * std::numbers::pi);
    if (rotor > kImuRate / 2.0) {
        rotor = kImuRate - std::fmod(rotor, kImuRate);
    }
    const double accelPeak = peak(cut.accel[0]);
    const double gyroPeak = peak(cut.gyro[0]);
    const double shake = std::hypot(amplitude(cut.accel[0], rotor), amplitude(cut.accel[1], rotor));
    const double calm =
        std::hypot(amplitude(whole.accel[0], rotor), amplitude(whole.accel[1], rotor));
    std::printf(
        "rotor_1 at %.0f rad/s (%.1f Hz; whole %.0f rad/s); thrust/ω² x %.4f (share %.4f); IMU "
        "peaks: accel %.2f Hz, gyro %.2f Hz; accel at the rotor's frequency %.2f m/s² "
        "(whole %.4f); height %.2f to %.2f m\n",
        cut.speed, rotor, whole.speed, cut.ratioAfter / cut.ratioBefore, share, accelPeak, gyroPeak,
        shake, calm, cut.lowest, cut.highest);
    EXPECT_NEAR(accelPeak, rotor, 1.0) << "the accelerometer shakes at the damaged rotor's speed";
    EXPECT_NEAR(gyroPeak, rotor, 1.0) << "and so does the gyro";
    EXPECT_GT(shake, 20.0 * calm);
    EXPECT_GT(cut.lowest, 1.0) << "it keeps flying";
}

TEST(BladeDamageRun, SameRunTwiceIsBitForBit) {
    const Flight a = fly(true);
    const Flight b = fly(true);
    ASSERT_FALSE(a.trace.empty());
    EXPECT_EQ(a.trace, b.trace);
    EXPECT_EQ(a.accel[0], b.accel[0]);
}

}  // namespace
}  // namespace lagrange::server
