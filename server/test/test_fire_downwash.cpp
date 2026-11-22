// RFC 0022 §5.2 in a whole run: the sample quadcopter takes off half a metre beside a small fire
// (a waste bin, made by an overlay) and hovers 2 m up. The aerodynamics domain writes its rotors'
// wake half way up the flame, fire/<n>/downwash (there, the ground outwash); the fire adds it to
// the wind, and with a blow-off speed below it the flame goes out (state 6, blown off) at the step
// after the air there passes that speed. With the aerodynamics domain off there is no downwash and
// it burns.
#include "lagrange/server/Server.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <unistd.h>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

constexpr double kBlowOff = 2.0;  // m/s: the bin's blow-off speed, an example value

struct Downwash {
    double peak{0.0};  // m/s, the strongest air at the base while it burned
    double state{0.0};
    double outAt{-1.0};
    double speedAtOut{0.0};
    double before{0.0};   // m/s at the last step before it went out
    double outward{0.0};  // m/s, the most air at it away from the drone (+x) while it burned
};

Downwash hoverOverABin(bool aero) {
    std::stringstream text;
    text << "schema_version: 1\napply:\n  - create: bin\n    schemas:\n      FireAPI_1:\n"
         << "        fire:class: a\n        fire:alpha: 46.889\n        fire:hrrPerArea: 200000\n"
         << "        fire:area: 0.05\n        fire:fuelMass: 2\n"
         << "        fire:heatOfCombustion: 12000000\n        fire:radiativeFraction: 0.3\n"
         << "        fire:sootYield: 0.015\n        fire:ignitionTime: 0\n"
         << "        fire:criticalWaterRate: 0.02\n"
         << "        fire:blowOffSpeed: " << kBlowOff << "\n";
    const std::filesystem::path overlay =
        std::filesystem::temp_directory_path() /
        ("lagrange_fire_downwash_" + std::to_string(::getpid()) + ".overlay.yaml");
    std::ofstream(overlay) << text.str();
    ServerOptions o;
    o.robotPath = LAGRANGE_TEST_QUAD_USDA;
    o.worldPath = "";
    o.airDensityKgM3 = 1.225;
    o.ros = false;
    o.realTimeFactor = 0.0;
    o.overlayPath = overlay.string();
    o.spawn.position = {-0.5, 0.0, 0.0};  // the bin half a metre off the wake's axis
    if (!aero) {
        o.substituted.insert("aerodynamics");
    }
    Server server(o);
    const Status loaded = server.load();
    std::filesystem::remove(overlay);
    EXPECT_TRUE(loaded.ok()) << loaded.message;
    Downwash out;
    if (!loaded.ok()) {
        return out;
    }
    double last = 0.0;
    for (int k = 1; k <= 100; ++k) {  // 10 s, every 100 ms: the fire's step
        if (!server.stepUntil(SimTime{k * 100'000'000LL}).ok()) {
            ADD_FAILURE();
            return out;
        }
        const std::vector<double> dw = server.read("fire/bin/downwash");
        const double speed = dw.size() == 3 ? std::hypot(dw[0], dw[1], dw[2]) : 0.0;
        const double state = server.read("fire/bin/state").at(0);
        if (out.outAt < 0.0) {
            out.peak = std::max(out.peak, speed);
            if (state == 6.0) {
                out.outAt = 0.1 * k;
                out.speedAtOut = speed;
                out.before = last;
            } else if (dw.size() == 3) {
                out.outward = std::max(out.outward, dw[0]);
            }
        }
        last = speed;
        out.state = state;
    }
    return out;
}

TEST(FireDownwash, AQuadcopterBlowsOutASmallFireUnderIt) {
    const Downwash d = hoverOverABin(true);
    std::printf("blown off at %.1f s: %.2f m/s at the base (%.2f m/s the step before)\n", d.outAt,
                d.speedAtOut, d.before);
    EXPECT_EQ(d.state, 6.0);
    // Half a metre off the wake's axis, on the ground, the wake is the outwash: away from the
    // drone, along the ground.
    EXPECT_GT(d.outward, 0.5 * kBlowOff);
    ASSERT_GT(d.outAt, 0.0);
    // The fire reads the domain's previous step: out at the step after the air passed it.
    EXPECT_GT(std::max(d.speedAtOut, d.before), kBlowOff);
    const Downwash still = hoverOverABin(false);
    EXPECT_EQ(still.peak, 0.0);
    EXPECT_NE(still.state, 6.0);
}

}  // namespace
}  // namespace lagrange::server
