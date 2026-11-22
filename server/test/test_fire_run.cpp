// RFC 0022 in a whole run: the sample's pallet fire grows as a fast t² fire and the sprinkler's
// rate from --fire-water decays it by Evans' law; a URDF's <fire> burns on its link; with
// `--domain fire=off` nothing burns; its smoke rises to the sample's roof, holds the soot it made
// and reaches the visibility meter there.
#include "lagrange/server/Server.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <unistd.h>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

constexpr double kFast = 46.889;  // W/s², NBSIR 85-3167

std::string fireWorld() {
    return (std::filesystem::path(LAGRANGE_TEST_WAREHOUSE_USDA).parent_path().parent_path() /
            "warehouse_fire" / "warehouse_fire.usda")
        .string();
}

ServerOptions burning() {
    ServerOptions o;
    o.robotPath = "";
    o.worldPath = fireWorld();
    o.ros = false;
    o.realTimeFactor = 0.0;
    o.fireWater = {"pallet=0.133@60"};
    return o;
}

TEST(FireRun, TheSamplePalletGrowsAndTheSprinklerDecaysIt) {
    Server server(burning());
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.stepUntil(SimTime{30'000'000'000}).ok());
    EXPECT_NEAR(server.read("fire/pallet/hrr").at(0), kFast * 900.0, 1e-3 * kFast * 900.0);
    EXPECT_EQ(server.read("fire/pallet/state").at(0), 1.0);
    // The flame's base is the top of the pallet's collider (0.6 m up at x 5.6, y 1.3).
    const std::vector<double> base = server.read("fire/pallet/base");
    ASSERT_EQ(base.size(), 3U);
    EXPECT_NEAR(base[0], 5.6, 1e-6);
    EXPECT_NEAR(base[2], 0.6, 1e-6);  // the collider's float scale
    ASSERT_TRUE(server.stepUntil(SimTime{60'000'000'000}).ok());
    const double atWater = server.read("fire/pallet/hrr").at(0);
    ASSERT_TRUE(server.stepUntil(SimTime{160'000'000'000}).ok());
    const double tau = 3.0 * std::pow(0.133, -1.85);
    EXPECT_NEAR(server.read("fire/pallet/hrr").at(0), atWater * std::exp(-100.0 / tau),
                0.01 * atWater * std::exp(-100.0 / tau));
    EXPECT_EQ(server.read("fire/pallet/state").at(0), 3.0);
    EXPECT_EQ(server.read("fire/pallet/water_rate").at(0), 0.133);
}

// The smoke (RFC 0022 §4): until the water, Q = α·t², so the soot made is y_s·α·t³/(3·ΔH_c);
// under the roof none of it leaves the box (2 %, the steps' sum against the integral). The meter
// near the roof reads it: S = 3/σ and 8/σ of the same σ.
TEST(FireRun, TheSmokeFillsUnderTheRoofAndTheMeterSeesIt) {
    ServerOptions o = burning();
    o.ambientTemperatureK = 293.15;
    o.airDensityKgM3 = 1.2;
    Server server(o);
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.stepUntil(SimTime{60'000'000'000}).ok());
    const double made = 0.015 * kFast * std::pow(60.0, 3) / (3.0 * 1.2e7);
    EXPECT_NEAR(server.read("env/smoke/mass").at(0), made, 0.02 * made);
    const double sign = server.read("sensor/smoke_meter/sign_visibility").at(0);
    const double light = server.read("sensor/smoke_meter/light_visibility").at(0);
    EXPECT_GT(sign, 0.0);
    EXPECT_LT(sign, 1000.0) << "smoke at the meter";
    EXPECT_NEAR(light / sign, 8.0 / 3.0, 1e-9) << "the same measured σ";
}

TEST(FireRun, OffNothingBurns) {
    ServerOptions o = burning();
    o.substituted.insert("fire");
    Server server(o);
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(server.stepUntil(SimTime{10'000'000'000}).ok());
    EXPECT_EQ(server.read("fire/pallet/hrr").at(0), 0.0);
}

TEST(FireRun, AUrdfFireBurnsOnItsLink) {
    const std::filesystem::path file = std::filesystem::temp_directory_path() /
                                       ("lagrange_fire_" + std::to_string(::getpid()) + ".urdf");
    std::ofstream(file) << R"(<?xml version="1.0"?>
<robot name="bin">
  <link name="base">
    <inertial><mass value="5"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial>
    <collision><geometry><box size="0.5 0.5 0.4"/></geometry></collision>
  </link>
  <gazebo reference="base">
    <fire alpha="fast" hrr_per_area="4e5" fuel_mass="5" heat_of_combustion="1.2e7"
          radiative_fraction="0.3" soot_yield="0.015" critical_water_rate="0.02"
          ignition_time="1"/>
  </gazebo>
</robot>
)";
    ServerOptions o;
    o.robotPath = file.string();
    o.worldPath = "";
    o.ros = false;
    o.realTimeFactor = 0.0;
    Server server(o);
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.stepUntil(SimTime{11'000'000'000}).ok());
    EXPECT_NEAR(server.read("fire/base/hrr").at(0), kFast * 100.0, 1e-3 * kFast * 100.0);
    // Full involvement from 47 s: 400 kW/m² over the box's 0.25 m² footprint.
    ASSERT_TRUE(server.stepUntil(SimTime{60'000'000'000}).ok());
    EXPECT_NEAR(server.read("fire/base/hrr").at(0), 1e5, 1e-6 * 1e5);
    std::filesystem::remove(file);
}

}  // namespace
}  // namespace lagrange::server
