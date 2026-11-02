// Water: buoyancy against Archimedes, and what a lidar sees of it.
#include "lagrange/server/Server.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <unistd.h>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

/// A world with a free 0.4 m box of `mass` kg, starting at height z.
std::string boxWorld(double mass, double z) {
    const std::filesystem::path path = std::filesystem::temp_directory_path() /
                                       ("lagrange_water_" + std::to_string(::getpid()) + ".sdf");
    const double i = mass * 0.16 / 6.0;
    std::ofstream(path)
        << "<?xml version=\"1.0\"?><sdf version=\"1.9\"><world name=\"w\">"
        << "<model name=\"box\"><pose>0 0 " << z << " 0 0 0</pose>"
        << "<link name=\"link\"><inertial><mass>" << mass << "</mass><inertia><ixx>" << i
        << "</ixx><iyy>" << i << "</iyy><izz>" << i << "</izz></inertia>"
        << "</inertial><collision name=\"c\"><geometry><box><size>0.4 0.4 0.4</size>"
        << "</box></geometry></collision></link></model></world></sdf>\n";
    return path.string();
}

ServerOptions pool(double mass, double z, const std::string& physics) {
    ServerOptions o;
    o.worldPath = boxWorld(mass, z);
    o.water.push_back({{0.0, 0.0, 2.0}, 10.0, 10.0, 2.0});  // surface at 2 m, ground under it
    o.ros = false;
    o.realTimeFactor = 0.0;
    o.physics = physics;
    return o;
}

// Archimedes: a floating box's draft is its mass over ρ × footprint: 32 kg, 0.16 m², 0.2 m. Its
// centre sits level with the surface (half of its 0.4 m height under water).
TEST(Water, ABoxFloatsAtItsDraft) {
    for (const char* physics : {"mujoco", "jolt"}) {
        Server server(pool(32.0, 2.6, physics));
        const Status loaded = server.load();
        ASSERT_TRUE(loaded.ok()) << loaded.message;
        // Quadratic drag alone calms a small bob slowly, as no waves carry its energy away, and
        // Jolt damps it less than MuJoCo's implicit steps do: the mean over a few bobs. A cube
        // half as dense as water is unstable upright (its metacentric height BM − BG =
        // 0.067 − 0.1 m is negative), so it also rolls slowly over towards a tilted float, its
        // centre at the surface at any tilt (half a cube is under any plane through its centre);
        // on Jolt that roll, with the added mass, takes several seconds: a 20 s mean.
        ASSERT_TRUE(server.stepUntil(SimTime{60'000'000'000}).ok());
        const std::string name = server.world().bodies.names.at(0);
        double height = 0.0;
        double wet = 0.0;
        constexpr int kSamples = 400;
        for (int k = 1; k <= kSamples; ++k) {  // 20 s
            ASSERT_TRUE(server.stepUntil(SimTime{60'000'000'000 + k * 50'000'000LL}).ok());
            height += server.read("body/" + name + "/pose").at(2) / kSamples;
            wet += server.read("body/" + name + "/submerged").at(0) / kSamples;
        }
        EXPECT_NEAR(height, 2.0, 0.01) << physics;
        EXPECT_NEAR(wet, 0.5, 0.03) << physics;
    }
}

TEST(Water, HeavyThingsSinkLightOnesRiseAndDragLimitsTheSpeed) {
    // 80 kg in 64 L: it sinks to the ground, below which water does nothing.
    Server heavy(pool(80.0, 1.8, "mujoco"));
    ASSERT_TRUE(heavy.load().ok());
    ASSERT_TRUE(heavy.stepUntil(SimTime{20'000'000'000}).ok());
    const std::string name = heavy.world().bodies.names.at(0);
    EXPECT_NEAR(heavy.read("body/" + name + "/pose").at(2), 0.2, 0.01) << "on the ground";
    EXPECT_NEAR(heavy.read("body/" + name + "/submerged").at(0), 1.0, 1e-6);
    // Off, a 32 kg box falls right through.
    ServerOptions o = pool(32.0, 2.6, "mujoco");
    o.substituted.insert("hydrodynamics");
    Server off(o);
    ASSERT_TRUE(off.load().ok());
    ASSERT_TRUE(off.stepUntil(SimTime{3'000'000'000}).ok());
    EXPECT_NEAR(off.read("body/" + off.world().bodies.names.at(0) + "/pose").at(2), 0.2, 0.01);
}

// Terminal speed in the water: (m − ρV) g = ½ ρ Cd A v², with A the face it falls on. Deep enough
// that it gets there; well above the ground.
TEST(Water, ASinkingBoxReachesItsTerminalSpeed) {
    ServerOptions o = pool(80.0, 49.0, "mujoco");
    o.water = {{{0.0, 0.0, 50.0}, 10.0, 10.0, 50.0}};
    Server server(o);
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(server.stepUntil(SimTime{10'000'000'000}).ok());
    const std::string name = server.world().bodies.names.at(0);
    constexpr double kG = 9.80665;
    const double expected = std::sqrt(2.0 * (80.0 - 1000.0 * 0.064) * kG / (1000.0 * 1.0 * 0.16));
    EXPECT_NEAR(std::abs(server.read("body/" + name + "/twist").at(2)), expected, 0.03 * expected);
}

/// A world with a free sphere of `mass` kg and `radius` m, its centre at height z.
std::string sphereWorld(double mass, double radius, double z) {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() /
        ("lagrange_water_sphere_" + std::to_string(::getpid()) + ".sdf");
    const double i = 0.4 * mass * radius * radius;
    std::ofstream(path)
        << "<?xml version=\"1.0\"?><sdf version=\"1.9\"><world name=\"w\">"
        << "<model name=\"ball\"><pose>0 0 " << z << " 0 0 0</pose>"
        << "<link name=\"link\"><inertial><mass>" << mass << "</mass><inertia><ixx>" << i
        << "</ixx><iyy>" << i << "</iyy><izz>" << i << "</izz></inertia>"
        << "</inertial><collision name=\"c\"><geometry><sphere><radius>" << radius
        << "</radius></sphere></geometry></collision></link></model></world></sdf>\n";
    return path.string();
}

// Added mass (Lamb 1932 §92: ½ρV for a sphere): a sphere half as dense as water, released at
// rest under water, starts rising at g(ρ_w V − m)/(m + ½ρ_w V) = g/2, not g as it would without.
TEST(Water, AReleasedSphereRisesWithItsAddedMass) {
    constexpr double kRadius = 0.1;
    constexpr double kG = 9.80665;
    const double volume = 4.0 / 3.0 * 3.14159265358979 * kRadius * kRadius * kRadius;
    const double mass = 500.0 * volume;
    for (const char* physics : {"mujoco", "jolt"}) {
        ServerOptions o;
        o.worldPath = sphereWorld(mass, kRadius, 1.0);
        o.water.push_back({{0.0, 0.0, 2.0}, 10.0, 10.0, 2.0});
        o.ros = false;
        o.realTimeFactor = 0.0;
        o.physics = physics;
        Server server(o);
        ASSERT_TRUE(server.load().ok());
        ASSERT_TRUE(server.stepUntil(SimTime{40'000'000}).ok());  // 40 ms: drag still small
        const std::string name = server.world().bodies.names.at(0);
        const double rise = server.read("body/" + name + "/twist").at(2) / 0.04;
        const double expected = kG * (1000.0 * volume - mass) / (mass + 0.5 * 1000.0 * volume);
        EXPECT_NEAR(rise, expected, 0.03 * expected) << physics << ": without added mass " << kG;
    }
}

// In a current the drag works on the speed through the water: a floating box drifts until it
// moves with the water.
TEST(Water, AFloatingBoxDriftsWithTheCurrent) {
    ServerOptions o = pool(32.0, 2.0, "mujoco");
    o.worldPath = boxWorld(32.0, 2.0);
    o.water = {{{0.0, 0.0, 2.0}, 200.0, 200.0, 2.0}};
    o.waterCurrent = {0.5, 0.0, 0.0};
    Server server(o);
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(server.stepUntil(SimTime{40'000'000'000}).ok());
    const std::vector<double> v =
        server.read("body/" + server.world().bodies.names.at(0) + "/twist");
    EXPECT_NEAR(v.at(0), 0.5, 0.02) << "with the water";
    EXPECT_NEAR(v.at(1), 0.0, 0.02);
}

// Linear (Airy) waves much longer than a float: it rides them, heaving with the waves' own
// amplitude (a heave response of 1 at long waves). 0.1 m, 4 s waves on 10 m of water are 25 m
// long; the box is 0.4 m.
TEST(Water, ASmallFloatRidesLongWaves) {
    ServerOptions o = pool(32.0, 10.0, "mujoco");
    o.worldPath = boxWorld(32.0, 10.0);
    o.water = {{{0.0, 0.0, 10.0}, 200.0, 200.0, 10.0}};
    o.waterWaves = {0.1, 4.0, 0.0};
    Server server(o);
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(server.stepUntil(SimTime{20'000'000'000}).ok());
    const std::string name = server.world().bodies.names.at(0);
    double low = 1e9;
    double high = -1e9;
    for (int k = 1; k <= 240; ++k) {  // 12 s, three waves
        ASSERT_TRUE(server.stepUntil(SimTime{20'000'000'000 + k * 50'000'000LL}).ok());
        const double z = server.read("body/" + name + "/pose").at(2);
        low = std::min(low, z);
        high = std::max(high, z);
    }
    EXPECT_NEAR((high - low) / 2.0, 0.1, 0.01) << "heaves with the 0.1 m waves";
}

// A hull whose origin is at one end: it floats level, at the draft of its true volume.
TEST(Water, AnOffCentreHullFloatsLevel) {
    const std::filesystem::path stl =
        std::filesystem::temp_directory_path() /
        ("lagrange_water_hull_" + std::to_string(::getpid()) + ".stl");
    {
        // x 0 to 2, y ±0.5, z ±0.125: flat, as a square log at this density rolls onto a corner.
        using P = std::array<float, 3>;
        const auto at = [](int x, int y, int z) {
            return P{x != 0 ? 2.0F : 0.0F, y != 0 ? 0.5F : -0.5F, z != 0 ? 0.125F : -0.125F};
        };
        const std::vector<std::array<P, 3>> triangles{
            {at(0, 0, 0), at(0, 1, 0), at(1, 1, 0)}, {at(0, 0, 0), at(1, 1, 0), at(1, 0, 0)},
            {at(0, 0, 1), at(1, 0, 1), at(1, 1, 1)}, {at(0, 0, 1), at(1, 1, 1), at(0, 1, 1)},
            {at(0, 0, 0), at(1, 0, 0), at(1, 0, 1)}, {at(0, 0, 0), at(1, 0, 1), at(0, 0, 1)},
            {at(0, 1, 0), at(0, 1, 1), at(1, 1, 1)}, {at(0, 1, 0), at(1, 1, 1), at(1, 1, 0)},
            {at(0, 0, 0), at(0, 0, 1), at(0, 1, 1)}, {at(0, 0, 0), at(0, 1, 1), at(0, 1, 0)},
            {at(1, 0, 0), at(1, 1, 0), at(1, 1, 1)}, {at(1, 0, 0), at(1, 1, 1), at(1, 0, 1)}};
        std::ofstream file(stl, std::ios::binary);
        const std::array<char, 80> header{};
        file.write(header.data(), header.size());
        const auto count = static_cast<std::uint32_t>(triangles.size());
        file.write(reinterpret_cast<const char*>(&count), sizeof(count));
        for (const auto& t : triangles) {
            const P normal{};
            file.write(reinterpret_cast<const char*>(normal.data()), sizeof(normal));
            file.write(reinterpret_cast<const char*>(t.data()), sizeof(t));
            const std::uint16_t attribute = 0;
            file.write(reinterpret_cast<const char*>(&attribute), sizeof(attribute));
        }
    }
    const std::filesystem::path urdf = stl.string() + ".urdf";
    // 250 kg with its centre of mass at the box's middle: half of its 0.5 m³ under water.
    std::ofstream(urdf) << R"(<?xml version="1.0"?><robot name="barge"><link name="hull"><inertial>
<origin xyz="1 0 0"/><mass value="250"/><inertia ixx="10" iyy="85" izz="85" ixy="0" ixz="0" iyz="0"/>
</inertial><collision><geometry><mesh filename="file://)"
                        << stl.string() << R"("/></geometry></collision></link></robot>)";
    ServerOptions o;
    o.robotPath = urdf.string();
    o.worldPath.clear();
    o.water.push_back({{0.0, 0.0, 2.0}, 20.0, 20.0, 2.0});
    o.spawn = Pose{{0.0, 0.0, 2.1}, {1.0, 0.0, 0.0, 0.0}};
    o.demoDrive = false;
    o.ros = false;
    o.realTimeFactor = 0.0;
    Server server(o);
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.stepUntil(SimTime{90'000'000'000}).ok());
    EXPECT_NEAR(server.read("body/hull/submerged").at(0), 0.5, 0.03);
    const std::vector<double> pose = server.read("body/hull/pose");
    EXPECT_NEAR(pose.at(2), 2.0, 0.02);
    EXPECT_NEAR(pose.at(4), 0.0, 0.02) << "level across";
    EXPECT_NEAR(pose.at(5), 0.0, 0.02) << "level along: its origin is at one end";
}

/// A lidar 2 m above the water, its 21 beams fanned ±0.3 rad about straight down.
std::string lidarOverWater() {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() /
        ("lagrange_over_water_" + std::to_string(::getpid()) + ".urdf");
    std::ofstream(path) << R"(<?xml version="1.0"?><robot name="mast"><link name="world"/>
<link name="head"><inertial><mass value="1"/><inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0"
ixz="0" iyz="0"/></inertial></link><joint name="fix" type="fixed"><parent link="world"/>
<child link="head"/><origin xyz="0 0 2.5"/></joint><gazebo reference="head">
<sensor name="down" type="ray"><pose>0 0 0 0 1.5708 0</pose><update_rate>20</update_rate><ray><scan>
<horizontal><samples>21</samples><min_angle>-0.3</min_angle><max_angle>0.3</max_angle></horizontal>
</scan><range><min>0.1</min><max>30</max></range></ray></sensor></gazebo></robot>)";
    return path.string();
}

// Straight down a lidar sees the surface, faintly; at a slant, nothing: not even the ground under
// the water, whatever the ground would give. On either lidar tier.
TEST(Water, ALidarSeesAMirror) {
    for (const auto tier : {ServerOptions::Lidar::Cpu, ServerOptions::Lidar::Gpu}) {
        ServerOptions o;
        o.robotPath = lidarOverWater();
        o.water.push_back({{0.0, 0.0, 0.5}, 20.0, 20.0, 0.5});
        o.ros = false;
        o.realTimeFactor = 0.0;
        o.lidar = tier;
        Server server(o);
        const Status loaded = server.load();
        if (!loaded.ok() && tier == ServerOptions::Lidar::Gpu) {
            GTEST_SKIP() << loaded.message;
        }
        ASSERT_TRUE(loaded.ok()) << loaded.message;
        ASSERT_TRUE(server.stepUntil(SimTime{200'000'000}).ok());
        const std::vector<double> ranges = server.read("sensor/down/ranges");
        const std::vector<double> intensities = server.read("sensor/down/intensities");
        ASSERT_EQ(ranges.size(), 21U);
        EXPECT_NEAR(ranges[10], 2.0, 1e-6) << "the middle beam: the surface";
        EXPECT_NEAR(intensities[10], 0.02 / 4.0, 1e-6);
        EXPECT_FALSE(std::isfinite(ranges[0])) << "a slant beam goes away";
        EXPECT_FALSE(std::isfinite(ranges[20]));
    }
}

}  // namespace
}  // namespace lagrange::server
