// Water: buoyancy against Archimedes, and what a lidar sees of it.
#include "lagrange/server/Server.hpp"

#include <cmath>
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
        ASSERT_TRUE(server.stepUntil(SimTime{20'000'000'000}).ok());
        const std::string name = server.world().bodies.names.at(0);
        EXPECT_NEAR(server.read("body/" + name + "/pose").at(2), 2.0, 0.01) << physics;
        EXPECT_NEAR(server.read("body/" + name + "/submerged").at(0), 0.5, 0.03) << physics;
        EXPECT_NEAR(server.read("body/" + name + "/twist").at(2), 0.0, 0.02) << physics;
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
// the water, whatever the ground would give.
TEST(Water, ALidarSeesAMirror) {
    ServerOptions o;
    o.robotPath = lidarOverWater();
    o.water.push_back({{0.0, 0.0, 0.5}, 20.0, 20.0, 0.5});
    o.ros = false;
    o.realTimeFactor = 0.0;
    Server server(o);
    const Status loaded = server.load();
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

}  // namespace
}  // namespace lagrange::server
