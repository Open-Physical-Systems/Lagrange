// The site changed while running: gravity, weather, wind (Server::setSite).
#include "lagrange/server/Server.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <unistd.h>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

std::string fallingBox() {
    const std::filesystem::path path = std::filesystem::temp_directory_path() /
                                       ("lagrange_site_box_" + std::to_string(::getpid()) + ".sdf");
    std::ofstream(path) << R"(<?xml version="1.0"?><sdf version="1.9"><world name="w">
<model name="box"><pose>0 0 200 0 0 0</pose><link name="link"><inertial><mass>1</mass><inertia>
<ixx>0.0067</ixx><iyy>0.0067</iyy><izz>0.0067</izz></inertia></inertial><collision name="c">
<geometry><box><size>0.2 0.2 0.2</size></box></geometry></collision></link></model>
</world></sdf>)";
    return path.string();
}

// Gravity changes mid-fall, on either engine: the speed gained follows the new value, and time
// and speed carry on.
TEST(Site, GravityChangesWhileFalling) {
    for (const char* physics : {"mujoco", "jolt"}) {
        ServerOptions o;
        o.worldPath = fallingBox();
        o.ros = false;
        o.realTimeFactor = 0.0;
        o.physics = physics;
        Server server(o);
        ASSERT_TRUE(server.load().ok());
        const std::string name = server.world().bodies.names.at(0);
        ASSERT_TRUE(server.stepUntil(SimTime{1'000'000'000}).ok());
        const double before = server.read("body/" + name + "/twist").at(2);
        EXPECT_NEAR(before, -9.80665, 0.05) << physics;
        const Status set = server.setSite("gravity", 1.62);
        ASSERT_TRUE(set.ok()) << set.message;
        EXPECT_EQ(server.now().ns, 1'000'000'000) << "time goes on";
        EXPECT_NEAR(server.read("body/" + name + "/twist").at(2), before, 0.05) << "so does speed";
        ASSERT_TRUE(server.stepUntil(SimTime{2'000'000'000}).ok());
        EXPECT_NEAR(server.read("body/" + name + "/twist").at(2), before - 1.62, 0.05) << physics;
    }
}

TEST(Site, WeatherChangesAndAMistakeIsRefused) {
    ServerOptions o;
    o.robotPath = LAGRANGE_TEST_AMR_URDF;
    o.ros = false;
    o.realTimeFactor = 0.0;
    o.site = {{"temperature", 290.0}, {"pressure", 101325.0}, {"relative_humidity", 0.5}};
    Server server(o);
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.stepUntil(SimTime{500'000'000}).ok());
    EXPECT_NEAR(server.read("env/air_temperature").at(0), 290.0, 1e-9);
    ASSERT_TRUE(server.setSite("temperature", 310.0).ok());
    ASSERT_TRUE(server.setSite("rain_rate", 2.8e-6).ok());
    ASSERT_TRUE(server.stepUntil(SimTime{700'000'000}).ok());
    EXPECT_NEAR(server.read("env/air_temperature").at(0), 310.0, 1e-9);
    EXPECT_GT(server.read("env/rain_rate").at(0), 0.0);
    const Status bad = server.setSite("banana", 1.0);
    ASSERT_FALSE(bad.ok());
    EXPECT_NE(bad.message.find("one of gravity"), std::string::npos) << bad.message;
}

// The first air density brings the rotors in: the run is rebuilt keeping time, and they fly.
TEST(Site, AValueThatBringsADomainInRebuildsTheRun) {
    ServerOptions o;
    o.robotPath = LAGRANGE_TEST_QUAD_USDA;
    o.ros = false;
    o.realTimeFactor = 0.0;
    Server server(o);
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(server.stepUntil(SimTime{200'000'000}).ok());
    bool rotors = false;
    for (const auto& signal : server.signals()) {
        rotors = rotors || signal.name.starts_with("rotor/");
    }
    EXPECT_FALSE(rotors);
    const Status set = server.setSite("air_density", 1.225);
    ASSERT_TRUE(set.ok()) << set.message;
    EXPECT_EQ(server.now().ns, 200'000'000);
    for (const auto& signal : server.signals()) {
        rotors = rotors || signal.name.starts_with("rotor/");
    }
    EXPECT_TRUE(rotors);
    ASSERT_TRUE(server.stepUntil(SimTime{4'000'000'000}).ok());
    EXPECT_GT(server.read("body/base_link/pose").at(2), 0.5) << "it takes off";
}

// The wind speed alone is enough, as the guide says: the rest of the wind has its conventions. A
// wind from the west pushes a hovering quadcopter east, and it goes away again when set to 0.
TEST(Site, WindSpeedAloneIsEnoughAndItPushes) {
    ServerOptions o;
    o.robotPath = LAGRANGE_TEST_QUAD_USDA;
    o.ros = false;
    o.realTimeFactor = 0.0;
    o.airDensityKgM3 = 1.225;
    Server server(o);
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(server.stepUntil(SimTime{3'000'000'000}).ok());
    const Status set = server.setSite("wind_speed", 10.0);
    ASSERT_TRUE(set.ok()) << set.message;
    ASSERT_TRUE(server.setSite("wind_from", 270.0).ok());
    const double then = server.read("body/base_link/pose").at(2);
    EXPECT_GT(then, 0.5) << "still in the air: the wind does not restart the run";
    ASSERT_TRUE(server.stepUntil(SimTime{3'100'000'000}).ok());
    const std::string drag = "body/base_link/wrench/aero";
    ASSERT_EQ(server.read(drag).size(), 6U);
    EXPECT_GT(server.read(drag)[0], 0.1) << "blowing towards +x";
    const Status calm = server.setSite("wind_speed", 0.0);
    ASSERT_TRUE(calm.ok()) << calm.message;
    ASSERT_TRUE(server.stepUntil(SimTime{3'200'000'000}).ok());
    EXPECT_LT(std::abs(server.read(drag)[0]), 0.1) << "only its own slow drift";
}

// A checkpoint holds the site: restoring it brings back the gravity and weather of its moment, also
// when the run has changed them since, and the run then repeats exactly.
TEST(Site, ACheckpointHoldsTheSite) {
    for (const char* physics : {"mujoco", "jolt"}) {
        ServerOptions o;
        o.worldPath = fallingBox();
        o.ros = false;
        o.realTimeFactor = 0.0;
        o.physics = physics;
        Server server(o);
        ASSERT_TRUE(server.load().ok());
        const std::string name = server.world().bodies.names.at(0);
        const std::string twist = "body/" + name + "/twist";
        ASSERT_TRUE(server.setSite("gravity", 1.62).ok());
        ASSERT_TRUE(server.stepUntil(SimTime{1'000'000'000}).ok());
        const auto saved = server.saveState();
        ASSERT_TRUE(saved) << saved.error().message;
        const double speed = server.read(twist).at(2);

        ASSERT_TRUE(server.setSite("gravity", 20.0).ok());
        ASSERT_TRUE(server.stepUntil(SimTime{1'500'000'000}).ok());
        const Status restored = server.restoreState(*saved);
        ASSERT_TRUE(restored.ok()) << restored.message;
        EXPECT_EQ(server.now().ns, 1'000'000'000);
        EXPECT_DOUBLE_EQ(server.read(twist).at(2), speed) << physics;
        EXPECT_DOUBLE_EQ(server.world().gravityLocal[2], -1.62);
        ASSERT_TRUE(server.stepUntil(SimTime{2'000'000'000}).ok());
        EXPECT_NEAR(server.read(twist).at(2), speed - 1.62, 0.05)
            << physics << ": moon gravity again, not the 20 of later";
    }
}

// What was not set at the checkpoint is not set after it: rain that came later goes away, and with
// a domain that came later the restore is still whole.
TEST(Site, ACheckpointTakesBackWhatCameLater) {
    ServerOptions o;
    o.robotPath = LAGRANGE_TEST_AMR_URDF;
    o.ros = false;
    o.realTimeFactor = 0.0;
    o.site = {{"temperature", 290.0}, {"pressure", 101325.0}, {"relative_humidity", 0.5}};
    Server server(o);
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(server.stepUntil(SimTime{200'000'000}).ok());
    const auto saved = server.saveState();
    ASSERT_TRUE(saved);
    ASSERT_TRUE(server.setSite("rain_rate", 2.8e-6).ok());
    ASSERT_TRUE(server.setSite("temperature", 310.0).ok());
    ASSERT_TRUE(server.stepUntil(SimTime{400'000'000}).ok());
    EXPECT_GT(server.read("env/rain_rate").at(0), 0.0);
    const Status restored = server.restoreState(*saved);
    ASSERT_TRUE(restored.ok()) << restored.message;
    ASSERT_TRUE(server.stepUntil(SimTime{500'000'000}).ok());
    EXPECT_EQ(server.read("env/rain_rate").at(0), 0.0);
    EXPECT_NEAR(server.read("env/air_temperature").at(0), 290.0, 1e-9);
}

// On Jolt too, a rebuild that changes the modules keeps the joints where they were.
TEST(Site, ARebuildOnJoltKeepsTheJoints) {
    ServerOptions o;
    o.robotPath = LAGRANGE_TEST_AMR_URDF;
    o.ros = false;
    o.realTimeFactor = 0.0;
    o.physics = "jolt";
    Server server(o);
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(server.stepUntil(SimTime{3'000'000'000}).ok());
    const double wheel = server.read("joint/left_wheel_joint/position").at(0);
    const std::vector<double> pose = server.read("body/left_wheel/pose");
    ASSERT_GT(std::abs(wheel), 0.5) << "it has turned";
    const auto mate = server.spawnRobot(LAGRANGE_TEST_AMR_URDF, "mate", Pose{{0.0, 5.0, 0.0}, {}});
    ASSERT_TRUE(mate) << mate.error().message;
    EXPECT_NEAR(server.read("joint/left_wheel_joint/position").at(0), wheel, 1e-3);
    const std::vector<double> after = server.read("body/left_wheel/pose");
    for (std::size_t i = 0; i < 3; ++i) {
        EXPECT_NEAR(after[i], pose[i], 1e-3) << i;
    }
}

}  // namespace
}  // namespace lagrange::server
