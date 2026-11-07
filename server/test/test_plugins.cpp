#include "lagrange/server/Plugins.hpp"
#include "lagrange/server/Robot.hpp"
#include "lagrange/server/Server.hpp"
#include "lagrange/stage/Stage.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

ServerOptions amr(std::vector<std::string> plugins) {
    ServerOptions options;
    options.robotPath = LAGRANGE_TEST_AMR_URDF;
    options.realTimeFactor = 0.0;
    options.ros = false;
    options.plugins = std::move(plugins);
    options.profileSchemaPath = LAGRANGE_TEST_SCHEMA;
    options.stageName = "amr_diffdrive.urdf";
    return options;
}

std::string profile(const std::string& electrical, const std::string& plugins) {
    return "schema_version: 1\nstage: amr_diffdrive.urdf\ndomains:\n"
           "  rigid_body:   { enabled: true, backend: mujoco }\n"
           "  electrical:   " +
           electrical +
           "\n"
           "  thermal:      { enabled: false, substitute: ambient }\n"
           "  aerodynamics: { enabled: false, substitute: zero }\n"
           "  weather:      { enabled: false, substitute: clear }\n"
           "  emi:          { enabled: false, substitute: zero }\n"
           "  radiation:    { enabled: false, substitute: zero }\n"
           "  sensors:      { enabled: true }\n" +
           plugins;
}

class Plugins : public ::testing::Test {
protected:
    void SetUp() override { ::setenv("LAGRANGE_PLUGIN_PATH", LAGRANGE_TEST_PLUGIN_DIR, 1); }
};

// A plugin with a domain of its own runs in the server's run: found by name on
// LAGRANGE_PLUGIN_PATH, wired to the physics' signals, configured by the profile, and turned off
// with --domain.
TEST_F(Plugins, AModuleWithItsOwnDomainRunsAndTakesItsConfig) {
    Server server(amr({"acme_probe"}));
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << static_cast<int>(loaded.code) << " [" << loaded.message << "]";
    ASSERT_TRUE(server.stepUntil(SimTime{2'000'000'000}).ok());
    const std::vector<double> x = server.read("acme/probe/x");
    ASSERT_EQ(x.size(), 1U) << "the plugin's output is on the bus";
    EXPECT_NEAR(x[0], server.read("body/base_link/pose")[0], 0.02);

    const Status applied = server.applyProfile(
        profile("{ enabled: false, substitute: ideal }",
                "plugins:\n  - { library: acme_probe, config: { offset: 100.0 } }\n"));
    ASSERT_TRUE(applied.ok()) << applied.message;
    ASSERT_TRUE(server.stepUntil(SimTime{500'000'000}).ok());
    EXPECT_NEAR(server.read("acme/probe/x")[0], server.read("body/base_link/pose")[0] + 100.0,
                0.02);

    // A profile cannot load a library the server was not started with (a viewer sends them).
    const Status refused = server.applyProfile(profile(
        "{ enabled: false, substitute: ideal }", "plugins:\n  - { library: other_plugin }\n"));
    EXPECT_FALSE(refused.ok());
    EXPECT_NE(refused.message.find("is not loaded"), std::string::npos) << refused.message;

    ServerOptions off = amr({"acme_probe"});
    off.substituted.insert("acme_probe");
    Server without(off);
    ASSERT_TRUE(without.load().ok());
    EXPECT_TRUE(without.read("acme/probe/x").empty()) << "--domain acme_probe=off";
}

// A plugin substitute of a built-in domain (the ABI sample: the ideal torque source in C) stands
// in when the profile names it.
TEST_F(Plugins, APluginSubstituteStandsInForABuiltInDomain) {
    Server server(amr({LAGRANGE_TEST_PLUGIN_DIR "/libideal_torque_source.so"}));
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << static_cast<int>(loaded.code) << " [" << loaded.message << "]";
    const Status applied = server.applyProfile(
        profile("{ enabled: false, substitute: ideal_torque_source }",
                "plugins:\n  - { library: " LAGRANGE_TEST_PLUGIN_DIR
                "/libideal_torque_source.so, config: { joint: left_wheel_joint } }\n"));
    ASSERT_TRUE(applied.ok()) << applied.message;
    EXPECT_NE(server.wiringDot().find("electrical/ideal_torque_source"), std::string::npos)
        << server.wiringDot();
    ASSERT_TRUE(server.stepUntil(SimTime{1'000'000'000}).ok());
    const std::vector<double> torque = server.read("joint/left_wheel_joint/torque");
    ASSERT_EQ(torque.size(), 1U);
    EXPECT_EQ(torque[0], server.read("joint/left_wheel_joint/torque_command")[0]);
    EXPECT_NE(torque[0], 0.0) << "the drive commands a torque and the plugin passes it on";
}

// ABI 1.1 (RFC 0008 §2): a plugin reads the world's tables and casts rays at the scene, with the
// robot's colliders where physics put them.
TEST_F(Plugins, APluginSeesTheWorldAndCastsRays) {
    ServerOptions options = amr({"scene_probe"});
    Server server(options);
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.stepUntil(SimTime{1'000'000'000}).ok());
    const World& world = server.world();
    const auto value = [&server](const std::string& name) {
        const std::vector<double> v = server.read(name);
        return v.empty() ? -1e9 : v[0];
    };
    EXPECT_EQ(value("probe/world/bodies"), static_cast<double>(world.bodies.names.size()));
    EXPECT_EQ(value("probe/world/colliders"), static_cast<double>(world.colliders.body.size()));
    EXPECT_EQ(value("probe/world/sensors"), static_cast<double>(world.sensors.names.size()));
    EXPECT_EQ(value("probe/world/lidars"), 2.0) << "the AMR's front and top lidars";
    EXPECT_EQ(value("probe/world/actuated"), 2.0) << "its two wheels";
    EXPECT_DOUBLE_EQ(value("probe/site/gravity_z"), world.gravityLocal[2]);

    EXPECT_NEAR(value("probe/ray/ground"), 5.0, 1e-9) << "the ground plane, 5 m down";
    const double robot = value("probe/ray/robot");
    EXPECT_GT(robot, 3.5);
    EXPECT_LT(robot, 5.0) << "the ray meets the robot before its origin";
    const double body = value("probe/ray/robot_body");
    ASSERT_GE(body, 0.0);
    ASSERT_LT(body, static_cast<double>(world.bodies.names.size()));
    EXPECT_EQ(value("probe/rays/hits"), 2.0) << "down and back hit; up and sideways do not";
    EXPECT_GE(value("probe/overlap"), 1.0) << "the base's own collider holds its origin";
}

// The C++ SDK reads the same world through the C ABI.
TEST_F(Plugins, TheSdkReadsTheWholeWorld) {
    Server server(amr({"sdk_probe"}));
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    const World& world = server.world();
    EXPECT_EQ(server.read("sdk/bodies").at(0), static_cast<double>(world.bodies.names.size()));
    EXPECT_EQ(server.read("sdk/joints").at(0), static_cast<double>(world.joints.names.size()));
    EXPECT_EQ(server.read("sdk/colliders").at(0), static_cast<double>(world.colliders.body.size()));
    EXPECT_EQ(server.read("sdk/sensors").at(0), static_cast<double>(world.sensors.names.size()));
    double mass = 0.0;
    for (const double m : world.bodies.mass) {
        mass += m;
    }
    EXPECT_DOUBLE_EQ(server.read("sdk/mass").at(0), mass);
}

// Custom sensors (RFC 0008 §3): a <sensor> of a type Lagrange does not simulate is kept, with
// its parameters by path, for a plugin to implement.
TEST_F(Plugins, UnknownSensorTypesBecomeCustomSensors) {
    const auto urdf = loadModel(LAGRANGE_TEST_DATA "/tof_box.urdf");
    ASSERT_TRUE(urdf) << urdf.error();
    const World::Sensors& sensors = urdf->world.sensors;
    ASSERT_EQ(sensors.names.size(), 1U);
    EXPECT_EQ(sensors.type[0], SensorType::Custom);
    EXPECT_EQ(sensors.kind[0], "tof");
    EXPECT_EQ(sensors.rateHz[0], 50.0);
    EXPECT_EQ(sensors.settings[0].at("range/max"), 4.0);
    EXPECT_EQ(sensors.settings[0].at("fov"), 0.0);
    EXPECT_EQ(sensors.text[0].at("noise/type"), "gaussian");

    const auto sdf = loadModel(LAGRANGE_TEST_DATA "/sonar_box.sdf");
    ASSERT_TRUE(sdf) << sdf.error();
    ASSERT_EQ(sdf->world.sensors.names.size(), 1U);
    EXPECT_EQ(sdf->world.sensors.type[0], SensorType::Custom);
    EXPECT_EQ(sdf->world.sensors.kind[0], "sonar");
    EXPECT_EQ(sdf->world.sensors.settings[0].at("max"), 5.0);
    EXPECT_EQ(sdf->world.sensors.settings[0].at("radius"), 0.3);

    // Through USD and back: CustomSensorAPI_1 keeps the kind and every parameter.
    const std::string usda =
        (std::filesystem::temp_directory_path() / "lagrange_tof_box.usda").string();
    ASSERT_TRUE(stage::writeStage(*urdf, usda));
    const auto back = loadModel(usda);
    ASSERT_TRUE(back) << back.error();
    ASSERT_EQ(back->world.sensors.names.size(), 1U);
    EXPECT_EQ(back->world.sensors.type[0], SensorType::Custom);
    EXPECT_EQ(back->world.sensors.kind[0], "tof");
    EXPECT_EQ(back->sensorPrim.size(), back->world.sensors.names.size()) << "a prim per sensor";
    EXPECT_EQ(back->world.sensors.settings[0], sensors.settings[0]);
    EXPECT_EQ(back->world.sensors.text[0], sensors.text[0]);
    std::filesystem::remove(usda);
}

// The time-of-flight plugin (core/plugin_abi/samples/tof_sensor.c, found by name in the
// lagrange_plugins index): its range to the ground is the exact distance along its beam, and it
// does not see its own body.
TEST_F(Plugins, TheTofPluginMeasuresTheGround) {
    ServerOptions options = amr({"lagrange_tof"});
    options.robotPath = LAGRANGE_TEST_DATA "/tof_box.urdf";
    options.overlayPath.clear();
    Server server(options);
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.stepUntil(SimTime{2'000'000'000}).ok());  // it has settled
    const std::vector<double> body = server.read("body/base_link/pose");
    ASSERT_EQ(body.size(), 7U);
    const Pose base{{body[0], body[1], body[2]}, {body[3], body[4], body[5], body[6]}};
    const Pose sensor = compose(base, server.world().sensors.poseInBody[0]);
    const std::array<double, 3> beam = rotate(sensor.orientation, {1.0, 0.0, 0.0});
    ASSERT_LT(beam[2], -0.4) << "it looks down";
    const double expected = -sensor.position[2] / beam[2];  // to the plane z = 0
    const std::vector<double> range = server.read("sensor/front_tof/range");
    ASSERT_EQ(range.size(), 1U);
    EXPECT_NEAR(range[0], expected, 1e-6);
    EXPECT_EQ(server.read("sensor/front_tof/valid").at(0), 1.0);
}

// An ultrasonic sensor takes the nearest thing in its cone, and reads nothing inside its dead zone.
TEST_F(Plugins, TheUltrasonicKindHasACone) {
    const auto write = [](const char* name, double height) {
        const auto file = std::filesystem::temp_directory_path() / name;
        std::ofstream(file) << "<robot name='r'><link name='base_link'><inertial><mass value='5'/>"
                               "<inertia ixx='1' iyy='1' izz='1' ixy='0' ixz='0' iyz='0'/>"
                               "</inertial><collision><geometry><box size='0.4 0.4 0.2'/>"
                               "</geometry></collision></link><gazebo reference='base_link'>"
                               "<sensor name='sonar' type='ultrasonic'><pose>0.25 0 "
                            << height
                            << " 0 1.5708 0</pose><update_rate>50</update_rate><ultrasonic/>"
                               "</sensor></gazebo></robot>";
        return file.string();
    };
    const auto read = [&](const char* name, double height) {
        ServerOptions options = amr({"lagrange_tof"});
        options.robotPath = write(name, height);
        options.overlayPath.clear();
        Server server(options);
        EXPECT_TRUE(server.load().ok());
        EXPECT_TRUE(server.stepUntil(SimTime{1'000'000'000}).ok());
        return std::array{server.read("sensor/sonar/range").at(0),
                          server.read("sensor/sonar/valid").at(0)};
    };
    // The box's lowest face is on the ground, so its centre is 0.1 m up: 0.5 m above that.
    const auto far = read("lagrange_sonar_far.urdf", 0.5);
    EXPECT_EQ(far[1], 1.0);
    EXPECT_NEAR(far[0], 0.6, 0.02);
    const auto close = read("lagrange_sonar_close.urdf", 0.0);  // 0.1 m: inside the 0.2 m dead zone
    EXPECT_EQ(close[1], 0.0);
}

// A plugin's sensor is delayed like any other, and what arrives is what was sampled then.
TEST_F(Plugins, APluginSensorCanBeDelayed) {
    ServerOptions options = amr({"lagrange_tof"});
    options.robotPath = LAGRANGE_TEST_DATA "/tof_box.urdf";
    options.overlayPath.clear();
    options.sensorTiming.push_back({"front_tof", 0.1, 0.0});
    Server server(options);
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.stepUntil(SimTime{2'000'000'000}).ok());
    const double stamp = server.read("sensor/front_tof/delayed/stamp").at(0);
    EXPECT_GE(stamp, 2.0 - 0.1 - 0.02 - 0.002);  // 50 Hz, and the 1 ms tick
    EXPECT_LE(stamp, 2.0 - 0.1 + 1e-9);
    EXPECT_NEAR(server.read("sensor/front_tof/delayed/range").at(0),
                server.read("sensor/front_tof/range").at(0), 0.05);
}

TEST_F(Plugins, LoadingFailuresNameTheLibrary) {
    Server missing(amr({"no_such_plugin"}));
    const Status status = missing.load();
    ASSERT_FALSE(status.ok());
    EXPECT_NE(status.message.find("no_such_plugin"), std::string::npos) << status.message;

    ServerOptions engine = amr({});
    engine.physics = "bullet";
    Server noEngine(engine);
    const Status refused = noEngine.load();
    ASSERT_FALSE(refused.ok());
    EXPECT_NE(refused.message.find("no rigid body backend bullet"), std::string::npos)
        << refused.message;

    EXPECT_TRUE(resolvePlugin("acme_probe"));
    EXPECT_FALSE(resolvePlugin("/no/such/libplugin.so"));
}

}  // namespace
}  // namespace lagrange::server
