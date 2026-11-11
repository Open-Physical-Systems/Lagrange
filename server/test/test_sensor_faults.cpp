// Sensor faults in a whole run (sensors.md "Sensor faults (as built)"): from the command line
// and from a profile, on the sample AMR's IMU and lidar, and refused when they name nothing.
#include "lagrange/server/Server.hpp"

#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

ServerOptions amr(std::vector<std::string> faults) {
    ServerOptions options;
    options.robotPath = LAGRANGE_TEST_AMR_URDF;
    options.realTimeFactor = 0.0;
    options.ros = false;
    options.profileSchemaPath = LAGRANGE_TEST_SCHEMA;
    options.stageName = "amr_diffdrive.urdf";
    options.sensorFaults = std::move(faults);
    options.demoDrive = false;  // at rest
    return options;
}

TEST(SensorFaultRun, ImuBiasAndLidarDisconnectReachWhatEveryoneReads) {
    Server server(amr({"imu:angular_rate=bias,value=0.1,at=0.5,component=2",
                       "front_lidar:ranges=disconnect,at=1,nan=1"}));
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.stepUntil(SimTime{400'000'000}).ok());
    EXPECT_NEAR(server.read("sensor/imu/angular_rate").at(2), 0.0, 0.02);
    EXPECT_EQ(server.read("sensor/imu/fault/active").at(0), 0.0);
    EXPECT_FALSE(std::isnan(server.read("sensor/front_lidar/ranges").at(0)));
    ASSERT_TRUE(server.stepUntil(SimTime{1'500'000'000}).ok());
    EXPECT_NEAR(server.read("sensor/imu/angular_rate").at(2), 0.1, 0.02);
    EXPECT_EQ(server.read("sensor/imu/fault/active").at(0), 1.0);
    for (const double r : server.read("sensor/front_lidar/ranges")) {
        EXPECT_TRUE(std::isnan(r));
    }
    EXPECT_GT(server.read("sensor/front_lidar/fault/dropped").at(0), 0.0);
    // The other lidar has no fault and no fault signals.
    EXPECT_FALSE(std::isnan(server.read("sensor/top_lidar/ranges").at(0)));
    EXPECT_TRUE(server.read("sensor/top_lidar/fault/active").empty());
}

TEST(SensorFaultRun, AFaultThatNamesNothingStopsTheStart) {
    Server noSensor(amr({"compass=disconnect"}));
    Status status = noSensor.load();
    ASSERT_FALSE(status.ok());
    EXPECT_NE(status.message.find("no sensor compass"), std::string::npos) << status.message;

    Server noQuantity(amr({"imu:heading=stuck"}));
    status = noQuantity.load();
    ASSERT_FALSE(status.ok());
    EXPECT_NE(status.message.find("has no quantity heading"), std::string::npos) << status.message;
    EXPECT_NE(status.message.find("angular_rate"), std::string::npos) << status.message;
}

std::string profile(const std::string& faults) {
    return "schema_version: 1\nstage: amr_diffdrive.urdf\ndomains:\n"
           "  rigid_body:   { enabled: true, backend: mujoco }\n"
           "  electrical:   { enabled: false, substitute: ideal }\n"
           "  thermal:      { enabled: false, substitute: ambient }\n"
           "  aerodynamics: { enabled: false, substitute: zero }\n"
           "  weather:      { enabled: false, substitute: clear }\n"
           "  emi:          { enabled: false, substitute: zero }\n"
           "  radiation:    { enabled: false, substitute: zero }\n"
           "  sensors:      { enabled: true }\n" +
           faults;
}

TEST(SensorFaultRun, AProfileSetsTheFaults) {
    Server server(amr({}));
    ASSERT_TRUE(server.load().ok());
    const Status applied = server.applyProfile(
        profile("sensor_faults:\n"
                "  - { sensor: imu, quantity: specific_force, mode: bias, value: -2.0, "
                "component: 2 }\n"
                "  - { sensor: front_lidar, mode: dropout, at: 0.2, for: 0.3 }\n"));
    ASSERT_TRUE(applied.ok()) << applied.message;
    ASSERT_TRUE(server.stepUntil(SimTime{1'000'000'000}).ok());
    EXPECT_NEAR(server.read("sensor/imu/specific_force").at(2), 9.80665 - 2.0, 0.2);
    EXPECT_GT(server.read("sensor/front_lidar/fault/dropped").at(0), 0.0);
    EXPECT_EQ(server.read("sensor/front_lidar/fault/active").at(0), 0.0) << "window over";

    // The schema asks for the size of a bias, and the run for a sensor that exists.
    EXPECT_FALSE(server
                     .applyProfile(profile("sensor_faults:\n  - { sensor: imu, quantity: "
                                           "angular_rate, mode: bias }\n"))
                     .ok());
    const Status missing =
        server.applyProfile(profile("sensor_faults:\n  - { sensor: sonar, mode: stuck }\n"));
    EXPECT_FALSE(missing.ok());
    EXPECT_NE(missing.message.find("no sensor sonar"), std::string::npos) << missing.message;
    // Refused, the run goes on with the faults it had.
    ASSERT_TRUE(server.stepUntil(SimTime{1'500'000'000}).ok());
    EXPECT_NEAR(server.read("sensor/imu/specific_force").at(2), 9.80665 - 2.0, 0.2);
}

}  // namespace
}  // namespace lagrange::server
