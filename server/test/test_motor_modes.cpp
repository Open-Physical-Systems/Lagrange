// Motors in velocity and voltage mode, and on prismatic joints, against the closed forms of a
// brushed DC motor (electrical.md): stall I = V / R, no-load speed V / (N Ke).
#include "lagrange/server/Server.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <string>
#include <unistd.h>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

constexpr double kR = 0.5;      // ohm
constexpr double kKt = 0.05;    // N·m/A, and V·s/rad: a brushed motor's Ke
constexpr double kBusV = 24.0;  // V: the bus gives this with its battery's r0 almost nothing

std::filesystem::path scratch(const std::string& name) {
    const std::filesystem::path dir =
        std::filesystem::temp_directory_path() / ("lagrange_motor_" + std::to_string(::getpid()));
    std::filesystem::create_directories(dir);
    return dir / name;
}

/// A joint with a motor in `mode` on a stiff 24 V bus; `gear` is N, or motor rad per m on a slide.
ServerOptions rig(const std::string& joint, const std::string& axis, const std::string& mode,
                  double gear, const std::string& physics = "mujoco", double currentLimit = 100.0) {
    const std::filesystem::path urdf = scratch(joint + "_" + mode + ".urdf");
    std::ofstream(urdf) << R"(<?xml version="1.0"?><robot name="rig"><link name="world"/>
<link name="head"><inertial><mass value="2"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0"
iyz="0"/></inertial></link><joint name="move" type=")"
                        << joint << R"("><parent link="world"/><child link="head"/>
<origin xyz="0 0 1"/><axis xyz=")"
                        << axis << R"("/><limit lower="-1000" upper="1000" effort="0" velocity="0"/>
</joint></robot>)";
    const std::filesystem::path overlay = scratch(joint + "_" + mode + ".overlay.yaml");
    std::ofstream(overlay) << R"(schema_version: 1
apply:
  - create: power/battery
    schemas:
      BatteryAPI_1:
        elec:battery:chemistry: lfp
        elec:battery:seriesCells: 1
        elec:battery:capacity: 3600000.0
        elec:battery:ocvSoc: [0.0, 1.0]
        elec:battery:ocvVoltage: [24.0, 24.0]
        elec:battery:r0: 0.00001
        elec:battery:cutoffVoltage: 1.0
        elec:battery:maxChargeCurrent: 1000.0
      PortAPI_1:dc_out:
        elec:port:dc_out:nominalVoltage: 24.0
        elec:port:dc_out:connectedTo: power/dc
  - create: power/dc
    schemas:
      ElectricalBusAPI_1:
        elec:bus:capacitance: 0.1
        elec:bus:undervoltageTrip: 1.0
        elec:bus:overvoltageTrip: 1000.0
  - prim: move
    create_child: motor
    schemas:
      ElectricalMotorAPI_1:
        elec:motorType: dc_brushed
        elec:windingResistance: 0.5
        elec:inductance: 0.0005
        elec:torqueConstant: 0.05
        elec:gearRatio: )" << gear
                           << R"(
        elec:maxCurrent: )" << currentLimit
                           << R"(
        elec:referenceTemperature: 298.15
        elec:resistanceTempCoeff: 0.0
        elec:controlMode: )"
                           << mode << R"(
        elec:joint: move
        elec:powerPort: power/dc
)";
    ServerOptions o;
    o.robotPath = urdf.string();
    o.overlayPath = overlay.string();
    o.worldPath.clear();
    o.physics = physics;
    o.jointControl = true;
    o.demoDrive = false;
    o.ros = false;
    o.realTimeFactor = 0.0;
    return o;
}

TEST(MotorModes, VoltageModeHitsTheNoLoadSpeedAndDrawsNothing) {
    constexpr double kGear = 10.0;
    constexpr double kVolts = 12.0;
    Server server(rig("continuous", "0 0 1", "voltage", kGear));
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.commandJoint("move", "voltage", kVolts));
    ASSERT_TRUE(server.stepUntil(SimTime{60'000'000'000}).ok());
    // The motor's time constant is J R / (N Kt)² = 1 × 0.5 / 0.25 = 2 s.
    EXPECT_NEAR(server.read("joint/move/velocity").at(0), kVolts / (kGear * kKt), 0.02);
    EXPECT_NEAR(server.read("motor/motor/current_q").at(0), 0.0, 0.05);
}

TEST(MotorModes, VoltageModeAtStallDrawsVoltsOverOhms) {
    constexpr double kGear = 10.0;
    constexpr double kVolts = 6.0;
    Server server(rig("continuous", "0 0 1", "voltage", kGear));
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(server.commandJoint("move", "voltage", kVolts));
    // Past L / R = 1 ms; the rotor has barely moved after 20 ms.
    ASSERT_TRUE(server.stepUntil(SimTime{20'000'000}).ok());
    const double iq = server.read("motor/motor/current_q").at(0);
    const double speed = server.read("joint/move/velocity").at(0);
    EXPECT_NEAR(iq, (kVolts - kGear * kKt * speed) / kR, 0.1);
    EXPECT_NEAR(server.read("joint/move/torque").at(0), kGear * kKt * iq, 0.1);
    EXPECT_NEAR(iq, kVolts / kR, 0.5);
}

TEST(MotorModes, VelocityModeHoldsItsSpeedOnBothEngines) {
    for (const char* physics : {"mujoco", "jolt"}) {
        Server server(rig("continuous", "0 0 1", "velocity", 10.0, physics));
        const Status loaded = server.load();
        ASSERT_TRUE(loaded.ok()) << physics << ": " << loaded.message;
        ASSERT_TRUE(server.commandJoint("move", "velocity", 3.0));
        ASSERT_TRUE(server.stepUntil(SimTime{3'000'000'000}).ok());
        EXPECT_NEAR(server.read("joint/move/velocity").at(0), 3.0, 0.02) << physics;
        // At steady speed it asks almost no torque of the current loop.
        EXPECT_NEAR(server.read("joint/move/torque").at(0), 0.0, 0.2) << physics;
    }
}

TEST(MotorModes, VelocityModeIsHeldByTheCurrentLimit) {
    // 1 A × 10 × 0.05 = 0.5 N·m on 1 kg·m²: 0.5 rad/s² however far the target is.
    Server server(rig("continuous", "0 0 1", "velocity", 10.0, "mujoco", 1.0));
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(server.commandJoint("move", "velocity", 20.0));
    ASSERT_TRUE(server.stepUntil(SimTime{2'000'000'000}).ok());
    EXPECT_NEAR(server.read("joint/move/velocity").at(0), 0.5 * 2.0, 0.03);
    EXPECT_NEAR(server.read("motor/motor/current_q").at(0), 1.0, 0.02);
}

TEST(MotorModes, APositionTargetGoesThroughTheSpeedLoop) {
    Server server(rig("continuous", "0 0 1", "velocity", 10.0));
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(server.commandJoint("move", "position", 1.0));
    ASSERT_TRUE(server.stepUntil(SimTime{3'000'000'000}).ok());
    EXPECT_NEAR(server.read("joint/move/position").at(0), 1.0, 0.01);
}

// A screw of 5 mm lead: 2π / 0.005 motor radians per metre, on 2 kg.
TEST(MotorModes, ASlideIsDrivenThroughItsScrewOnBothEngines) {
    const double gear = 2.0 * std::numbers::pi / 0.005;
    for (const char* physics : {"mujoco", "jolt"}) {
        constexpr double kVolts = 12.0;
        Server server(rig("prismatic", "1 0 0", "voltage", gear, physics));
        const Status loaded = server.load();
        ASSERT_TRUE(loaded.ok()) << physics << ": " << loaded.message;
        ASSERT_TRUE(server.commandJoint("move", "voltage", kVolts));
        ASSERT_TRUE(server.stepUntil(SimTime{1'000'000'000}).ok());
        // τ = J R / (N Kt)² = 2 × 0.5 / (62.8)² is tiny: it is at its no-load speed.
        EXPECT_NEAR(server.read("joint/move/velocity").at(0), kVolts / (gear * kKt), 2e-3)
            << physics;
    }
    Server server(rig("prismatic", "1 0 0", "velocity", gear));
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(server.commandJoint("move", "velocity", 0.1));
    ASSERT_TRUE(server.stepUntil(SimTime{1'000'000'000}).ok());
    EXPECT_NEAR(server.read("joint/move/velocity").at(0), 0.1, 2e-3);
}

// A controller that writes only torques cannot run a velocity motor: the start says so.
TEST(MotorModes, WithNothingToCommandItTheStartStops) {
    ServerOptions o = rig("continuous", "0 0 1", "velocity", 10.0);
    o.jointControl = false;
    Server server(o);
    const Status loaded = server.load();
    ASSERT_FALSE(loaded.ok());
    EXPECT_NE(loaded.message.find("velocity_command"), std::string::npos) << loaded.message;
}

TEST(MotorModes, ElectricalOffTheJointStillGoes) {
    ServerOptions o = rig("continuous", "0 0 1", "velocity", 10.0);
    o.substituted.insert("electrical");
    Server server(o);
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.commandJoint("move", "velocity", 2.0));
    ASSERT_TRUE(server.stepUntil(SimTime{2'000'000'000}).ok());
    EXPECT_NEAR(server.read("joint/move/velocity").at(0), 2.0, 0.02);
}

}  // namespace
}  // namespace lagrange::server
