// RFC 0020 §4.3 in whole runs, on MuJoCo, with the sample pan-tilt (two hobby servos, brushed
// motors in voltage mode on a 2S battery) and a smart servo fitted to ROBOTIS's XM430-W350 table
// with its loop on its own drive (elec:controlMode position):
//   - a 60° step of the pan, saturated until within 1/Kp, against the brushed motor's closed form
//     θ = ω0 (t − τm (1 − e^(−t/τm))) at the bus voltage of the run, and the same move with the
//     electrical domain off (the ideal fidelity, from the overlay's stall torque and no-load
//     speed) agreeing with it;
//   - the tilt holding its camera block against gravity at the dead zone's edge plus
//     τ_g / (Kp τ_stall), still (no hunting), and a step inside the dead zone not moving;
//   - the smart servo's no-load speed at the bus voltage (46 rev/min × V / 12 V) and its
//     profile's cruise speed within 1 %, its current limit, and its overload shutdown letting go
//     of a load past stall;
//   - the same run twice bit for bit; the domain off turns the torque off.
#include "lagrange/server/Server.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <string>
#include <unistd.h>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kG = 9.80665;
constexpr double kRpm = 2.0 * kPi / 60.0;

// The sample's motors (pan_tilt.overlay.yaml): R, N·Kt, gear efficiency; the boards' Kp and dead
// zone.
constexpr double kResistance = 9.6;
constexpr double kBackEmf = 200.0 * 0.0034;
constexpr double kEfficiency = 0.7;
constexpr double kKp = 20.0;
constexpr double kDeadband = 0.01;
constexpr double kResolution = 0.0;  // an analog potentiometer

ServerOptions panTilt() {
    ServerOptions o;
    o.robotPath = LAGRANGE_TEST_PAN_TILT_URDF;
    o.overlayPath = LAGRANGE_TEST_PAN_TILT_OVERLAY;
    o.ros = false;
    o.realTimeFactor = 0.0;
    o.jointControl = true;
    return o;
}

double at(Server& server, const std::string& signal) {
    return server.read(signal).at(0);
}

/// Steps in 1 ms until `signal` reaches `value` (rising) or `limit` s pass; the time it took.
double timeToReach(Server& server, const std::string& signal, double value, double limit) {
    const SimTime start = server.simTime();
    for (int k = 0; k < static_cast<int>(limit * 1000.0); ++k) {
        EXPECT_TRUE(server.stepUntil(SimTime{server.simTime().ns + 1'000'000}).ok());
        if (at(server, signal) >= value) {
            return static_cast<double>(server.simTime().ns - start.ns) * 1e-9;
        }
    }
    return -1.0;
}

/// The time for θ = ω0 (t − τm (1 − e^(−t/τm))) to reach `angle`.
double closedForm(double angle, double noLoad, double tm) {
    double t = angle / noLoad + tm;
    for (int i = 0; i < 60; ++i) {
        const double theta = noLoad * (t - tm * (1.0 - std::exp(-t / tm)));
        const double omega = noLoad * (1.0 - std::exp(-t / tm));
        t -= (theta - angle) / omega;
    }
    return t;
}

TEST(ServosRun, PanStepFollowsTheMotorsClosedForm) {
    Server server(panTilt());
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.stepUntil(SimTime{200'000'000}).ok());
    const double volts = at(server, "bus/dc_6v/voltage");
    EXPECT_NEAR(volts, 6.56, 0.05);  // 2 × 3.28 V at half charge
    const double noLoad = volts / kBackEmf;
    const double stall = kEfficiency * kBackEmf * volts / kResistance;
    // The pan turns the bracket and the camera block about z: 1.0e-5 + 3.0e-5 + 0.1 × 0.03².
    const double inertia = 1.0e-5 + 3.0e-5 + 0.1 * 0.03 * 0.03;
    const double tm = inertia * noLoad / stall;
    const double target = kPi / 3.0;
    ASSERT_TRUE(server.commandJoint("pan", "position", target));
    const double reach = target - 1.0 / kKp - kDeadband;
    const double t = timeToReach(server, "joint/pan/position", reach, 1.0);
    const double expected = closedForm(reach, noLoad, tm);
    std::printf(
        "pan 60° step: reached %.4f rad in %.4f s, closed form %.4f s (V %.3f, w0 %.3f "
        "rad/s, stall %.4f N·m, tm %.4f s)\n",
        reach, t, expected, volts, noLoad, stall, tm);
    EXPECT_NEAR(t, expected, 0.03 * expected);
    // It settles inside the dead zone and stays: no hunting, the drive at 0 V.
    ASSERT_TRUE(server.stepUntil(SimTime{server.simTime().ns + 600'000'000}).ok());
    EXPECT_LE(std::abs(target - at(server, "joint/pan/position")), kDeadband + kResolution);
    double worst = 0.0;
    for (int k = 0; k < 200; ++k) {
        ASSERT_TRUE(server.stepUntil(SimTime{server.simTime().ns + 1'000'000}).ok());
        worst = std::max(worst, std::abs(at(server, "joint/pan/voltage_command")));
    }
    EXPECT_EQ(worst, 0.0);
    EXPECT_NEAR(at(server, "joint/pan/velocity"), 0.0, 1e-3);
    EXPECT_NEAR(at(server, "act/pan/present_position"), at(server, "joint/pan/position"),
                kResolution / 2.0 + 1e-12);
}

TEST(ServosRun, IdealFidelityAgreesWithTheElectricalOne) {
    // The electrical domain off: the boards drive the torque line of the overlay's
    // stallTorque 0.3253 N·m and noLoadSpeed 9.647 rad/s (the motor's at 6.56 V).
    ServerOptions o = panTilt();
    o.substituted.insert("electrical");
    Server server(o);
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.stepUntil(SimTime{200'000'000}).ok());
    const double target = kPi / 3.0;
    ASSERT_TRUE(server.commandJoint("pan", "position", target));
    const double reach = target - 1.0 / kKp - kDeadband;
    const double t = timeToReach(server, "joint/pan/position", reach, 1.0);
    const double inertia = 1.0e-5 + 3.0e-5 + 0.1 * 0.03 * 0.03;
    const double expected = closedForm(reach, 9.647, inertia * 9.647 / 0.3253);
    std::printf("ideal pan 60° step: %.4f s, closed form %.4f s\n", t, expected);
    EXPECT_NEAR(t, expected, 0.02 * expected);
}

TEST(ServosRun, TiltHoldsItsLoadAtTheDeadZonesEdge) {
    Server server(panTilt());
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(server.stepUntil(SimTime{1'500'000'000}).ok());
    const double volts = at(server, "bus/dc_6v/voltage");
    const double stall = kEfficiency * kBackEmf * volts / kResistance;
    // The block's weight 3 cm ahead of the tilt axis (y): it pitches the block down, +θ about y.
    // Held still, the gear's losses side with the motor or against it as the load creeps either
    // way, so the sag lies between the dead zone's edge plus η²·τ_g/(Kp τs) (the load
    // back-driving the gear, joint torque = motor's / η) and plus τ_g/(Kp τs) (motoring, × η).
    const double load = 0.1 * kG * 0.03;
    const double sag = at(server, "joint/tilt/position");
    const double low = kDeadband + kEfficiency * kEfficiency * load / (kKp * stall);
    const double high = kDeadband + load / (kKp * stall);
    std::printf("tilt holds %.4f N·m at %.5f rad, between %.5f and %.5f\n", load, sag, low, high);
    EXPECT_GE(sag, low);
    EXPECT_LE(sag, high);
    // Still: within 0.1 mrad over the next half second, the drive pushing against the sag.
    double lowest = sag;
    double highest = sag;
    for (int k = 0; k < 500; ++k) {
        ASSERT_TRUE(server.stepUntil(SimTime{server.simTime().ns + 1'000'000}).ok());
        lowest = std::min(lowest, at(server, "joint/tilt/position"));
        highest = std::max(highest, at(server, "joint/tilt/position"));
        EXPECT_LT(at(server, "joint/tilt/voltage_command"), 0.0);
    }
    EXPECT_LT(highest - lowest, 1e-4);
    EXPECT_LT(at(server, "act/tilt/present_current"), 0.0);
    EXPECT_LT(at(server, "act/tilt/present_load"), 0.0);
    // A step inside the dead zone moves nothing.
    const double pan = at(server, "joint/pan/position");
    ASSERT_TRUE(server.commandJoint("pan", "position", pan + 0.5 * kDeadband));
    ASSERT_TRUE(server.stepUntil(SimTime{server.simTime().ns + 300'000'000}).ok());
    EXPECT_NEAR(at(server, "joint/pan/position"), pan, 1e-4);
}

TEST(ServosRun, SameRunTwiceIsBitIdenticalAndOffMeansTorqueOff) {
    std::uint64_t hashes[2] = {0, 0};
    for (std::uint64_t& hash : hashes) {
        Server server(panTilt());
        ASSERT_TRUE(server.load().ok());
        ASSERT_TRUE(server.commandJoint("pan", "position", 0.7));
        ASSERT_TRUE(server.commandJoint("tilt", "position", -0.4));
        ASSERT_TRUE(server.stepUntil(SimTime{1'000'000'000}).ok());
        hash = server.stateHash().value();
    }
    EXPECT_EQ(hashes[0], hashes[1]);

    ServerOptions o = panTilt();
    o.substituted.insert("actuators");
    Server off(o);
    ASSERT_TRUE(off.load().ok());
    ASSERT_TRUE(off.commandJoint("pan", "position", 0.7));
    ASSERT_TRUE(off.stepUntil(SimTime{1'000'000'000}).ok());
    EXPECT_NEAR(off.read("joint/pan/position").at(0), 0.0, 1e-3);  // nothing turns it
    EXPECT_EQ(off.read("joint/pan/voltage_command").at(0), 0.0);
    // The tilt, unpowered, sinks against its shorted motor.
    EXPECT_GT(off.read("joint/tilt/position").at(0), 0.1);
}

// A smart servo fitted to the XM430-W350's 12 V row (test_servo.cpp): R = 12/2.3 Ω,
// N·Kt = 12 / (46 rev/min), N = 353.5, η = (4.1/2.3) / (N·Kt); its loop on the drive.
std::filesystem::path writeText(const std::string& name, const std::string& text) {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() /
        ("lagrange_servo_" + std::to_string(::getpid()) + "_" + name);
    std::ofstream(path) << text;
    return path;
}

std::string smartArm(double mass) {
    char text[2000];
    std::snprintf(text, sizeof text, R"(<?xml version="1.0"?>
<robot name="smart">
  <link name="world"/>
  <joint name="fix" type="fixed"><parent link="world"/><child link="base"/><origin xyz="0 0 0.5"/></joint>
  <link name="base"><inertial><mass value="1"/><inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial></link>
  <joint name="joint1" type="revolute">
    <parent link="base"/><child link="arm"/><axis xyz="0 1 0"/>
    <limit lower="-3.0" upper="3.0" effort="10" velocity="10"/>
  </joint>
  <link name="arm">
    <inertial><origin xyz="0.1 0 0"/><mass value="%.9g"/><inertia ixx="1e-5" iyy="1e-5" izz="1e-5" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>
</robot>
)",
                  mass);
    return text;
}

std::string smartOverlay(const char* extra) {
    const double kt = 12.0 / (46.0 * kRpm) / 353.5;
    const double eta = (4.1 / 2.3) / (12.0 / (46.0 * kRpm));
    char text[3000];
    std::snprintf(text, sizeof text, R"(schema_version: 1
apply:
  - create: power/battery
    schemas:
      BatteryAPI_1:
        elec:battery:chemistry: lfp
        elec:battery:seriesCells: 4
        elec:battery:capacity: 36000.0
        elec:battery:initialSoc: 0.5
        elec:battery:ocvSoc: [0.0, 0.1, 0.5, 0.9, 1.0]
        elec:battery:ocvVoltage: [2.80, 3.20, 3.28, 3.33, 3.45]
        elec:battery:r0: 0.001
        elec:battery:cutoffVoltage: 11.2
        elec:battery:maxChargeCurrent: 10.0
      PortAPI_1:dc_out:
        elec:port:dc_out:nominalVoltage: 13.1
        elec:port:dc_out:maxCurrent: 20.0
        elec:port:dc_out:connectedTo: power/bus
  - create: power/bus
    schemas:
      ElectricalBusAPI_1:
        elec:bus:capacitance: 0.001
        elec:bus:undervoltageTrip: 9.0
        elec:bus:overvoltageTrip: 16.0
  - prim: joint1
    create_child: joint1_motor
    schemas:
      ElectricalMotorAPI_1:
        elec:motorType: dc_brushed
        elec:controlMode: position
        elec:windingResistance: %.9g
        elec:inductance: 0.0002
        elec:torqueConstant: %.9g
        elec:gearRatio: 353.5
        elec:gearEfficiency: %.9g
        elec:maxCurrent: %s
        elec:resistanceTempCoeff: 0.0
        elec:joint: joint1
        elec:powerPort: power/bus
  - prim: joint1
    schemas:
      ServoAPI_1:
        act:servo:kind: smart
        act:servo:kp: 30.0
        act:servo:resolution: 0.0015339808
%s
)",
                  12.0 / 2.3, kt, eta, "3.0", extra);
    return text;
}

ServerOptions smartRun(const std::string& tag, double mass, const char* extra) {
    ServerOptions o;
    o.robotPath = writeText(tag + ".urdf", smartArm(mass)).string();
    o.overlayPath = writeText(tag + ".yaml", smartOverlay(extra)).string();
    o.ros = false;
    o.realTimeFactor = 0.0;
    o.jointControl = true;
    o.site["gravity"] = 0.0;
    return o;
}

TEST(ServosRun, SmartServoNoLoadSpeedAndProfileOnItsOwnDrive) {
    {
        Server server(smartRun("free", 0.05, ""));
        const Status loaded = server.load();
        ASSERT_TRUE(loaded.ok()) << loaded.message;
        ASSERT_TRUE(server.commandJoint("joint1", "position", 2.9));
        ASSERT_TRUE(server.stepUntil(SimTime{300'000'000}).ok());  // mid move, at speed
        const double volts = at(server, "bus/bus/voltage");
        const double speed = at(server, "joint/joint1/velocity");
        const double datasheet = 46.0 * kRpm * volts / 12.0;
        std::printf("smart servo no-load speed %.4f rad/s at %.3f V; datasheet line %.4f\n", speed,
                    volts, datasheet);
        EXPECT_NEAR(speed, datasheet, 0.02 * datasheet);
        EXPECT_LE(std::abs(at(server, "act/joint1/present_current")), 3.0 + 1e-9);
    }
    {
        // Profile Velocity 2 rad/s, Acceleration 20 rad/s²: the joint cruises at 2 rad/s.
        Server server(smartRun("profile", 0.05,
                               "        act:servo:profileVelocity: 2.0\n"
                               "        act:servo:profileAcceleration: 20.0"));
        ASSERT_TRUE(server.load().ok());
        ASSERT_TRUE(server.commandJoint("joint1", "position", 2.0));
        // The mean speed over 100 ms of the cruise (the 4096-step encoder makes the duty, and so
        // the speed, ripple from one 100 µs step to the next).
        ASSERT_TRUE(server.stepUntil(SimTime{500'000'000}).ok());
        const double from = at(server, "joint/joint1/position");
        ASSERT_TRUE(server.stepUntil(SimTime{600'000'000}).ok());
        const double speed = (at(server, "joint/joint1/position") - from) / 0.1;
        std::printf("profiled cruise %.4f rad/s (set point %.4f)\n", speed,
                    at(server, "act/joint1/goal"));
        EXPECT_NEAR(speed, 2.0, 0.01 * 2.0);
        ASSERT_TRUE(server.stepUntil(SimTime{2'500'000'000}).ok());
        EXPECT_NEAR(at(server, "joint/joint1/position"), 2.0, 0.01);
    }
}

TEST(ServosRun, SmartServoOverloadLetsGo) {
    // 6 kg 0.1 m out under gravity: 5.9 N·m, past what the 3 A current limit lifts
    // (3 A × 4.1/2.3 N·m/A = 5.3 N·m): the duty sits at the limit for overloadTime 0.5 s and the
    // shutdown turns the torque off.
    ServerOptions o = smartRun("overload", 6.0,
                               "        act:servo:overloadTime: 0.5\n"
                               "        act:servo:kd: 0.5");
    o.site["gravity"] = kG;
    Server server(o);
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(server.commandJoint("joint1", "position", -0.5));  // lift it
    ASSERT_TRUE(server.stepUntil(SimTime{300'000'000}).ok());
    EXPECT_EQ(at(server, "act/joint1/overload"), 0.0);
    ASSERT_TRUE(server.stepUntil(SimTime{1'500'000'000}).ok());
    std::printf("overload %.0f, current %.3f A, position %.3f rad\n",
                at(server, "act/joint1/overload"), at(server, "act/joint1/present_current"),
                at(server, "joint/joint1/position"));
    EXPECT_EQ(at(server, "act/joint1/overload"), 1.0);
    EXPECT_EQ(at(server, "act/joint1/present_current"), 0.0);  // torque off: the winding open
}

}  // namespace
}  // namespace lagrange::server
