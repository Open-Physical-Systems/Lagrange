// The rotors' top speed at an autopilot's full output (RotorTopSpeed.hpp), checked against the
// steady state of the electrical domain's motor model written out by hand.
#include "lagrange/mavlink/RotorTopSpeed.hpp"

#include <cmath>
#include <numbers>
#include <string>

#include <gtest/gtest.h>

namespace lagrange::mavlink {
namespace {

constexpr double kRho = 1.225;  // kg/m³, ISA sea level

struct Motor {
    std::string type{"bldc"};
    double kt{0.01557};  // N·m/A: the sample quadcopter's 920 KV, 1.5·60/(2π·920)
    double r{0.1};       // ohm
    double l{0.00003};   // H
    int poles{7};
    double maxCurrent{25.0};  // A
};

/// A quadcopter of the sample's class: 2 kg, four 10 in rotors (APC 10x4.7 SF coefficients),
/// each driven by `motor` from a 4S battery (4.2 V a cell full) on one bus. `motors` false: no
/// electrical prims at all.
World quad(const Motor& motor, bool motors = true) {
    World w;
    const BodyId base = w.addBody("base", kWorldBody, Pose{}, 1.9, {}, {0.02, 0.02, 0.04, 0, 0, 0});
    w.addJoint("base_free", JointType::Free, base, {0, 0, 1}, false);
    const PrimId bus = w.addPrim("/quad/power/dc_bus", "dc_bus");
    const PrimId battery = w.addPrim("/quad/power/battery", "battery");
    if (motors) {
        SchemaInstance b;
        b.schema = "BatteryAPI_1";
        b.prim = battery;
        b.integers = {{"elec:battery:seriesCells", 4}};
        b.realArrays = {{"elec:battery:ocvVoltage", {3.3, 3.7, 4.2}}};
        w.schemaInstances.push_back(b);
        SchemaInstance port;
        port.schema = "PortAPI_1";
        port.instance = "dc_out";
        port.prim = battery;
        port.relationships = {{"elec:port:dc_out:connectedTo", {bus}}};
        w.schemaInstances.push_back(port);
    }
    for (int i = 1; i <= 4; ++i) {
        const std::string name = "rotor_" + std::to_string(i);
        const BodyId rotor =
            w.addBody(name + "_link", base, Pose{}, 0.025, {}, {4.5e-5, 4.5e-5, 9e-5, 0, 0, 0});
        w.addJoint(name + "_joint", JointType::Continuous, rotor, {0, 0, 1}, true);
        const PrimId joint = w.joints.prim.back();
        SchemaInstance s;
        s.schema = "RotorAPI_1";
        s.prim = joint;
        s.reals = {{"aero:rotor:radius", 0.127},
                   {"aero:rotor:thrustCoeff", 0.125},
                   {"aero:rotor:powerCoeff", 0.0507}};
        w.schemaInstances.push_back(s);
        if (motors) {
            SchemaInstance m;
            m.schema = "ElectricalMotorAPI_1";
            m.prim = w.addPrim("/quad/" + name + "_motor", name + "_motor");
            m.tokens = {{"elec:motorType", motor.type}};
            m.reals = {{"elec:torqueConstant", motor.kt},
                       {"elec:windingResistance", motor.r},
                       {"elec:inductance", motor.l},
                       {"elec:maxCurrent", motor.maxCurrent}};
            m.integers = {{"elec:polePairs", motor.poles}};
            m.relationships = {{"elec:joint", {joint}}, {"elec:powerPort", {bus}}};
            w.schemaInstances.push_back(m);
        }
    }
    return w;
}

const SchemaInstance& rotor1(const World& w) {
    for (const SchemaInstance& s : w.schemaInstances) {
        if (s.schema == "RotorAPI_1") {
            return s;
        }
    }
    throw std::runtime_error("no rotor");
}

/// The propeller's torque at ω (rad/s): CP·ρ·n²·D⁵/(2π), n = ω/(2π).
double propellerTorque(double omega) {
    const double n = omega / (2.0 * std::numbers::pi);
    return 0.0507 * kRho * n * n * std::pow(0.254, 5) / (2.0 * std::numbers::pi);
}

TEST(RotorTopSpeed, IsWhereTheDriveRunsOutOfVoltageTurningThePropeller) {
    const Motor motor;
    const World w = quad(motor);
    const RotorTopSpeed top = rotorTopSpeed(WorldView(w), rotor1(w), kRho);
    // Steady state of the dq model at that speed: v_q = R·i + Ke·ω, v_d = −p·ω·L·i, i = Q/Kt,
    // Ke = Kt/1.5; |v| is the drive's limit, V/√3 of the full 16.8 V.
    const double i = propellerTorque(top.speed) / motor.kt;
    const double vq = motor.r * i + motor.kt / 1.5 * top.speed;
    const double vd = motor.poles * top.speed * motor.l * i;
    EXPECT_NEAR(std::hypot(vq, vd), 16.8 / std::sqrt(3.0), 1e-9);
    EXPECT_LT(i, motor.maxCurrent);
    // The sample's class: about 0.8 krad/s (the bridge's fixed 850 before), below no load.
    EXPECT_GT(top.speed, 750.0);
    EXPECT_LT(top.speed, 16.8 / std::sqrt(3.0) / (motor.kt / 1.5));
    EXPECT_NE(top.from.find("rotor_1_motor"), std::string::npos) << top.from;
    EXPECT_NE(top.from.find("voltage-limited"), std::string::npos) << top.from;
}

TEST(RotorTopSpeed, StopsAtTheMotorsCurrentLimit) {
    Motor motor;
    motor.maxCurrent = 5.0;  // A: below what the propeller asks near top speed
    const World w = quad(motor);
    const RotorTopSpeed top = rotorTopSpeed(WorldView(w), rotor1(w), kRho);
    EXPECT_NEAR(propellerTorque(top.speed), motor.maxCurrent * motor.kt, 1e-9);
    EXPECT_NE(top.from.find("current-limited"), std::string::npos) << top.from;
}

TEST(RotorTopSpeed, ABrushedMotorHasTheWholeBusVoltage) {
    Motor motor;
    motor.type = "dc_brushed";  // Ke = Kt, the limit V, no d axis
    const World w = quad(motor);
    const RotorTopSpeed top = rotorTopSpeed(WorldView(w), rotor1(w), kRho);
    const double i = propellerTorque(top.speed) / motor.kt;
    EXPECT_NEAR(motor.r * i + motor.kt * top.speed, 16.8, 1e-9);
}

TEST(RotorTopSpeed, WithoutAMotorTheRotorsLiftTwiceTheWeight) {
    const World w = quad(Motor{}, false);
    const RotorTopSpeed top = rotorTopSpeed(WorldView(w), rotor1(w), kRho);
    const double n = top.speed / (2.0 * std::numbers::pi);
    const double thrust = 4.0 * 0.125 * kRho * n * n * std::pow(0.254, 4);  // CT·ρ·n²·D⁴ each
    EXPECT_NEAR(thrust, 2.0 * (1.9 + 4 * 0.025) * 9.80665, 1e-6);
    EXPECT_NE(top.from.find("no motor"), std::string::npos) << top.from;
}

}  // namespace
}  // namespace lagrange::mavlink
