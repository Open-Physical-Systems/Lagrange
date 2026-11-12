// Kinds `force_torque` and `joint_torque` (sensors.md §9): an arm on a hinge about +y held
// horizontal by a motor torque that balances its weight exactly (2 kg at 0.5 m: m·g·d =
// 9.80665 N·m). Closed forms: at the joint the parent holds the arm with F = (0, 0, m·g) and
// τ = (0, −m·g·d, 0); at the arm's centre of mass the same wrench has no moment; the joint torque
// sensor reads the motor's torque. A Mini45's ranges saturate, its resolutions quantise, and a
// first-order low-pass rises as 1 − exp(−2π·f_c·t).
#include "amr_kinds_test_util.hpp"
#include "lagrange/sensors/Kinds.hpp"

#include <numbers>

namespace lagrange::sensors {
namespace {

using namespace kindtest;

constexpr double kMass = 2.0;  // kg
constexpr double kArm = 0.5;   // m, centre of mass from the hinge
constexpr double kWeight = kMass * kGravity;
constexpr double kHold = kWeight * kArm;  // N·m
constexpr double kCutoff = 5.0;           // Hz

World arm() {
    World w;
    const BodyId link = w.addBody("arm", kWorldBody, at(0, 0, 1), kMass, {kArm, 0, 0},
                                  {0.001, 0.01, 0.01, 0, 0, 0});
    w.addJoint("shoulder", JointType::Revolute, link, {0, 1, 0}, true);
    w.joints.lowerLimit.back() = -3.0;
    w.joints.upperLimit.back() = 3.0;
    w.addCustomSensor("at_joint", "force_torque", link, Pose{}, 1000.0, {}, {});
    w.addCustomSensor("at_tool", "force_torque", link, at(kArm, 0, 0), 1000.0, {}, {});
    w.addCustomSensor("reversed", "force_torque", link, Pose{}, 1000.0, {},
                      {{"measure_direction", "child_to_parent"}});
    // Turned 90° about z: the sensor's x is the world's y.
    w.addCustomSensor("turned", "force_torque", link, turned(std::numbers::pi / 2.0, 0, 0, 1),
                      1000.0, {}, {});
    w.addCustomSensor("mini45", "force_torque", link, Pose{}, 1000.0, {{"fz_range", 10.0}},
                      {{"preset", "mini45_si_580_20"}});
    w.addCustomSensor("filtered", "force_torque", link, Pose{}, 1000.0,
                      {{"cutoff_frequency", kCutoff}}, {});
    w.addCustomSensor("tau", "joint_torque", link, Pose{}, 1000.0, {}, {{"joint", "shoulder"}});
    w.addCustomSensor("tau_coarse", "joint_torque", link, Pose{}, 1000.0,
                      {{"resolution", 0.5}, {"range", 5.0}}, {});
    return w;
}

std::vector<Writer::Output> hold() {
    return {{specOf("joint/shoulder/torque", SignalType::Scalar, units::N_m),
             [](double) { return std::vector<double>{-kHold}; }}};
}

TEST(KindForceTorque, AreBuiltIn) {
    EXPECT_TRUE(isBuiltinKind("force_torque"));
    EXPECT_TRUE(isBuiltinKind("joint_torque"));
}

TEST(KindForceTorque, ReadsTheWrenchThatHoldsTheArm) {
    const World w = arm();
    KindRun r = start(w, true, hold());
    ASSERT_TRUE(r.scheduler);
    ASSERT_TRUE(r.until(0.5));
    EXPECT_NEAR(r.read("joint/shoulder/position"), 0.0, 1e-3) << "held still";
    const std::vector<double> j = r.readN("sensor/at_joint/wrench");
    EXPECT_NEAR(j[0], 0.0, 0.01);
    EXPECT_NEAR(j[1], 0.0, 0.01);
    EXPECT_NEAR(j[2], kWeight, 0.005 * kWeight);
    EXPECT_NEAR(j[3], 0.0, 0.01);
    EXPECT_NEAR(j[4], -kHold, 0.005 * kHold);
    EXPECT_NEAR(j[5], 0.0, 0.01);
    // At the centre of mass the weight has no moment arm.
    const std::vector<double> t = r.readN("sensor/at_tool/torque");
    for (const double m : t) {
        EXPECT_NEAR(m, 0.0, 0.01 * kHold);
    }
    EXPECT_NEAR(r.readN("sensor/at_tool/force")[2], kWeight, 0.005 * kWeight);
    // child_to_parent is the opposite wrench.
    EXPECT_NEAR(r.readN("sensor/reversed/force")[2], -j[2], 1e-12);
    // Turned 90° about z: the moment about the world's y is about the sensor's x.
    EXPECT_NEAR(r.readN("sensor/turned/torque")[0], -kHold, 0.005 * kHold);
    EXPECT_NEAR(r.readN("sensor/turned/torque")[1], 0.0, 0.01);
    // The joint torque sensor: the motor's torque about the axis.
    EXPECT_NEAR(r.read("sensor/tau/torque"), -kHold, 0.005 * kHold);
    EXPECT_EQ(r.read("sensor/tau/saturated"), 0.0);
}

TEST(KindForceTorque, RangesSaturateAndResolutionsQuantise) {
    const World w = arm();
    KindRun r = start(w, true, hold());
    ASSERT_TRUE(r.scheduler);
    ASSERT_TRUE(r.until(0.5));
    const std::vector<double> m = r.readN("sensor/mini45/wrench");
    EXPECT_EQ(m[2], 10.0) << "fz clipped at its 10 N range";
    EXPECT_EQ(r.read("sensor/mini45/saturated"), 1.0);
    // Whole steps of 1/4 N and 1/188 N·m; noise of about a step.
    EXPECT_DOUBLE_EQ(m[0] / 0.25, std::round(m[0] / 0.25));
    EXPECT_NEAR(m[4] * 188.0, std::round(m[4] * 188.0), 1e-9);
    EXPECT_NEAR(m[4], -kHold, 5.0 / 188.0);
    EXPECT_EQ(r.read("sensor/tau_coarse/torque"), -5.0) << "clipped at 5 N·m";
    EXPECT_EQ(r.read("sensor/tau_coarse/saturated"), 1.0);
}

TEST(KindForceTorque, TheLowPassRisesAsAFirstOrderFilter) {
    const World w = arm();
    KindRun r = start(w, true, hold());
    ASSERT_TRUE(r.scheduler);
    for (const double t : {0.02, 0.05, 0.1}) {
        ASSERT_TRUE(r.until(t));
        const double expected = kWeight * (1.0 - std::exp(-2.0 * std::numbers::pi * kCutoff * t));
        EXPECT_NEAR(r.readN("sensor/filtered/force")[2], expected, 0.02 * kWeight) << t;
    }
}

TEST(KindForceTorque, GroundTruthIsExact) {
    const World w = arm();
    KindRun r = start(w, true, hold(), true);
    ASSERT_TRUE(r.scheduler);
    ASSERT_TRUE(r.until(0.5));
    EXPECT_NEAR(r.readN("sensor/mini45/wrench")[2], kWeight, 0.005 * kWeight) << "no range";
    EXPECT_NEAR(r.readN("sensor/filtered/force")[2], r.readN("sensor/at_joint/force")[2], 1e-12);
}

}  // namespace
}  // namespace lagrange::sensors
