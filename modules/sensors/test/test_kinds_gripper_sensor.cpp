// gripper_sensor (sensors.md §9): the opening from the finger joints, the current from their
// torque or the motors, an object held only when both fingers press on something apart, and
// slip as the object's velocity across the contact force.
#include "more_kinds_util.hpp"

#include <cmath>

namespace lagrange::sensors::test {
namespace {

struct Grasp {
    double opening{0.02};  // m, each finger's joint position
    double force{5.0};     // N, on each finger
    std::array<double, 3> cubeVelocity{0.0, 0.0, 0.0};
    double torque{-5.0};  // N per finger joint
};

/// A parallel gripper (two prismatic fingers along y) round a 4 cm cube.
KindRun gripper(const Grasp& g, std::map<std::string, double, std::less<>> settings = {},
                std::string motors = {}, const std::function<void(Writer&)>& extra = {}) {
    KindRun r;
    World& w = r.world;
    const BodyId base = movingBody(w, "hand");
    Pose left;
    left.position = {0.0, 0.027, 0.0};
    Pose right;
    right.position = {0.0, -0.027, 0.0};
    const BodyId fl = w.addBody("finger_l", base, left, 0.1, {}, {1e-4, 1e-4, 1e-4, 0, 0, 0});
    w.addJoint("finger_l_joint", JointType::Prismatic, fl, {0.0, 1.0, 0.0}, true);
    const BodyId fr = w.addBody("finger_r", base, right, 0.1, {}, {1e-4, 1e-4, 1e-4, 0, 0, 0});
    w.addJoint("finger_r_joint", JointType::Prismatic, fr, {0.0, -1.0, 0.0}, true);
    w.addCollider(fl, ShapeType::Box, {0.005, 0.005, 0.02}, Pose{}, 1.0);
    w.addCollider(fr, ShapeType::Box, {0.005, 0.005, 0.02}, Pose{}, 1.0);
    const BodyId cube = movingBody(w, "cube");
    w.addCollider(cube, ShapeType::Box, {0.02, 0.02, 0.02}, Pose{}, 1.0);
    std::map<std::string, std::string, std::less<>> text{
        {"fingers", "finger_l_joint finger_r_joint"}};
    if (!motors.empty()) {
        text["motors"] = motors;
    }
    settings.emplace("torque_constant", 10.0);
    w.addCustomSensor("grip", "gripper_sensor", base, Pose{}, 100.0, std::move(settings),
                      std::move(text));
    auto out = std::make_unique<Writer>();
    const auto still = [](std::array<double, 3> p) { return [p](double) { return p; }; };
    out->body("hand", still({0.0, 0.0, 0.0}));
    out->body("finger_l", still({0.0, 0.027, 0.0}));
    out->body("finger_r", still({0.0, -0.027, 0.0}));
    out->body("cube", still({0.0, 0.0, 0.0}), g.cubeVelocity);
    for (const char* f : {"finger_l", "finger_r"}) {
        const double sign = f[7] == 'l' ? 1.0 : -1.0;
        out->add(std::string("joint/") + f + "_joint/position", SignalType::Scalar, units::m,
                 [g](double) { return std::vector<double>{g.opening}; });
        out->add(std::string("joint/") + f + "_joint/torque", SignalType::Scalar, units::N,
                 [g](double) { return std::vector<double>{g.torque}; });
        out->add(
            std::string("body/") + f + "/contact_force", SignalType::Vec3, units::N,
            [g, sign](double) { return std::vector<double>{0.0, sign * g.force, 0.0}; }, 1,
            "world_local");
    }
    if (extra) {
        extra(*out);
    }
    r.wire(std::move(out));
    return r;
}

TEST(GripperSensor, HoldsACube) {
    KindRun r = gripper({}, {}, {});
    ASSERT_TRUE(r.runTo(0.1));
    EXPECT_DOUBLE_EQ(r.read("sensor/grip/width"), 0.04);
    EXPECT_DOUBLE_EQ(r.read("sensor/grip/current"), 1.0);  // 2 × 5 N / 10 N/A
    EXPECT_EQ(r.read("sensor/grip/object_detected"), 1.0);
    EXPECT_EQ(r.read("sensor/grip/grip_force"), 5.0);
    EXPECT_EQ(r.read("sensor/grip/slip"), 0.0);
}

TEST(GripperSensor, ASlidingCubeSlips) {
    Grasp g;
    g.cubeVelocity = {0.0, 0.0, -0.02};  // down between the fingers
    KindRun r = gripper(g);
    ASSERT_TRUE(r.runTo(0.1));
    EXPECT_EQ(r.read("sensor/grip/slip"), 1.0);
    EXPECT_NEAR(r.read("sensor/grip/slip_speed"), 0.02, 1e-12);
    // Along the contact force (squeezed), not across it: no slip.
    g.cubeVelocity = {0.0, 0.02, 0.0};
    KindRun s = gripper(g);
    ASSERT_TRUE(s.runTo(0.1));
    EXPECT_EQ(s.read("sensor/grip/slip"), 0.0);
    EXPECT_NEAR(s.read("sensor/grip/slip_speed"), 0.0, 1e-12);
}

TEST(GripperSensor, ClosedOnItselfOrOpenIsNotAnObject) {
    Grasp closed;
    closed.opening = 0.0;
    closed.force = 20.0;
    KindRun r = gripper(closed);
    ASSERT_TRUE(r.runTo(0.1));
    EXPECT_EQ(r.read("sensor/grip/object_detected"), 0.0);
    Grasp open;
    open.force = 0.0;
    KindRun s = gripper(open);
    ASSERT_TRUE(s.runTo(0.1));
    EXPECT_EQ(s.read("sensor/grip/object_detected"), 0.0);
    EXPECT_EQ(s.read("sensor/grip/grip_force"), 0.0);
}

TEST(GripperSensor, TheMotorsCurrentWhenNamed) {
    // Unconnected motor currents (the electrical domain off): the torque proxy.
    KindRun r = gripper({}, {}, "m_l m_r");
    ASSERT_TRUE(r.runTo(0.1));
    EXPECT_DOUBLE_EQ(r.read("sensor/grip/current"), 1.0);
    // The drives' torque currents, by magnitude.
    KindRun s = gripper({}, {}, "m_l m_r", [](Writer& w) {
        w.add("motor/m_l/current_q", SignalType::Scalar, units::A,
              [](double) { return std::vector<double>{0.7}; });
        w.add("motor/m_r/current_q", SignalType::Scalar, units::A,
              [](double) { return std::vector<double>{-0.8}; });
    });
    ASSERT_TRUE(s.runTo(0.1));
    EXPECT_DOUBLE_EQ(s.read("sensor/grip/current"), 1.5);
}

}  // namespace
}  // namespace lagrange::sensors::test
