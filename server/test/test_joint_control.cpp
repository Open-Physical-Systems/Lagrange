// Joint control: any robot, joint by joint, with no controller.
#include "lagrange/server/Server.hpp"

#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

ServerOptions arm(const std::string& physics = "mujoco") {
    ServerOptions o;
    o.robotPath = LAGRANGE_TEST_ARM_URDF;
    o.jointControl = true;
    o.demoDrive = false;
    o.ros = false;
    o.realTimeFactor = 0.0;
    o.physics = physics;
    return o;
}

double value(const Server& s, const std::string& name) {
    return s.read(name).at(0);
}

// A plain URDF: position holds near the target against gravity, on either engine.
TEST(JointControl, PositionReachesTheTargetOnAPlainUrdf) {
    for (const char* physics : {"mujoco", "jolt"}) {
        Server server(arm(physics));
        const Status loaded = server.load();
        ASSERT_TRUE(loaded.ok()) << loaded.message;
        ASSERT_TRUE(server.commandJoint("base_to_shoulder", "position", 0.5));
        ASSERT_TRUE(server.commandJoint("shoulder_to_forearm", "position", -0.4));
        ASSERT_TRUE(server.stepUntil(SimTime{3'000'000'000}).ok());
        EXPECT_NEAR(value(server, "joint/base_to_shoulder/position"), 0.5, 0.05) << physics;
        EXPECT_NEAR(value(server, "joint/shoulder_to_forearm/position"), -0.4, 0.05) << physics;
    }
}

TEST(JointControl, VelocityAndEffort) {
    Server server(arm());
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(server.commandJoint("shoulder_to_forearm", "velocity", 2.0));
    const Status stepped = server.stepUntil(SimTime{1'000'000'000});
    ASSERT_TRUE(stepped.ok()) << stepped.message;
    EXPECT_NEAR(value(server, "joint/shoulder_to_forearm/velocity"), 2.0, 0.3);
    ASSERT_TRUE(server.commandJoint("shoulder_to_forearm", "effort", 0.002));
    const Status after = server.stepUntil(SimTime{1'100'000'000});
    ASSERT_TRUE(after.ok()) << after.message;
    EXPECT_DOUBLE_EQ(value(server, "joint/shoulder_to_forearm/torque_command"), 0.002);
    EXPECT_FALSE(server.commandJoint("shoulder_to_forearm", "banana", 0.25));
}

// A joint not commanded gets no torque; a wheel robot's wheels spin on command, no drive
// controller.
TEST(JointControl, WheelsWithoutADriveController) {
    ServerOptions o = arm();
    o.robotPath = LAGRANGE_TEST_AMR_URDF;
    Server server(o);
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(server.stepUntil(SimTime{500'000'000}).ok());
    EXPECT_EQ(value(server, "joint/left_wheel_joint/torque_command"), 0.0);
    ASSERT_TRUE(server.commandJoint("left_wheel_joint", "velocity", 5.0));
    ASSERT_TRUE(server.commandJoint("right_wheel_joint", "velocity", 5.0));
    ASSERT_TRUE(server.stepUntil(SimTime{3'000'000'000}).ok());
    const std::vector<double> base = server.read("body/base_link/pose");
    EXPECT_GT(std::hypot(base[0], base[1]), 0.3) << "it drove";
}

}  // namespace
}  // namespace lagrange::server
