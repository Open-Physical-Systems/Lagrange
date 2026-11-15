// Joint control: any robot, joint by joint, with no controller.
#include "lagrange/server/JointControl.hpp"
#include "lagrange/server/Server.hpp"

#include <atomic>
#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <thread>
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

// Review 2 L3: a rebuild keeps the targets of the joints still in the run in one locked filter,
// so a target set by another thread (ROS's /joint_commands) at the same time is never lost.
TEST(JointControl, KeepingTargetsLosesNoneSetMeanwhile) {
    JointTargets targets;
    targets.set("kept", {JointTarget::Mode::Position, 1.0});
    targets.set("gone", {JointTarget::Mode::Velocity, 2.0});
    std::uint64_t seen = 0;
    std::map<std::string, JointTarget> copy;
    targets.refresh(seen, copy);
    targets.keepIf([](const std::string& joint) { return joint != "gone"; });
    const std::map<std::string, JointTarget> after = targets.all();
    ASSERT_EQ(after.size(), 1U);
    EXPECT_EQ(after.at("kept").value, 1.0);
    const std::uint64_t before = seen;
    targets.refresh(seen, copy);
    EXPECT_NE(seen, before) << "the module sees the change";
    EXPECT_EQ(copy.size(), 1U);

    // A writer setting fresh joints while filters run: every one it set survives.
    std::atomic<bool> done{false};
    std::thread writer([&] {
        for (int i = 0; i < 2000; ++i) {
            targets.set("w" + std::to_string(i),
                        {JointTarget::Mode::Effort, static_cast<double>(i)});
        }
        done = true;
    });
    while (!done) {
        targets.keepIf([](const std::string& joint) { return joint != "gone"; });
    }
    writer.join();
    EXPECT_EQ(targets.all().size(), 2001U);
}

// A robot fixed to the ground stays up with nothing told to it, as servos hold.
TEST(JointControl, AGroundedJointHoldsUntilTold) {
    Server server(arm("mujoco"));
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.stepUntil(SimTime{3'000'000'000}).ok());
    EXPECT_NEAR(value(server, "joint/shoulder_to_forearm/position"), 0.0, 0.05);
    EXPECT_NEAR(value(server, "joint/base_to_shoulder/position"), 0.0, 0.05);
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
