// Spike 0.2-C6: Jolt runs the pendulum and the AMR. Tolerances are a spike's: the full
// backend gets MuJoCo's validation suite (docs/plan/spikes/0.2-C1-mujoco.md).
#include "lagrange/rigid_body/JoltBackend.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/server/DiffDrive.hpp"
#include "lagrange/server/Robot.hpp"

#include <cmath>
#include <cstdio>
#include <memory>
#include <numbers>
#include <span>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::rigid_body {
namespace {

using namespace literals;

TEST(Jolt, APendulumSwingsWithItsSmallAnglePeriod) {
    // A point mass on a 1 m massless rod, hinged to the world about y, released 0.1 rad out.
    constexpr double kLengthM = 1.0;
    constexpr double kMassKg = 1.0;
    constexpr double kStartRad = 0.1;
    World world;
    world.robots.push_back("pendulum");
    const Pose released{{0, 0, 2.0}, {std::cos(kStartRad / 2), 0, std::sin(kStartRad / 2), 0}};
    const BodyId bob = world.addBody("bob", kWorldBody, released, kMassKg, {0, 0, -kLengthM},
                                     {1e-6, 1e-6, 1e-6, 0, 0, 0});
    world.bodies.robot[bob.index] = 0;
    world.addJoint("pivot", JointType::Continuous, bob, {0, 1, 0}, false);

    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<JoltBackend>());
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    const SignalId velocity = scheduler.bus().find("joint/pivot/velocity");
    // Period from the times the swing turns (velocity sign changes, every half period).
    std::vector<double> turns;
    double previous = 0.0;
    for (int k = 0; k < 6000; ++k) {
        ASSERT_TRUE(scheduler.stepTick().ok());
        const double v = scheduler.bus().read(velocity);
        if (k > 10 && v * previous < 0.0) {
            turns.push_back(scheduler.now().seconds());
        }
        previous = v;
    }
    ASSERT_GE(turns.size(), 4U);
    const double period =
        2.0 * (turns.back() - turns.front()) / static_cast<double>(turns.size() - 1);
    const double expected = 2.0 * std::numbers::pi * std::sqrt(kLengthM / 9.80665);
    std::printf("pendulum period %.5f s, small-angle %.5f s\n", period, expected);
    // Small-angle period, with the 0.1 rad amplitude correction (+0.06 %) inside the band.
    EXPECT_NEAR(period, expected, 0.01 * expected);
}

TEST(Jolt, TheAmrDrivesLikeOnMujoco) {
    auto robot = server::loadUrdf(LAGRANGE_TEST_AMR_URDF);
    ASSERT_TRUE(robot) << robot.error();
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<JoltBackend>());
    scheduler.add(
        std::make_unique<server::DriveCommand>(server::DriveCommand::Settings{}, nullptr));
    server::DiffDriveController::Settings settings;
    settings.leftJoint = robot->actuatedJoints.at(0);
    settings.rightJoint = robot->actuatedJoints.at(1);
    scheduler.add(std::make_unique<server::DiffDriveController>(settings));
    auto wired = scheduler.wire(WorldView(robot->world));
    ASSERT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    ASSERT_TRUE(scheduler.stepUntil(SimTime{6'000'000'000}).ok());

    const SignalBus& bus = scheduler.bus();
    const std::span<const double> pose = bus.readN(bus.find("body/base_link/pose"));
    std::printf("AMR after 6 s: chord %.3f m, z %.4f m, yaw rate %.4f rad/s\n",
                std::hypot(pose[0], pose[1]), pose[2],
                bus.readN(bus.find("body/base_link/twist"))[5]);
    // MuJoCo (test_robot.cpp): 3 m along a 1.2 m circle after the 2 s ramp, a 2.28 m chord.
    EXPECT_NEAR(std::hypot(pose[0], pose[1]), 2.28, 0.3);
    EXPECT_NEAR(pose[2], 0.1, 0.02) << "the chassis stays at axle height";
    const double tiltCos = 1.0 - 2.0 * (pose[4] * pose[4] + pose[5] * pose[5]);
    EXPECT_GT(tiltCos, std::cos(5.0 * std::numbers::pi / 180.0)) << "upright";
    EXPECT_NEAR(bus.readN(bus.find("body/base_link/twist"))[5], 0.5, 0.05) << "turns at 0.5 rad/s";
}

TEST(Jolt, TwoRunsAreIdentical) {
    auto robot = server::loadUrdf(LAGRANGE_TEST_AMR_URDF);
    ASSERT_TRUE(robot) << robot.error();
    const auto run = [&]() {
        auto scheduler = std::make_unique<Scheduler>(SchedulerOptions{.deterministic = true});
        scheduler->add(std::make_unique<JoltBackend>());
        scheduler->add(
            std::make_unique<server::DriveCommand>(server::DriveCommand::Settings{}, nullptr));
        server::DiffDriveController::Settings settings;
        settings.leftJoint = robot->actuatedJoints.at(0);
        settings.rightJoint = robot->actuatedJoints.at(1);
        scheduler->add(std::make_unique<server::DiffDriveController>(settings));
        EXPECT_TRUE(scheduler->wire(WorldView(robot->world)));
        EXPECT_TRUE(scheduler->stepUntil(SimTime{2'000'000'000}).ok());
        return scheduler->stateHash().value();
    };
    EXPECT_EQ(run(), run());
}

}  // namespace
}  // namespace lagrange::rigid_body
