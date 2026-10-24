// Keeps docs/guide/writing-a-module.md true: the Odometer wires next to the MuJoCo backend on
// the sample AMR, agrees with the true motion, and a wrong unit is refused at wiring.
#include "lagrange/rigid_body/MujocoBackend.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/server/DiffDrive.hpp"
#include "lagrange/server/Robot.hpp"
#include "lagrange/tutorial/Odometer.hpp"

#include <cmath>
#include <memory>
#include <numbers>
#include <string>

#include <gtest/gtest.h>

namespace lagrange::tutorial {
namespace {

using namespace literals;

double yaw(const double* q) {
    return std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
}

void addDrive(Scheduler& scheduler) {
    scheduler.add(std::make_unique<rigid_body::MujocoBackend>());
    scheduler.add(
        std::make_unique<server::DriveCommand>(server::DriveCommand::Settings{}, nullptr));
    server::DiffDriveController::Settings drive;
    drive.leftJoint = "left_wheel_joint";
    drive.rightJoint = "right_wheel_joint";
    scheduler.add(std::make_unique<server::DiffDriveController>(drive));
}

TEST(Odometer, AgreesWithTheTrueMotionOfTheSampleAmr) {
    auto robot = server::loadUrdf(LAGRANGE_TEST_AMR_URDF);
    ASSERT_TRUE(robot) << robot.error();
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    addDrive(scheduler);
    scheduler.add(std::make_unique<Odometer>(Odometer::Settings{}));
    auto wired = scheduler.wire(WorldView(robot->world));
    ASSERT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));

    // The true path length and heading change, sampled every physics step.
    const SignalBus& bus = scheduler.bus();
    const SignalId pose = bus.find("body/base_link/pose");
    double path = 0.0;
    double turned = 0.0;
    auto last = bus.readN(pose);
    double x = last[0];
    double y = last[1];
    double heading = yaw(last.data() + 3);
    for (int k = 0; k < 10'000; ++k) {  // 10 s: the ramp, then most of the left loop
        ASSERT_TRUE(scheduler.stepTick().ok());
        const auto now = bus.readN(pose);
        path += std::hypot(now[0] - x, now[1] - y);
        turned += std::remainder(yaw(now.data() + 3) - heading, 2.0 * std::numbers::pi);
        x = now[0];
        y = now[1];
        heading = yaw(now.data() + 3);
    }
    const double odomDistance = bus.read(bus.find("base/odometer/distance"));
    const double odomHeading = bus.read(bus.find("base/odometer/heading"));
    EXPECT_GT(path, 4.0);
    // Within 2 % of the path. The heading reads about 2 % high: the simulated robot turns a
    // little less than ideal kinematics on the nominal 0.46 m track predict, the kind of
    // systematic error a real odometer is calibrated for.
    EXPECT_NEAR(odomDistance, path, 0.02 * path);
    EXPECT_NEAR(odomHeading, turned, 0.03 * std::abs(turned));
}

TEST(Odometer, AWrongUnitIsRefusedAtWiring) {
    auto robot = server::loadUrdf(LAGRANGE_TEST_AMR_URDF);
    ASSERT_TRUE(robot) << robot.error();

    // A reader that takes the wheel speed as m/s, where the physics writes rad/s.
    class WrongUnit final : public IModule {
    public:
        [[nodiscard]] ModuleInfo info() const override {
            return {"sensors", "wrong_unit", 0, 1, false, true, CapabilityTier::Tier0};
        }
        [[nodiscard]] Duration preferredStep() const override { return 10_ms; }
        Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
            SignalSpec spec;
            spec.name = "joint/left_wheel_joint/velocity";
            spec.unit = units::m_per_s;
            (void)d.input(spec);
            return {};
        }
        Status init(const WorldView&, const ModuleConfig&, SignalBus&) override { return {}; }
        Status step(const StepContext&, SignalBus&) override { return {}; }
    };
    Scheduler scheduler;
    addDrive(scheduler);
    scheduler.add(std::make_unique<WrongUnit>());
    auto wired = scheduler.wire(WorldView(robot->world));
    ASSERT_FALSE(wired);
    ASSERT_EQ(wired.error().size(), 1U);
    EXPECT_EQ(wired.error()[0].kind, WiringError::Kind::UnitMismatch);
    const std::string text = to_string(wired.error()[0]);
    EXPECT_NE(text.find("rigid_body/mujoco"), std::string::npos) << text;
    EXPECT_NE(text.find("sensors/wrong_unit"), std::string::npos) << text;
}

TEST(Odometer, AMissingJointFailsWithItsName) {
    auto robot = server::loadUrdf(LAGRANGE_TEST_AMR_URDF);
    ASSERT_TRUE(robot) << robot.error();
    Scheduler scheduler;
    addDrive(scheduler);
    Odometer::Settings settings;
    settings.leftJoint = "front_wheel_joint";
    scheduler.add(std::make_unique<Odometer>(settings));
    auto wired = scheduler.wire(WorldView(robot->world));
    ASSERT_FALSE(wired);
    EXPECT_EQ(wired.error()[0].kind, WiringError::Kind::ModuleFailed);
    EXPECT_NE(wired.error()[0].detail.find("front_wheel_joint"), std::string::npos);
}

}  // namespace
}  // namespace lagrange::tutorial
