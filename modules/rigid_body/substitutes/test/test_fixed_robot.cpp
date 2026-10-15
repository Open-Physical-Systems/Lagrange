// The "fixed" rigid_body substitute: the robot stays at rest, with the backend's signals.
#include "lagrange/rigid_body/FixedRobot.hpp"
#include "lagrange/scheduler/Scheduler.hpp"

#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

namespace lagrange::rigid_body {
namespace {

using namespace literals;

/// A motor controller: commands 2 N·m on the wheel.
class Motor final : public IModule {
public:
    [[nodiscard]] ModuleInfo info() const override { return {"control", "motor"}; }
    [[nodiscard]] Duration preferredStep() const override { return 1_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec out;
        out.name = "joint/wheel/torque";
        out.unit = units::N_m;
        torque_ = d.output(out);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        bus.write(torque_, 2.0);
        return {};
    }
    Status step(const StepContext&, SignalBus& bus) override {
        bus.write(torque_, 2.0);
        return {};
    }

private:
    SignalId torque_;
};

TEST(FixedRobot, HoldsTheRobotAtRestWithTheBackendsSignals) {
    World world;
    // The base turned 90 degrees about x: its y axis points up.
    const double h = std::numbers::sqrt2 / 2.0;
    const Pose base{{1.0, 2.0, 0.3}, {h, h, 0.0, 0.0}};
    const BodyId body = world.addBody("base", kWorldBody, base, 10.0, {}, {1, 1, 1, 0, 0, 0});
    world.addJoint("base_free", JointType::Free, body, {0, 0, 1}, false);
    const BodyId wheel = world.addBody("wheel", body, Pose{{0, 0.2, 0}, {1, 0, 0, 0}}, 1.0, {},
                                       {0.1, 0.4, 0.1, 0, 0, 0});
    world.addJoint("wheel", JointType::Continuous, wheel, {0, 1, 0}, true);

    Scheduler scheduler;
    scheduler.add(std::make_unique<FixedRobot>());
    scheduler.add(std::make_unique<Motor>());
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_TRUE(wired) << to_string(wired.error().front());
    ASSERT_TRUE(scheduler.stepUntil(SimTime{100'000'000}).ok());
    const SignalBus& bus = scheduler.bus();
    const auto read = [&bus](const char* name) { return bus.readN(bus.find(name)); };

    EXPECT_EQ(read("joint/wheel/position")[0], 0.0);
    EXPECT_EQ(read("joint/wheel/velocity")[0], 0.0);
    EXPECT_DOUBLE_EQ(read("joint/wheel/effective_inertia")[0], 0.4) << "the moment about y";
    EXPECT_DOUBLE_EQ(read("joint/wheel/constraint_torque")[0], -2.0) << "the clamp holds it";
    const auto pose = read("body/base/pose");
    EXPECT_DOUBLE_EQ(pose[0], 1.0);
    EXPECT_DOUBLE_EQ(pose[2], 0.3);
    for (const double v : read("body/base/twist")) {
        EXPECT_EQ(v, 0.0);
    }
    // At rest an accelerometer reads 1 g upwards, which is the body's +y here.
    const auto accel = read("body/base/acceleration");
    EXPECT_NEAR(accel[0], 0.0, 1e-12);
    EXPECT_NEAR(accel[1], 9.80665, 1e-9);
    EXPECT_NEAR(accel[2], 0.0, 1e-9);
}

}  // namespace
}  // namespace lagrange::rigid_body
