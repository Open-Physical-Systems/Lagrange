// The MuJoCo backend as a scheduler module: worlds built in code, closed-form expectations.
#include "lagrange/rigid_body/MujocoBackend.hpp"
#include "lagrange/scheduler/Scheduler.hpp"

#include <cmath>
#include <memory>
#include <numbers>
#include <string>
#include <utility>

#include <gtest/gtest.h>

namespace lagrange::rigid_body {
namespace {

using namespace literals;

constexpr double kGravity = 9.80665;  // m/s², World::gravityLocal default (standard gravity)

/// Writes a constant to one signal every step.
class Constant final : public IModule {
public:
    Constant(std::string signal, Unit unit, double value, Duration step = 1_ms)
        : signal_(std::move(signal)), unit_(unit), value_(value), step_(step) {}
    [[nodiscard]] ModuleInfo info() const override { return {"controller", "constant"}; }
    [[nodiscard]] Duration preferredStep() const override { return step_; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec spec;
        spec.name = signal_;
        spec.unit = unit_;
        id_ = d.output(spec);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        bus.write(id_, value_);
        return {};
    }
    Status step(const StepContext&, SignalBus& bus) override {
        bus.write(id_, value_);
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    std::string signal_;
    Unit unit_;
    double value_;
    Duration step_;
    SignalId id_;
};

std::array<double, 6> diagonal(double xx, double yy, double zz) {
    return {xx, yy, zz, 0.0, 0.0, 0.0};
}

double read(const Scheduler& scheduler, std::string_view signal, std::size_t component = 0) {
    const SignalId id = scheduler.bus().find(signal);
    EXPECT_TRUE(id.valid()) << signal;
    return id.valid() ? scheduler.bus().readN(id)[component] : NAN;
}

void wire(Scheduler& scheduler, const World& world) {
    auto result = scheduler.wire(WorldView(world));
    ASSERT_TRUE(result) << (result ? "" : to_string(result.error().front()));
}

// --- free fall -----------------------------------------------------------------------------

World fallingBall() {
    World world;
    const BodyId ball = world.addBody("ball", kWorldBody, Pose{{0.0, 0.0, 10.0}}, 1.0, {},
                                      diagonal(0.004, 0.004, 0.004));
    world.addJoint("ball_free", JointType::Free, ball, {0.0, 0.0, 1.0}, false);
    world.addCollider(ball, ShapeType::Sphere, {0.1, 0.0, 0.0}, Pose{}, 1.0);
    return world;
}

TEST(MujocoBackend, FreeFallFollowsGravity) {
    const World world = fallingBall();
    Scheduler scheduler;
    scheduler.add(std::make_unique<MujocoBackend>());
    wire(scheduler, world);
    ASSERT_TRUE(scheduler.stepUntil(SimTime{1'000'000'000}).ok());
    // Semi-implicit integration lags the parabola by g t dt / 2 = 4.9 mm after 1 s at 1 ms.
    const double t = 1.0;
    EXPECT_NEAR(read(scheduler, "body/ball/pose", 2), 10.0 - 0.5 * kGravity * t * t, 6e-3);
    EXPECT_NEAR(read(scheduler, "body/ball/twist", 2), -kGravity * t, 1e-9);
    // A falling body feels no proper acceleration: the IMU reads zero.
    EXPECT_NEAR(read(scheduler, "body/ball/acceleration", 2), 0.0, 1e-9);
}

// --- driven joint --------------------------------------------------------------------------

/// A flywheel on a vertical axle: gravity does not act about the axis.
World flywheel(double inertia) {
    World world;
    const BodyId wheel = world.addBody("flywheel", kWorldBody, Pose{{0.0, 0.0, 0.5}}, 2.0, {},
                                       diagonal(0.01, 0.01, inertia));
    world.addJoint("axle", JointType::Continuous, wheel, {0.0, 0.0, 1.0}, true);
    return world;
}

TEST(MujocoBackend, ConstantTorqueSpinsUpAtTorqueOverInertia) {
    constexpr double kInertia = 0.02;  // kg·m²
    constexpr double kTorque = 0.05;   // N·m
    const World world = flywheel(kInertia);
    Scheduler scheduler;
    scheduler.add(std::make_unique<MujocoBackend>());
    scheduler.add(std::make_unique<Constant>("joint/axle/torque", units::N_m, kTorque));
    wire(scheduler, world);
    ASSERT_TRUE(scheduler.stepUntil(SimTime{2'000'000'000}).ok());
    const double t = 2.0;
    EXPECT_NEAR(read(scheduler, "joint/axle/velocity"), kTorque / kInertia * t, 1e-9);
    EXPECT_NEAR(read(scheduler, "joint/axle/acceleration"), kTorque / kInertia, 1e-9);
    // architecture.md §8: the inertia the motor sees.
    EXPECT_NEAR(read(scheduler, "joint/axle/effective_inertia"), kInertia, 1e-12);
}

TEST(MujocoBackend, TorqueInNewtonsIsRefusedAtWiring) {
    const World world = flywheel(0.02);
    Scheduler scheduler;
    scheduler.add(std::make_unique<MujocoBackend>());
    scheduler.add(std::make_unique<Constant>("joint/axle/torque", units::N, 0.05));
    auto result = scheduler.wire(WorldView(world));
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error().front().kind, WiringError::Kind::UnitMismatch);
    EXPECT_EQ(result.error().front().detail,
              "writer controller/constant: N (Force); reader rigid_body/mujoco: N·m (Torque)");
}

TEST(MujocoBackend, ActuatedJointNeedsATorqueWriter) {
    const World world = flywheel(0.02);
    Scheduler scheduler;
    scheduler.add(std::make_unique<MujocoBackend>());
    auto result = scheduler.wire(WorldView(world));
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error().front().kind, WiringError::Kind::MissingWriter);
    EXPECT_EQ(result.error().front().signal, "joint/axle/torque");
}

// --- contact -------------------------------------------------------------------------------

/// A 1 kg box resting on the ground plane, gravity optionally tilted to make a slope.
World boxOnGround(double slopeDegrees, double friction) {
    World world;
    const double angle = slopeDegrees * std::numbers::pi / 180.0;
    world.gravityLocal = {kGravity * std::sin(angle), 0.0, -kGravity * std::cos(angle)};
    world.addCollider(kWorldBody, ShapeType::Plane, {5.0, 5.0, 0.1}, Pose{}, friction);
    const BodyId box = world.addBody("box", kWorldBody, Pose{{0.0, 0.0, 0.1}}, 1.0, {},
                                     diagonal(0.0067, 0.0067, 0.0067));
    world.addJoint("box_free", JointType::Free, box, {0.0, 0.0, 1.0}, false);
    world.addCollider(box, ShapeType::Box, {0.1, 0.1, 0.1}, Pose{}, friction);
    return world;
}

TEST(MujocoBackend, BoxRestsOnTheGroundAndTheImuReadsGravity) {
    const World world = boxOnGround(0.0, 1.0);
    Scheduler scheduler;
    scheduler.add(std::make_unique<MujocoBackend>());
    wire(scheduler, world);
    ASSERT_TRUE(scheduler.stepUntil(SimTime{1'000'000'000}).ok());
    // Soft contact allows a small penetration; MuJoCo's defaults keep it below a millimetre.
    EXPECT_NEAR(read(scheduler, "body/box/pose", 2), 0.1, 1e-3);
    // Proper acceleration of a body at rest: +g along the body's z axis.
    EXPECT_NEAR(read(scheduler, "body/box/acceleration", 2), kGravity, 0.01 * kGravity);
}

TEST(MujocoBackend, FrictionScaleMakesTheBoxSlide) {
    // tan(20°) = 0.364: with mu = 0.5 the box sticks, at half friction (0.25) it slides.
    for (const double scale : {1.0, 0.5}) {
        const World world = boxOnGround(20.0, 0.5);
        Scheduler scheduler;
        scheduler.add(std::make_unique<MujocoBackend>());
        scheduler.add(std::make_unique<Constant>("env/friction_scale", units::one, scale));
        wire(scheduler, world);
        ASSERT_TRUE(scheduler.stepUntil(SimTime{1'000'000'000}).ok());
        const double moved = std::abs(read(scheduler, "body/box/pose", 0));
        if (scale == 1.0) {
            EXPECT_LT(moved, 0.01) << "sticks at full friction";
        } else {
            EXPECT_GT(moved, 0.1) << "slides at half friction";
        }
    }
}

// --- state ---------------------------------------------------------------------------------

std::uint64_t runAndHash(std::uint64_t seed) {
    World world = boxOnGround(10.0, 0.3);
    const BodyId wheel = world.addBody("wheel", kWorldBody, Pose{{1.0, 0.0, 0.5}}, 2.0, {},
                                       diagonal(0.01, 0.01, 0.02));
    world.addJoint("axle", JointType::Continuous, wheel, {0.0, 0.0, 1.0}, true);
    Scheduler scheduler(SchedulerOptions{.deterministic = true, .runSeed = seed});
    scheduler.add(std::make_unique<MujocoBackend>());
    scheduler.add(std::make_unique<Constant>("joint/axle/torque", units::N_m, 0.03));
    auto result = scheduler.wire(WorldView(world));
    EXPECT_TRUE(result);
    EXPECT_TRUE(scheduler.stepUntil(SimTime{500'000'000}).ok());
    return scheduler.stateHash().value();
}

TEST(MujocoBackend, TwoRunsGiveTheSameStateHash) {
    EXPECT_EQ(runAndHash(1), runAndHash(1));
}

TEST(MujocoBackend, SaveRunRestoreRunIsIdentical) {
    const World world = boxOnGround(15.0, 0.2);
    Scheduler scheduler;
    scheduler.add(std::make_unique<MujocoBackend>());
    wire(scheduler, world);
    ASSERT_TRUE(scheduler.stepUntil(SimTime{200'000'000}).ok());
    StateWriter saved;
    ASSERT_TRUE(scheduler.saveState(saved).ok());
    ASSERT_TRUE(scheduler.stepUntil(SimTime{600'000'000}).ok());
    const std::uint64_t first = scheduler.stateHash().value();
    const double x = read(scheduler, "body/box/pose", 0);

    StateReader reader(saved.bytes());
    ASSERT_TRUE(scheduler.restoreState(reader).ok());
    ASSERT_TRUE(scheduler.stepUntil(SimTime{600'000'000}).ok());
    EXPECT_EQ(read(scheduler, "body/box/pose", 0), x);
    EXPECT_EQ(scheduler.stateHash().value(), first);
}

// --- configuration errors ------------------------------------------------------------------

TEST(MujocoBackend, RefusesBatchesAndUnknownFidelity) {
    {
        World world = fallingBall();
        world.batchCount = 4;
        Scheduler scheduler;
        scheduler.add(std::make_unique<MujocoBackend>());
        auto result = scheduler.wire(WorldView(world));
        ASSERT_FALSE(result);
        EXPECT_EQ(result.error().front().kind, WiringError::Kind::ModuleFailed);
        EXPECT_NE(result.error().front().detail.find("one environment"), std::string::npos);
    }
    {
        const World world = fallingBall();
        Scheduler scheduler;
        ModuleConfigData config;
        config.fidelity = "articulated_gpu";
        scheduler.add(std::make_unique<MujocoBackend>(), ModuleConfig(config));
        auto result = scheduler.wire(WorldView(world));
        ASSERT_FALSE(result);
        EXPECT_NE(result.error().front().detail.find("articulated_gpu"), std::string::npos);
    }
}

TEST(MujocoBackend, InvalidInertiaIsReportedFromMuJoCo) {
    World world;
    // Violates the triangle inequality of principal moments (A + B >= C).
    const BodyId bad =
        world.addBody("bad", kWorldBody, Pose{}, 1.0, {}, diagonal(0.01, 0.01, 0.05));
    world.addJoint("bad_free", JointType::Free, bad, {0.0, 0.0, 1.0}, false);
    Scheduler scheduler;
    scheduler.add(std::make_unique<MujocoBackend>());
    auto result = scheduler.wire(WorldView(world));
    ASSERT_FALSE(result);
    EXPECT_NE(result.error().front().detail.find("MuJoCo refused the world"), std::string::npos);
    EXPECT_NE(result.error().front().detail.find("inertia"), std::string::npos);
}

}  // namespace
}  // namespace lagrange::rigid_body
