// The MuJoCo backend as a scheduler module: worlds built in code, closed-form expectations.
#include "lagrange/rigid_body/MujocoBackend.hpp"
#include "lagrange/scheduler/Scheduler.hpp"

#include <algorithm>
#include <array>
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

TEST(MujocoBackend, RevoluteLimitsAreInRadians) {
    // A hinge limited to ±2 rad, driven by a constant torque: it stops at 2 rad, not at 2°
    // (MuJoCo's compiler reads degrees unless told otherwise).
    World world = flywheel(0.02);
    world.joints.type.back() = JointType::Revolute;
    world.joints.lowerLimit.back() = -2.0;
    world.joints.upperLimit.back() = 2.0;
    Scheduler scheduler;
    scheduler.add(std::make_unique<MujocoBackend>());
    scheduler.add(std::make_unique<Constant>("joint/axle/torque", units::N_m, 0.05));
    wire(scheduler, world);
    ASSERT_TRUE(scheduler.stepUntil(SimTime{5'000'000'000}).ok());
    EXPECT_NEAR(read(scheduler, "joint/axle/position"), 2.0, 0.02);
}

TEST(MujocoBackend, AnActuatedJointNothingDrivesIsFree) {
    // A robot without a controller still loads (an arm without ROS, `lagrange graph`): its
    // actuated joints get no torque. A writer with the wrong unit is still refused (above).
    const World world = flywheel(0.02);
    Scheduler scheduler;
    scheduler.add(std::make_unique<MujocoBackend>());
    auto result = scheduler.wire(WorldView(world));
    ASSERT_TRUE(result) << to_string(result.error().front());
    ASSERT_TRUE(scheduler.stepUntil(SimTime{100'000'000}).ok());
    const SignalId speed = scheduler.bus().find("joint/axle/velocity");
    EXPECT_EQ(scheduler.bus().read(speed), 0.0);
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

TEST(MujocoBackend, LinksOfOneTreeDoNotCollideButTreesDo) {
    // A ball hangs from an arm on a slider so that it overlaps the arm's base block: inside one
    // tree that overlap is ignored, so the slider moves freely under a small force.
    World world;
    world.gravityLocal = {0.0, 0.0, 0.0};
    const BodyId base = world.addBody("base", kWorldBody, Pose{}, 5.0, {}, diagonal(0.1, 0.1, 0.1));
    world.addJoint("rail", JointType::Prismatic, base, {1.0, 0.0, 0.0}, true);
    world.addCollider(base, ShapeType::Box, {0.2, 0.2, 0.2}, Pose{}, 1.0);
    const BodyId arm = world.addBody("arm", base, Pose{}, 0.5, {}, diagonal(0.01, 0.01, 0.01));
    world.addJoint("pin", JointType::Revolute, arm, {0.0, 0.0, 1.0}, false);
    const BodyId ball =
        world.addBody("ball", arm, Pose{{0.15, 0.0, 0.0}}, 0.2, {}, diagonal(0.001, 0.001, 0.001));
    world.addCollider(ball, ShapeType::Sphere, {0.1, 0.0, 0.0}, Pose{}, 1.0);
    // A separate free block that the base will run into.
    const BodyId wall =
        world.addBody("wall", kWorldBody, Pose{{1.0, 0.0, 0.0}}, 50.0, {}, diagonal(1.0, 1.0, 1.0));
    world.addJoint("wall_free", JointType::Free, wall, {0.0, 0.0, 1.0}, false);
    world.addCollider(wall, ShapeType::Box, {0.1, 0.5, 0.5}, Pose{}, 1.0);

    Scheduler scheduler;
    scheduler.add(std::make_unique<MujocoBackend>());
    scheduler.add(std::make_unique<Constant>("joint/rail/torque", units::N, 5.0));
    wire(scheduler, world);
    ASSERT_TRUE(scheduler.stepUntil(SimTime{400'000'000}).ok());
    // Free motion: x = F t² / (2 m) with the whole tree's mass (5.7 kg), 0.07 m after 0.4 s.
    EXPECT_NEAR(read(scheduler, "joint/rail/position"), 5.0 * 0.16 / (2.0 * 5.7), 2e-3);
    ASSERT_TRUE(scheduler.stepUntil(SimTime{3'000'000'000}).ok());
    // Separate trees collide: the wall was pushed.
    EXPECT_GT(read(scheduler, "body/wall/pose", 0), 1.0 + 1e-3);
}

// --- kinematic bodies ----------------------------------------------------------------------

/// Moves a kinematic body along x at a constant speed through its pose command.
class Mover final : public IModule {
public:
    Mover(std::string body, double startX, double speed)
        : body_(std::move(body)), startX_(startX), speed_(speed) {}
    [[nodiscard]] ModuleInfo info() const override { return {"controller", "mover"}; }
    [[nodiscard]] Duration preferredStep() const override { return 1_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec spec;
        spec.name = "body/" + body_ + "/pose_command";
        spec.type = SignalType::Pose;
        spec.unit = units::m;
        spec.frame = "world_local";
        id_ = d.output(spec);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        write(bus, 0.0);
        return {};
    }
    Status step(const StepContext& context, SignalBus& bus) override {
        write(bus, (context.t + context.dt).seconds());
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    void write(SignalBus& bus, double t) const {
        bus.writeN(id_, std::array<double, 7>{startX_ + speed_ * t, 0.0, 0.15, 1.0, 0.0, 0.0, 0.0});
    }
    std::string body_;
    double startX_, speed_;
    SignalId id_;
};

/// A box on the ground, a kinematic pusher coming at it from 6 m away, and a crowd of kinematic
/// bodies standing on the ground far off (which never touch anything).
World pushedBox() {
    World world = boxOnGround(0.0, 0.5);
    const auto kinematic = [&world](const std::string& name, const Pose& pose) {
        const BodyId b = world.addBody(name, kWorldBody, pose, 10.0, {}, diagonal(0.1, 0.1, 0.1));
        world.bodies.kinematic[b.index] = true;
        world.addCollider(b, ShapeType::Box, {0.15, 0.3, 0.15}, Pose{}, 0.5);
        return b;
    };
    kinematic("pusher", Pose{{6.0, 0.0, 0.15}});
    for (int k = 0; k < 20; ++k) {
        kinematic("crowd_" + std::to_string(k), Pose{{-20.0 - k, 3.0 * (k % 3), 0.14}});
    }
    return world;
}

TEST(MujocoBackend, CullingFarKinematicBodiesLeavesTheRunUnchanged) {
    // The pusher starts out of reach (left out of the broad phase), comes near and pushes the
    // box: the state matches a run that never culls, bit for bit, at every step.
    const World world = pushedBox();
    std::array<Scheduler, 2> runs{Scheduler(SchedulerOptions{.deterministic = true}),
                                  Scheduler(SchedulerOptions{.deterministic = true})};
    std::array<MujocoBackend*, 2> backends{};
    for (std::size_t r = 0; r < 2; ++r) {
        auto backend = std::make_unique<MujocoBackend>();
        backend->setKinematicCulling(r == 0);
        backends.at(r) = backend.get();
        runs.at(r).add(std::move(backend));
        runs.at(r).add(std::make_unique<Mover>("pusher", 6.0, -2.0));
        wire(runs.at(r), world);
    }
    std::size_t mostCulled = 0;
    std::size_t leastCulled = 100;
    for (std::int64_t ms = 10; ms <= 4000; ms += 10) {
        for (Scheduler& run : runs) {
            ASSERT_TRUE(run.stepUntil(SimTime{ms * 1'000'000}).ok());
        }
        ASSERT_EQ(runs[0].stateHash().value(), runs[1].stateHash().value()) << "at " << ms << " ms";
        mostCulled = std::max(mostCulled, backends[0]->culledKinematicBodies());
        leastCulled = std::min(leastCulled, backends[0]->culledKinematicBodies());
    }
    EXPECT_EQ(backends[1]->culledKinematicBodies(), 0U);
    EXPECT_EQ(mostCulled, 21U) << "the pusher and the crowd while far";
    EXPECT_EQ(leastCulled, 20U) << "the pusher while near the box";
    // It was pushed: the pusher passed x = 0 at 3 s and is at -2 m.
    EXPECT_LT(read(runs[0], "body/box/pose", 0), -1.5);
}

TEST(MujocoBackend, AWorldWithoutDegreesOfFreedomStillMovesItsKinematicBodies) {
    // Scenery and one kinematic body only (as the town without a robot): the step skips the
    // dynamics, and the body still follows its command with its twist from its poses.
    World world;
    world.addCollider(kWorldBody, ShapeType::Plane, {5.0, 5.0, 0.1}, Pose{}, 1.0);
    const BodyId b = world.addBody("pusher", kWorldBody, Pose{{6.0, 0.0, 0.15}}, 10.0, {},
                                   diagonal(0.1, 0.1, 0.1));
    world.bodies.kinematic[b.index] = true;
    world.addCollider(b, ShapeType::Box, {0.15, 0.3, 0.15}, Pose{}, 0.5);
    Scheduler scheduler;
    scheduler.add(std::make_unique<MujocoBackend>());
    scheduler.add(std::make_unique<Mover>("pusher", 6.0, -2.0));
    wire(scheduler, world);
    ASSERT_TRUE(scheduler.stepUntil(SimTime{1'000'000'000}).ok());
    EXPECT_NEAR(read(scheduler, "body/pusher/pose", 0), 4.0, 1e-9);
    EXPECT_NEAR(read(scheduler, "body/pusher/pose", 2), 0.15, 1e-12);
    EXPECT_NEAR(read(scheduler, "body/pusher/twist", 0), -2.0, 1e-6);
    EXPECT_DOUBLE_EQ(read(scheduler, "body/pusher/acceleration", 2), 0.0);
}

/// Moves a kinematic body along x at 1 m/s, except while it stands from 0.5 s to 1 s.
class StopAndGo final : public IModule {
public:
    [[nodiscard]] ModuleInfo info() const override { return {"controller", "stop_and_go"}; }
    [[nodiscard]] Duration preferredStep() const override { return 1_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec spec;
        spec.name = "body/pusher/pose_command";
        spec.type = SignalType::Pose;
        spec.unit = units::m;
        spec.frame = "world_local";
        id_ = d.output(spec);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        write(bus, 0.0);
        return {};
    }
    Status step(const StepContext& context, SignalBus& bus) override {
        write(bus, (context.t + context.dt).seconds());
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    void write(SignalBus& bus, double t) const {
        const double moved = std::min(t, 0.5) + std::max(t - 1.0, 0.0);  // m, at 1 m/s
        bus.writeN(id_, std::array<double, 7>{6.0 - moved, 0.0, 0.15, 1.0, 0.0, 0.0, 0.0});
    }
    SignalId id_;
};

TEST(MujocoBackend, AKinematicBodyThatStandsHasNoTwistAndMovesOnAgain) {
    // Its outputs are written only when they change; standing, its twist (from two equal poses)
    // is not worked out again. A run restored while it stands carries on bit for bit.
    const World world = pushedBox();
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<MujocoBackend>());
    scheduler.add(std::make_unique<StopAndGo>());
    wire(scheduler, world);
    ASSERT_TRUE(scheduler.stepUntil(SimTime{400'000'000}).ok());
    EXPECT_NEAR(read(scheduler, "body/pusher/twist", 0), -1.0, 1e-6);
    ASSERT_TRUE(scheduler.stepUntil(SimTime{700'000'000}).ok());
    StateWriter saved;
    ASSERT_TRUE(scheduler.saveState(saved).ok());
    for (std::size_t k = 0; k < 6; ++k) {
        EXPECT_EQ(read(scheduler, "body/pusher/twist", k), 0.0) << k;
    }
    EXPECT_NEAR(read(scheduler, "body/pusher/pose", 0), 5.5, 1e-12);
    ASSERT_TRUE(scheduler.stepUntil(SimTime{1'200'000'000}).ok());
    EXPECT_NEAR(read(scheduler, "body/pusher/twist", 0), -1.0, 1e-6);
    EXPECT_NEAR(read(scheduler, "body/pusher/pose", 0), 5.3, 1e-9);
    const std::uint64_t first = scheduler.stateHash().value();

    StateReader reader(saved.bytes());
    ASSERT_TRUE(scheduler.restoreState(reader).ok());
    ASSERT_TRUE(scheduler.stepUntil(SimTime{1'200'000'000}).ok());
    EXPECT_EQ(scheduler.stateHash().value(), first);
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
