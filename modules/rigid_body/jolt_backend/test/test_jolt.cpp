// The Jolt backend on its own: closed-form pendulums. The robot-level comparison with MuJoCo is
// the agreement suite, server/test/test_backends.cpp.
#include "lagrange/rigid_body/JoltBackend.hpp"
#include "lagrange/scheduler/Scheduler.hpp"

#include <algorithm>
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

/// A point mass on a 1 m massless rod hinged to the world about y, `start` rad from hanging.
World pendulum(double start, double armature = 0.0, double lower = -INFINITY,
               double upper = INFINITY, double bobInertia = 1e-6) {
    World world;
    world.robots.push_back("pendulum");
    const Pose released{{0, 0, 2.0}, {std::cos(start / 2), 0, std::sin(start / 2), 0}};
    const BodyId bob = world.addBody("bob", kWorldBody, released, 1.0, {0, 0, -1.0},
                                     {bobInertia, bobInertia, bobInertia, 0, 0, 0});
    world.bodies.robot[bob.index] = 0;
    world.addJoint("pivot", std::isfinite(lower) ? JointType::Revolute : JointType::Continuous, bob,
                   {0, 1, 0}, true);
    world.joints.armature.back() = armature;
    world.joints.lowerLimit.back() = lower;
    world.joints.upperLimit.back() = upper;
    return world;
}

TEST(Jolt, ArmatureSlowsThePendulumAsItsInertiaSays) {
    // T = 2 pi sqrt((m L^2 + Ia) / (m g L)): an armature of 1 kg·m² doubles m L^2.
    const World world = pendulum(0.1, 1.0);
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<JoltBackend>());
    ASSERT_TRUE(scheduler.wire(WorldView(world)));
    const SignalBus& bus = scheduler.bus();
    const SignalId velocity = bus.find("joint/pivot/velocity");
    std::vector<double> turns;
    double previous = 0.0;
    for (int k = 0; k < 9000; ++k) {
        ASSERT_TRUE(scheduler.stepTick().ok());
        const double v = bus.read(velocity);
        if (k > 10 && v * previous < 0.0) {
            turns.push_back(scheduler.now().seconds());
        }
        previous = v;
    }
    ASSERT_GE(turns.size(), 4U);
    const double period =
        2.0 * (turns.back() - turns.front()) / static_cast<double>(turns.size() - 1);
    const double expected = 2.0 * std::numbers::pi * std::sqrt(2.0 / 9.80665);
    RecordProperty("period_s", std::to_string(period));
    EXPECT_NEAR(period, expected, 0.01 * expected);
    EXPECT_NEAR(bus.read(bus.find("joint/pivot/effective_inertia")), 2.0, 1e-3);
}

TEST(Jolt, AccelerationsAreWhatAnImuAndAMotorRead) {
    const World world = pendulum(0.3);
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<JoltBackend>());
    ASSERT_TRUE(scheduler.wire(WorldView(world)));
    ASSERT_TRUE(scheduler.stepTick().ok());
    ASSERT_TRUE(scheduler.stepTick().ok());
    const SignalBus& bus = scheduler.bus();
    // Joint positions count from the pose in the world tables, here 0.3 rad from hanging:
    // q'' = -(g / L) sin(0.3 + q).
    const double angle = 0.3 + bus.read(bus.find("joint/pivot/position"));
    EXPECT_NEAR(bus.read(bus.find("joint/pivot/acceleration")), -9.80665 * std::sin(angle), 0.02);
    // At the body origin, the pivot, which does not move: the proper acceleration is -g, in the
    // body frame (turned by `angle` about y), as MuJoCo reports it.
    const std::span<const double> a = bus.readN(bus.find("body/bob/acceleration"));
    EXPECT_NEAR(a[0], -9.80665 * std::sin(angle), 0.02);
    EXPECT_NEAR(a[1], 0.0, 0.02);
    EXPECT_NEAR(a[2], 9.80665 * std::cos(angle), 0.02);
}

TEST(Jolt, ALimitHoldsTheJointAndReportsItsTorque) {
    // Gravity's torque about +y on the bob 0.5 rad from hanging is -m g L sin(0.5): released
    // there (q = 0) on a lower limit at q = 0, the limit holds it with +m g L sin(0.5).
    const World world = pendulum(0.5, 0.0, 0.0, 1.0, 1e-3);
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<JoltBackend>());
    ASSERT_TRUE(scheduler.wire(WorldView(world)));
    ASSERT_TRUE(scheduler.stepUntil(SimTime{2'000'000'000}).ok());
    const SignalBus& bus = scheduler.bus();
    EXPECT_NEAR(bus.read(bus.find("joint/pivot/position")), 0.0, 0.01);
    EXPECT_NEAR(bus.read(bus.find("joint/pivot/velocity")), 0.0, 1e-3);
    const double held = 9.80665 * std::sin(0.5);
    EXPECT_NEAR(bus.read(bus.find("joint/pivot/constraint_torque")), held, 0.05 * held);
}

/// The energy a pendulum from 1 rad keeps after 60 s at 1 ms on `substeps` Jolt steps per step,
/// as a fraction of its start (I = 1 + 1e-6 kg·m² about the pivot, m g d = g).
double energyKept(std::int64_t substeps) {
    const World world = pendulum(1.0);
    ModuleConfigData config;
    config.values["substeps"] = substeps;
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<JoltBackend>(), ModuleConfig(config));
    EXPECT_TRUE(scheduler.wire(WorldView(world)));
    const SignalId q = scheduler.bus().find("joint/pivot/position");
    const SignalId w = scheduler.bus().find("joint/pivot/velocity");
    const auto energy = [&] {
        const double theta = 1.0 + scheduler.bus().read(q);
        const double omega = scheduler.bus().read(w);
        return 0.5 * (1.0 + 1e-6) * omega * omega + 9.80665 * (1.0 - std::cos(theta));
    };
    const double start = energy();
    for (int k = 0; k < 60'000; ++k) {
        EXPECT_TRUE(scheduler.stepTick().ok());
    }
    return energy() / start;
}

TEST(Jolt, APendulumLosesEnergyAsJointsAsConstraintsDoAndSubstepsDivideIt) {
    // Jolt keeps each body free and holds the joint by projecting velocities: a body turned by
    // ω Δt over a step has that share of its velocity made radial, and the projection takes it
    // off, (ω Δt)² of the kinetic energy each step. Semi-implicit Euler with that projection, on
    // a point mass in double precision, loses 16.35 % of this pendulum's energy in 60 s at 1 ms
    // (rigid_body.md §8.1); Jolt, in single precision, 16.4 %. Nothing of Lagrange's damps it:
    // the body damping is zero and the joint has no friction or motor.
    const double one = energyKept(1);
    const double four = energyKept(4);
    std::printf("energy kept over 60 s: %.4f (1 step), %.4f (4 sub-steps)\n", one, four);
    EXPECT_NEAR(1.0 - one, 0.1635, 0.01);
    // Four sub-steps: a quarter of the step, a quarter of the loss (and a little more in single
    // precision: 5.3 %, against 4.65 % for the double-precision model).
    EXPECT_LT(1.0 - four, 0.06);
    EXPECT_GT(1.0 - four, 0.03);
    EXPECT_LE(four, 1.0) << "never gains";
}

TEST(Jolt, SubstepsKeepTheReactionAndInventNoContact) {
    // A 1 kg bob hanging still from its pivot: the pivot holds it up with m g and nothing touches
    // it, whatever the number of Jolt steps per step (each sub-step's impulse is over dt / n).
    for (const std::int64_t substeps : {std::int64_t{1}, std::int64_t{4}}) {
        const World world = pendulum(0.0);
        ModuleConfigData config;
        config.values["substeps"] = substeps;
        Scheduler scheduler(SchedulerOptions{.deterministic = true});
        scheduler.add(std::make_unique<JoltBackend>(), ModuleConfig(config));
        ASSERT_TRUE(scheduler.wire(WorldView(world)));
        ASSERT_TRUE(scheduler.stepUntil(SimTime{500'000'000}).ok());
        const SignalBus& bus = scheduler.bus();
        const std::span<const double> reaction = bus.readN(bus.find("joint/pivot/reaction"));
        EXPECT_NEAR(reaction[0], 0.0, 0.05) << substeps;
        EXPECT_NEAR(reaction[1], 0.0, 0.05) << substeps;
        EXPECT_NEAR(reaction[2], 9.80665, 0.05) << substeps;
        const std::span<const double> contact = bus.readN(bus.find("body/bob/contact_force"));
        for (std::size_t k = 0; k < 3; ++k) {
            EXPECT_NEAR(contact[k], 0.0, 0.05) << substeps << " axis " << k;
        }
    }
}

TEST(Jolt, SubstepsInventNoContactOnASwingingPendulum) {
    // A bob swinging from 2 rad: the pivot's force changes within a step, so a reaction taken
    // from the last sub-step alone leaves part of the momentum change to "contact" (QA3 F2,
    // 4 % of m g at 4 sub-steps). The mean over every sub-step leaves next to none.
    for (const std::int64_t substeps : {std::int64_t{1}, std::int64_t{4}, std::int64_t{8}}) {
        const World world = pendulum(2.0);
        ModuleConfigData config;
        config.values["substeps"] = substeps;
        Scheduler scheduler(SchedulerOptions{.deterministic = true});
        scheduler.add(std::make_unique<JoltBackend>(), ModuleConfig(config));
        ASSERT_TRUE(scheduler.wire(WorldView(world)));
        const SignalBus& bus = scheduler.bus();
        const SignalId contactId = bus.find("body/bob/contact_force");
        const SignalId reactionId = bus.find("joint/pivot/reaction");
        double worst = 0.0;
        double strongest = 0.0;
        for (int k = 0; k < 3000; ++k) {
            ASSERT_TRUE(scheduler.stepTick().ok());
            if (k < 2) {
                continue;  // the first step's momentum change is from rest
            }
            const std::span<const double> c = bus.readN(contactId);
            worst = std::max(worst, std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]));
            const std::span<const double> r = bus.readN(reactionId);
            strongest = std::max(strongest, std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]));
        }
        std::printf("swinging pendulum, %lld sub-steps: largest contact %.4f N, pivot %.2f N\n",
                    static_cast<long long>(substeps), worst, strongest);
        EXPECT_LT(worst, 0.01) << substeps;
        // At the bottom the pivot holds m g (3 − 2 cos 2) = 3.83 m g (less what the joint loses).
        EXPECT_NEAR(strongest, 9.80665 * (3.0 - 2.0 * std::cos(2.0)), 0.5) << substeps;
    }
}

TEST(Jolt, RefusesSubstepsOutOfRange) {
    const World world = pendulum(0.1);
    for (const std::int64_t substeps : {std::int64_t{0}, std::int64_t{65}}) {
        ModuleConfigData config;
        config.values["substeps"] = substeps;
        Scheduler scheduler(SchedulerOptions{.deterministic = true});
        scheduler.add(std::make_unique<JoltBackend>(), ModuleConfig(config));
        EXPECT_FALSE(scheduler.wire(WorldView(world))) << substeps;
    }
}

}  // namespace
}  // namespace lagrange::rigid_body
