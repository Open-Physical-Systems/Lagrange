#include "lagrange/humans/Body.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

namespace lagrange::humans {
namespace {

// Standing on the ground: the ankles at their height, the head's top at the stature's scale.
TEST(Body, StandsOnTheGround) {
    const Proportions body(1.75);
    const auto poses = segmentPoses(body, standingPelvis(body, Pose{}), Posture{});
    EXPECT_NEAR(poses[kFootLeft].position[2], body.ankle, 1e-12);
    EXPECT_NEAR(poses[kFootRight].position[2], body.ankle, 1e-12);
    EXPECT_NEAR(poses[kFootLeft].position[1], -poses[kFootRight].position[1], 1e-12);
    EXPECT_NEAR(poses[kHead].position[2], 0.87 * 1.75, 1e-12);
}

// Flexion is forward: a flexed hip brings the knee ahead, a bent knee takes the ankle back.
TEST(Body, JointsBendTheWayPeopleDo) {
    const Proportions body(1.75);
    Posture posture;
    posture.hipLeft = 0.5;
    const Pose pelvis = standingPelvis(body, Pose{});
    auto poses = segmentPoses(body, pelvis, posture);
    EXPECT_GT(poses[kShinLeft].position[0], 0.1) << "the knee is ahead";
    posture = Posture{};
    posture.kneeLeft = 1.0;
    poses = segmentPoses(body, pelvis, posture);
    EXPECT_LT(poses[kFootLeft].position[0], -0.1) << "the ankle is behind";
    EXPECT_NEAR(poses[kShinLeft].position[0], 0.0, 1e-12);
    posture = Posture{};
    posture.shoulderLeft = 0.5;
    poses = segmentPoses(body, pelvis, posture);
    EXPECT_GT(poses[kForearmLeft].position[0], 0.05) << "the elbow is ahead";
    posture = Posture{};
    posture.ankleLeft = 0.5;
    poses = segmentPoses(body, pelvis, posture);
    const auto toe = rotate(poses[kFootLeft].orientation, {1.0, 0.0, 0.0});
    EXPECT_GT(toe[2], 0.4) << "toe up";
}

TEST(Body, AddsKinematicSegments) {
    Model model;
    Spec spec;
    spec.name = "ana";
    spec.start.position = {2.0, 1.0, 0.0};
    addHuman(model, spec);
    spec.name = "ana";
    addHuman(model, spec);
    ASSERT_EQ(model.world.bodies.names.size(), 2 * kSegments);
    EXPECT_EQ(model.world.bodies.names[0], "human_ana_pelvis");
    EXPECT_EQ(model.world.bodies.names[kSegments], "human_ana_2_pelvis") << "names stay unique";
    for (std::size_t b = 0; b < model.world.bodies.names.size(); ++b) {
        EXPECT_TRUE(model.world.bodies.kinematic[b]);
        EXPECT_EQ(model.world.bodies.parent[b], kWorldBody);
    }
    EXPECT_EQ(model.world.colliders.body.size(), 2 * kSegments);
    const auto specs = takeSpecs(model);
    ASSERT_EQ(specs.size(), 2U);
    EXPECT_EQ(specs[1].name, "ana_2");
    EXPECT_TRUE(model.world.schemaInstances.empty());
}

}  // namespace
}  // namespace lagrange::humans
