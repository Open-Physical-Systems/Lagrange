#include "lagrange/humans/Body.hpp"
#include "lagrange/humans/Gait.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
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

// Walking at 1.34 m/s: step rate and length as people's walk ratio says, about 110 steps/min and
// 0.72 m.
TEST(Gait, StepsAsPeopleDo) {
    const Stride stride = strideAt(1.75, 1.34);
    EXPECT_NEAR(stride.stepLength, 0.72, 0.02);
    EXPECT_NEAR(stride.cadence * 60.0, 112.0, 4.0);
    const double ratio = stride.stepLength / (stride.cadence * 60.0);
    EXPECT_GE(ratio, 0.006);
    EXPECT_LE(ratio, 0.007);
}

/// World position of a foot's point on the ground over the leg's stance, walking along x.
struct Slip {
    double most{0.0};
    double lowest{1e9};
    double highest{-1e9};
};

Slip stanceSlip(double height, double speed, double fromU, double toU) {
    const Proportions body(height);
    const double strideLength = 2.0 * strideAt(height, std::max(speed, kBlendSpeed)).stepLength;
    Slip out;
    bool first = true;
    double start = 0.0;
    for (double phase = kStanceFraction * fromU; phase < kStanceFraction * toU; phase += 0.002) {
        const GaitSample g = gaitAt(body, speed, phase);
        Pose pelvis;
        pelvis.position = {phase * strideLength, 0.0, g.pelvisHeight};
        const auto poses = segmentPoses(body, pelvis, g.posture);
        const std::array<double, 3> c =
            compose(poses[kFootLeft], Pose{contactPoint(body, phase), {1, 0, 0, 0}}).position;
        if (first) {
            start = c[0];
            first = false;
        }
        out.most = std::max(out.most, std::abs(c[0] - start));
        out.lowest = std::min(out.lowest, c[2]);
        out.highest = std::max(out.highest, c[2]);
    }
    return out;
}

// RFC 0009 validation: the stance foot does not skate (under 2 cm) and stays on the ground, in
// each part of stance, for short and tall people at slow and brisk speeds.
TEST(Gait, TheStanceFootStaysPlanted) {
    for (const double height : {1.55, 1.75, 1.95}) {
        for (const double speed : {0.8, 1.34, 1.7}) {
            for (const auto& [from, to] :
                 {std::pair{0.0, 0.119}, std::pair{0.12, 0.499}, std::pair{0.5, 0.999}}) {
                const Slip slip = stanceSlip(height, speed, from, to);
                EXPECT_LT(slip.most, 0.02) << height << " m, " << speed << " m/s, " << from;
                EXPECT_NEAR(slip.lowest, 0.0, 0.02) << height << " m, " << speed << " m/s";
                EXPECT_NEAR(slip.highest, 0.0, 0.02) << height << " m, " << speed << " m/s";
            }
        }
    }
}

// Joint ranges near normal walking at 1.34 m/s. Reference, widely reported for normal gait
// (TODO(verify) against Winter's tables): hip about 30° flexion to 10° extension, knee to about
// 60° in swing and nearly straight at the end of stance.
TEST(Gait, JointRangesAreThoseOfWalking) {
    const Proportions body(1.75);
    double hipMax = -9.0;
    double hipMin = 9.0;
    double kneeMax = -9.0;
    double kneeMin = 9.0;
    for (double phase = 0.0; phase < 1.0; phase += 0.002) {
        const Posture p = gaitAt(body, 1.34, phase).posture;
        hipMax = std::max(hipMax, p.hipLeft);
        hipMin = std::min(hipMin, p.hipLeft);
        kneeMax = std::max(kneeMax, p.kneeLeft);
        kneeMin = std::min(kneeMin, p.kneeLeft);
    }
    constexpr double kDegree = std::numbers::pi / 180.0;
    std::printf("hip %.1f..%.1f knee %.1f..%.1f deg\n", hipMin / kDegree, hipMax / kDegree,
                kneeMin / kDegree, kneeMax / kDegree);
    EXPECT_NEAR(hipMax, 30.0 * kDegree, 10.0 * kDegree);
    EXPECT_NEAR(hipMin, -10.0 * kDegree, 12.0 * kDegree);
    EXPECT_NEAR(kneeMax, 60.0 * kDegree, 15.0 * kDegree);
    EXPECT_LT(kneeMin, 12.0 * kDegree);
}

// Stopped, a person stands.
TEST(Gait, AtRestTheBodyStands) {
    const Proportions body(1.75);
    const GaitSample g = gaitAt(body, 0.0, 0.3);
    EXPECT_NEAR(g.pelvisHeight, body.hip, 1e-12);
    EXPECT_EQ(g.posture.kneeLeft, 0.0);
}

}  // namespace
}  // namespace lagrange::humans
