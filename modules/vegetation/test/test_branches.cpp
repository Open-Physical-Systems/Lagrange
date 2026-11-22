// Branches (docs/physics/ground.md "Branches") against their closed forms: the pipe model's limb
// diameters D √(L/Λ) and cross-sections that add up at every junction (Leonardo's rule: d/√2
// per fork); a disc meeting a twig cuts it for E = σ π d²/4; a disc meeting a branch thicker
// than tree:maxCutDiameter is stopped, the push taking the propeller's angular momentum I ω in
// the step; the wood is a pure function of the tree and its index, and survives save/restore.
#include "lagrange/vegetation/Branches.hpp"
#include "lagrange/vegetation/Vegetation.hpp"

#include <cmath>
#include <cstdio>
#include <numbers>
#include <span>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::vegetation {
namespace {

using V3 = std::array<double, 3>;

// The open field's tree_1 (assets/worlds/open_field), at the origin, with the fallbacks.
TreeSpec tree() {
    TreeSpec t;
    t.name = "tree_1";
    t.height = 6.1;
    t.crownRadius = 1.8;
    t.crownBase = 2.5;
    t.trunkDiameter = 0.5;
    t.leafAreaDensity = kTreeLeafAreaDensity;
    t.dragCoefficient = kTreeDragCoefficient;
    t.naturalFrequency = kTreeNaturalFrequency;
    t.dampingRatio = kTreeDampingRatio;
    t.crownMass = kTreeCrownDensity * t.crownVolume();
    t.cutEnergy = kTreeCutEnergy;
    t.leafArea = kTreeLeafArea;
    t.branchCount = kTreeBranchCount;
    t.woodCutEnergy = kTreeWoodCutEnergy;
    t.maxCutDiameter = kTreeMaxCutDiameter;
    return t;
}

double area(double d) {
    return std::numbers::pi * d * d / 4.0;
}

V3 world(const TreeSpec& t, const V3& crown) {  // the tree stands at the origin, unrotated
    return {crown[0], crown[1], crown[2] + t.centreHeight()};
}

TEST(Branches, ThePipeModelAndLeonardosRule) {
    const TreeSpec t = tree();
    const Skeleton s = skeletonOf(t, 0);
    ASSERT_EQ(s.limbs.size(), static_cast<std::size_t>(t.branchCount) + 1U);
    double total = 0.0;
    for (const double l : s.limbs) {
        EXPECT_GT(l, 0.0);
        total += l;
    }
    EXPECT_DOUBLE_EQ(limbDiameter(0.5, 1.0, 4.0), 0.25);
    EXPECT_DOUBLE_EQ(forkDiameter(0.2, 2), 0.1);
    // Every segment's children: generation g+1 at d/√2, and the cross-sections add up at a fork
    // (children's sum = parent's) and at a junction of the trunk (limb + trunk above = below).
    double limbBases = 0.0;
    int forks = 0;
    int clipped = 0;
    for (std::size_t i = 0; i < s.segments.size(); ++i) {
        const BranchSegment& b = s.segments[i];
        double children = 0.0;
        int count = 0;
        for (std::size_t j = i + 1; j < b.end; j = s.segments[j].end) {
            const BranchSegment& c = s.segments[j];
            children += area(c.diameter);
            ++count;
            if (b.generation >= 0) {
                EXPECT_EQ(c.generation, b.generation + 1);
                EXPECT_NEAR(c.diameter, b.diameter / std::numbers::sqrt2, 1e-15);
            } else if (c.generation == 0) {
                limbBases += area(c.diameter);
            }
        }
        if (count > 0) {
            EXPECT_NEAR(children, area(b.diameter), 1e-12 * area(t.trunkDiameter)) << i;
            forks += b.generation >= 0 ? 1 : 0;
        } else {
            // A tip: its children would be under kTwigDiameter, or it reached the crown's edge.
            EXPECT_GE(b.diameter, kTwigDiameter);
            const double x = b.to[0] / t.crownRadius;
            const double y = b.to[1] / t.crownRadius;
            const double z = b.to[2] / t.halfHeight();
            if (b.diameter / std::numbers::sqrt2 >= kTwigDiameter) {
                EXPECT_NEAR(x * x + y * y + z * z, 1.0, 1e-9) << i;
                ++clipped;
            }
        }
    }
    EXPECT_GT(forks, 1000);
    // The crown base's trunk carries every limb and the leader: πD²/4.
    EXPECT_NEAR(area(s.segments[0].diameter), s.trunkArea, 1e-15);
    EXPECT_NEAR(limbBases, s.trunkArea, 1e-12);
    // Limb k's base diameter is D √(L_k / Λ).
    std::size_t k = 0;
    for (std::size_t i = 0; i < s.segments.size(); ++i) {
        if (s.segments[i].generation == 0) {
            EXPECT_NEAR(s.segments[i].diameter, limbDiameter(0.5, s.limbs.at(k), total), 1e-15);
            ++k;
        }
    }
    EXPECT_EQ(k, s.limbs.size());
    std::printf(
        "tree_1: %zu segments, limb bases %.3f-%.3f m, leader %.3f m, %d forks, %d tips at "
        "the crown's edge\n",
        s.segments.size(), limbDiameter(0.5, s.limbs[0], total),
        limbDiameter(0.5, s.limbs[t.branchCount - 1], total),
        limbDiameter(0.5, s.limbs.back(), total), forks, clipped);
}

TEST(Branches, TheWoodIsAFunctionOfTheTreeAndItsIndex) {
    const Skeleton a = skeletonOf(tree(), 0);
    const Skeleton b = skeletonOf(tree(), 0);
    const Skeleton c = skeletonOf(tree(), 1);
    ASSERT_EQ(a.segments.size(), b.segments.size());
    for (std::size_t i = 0; i < a.segments.size(); ++i) {
        EXPECT_EQ(a.segments[i].to, b.segments[i].to);
    }
    EXPECT_NE(a.segments[2].to, c.segments[2].to);  // the first limb turned by the index
    // The wood stays in the crown: a branch that would leave it ends at its edge.
    const TreeSpec t = tree();
    for (const BranchSegment& s : a.segments) {
        const double x = s.to[0] / t.crownRadius;
        const double y = s.to[1] / t.crownRadius;
        const double z = s.to[2] / t.halfHeight();
        EXPECT_LE(x * x + y * y + z * z, 1.0 + 1e-9);
    }
}

TEST(Branches, ADiscMeetsASegmentWhereItCrossesTheSheet) {
    // Upright segment 5 cm off the axis of a level disc of 10 cm: met at the sheet's underside.
    const double d = 0.002;
    const double t = kBladeHalfThickness + d / 2.0;
    EXPECT_NEAR(discMeets({0.05, 0.0, -1.0}, {0.05, 0.0, 1.0}, d, {}, {0.0, 0.0, 1.0}, 0.1),
                1.0 - t, 1e-12);
    EXPECT_LT(discMeets({0.15, 0.0, -1.0}, {0.15, 0.0, 1.0}, d, {}, {0.0, 0.0, 1.0}, 0.1), 0.0);
    // Lying in the sheet, it is met where it enters the disc.
    EXPECT_NEAR(discMeets({-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, d, {}, {0.0, 0.0, 1.0}, 0.1), 0.9,
                1e-12);
    EXPECT_LT(discMeets({-1.0, 0.0, 0.1}, {1.0, 0.0, 0.1}, d, {}, {0.0, 0.0, 1.0}, 0.1), 0.0);
}

// The first twig with children, or the first branch of the second fork well over the cut.
std::size_t find(const Skeleton& s, bool twig) {
    const double cut = kTreeMaxCutDiameter;
    for (std::size_t i = 0; i < s.segments.size(); ++i) {
        const BranchSegment& b = s.segments[i];
        const double length =
            std::hypot(b.to[0] - b.from[0], b.to[1] - b.from[1], b.to[2] - b.from[2]);
        if (twig ? (b.diameter <= cut && b.end > i + 1 && length > 0.06)
                 : (b.generation == 2 && b.diameter > 2.0 * cut && length > 0.1)) {
            return i;
        }
    }
    return s.segments.size();
}

V3 unit(const V3& v) {
    const double n = std::hypot(v[0], v[1], v[2]);
    return {v[0] / n, v[1] / n, v[2] / n};
}

TEST(Branches, ATwigIsCutForItsCrossSection) {
    const TreeSpec t = tree();
    Branches branches;
    branches.build({t});
    branches.addRotor({0.0, 0.0, 0.0});
    const std::size_t i = find(branches.skeleton(0), true);
    ASSERT_LT(i, branches.skeleton(0).segments.size());
    const BranchSegment s = branches.skeleton(0).segments[i];
    const V3 u = unit({s.to[0] - s.from[0], s.to[1] - s.from[1], s.to[2] - s.from[2]});
    const V3 mid{(s.from[0] + s.to[0]) / 2.0, (s.from[1] + s.to[1]) / 2.0,
                 (s.from[2] + s.to[2]) / 2.0};
    // A small disc square to the twig at its middle: only the twig crosses it.
    const Pose hub{world(t, mid), {1.0, 0.0, 0.0, 0.0}};
    const std::vector<V3> still(1);
    const BranchHit hit = branches.sweep(0, {t}, hub, u, 0.01, still, 600.0, 9e-5, 1e-3);
    EXPECT_FALSE(hit.strike);
    ASSERT_EQ(hit.twigsCut, 1);
    EXPECT_NEAR(hit.cutEnergy, t.woodCutEnergy * area(s.diameter), 1e-12);
    std::printf("a %.1f mm twig: %.3f J to cut (σ %.0f J/m²)\n", s.diameter * 1e3, hit.cutEnergy,
                t.woodCutEnergy);
    // Cut where the sheet meets it: what is beyond is gone.
    EXPECT_LT(branches.standing(0, i), 0.5);
    EXPECT_GT(branches.standing(0, i), 0.3);
    for (std::size_t j = i + 1; j < s.end; ++j) {
        EXPECT_EQ(branches.standing(0, j), 0.0);
    }
    // Again: nothing more to cut there.
    EXPECT_EQ(branches.sweep(0, {t}, hub, u, 0.01, still, 600.0, 9e-5, 1e-3).twigsCut, 0);
    // Save and restore into a fresh one.
    StateWriter w;
    branches.save(w);
    Branches back;
    back.build({t});
    back.addRotor({0.0, 0.0, 0.0});
    StateReader r(w.bytes());
    ASSERT_TRUE(back.restore(r));
    for (std::size_t j = 0; j < s.end; ++j) {
        EXPECT_EQ(back.standing(0, j), branches.standing(0, j));
    }
}

TEST(Branches, ABranchStopsThePropeller) {
    const TreeSpec t = tree();
    Branches branches;
    branches.build({t});
    branches.addRotor({0.0, 0.0, 0.0});
    const std::size_t i = find(branches.skeleton(0), false);
    ASSERT_LT(i, branches.skeleton(0).segments.size());
    const BranchSegment s = branches.skeleton(0).segments[i];
    ASSERT_GT(s.diameter, t.maxCutDiameter);
    const V3 u = unit({s.to[0] - s.from[0], s.to[1] - s.from[1], s.to[2] - s.from[2]});
    const V3 mid{(s.from[0] + s.to[0]) / 2.0, (s.from[1] + s.to[1]) / 2.0,
                 (s.from[2] + s.to[2]) / 2.0};
    // A 10-inch propeller whose axis runs along the branch, its hub 10 cm to one side, square to
    // the fork's plane (so the branch's sibling passes outside the disc): the blade meets the
    // branch 10 cm from the hub, moving square to it.
    const std::vector<BranchSegment>& all = branches.skeleton(0).segments;
    std::size_t parent = 0;
    for (std::size_t j = 0; j < i; ++j) {
        if (all[j].generation == s.generation - 1 && all[j].end > i) {
            parent = j;
        }
    }
    std::size_t sibling = parent + 1;
    while (sibling == i) {
        sibling = all[sibling].end;
    }
    const BranchSegment& sib = all[sibling];
    const V3 w = unit({sib.to[0] - sib.from[0], sib.to[1] - sib.from[1], sib.to[2] - sib.from[2]});
    const V3 side =
        unit({u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]});
    const double r = 0.1;
    const V3 hubAt{mid[0] + r * side[0], mid[1] + r * side[1], mid[2] + r * side[2]};
    const Pose hub{world(t, hubAt), {1.0, 0.0, 0.0, 0.0}};
    const double inertia = 9e-5;  // quad_x500's rotor link about its axis
    const double omega = 700.0;
    const double dt = 1e-3;
    const std::vector<V3> still(1);
    const BranchHit hit = branches.sweep(0, {t}, hub, u, 0.127, still, omega, inertia, dt);
    ASSERT_TRUE(hit.strike);
    EXPECT_NEAR(hit.diameter, s.diameter, 1e-15);
    // The push about the axis over the step takes the angular momentum: I ω' = I ω + τ dt = 0.
    const double torque = hit.wrench[3] * u[0] + hit.wrench[4] * u[1] + hit.wrench[5] * u[2];
    EXPECT_NEAR(inertia * omega + torque * dt, 0.0, 1e-12);
    // Through the contact, 10 cm out, square to the limb and to the blade's motion's opposite.
    const double force = std::hypot(hit.wrench[0], hit.wrench[1], hit.wrench[2]);
    EXPECT_NEAR(force, inertia * omega / (dt * r), 1e-9);
    EXPECT_NEAR(hit.wrench[0] * u[0] + hit.wrench[1] * u[1] + hit.wrench[2] * u[2], 0.0, 1e-9);
    const V3 blade{u[1] * -side[2] - u[2] * -side[1], u[2] * -side[0] - u[0] * -side[2],
                   u[0] * -side[1] - u[1] * -side[0]};  // ω̂ × r, r from the hub to the limb
    EXPECT_LT(hit.wrench[0] * blade[0] + hit.wrench[1] * blade[1] + hit.wrench[2] * blade[2],
              -0.99 * force);
    // The strike took ½ I ω², and as much over kBladeBreakEnergy of the blades.
    EXPECT_NEAR(hit.energy, 0.5 * inertia * omega * omega, 1e-12);
    EXPECT_NEAR(branches.damage(0), std::min(1.0, hit.energy / kBladeBreakEnergy), 1e-12);
    std::printf(
        "a %.0f mm limb stops a propeller at %.0f rad/s: %.0f N for 1 ms, %.1f J taken, "
        "damage %.2f\n",
        s.diameter * 1e3, omega, force, hit.energy, branches.damage(0));
    // Nothing was cut, and the limb still stands.
    EXPECT_EQ(hit.twigsCut, 0);
    EXPECT_EQ(branches.standing(0, i), 1.0);
    // Spinning the other way, the push turns round and still stops it.
    const BranchHit back = branches.sweep(0, {t}, hub, u, 0.127, still, -omega, inertia, dt);
    ASSERT_TRUE(back.strike);
    const double t2 = back.wrench[3] * u[0] + back.wrench[4] * u[1] + back.wrench[5] * u[2];
    EXPECT_NEAR(-inertia * omega + t2 * dt, 0.0, 1e-12);
}

// Bodies against the wood (Branches.hpp "What a body does"): a 6 cm ball pushed by a steady
// 5 N into a branch, square to it, comes to rest against it: the wood's push balances the 5 N,
// the overlap is the spring's F/k with k = m/τ² (critically damped, c = 2m/τ), the ball's centre
// stands r + R - δ from the branch's axis, and the crown is pushed back by the same force.
TEST(Branches, ABodyPushedIntoABranchStops) {
    const TreeSpec t = tree();
    Branches branches;
    branches.build({t});
    const std::vector<BranchSegment>& segs = branches.skeleton(0).segments;
    const std::size_t i = find(branches.skeleton(0), false);  // a 71 mm branch
    ASSERT_LT(i, segs.size());
    const BranchSegment s = segs[i];
    ASSERT_GT(s.diameter, t.maxCutDiameter);
    const V3 u = unit({s.to[0] - s.from[0], s.to[1] - s.from[1], s.to[2] - s.from[2]});
    // Square to the fork's plane, so that the branch's sibling is not in the way.
    std::size_t sibling = 0;
    for (std::size_t j = 0; j < i; ++j) {
        if (segs[j].generation == s.generation - 1 && segs[j].end > i) {
            sibling = j + 1;
        }
    }
    while (sibling == i) {
        sibling = segs[sibling].end;
    }
    const V3 w = unit({segs[sibling].to[0] - segs[sibling].from[0],
                       segs[sibling].to[1] - segs[sibling].from[1],
                       segs[sibling].to[2] - segs[sibling].from[2]});
    const V3 side =
        unit({u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]});
    const V3 mid = world(
        t, {(s.from[0] + s.to[0]) / 2.0, (s.from[1] + s.to[1]) / 2.0, (s.from[2] + s.to[2]) / 2.0});
    const double radius = 0.03;
    const double mass = 1.0;
    const double push = 5.0;
    WoodShape ball;
    ball.type = ShapeType::Sphere;
    ball.size = {radius, 0.0, 0.0};
    ball.bound = radius;
    const double gap = 0.5 * s.diameter + radius + 0.02;
    V3 x{mid[0] + gap * side[0], mid[1] + gap * side[1], mid[2] + gap * side[2]};
    V3 v{};
    const double dt = 1e-3;
    BodyContact last;
    std::vector<V3> crownPush(1);
    const std::vector<V3> still(1);
    for (int k = 0; k < 3000; ++k) {  // 3 s, 150 τ
        crownPush.assign(1, V3{});
        last = branches.touch({t}, std::span<const WoodShape>(&ball, 1), radius,
                              Pose{x, {1.0, 0.0, 0.0, 0.0}}, {v[0], v[1], v[2], 0.0, 0.0, 0.0},
                              {0.0, 0.0, 0.0}, mass, still, still, crownPush);
        for (std::size_t c = 0; c < 3; ++c) {
            v.at(c) += dt * (last.wrench.at(c) - push * side.at(c)) / mass;
            x.at(c) += dt * v.at(c);
        }
    }
    ASSERT_EQ(last.contacts, 1);
    EXPECT_EQ(last.broken, 0);
    EXPECT_LT(std::hypot(v[0], v[1], v[2]), 1e-6) << "it stopped";
    // The force balance: the wood's push is the 5 N, back along the side.
    for (std::size_t c = 0; c < 3; ++c) {
        EXPECT_NEAR(last.wrench.at(c), push * side.at(c), 1e-6 * push);
        EXPECT_NEAR(last.wrench.at(c + 3), 0.0, 1e-6);  // through the ball's centre
    }
    EXPECT_NEAR(last.force, push, 1e-6);
    const double k = mass / (kWoodContactTime * kWoodContactTime);
    EXPECT_NEAR(last.depth, push / k, 1e-6);
    const V3 off{x[0] - mid[0], x[1] - mid[1], x[2] - mid[2]};
    EXPECT_NEAR(off[0] * side[0] + off[1] * side[1] + off[2] * side[2],
                0.5 * s.diameter + radius - push / k, 1e-6);
    // And the crown is pushed the other way, level.
    EXPECT_NEAR(crownPush[0][0], -push * side[0], 1e-6);
    EXPECT_NEAR(crownPush[0][1], -push * side[1], 1e-6);
    std::printf("a 6 cm ball pushed by %.0f N rests on a %.0f mm branch, %.2f mm into it (F/k)\n",
                push, s.diameter * 1e3, last.depth * 1e3);
}

// A ball moving through a twig no thicker than tree:maxCutDiameter breaks it: no push, the twig
// and what grows beyond it gone, counted with the propellers' twigs.
TEST(Branches, ABodyBreaksATwig) {
    const TreeSpec t = tree();
    Branches branches;
    branches.build({t});
    const std::size_t i = find(branches.skeleton(0), true);
    ASSERT_LT(i, branches.skeleton(0).segments.size());
    const BranchSegment s = branches.skeleton(0).segments[i];
    const V3 u = unit({s.to[0] - s.from[0], s.to[1] - s.from[1], s.to[2] - s.from[2]});
    const V3 side = unit({u[1], -u[0], 0.0});
    const V3 mid = world(
        t, {(s.from[0] + s.to[0]) / 2.0, (s.from[1] + s.to[1]) / 2.0, (s.from[2] + s.to[2]) / 2.0});
    WoodShape ball;
    ball.type = ShapeType::Sphere;
    ball.size = {0.02, 0.0, 0.0};
    ball.bound = 0.02;
    std::vector<V3> push(1);
    const std::vector<V3> still(1);
    int broken = 0;
    for (int k = -10; k <= 10; ++k) {  // across it, 1 cm a step
        const double at = 0.01 * k;
        const V3 x{mid[0] + at * side[0], mid[1] + at * side[1], mid[2] + at * side[2]};
        const BodyContact c =
            branches.touch({t}, std::span<const WoodShape>(&ball, 1), 0.02,
                           Pose{x, {1.0, 0.0, 0.0, 0.0}}, {}, {}, 1.0, still, still, push);
        broken += c.broken;
        EXPECT_EQ(c.contacts, 0) << "a twig does not push";
    }
    EXPECT_GE(broken, 1);
    EXPECT_LT(branches.standing(0, i), 1.0);
    for (std::size_t j = i + 1; j < s.end; ++j) {
        EXPECT_EQ(branches.standing(0, j), 0.0);
    }
    // The broken wood is in the signal the lidars read.
    TreeWood seen;
    seen.build({t});
    seen.load(branches.cutValues());
    EXPECT_EQ(seen.standing(0, i), branches.standing(0, i));
}

// Wood inside a fixed collider of the world (the trunk the stage gives the tree) is its: a ball
// overlapping the trunk's crown segment there is not pushed by the wood a second time.
TEST(Branches, TheTrunkColliderKeepsItsWood) {
    const TreeSpec t = tree();
    Branches branches;
    branches.build({t});
    WoodShape ball;
    ball.type = ShapeType::Sphere;
    ball.size = {0.1, 0.0, 0.0};
    ball.bound = 0.1;
    const Pose at{{0.25 + 0.1 - 0.01, 0.0, t.crownBase + 0.1}, {1.0, 0.0, 0.0, 0.0}};
    std::vector<V3> push(1);
    const std::vector<V3> still(1);
    const BodyContact bare = branches.touch({t}, std::span<const WoodShape>(&ball, 1), 0.1, at, {},
                                            {}, 1.0, still, still, push);
    EXPECT_EQ(bare.contacts, 1) << "the wood alone pushes";
    WoodShape trunk;
    trunk.type = ShapeType::Cylinder;
    trunk.size = {0.25, 1.0, 0.0};
    trunk.pose = {{0.0, 0.0, t.crownBase}, {1.0, 0.0, 0.0, 0.0}};
    trunk.bound = std::hypot(0.25, 1.0);
    branches.setFixed({trunk});
    const BodyContact kept = branches.touch({t}, std::span<const WoodShape>(&ball, 1), 0.1, at, {},
                                            {}, 1.0, still, still, push);
    EXPECT_EQ(kept.contacts, 0);
}

// The signed distances the contacts use, against their closed forms.
TEST(Branches, ShapeDistances) {
    WoodShape box;
    box.type = ShapeType::Box;
    box.size = {1.0, 2.0, 3.0};
    EXPECT_DOUBLE_EQ(shapeDistance(box, {2.0, 0.0, 0.0}), 1.0);
    EXPECT_DOUBLE_EQ(shapeDistance(box, {0.5, 0.0, 0.0}), -0.5);
    EXPECT_DOUBLE_EQ(shapeDistance(box, {4.0, 6.0, 0.0}), 5.0);  // past a corner: 3-4-5
    WoodShape cylinder;
    cylinder.type = ShapeType::Cylinder;
    cylinder.size = {1.0, 2.0, 0.0};
    EXPECT_DOUBLE_EQ(shapeDistance(cylinder, {3.0, 0.0, 0.0}), 2.0);
    EXPECT_DOUBLE_EQ(shapeDistance(cylinder, {0.0, 0.0, 2.5}), 0.5);
    EXPECT_DOUBLE_EQ(shapeDistance(cylinder, {0.0, 0.0, 1.5}), -0.5);
    WoodShape capsule;
    capsule.type = ShapeType::Capsule;
    capsule.size = {0.5, 1.0, 0.0};
    EXPECT_DOUBLE_EQ(shapeDistance(capsule, {0.0, 0.0, 2.0}), 0.5);
    EXPECT_DOUBLE_EQ(shapeDistance(capsule, {1.0, 0.0, 0.5}), 0.5);
}

TEST(Branches, TreeApiReadsTheBranchFallbacks) {
    World w;
    w.schemaInstances.push_back({});
    SchemaInstance& s = w.schemaInstances.back();
    s.schema = "TreeAPI_1";
    s.prim = w.addPrim("/oak", "oak");
    s.reals = {{"tree:height", 8.0}, {"tree:crownRadius", 2.0}};
    std::string error;
    std::vector<TreeSpec> trees = treesOf(w, error);
    ASSERT_EQ(trees.size(), 1U);
    EXPECT_EQ(trees[0].branchCount, kTreeBranchCount);
    EXPECT_DOUBLE_EQ(trees[0].woodCutEnergy, kTreeWoodCutEnergy);
    EXPECT_DOUBLE_EQ(trees[0].maxCutDiameter, kTreeMaxCutDiameter);
    s.integers["tree:branchCount"] = 3;
    trees = treesOf(w, error);
    ASSERT_EQ(trees.size(), 1U);
    EXPECT_EQ(trees[0].branchCount, 3);
    s.integers["tree:branchCount"] = -1;
    EXPECT_TRUE(treesOf(w, error).empty());
}

}  // namespace
}  // namespace lagrange::vegetation
