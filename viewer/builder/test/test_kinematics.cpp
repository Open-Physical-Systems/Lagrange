// The link tree and forward kinematics (RFC 0016 §3, plan WP-A).
#include "lagrange/viewer/builder/Kinematics.hpp"

#include <cmath>
#include <numbers>
#include <string>
#include <tuple>

#include <gtest/gtest.h>

namespace lagrange::viewer::builder {
namespace {

constexpr double kPi = std::numbers::pi;

Link link(const std::string& name) {
    Link out;
    out.name = name;
    return out;
}

Joint joint(const std::string& name, Joint::Type type, const std::string& parent,
            const std::string& child) {
    Joint out;
    out.name = name;
    out.type = type;
    out.parent = parent;
    out.child = child;
    return out;
}

/// A point `p` of link `index`, in the root frame.
std::array<double, 3> pointOf(const std::vector<Frame>& frames, std::size_t index,
                              std::array<double, 3> p) {
    Frame local;
    local.position = p;
    return compose(frames.at(index), local).position;
}

void expectPoint(const std::array<double, 3>& actual, const std::array<double, 3>& expected) {
    for (std::size_t k = 0; k < 3; ++k) {
        EXPECT_NEAR(actual.at(k), expected.at(k), 1e-12) << "component " << k;
    }
}

TEST(Kinematics, RevolutePrismaticContinuousMimic) {
    Document document;
    document.robot = "r";
    document.links = {link("base"), link("turret"), link("slider"), link("wheel"), link("finger")};
    Joint pan = joint("pan", Joint::Type::Revolute, "base", "turret");
    pan.origin.xyz = {0, 0, 0.5};
    pan.axis = {0, 0, 2};  // not unit: normalised for the motion
    Joint lift = joint("lift", Joint::Type::Prismatic, "turret", "slider");
    lift.origin.xyz = {1, 0, 0};
    lift.axis = {0, 0, 1};
    Joint spin = joint("spin", Joint::Type::Continuous, "base", "wheel");
    spin.origin.xyz = {0, -1, 0};
    spin.axis = {0, 1, 0};
    Joint copy = joint("copy", Joint::Type::Prismatic, "turret", "finger");
    copy.axis = {1, 0, 0};
    copy.mimic = std::make_tuple(std::string("lift"), 2.0, 0.1);
    document.joints = {pan, lift, spin, copy};

    const Tree tree = buildTree(document);
    ASSERT_TRUE(tree.ok) << tree.error;
    EXPECT_EQ(tree.root, 0);
    EXPECT_EQ(tree.parentJoint, (std::vector<int>{-1, 0, 1, 2, 3}));

    // At zero every link sits at its joint origins.
    const auto rest = linkFrames(document, tree, {});
    ASSERT_EQ(rest.size(), 5U);
    expectPoint(rest[0].position, {0, 0, 0});
    expectPoint(rest[1].position, {0, 0, 0.5});
    expectPoint(rest[2].position, {1, 0, 0.5});
    expectPoint(rest[3].position, {0, -1, 0});
    expectPoint(rest[4].position, {0.1, 0, 0.5});  // the mimic's offset applies at zero

    // pan 90° turns the slider from +x to +y; lift 0.2 m raises it; the mimic finger moves
    // 2 · 0.2 + 0.1 = 0.5 m along the turret's x, which is the root's y now (whatever the
    // preview says for it); spin 90° about y takes the wheel's +x to −z.
    const Preview preview{{"pan", kPi / 2}, {"lift", 0.2}, {"spin", kPi / 2}, {"copy", 9.0}};
    const auto posed = linkFrames(document, tree, preview);
    expectPoint(posed[2].position, {0, 1, 0.7});
    expectPoint(posed[4].position, {0, 0.5, 0.5});
    expectPoint(pointOf(posed, 3, {1, 0, 0}), {0, -1, -1});
    // The slider turned with the turret: its +x is the root's +y.
    expectPoint(pointOf(posed, 2, {1, 0, 0}), {0, 2, 0.7});

    // A fixed joint ignores the preview; a joint origin's rpy is applied before the motion.
    Document fixed = document;
    fixed.joints[0].type = Joint::Type::Fixed;
    fixed.joints[0].origin.rpy = {0, 0, kPi};
    const auto still = linkFrames(fixed, buildTree(fixed), preview);
    expectPoint(still[2].position, {-1, 0, 0.7});
}

TEST(Kinematics, RefusesLoopsAndTwoRoots) {
    Document document;
    document.robot = "r";
    document.links = {link("a"), link("b"), link("c")};
    document.joints = {joint("ab", Joint::Type::Fixed, "a", "b")};
    // Two roots: a and c.
    Tree tree = buildTree(document);
    EXPECT_FALSE(tree.ok);
    EXPECT_EQ(tree.root, -1);
    EXPECT_NE(tree.error.find("'c'"), std::string::npos) << tree.error;
    EXPECT_NE(tree.error.find("one root"), std::string::npos) << tree.error;

    // A loop: b → c → b, and a is the root.
    document.joints = {joint("ab", Joint::Type::Fixed, "a", "b"),
                       joint("bc", Joint::Type::Fixed, "b", "c"),
                       joint("cb", Joint::Type::Fixed, "c", "b")};
    tree = buildTree(document);
    EXPECT_FALSE(tree.ok);
    EXPECT_FALSE(tree.error.empty());

    // A loop through every link: no root at all.
    document.joints = {joint("ab", Joint::Type::Fixed, "a", "b"),
                       joint("bc", Joint::Type::Fixed, "b", "c"),
                       joint("ca", Joint::Type::Fixed, "c", "a")};
    tree = buildTree(document);
    EXPECT_FALSE(tree.ok);
    EXPECT_EQ(tree.root, -1);
    EXPECT_NE(tree.error.find("loop"), std::string::npos) << tree.error;
    // Frames still come back, one per link, at the identity.
    const auto frames = linkFrames(document, tree, {});
    ASSERT_EQ(frames.size(), 3U);
    expectPoint(frames[1].position, {0, 0, 0});

    // A joint naming a missing link.
    document.joints = {joint("ab", Joint::Type::Fixed, "a", "b"),
                       joint("bx", Joint::Type::Fixed, "b", "x")};
    document.links.pop_back();
    tree = buildTree(document);
    EXPECT_FALSE(tree.ok);
    EXPECT_NE(tree.error.find("'x'"), std::string::npos) << tree.error;

    // An empty document has no tree.
    EXPECT_FALSE(buildTree(Document{}).ok);
    // A single link is a tree.
    EXPECT_TRUE(buildTree(newDocument()).ok);
}

TEST(Kinematics, MimicChainsAndSelfMimicStayFinite) {
    Document document;
    document.robot = "r";
    document.links = {link("a"), link("b"), link("c")};
    Joint first = joint("j1", Joint::Type::Revolute, "a", "b");
    first.axis = {0, 0, 1};
    first.origin.xyz = {1, 0, 0};
    Joint second = joint("j2", Joint::Type::Revolute, "b", "c");
    second.axis = {0, 0, 1};
    second.origin.xyz = {1, 0, 0};
    second.mimic = std::make_tuple(std::string("j2"), 1.0, 0.0);  // itself: the preview is used
    document.joints = {first, second};
    const auto frames = linkFrames(document, buildTree(document), {{"j2", kPi / 2}});
    expectPoint(pointOf(frames, 2, {1, 0, 0}), {2, 1, 0});
    // A zero axis does not move.
    document.joints[1].mimic.reset();
    document.joints[1].axis = {0, 0, 0};
    const auto zero = linkFrames(document, buildTree(document), {{"j2", 1.0}});
    expectPoint(pointOf(zero, 2, {1, 0, 0}), {3, 0, 0});
}

}  // namespace
}  // namespace lagrange::viewer::builder
