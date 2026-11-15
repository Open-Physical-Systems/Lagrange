// The Builder view's gizmo geometry (RFC 0016; Gizmo.hpp): the ray tests that pick handles and
// shapes, snapping, and what a drag does to the document.

#include "lagrange/viewer/builder/Document.hpp"
#include "lagrange/viewer/builder/Gizmo.hpp"
#include "lagrange/viewer/builder/Kinematics.hpp"

#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

namespace lagrange::viewer::builder::gizmo {
namespace {

constexpr double kTight = 1e-12;
constexpr double kDeg = std::numbers::pi / 180.0;

Line lineFrom(Vec3 origin, Vec3 direction) {
    const double n = std::sqrt(direction[0] * direction[0] + direction[1] * direction[1] +
                               direction[2] * direction[2]);
    return Line{origin, {direction[0] / n, direction[1] / n, direction[2] / n}};
}

TEST(Gizmo, SnapRoundsToTheStepAndOffKeepsTheValue) {
    EXPECT_DOUBLE_EQ(snap(0.0124, kMoveStep), 0.010);
    EXPECT_DOUBLE_EQ(snap(0.0126, kMoveStep), 0.015);
    EXPECT_DOUBLE_EQ(snap(-0.0126, kMoveStep), -0.015);
    EXPECT_EQ(std::signbit(snap(-0.001, kMoveStep)), false);  // no minus zero in a file
    EXPECT_DOUBLE_EQ(snap(0.0124, 0.0), 0.0124);
    EXPECT_NEAR(snap(7.4 * kDeg, kAngleStep), 5.0 * kDeg, kTight);
}

TEST(Gizmo, AlongAxisFindsWhereTheMouseRayPassesTheArrow) {
    // An x arrow at the origin, seen from above: a ray straight down at x = 0.3 is at t = 0.3.
    const auto t = alongAxis(lineFrom({0.3, 0.0, 5.0}, {0, 0, -1}), {0, 0, 0}, {1, 0, 0});
    ASSERT_TRUE(t.has_value());
    EXPECT_NEAR(*t, 0.3, kTight);
    // A slanted ray: its closest approach to the axis line is still at x = 0.3.
    const auto slanted =
        alongAxis(lineFrom({0.3, -2.0, 2.0}, {0.0, 1.0, -1.0}), {0, 0, 0}, {1, 0, 0});
    ASSERT_TRUE(slanted.has_value());
    EXPECT_NEAR(*slanted, 0.3, 1e-9);
    // Looking straight down the arrow there is no answer.
    EXPECT_FALSE(alongAxis(lineFrom({-5, 0, 0}, {1, 0, 0}), {0, 0, 0}, {1, 0, 0}).has_value());
}

TEST(Gizmo, OnPlaneAndAngleOnRing) {
    const auto hit = onPlane(lineFrom({0.2, 0.1, 3.0}, {0, 0, -1}), {0, 0, 0}, {0, 0, 1});
    ASSERT_TRUE(hit.has_value());
    EXPECT_NEAR((*hit)[0], 0.2, kTight);
    EXPECT_NEAR((*hit)[2], 0.0, kTight);
    EXPECT_FALSE(onPlane(lineFrom({0, 0, 3}, {0, 0, 1}), {0, 0, 0}, {0, 0, 1}).has_value());
    EXPECT_FALSE(onPlane(lineFrom({0, 0, 3}, {1, 0, 0}), {0, 0, 0}, {0, 0, 1}).has_value());
    // A z ring: from +x a point at +y is a quarter turn, right hand about z.
    const auto angle =
        angleOnRing(lineFrom({0.0, 0.5, 3.0}, {0, 0, -1}), {0, 0, 0}, {0, 0, 1}, {1, 0, 0});
    ASSERT_TRUE(angle.has_value());
    EXPECT_NEAR(*angle, std::numbers::pi / 2, kTight);
}

TEST(Gizmo, RpyRoundTripsThroughToFrame) {
    for (const Vec3 rpy : {Vec3{0.1, -0.4, 2.0}, Vec3{0, 0, 0}, Vec3{-1.2, 0.3, -3.0}}) {
        Pose pose;
        pose.rpy = rpy;
        const Vec3 back = rpyOf(toFrame(pose).rotation);
        for (std::size_t k = 0; k < 3; ++k) {
            EXPECT_NEAR(back.at(k), rpy.at(k), 1e-12);
        }
    }
}

TEST(Gizmo, MoveSnapsTheMovedCoordinateOnly) {
    Pose start;
    start.xyz = {0.1, 0.02, 0.05};
    start.rpy = {0, 0, 0.3};
    const Pose out = moved(start, 0, 0.0337, kMoveStep);
    EXPECT_DOUBLE_EQ(out.xyz[0], 0.135);
    EXPECT_DOUBLE_EQ(out.xyz[1], 0.02);
    EXPECT_DOUBLE_EQ(out.xyz[2], 0.05);
    EXPECT_DOUBLE_EQ(out.rpy[2], 0.3);
    EXPECT_DOUBLE_EQ(moved(start, 2, -0.0337, 0.0).xyz[2], 0.05 - 0.0337);  // Ctrl: no snap
}

TEST(Gizmo, RotateTurnsAboutTheParentAxisBySnappedSteps) {
    Pose start;
    start.xyz = {1, 2, 3};
    start.rpy = {0, 0, 10 * kDeg};
    const Pose out = rotated(start, 2, 33 * kDeg, kAngleStep);  // snaps to 35°
    EXPECT_NEAR(out.rpy[2], 45 * kDeg, kTight);
    EXPECT_EQ(out.rpy[0], 0.0);
    EXPECT_EQ(out.xyz, start.xyz);
    // About x from zero: roll only, exactly on the step.
    const Pose roll = rotated(Pose{}, 0, -91 * kDeg, kAngleStep);
    EXPECT_EQ(roll.rpy[0], snap(-90 * kDeg, kAngleStep));
    EXPECT_EQ(roll.rpy[1], 0.0);
    // Less than half a step does nothing.
    EXPECT_EQ(rotated(start, 1, 2 * kDeg, kAngleStep).rpy, start.rpy);
}

TEST(Gizmo, ResizeKeepsTheOppositeFace) {
    Shape box;
    box.kind = Shape::Kind::Box;
    box.size = {0.3, 0.2, 0.1};
    ASSERT_TRUE(resize(box, 0, +1, 0.0437, kMoveStep));
    EXPECT_DOUBLE_EQ(box.size[0], 0.345);
    EXPECT_NEAR(box.origin.xyz[0], 0.0225, kTight);  // the -x face stays at -0.15
    EXPECT_NEAR(box.origin.xyz[0] - box.size[0] / 2, -0.15, kTight);
    // The -y face pulled in: the +y face stays at 0.1.
    ASSERT_TRUE(resize(box, 1, -1, -0.05, kMoveStep));
    EXPECT_DOUBLE_EQ(box.size[1], 0.15);
    EXPECT_NEAR(box.origin.xyz[1] + box.size[1] / 2, 0.1, kTight);
    // Never below the smallest size.
    ASSERT_TRUE(resize(box, 2, +1, -1.0, kMoveStep));
    EXPECT_DOUBLE_EQ(box.size[2], kMinSize);
}

TEST(Gizmo, ResizeATurnedShapeMovesItsOriginInTheLinkFrame) {
    Shape box;
    box.kind = Shape::Kind::Box;
    box.size = {0.2, 0.2, 0.2};
    box.origin.rpy = {0, 0, std::numbers::pi / 2};  // its x is the link's y
    ASSERT_TRUE(resize(box, 0, +1, 0.1, kMoveStep));
    EXPECT_NEAR(box.origin.xyz[0], 0.0, 1e-12);
    EXPECT_NEAR(box.origin.xyz[1], 0.05, 1e-12);
}

TEST(Gizmo, ResizeCylinderAndSphere) {
    Shape cylinder;
    cylinder.kind = Shape::Kind::Cylinder;
    cylinder.size = {0.05, 0.1, 0};
    ASSERT_TRUE(resize(cylinder, 0, +1, 0.02, kMoveStep));  // diameter 0.1 → 0.12
    EXPECT_DOUBLE_EQ(cylinder.size[0], 0.06);
    EXPECT_NEAR(cylinder.origin.xyz[0], 0.01, kTight);
    ASSERT_TRUE(resize(cylinder, 2, -1, 0.05, kMoveStep));  // length 0.1 → 0.15, down
    EXPECT_DOUBLE_EQ(cylinder.size[1], 0.15);
    EXPECT_NEAR(cylinder.origin.xyz[2], -0.025, kTight);
    Shape sphere;
    sphere.kind = Shape::Kind::Sphere;
    sphere.size = {0.05, 0, 0};
    ASSERT_TRUE(resize(sphere, 2, +1, 0.1, kMoveStep));
    EXPECT_DOUBLE_EQ(sphere.size[0], 0.1);
    EXPECT_NEAR(sphere.origin.xyz[2], 0.05, kTight);
    Shape mesh;
    mesh.kind = Shape::Kind::Mesh;
    EXPECT_FALSE(resize(mesh, 0, 1, 0.1, kMoveStep));
    EXPECT_FALSE(extentOf(mesh, 0).has_value());
}

TEST(Gizmo, JointAxisSnapsToPrincipalAxesUnlessFree) {
    const Quat identity{1, 0, 0, 0};
    EXPECT_EQ(snapAxis({0.2, 0.1, 0.9}, identity, false), (Vec3{0, 0, 1}));
    EXPECT_EQ(snapAxis({-0.9, 0.3, 0.1}, identity, false), (Vec3{-1, 0, 0}));
    const Vec3 free = snapAxis({3, 4, 0}, identity, true);
    EXPECT_NEAR(free[0], 0.6, kTight);
    EXPECT_NEAR(free[1], 0.8, kTight);
    EXPECT_EQ(snapAxis({0, 0, 0}, identity, false), (Vec3{1, 0, 0}));
    // A joint origin turned 30° about z: the parent's x, seen from the joint frame, is a
    // candidate too, at -30°.
    Pose origin;
    origin.rpy = {0, 0, 30 * kDeg};
    const Vec3 parentX =
        snapAxis({std::cos(-28 * kDeg), std::sin(-28 * kDeg), 0}, toFrame(origin).rotation, false);
    EXPECT_NEAR(parentX[0], std::cos(-30 * kDeg), 1e-12);
    EXPECT_NEAR(parentX[1], std::sin(-30 * kDeg), 1e-12);
}

TEST(Gizmo, PickShapeFindsTheNearestShapeAndThePoint) {
    Document document = newDocument("r");  // base_link: box 0.3 × 0.2 × 0.1 at the origin
    Shape sphere;
    sphere.kind = Shape::Kind::Sphere;
    sphere.size = {0.05, 0, 0};
    sphere.origin.xyz = {0.5, 0, 0};
    sphere.visual = false;  // collision only: still picked
    document.links[0].shapes.push_back(sphere);
    const std::vector<Frame> links{Frame{}};
    const auto top = pickShape(document, links, lineFrom({0.1, 0.05, 2}, {0, 0, -1}));
    ASSERT_TRUE(top.has_value());
    EXPECT_EQ(top->shape, 0);
    EXPECT_NEAR(top->point[2], 0.05, kTight);
    const auto ball = pickShape(document, links, lineFrom({0.5, 0, 2}, {0, 0, -1}));
    ASSERT_TRUE(ball.has_value());
    EXPECT_EQ(ball->shape, 1);
    EXPECT_NEAR(ball->point[2], 0.05, kTight);
    // Collision-only shapes are always drawn, translucent (RFC 0016 Addendum A), so the sphere
    // is picked with the collision toggle off too.
    EXPECT_TRUE(pickShape(document, links, lineFrom({0.5, 0, 2}, {0, 0, -1}), false));
    EXPECT_FALSE(pickShape(document, links, lineFrom({2, 2, 2}, {0, 0, -1})).has_value());
    // A link placed by its frame: the same box 1 m up is hit 1 m higher.
    const std::vector<Frame> raised{Frame{{0, 0, 1}}};
    const auto high = pickShape(document, raised, lineFrom({0, 0, 3}, {0, 0, -1}));
    ASSERT_TRUE(high.has_value());
    EXPECT_NEAR(high->point[2], 1.05, kTight);
}

TEST(Gizmo, PickCylinderSideAndCap) {
    Document document = newDocument("r");
    Shape& shape = document.links[0].shapes[0];
    shape.kind = Shape::Kind::Cylinder;
    shape.size = {0.1, 0.4, 0};
    const std::vector<Frame> links{Frame{}};
    const auto side = pickShape(document, links, lineFrom({2, 0, 0.1}, {-1, 0, 0}));
    ASSERT_TRUE(side.has_value());
    EXPECT_NEAR(side->point[0], 0.1, kTight);
    const auto cap = pickShape(document, links, lineFrom({0.05, 0, 2}, {0, 0, -1}));
    ASSERT_TRUE(cap.has_value());
    EXPECT_NEAR(cap->point[2], 0.2, kTight);
    EXPECT_FALSE(pickShape(document, links, lineFrom({2, 0, 0.3}, {-1, 0, 0})).has_value());
}

TEST(Gizmo, TargetsFindTheirPoseAndParentFrame) {
    Document document = newDocument("r");
    Link arm;
    arm.name = "arm";
    document.links.push_back(arm);
    Joint joint;
    joint.name = "arm_joint";
    joint.parent = "base_link";
    joint.child = "arm";
    joint.origin.xyz = {0, 0, 0.1};
    document.joints.push_back(joint);
    Sensor sensor;
    sensor.name = "imu";
    sensor.link = "arm";
    sensor.type = "imu";
    document.sensors.push_back(sensor);
    const std::vector<Frame> links{Frame{}, Frame{{0, 0, 0.1}}};

    const Target joints{Target::Kind::JointOrigin, "arm_joint"};
    ASSERT_NE(poseOf(document, joints), nullptr);
    EXPECT_EQ(poseOf(document, joints)->xyz[2], 0.1);
    EXPECT_EQ(parentFrame(document, links, joints)->position[2], 0.0);  // base_link's frame
    const Target imu{Target::Kind::Sensor, "imu"};
    EXPECT_EQ(parentFrame(document, links, imu)->position[2], 0.1);  // arm's frame
    const Target shape{Target::Kind::Shape, "base_link", 0};
    EXPECT_NE(poseOf(document, shape), nullptr);
    EXPECT_EQ(poseOf(document, Target{Target::Kind::Shape, "base_link", 3}), nullptr);
    EXPECT_EQ(poseOf(document, Target{}), nullptr);
}

TEST(Gizmo, DistanceToSegment) {
    EXPECT_FLOAT_EQ(distanceToSegment({5, 3}, {0, 0}, {10, 0}), 3.0F);
    EXPECT_FLOAT_EQ(distanceToSegment({-4, 3}, {0, 0}, {10, 0}), 5.0F);
    EXPECT_FLOAT_EQ(distanceToSegment({1, 1}, {2, 2}, {2, 2}), std::sqrt(2.0F));
}

TEST(Gizmo, GeometryJsonNamesEveryItem) {
    const std::string json = geometryJson(newDocument("r"));
    EXPECT_NE(json.find("\"base_link\": [{\"kind\": \"box\", \"size\": [0.3, 0.2, 0.1]"),
              std::string::npos)
        << json;
}

}  // namespace
}  // namespace lagrange::viewer::builder::gizmo
