// The Builder view's scene (RFC 0016 WP-D): primitives, the document as a SceneFrame, the joint
// preview and picking.
#include "lagrange/viewer/Picking.hpp"
#include "lagrange/viewer/builder/Document.hpp"
#include "lagrange/viewer/builder/Primitives.hpp"
#include "lagrange/viewer/builder/SceneAdapter.hpp"

#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

namespace lagrange::viewer::builder {
namespace {

Shape box(std::array<double, 3> size, std::array<double, 3> at = {}) {
    Shape s;
    s.kind = Shape::Kind::Box;
    s.size = size;
    s.origin.xyz = at;
    return s;
}

/// A part of rover_mk1 (viewer/test/builder/rover_mk1.urdf), built in code: base_link, a
/// wheel on a continuous joint, the pan and tilt head, the rangefinder; plus a mimic joint.
Document rover() {
    Document d;
    d.robot = "rover_mk1";
    d.links.push_back({"base_link", {box({0.5, 0.36, 0.12})}, {}, {}});
    Shape wheel;
    wheel.kind = Shape::Kind::Cylinder;
    wheel.size = {0.08, 0.04, 0};
    wheel.origin.rpy = {std::numbers::pi / 2, 0, 0};
    d.links.push_back({"left_wheel", {wheel}, {}, {}});
    d.links.push_back({"pan_link", {box({0.06, 0.06, 0.04}, {0, 0, 0.02})}, {}, {}});
    d.links.push_back({"tilt_link", {box({0.04, 0.08, 0.04})}, {}, {}});
    Shape visualOnly = box({0.01, 0.01, 0.01});
    visualOnly.collision = false;
    d.links.back().shapes.push_back(visualOnly);
    Joint j;
    j.type = Joint::Type::Continuous;
    j.name = "left_wheel_joint";
    j.parent = "base_link";
    j.child = "left_wheel";
    j.origin.xyz = {0.1, 0.2, -0.05};
    j.axis = {0, 1, 0};
    d.joints.push_back(j);
    j = {};
    j.type = Joint::Type::Revolute;
    j.name = "pan_joint";
    j.parent = "base_link";
    j.child = "pan_link";
    j.origin.xyz = {0.15, 0, 0.06};
    j.axis = {0, 0, 1};
    j.lower = -2.9;
    j.upper = 2.9;
    d.joints.push_back(j);
    j = {};
    j.type = Joint::Type::Revolute;
    j.name = "tilt_joint";
    j.parent = "pan_link";
    j.child = "tilt_link";
    j.origin.xyz = {0, 0, 0.06};
    j.axis = {0, 1, 0};
    j.mimic = std::make_tuple(std::string("pan_joint"), 0.5, 0.0);
    d.joints.push_back(j);
    Sensor range;
    range.name = "front_range";
    range.link = "base_link";
    range.type = "rangefinder";
    range.origin.xyz = {0.25, 0, -0.03};
    d.sensors.push_back(range);
    return d;
}

TEST(Primitives, SameVertexCountsAsCore) {
    // core/world/src/Meshes.cpp: a box has 6 faces of 2 flat triangles (a vertex per corner of
    // each); a cylinder 2 per segment on the wall plus a centre and a ring per cap; a sphere
    // (segments + 1) × (rings + 1) vertices.
    const MeshData b = boxMesh({0.3, 0.2, 0.1});
    EXPECT_EQ(b.positions.size() / 3, 36U);
    EXPECT_EQ(b.indices.size(), 36U);
    const MeshData c = cylinderMesh(0.1, 0.2);
    EXPECT_EQ(c.positions.size() / 3, 2U * 32U + 2U * 33U);
    EXPECT_EQ(c.indices.size(), 32U * 6U + 2U * 32U * 3U);
    const MeshData s = sphereMesh(0.1);
    EXPECT_EQ(s.positions.size() / 3, 33U * 17U);
    EXPECT_EQ(s.indices.size(), 32U * 16U * 6U);
    for (const MeshData* mesh : {&b, &c, &s}) {
        EXPECT_EQ(mesh->normals.size(), mesh->positions.size());
    }
    EXPECT_FLOAT_EQ(b.aabb[0], -0.15F);
    EXPECT_FLOAT_EQ(b.aabb[4], 0.1F);
    EXPECT_FLOAT_EQ(c.aabb[5], 0.1F);
    EXPECT_FLOAT_EQ(s.aabb[3], 0.1F);
}

TEST(SceneAdapter, OneMeshPerShapeAndCollidersSeparate) {
    const BuiltScene empty = buildStatics(newDocument("robot"), 1);
    ASSERT_TRUE(empty.statics);
    EXPECT_EQ(empty.statics->generation, 1U);
    ASSERT_EQ(empty.statics->prims.size(), 2U);  // base_link and its box
    EXPECT_EQ(empty.statics->meshes.size(), 1U);
    ASSERT_EQ(empty.statics->colliders.size(), 1U);
    EXPECT_EQ(empty.statics->colliders[0].prim, 0U);
    EXPECT_EQ(empty.statics->prims[1].parent, 0U);
    EXPECT_EQ(empty.primItem[1], "base_link");

    const BuiltScene scene = buildStatics(rover(), 7);
    const SceneStatics& s = *scene.statics;
    // 4 link prims, 5 visual shapes, 1 sensor marker; 4 colliders (the visual-only shape has
    // none); 5 shape meshes and one marker mesh.
    EXPECT_EQ(s.prims.size(), 10U);
    EXPECT_EQ(s.colliders.size(), 4U);
    EXPECT_EQ(s.meshes.size(), 6U);
    EXPECT_EQ(scene.primItem.size(), s.prims.size());
    EXPECT_EQ(scene.primLink.size(), s.prims.size());
    EXPECT_EQ(scene.primOffset.size(), s.prims.size());
    const std::uint32_t sensor = primOf(scene, "front_range");
    ASSERT_NE(sensor, kNone);
    EXPECT_EQ(scene.primSensor[sensor], 0);
    EXPECT_EQ(s.prims[sensor].parent, primOf(scene, "base_link"));
    // A shape without rgba gets the default colour.
    EXPECT_EQ(s.materials[s.prims[1].material].baseColor, kDefaultShapeColor);
}

TEST(SceneAdapter, PreviewChangesTransformsOnly) {
    const Document document = rover();
    const BuiltScene scene = buildStatics(document, 3);
    const auto still = poseScene(scene, previewFrames(document, {}));
    const auto turned =
        poseScene(scene, previewFrames(document, {{"pan_joint", std::numbers::pi / 2}}));
    EXPECT_EQ(still->statics.get(), turned->statics.get());
    EXPECT_EQ(turned->statics->generation, 3U);
    const std::uint32_t tilt = primOf(scene, "tilt_link");
    const std::uint32_t pan = primOf(scene, "pan_link");
    // pan_link's origin does not move as it turns about its own z; its box turns.
    EXPECT_NEAR(still->transforms[pan].position[0], 0.15, 1e-12);
    EXPECT_NEAR(turned->transforms[pan].position[0], 0.15, 1e-12);
    EXPECT_NEAR(turned->transforms[pan].rotation[0], std::cos(std::numbers::pi / 4), 1e-6);
    // tilt_link sits 0.06 above pan_link at 0.12, and turns by the mimic: half of pan.
    EXPECT_NEAR(turned->transforms[tilt].position[2], 0.12, 1e-12);
    const auto& q = turned->transforms[tilt].rotation;
    // pan 90° about z then tilt 45° about y: w = cos(45°) cos(22.5°).
    EXPECT_NEAR(q[0], std::cos(std::numbers::pi / 4) * std::cos(std::numbers::pi / 8), 1e-6);
    // The wheel's visual sits at its joint, turned by its own rpy.
    const std::uint32_t wheelShape = primOf(scene, "left_wheel") + 1;
    EXPECT_NEAR(still->transforms[wheelShape].position[1], 0.2, 1e-12);
    EXPECT_NEAR(still->transforms[wheelShape].rotation[1], std::sin(std::numbers::pi / 4), 1e-6);
}

TEST(SceneAdapter, PickReturnsTheLink) {
    const Document document = rover();
    const BuiltScene scene = buildStatics(document, 1);
    const auto frame = poseScene(scene, previewFrames(document, {}));
    // Straight down onto base_link's top, away from the head.
    const Ray down{{-0.15, 0.1, 2.0}, {0.0, 0.0, -1.0}};
    const std::uint32_t hit = pickPrim(*frame, down);
    ASSERT_NE(hit, kNone);
    EXPECT_EQ(scene.primItem[hit], "base_link");
    EXPECT_LT(scene.primSensor[hit], 0);
    // Onto the pan head, which stands above base_link.
    const Ray head{{0.15, 0.0, 2.0}, {0.0, 0.0, -1.0}};
    const std::uint32_t top = pickPrim(*frame, head);
    ASSERT_NE(top, kNone);
    EXPECT_EQ(scene.primItem[top], "tilt_link");
    // Along the rangefinder's beam backwards: its marker sticks out of the box's front face.
    const Ray back{{1.0, 0.0, -0.03}, {-1.0, 0.0, 0.0}};
    const std::uint32_t sensor = pickPrim(*frame, back);
    ASSERT_NE(sensor, kNone);
    EXPECT_EQ(scene.primItem[sensor], "front_range");
    EXPECT_EQ(scene.primSensor[sensor], 0);
    // Nothing in the sky.
    EXPECT_EQ(pickPrim(*frame, Ray{{0.0, 0.0, 2.0}, {0.0, 0.0, 1.0}}), kNone);
}

TEST(SceneAdapter, PreviewFramesLeaveUnreachedLinksAtTheIdentity) {
    Document document = rover();
    document.links.push_back({"floating", {box({0.1, 0.1, 0.1})}, {}, {}});
    const auto frames = previewFrames(document, {});
    ASSERT_EQ(frames.size(), document.links.size());
    EXPECT_EQ(frames.back().position, (std::array<double, 3>{0, 0, 0}));
    EXPECT_NEAR(frames[1].position[2], -0.05, 1e-12);  // left_wheel at its joint
}

}  // namespace
}  // namespace lagrange::viewer::builder
