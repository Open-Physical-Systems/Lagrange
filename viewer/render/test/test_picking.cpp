#include "lagrange/viewer/Picking.hpp"

#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

namespace {

using lagrange::viewer::Camera;
using lagrange::viewer::intersectGround;
using lagrange::viewer::kNone;
using lagrange::viewer::pickPrim;
using lagrange::viewer::Ray;
using lagrange::viewer::SceneFrame;
using lagrange::viewer::SceneStatics;

// Two unit boxes: prim 0 at the origin, prim 1 at x = 5; prim 2 has no mesh.
SceneFrame scene() {
    auto statics = std::make_shared<SceneStatics>();
    statics->meshes.resize(1);
    statics->meshes[0].aabb = {-0.5F, -0.5F, -0.5F, 0.5F, 0.5F, 0.5F};
    statics->prims.resize(3);
    statics->prims[0].mesh = 0;
    statics->prims[1].mesh = 0;
    SceneFrame frame;
    frame.statics = statics;
    frame.transforms.resize(3);
    frame.transforms[1].position = {5.0, 0.0, 0.0};
    return frame;
}

TEST(CameraRay, CentreRayPointsAtTheTarget) {
    Camera camera;
    camera.target = {2.0, -1.0, 0.5};
    const Ray ray = camera.ray(0.0, 0.0, 1.6);
    const auto eye = camera.eye();
    for (std::size_t i = 0; i < 3; ++i) {
        EXPECT_NEAR(eye.at(i) + ray.direction.at(i) * camera.distance, camera.target.at(i), 1e-9);
    }
}

// A ray through a point of the view must project back to that point.
TEST(CameraRay, IsTheInverseOfTheProjection) {
    Camera camera;
    camera.yaw = 0.7;
    camera.pitch = 0.4;
    const double aspect = 1.6;
    const lagrange::viewer::Mat4 m = camera.viewProjection(aspect);
    for (const auto& [x, y] : {std::pair{-0.8, 0.3}, std::pair{0.5, -0.9}, std::pair{0.0, 0.0}}) {
        const Ray ray = camera.ray(x, y, aspect);
        const auto& d = ray.direction;
        const double cx = m.at(0) * d[0] + m.at(1) * d[1] + m.at(2) * d[2];
        const double cy = m.at(4) * d[0] + m.at(5) * d[1] + m.at(6) * d[2];
        const double cw = m.at(12) * d[0] + m.at(13) * d[1] + m.at(14) * d[2];
        EXPECT_NEAR(cx / cw, x, 1e-5);
        EXPECT_NEAR(cy / cw, y, 1e-5);
    }
}

TEST(CameraRay, AxesAreOrthonormal) {
    Camera camera;
    camera.yaw = 1.1;
    camera.pitch = -0.3;
    const auto axes = camera.axes();
    for (std::size_t a = 0; a < 3; ++a) {
        for (std::size_t b = 0; b < 3; ++b) {
            const double dot = axes.at(a)[0] * axes.at(b)[0] + axes.at(a)[1] * axes.at(b)[1] +
                               axes.at(a)[2] * axes.at(b)[2];
            EXPECT_NEAR(dot, a == b ? 1.0 : 0.0, 1e-12);
        }
    }
}

TEST(Picking, TheNearestBoxAlongTheRayWins) {
    const SceneFrame frame = scene();
    EXPECT_EQ(pickPrim(frame, {{-10.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}), 0U);
    EXPECT_EQ(pickPrim(frame, {{10.0, 0.0, 0.0}, {-1.0, 0.0, 0.0}}), 1U);
    EXPECT_EQ(pickPrim(frame, {{5.0, 0.2, 10.0}, {0.0, 0.0, -1.0}}), 1U);
}

TEST(Picking, AMissSelectsNothing) {
    const SceneFrame frame = scene();
    EXPECT_EQ(pickPrim(frame, {{-10.0, 2.0, 0.0}, {1.0, 0.0, 0.0}}), kNone);
    EXPECT_EQ(pickPrim(frame, {{-10.0, 0.0, 0.0}, {-1.0, 0.0, 0.0}}), kNone);  // behind the ray
    EXPECT_EQ(pickPrim(SceneFrame{}, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}), kNone);
}

TEST(Picking, FollowsThePrimsRotationAndScale) {
    SceneFrame frame = scene();
    // Prim 0 becomes a 4 x 1 x 1 bar turned 90 degrees about Z: long along Y.
    frame.transforms[0].scale = {4.0F, 1.0F, 1.0F};
    const float half = std::sqrt(0.5F);
    frame.transforms[0].rotation = {half, 0.0F, 0.0F, half};
    EXPECT_EQ(pickPrim(frame, {{0.0, 1.8, 10.0}, {0.0, 0.0, -1.0}}), 0U);     // on the bar
    EXPECT_EQ(pickPrim(frame, {{1.8, 0.0, 10.0}, {0.0, 0.0, -1.0}}), kNone);  // beside it
}

TEST(Picking, HiddenPrimsAreNotPicked) {
    SceneFrame frame = scene();
    auto statics = std::make_shared<SceneStatics>(*frame.statics);
    statics->prims[0].visible = false;
    frame.statics = statics;
    EXPECT_EQ(pickPrim(frame, {{-10.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}), 1U);
}

TEST(Picking, GroundIntersection) {
    const auto hit = intersectGround({{1.0, 2.0, 4.0}, {0.0, 0.6, -0.8}});
    ASSERT_TRUE(hit.has_value());
    EXPECT_NEAR((*hit)[0], 1.0, 1e-12);
    EXPECT_NEAR((*hit)[1], 5.0, 1e-12);
    EXPECT_NEAR((*hit)[2], 0.0, 1e-12);
    EXPECT_FALSE(intersectGround({{0.0, 0.0, 4.0}, {0.0, 0.6, 0.8}}).has_value());  // looking up
    EXPECT_FALSE(intersectGround({{0.0, 0.0, 4.0}, {1.0, 0.0, 0.0}}).has_value());  // level
}

}  // namespace
