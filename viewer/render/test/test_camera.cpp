#include "lagrange/viewer/Camera.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace {

using lagrange::viewer::Camera;
using lagrange::viewer::Mat4;

struct Clip {
    double x, y, z, w;
};

// Applies the matrix to a world point given relative to the camera eye.
Clip project(const Camera& camera, const std::array<double, 3>& world, double aspect = 1.0) {
    const Mat4 m = camera.viewProjection(aspect);
    const std::array<double, 3> eye = camera.eye();
    const std::array<double, 4> p{world[0] - eye[0], world[1] - eye[1], world[2] - eye[2], 1.0};
    const auto row = [&](std::size_t r) {
        return m.at(r * 4) * p[0] + m.at(r * 4 + 1) * p[1] + m.at(r * 4 + 2) * p[2] +
               m.at(r * 4 + 3) * p[3];
    };
    return {row(0), row(1), row(2), row(3)};
}

TEST(Camera, EyeIsAtTheOrbitDistanceFromTheTarget) {
    Camera camera;
    camera.target = {10.0, -4.0, 1.0};
    camera.distance = 7.5;
    const auto eye = camera.eye();
    const double d = std::hypot(eye[0] - 10.0, eye[1] + 4.0, eye[2] - 1.0);
    EXPECT_NEAR(d, 7.5, 1e-9);
    EXPECT_GT(eye[2], 1.0);  // positive pitch looks down from above
}

TEST(Camera, TargetProjectsToTheCentreWithReversedDepth) {
    Camera camera;
    camera.target = {3.0, 2.0, 0.5};
    const Clip c = project(camera, camera.target);
    EXPECT_NEAR(c.x / c.w, 0.0, 1e-6);
    EXPECT_NEAR(c.y / c.w, 0.0, 1e-6);
    EXPECT_NEAR(c.w, camera.distance, 1e-5);                           // w is the view depth
    EXPECT_NEAR(c.z / c.w, camera.nearPlane / camera.distance, 1e-7);  // near = 1, far -> 0
}

TEST(Camera, UpIsUpOnScreenAndNearerMeansLargerDepth) {
    Camera camera;
    camera.pitch = 0.0;  // looking horizontally
    const Clip above =
        project(camera, {camera.target[0], camera.target[1], camera.target[2] + 1.0});
    EXPECT_LT(above.y / above.w, 0.0);  // Vulkan clip space: -y is the top of the image

    const auto eye = camera.eye();
    const std::array<double, 3> nearer{(camera.target[0] + eye[0]) / 2,
                                       (camera.target[1] + eye[1]) / 2,
                                       (camera.target[2] + eye[2]) / 2};
    const Clip farClip = project(camera, camera.target);
    const Clip nearClip = project(camera, nearer);
    EXPECT_GT(nearClip.z / nearClip.w, farClip.z / farClip.w);
}

TEST(Camera, ProjectionIsTheSameFarFromTheOrigin) {
    // Camera-relative rendering: moving everything 500 km must not change the result.
    Camera here;
    here.target = {1.0, 2.0, 0.0};
    Camera there = here;
    there.target = {500'001.0, 500'002.0, 0.0};
    const Clip a = project(here, {2.0, 2.5, 0.3}, 1.6);
    const Clip b = project(there, {500'002.0, 500'002.5, 0.3}, 1.6);
    EXPECT_NEAR(a.x / a.w, b.x / b.w, 1e-6);
    EXPECT_NEAR(a.y / a.w, b.y / b.w, 1e-6);
    EXPECT_NEAR(a.z / a.w, b.z / b.w, 1e-7);
}

TEST(Camera, PitchAndZoomAreClamped) {
    Camera camera;
    camera.orbit(0.0, 10.0);
    EXPECT_LT(camera.pitch, 1.5708);
    camera.orbit(0.0, -20.0);
    EXPECT_GT(camera.pitch, -1.5708);
    for (int i = 0; i < 400; ++i) {
        camera.zoom(0.5);
    }
    EXPECT_GT(camera.distance, 0.0);
    camera.pan(0.5, 0.0);
    EXPECT_TRUE(std::isfinite(camera.target[0]));
}

}  // namespace
