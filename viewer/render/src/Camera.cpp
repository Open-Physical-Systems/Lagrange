#include "lagrange/viewer/Camera.hpp"

#include <algorithm>
#include <cmath>

namespace lagrange::viewer {

namespace {

constexpr double kMaxPitch = 1.55;  // just under 90 degrees
constexpr double kMinDistance = 0.05;
constexpr double kMaxDistance = 1.0e7;

struct Basis {
    std::array<double, 3> forward, right, up;
};

Basis basis(double yaw, double pitch) {
    const double cp = std::cos(pitch);
    const std::array<double, 3> forward{-cp * std::cos(yaw), -cp * std::sin(yaw), -std::sin(pitch)};
    const std::array<double, 3> right{-std::sin(yaw), std::cos(yaw), 0.0};  // forward x Z, unit
    const std::array<double, 3> up{
        right[1] * forward[2] - right[2] * forward[1],
        right[2] * forward[0] - right[0] * forward[2],
        right[0] * forward[1] - right[1] * forward[0],
    };
    return {forward, right, up};
}

}  // namespace

void Camera::orbit(double deltaYaw, double deltaPitch) {
    yaw += deltaYaw;
    pitch = std::clamp(pitch + deltaPitch, -kMaxPitch, kMaxPitch);
}

void Camera::zoom(double factor) {
    distance = std::clamp(distance * factor, kMinDistance, kMaxDistance);
}

void Camera::pan(double right, double up) {
    const Basis b = basis(yaw, pitch);
    const double height = 2.0 * distance * std::tan(verticalFov / 2.0);
    for (std::size_t i = 0; i < 3; ++i) {
        target.at(i) += (b.right.at(i) * right + b.up.at(i) * up) * height;
    }
}

std::array<double, 3> Camera::eye() const {
    const Basis b = basis(yaw, pitch);
    return {target[0] - b.forward[0] * distance, target[1] - b.forward[1] * distance,
            target[2] - b.forward[2] * distance};
}

Mat4 Camera::viewProjection(double aspect) const {
    const Basis b = basis(yaw, pitch);
    const double f = 1.0 / std::tan(verticalFov / 2.0);
    // View space: x right, y down (Vulkan), z forward. Rows of the rotation are the basis.
    // Projection: x' = f/aspect * x, y' = f * y, z' = near (reversed, infinite far), w' = z.
    const double sx = f / aspect;
    Mat4 m{};
    for (std::size_t c = 0; c < 3; ++c) {
        m.at(0 + c) = static_cast<float>(sx * b.right.at(c));
        m.at(4 + c) = static_cast<float>(-f * b.up.at(c));
        m.at(8 + c) = 0.0F;
        m.at(12 + c) = static_cast<float>(b.forward.at(c));
    }
    m.at(11) = static_cast<float>(nearPlane);  // clip z = near; after divide: near / distance
    return m;
}

Ray Camera::ray(double ndcX, double ndcY, double aspect) const {
    const Basis b = basis(yaw, pitch);
    // The inverse of viewProjection(): x' = (f / aspect) x / z and y' = -f y / z.
    const double f = 1.0 / std::tan(verticalFov / 2.0);
    Ray result;
    result.origin = eye();
    double length = 0.0;
    for (std::size_t i = 0; i < 3; ++i) {
        result.direction.at(i) =
            b.forward.at(i) + b.right.at(i) * (ndcX * aspect / f) - b.up.at(i) * (ndcY / f);
        length += result.direction.at(i) * result.direction.at(i);
    }
    length = std::sqrt(length);
    for (double& component : result.direction) {
        component /= length;
    }
    return result;
}

std::array<std::array<double, 3>, 3> Camera::axes() const {
    const Basis b = basis(yaw, pitch);
    return {b.right, b.up, b.forward};
}

}  // namespace lagrange::viewer
