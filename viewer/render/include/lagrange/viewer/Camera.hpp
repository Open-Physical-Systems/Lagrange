#pragma once

#include <array>

namespace lagrange::viewer {

/// Row-major 4x4 matrix: m[row * 4 + column].
using Mat4 = std::array<float, 16>;

/// Orbit camera. Its target and eye are double (world scale); everything handed to the GPU is
/// relative to the eye, so precision does not depend on distance from the origin.
class Camera {
public:
    std::array<double, 3> target{0.0, 0.0, 0.0};
    double distance{6.0};     // m
    double yaw{-2.2};         // rad, about +Z
    double pitch{0.55};       // rad, above the horizon
    double verticalFov{0.9};  // rad
    double nearPlane{0.05};   // m

    void orbit(double deltaYaw, double deltaPitch);
    void zoom(double factor);
    /// Moves the target in the view plane by a fraction of the view height.
    void pan(double right, double up);

    [[nodiscard]] std::array<double, 3> eye() const;

    /// Clip-from-camera-relative-world: rotation and projection only, no translation.
    /// Reversed depth (near = 1, far = 0 at infinity) and Vulkan clip space (y down).
    [[nodiscard]] Mat4 viewProjection(double aspect) const;
};

}  // namespace lagrange::viewer
