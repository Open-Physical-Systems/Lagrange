#pragma once

#include <array>

namespace lagrange::viewer {

/// Row-major 4x4 matrix: m[row * 4 + column].
using Mat4 = std::array<float, 16>;

/// A half-line in the region frame.
struct Ray {
    std::array<double, 3> origin{};
    std::array<double, 3> direction{};  ///< Unit length.
};

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
    /// The ray from the eye through a point of the view, given in normalised device
    /// coordinates: x from -1 (left) to 1 (right), y from -1 (top) to 1 (bottom).
    [[nodiscard]] Ray ray(double ndcX, double ndcY, double aspect) const;
    /// Unit vectors of the view in the region frame: right, up, forward.
    [[nodiscard]] std::array<std::array<double, 3>, 3> axes() const;
};

}  // namespace lagrange::viewer
