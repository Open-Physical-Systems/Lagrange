#pragma once

// Small 3D helpers of the builder's model (RFC 0016 WP-A): 3×3 matrices, quaternions {w, x, y,
// z} and symmetric inertia tensors in URDF's order (ixx ixy ixz iyy iyz izz). Private to
// viewer/builder/src; nothing here is part of an interface.

#include <array>
#include <cmath>
#include <cstddef>

namespace lagrange::viewer::builder::detail {

using Vec3 = std::array<double, 3>;
using Mat3 = std::array<std::array<double, 3>, 3>;  // row major: m[row][column]
using Quat = std::array<double, 4>;                 // w x y z
using Tensor6 = std::array<double, 6>;              // ixx ixy ixz iyy iyz izz

inline Quat multiply(const Quat& a, const Quat& b) {
    return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}

/// The rotation matrix of a unit quaternion.
inline Mat3 matrixOf(const Quat& q) {
    const double w = q[0];
    const double x = q[1];
    const double y = q[2];
    const double z = q[3];
    return {{{1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
             {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
             {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)}}};
}

inline Vec3 apply(const Mat3& m, const Vec3& v) {
    Vec3 out{};
    for (std::size_t r = 0; r < 3; ++r) {
        out.at(r) = m.at(r)[0] * v[0] + m.at(r)[1] * v[1] + m.at(r)[2] * v[2];
    }
    return out;
}

inline Mat3 toMatrix(const Tensor6& t) {
    return {{{t[0], t[1], t[2]}, {t[1], t[3], t[4]}, {t[2], t[4], t[5]}}};
}

inline Tensor6 toTensor(const Mat3& m) {
    return {m[0][0], m[0][1], m[0][2], m[1][1], m[1][2], m[2][2]};
}

/// R I Rᵀ: a tensor given in a rotated frame, expressed in the outer frame.
inline Tensor6 rotateTensor(const Mat3& r, const Tensor6& tensor) {
    const Mat3 inertia = toMatrix(tensor);
    Mat3 ri{};
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            for (std::size_t k = 0; k < 3; ++k) {
                ri.at(i).at(j) += r.at(i).at(k) * inertia.at(k).at(j);
            }
        }
    }
    Mat3 out{};
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            for (std::size_t k = 0; k < 3; ++k) {
                out.at(i).at(j) += ri.at(i).at(k) * r.at(j).at(k);
            }
        }
    }
    return toTensor(out);
}

/// The parallel axis theorem: the tensor of mass `mass` about a point displaced by `d` from its
/// COM is I + m (|d|² E − d dᵀ).
inline Tensor6 shiftTensor(const Tensor6& about, double mass, const Vec3& d) {
    const double dd = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
    return {about[0] + mass * (dd - d[0] * d[0]), about[1] - mass * d[0] * d[1],
            about[2] - mass * d[0] * d[2],        about[3] + mass * (dd - d[1] * d[1]),
            about[4] - mass * d[1] * d[2],        about[5] + mass * (dd - d[2] * d[2])};
}

}  // namespace lagrange::viewer::builder::detail
