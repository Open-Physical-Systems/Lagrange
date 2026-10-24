#pragma once

// Small rigid-body maths for the stage reader and writer.
#include "lagrange/world/PoseMath.hpp"

#include <array>
#include <cmath>

namespace lagrange::stage {

/// The pose of `b` in the frame of `a` (both in the same parent frame).
[[nodiscard]] inline Pose relative(const Pose& a, const Pose& b) {
    const std::array<double, 4> inverse = conjugate(a.orientation);
    return {rotate(inverse, {b.position[0] - a.position[0], b.position[1] - a.position[1],
                             b.position[2] - a.position[2]}),
            multiply(inverse, b.orientation)};
}

/// The shortest rotation taking the x axis onto unit vector `axis`.
[[nodiscard]] inline std::array<double, 4> rotationFromXTo(const std::array<double, 3>& axis) {
    const double n = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    if (n == 0.0) {
        return {1.0, 0.0, 0.0, 0.0};
    }
    const std::array<double, 3> a{axis[0] / n, axis[1] / n, axis[2] / n};
    const double d = a[0];  // x . a
    if (d < -1.0 + 1e-12) {
        return {0.0, 0.0, 0.0, 1.0};  // half turn about z
    }
    // q = (1 + x.a, x cross a), normalised; x cross a = (0, -a.z, a.y).
    const std::array<double, 4> q{1.0 + d, 0.0, -a[2], a[1]};
    const double m = std::sqrt(q[0] * q[0] + q[2] * q[2] + q[3] * q[3]);
    return {q[0] / m, 0.0, q[2] / m, q[3] / m};
}

/// Inertia (xx, yy, zz, xy, xz, yz) as principal moments and the rotation of the principal
/// frame in the body frame (UsdPhysicsMassAPI's diagonalInertia and principalAxes): Jacobi
/// rotations on the symmetric 3x3 tensor.
struct Principal {
    std::array<double, 3> moments{};
    std::array<double, 4> axes{1.0, 0.0, 0.0, 0.0};
};

[[nodiscard]] inline Principal principalAxes(const std::array<double, 6>& inertia) {
    double m[3][3] = {{inertia[0], inertia[3], inertia[4]},
                      {inertia[3], inertia[1], inertia[5]},
                      {inertia[4], inertia[5], inertia[2]}};
    double v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};  // columns: eigenvectors
    for (int sweep = 0; sweep < 50; ++sweep) {
        const double off = std::abs(m[0][1]) + std::abs(m[0][2]) + std::abs(m[1][2]);
        if (off < 1e-15 * (std::abs(m[0][0]) + std::abs(m[1][1]) + std::abs(m[2][2]) + 1e-300)) {
            break;
        }
        for (int p = 0; p < 2; ++p) {
            for (int q = p + 1; q < 3; ++q) {
                if (m[p][q] == 0.0) {
                    continue;
                }
                const double theta = (m[q][q] - m[p][p]) / (2.0 * m[p][q]);
                const double t =
                    (theta >= 0 ? 1.0 : -1.0) / (std::abs(theta) + std::sqrt(theta * theta + 1.0));
                const double c = 1.0 / std::sqrt(t * t + 1.0);
                const double s = t * c;
                for (int k = 0; k < 3; ++k) {  // m = J^T m J
                    const double mkp = m[k][p];
                    const double mkq = m[k][q];
                    m[k][p] = c * mkp - s * mkq;
                    m[k][q] = s * mkp + c * mkq;
                }
                for (int k = 0; k < 3; ++k) {
                    const double mpk = m[p][k];
                    const double mqk = m[q][k];
                    m[p][k] = c * mpk - s * mqk;
                    m[q][k] = s * mpk + c * mqk;
                }
                for (int k = 0; k < 3; ++k) {
                    const double vkp = v[k][p];
                    const double vkq = v[k][q];
                    v[k][p] = c * vkp - s * vkq;
                    v[k][q] = s * vkp + c * vkq;
                }
            }
        }
    }
    // A right-handed frame: flip the third axis if needed.
    const double det = v[0][0] * (v[1][1] * v[2][2] - v[2][1] * v[1][2]) -
                       v[0][1] * (v[1][0] * v[2][2] - v[2][0] * v[1][2]) +
                       v[0][2] * (v[1][0] * v[2][1] - v[2][0] * v[1][1]);
    if (det < 0) {
        for (auto& row : v) {
            row[2] = -row[2];
        }
    }
    // Rotation matrix (columns v) to quaternion (w, x, y, z).
    const double trace = v[0][0] + v[1][1] + v[2][2];
    std::array<double, 4> q{};
    if (trace > 0) {
        const double s = 2.0 * std::sqrt(trace + 1.0);
        q = {0.25 * s, (v[2][1] - v[1][2]) / s, (v[0][2] - v[2][0]) / s, (v[1][0] - v[0][1]) / s};
    } else if (v[0][0] > v[1][1] && v[0][0] > v[2][2]) {
        const double s = 2.0 * std::sqrt(1.0 + v[0][0] - v[1][1] - v[2][2]);
        q = {(v[2][1] - v[1][2]) / s, 0.25 * s, (v[0][1] + v[1][0]) / s, (v[0][2] + v[2][0]) / s};
    } else if (v[1][1] > v[2][2]) {
        const double s = 2.0 * std::sqrt(1.0 + v[1][1] - v[0][0] - v[2][2]);
        q = {(v[0][2] - v[2][0]) / s, (v[0][1] + v[1][0]) / s, 0.25 * s, (v[1][2] + v[2][1]) / s};
    } else {
        const double s = 2.0 * std::sqrt(1.0 + v[2][2] - v[0][0] - v[1][1]);
        q = {(v[1][0] - v[0][1]) / s, (v[0][2] + v[2][0]) / s, (v[1][2] + v[2][1]) / s, 0.25 * s};
    }
    return {{m[0][0], m[1][1], m[2][2]}, q};
}

/// The inverse of principalAxes: R diag(moments) R^T as (xx, yy, zz, xy, xz, yz).
[[nodiscard]] inline std::array<double, 6> inertiaTensor(const std::array<double, 3>& moments,
                                                         const std::array<double, 4>& axes) {
    std::array<std::array<double, 3>, 3> r{};
    for (int c = 0; c < 3; ++c) {
        std::array<double, 3> unit{0, 0, 0};
        unit[static_cast<std::size_t>(c)] = 1.0;
        const std::array<double, 3> col = rotate(axes, unit);
        for (int k = 0; k < 3; ++k) {
            r[static_cast<std::size_t>(k)][static_cast<std::size_t>(c)] =
                col[static_cast<std::size_t>(k)];
        }
    }
    const auto at = [&](int i, int j) {
        double sum = 0.0;
        for (int k = 0; k < 3; ++k) {
            sum += r[static_cast<std::size_t>(i)][static_cast<std::size_t>(k)] *
                   moments[static_cast<std::size_t>(k)] *
                   r[static_cast<std::size_t>(j)][static_cast<std::size_t>(k)];
        }
        return sum;
    };
    return {at(0, 0), at(1, 1), at(2, 2), at(0, 1), at(0, 2), at(1, 2)};
}

}  // namespace lagrange::stage
