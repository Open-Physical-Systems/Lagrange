// Mass, centre of mass and inertia from shapes (RFC 0016 §4).
//
// Uniform solids, about their centre, in the shape frame (the closed forms of RFC 0016 §4):
//   box a × b × c:          m = ρabc,      Ixx = m(b² + c²)/12, Iyy = m(a² + c²)/12,
//                                           Izz = m(a² + b²)/12
//   cylinder r, h (axis z): m = ρπr²h,     Ixx = Iyy = m(3r² + h²)/12, Izz = mr²/2
//   sphere r:               m = ρ·4πr³/3,  I = 2mr²/5
// Shapes combine by R I Rᵀ into the link frame and the parallel axis theorem to the common COM.
#include "Math.hpp"
#include "lagrange/viewer/builder/Kinematics.hpp"
#include "lagrange/viewer/builder/MassProperties.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <vector>

namespace lagrange::viewer::builder {

namespace {

using detail::Tensor6;
using detail::Vec3;

/// The shape's own mass properties about its centre, in its own frame (origin not applied).
MassProperties ownMass(const Shape& shape, double density) {
    MassProperties out;
    if (density <= 0.0 || !std::isfinite(density)) {
        return out;
    }
    switch (shape.kind) {
        case Shape::Kind::Box: {
            const double a = shape.size[0];
            const double b = shape.size[1];
            const double c = shape.size[2];
            if (a <= 0.0 || b <= 0.0 || c <= 0.0) {
                return out;
            }
            const double m = density * a * b * c;
            out.mass = m;
            out.inertia = {m * (b * b + c * c) / 12.0, 0.0, 0.0,
                           m * (a * a + c * c) / 12.0, 0.0, m * (a * a + b * b) / 12.0};
            return out;
        }
        case Shape::Kind::Cylinder: {
            const double r = shape.size[0];
            const double h = shape.size[1];
            if (r <= 0.0 || h <= 0.0) {
                return out;
            }
            const double m = density * std::numbers::pi * r * r * h;
            const double across = m * (3.0 * r * r + h * h) / 12.0;
            out.mass = m;
            out.inertia = {across, 0.0, 0.0, across, 0.0, m * r * r / 2.0};
            return out;
        }
        case Shape::Kind::Sphere: {
            const double r = shape.size[0];
            if (r <= 0.0) {
                return out;
            }
            const double m = density * 4.0 * std::numbers::pi * r * r * r / 3.0;
            const double i = 2.0 * m * r * r / 5.0;
            out.mass = m;
            out.inertia = {i, 0.0, 0.0, i, 0.0, i};
            return out;
        }
        case Shape::Kind::Mesh:
            return out;  // phase 2 (R12): meshes get mass from lagrange_meshes
    }
    return out;
}

/// Shapes of one link combined in the link frame (no override applied).
MassProperties combine(const std::vector<MassProperties>& parts) {
    MassProperties out;
    for (const MassProperties& part : parts) {
        out.mass += part.mass;
    }
    if (out.mass <= 0.0) {
        return {};
    }
    Vec3 com{};
    for (const MassProperties& part : parts) {
        for (std::size_t k = 0; k < 3; ++k) {
            com.at(k) += part.mass * part.com.at(k) / out.mass;
        }
    }
    out.com = com;
    Tensor6 sum{};
    for (const MassProperties& part : parts) {
        const Vec3 d{part.com[0] - com[0], part.com[1] - com[1], part.com[2] - com[2]};
        const Tensor6 moved = detail::shiftTensor(part.inertia, part.mass, d);
        for (std::size_t k = 0; k < 6; ++k) {
            sum.at(k) += moved.at(k);
        }
    }
    out.inertia = sum;
    return out;
}

/// Eigenvalues of a symmetric 3×3 matrix, ascending (Smith, "Eigenvalues of a symmetric 3 × 3
/// matrix", Comm. ACM 4(4), 1961: the trigonometric closed form).
std::array<double, 3> eigenvalues(const Tensor6& t) {
    const double a = t[0];
    const double b = t[3];
    const double c = t[5];
    const double d = t[1];
    const double e = t[4];
    const double f = t[2];
    const double off = d * d + e * e + f * f;
    std::array<double, 3> out{};
    if (off == 0.0) {
        out = {a, b, c};
        std::ranges::sort(out);
        return out;
    }
    const double q = (a + b + c) / 3.0;
    const double p2 = (a - q) * (a - q) + (b - q) * (b - q) + (c - q) * (c - q) + 2.0 * off;
    const double p = std::sqrt(p2 / 6.0);
    // B = (A − qE)/p; r = det(B)/2.
    const double b11 = (a - q) / p;
    const double b22 = (b - q) / p;
    const double b33 = (c - q) / p;
    const double b12 = d / p;
    const double b23 = e / p;
    const double b13 = f / p;
    const double det = b11 * (b22 * b33 - b23 * b23) - b12 * (b12 * b33 - b23 * b13) +
                       b13 * (b12 * b23 - b22 * b13);
    const double r = std::clamp(det / 2.0, -1.0, 1.0);
    const double phi = std::acos(r) / 3.0;
    const double largest = q + 2.0 * p * std::cos(phi);
    const double smallest = q + 2.0 * p * std::cos(phi + 2.0 * std::numbers::pi / 3.0);
    out = {smallest, 3.0 * q - largest - smallest, largest};
    std::ranges::sort(out);
    return out;
}

}  // namespace

MassProperties shapeMass(const Shape& shape, double density) {
    MassProperties own = ownMass(shape, density);
    if (own.mass <= 0.0) {
        return {};
    }
    const Frame frame = toFrame(shape.origin);
    own.inertia = detail::rotateTensor(detail::matrixOf(frame.rotation), own.inertia);
    own.com = frame.position;
    return own;
}

std::optional<MassProperties> linkMass(const Link& link) {
    const Inertial& given = link.inertial;
    // The distribution of the solid shapes. With a mass override and no density, the shapes
    // still give the distribution (any uniform density does: the override scales it).
    const double density = given.density > 0.0 ? given.density : (given.mass ? 1.0 : 0.0);
    std::vector<MassProperties> parts;
    for (const Shape& shape : link.shapes) {
        if (shape.solid) {
            if (const MassProperties part = shapeMass(shape, density); part.mass > 0.0) {
                parts.push_back(part);
            }
        }
    }
    MassProperties out = combine(parts);
    if (given.mass) {
        if (out.mass > 0.0) {
            const double scale = *given.mass / out.mass;
            for (double& value : out.inertia) {
                value *= scale;
            }
        }
        out.mass = *given.mass;
    }
    if (out.mass <= 0.0 && !given.mass) {
        return std::nullopt;  // nothing gives this link a mass
    }
    if (given.inertia) {
        out.inertia = *given.inertia;
    }
    if (given.com) {
        out.com = *given.com;  // the tensor stays about the stated COM, as URDF's <inertial>
    }
    return out;
}

InertiaCheck checkInertia(const std::array<double, 6>& inertia) {
    if (!std::ranges::all_of(inertia, [](double v) { return std::isfinite(v); })) {
        return InertiaCheck::NotPositiveDefinite;
    }
    const std::array<double, 3> moments = eigenvalues(inertia);
    if (moments[0] <= 0.0) {
        return InertiaCheck::NotPositiveDefinite;
    }
    // The two smallest principal moments must add up to at least the largest. A thin disc or
    // rod meets it with equality; allow rounding of the order of the tensor's own size.
    const double tolerance = 1e-9 * (moments[0] + moments[1] + moments[2]);
    if (moments[0] + moments[1] < moments[2] - tolerance) {
        return InertiaCheck::TriangleInequality;
    }
    return InertiaCheck::Ok;
}

}  // namespace lagrange::viewer::builder
