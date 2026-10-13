#include "lagrange/viewer/Picking.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lagrange::viewer {

namespace {

using Vec3 = std::array<double, 3>;

/// Rotates `v` by the inverse of the unit quaternion q = (w, x, y, z).
Vec3 rotateInverse(const std::array<float, 4>& q, const Vec3& v) {
    // v' = v + 2 u x (u x v + w v) with u = -(x, y, z) for the inverse.
    const Vec3 u{-static_cast<double>(q[1]), -static_cast<double>(q[2]),
                 -static_cast<double>(q[3])};
    const double w = q[0];
    const Vec3 t{u[1] * v[2] - u[2] * v[1] + w * v[0], u[2] * v[0] - u[0] * v[2] + w * v[1],
                 u[0] * v[1] - u[1] * v[0] + w * v[2]};
    return {v[0] + 2.0 * (u[1] * t[2] - u[2] * t[1]), v[1] + 2.0 * (u[2] * t[0] - u[0] * t[2]),
            v[2] + 2.0 * (u[0] * t[1] - u[1] * t[0])};
}

}  // namespace

std::uint32_t pickPrim(const SceneFrame& frame, const Ray& ray) {
    if (!frame.statics) {
        return kNone;
    }
    const SceneStatics& statics = *frame.statics;
    std::uint32_t best = kNone;
    double nearest = std::numeric_limits<double>::infinity();
    for (std::uint32_t i = 0; i < statics.prims.size(); ++i) {
        const PrimData& prim = statics.prims[i];
        if (prim.mesh == kNone || !prim.visible || prim.mesh >= statics.meshes.size()) {
            continue;
        }
        const Transform& transform = frame.transforms[i];
        const std::array<float, 6>& box = statics.meshes[prim.mesh].aabb;
        // The ray in the prim's own frame; the distance along it stays the same.
        const Vec3 origin =
            rotateInverse(transform.rotation, {ray.origin[0] - transform.position[0],
                                               ray.origin[1] - transform.position[1],
                                               ray.origin[2] - transform.position[2]});
        const Vec3 direction = rotateInverse(transform.rotation, ray.direction);
        double enter = 0.0;
        double leave = std::numeric_limits<double>::infinity();
        bool hit = true;
        for (std::size_t axis = 0; axis < 3 && hit; ++axis) {
            const double scale = std::abs(static_cast<double>(transform.scale.at(axis)));
            const double low = static_cast<double>(box.at(axis)) * scale;
            const double high = static_cast<double>(box.at(axis + 3)) * scale;
            if (std::abs(direction.at(axis)) < 1e-12) {
                hit = origin.at(axis) >= low && origin.at(axis) <= high;
                continue;
            }
            double t0 = (low - origin.at(axis)) / direction.at(axis);
            double t1 = (high - origin.at(axis)) / direction.at(axis);
            if (t0 > t1) {
                std::swap(t0, t1);
            }
            enter = std::max(enter, t0);
            leave = std::min(leave, t1);
            hit = enter <= leave;
        }
        if (hit && enter < nearest) {
            nearest = enter;
            best = i;
        }
    }
    return best;
}

std::optional<std::array<double, 3>> intersectGround(const Ray& ray, double height) {
    if (std::abs(ray.direction[2]) < 1e-9) {
        return std::nullopt;
    }
    const double t = (height - ray.origin[2]) / ray.direction[2];
    if (t <= 0.0) {
        return std::nullopt;
    }
    return std::array<double, 3>{ray.origin[0] + ray.direction[0] * t,
                                 ray.origin[1] + ray.direction[1] * t, height};
}

}  // namespace lagrange::viewer
