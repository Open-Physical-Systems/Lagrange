#include "lagrange/sensors/Sensors.hpp"
#include "lagrange/sensors/ShapeCaster.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <embree4/rtcore.h>

namespace lagrange::sensors {

namespace {

// Margin around every bounding sphere, so that Embree's traversal in float never culls a shape
// that the exact test in double hits. It covers the rounding of the ray to float: about 6e-8 of
// the origin's distance from the region origin (at most a few km: huge-worlds.md rebases at
// 1 km) plus 1.2e-7 of the distance along the ray (lidars reach a few hundred metres), both
// well under a millimetre. A wider margin costs a few extra exact tests, never accuracy.
constexpr double kMarginM = 1e-3;
constexpr double kMarginPerM = 1e-6;  // more for shapes far from the region origin

/// The ray of one cast, and its nearest hit so far in double.
struct Query {
    RTCRayQueryContext context;  // first, so that Embree's pointer to it is a pointer to this
    std::array<double, 3> origin;
    std::array<double, 3> dir;
    double nearest;
    std::uint32_t shape;  // of the nearest hit; services::RayHit::kNone for none
};

/// Keeps the nearer hit, and on a tie the lower shape index, whatever order shapes are tested.
bool keep(Query& q, double t, std::uint32_t shape) {
    if (t < q.nearest || (t == q.nearest && t < kNoHit && shape < q.shape)) {
        q.nearest = t;
        q.shape = shape;
        return true;
    }
    return false;
}

struct Geometry {
    const ShapeCaster::Impl* caster;
    std::uint32_t shape;
};

float down(double v) {
    const auto f = static_cast<float>(v);
    return static_cast<double>(f) > v ? std::nextafter(f, -std::numeric_limits<float>::infinity())
                                      : f;
}

float up(double v) {
    const auto f = static_cast<float>(v);
    return static_cast<double>(f) < v ? std::nextafter(f, std::numeric_limits<float>::infinity())
                                      : f;
}

}  // namespace

double boundingRadius(ShapeType type, const std::array<double, 3>& size) {
    switch (type) {
        case ShapeType::Plane:
            return std::numeric_limits<double>::infinity();
        case ShapeType::Sphere:
            return size[0];
        case ShapeType::Capsule:
            return size[0] + size[1];
        case ShapeType::Cylinder:
            return std::hypot(size[0], size[1]);
        case ShapeType::Box:
        case ShapeType::ConvexHull:   // its size bounds every point's coordinates
        case ShapeType::HeightField:  // likewise
            return std::sqrt(size[0] * size[0] + size[1] * size[1] + size[2] * size[2]);
    }
    return std::numeric_limits<double>::infinity();
}

struct ShapeCaster::Impl {
    RTCDevice device{nullptr};
    // Two levels: the shapes that have never moved, built once, and the ones that have, rebuilt
    // whenever one moves. `scene` holds an instance of each (identity transforms), so a scan of
    // a robot in a town no longer rebuilds the town's hierarchy. Which shapes a ray tests
    // changes, never its result: every hit is exact in double, the nearest kept, the lower
    // index on a tie.
    RTCScene scene{nullptr};
    RTCScene fixedScene{nullptr};
    RTCScene movingScene{nullptr};
    unsigned fixedInstance{RTC_INVALID_GEOMETRY_ID};
    unsigned movingInstance{RTC_INVALID_GEOMETRY_ID};
    std::vector<std::uint8_t> moving;  // per shape: has moved, so lives in movingScene
    std::vector<Shape> shapes;
    std::vector<double> radius;         // bounding sphere, with the margin
    std::vector<std::uint32_t> planes;  // tested directly
    std::vector<Geometry> geometries;   // user data of the bounded shapes
    std::vector<unsigned> geometryIds;  // per shape; RTC_INVALID_GEOMETRY_ID for planes
    bool built{false};

    [[nodiscard]] double exact(std::uint32_t k, const Query& q) const {
        const Shape& s = shapes[k];
        return raycast(s.type, s.size, s.pose, q.origin, q.dir, s.hull, s.field);
    }

    static void bounds(const RTCBoundsFunctionArguments* args) {
        const auto* g = static_cast<const Geometry*>(args->geometryUserPtr);
        const Impl& self = *g->caster;
        const std::array<double, 3>& c = self.shapes[g->shape].pose.position;
        const double r = self.radius[g->shape] +
                         kMarginPerM * std::max({std::abs(c[0]), std::abs(c[1]), std::abs(c[2])});
        RTCBounds& out = *args->bounds_o;
        out.lower_x = down(c[0] - r);
        out.lower_y = down(c[1] - r);
        out.lower_z = down(c[2] - r);
        out.upper_x = up(c[0] + r);
        out.upper_y = up(c[1] + r);
        out.upper_z = up(c[2] + r);
    }

    static void intersect(const RTCIntersectFunctionNArguments* args) {
        if (args->N != 1 || args->valid[0] == 0) {
            return;
        }
        const auto* g = static_cast<const Geometry*>(args->geometryUserPtr);
        auto* query = reinterpret_cast<Query*>(args->context);
        const double t = g->caster->exact(g->shape, *query);
        if (!keep(*query, t, g->shape)) {
            return;
        }
        // Embree may now skip what lies beyond, with the margin for its float traversal.
        RTCRayN* ray = RTCRayHitN_RayN(args->rayhit, 1);
        RTCRayN_tfar(ray, 1, 0) = up(t + kMarginM);
        RTCHitN* hit = RTCRayHitN_HitN(args->rayhit, 1);
        RTCHitN_geomID(hit, 1, 0) = args->geomID;
        RTCHitN_primID(hit, 1, 0) = args->primID;
    }

    /// A user geometry for bounded shape `k` (its user data in `geometries`), attached to
    /// `target`; returns its id there.
    unsigned attach(RTCScene target, Geometry& user) const {
        RTCGeometry geometry = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_USER);
        rtcSetGeometryUserPrimitiveCount(geometry, 1);
        rtcSetGeometryUserData(geometry, &user);
        rtcSetGeometryBoundsFunction(geometry, &Impl::bounds, nullptr);
        rtcSetGeometryIntersectFunction(geometry, &Impl::intersect);
        rtcCommitGeometry(geometry);
        const unsigned id = rtcAttachGeometry(target, geometry);
        rtcReleaseGeometry(geometry);
        return id;
    }

    unsigned instance(RTCScene of) const {
        RTCGeometry geometry = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_INSTANCE);
        rtcSetGeometryInstancedScene(geometry, of);
        constexpr std::array<float, 12> kIdentity{1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F,
                                                  0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F};
        rtcSetGeometryTransform(geometry, 0, RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR, kIdentity.data());
        rtcCommitGeometry(geometry);
        const unsigned id = rtcAttachGeometry(scene, geometry);
        rtcReleaseGeometry(geometry);
        return id;
    }

    void build() {
        device = rtcNewDevice(nullptr);
        if (device == nullptr) {
            return;  // cast() then tests every shape
        }
        scene = rtcNewScene(device);
        rtcSetSceneFlags(scene, RTC_SCENE_FLAG_DYNAMIC | RTC_SCENE_FLAG_ROBUST);
        fixedScene = rtcNewScene(device);
        rtcSetSceneFlags(fixedScene, RTC_SCENE_FLAG_ROBUST);
        rtcSetSceneBuildQuality(fixedScene, RTC_BUILD_QUALITY_MEDIUM);
        movingScene = rtcNewScene(device);
        rtcSetSceneFlags(movingScene, RTC_SCENE_FLAG_DYNAMIC | RTC_SCENE_FLAG_ROBUST);
        rtcSetSceneBuildQuality(movingScene, RTC_BUILD_QUALITY_LOW);
        geometries.reserve(shapes.size());  // stable addresses: Embree keeps pointers to them
        geometryIds.assign(shapes.size(), RTC_INVALID_GEOMETRY_ID);
        moving.assign(shapes.size(), 0);
        for (std::uint32_t k = 0; k < shapes.size(); ++k) {
            if (shapes[k].type == ShapeType::Plane) {
                planes.push_back(k);
                continue;
            }
            geometries.push_back({this, k});
            geometryIds[k] = attach(fixedScene, geometries.back());
        }
        rtcCommitScene(fixedScene);
        rtcCommitScene(movingScene);
        fixedInstance = instance(fixedScene);
        movingInstance = instance(movingScene);
        rtcCommitScene(scene);
    }
};

ShapeCaster::ShapeCaster() : impl_(std::make_unique<Impl>()) {}

ShapeCaster::~ShapeCaster() {
    for (RTCScene scene : {impl_->scene, impl_->fixedScene, impl_->movingScene}) {
        if (scene != nullptr) {
            rtcReleaseScene(scene);
        }
    }
    if (impl_->device != nullptr) {
        rtcReleaseDevice(impl_->device);
    }
}

void ShapeCaster::update(std::span<const Shape> shapes) {
    Impl& s = *impl_;
    if (!s.built) {
        s.shapes.assign(shapes.begin(), shapes.end());
        s.radius.resize(shapes.size());
        for (std::size_t k = 0; k < shapes.size(); ++k) {
            s.radius[k] = boundingRadius(shapes[k].type, shapes[k].size) + kMarginM;
        }
        s.build();
        s.built = true;
        return;
    }
    bool moved = false;
    bool fixedChanged = false;
    std::size_t user = 0;  // index in geometries of shape k, when bounded
    for (std::size_t k = 0; k < shapes.size(); ++k) {
        const bool bounded =
            s.geometryIds.size() > k && s.geometryIds[k] != RTC_INVALID_GEOMETRY_ID;
        const Pose& now = shapes[k].pose;
        if (now.position == s.shapes[k].pose.position &&
            now.orientation == s.shapes[k].pose.orientation) {
            user += bounded ? 1 : 0;
            continue;
        }
        s.shapes[k].pose = now;
        if (s.scene != nullptr && bounded) {
            if (s.moving[k] == 0) {
                // Its first move: out of the hierarchy built once, into the one rebuilt.
                rtcDetachGeometry(s.fixedScene, s.geometryIds[k]);
                s.geometryIds[k] = s.attach(s.movingScene, s.geometries[user]);
                s.moving[k] = 1;
                fixedChanged = true;
            } else {
                rtcCommitGeometry(rtcGetGeometry(s.movingScene, s.geometryIds[k]));
            }
            moved = true;
        }
        user += bounded ? 1 : 0;
    }
    if (moved) {
        if (fixedChanged) {
            rtcCommitScene(s.fixedScene);
            rtcCommitGeometry(rtcGetGeometry(s.scene, s.fixedInstance));
        }
        rtcCommitScene(s.movingScene);
        rtcCommitGeometry(rtcGetGeometry(s.scene, s.movingInstance));
        rtcCommitScene(s.scene);
    }
}

double ShapeCaster::cast(const std::array<double, 3>& origin,
                         const std::array<double, 3>& dir) const {
    return castHit(origin, dir).distance;
}

services::RayHit ShapeCaster::castHit(const std::array<double, 3>& origin,
                                      const std::array<double, 3>& dir) const {
    const Impl& s = *impl_;
    Query query;
    rtcInitRayQueryContext(&query.context);
    query.origin = origin;
    query.dir = dir;
    query.nearest = kNoHit;
    query.shape = services::RayHit::kNone;
    if (s.scene == nullptr) {
        for (std::uint32_t k = 0; k < s.shapes.size(); ++k) {
            keep(query, s.exact(k, query), k);
        }
        return {query.nearest, query.shape};
    }
    for (const std::uint32_t k : s.planes) {
        keep(query, s.exact(k, query), k);
    }
    RTCRayHit rayhit{};
    rayhit.ray.org_x = static_cast<float>(origin[0]);
    rayhit.ray.org_y = static_cast<float>(origin[1]);
    rayhit.ray.org_z = static_cast<float>(origin[2]);
    rayhit.ray.dir_x = static_cast<float>(dir[0]);
    rayhit.ray.dir_y = static_cast<float>(dir[1]);
    rayhit.ray.dir_z = static_cast<float>(dir[2]);
    rayhit.ray.tnear = 0.0F;
    // A plane already hit bounds the search, with the same margin as in intersect().
    rayhit.ray.tfar = query.nearest < kNoHit ? up(query.nearest + kMarginM)
                                             : std::numeric_limits<float>::infinity();
    rayhit.ray.mask = 0xFFFFFFFFU;
    rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    RTCIntersectArguments arguments;
    rtcInitIntersectArguments(&arguments);
    arguments.context = &query.context;
    arguments.intersect = &Impl::intersect;
    rtcIntersect1(s.scene, &rayhit, &arguments);
    return {query.nearest, query.shape};
}

services::RayHit ShapeCaster::ray(const std::array<double, 3>& origin,
                                  const std::array<double, 3>& dir, double maxDistance) const {
    const services::RayHit hit = castHit(origin, dir);
    return hit.distance <= maxDistance ? hit : services::RayHit{};
}

std::vector<std::uint32_t> ShapeCaster::overlapSphere(const std::array<double, 3>& center,
                                                      double radius) const {
    const Impl& s = *impl_;
    std::vector<std::uint32_t> out;
    for (std::uint32_t k = 0; k < s.shapes.size(); ++k) {
        const Shape& shape = s.shapes[k];
        // Bounding spheres first: only shapes that could reach the query are tested exactly.
        if (shape.type != ShapeType::Plane) {
            const std::array<double, 3>& c = shape.pose.position;
            const double gap = std::hypot(center[0] - c[0], center[1] - c[1], center[2] - c[2]);
            if (gap > radius + s.radius[k]) {
                continue;
            }
        }
        if (distanceToShape(shape.type, shape.size, shape.pose, center, shape.hull) <= radius) {
            out.push_back(k);
        }
    }
    return out;
}

std::array<double, 3> surfaceNormal(ShapeType type, const std::array<double, 3>& size,
                                    const Pose& pose, const std::array<double, 3>& point,
                                    const ConvexHull* hull, const HeightField* field) {
    const std::array<double, 3> q = rotate(
        conjugate(pose.orientation),
        {point[0] - pose.position[0], point[1] - pose.position[1], point[2] - pose.position[2]});
    std::array<double, 3> n{0.0, 0.0, 1.0};
    switch (type) {
        case ShapeType::Plane:
            break;
        case ShapeType::Sphere:
            n = q;
            break;
        case ShapeType::Box: {
            // The face whose distance from the surface is smallest.
            double best = std::numeric_limits<double>::infinity();
            for (std::size_t i = 0; i < 3; ++i) {
                const double gap = std::abs(std::abs(q.at(i)) - size.at(i));
                if (gap < best) {
                    best = gap;
                    n = {0.0, 0.0, 0.0};
                    n.at(i) = q.at(i) < 0.0 ? -1.0 : 1.0;
                }
            }
            break;
        }
        case ShapeType::Capsule:
            n = {q[0], q[1], q[2] - std::clamp(q[2], -size[1], size[1])};
            break;
        case ShapeType::Cylinder:
            n = std::abs(std::abs(q[2]) - size[1]) < std::abs(std::hypot(q[0], q[1]) - size[0])
                    ? std::array<double, 3>{0.0, 0.0, q[2] < 0.0 ? -1.0 : 1.0}
                    : std::array<double, 3>{q[0], q[1], 0.0};
            break;
        case ShapeType::ConvexHull:
            if (hull != nullptr) {
                double best = -std::numeric_limits<double>::infinity();
                for (const auto& t : hull->triangles) {
                    const auto& a = hull->points.at(t[0]);
                    const auto& b = hull->points.at(t[1]);
                    const auto& c = hull->points.at(t[2]);
                    const std::array<double, 3> u{b[0] - a[0], b[1] - a[1], b[2] - a[2]};
                    const std::array<double, 3> v{c[0] - a[0], c[1] - a[1], c[2] - a[2]};
                    std::array<double, 3> m{u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
                                            u[0] * v[1] - u[1] * v[0]};
                    const double length = std::hypot(m[0], m[1], m[2]);
                    if (length <= 0.0) {
                        continue;
                    }
                    m = {m[0] / length, m[1] / length, m[2] / length};
                    const double along =
                        (q[0] - a[0]) * m[0] + (q[1] - a[1]) * m[1] + (q[2] - a[2]) * m[2];
                    if (along > best) {
                        best = along;
                        n = m;
                    }
                }
            }
            break;
        case ShapeType::HeightField:
            if (field != nullptr) {
                const double h = std::max(field->cellX, 1e-3);
                const double k = std::max(field->cellY, 1e-3);
                const double x0 = field->heightAt(q[0] - h, q[1]).value_or(q[2]);
                const double x1 = field->heightAt(q[0] + h, q[1]).value_or(q[2]);
                const double y0 = field->heightAt(q[0], q[1] - k).value_or(q[2]);
                const double y1 = field->heightAt(q[0], q[1] + k).value_or(q[2]);
                n = {(x0 - x1) / (2.0 * h), (y0 - y1) / (2.0 * k), 1.0};
            }
            break;
    }
    const double length = std::hypot(n[0], n[1], n[2]);
    if (length <= 0.0) {
        return rotate(pose.orientation, {0.0, 0.0, 1.0});
    }
    return rotate(pose.orientation, {n[0] / length, n[1] / length, n[2] / length});
}

LidarReturn lidarReturn(const SurfaceMaterial& m, bool hasMaterial, bool wavelength1550,
                        double cosIncidence, double range, double maxRange,
                        double referenceReflectivity) {
    const double rho = wavelength1550 ? m.reflectivity1550 : m.reflectivity905;
    const double cosine = std::clamp(cosIncidence, 0.0, 1.0);
    double power = 0.0;  // relative to a white wall, head-on, at any range
    if (m.retroreflective) {
        power = kRetroGain * rho;
    } else {
        const double angle = std::acos(cosine) / kSpecularLobeRad;
        power = rho * ((1.0 - m.specular) * cosine + m.specular * std::exp(-angle * angle));
    }
    const double r = std::max(range, 1e-3);
    LidarReturn out;
    out.intensity = std::min(power * kIntensityReferenceM * kIntensityReferenceM / (r * r), 1.0);
    if (hasMaterial && referenceReflectivity > 0.0) {
        out.detected = power / (r * r) >= referenceReflectivity / (maxRange * maxRange);
    }
    return out;
}

double distanceToShape(ShapeType type, const std::array<double, 3>& size, const Pose& pose,
                       const std::array<double, 3>& point, const ConvexHull* hull) {
    const std::array<double, 3> q = rotate(
        conjugate(pose.orientation),
        {point[0] - pose.position[0], point[1] - pose.position[1], point[2] - pose.position[2]});
    switch (type) {
        case ShapeType::Plane:
            return std::max(q[2], 0.0);
        case ShapeType::Sphere:
            return std::max(std::hypot(q[0], q[1], q[2]) - size[0], 0.0);
        case ShapeType::Box:
        case ShapeType::HeightField: {  // its box: a lower bound of the distance
            const double dx = std::max(std::abs(q[0]) - size[0], 0.0);
            const double dy = std::max(std::abs(q[1]) - size[1], 0.0);
            const double dz = std::max(std::abs(q[2]) - size[2], 0.0);
            return std::hypot(dx, dy, dz);
        }
        case ShapeType::Capsule: {
            const double z = std::clamp(q[2], -size[1], size[1]);
            return std::max(std::hypot(q[0], q[1], q[2] - z) - size[0], 0.0);
        }
        case ShapeType::Cylinder: {
            const double radial = std::max(std::hypot(q[0], q[1]) - size[0], 0.0);
            const double axial = std::max(std::abs(q[2]) - size[1], 0.0);
            return std::hypot(radial, axial);
        }
        case ShapeType::ConvexHull: {
            if (hull == nullptr) {
                return std::numeric_limits<double>::infinity();
            }
            double farthest = 0.0;
            std::array<double, 3> centre{};
            for (const auto& p : hull->points) {
                for (std::size_t i = 0; i < 3; ++i) {
                    centre.at(i) += p.at(i) / static_cast<double>(hull->points.size());
                }
            }
            for (const auto& t : hull->triangles) {
                const auto& a = hull->points.at(t[0]);
                const auto& b = hull->points.at(t[1]);
                const auto& c = hull->points.at(t[2]);
                const std::array<double, 3> u{b[0] - a[0], b[1] - a[1], b[2] - a[2]};
                const std::array<double, 3> v{c[0] - a[0], c[1] - a[1], c[2] - a[2]};
                std::array<double, 3> n{u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
                                        u[0] * v[1] - u[1] * v[0]};
                const double length = std::hypot(n[0], n[1], n[2]);
                if (length <= 0.0) {
                    continue;
                }
                const auto side = [&n, &a](const std::array<double, 3>& x) {
                    return n[0] * (x[0] - a[0]) + n[1] * (x[1] - a[1]) + n[2] * (x[2] - a[2]);
                };
                const double sign = side(centre) > 0.0 ? -1.0 : 1.0;  // outward
                farthest = std::max(farthest, sign * side(q) / length);
            }
            return farthest;
        }
    }
    return std::numeric_limits<double>::infinity();
}

}  // namespace lagrange::sensors
