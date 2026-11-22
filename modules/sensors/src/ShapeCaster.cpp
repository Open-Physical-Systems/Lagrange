#include "lagrange/sensors/Sensors.hpp"
#include "lagrange/sensors/ShapeCaster.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

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
    services::RayHit hit = castHit(origin, dir);
    if (!(hit.distance <= maxDistance)) {
        return services::RayHit{};
    }
    // The surface's outward normal where the ray meets it (vehicle contact planes).
    const Shape& s = impl_->shapes[hit.shape];
    const std::array<double, 3> at{origin[0] + dir[0] * hit.distance,
                                   origin[1] + dir[1] * hit.distance,
                                   origin[2] + dir[2] * hit.distance};
    hit.normal = surfaceNormal(s.type, s.size, s.pose, at, s.hull, s.field);
    return hit;
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

namespace {
// The canopy's path along the beam and its extinction per metre.
std::pair<double, double> canopyPath(double canopyHeight, double leafAreaIndex, double cosIncidence,
                                     double range) {
    constexpr double kMinCosine = 1e-3;  // a beam along the soil: bounded by its range
    const double c = std::max(std::abs(cosIncidence), kMinCosine);
    const double path = std::min(canopyHeight / c, std::max(range, 0.0));
    return {path, kCanopyProjection * std::max(leafAreaIndex, 0.0) / canopyHeight};
}
}  // namespace

double canopyShortening(double canopyHeight, double cosIncidence, double range, double u) {
    return canopyShortening(canopyHeight, kCanopyLeafAreaIndex, cosIncidence, range, u);
}

double canopyMeanShortening(double canopyHeight, double cosIncidence, double range) {
    return canopyMeanShortening(canopyHeight, kCanopyLeafAreaIndex, cosIncidence, range);
}

double canopyShortening(double canopyHeight, double leafAreaIndex, double cosIncidence,
                        double range, double u) {
    if (!(canopyHeight > 0.0) || !(leafAreaIndex > 0.0) || !std::isfinite(range)) {
        return 0.0;
    }
    const auto [path, mu] = canopyPath(canopyHeight, leafAreaIndex, cosIncidence, range);
    const double s = -std::log1p(-std::clamp(u, 0.0, 1.0 - 1e-16)) / mu;
    return s >= path ? 0.0 : path - s;
}

double canopyMeanShortening(double canopyHeight, double leafAreaIndex, double cosIncidence,
                            double range) {
    if (!(canopyHeight > 0.0) || !(leafAreaIndex > 0.0) || !std::isfinite(range)) {
        return 0.0;
    }
    const auto [path, mu] = canopyPath(canopyHeight, leafAreaIndex, cosIncidence, range);
    return path + std::expm1(-mu * path) / mu;
}

bool canopyVolumeSpan(ShapeType type, const std::array<double, 3>& size, const Pose& placed,
                      const std::array<double, 3>& origin, const std::array<double, 3>& direction,
                      double canopyHeight, double range, double& enter, double& leave) {
    if (!(canopyHeight > 0.0) || (type != ShapeType::Box && type != ShapeType::Plane)) {
        return false;
    }
    // The beam in the collider's frame.
    const std::array<double, 4> back = conjugate(placed.orientation);
    const std::array<double, 3> o =
        rotate(back, {origin[0] - placed.position[0], origin[1] - placed.position[1],
                      origin[2] - placed.position[2]});
    const std::array<double, 3> d = rotate(back, direction);
    constexpr double kUnbounded = std::numeric_limits<double>::infinity();
    std::array<double, 3> lo{-kUnbounded, -kUnbounded, 0.0};
    std::array<double, 3> hi{kUnbounded, kUnbounded, canopyHeight};
    if (type == ShapeType::Box) {
        lo = {-size[0], -size[1], size[2]};
        hi = {size[0], size[1], size[2] + canopyHeight};
    }
    double t0 = 0.0;
    double t1 = std::isfinite(range) ? range : kUnbounded;
    for (std::size_t k = 0; k < 3; ++k) {  // slabs (Kay and Kajiya 1986)
        if (std::abs(d.at(k)) < 1e-15) {
            if (o.at(k) < lo.at(k) || o.at(k) > hi.at(k)) {
                return false;
            }
            continue;
        }
        double a = (lo.at(k) - o.at(k)) / d.at(k);
        double b = (hi.at(k) - o.at(k)) / d.at(k);
        if (a > b) {
            std::swap(a, b);
        }
        t0 = std::max(t0, a);
        t1 = std::min(t1, b);
        if (t0 >= t1) {
            return false;
        }
    }
    if (!std::isfinite(t1)) {
        return false;  // along an unbounded slab with no end: no range to bound it
    }
    enter = t0;
    leave = t1;
    return true;
}

double crownStop(const TreeSpec& tree, std::uint32_t index, const std::array<double, 3>& origin,
                 const std::array<double, 3>& direction, double range,
                 const std::array<double, 3>& sway, const TreeCuts* cuts, double u) {
    constexpr double kNever = std::numeric_limits<double>::infinity();
    const double mu = kCanopyProjection * tree.leafAreaDensity;
    if (!(mu > 0.0)) {
        return kNever;
    }
    // In the crown's frame, scaled to a unit sphere: |o + t d| = 1.
    const std::array<double, 3> o = inCrown(tree, origin, sway);
    const std::array<double, 3> d = rotate(conjugate(tree.pose.orientation), direction);
    const std::array<double, 3> scale{1.0 / tree.crownRadius, 1.0 / tree.crownRadius,
                                      1.0 / tree.halfHeight()};
    const std::array<double, 3> os{o[0] * scale[0], o[1] * scale[1], o[2] * scale[2]};
    const std::array<double, 3> ds{d[0] * scale[0], d[1] * scale[1], d[2] * scale[2]};
    const double a = ds[0] * ds[0] + ds[1] * ds[1] + ds[2] * ds[2];
    const double b = os[0] * ds[0] + os[1] * ds[1] + os[2] * ds[2];
    const double c = os[0] * os[0] + os[1] * os[1] + os[2] * os[2] - 1.0;
    const double disc = b * b - a * c;
    if (!(disc > 0.0) || !(a > 0.0)) {
        return kNever;
    }
    const double root = std::sqrt(disc);
    const double enter = std::max(0.0, (-b - root) / a);
    const double leave = std::min(range, (-b + root) / a);
    if (!(leave > enter)) {
        return kNever;
    }
    // The optical depth to reach: -ln(1 - u).
    const double need = -std::log1p(-std::clamp(u, 0.0, 1.0 - 1e-16));
    if (cuts == nullptr || cuts->empty()) {
        const double s = need / mu;
        return enter + s < leave ? enter + s : kNever;
    }
    const double step = 0.5 * kTreeCutCell;
    double depth = 0.0;
    for (double t = enter; t < leave; t += step) {
        const double ds2 = std::min(step, leave - t);
        const double mid = t + 0.5 * ds2;
        const std::array<double, 3> q{o[0] + mid * d[0], o[1] + mid * d[1], o[2] + mid * d[2]};
        if (cuts->cut(index, q)) {
            continue;
        }
        if (depth + mu * ds2 >= need) {
            return t + (need - depth) / mu;
        }
        depth += mu * ds2;
    }
    return kNever;
}

bool crownCrossed(const TreeSpec& tree, const std::array<double, 3>& origin,
                  const std::array<double, 3>& direction, double range,
                  const std::array<double, 3>& sway) {
    // crownStop's span, the same arithmetic.
    const std::array<double, 3> o = inCrown(tree, origin, sway);
    const std::array<double, 3> d = rotate(conjugate(tree.pose.orientation), direction);
    const std::array<double, 3> scale{1.0 / tree.crownRadius, 1.0 / tree.crownRadius,
                                      1.0 / tree.halfHeight()};
    const std::array<double, 3> os{o[0] * scale[0], o[1] * scale[1], o[2] * scale[2]};
    const std::array<double, 3> ds{d[0] * scale[0], d[1] * scale[1], d[2] * scale[2]};
    const double a = ds[0] * ds[0] + ds[1] * ds[1] + ds[2] * ds[2];
    const double b = os[0] * ds[0] + os[1] * ds[1] + os[2] * ds[2];
    const double c = os[0] * os[0] + os[1] * os[1] + os[2] * os[2] - 1.0;
    const double disc = b * b - a * c;
    if (!(disc > 0.0) || !(a > 0.0)) {
        return false;
    }
    const double root = std::sqrt(disc);
    const double enter = std::max(0.0, (-b - root) / a);
    const double leave = std::min(range, (-b + root) / a);
    return leave > enter;
}

std::array<double, 4> woodBound(const TreeSpec& tree, const Skeleton& wood,
                                const std::array<double, 3>& sway) {
    const std::vector<BranchSegment>& segs = wood.segments;
    const std::array<double, 3> c0 = segs.empty() ? std::array<double, 3>{} : segs[0].centre;
    double radius = 0.0;
    for (std::size_t i = 0; i < segs.size(); i = std::max<std::size_t>(segs[i].end, i + 1)) {
        radius = std::max(radius, std::hypot(segs[i].centre[0] - c0[0], segs[i].centre[1] - c0[1],
                                             segs[i].centre[2] - c0[2]) +
                                      segs[i].radius);
    }
    const std::array<double, 3> up =
        rotate(tree.pose.orientation, {c0[0], c0[1], c0[2] + tree.centreHeight()});
    return {tree.pose.position[0] + up[0] + sway[0], tree.pose.position[1] + up[1] + sway[1],
            tree.pose.position[2] + up[2] + sway[2], radius};
}

void CrownIndex::build(std::span<const TreeSpec> trees,
                       std::span<const std::array<double, 3>> sways) {
    // Each crown's bounding sphere, at its sway.
    std::vector<std::array<double, 4>> spheres(trees.size());
    for (std::size_t k = 0; k < trees.size(); ++k) {
        const TreeSpec& t = trees[k];
        const std::array<double, 3> up = rotate(t.pose.orientation, {0.0, 0.0, t.centreHeight()});
        const std::array<double, 3> s = k < sways.size() ? sways[k] : std::array<double, 3>{};
        spheres[k] = {t.pose.position[0] + up[0] + s[0], t.pose.position[1] + up[1] + s[1],
                      t.pose.position[2] + up[2] + s[2], std::max(t.crownRadius, t.halfHeight())};
    }
    buildSpheres(spheres);
}

void CrownIndex::buildSpheres(std::span<const std::array<double, 4>> given) {
    items_.clear();
    start_.clear();
    nx_ = 0;
    ny_ = 0;
    if (given.empty()) {
        return;
    }
    constexpr double kInf = std::numeric_limits<double>::infinity();
    std::vector<std::array<double, 4>> spheres(given.begin(), given.end());
    lo_ = {kInf, kInf, kInf};
    hi_ = {-kInf, -kInf, -kInf};
    double diameters = 0.0;
    for (std::array<double, 4>& sphere : spheres) {
        // A little over, so rounding never leaves a crossing tree out.
        const double r = (sphere[3] * (1.0 + 1e-9)) + 1e-6;
        sphere[3] = r;
        for (std::size_t c = 0; c < 3; ++c) {
            lo_.at(c) = std::min(lo_.at(c), sphere.at(c) - r);
            hi_.at(c) = std::max(hi_.at(c), sphere.at(c) + r);
        }
        diameters += 2.0 * r;
    }
    // Cells about a crown across: a crown in at most four, a few crowns a cell in an orchard.
    cell_ = std::max(diameters / static_cast<double>(spheres.size()), 0.5);
    constexpr std::int64_t kMostCellsASide = 1024;
    cell_ =
        std::max({cell_, (hi_[0] - lo_[0]) / kMostCellsASide, (hi_[1] - lo_[1]) / kMostCellsASide});
    nx_ =
        std::max<std::int64_t>(1, static_cast<std::int64_t>(std::ceil((hi_[0] - lo_[0]) / cell_)));
    ny_ =
        std::max<std::int64_t>(1, static_cast<std::int64_t>(std::ceil((hi_[1] - lo_[1]) / cell_)));
    const auto cellOf = [&](double v, double lo, std::int64_t n) {
        return std::clamp(static_cast<std::int64_t>(std::floor((v - lo) / cell_)), std::int64_t{0},
                          n - 1);
    };
    std::vector<std::uint32_t> counts(static_cast<std::size_t>(nx_ * ny_) + 1, 0);
    const auto each = [&](auto&& visit) {
        for (std::size_t k = 0; k < spheres.size(); ++k) {
            const std::array<double, 4>& s = spheres[k];
            for (std::int64_t y = cellOf(s[1] - s[3], lo_[1], ny_);
                 y <= cellOf(s[1] + s[3], lo_[1], ny_); ++y) {
                for (std::int64_t x = cellOf(s[0] - s[3], lo_[0], nx_);
                     x <= cellOf(s[0] + s[3], lo_[0], nx_); ++x) {
                    visit(static_cast<std::size_t>(y * nx_ + x), static_cast<std::uint32_t>(k));
                }
            }
        }
    };
    each([&](std::size_t cell, std::uint32_t) { ++counts[cell + 1]; });
    for (std::size_t c = 1; c < counts.size(); ++c) {
        counts[c] += counts[c - 1];
    }
    start_ = counts;
    items_.resize(start_.back());
    each([&](std::size_t cell, std::uint32_t k) { items_[counts[cell]++] = k; });
}

void CrownIndex::candidates(const std::array<double, 3>& origin,
                            const std::array<double, 3>& direction, double range,
                            std::vector<std::uint32_t>& out) const {
    out.clear();
    if (items_.empty()) {
        return;
    }
    // The beam's segment clipped to the box of every crown (slabs).
    double t0 = 0.0;
    double t1 = range;
    for (std::size_t c = 0; c < 3; ++c) {
        if (std::abs(direction.at(c)) < 1e-15) {
            if (origin.at(c) < lo_.at(c) || origin.at(c) > hi_.at(c)) {
                return;
            }
            continue;
        }
        double a = (lo_.at(c) - origin.at(c)) / direction.at(c);
        double b = (hi_.at(c) - origin.at(c)) / direction.at(c);
        if (a > b) {
            std::swap(a, b);
        }
        t0 = std::max(t0, a);
        t1 = std::min(t1, b);
        if (!(t0 <= t1)) {
            return;
        }
    }
    // A 2D DDA over the cells from t0 to t1 (Amanatides and Woo 1987).
    const double px = origin[0] + (direction[0] * t0);
    const double py = origin[1] + (direction[1] * t0);
    std::int64_t x = std::clamp(static_cast<std::int64_t>(std::floor((px - lo_[0]) / cell_)),
                                std::int64_t{0}, nx_ - 1);
    std::int64_t y = std::clamp(static_cast<std::int64_t>(std::floor((py - lo_[1]) / cell_)),
                                std::int64_t{0}, ny_ - 1);
    constexpr double kInf = std::numeric_limits<double>::infinity();
    const auto axis = [&](double d, double o, double lo, std::int64_t at, std::int64_t& step,
                          double& next, double& delta) {
        if (std::abs(d) < 1e-15) {
            step = 0;
            next = kInf;
            delta = kInf;
            return;
        }
        step = d > 0.0 ? 1 : -1;
        const double edge = lo + (static_cast<double>(at + (d > 0.0 ? 1 : 0)) * cell_);
        next = (edge - o) / d;
        delta = cell_ / std::abs(d);
    };
    std::int64_t sx = 0;
    std::int64_t sy = 0;
    double nextX = 0.0;
    double nextY = 0.0;
    double dX = 0.0;
    double dY = 0.0;
    axis(direction[0], origin[0], lo_[0], x, sx, nextX, dX);
    axis(direction[1], origin[1], lo_[1], y, sy, nextY, dY);
    for (;;) {
        const auto cell = static_cast<std::size_t>(y * nx_ + x);
        out.insert(out.end(), items_.begin() + start_[cell], items_.begin() + start_[cell + 1]);
        if (nextX <= nextY) {
            if (nextX > t1) {
                break;
            }
            x += sx;
            nextX += dX;
        } else {
            if (nextY > t1) {
                break;
            }
            y += sy;
            nextY += dY;
        }
        if (x < 0 || x >= nx_ || y < 0 || y >= ny_) {
            break;
        }
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
}

void FlyingLeaves::load(std::span<const double> values) {
    if (values.size() < kLeavesHeader || values[0] == version_) {
        return;
    }
    version_ = values[0];
    leaves_.clear();
    const auto count = std::min(static_cast<std::size_t>(std::max(0.0, values[1])),
                                (values.size() - kLeavesHeader) / kLeavesFields);
    std::array<double, 3> lo{std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::infinity()};
    std::array<double, 3> hi{-lo[0], -lo[1], -lo[2]};
    double largest = 0.0;
    for (std::size_t k = 0; k < count; ++k) {
        const std::size_t at = kLeavesHeader + (kLeavesFields * k);
        const double radius = 0.5 * values[at + 5];
        if (!(radius > 0.0)) {
            continue;
        }
        leaves_.push_back({values[at], values[at + 1], values[at + 2], radius});
        largest = std::max(largest, radius);
        for (std::size_t c = 0; c < 3; ++c) {
            lo.at(c) = std::min(lo.at(c), values[at + c]);
            hi.at(c) = std::max(hi.at(c), values[at + c]);
        }
    }
    if (leaves_.empty()) {
        return;
    }
    centre_ = {0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]), 0.5 * (lo[2] + hi[2])};
    radius_ = (0.5 * std::hypot(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])) + largest;
}

double FlyingLeaves::stop(const std::array<double, 3>& origin,
                          const std::array<double, 3>& direction, double range) const {
    constexpr double kNever = std::numeric_limits<double>::infinity();
    const auto meets = [&](const std::array<double, 3>& c, double r, double& along) {
        const std::array<double, 3> d{c[0] - origin[0], c[1] - origin[1], c[2] - origin[2]};
        along = d[0] * direction[0] + d[1] * direction[1] + d[2] * direction[2];
        const double across = (d[0] * d[0]) + (d[1] * d[1]) + (d[2] * d[2]) - (along * along);
        return across <= r * r;
    };
    double along = 0.0;
    if (leaves_.empty() || !meets(centre_, radius_, along) || along + radius_ < 0.0 ||
        along - radius_ > range) {
        return kNever;
    }
    double best = kNever;
    for (const std::array<double, 4>& leaf : leaves_) {
        if (meets({leaf[0], leaf[1], leaf[2]}, leaf[3], along) && along >= 0.0 && along <= range &&
            along < best) {
            best = along;
        }
    }
    return best;
}

double canopyVolumeStop(double enter, double leave, double canopyHeight, double leafAreaIndex,
                        double u) {
    if (!(canopyHeight > 0.0) || !(leafAreaIndex > 0.0) || !(leave > enter)) {
        return std::numeric_limits<double>::infinity();
    }
    const double mu = kCanopyProjection * leafAreaIndex / canopyHeight;
    const double s = -std::log1p(-std::clamp(u, 0.0, 1.0 - 1e-16)) / mu;
    return enter + s < leave ? enter + s : std::numeric_limits<double>::infinity();
}

namespace {
// Value noise on a unit lattice, smoothly interpolated, in [0, 1]: a pure function of the
// position and a seed (no state), so the same plants stand on every run and every tier.
double latticeValue(std::int64_t i, std::int64_t j, std::uint32_t seed) {
    auto h = static_cast<std::uint32_t>(i) * 0x8DA6B343U ^
             static_cast<std::uint32_t>(j) * 0xD8163841U ^ seed * 0xCB1AB31FU;
    h ^= h >> 16U;  // a 32-bit finaliser (Wang/Jenkins style mix)
    h *= 0x7FEB352DU;
    h ^= h >> 15U;
    h *= 0x846CA68BU;
    h ^= h >> 16U;
    return static_cast<double>(h) / 4294967295.0;
}

double valueNoise(double x, double y, std::uint32_t seed) {
    const double fx = std::floor(x);
    const double fy = std::floor(y);
    const auto i = static_cast<std::int64_t>(fx);
    const auto j = static_cast<std::int64_t>(fy);
    const double tx = x - fx;
    const double ty = y - fy;
    const double sx = tx * tx * (3.0 - 2.0 * tx);
    const double sy = ty * ty * (3.0 - 2.0 * ty);
    const double a = latticeValue(i, j, seed);
    const double b = latticeValue(i + 1, j, seed);
    const double c = latticeValue(i, j + 1, seed);
    const double d = latticeValue(i + 1, j + 1, seed);
    return (a + (b - a) * sx) + ((c + (d - c) * sx) - (a + (b - a) * sx)) * sy;
}

// Two octaves, the second at 2.3x the frequency and half the weight, renormalised to [0, 1].
double patchNoise(double x, double y, std::uint32_t seed) {
    constexpr double kLacunarity = 2.3;
    return (2.0 * valueNoise(x, y, seed) +
            valueNoise(x * kLacunarity + 17.1, y * kLacunarity - 3.7, seed + 1U)) /
           3.0;
}

double smoothstep(double a, double b, double x) {
    const double t = std::clamp((x - a) / (b - a), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

constexpr std::uint32_t kHeightSeed = 0x51U;
constexpr std::uint32_t kGapSeed = 0x6BU;
constexpr std::uint32_t kEdgeSeed = 0x7DU;
constexpr std::uint32_t kStemSeed = 0x8FU;
}  // namespace

CanopyFormShape canopyFormOf(CanopyForm form) {
    // TODO(verify) all: sizes of tufts, clumps and bushes and how much they vary, from field
    // photographs, not measurements. Grass: tufts a few decimetres across, a few bare spots.
    // Reeds: Phragmites stands in clumps about a metre across with lanes between. Shrubs:
    // bushes a metre or two across with more ground between them.
    switch (form) {
        case CanopyForm::Reeds:
            return {1.2, 0.35, 0.12, 0.6, 0.12, 0.25};
        case CanopyForm::Shrub:
            return {1.5, 0.3, 0.3, 0.5, 0.2, 0.15};
        case CanopyForm::Grass:
        default:
            return {0.25, 0.35, 0.08, 0.15, 0.04, 0.25};
    }
}

double canopyLeafProfile(CanopyForm form, double zeta) {
    if (!(zeta >= 0.0) || !(zeta <= 1.0)) {
        return 0.0;
    }
    // Leaf area density profiles of the kinds measured in crops and stands (e.g. Lalic and
    // Mihailovic 2004, J. Appl. Meteor. 43, an empirical profile with its maximum in the upper
    // part of the canopy); the shapes here are simple polynomials, TODO(verify) each against a
    // measured profile of the plant.
    switch (form) {
        case CanopyForm::Reeds:  // stems throughout (0.3), leaves peaking at 2/3 of the height
            return 0.3 + 0.7 * 12.0 * zeta * zeta * (1.0 - zeta);
        case CanopyForm::Shrub:  // bare stems (0.1), a crown peaking at 3/4 of the height
            return 0.1 + 0.9 * 20.0 * zeta * zeta * zeta * (1.0 - zeta);
        case CanopyForm::Grass:  // dense at the base, thinning to the blade tips
        default:
            return 1.5 * (1.0 - zeta * zeta);
    }
}

double canopyHeightFactor(const SurfaceMaterial& material, double x, double y,
                          double edgeDistance) {
    const CanopyFormShape f = canopyFormOf(material.canopyForm);
    const double u = x / f.patch;
    const double v = y / f.patch;
    const double height =
        (1.0 + f.spread * (2.0 * patchNoise(u, v, kHeightSeed) - 1.0)) *
        (1.0 + f.jitter * (2.0 * valueNoise(x / f.stem, y / f.stem, kStemSeed) - 1.0));
    constexpr double kGapRamp = 0.12;  // how sharply the plants end at a bare patch
    const double ground = smoothstep(f.gap, f.gap + kGapRamp, patchNoise(u, v, kGapSeed));
    double border = 1.0;
    if (std::isfinite(edgeDistance)) {
        constexpr double kBorderRamp = 0.3;  // of the edge length: how soft the border is
        const double limit = f.edge * patchNoise(u * 1.7, v * 1.7, kEdgeSeed);
        border = smoothstep(0.0, kBorderRamp * f.edge, edgeDistance - limit);
    }
    return height * ground * border;
}

double canopyTallest(const SurfaceMaterial& material) {
    const CanopyFormShape f = canopyFormOf(material.canopyForm);
    return material.canopyHeight * (1.0 + f.spread) * (1.0 + f.jitter);
}

double canopyShapedStop(const SurfaceMaterial& material, ShapeType type,
                        const std::array<double, 3>& size, const Pose& placed,
                        const std::array<double, 3>& origin, const std::array<double, 3>& direction,
                        double enter, double leave, double u, const CanopyCuts* cuts,
                        std::uint32_t collider) {
    constexpr double kNever = std::numeric_limits<double>::infinity();
    const double big = material.canopyHeight;
    if (!(big > 0.0) || !(material.leafAreaIndex > 0.0) || !(leave > enter)) {
        return kNever;
    }
    const std::array<double, 4> back = conjugate(placed.orientation);
    const std::array<double, 3> o =
        rotate(back, {origin[0] - placed.position[0], origin[1] - placed.position[1],
                      origin[2] - placed.position[2]});
    const std::array<double, 3> d = rotate(back, direction);
    const bool box = type == ShapeType::Box;
    const double soil = box ? size[2] : 0.0;
    const CanopyFormShape form = canopyFormOf(material.canopyForm);
    // Steps: a twelfth of the height up or down, half a stem across, at most 4096.
    constexpr double kVerticalSteps = 12.0;
    constexpr double kAcrossSteps = 2.0;
    constexpr double kMaxSteps = 4096.0;
    const double across = std::hypot(d[0], d[1]);
    double dt = std::numeric_limits<double>::infinity();
    if (std::abs(d[2]) > 1e-12) {
        dt = std::min(dt, canopyTallest(material) / kVerticalSteps / std::abs(d[2]));
    }
    if (across > 1e-12) {
        dt = std::min(dt, form.stem / kAcrossSteps / across);
    }
    dt = std::max(dt, (leave - enter) / kMaxSteps);
    const double target = -std::log1p(-std::clamp(u, 0.0, 1.0 - 1e-16));
    const double density = kCanopyProjection * material.leafAreaIndex / big;  // G LAI / H
    double depth = 0.0;
    for (double t0 = enter; t0 < leave; t0 += dt) {
        double t = t0;
        double step = std::min(dt, leave - t0);
        // Cut plants (ground.md "Cutting the plants"): only the part of the step under the cut
        // at its middle counts, so no beam stops above the cut.
        if (cuts != nullptr) {
            const double mid = t0 + 0.5 * step;
            const double top = cuts->top(collider, o[0] + d[0] * mid, o[1] + d[1] * mid);
            if (std::isfinite(top) && std::abs(d[2]) > 1e-12) {
                const double at = (soil + top - o[2]) / d[2];  // where the beam meets the cut
                double lo = t0;
                double hi = t0 + step;
                if (d[2] < 0.0) {
                    lo = std::max(lo, at);  // going down: under the cut after it
                } else {
                    hi = std::min(hi, at);  // going up: under it before it
                }
                if (!(hi > lo)) {
                    continue;
                }
                t = lo;
                step = hi - lo;
            }
        }
        const double m = t + 0.5 * step;
        const double x = o[0] + d[0] * m;
        const double y = o[1] + d[1] * m;
        const double edgeDistance =
            box ? std::min(size[0] - std::abs(x), size[1] - std::abs(y)) : kNever;
        const double factor = canopyHeightFactor(material, x, y, edgeDistance);
        if (!(factor > 0.0)) {
            continue;
        }
        // Leaf area density here: (LAI f / (H f)) p(zeta) = (LAI / H) p(zeta); none above a cut.
        const double over = o[2] + d[2] * m - soil;
        if (cuts != nullptr && over > cuts->top(collider, x, y)) {
            continue;
        }
        const double zeta = over / (big * factor);
        const double a = density * canopyLeafProfile(material.canopyForm, zeta);
        if (a > 0.0 && depth + a * step >= target) {
            return t + (target - depth) / a;
        }
        depth += a * step;
    }
    return kNever;
}

double canopyShapedShortening(const SurfaceMaterial& material, double x, double y,
                              double cosIncidence, double range, double u, double cutTop) {
    const double big = material.canopyHeight;
    if (!(big > 0.0) || !(material.leafAreaIndex > 0.0) || !std::isfinite(range)) {
        return 0.0;
    }
    const double h =
        big * canopyHeightFactor(material, x, y, std::numeric_limits<double>::infinity());
    if (!(h > 0.0)) {
        return 0.0;
    }
    constexpr double kMinCosine = 1e-3;
    const double c = std::max(std::abs(cosIncidence), kMinCosine);
    const double stand = std::min(h, std::max(cutTop, 0.0));  // the plants' top, cut or not
    if (!(stand > 0.0)) {
        return 0.0;
    }
    const double path = std::min(stand / c, std::max(range, 0.0));
    // Along the beam from where it enters the canopy (zeta = 1 when the path is the full h / c);
    // the profile is integrated in 32 midpoint steps.
    constexpr int kSteps = 32;
    const double dt = path / kSteps;
    const double target = -std::log1p(-std::clamp(u, 0.0, 1.0 - 1e-16));
    const double density = kCanopyProjection * material.leafAreaIndex / big;
    const double top = path * c;  // the height over the soil where the path starts
    double depth = 0.0;
    for (int k = 0; k < kSteps; ++k) {
        const double s = (k + 0.5) * dt;
        const double a = density * canopyLeafProfile(material.canopyForm, (top - s * c) / h);
        if (a > 0.0 && depth + a * dt >= target) {
            return path - (k * dt + (target - depth) / a);
        }
        depth += a * dt;
    }
    return 0.0;
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
