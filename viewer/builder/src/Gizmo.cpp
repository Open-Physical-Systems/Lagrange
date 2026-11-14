// The Builder view's gizmos as geometry (RFC 0016; docs/design/ui/12-builder/SPEC.md "Builder
// view"). See Gizmo.hpp.

#include "lagrange/viewer/builder/Gizmo.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>

namespace lagrange::viewer::builder::gizmo {

namespace {

/// Below this |sin| a ray counts as parallel to an axis or a plane (no stable answer).
constexpr double kParallel = 1e-6;
/// rpy within this (rad) of a snap step is put on it (float noise of the quaternion round trip).
constexpr double kAngleClean = 1e-9;

double dot(const Vec3& a, const Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
Vec3 sub(const Vec3& a, const Vec3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
Vec3 add(const Vec3& a, const Vec3& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}
Vec3 scale(const Vec3& a, double s) {
    return {a[0] * s, a[1] * s, a[2] * s};
}
double length(const Vec3& a) {
    return std::sqrt(dot(a, a));
}
Quat conjugate(const Quat& q) {
    return {q[0], -q[1], -q[2], -q[3]};
}
Quat multiply(const Quat& a, const Quat& b) {
    return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}
Vec3 unit(int axis) {
    Vec3 v{};
    v.at(static_cast<std::size_t>(axis)) = 1.0;
    return v;
}

/// The nearest t >= 0 at which a ray in a box's frame enters it (half extents `half`).
std::optional<double> hitBox(const Vec3& o, const Vec3& d, const Vec3& half) {
    double near = 0.0;
    double far = std::numeric_limits<double>::infinity();
    for (std::size_t k = 0; k < 3; ++k) {
        if (std::abs(d.at(k)) < kParallel) {
            if (std::abs(o.at(k)) > half.at(k)) {
                return std::nullopt;
            }
            continue;
        }
        double a = (-half.at(k) - o.at(k)) / d.at(k);
        double b = (half.at(k) - o.at(k)) / d.at(k);
        if (a > b) {
            std::swap(a, b);
        }
        near = std::max(near, a);
        far = std::min(far, b);
        if (near > far) {
            return std::nullopt;
        }
    }
    return near;
}

std::optional<double> hitSphere(const Vec3& o, const Vec3& d, double r) {
    const double b = dot(o, d);
    const double c = dot(o, o) - r * r;
    const double disc = b * b - c;
    if (disc < 0.0) {
        return std::nullopt;
    }
    const double root = std::sqrt(disc);
    const double t = -b - root >= 0.0 ? -b - root : -b + root;
    if (t < 0.0) {
        return std::nullopt;
    }
    return t;
}

/// A cylinder of radius r along z from -h to h.
std::optional<double> hitCylinder(const Vec3& o, const Vec3& d, double r, double h) {
    std::optional<double> best;
    const auto consider = [&](double t) {
        if (t >= 0.0 && (!best || t < *best)) {
            best = t;
        }
    };
    const double a = d[0] * d[0] + d[1] * d[1];
    if (a > kParallel * kParallel) {
        const double b = o[0] * d[0] + o[1] * d[1];
        const double c = o[0] * o[0] + o[1] * o[1] - r * r;
        const double disc = b * b - a * c;
        if (disc >= 0.0) {
            const double root = std::sqrt(disc);
            for (const double t : {(-b - root) / a, (-b + root) / a}) {
                if (std::abs(o[2] + t * d[2]) <= h) {
                    consider(t);
                }
            }
        }
    }
    if (std::abs(d[2]) > kParallel) {
        for (const double cap : {-h, h}) {
            const double t = (cap - o[2]) / d[2];
            const double x = o[0] + t * d[0];
            const double y = o[1] + t * d[1];
            if (x * x + y * y <= r * r) {
                consider(t);
            }
        }
    }
    return best;
}

void appendNumber(std::string& out, double value) {
    char text[32];
    std::snprintf(text, sizeof(text), "%.9g", value == 0.0 ? 0.0 : value);
    out += text;
}

void appendVec(std::string& out, const std::array<double, 3>& v) {
    out += '[';
    for (std::size_t k = 0; k < 3; ++k) {
        if (k > 0) {
            out += ", ";
        }
        appendNumber(out, v.at(k));
    }
    out += ']';
}

void appendPose(std::string& out, const Pose& pose) {
    out += "\"xyz\": ";
    appendVec(out, pose.xyz);
    out += ", \"rpy\": ";
    appendVec(out, pose.rpy);
}

void appendName(std::string& out, const std::string& name) {
    out += '"';
    for (const char c : name) {
        if (c == '"' || c == '\\') {
            out += '\\';
        }
        out += static_cast<unsigned char>(c) < 0x20 ? ' ' : c;
    }
    out += '"';
}

}  // namespace

double snap(double value, double step) {
    if (step <= 0.0) {
        return value;
    }
    const double snapped = std::round(value / step) * step;
    return snapped == 0.0 ? 0.0 : snapped;
}

std::optional<double> alongAxis(const Line& ray, const Vec3& point, const Vec3& axis) {
    // Closest points of two lines: point + t·axis and ray.origin + s·ray.direction.
    const Vec3 w = sub(point, ray.origin);
    const double b = dot(axis, ray.direction);
    const double denominator = 1.0 - b * b;
    if (denominator < kParallel) {
        return std::nullopt;
    }
    const double d = dot(axis, w);
    const double e = dot(ray.direction, w);
    return (b * e - d) / denominator;
}

std::optional<Vec3> onPlane(const Line& ray, const Vec3& point, const Vec3& normal) {
    const double facing = dot(ray.direction, normal);
    if (std::abs(facing) < kParallel) {
        return std::nullopt;
    }
    const double t = dot(sub(point, ray.origin), normal) / facing;
    if (t < 0.0) {
        return std::nullopt;
    }
    return add(ray.origin, scale(ray.direction, t));
}

std::optional<double> angleOnRing(const Line& ray, const Vec3& centre, const Vec3& axis,
                                  const Vec3& reference) {
    const std::optional<Vec3> hit = onPlane(ray, centre, axis);
    if (!hit) {
        return std::nullopt;
    }
    const Vec3 v = sub(*hit, centre);
    // Reference and its quarter turn about the axis span the ring's plane.
    const Vec3 u = sub(reference, scale(axis, dot(reference, axis)));
    const double ul = length(u);
    if (ul < kParallel || length(v) < kParallel) {
        return std::nullopt;
    }
    const Vec3 x = scale(u, 1.0 / ul);
    const Vec3 y = cross(axis, x);
    return std::atan2(dot(v, y), dot(v, x));
}

Vec3 rotate(const Quat& rotation, const Vec3& v) {
    const Quat p{0.0, v[0], v[1], v[2]};
    const Quat r = multiply(multiply(rotation, p), conjugate(rotation));
    return {r[1], r[2], r[3]};
}

Vec3 rpyOf(const Quat& q) {
    const double w = q[0];
    const double x = q[1];
    const double y = q[2];
    const double z = q[3];
    Vec3 rpy{std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)),
             std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0)),
             std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))};
    for (double& angle : rpy) {
        const double on = snap(angle, kAngleStep);
        if (std::abs(angle - on) < kAngleClean) {
            angle = on;
        }
    }
    return rpy;
}

Pose* poseOf(Document& document, const Target& target) {
    return const_cast<Pose*>(poseOf(static_cast<const Document&>(document), target));
}

const Pose* poseOf(const Document& document, const Target& target) {
    switch (target.kind) {
        case Target::Kind::JointOrigin: {
            const Joint* joint = findJoint(document, target.name);
            return joint != nullptr ? &joint->origin : nullptr;
        }
        case Target::Kind::Shape: {
            const Link* link = findLink(document, target.name);
            if (link == nullptr || target.shape < 0 ||
                static_cast<std::size_t>(target.shape) >= link->shapes.size()) {
                return nullptr;
            }
            return &link->shapes[static_cast<std::size_t>(target.shape)].origin;
        }
        case Target::Kind::Sensor: {
            const Sensor* sensor = findSensor(document, target.name);
            return sensor != nullptr ? &sensor->origin : nullptr;
        }
        case Target::Kind::None:
            break;
    }
    return nullptr;
}

std::optional<Frame> parentFrame(const Document& document, const std::vector<Frame>& links,
                                 const Target& target) {
    std::string_view link;
    switch (target.kind) {
        case Target::Kind::JointOrigin: {
            const Joint* joint = findJoint(document, target.name);
            if (joint == nullptr) {
                return std::nullopt;
            }
            link = joint->parent;
            break;
        }
        case Target::Kind::Shape:
            link = target.name;
            break;
        case Target::Kind::Sensor: {
            const Sensor* sensor = findSensor(document, target.name);
            if (sensor == nullptr) {
                return std::nullopt;
            }
            link = sensor->link;
            break;
        }
        case Target::Kind::None:
            return std::nullopt;
    }
    for (std::size_t l = 0; l < document.links.size() && l < links.size(); ++l) {
        if (document.links[l].name == link) {
            return links[l];
        }
    }
    return std::nullopt;
}

Pose moved(const Pose& start, int axis, double delta, double step) {
    Pose out = start;
    double& value = out.xyz.at(static_cast<std::size_t>(axis));
    value = snap(value + delta, step);
    return out;
}

Pose rotated(const Pose& start, int axis, double angle, double step) {
    const double turn = snap(angle, step);
    if (turn == 0.0) {
        return start;
    }
    const Vec3 a = unit(axis);
    const double s = std::sin(turn * 0.5);
    const Quat about{std::cos(turn * 0.5), a[0] * s, a[1] * s, a[2] * s};
    Pose out = start;
    out.rpy = rpyOf(multiply(about, toFrame(start).rotation));
    return out;
}

std::optional<double> extentOf(const Shape& shape, int axis) {
    if (axis < 0 || axis > 2) {
        return std::nullopt;
    }
    switch (shape.kind) {
        case Shape::Kind::Box:
            return shape.size.at(static_cast<std::size_t>(axis));
        case Shape::Kind::Cylinder:
            return axis == 2 ? shape.size[1] : 2.0 * shape.size[0];
        case Shape::Kind::Sphere:
            return 2.0 * shape.size[0];
        default:  // a mesh has no size of its own; other kinds are not sized here
            return std::nullopt;
    }
}

bool resize(Shape& shape, int axis, int sign, double delta, double step) {
    const std::optional<double> before = extentOf(shape, axis);
    if (!before) {
        return false;
    }
    const double wanted = *before + delta;
    // The value the file holds: an edge length, or a radius (half the extent).
    const bool radius =
        shape.kind == Shape::Kind::Sphere || (shape.kind == Shape::Kind::Cylinder && axis != 2);
    const double value = std::max(kMinSize, snap(radius ? wanted * 0.5 : wanted, step));
    double after = value;
    if (shape.kind == Shape::Kind::Box) {
        shape.size.at(static_cast<std::size_t>(axis)) = value;
    } else if (radius) {
        shape.size[0] = value;
        after = 2.0 * value;
    } else {
        shape.size[1] = value;
    }
    // The opposite face stays: the centre goes half the change towards the dragged face, along
    // the shape's own axis turned into the link frame.
    const Vec3 shift =
        rotate(toFrame(shape.origin).rotation, scale(unit(axis), sign * (after - *before) * 0.5));
    shape.origin.xyz = add(shape.origin.xyz, shift);
    return true;
}

Vec3 snapAxis(const Vec3& direction, const Quat& originRotation, bool free) {
    const double norm = length(direction);
    if (norm < kParallel) {
        return {1, 0, 0};
    }
    const Vec3 d = scale(direction, 1.0 / norm);
    if (free) {
        return d;
    }
    Vec3 best{1, 0, 0};
    double bestDot = -2.0;
    const Quat toJoint = conjugate(originRotation);
    for (int axis = 0; axis < 3; ++axis) {
        for (const double sign : {1.0, -1.0}) {
            // The joint frame's own axis, and the parent link's axis seen from the joint frame.
            const Vec3 own = scale(unit(axis), sign);
            for (const Vec3& candidate : {own, rotate(toJoint, own)}) {
                const double along = dot(candidate, d);
                if (along > bestDot + 1e-12) {
                    bestDot = along;
                    best = candidate;
                }
            }
        }
    }
    for (double& component : best) {
        component = std::abs(component) < kAngleClean ? 0.0 : component;
    }
    return best;
}

std::optional<ShapeHit> pickShape(const Document& document, const std::vector<Frame>& links,
                                  const Line& ray, bool collisionShown) {
    std::optional<ShapeHit> best;
    for (std::size_t l = 0; l < document.links.size() && l < links.size(); ++l) {
        const Link& link = document.links[l];
        for (std::size_t s = 0; s < link.shapes.size(); ++s) {
            const Shape& shape = link.shapes[s];
            if (!shape.visual && !(shape.collision && collisionShown)) {
                continue;
            }
            const Frame frame = compose(links[l], toFrame(shape.origin));
            const Quat back = conjugate(frame.rotation);
            const Vec3 o = rotate(back, sub(ray.origin, frame.position));
            const Vec3 d = rotate(back, ray.direction);
            std::optional<double> t;
            switch (shape.kind) {
                case Shape::Kind::Box:
                    t = hitBox(o, d, scale(shape.size, 0.5));
                    break;
                case Shape::Kind::Cylinder:
                    t = hitCylinder(o, d, shape.size[0], shape.size[1] * 0.5);
                    break;
                case Shape::Kind::Sphere:
                    t = hitSphere(o, d, shape.size[0]);
                    break;
                default:
                    break;
            }
            if (t && (!best || *t < best->distance)) {
                best = ShapeHit{static_cast<int>(l), static_cast<int>(s), *t,
                                add(ray.origin, scale(ray.direction, *t))};
            }
        }
    }
    return best;
}

float distanceToSegment(const std::array<float, 2>& p, const std::array<float, 2>& a,
                        const std::array<float, 2>& b) {
    const float abx = b[0] - a[0];
    const float aby = b[1] - a[1];
    const float apx = p[0] - a[0];
    const float apy = p[1] - a[1];
    const float ab2 = abx * abx + aby * aby;
    const float t = ab2 > 0.0F ? std::clamp((apx * abx + apy * aby) / ab2, 0.0F, 1.0F) : 0.0F;
    const float dx = apx - t * abx;
    const float dy = apy - t * aby;
    return std::sqrt(dx * dx + dy * dy);
}

std::string geometryJson(const Document& document) {
    std::string out = "{\"links\": {";
    for (std::size_t l = 0; l < document.links.size(); ++l) {
        const Link& link = document.links[l];
        out += l > 0 ? ", " : "";
        appendName(out, link.name);
        out += ": [";
        for (std::size_t s = 0; s < link.shapes.size(); ++s) {
            const Shape& shape = link.shapes[s];
            out += s > 0 ? ", {\"kind\": \"" : "{\"kind\": \"";
            out += std::string(toString(shape.kind)) + "\", \"size\": ";
            appendVec(out, shape.size);
            out += ", ";
            appendPose(out, shape.origin);
            out += '}';
        }
        out += ']';
    }
    out += "}, \"joints\": {";
    for (std::size_t j = 0; j < document.joints.size(); ++j) {
        const Joint& joint = document.joints[j];
        out += j > 0 ? ", " : "";
        appendName(out, joint.name);
        out += ": {";
        appendPose(out, joint.origin);
        out += ", \"axis\": ";
        appendVec(out, joint.axis);
        out += '}';
    }
    out += "}, \"sensors\": {";
    for (std::size_t s = 0; s < document.sensors.size(); ++s) {
        out += s > 0 ? ", " : "";
        appendName(out, document.sensors[s].name);
        out += ": {";
        appendPose(out, document.sensors[s].origin);
        out += '}';
    }
    out += "}}";
    return out;
}

}  // namespace lagrange::viewer::builder::gizmo
