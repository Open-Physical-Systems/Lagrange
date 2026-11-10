#include "lagrange/tether/Cable.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lagrange::tether {

namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();
// Near an end, a mass does not touch a shape that holds that end (the drone's hull around its
// attach point, a ground station's box around its outlet): the cable starts inside them.
// It does not touch the robot an end is tied to there either: a drone's arms and rotors around
// its attach point.
constexpr double kEndClearanceM = 0.5;  // m of cable from either end
// A mass found deep inside a shape leaves it at most this fast along its normal: a cable laid
// through something (or a body moved into it) comes out over a few steps, not in one substep with
// a force of thousands of newtons. A resting or arriving mass is stopped exactly (its own inward
// move of the substep is always undone).
constexpr double kMaxDepenetrationMps = 1.0;
// The winch: m/s paid out per N of tension over its set point, and a free drum's drag.
constexpr double kWinchGain = 0.5;       // (m/s)/N
constexpr double kWinchIntegral = 2.0;   // (m/s)/(N·s): a tension winch keeps its set point
constexpr double kFreeDrumN = 0.2;       // N
constexpr double kAnchorFilterS = 0.05;  // s, the winch's view of the anchor tension
constexpr double kMinLengthM = 0.05;     // m, the shortest a one-segment cable reels in to
constexpr int kLayoutSamples = 400;      // polyline points of the starting sag

Vec3 add(const Vec3& a, const Vec3& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}
Vec3 sub(const Vec3& a, const Vec3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
Vec3 scale(const Vec3& a, double s) {
    return {a[0] * s, a[1] * s, a[2] * s};
}
double dot(const Vec3& a, const Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
double norm(const Vec3& a) {
    return std::sqrt(dot(a, a));
}
Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
void addTo(Vec3& a, const Vec3& b) {
    a[0] += b[0];
    a[1] += b[1];
    a[2] += b[2];
}

/// Closest points of segments a0-a1 and b0-b1: the parameters s and t along each (Ericson 2005,
/// "Real-Time Collision Detection", 5.1.9).
void closestOnSegments(const Vec3& a0, const Vec3& a1, const Vec3& b0, const Vec3& b1, double& s,
                       double& t) {
    const Vec3 d1 = sub(a1, a0);
    const Vec3 d2 = sub(b1, b0);
    const Vec3 r = sub(a0, b0);
    const double a = dot(d1, d1);
    const double e = dot(d2, d2);
    const double f = dot(d2, r);
    constexpr double kTiny = 1e-18;
    if (a <= kTiny && e <= kTiny) {
        s = t = 0.0;
        return;
    }
    if (a <= kTiny) {
        s = 0.0;
        t = std::clamp(f / e, 0.0, 1.0);
        return;
    }
    const double c = dot(d1, r);
    if (e <= kTiny) {
        t = 0.0;
        s = std::clamp(-c / a, 0.0, 1.0);
        return;
    }
    const double b = dot(d1, d2);
    const double denom = a * e - b * b;
    s = denom > kTiny ? std::clamp((b * f - c * e) / denom, 0.0, 1.0) : 0.0;
    t = (b * s + f) / e;
    if (t < 0.0) {
        t = 0.0;
        s = std::clamp(-c / a, 0.0, 1.0);
    } else if (t > 1.0) {
        t = 1.0;
        s = std::clamp((b - c) / a, 0.0, 1.0);
    }
}

std::int64_t cellKey(const Vec3& p, double size) {
    constexpr std::int64_t kMask = (1 << 21) - 1;
    const auto i = static_cast<std::int64_t>(std::floor(p[0] / size)) & kMask;
    const auto j = static_cast<std::int64_t>(std::floor(p[1] / size)) & kMask;
    const auto k = static_cast<std::int64_t>(std::floor(p[2] / size)) & kMask;
    return (i << 42) | (j << 21) | k;
}

}  // namespace

double reachOf(ShapeType type, const Vec3& size, const ConvexHull* hull) {
    switch (type) {
        case ShapeType::Plane:
            return kInfinity;
        case ShapeType::Sphere:
            return size[0];
        case ShapeType::Capsule:
            return size[0] + size[1];
        case ShapeType::Cylinder:
            return std::hypot(size[0], size[1]);
        case ShapeType::ConvexHull:
            if (hull != nullptr) {
                double far = 0.0;
                for (const auto& p : hull->points) {
                    far = std::max(far, norm(p));
                }
                return far;
            }
            return std::hypot(size[0], size[1], size[2]);
        case ShapeType::Box:
        case ShapeType::HeightField:
            return std::hypot(size[0], size[1], size[2]);
    }
    return kInfinity;
}

SurfacePoint surfacePoint(const Obstacle& shape, const Vec3& at) {
    const Vec3 q = rotate(conjugate(shape.pose.orientation), sub(at, shape.pose.position));
    const Vec3& s = shape.size;
    double distance = kInfinity;
    Vec3 n{0.0, 0.0, 1.0};
    Vec3 cp = q;
    switch (shape.type) {
        case ShapeType::Plane:
            distance = q[2];
            cp = {q[0], q[1], 0.0};
            break;
        case ShapeType::Sphere: {
            const double r = norm(q);
            n = r > 1e-12 ? scale(q, 1.0 / r) : Vec3{0.0, 0.0, 1.0};
            distance = r - s[0];
            cp = scale(n, s[0]);
            break;
        }
        case ShapeType::Capsule: {
            const Vec3 c{0.0, 0.0, std::clamp(q[2], -s[1], s[1])};
            const Vec3 d = sub(q, c);
            const double r = norm(d);
            n = r > 1e-12 ? scale(d, 1.0 / r) : Vec3{1.0, 0.0, 0.0};
            distance = r - s[0];
            cp = add(c, scale(n, s[0]));
            break;
        }
        case ShapeType::Cylinder: {
            const double rho = std::hypot(q[0], q[1]);
            const Vec3 u = rho > 1e-12 ? Vec3{q[0] / rho, q[1] / rho, 0.0} : Vec3{1.0, 0.0, 0.0};
            const double side = q[2] >= 0.0 ? 1.0 : -1.0;
            if (rho <= s[0] && std::abs(q[2]) <= s[1]) {
                const double radial = s[0] - rho;
                const double axial = s[1] - std::abs(q[2]);
                if (radial < axial) {
                    distance = -radial;
                    n = u;
                    cp = {u[0] * s[0], u[1] * s[0], q[2]};
                } else {
                    distance = -axial;
                    n = {0.0, 0.0, side};
                    cp = {q[0], q[1], side * s[1]};
                }
            } else {
                const double r = std::min(rho, s[0]);
                cp = {u[0] * r, u[1] * r, std::clamp(q[2], -s[1], s[1])};
                const Vec3 d = sub(q, cp);
                distance = norm(d);
                n = distance > 1e-12 ? scale(d, 1.0 / distance) : u;
            }
            break;
        }
        case ShapeType::Box: {
            const bool inside =
                std::abs(q[0]) <= s[0] && std::abs(q[1]) <= s[1] && std::abs(q[2]) <= s[2];
            if (inside) {
                std::size_t axis = 0;
                double margin = kInfinity;
                for (std::size_t i = 0; i < 3; ++i) {
                    const double m = s.at(i) - std::abs(q.at(i));
                    if (m < margin) {
                        margin = m;
                        axis = i;
                    }
                }
                const double side = q.at(axis) >= 0.0 ? 1.0 : -1.0;
                distance = -margin;
                n = {0.0, 0.0, 0.0};
                n.at(axis) = side;
                cp = q;
                cp.at(axis) = side * s.at(axis);
            } else {
                cp = {std::clamp(q[0], -s[0], s[0]), std::clamp(q[1], -s[1], s[1]),
                      std::clamp(q[2], -s[2], s[2])};
                const Vec3 d = sub(q, cp);
                distance = norm(d);
                n = distance > 1e-12 ? scale(d, 1.0 / distance) : Vec3{0.0, 0.0, 1.0};
            }
            break;
        }
        case ShapeType::ConvexHull: {
            if (shape.hull == nullptr || shape.hull->faces.empty()) {
                break;
            }
            distance = -kInfinity;
            for (const auto& f : shape.hull->faces) {
                const double side = f[0] * q[0] + f[1] * q[1] + f[2] * q[2] - f[3];
                if (side > distance) {
                    distance = side;
                    n = {f[0], f[1], f[2]};
                }
            }
            cp = sub(q, scale(n, distance));
            break;
        }
        case ShapeType::HeightField: {
            if (shape.field == nullptr) {
                break;
            }
            const auto h = shape.field->heightAt(q[0], q[1]);
            if (!h) {
                break;
            }
            const double e = 0.5 * std::min(shape.field->cellX, shape.field->cellY);
            const auto slope = [&](double dx, double dy) {
                const auto a = shape.field->heightAt(q[0] + dx, q[1] + dy);
                const auto b = shape.field->heightAt(q[0] - dx, q[1] - dy);
                return a && b ? (*a - *b) / (2.0 * e) : 0.0;
            };
            const Vec3 up{-slope(e, 0.0), -slope(0.0, e), 1.0};
            n = scale(up, 1.0 / norm(up));
            distance = (q[2] - *h) * n[2];
            cp = sub(q, scale(n, distance));
            break;
        }
    }
    SurfacePoint out;
    out.distance = distance;
    out.normal = rotate(shape.pose.orientation, n);
    out.point = add(shape.pose.position, rotate(shape.pose.orientation, cp));
    return out;
}

Cable::Cable(const CableParams& params, const Vec3& anchor, const Vec3& attach, std::size_t bodies)
    : params_(params) {
    params_.segmentLength = std::max(params_.segmentLength, 1e-3);
    params_.substeps = std::max(params_.substeps, 1);
    radius_ = 0.5 * params_.diameter;
    maxLength_ = std::max(params_.maxLength, params_.length);
    const auto capacity =
        static_cast<std::size_t>(std::ceil(1.5 * maxLength_ / params_.segmentLength)) + 3;
    x_.assign(capacity, Vec3{});
    v_.assign(capacity, Vec3{});
    p_.assign(capacity, Vec3{});
    force_.assign(capacity, Vec3{});
    mass_.assign(capacity, 0.0);
    w_.assign(capacity, 0.0);
    touching_.assign(capacity, 0);
    rest_.assign(capacity - 1, 0.0);
    tension_.assign(capacity - 1, 0.0);
    lambda_.assign(capacity - 1, 0.0);
    pushes_.assign(bodies, Push{});
    near_.reserve(256);
    endInside_.reserve(256);
    cells_.reserve(capacity);
    pairs_.reserve(capacity * 16);
    layOut(anchor, attach);
}

void Cable::layOut(const Vec3& anchor, const Vec3& attach) {
    const double length = std::max(params_.length, kMinLengthM);
    const auto segments = std::clamp<std::size_t>(
        static_cast<std::size_t>(std::lround(length / params_.segmentLength)), 1, x_.size() - 1);
    count_ = segments + 1;
    std::fill(rest_.begin(), rest_.end(), 0.0);
    for (std::size_t j = 0; j < segments; ++j) {
        rest_[j] = length / static_cast<double>(segments);
    }
    const Vec3 chord = sub(attach, anchor);
    // A parabola of rise s above the chord, its length matched to the cable's by bisection. Above,
    // not below: a sag would start the slack inside the ground, a slab or a pad it should lie
    // on (a mass inside a thin box resting on another can be pushed back and forth between them
    // and never get out); laid above, it falls into place.
    std::vector<Vec3> line(kLayoutSamples + 1);
    const auto trace = [&](double sag) {
        double total = 0.0;
        for (int k = 0; k <= kLayoutSamples; ++k) {
            const double u = static_cast<double>(k) / kLayoutSamples;
            line[k] = add(anchor, scale(chord, u));
            line[k][2] += sag * 4.0 * u * (1.0 - u);
            if (k > 0) {
                total += norm(sub(line[k], line[k - 1]));
            }
        }
        return total;
    };
    double sag = 0.0;
    if (length > norm(chord) * 1.0001) {
        double lo = 0.0;
        double hi = length;
        for (int it = 0; it < 60; ++it) {
            sag = 0.5 * (lo + hi);
            (trace(sag) > length ? hi : lo) = sag;
        }
    }
    const double total = trace(sag);
    // Resample at equal steps of arc length.
    std::size_t k = 1;
    double walked = 0.0;
    for (std::size_t i = 0; i < count_; ++i) {
        const double target = total * static_cast<double>(i) / static_cast<double>(segments);
        while (k < line.size() - 1 && walked + norm(sub(line[k], line[k - 1])) < target) {
            walked += norm(sub(line[k], line[k - 1]));
            ++k;
        }
        const double piece = norm(sub(line[k], line[k - 1]));
        const double f = piece > 0.0 ? std::clamp((target - walked) / piece, 0.0, 1.0) : 0.0;
        x_[i] = add(line[k - 1], scale(sub(line[k], line[k - 1]), f));
        v_[i] = {};
    }
    x_[0] = anchor;
    x_[count_ - 1] = attach;
    anchor_ = anchor;
    attach_ = attach;
    updateMasses();
}

void Cable::updateMasses() {
    for (std::size_t i = 0; i < count_; ++i) {
        const double before = i > 0 ? rest_[i - 1] : 0.0;
        const double after = i + 1 < count_ ? rest_[i] : 0.0;
        mass_[i] = params_.linearDensity * 0.5 * (before + after);
        w_[i] = (i == 0 || i + 1 == count_ || mass_[i] <= 0.0) ? 0.0 : 1.0 / mass_[i];
    }
}

double Cable::length() const {
    double total = 0.0;
    for (std::size_t j = 0; j + 1 < count_; ++j) {
        total += rest_[j];
    }
    return total;
}

void Cable::reelWinch(double dt) {
    double rate = 0.0;
    const double t = anchorFilter_;
    switch (params_.winch) {
        case Winch::Fixed:
            break;
        case Winch::Tension: {
            const double error = t - params_.winchTension;
            winchIntegral_ = std::clamp(winchIntegral_ + kWinchIntegral * error * dt,
                                        -params_.winchSpeed, params_.winchSpeed);
            rate = std::clamp(kWinchGain * error + winchIntegral_, -params_.winchSpeed,
                              params_.winchSpeed);
            break;
        }
        case Winch::Free:
            rate = std::clamp(kWinchGain * (t - kFreeDrumN), 0.0, params_.winchSpeed);
            break;
    }
    const double now = length();
    if (rate > 0.0) {
        rate = std::min(rate, std::max(0.0, (maxLength_ - now) / dt));
    } else if (rate < 0.0 && count_ == 2) {
        rate = std::max(rate, -std::max(0.0, (rest_[0] - kMinLengthM) / dt));
    }
    rest_[0] = std::max(rest_[0] + rate * dt, 1e-4);
    winchSpeed_ = rate;
    splitOrMerge();
}

void Cable::splitOrMerge() {
    const double seg = params_.segmentLength;
    bool changed = false;
    while (rest_[0] > 1.5 * seg && count_ < x_.size()) {
        // New cable comes off the drum at the anchor: a mass appears on the first segment, a
        // segment from the anchor, which keeps the short remainder.
        const double f = (rest_[0] - seg) / rest_[0];
        const Vec3 at = add(x_[0], scale(sub(x_[1], x_[0]), f));
        const Vec3 vel = add(v_[0], scale(sub(v_[1], v_[0]), f));
        for (std::size_t i = count_; i > 1; --i) {
            x_[i] = x_[i - 1];
            v_[i] = v_[i - 1];
        }
        for (std::size_t j = count_ - 1; j > 1; --j) {
            rest_[j] = rest_[j - 1];
            lambda_[j] = lambda_[j - 1];
        }
        x_[1] = at;
        v_[1] = vel;
        rest_[1] = seg;
        rest_[0] -= seg;
        ++count_;
        changed = true;
    }
    while (rest_[0] < 0.5 * seg && count_ > 2) {
        // Reeled in: the first mass goes back on the drum.
        rest_[0] += rest_[1];
        for (std::size_t i = 1; i + 1 < count_; ++i) {
            x_[i] = x_[i + 1];
            v_[i] = v_[i + 1];
        }
        for (std::size_t j = 1; j + 2 < count_; ++j) {
            rest_[j] = rest_[j + 1];
            lambda_[j] = lambda_[j + 1];
        }
        rest_[count_ - 2] = 0.0;
        --count_;
        changed = true;
    }
    if (changed) {
        updateMasses();
    }
}

void Cable::solveLengths(double h, std::span<double> lambda) {
    const double ea = std::max(params_.axialStiffness, 1e-9);
    const auto one = [&](std::size_t j) {
        const std::size_t i = j;
        const std::size_t k = j + 1;
        const Vec3 d = sub(p_[k], p_[i]);
        const double l = norm(d);
        const double c = l - rest_[j];
        if ((c <= 0.0 && lambda[j] == 0.0) || l < 1e-12) {
            return;  // slack: a cable does not push
        }
        const double ws = w_[i] + w_[k];
        if (ws <= 0.0) {
            return;
        }
        const Vec3 n = scale(d, 1.0 / l);
        const double alpha = rest_[j] / ea;  // m/N
        const double at = alpha / (h * h);
        // Axial damping: a share of critical for the segment's spring and its masses.
        const double m = 1.0 / ws;
        const double beta = 2.0 * params_.dampingRatio * std::sqrt(ea / rest_[j] * m);
        const double gamma = alpha * beta / h;
        const double rate = dot(n, sub(sub(p_[k], x_[k]), sub(p_[i], x_[i])));
        double dl = (-c - at * lambda[j] - gamma * rate) / ((1.0 + gamma) * ws + at);
        const double next = std::min(lambda[j] + dl, 0.0);  // tension only
        dl = next - lambda[j];
        lambda[j] = next;
        addTo(p_[i], scale(n, -w_[i] * dl));
        addTo(p_[k], scale(n, w_[k] * dl));
    };
    for (std::size_t j = 0; j + 1 < count_; ++j) {
        one(j);
    }
    for (std::size_t j = count_ - 1; j-- > 0;) {
        one(j);
    }
}

void Cable::solveBends(double h) {
    if (params_.bendStiffness <= 0.0) {
        return;
    }
    for (std::size_t i = 0; i + 2 < count_; ++i) {
        const std::size_t k = i + 2;
        const Vec3 d = sub(p_[k], p_[i]);
        const double l = norm(d);
        const double span = rest_[i] + rest_[i + 1];
        const double c = l - span;
        const double ws = w_[i] + w_[k];
        if (c >= 0.0 || ws <= 0.0 || l < 1e-12) {
            continue;
        }
        // A bent pair of segments as a beam pinned at its ends, loaded at the middle:
        // k = 48 EI / span³ across it, about straightening it along the chord.
        const double stiffness = 48.0 * params_.bendStiffness / (span * span * span);
        const double at = 1.0 / (stiffness * h * h);
        const double dl = -c / (ws + at);
        const Vec3 n = scale(d, 1.0 / l);
        addTo(p_[i], scale(n, -w_[i] * dl));
        addTo(p_[k], scale(n, w_[k] * dl));
    }
}

void Cable::solveObstacles(double h, std::span<const Obstacle> obstacles, double share) {
    const double total = length();
    double along = 0.0;
    for (std::size_t i = 1; i + 1 < count_; ++i) {
        along += rest_[i - 1];
        if (w_[i] <= 0.0) {
            continue;
        }
        const bool nearAnchor = along < kEndClearanceM;
        const bool nearAttach = total - along < kEndClearanceM;
        // Moving bodies first, scenery last: scenery always wins, and a body feels only how far
        // the mass ends up moved away from it. A cable squeezed between a skid and the ground
        // then holds the drone up by nothing (it is thinner than the skid's contact), instead of
        // passing on the two pushes that undo each other every substep.
        int hitBody = -1;
        Vec3 hitNormal{};
        Vec3 hitPoint{};
        Vec3 hitCom{};
        Vec3 hitStart{};
        for (int pass = 0; pass < 2; ++pass) {
            for (std::size_t k = 0; k < near_.size(); ++k) {
                const Obstacle& ob = obstacles[near_[k]];
                if ((pass == 0) != (ob.body >= 0)) {
                    continue;
                }
                if ((nearAnchor &&
                     ((endInside_[k] & 1U) != 0 || (ob.robot >= 0 && ob.robot == anchorRobot_))) ||
                    (nearAttach &&
                     ((endInside_[k] & 2U) != 0 || (ob.robot >= 0 && ob.robot == attachRobot_)))) {
                    continue;
                }
                if (std::isfinite(ob.reach) &&
                    norm(sub(p_[i], ob.pose.position)) > ob.reach + radius_) {
                    continue;
                }
                const SurfacePoint sp = surfacePoint(ob, p_[i]);
                if (!(sp.distance < radius_)) {
                    continue;
                }
                if (ob.body >= 0 && hitBody < 0) {
                    hitBody = ob.body;
                    hitNormal = sp.normal;
                    hitPoint = sp.point;
                    hitCom = ob.com;
                    hitStart = p_[i];
                }
                Vec3 surfaceMove{};  // how far the surface moves over the substep
                if (ob.body >= 0) {
                    surfaceMove =
                        scale(add(ob.velocity, cross(ob.angular, sub(sp.point, ob.com))), h);
                }
                // Out by the penetration, but leaving the surface at no more than
                // kMaxDepenetrationMps along its normal: what it already moves away counts.
                const double away = dot(sub(sub(p_[i], x_[i]), surfaceMove), sp.normal);
                const double depth =
                    std::min(radius_ - sp.distance, std::max(0.0, kMaxDepenetrationMps * h - away));
                addTo(p_[i], scale(sp.normal, depth));
                // Friction, as a position limit on the sliding over the substep (Macklin et
                // al. 2014, "Unified Particle Physics for Real-Time Applications", §6.1):
                // within μ·depth it sticks.
                const Vec3 dx = sub(sub(p_[i], x_[i]), surfaceMove);
                const Vec3 slide = sub(dx, scale(sp.normal, dot(dx, sp.normal)));
                const double sliding = norm(slide);
                const double limit = params_.friction * depth;
                if (sliding <= limit) {
                    p_[i] = sub(p_[i], slide);
                    if (sliding > 1e-7) {
                        caught_ = true;
                    }
                } else {
                    p_[i] = sub(p_[i], scale(slide, limit / sliding));
                }
                touching_[i] = 1;
            }
        }
        if (hitBody >= 0 && static_cast<std::size_t>(hitBody) < pushes_.size()) {
            // Equal and opposite: how far the mass was moved away from the body (never into
            // it), and along it, pushes the body at the contact.
            const Vec3 moved = sub(p_[i], hitStart);
            const double away = std::max(0.0, dot(moved, hitNormal));
            const Vec3 sliding = sub(moved, scale(hitNormal, dot(moved, hitNormal)));
            const Vec3 net = add(scale(hitNormal, away), sliding);
            const Vec3 force = scale(net, -mass_[i] / (h * h) * share);
            Push& push = pushes_[static_cast<std::size_t>(hitBody)];
            addTo(push.force, force);
            addTo(push.torque, cross(sub(hitPoint, hitCom), force));
        }
    }
}

void Cable::solveSegments(double h, std::span<const Obstacle> obstacles, double share) {
    // A straight segment between two masses outside a shape can still cut through it where the
    // cable bends over an edge or a corner. For each segment that could reach a shape, the point
    // along it that goes deepest (the signed distance along a line into a convex shape is
    // convex: golden-section search) is pushed out, the two masses sharing the move as the
    // point's position does (Müller et al. 2007's weighting), with friction and the reaction on
    // a moving body as for a mass.
    constexpr int kSearch = 16;     // golden-section steps: 0.2 mm on a 0.2 m segment
    constexpr double kEnds = 0.02;  // nearer an end than this, the mass's own contact does
    const double golden = 0.5 * (std::sqrt(5.0) - 1.0);
    const double total = length();
    double along = 0.0;
    for (std::size_t j = 0; j + 1 < count_; ++j) {
        const std::size_t i = j;
        const std::size_t k = j + 1;
        const double middle = along + 0.5 * rest_[j];
        along += rest_[j];
        if (w_[i] <= 0.0 && w_[k] <= 0.0) {
            continue;
        }
        const Vec3 a = p_[i];
        const Vec3 b = p_[k];
        const Vec3 ab = sub(b, a);
        const double l = norm(ab);
        if (l < 1e-9) {
            continue;
        }
        const Vec3 mid = add(a, scale(ab, 0.5));
        const bool nearAnchor = middle < kEndClearanceM;
        const bool nearAttach = total - middle < kEndClearanceM;
        for (std::size_t q = 0; q < near_.size(); ++q) {
            const Obstacle& ob = obstacles[near_[q]];
            if (ob.type == ShapeType::Plane) {
                continue;  // a segment is deepest at an end: the masses' contacts
            }
            if ((nearAnchor &&
                 ((endInside_[q] & 1U) != 0 || (ob.robot >= 0 && ob.robot == anchorRobot_))) ||
                (nearAttach &&
                 ((endInside_[q] & 2U) != 0 || (ob.robot >= 0 && ob.robot == attachRobot_)))) {
                continue;
            }
            if (std::isfinite(ob.reach) &&
                norm(sub(mid, ob.pose.position)) > ob.reach + 0.5 * l + radius_) {
                continue;
            }
            // The distance is 1-Lipschitz: no point of the segment is deeper than the nearer
            // end less half its length.
            const double da = surfacePoint(ob, a).distance;
            const double db = surfacePoint(ob, b).distance;
            if (std::min(da, db) - 0.5 * l >= radius_) {
                continue;
            }
            const auto at = [&](double t) { return surfacePoint(ob, add(a, scale(ab, t))); };
            double lo = 0.0;
            double hi = 1.0;
            double x1 = hi - golden * (hi - lo);
            double x2 = lo + golden * (hi - lo);
            double f1 = at(x1).distance;
            double f2 = at(x2).distance;
            for (int s = 0; s < kSearch; ++s) {
                if (f1 < f2) {
                    hi = x2;
                    x2 = x1;
                    f2 = f1;
                    x1 = hi - golden * (hi - lo);
                    f1 = at(x1).distance;
                } else {
                    lo = x1;
                    x1 = x2;
                    f1 = f2;
                    x2 = lo + golden * (hi - lo);
                    f2 = at(x2).distance;
                }
            }
            const double t = 0.5 * (lo + hi);
            if (t < kEnds || t > 1.0 - kEnds) {
                continue;
            }
            const SurfacePoint sp = at(t);
            if (!(sp.distance < radius_)) {
                continue;
            }
            const double ws = (1.0 - t) * (1.0 - t) * w_[i] + t * t * w_[k];
            if (ws <= 0.0) {
                continue;
            }
            Vec3 surfaceMove{};
            if (ob.body >= 0) {
                surfaceMove = scale(add(ob.velocity, cross(ob.angular, sub(sp.point, ob.com))), h);
            }
            // How the point moved this substep, relative to the surface.
            const Vec3 moved = sub(
                add(scale(sub(p_[i], x_[i]), 1.0 - t), scale(sub(p_[k], x_[k]), t)), surfaceMove);
            const double away = dot(moved, sp.normal);
            const double depth =
                std::min(radius_ - sp.distance, std::max(0.0, kMaxDepenetrationMps * h - away));
            Vec3 correction = scale(sp.normal, depth);
            // Friction on the point's slide, within μ·depth it sticks.
            const Vec3 slide = sub(moved, scale(sp.normal, away));
            const double sliding = norm(slide);
            const double limit = params_.friction * depth;
            if (sliding <= limit) {
                correction = sub(correction, slide);
                if (sliding > 1e-7) {
                    caught_ = true;
                }
            } else if (sliding > 0.0) {
                correction = sub(correction, scale(slide, limit / sliding));
            }
            // Move the masses so that the point moves by the correction.
            const Vec3 di = scale(correction, (1.0 - t) * w_[i] / ws);
            const Vec3 dk = scale(correction, t * w_[k] / ws);
            addTo(p_[i], di);
            addTo(p_[k], dk);
            touching_[i] = touching_[k] = 1;
            if (ob.body >= 0 && static_cast<std::size_t>(ob.body) < pushes_.size()) {
                const Vec3 impulse = add(scale(di, mass_[i]), scale(dk, mass_[k]));
                const Vec3 force = scale(impulse, -1.0 / (h * h) * share);
                Push& push = pushes_[static_cast<std::size_t>(ob.body)];
                addTo(push.force, force);
                addTo(push.torque, cross(sub(sp.point, ob.com), force));
            }
        }
    }
}

void Cable::findSelfPairs(double margin) {
    // Once a step: segments that could touch within it (their midpoints within half their
    // lengths, the diameter and how far they can move), found through a grid of midpoints.
    pairs_.clear();
    if (count_ < 5) {
        return;
    }
    double longest = 0.0;
    for (std::size_t j = 0; j + 1 < count_; ++j) {
        longest = std::max(longest, rest_[j]);
    }
    const double size = 1.5 * longest + params_.diameter + margin;
    cells_.clear();
    for (std::size_t j = 0; j + 1 < count_; ++j) {
        const Vec3 mid = scale(add(x_[j], x_[j + 1]), 0.5);
        cells_.push_back({cellKey(mid, size), static_cast<std::int64_t>(j)});
    }
    std::sort(cells_.begin(), cells_.end());
    for (std::size_t j = 0; j + 1 < count_; ++j) {
        const Vec3 mid = scale(add(x_[j], x_[j + 1]), 0.5);
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    const Vec3 at{mid[0] + dx * size, mid[1] + dy * size, mid[2] + dz * size};
                    const std::int64_t key = cellKey(at, size);
                    auto it = std::lower_bound(
                        cells_.begin(), cells_.end(),
                        std::array<std::int64_t, 2>{key, std::numeric_limits<std::int64_t>::min()});
                    for (; it != cells_.end() && (*it)[0] == key; ++it) {
                        const auto k = static_cast<std::size_t>((*it)[1]);
                        if (k <= j + 2) {
                            continue;
                        }
                        const Vec3 other = scale(add(x_[k], x_[k + 1]), 0.5);
                        const double within =
                            0.5 * (norm(sub(x_[j + 1], x_[j])) + norm(sub(x_[k + 1], x_[k]))) +
                            params_.diameter + margin;
                        if (norm(sub(mid, other)) <= within) {
                            pairs_.push_back(
                                {static_cast<std::uint32_t>(j), static_cast<std::uint32_t>(k)});
                        }
                    }
                }
            }
        }
    }
    std::sort(pairs_.begin(), pairs_.end());
    pairs_.erase(std::unique(pairs_.begin(), pairs_.end()), pairs_.end());
}

void Cable::solveSelf() {
    const double reach = params_.diameter;
    for (const auto& [ja, kb] : pairs_) {
        const std::size_t a0 = ja;
        const std::size_t a1 = ja + 1;
        const std::size_t b0 = kb;
        const std::size_t b1 = kb + 1;
        double s = 0.0;
        double t = 0.0;
        closestOnSegments(p_[a0], p_[a1], p_[b0], p_[b1], s, t);
        const Vec3 pa = add(p_[a0], scale(sub(p_[a1], p_[a0]), s));
        const Vec3 pb = add(p_[b0], scale(sub(p_[b1], p_[b0]), t));
        const Vec3 d = sub(pa, pb);
        const double l = norm(d);
        if (l >= reach) {
            continue;
        }
        const Vec3 n = l > 1e-12 ? scale(d, 1.0 / l) : Vec3{0.0, 0.0, 1.0};
        const double wsum = (1.0 - s) * (1.0 - s) * w_[a0] + s * s * w_[a1] +
                            (1.0 - t) * (1.0 - t) * w_[b0] + t * t * w_[b1];
        if (wsum <= 0.0) {
            continue;
        }
        const double lam = (reach - l) / wsum;
        addTo(p_[a0], scale(n, (1.0 - s) * w_[a0] * lam));
        addTo(p_[a1], scale(n, s * w_[a1] * lam));
        addTo(p_[b0], scale(n, -(1.0 - t) * w_[b0] * lam));
        addTo(p_[b1], scale(n, -t * w_[b1] * lam));
        touching_[a0] = touching_[a1] = touching_[b0] = touching_[b1] = 1;
    }
}

void Cable::step(double dt, const Vec3& anchor, const Vec3& attach,
                 std::span<const Obstacle> obstacles, const Air& air) {
    if (dt <= 0.0) {
        return;
    }
    reelWinch(dt);
    const int substeps = params_.substeps;
    const double h = dt / substeps;
    const double share = 1.0 / substeps;

    // Obstacles near the cable this step, and which of them hold an end.
    Vec3 lo{kInfinity, kInfinity, kInfinity};
    Vec3 hi{-kInfinity, -kInfinity, -kInfinity};
    double fastest = 0.0;
    for (std::size_t i = 0; i < count_; ++i) {
        for (std::size_t a = 0; a < 3; ++a) {
            lo.at(a) = std::min(lo.at(a), x_[i].at(a));
            hi.at(a) = std::max(hi.at(a), x_[i].at(a));
        }
        fastest = std::max(fastest, norm(v_[i]));
    }
    for (const Vec3& e : {anchor, attach}) {
        for (std::size_t a = 0; a < 3; ++a) {
            lo.at(a) = std::min(lo.at(a), e.at(a));
            hi.at(a) = std::max(hi.at(a), e.at(a));
        }
    }
    const double margin = radius_ + fastest * dt + 0.5;
    findSelfPairs(2.0 * fastest * dt + params_.diameter);
    near_.clear();
    endInside_.clear();
    for (std::size_t o = 0; o < obstacles.size(); ++o) {
        const Obstacle& ob = obstacles[o];
        if (std::isfinite(ob.reach)) {
            const double r = ob.reach + margin;
            const Vec3& c = ob.pose.position;
            if (c[0] + r < lo[0] || c[0] - r > hi[0] || c[1] + r < lo[1] || c[1] - r > hi[1] ||
                c[2] + r < lo[2] || c[2] - r > hi[2]) {
                continue;
            }
        }
        std::uint8_t ends = 0;
        if (surfacePoint(ob, anchor).distance < radius_ + 1e-6) {
            ends |= 1U;
        }
        if (surfacePoint(ob, attach).distance < radius_ + 1e-6) {
            ends |= 2U;
        }
        near_.push_back(static_cast<std::uint32_t>(o));
        endInside_.push_back(ends);
    }

    for (Push& push : pushes_) {
        push = Push{};
    }
    std::fill(tension_.begin(), tension_.end(), 0.0);
    std::fill(touching_.begin(), touching_.end(), 0);
    forceOnAttach_ = {};
    forceOnAnchor_ = {};
    caught_ = false;
    const Vec3 anchorFrom = anchor_;
    const Vec3 attachFrom = attach_;
    const double rho = air.density;
    const double settle = std::exp(-params_.settle * h);

    for (int s = 1; s <= substeps; ++s) {
        const double f = static_cast<double>(s) / substeps;
        const Vec3 a = add(anchorFrom, scale(sub(anchor, anchorFrom), f));
        const Vec3 b = add(attachFrom, scale(sub(attach, attachFrom), f));
        // Air drag on each segment, half to each of its masses.
        for (std::size_t i = 0; i < count_; ++i) {
            force_[i] = {};
        }
        if (rho > 0.0) {
            for (std::size_t j = 0; j + 1 < count_; ++j) {
                const Vec3 d = sub(x_[j + 1], x_[j]);
                const double l = norm(d);
                if (l < 1e-12) {
                    continue;
                }
                const Vec3 t = scale(d, 1.0 / l);
                const Vec3 rel = sub(air.wind, scale(add(v_[j], v_[j + 1]), 0.5));
                const double along = dot(rel, t);
                const Vec3 across = sub(rel, scale(t, along));
                const double q = 0.5 * rho * params_.diameter * l;
                Vec3 drag = scale(across, q * params_.dragCoeff * norm(across));
                addTo(drag, scale(t, q * params_.dragCoeff / 50.0 * std::abs(along) * along));
                addTo(force_[j], scale(drag, 0.5));
                addTo(force_[j + 1], scale(drag, 0.5));
            }
        }
        for (std::size_t i = 0; i < count_; ++i) {
            if (w_[i] > 0.0) {
                addTo(v_[i], scale(add(air.gravity, scale(force_[i], w_[i])), h));
                v_[i] = scale(v_[i], settle);
                p_[i] = add(x_[i], scale(v_[i], h));
            }
        }
        p_[0] = a;
        p_[count_ - 1] = b;
        std::fill(lambda_.begin(), lambda_.end(), 0.0);
        solveLengths(h, lambda_);
        solveBends(h);
        solveObstacles(h, obstacles, share);
        solveSegments(h, obstacles, share);
        solveSelf();
        for (std::size_t i = 0; i < count_; ++i) {
            v_[i] = scale(sub(p_[i], x_[i]), 1.0 / h);
            x_[i] = p_[i];
        }
        for (std::size_t j = 0; j + 1 < count_; ++j) {
            tension_[j] += -lambda_[j] / (h * h) * share;
        }
        // The ends: the end segment's pull, and the end masses' weight and drag, which the
        // pinned ends carry.
        const std::size_t last = count_ - 2;
        const Vec3 d = sub(x_[count_ - 1], x_[last]);
        const double l = norm(d);
        if (l > 1e-12) {
            addTo(forceOnAttach_, scale(d, lambda_[last] / (h * h) / l * share));
        }
        addTo(forceOnAttach_,
              scale(add(scale(air.gravity, mass_[count_ - 1]), force_[count_ - 1]), share));
        const Vec3 e = sub(x_[0], x_[1]);
        const double m = norm(e);
        if (m > 1e-12) {
            addTo(forceOnAnchor_, scale(e, lambda_[0] / (h * h) / m * share));
        }
        addTo(forceOnAnchor_, scale(add(scale(air.gravity, mass_[0]), force_[0]), share));
    }
    anchor_ = anchor;
    attach_ = attach;
    tensionAttach_ = tension_[count_ - 2];
    tensionAnchor_ = tension_[0];
    const double k = dt / (kAnchorFilterS + dt);
    anchorFilter_ += k * (tensionAnchor_ - anchorFilter_);
    contacts_ = 0;
    for (std::size_t i = 0; i < count_; ++i) {
        contacts_ += touching_[i] != 0 ? 1 : 0;
    }
}

void Cable::save(std::vector<double>& out) const {
    out.push_back(static_cast<double>(count_));
    for (std::size_t i = 0; i < count_; ++i) {
        out.insert(out.end(), x_[i].begin(), x_[i].end());
        out.insert(out.end(), v_[i].begin(), v_[i].end());
    }
    for (std::size_t j = 0; j + 1 < count_; ++j) {
        out.push_back(rest_[j]);
    }
    out.insert(out.end(), anchor_.begin(), anchor_.end());
    out.insert(out.end(), attach_.begin(), attach_.end());
    out.push_back(anchorFilter_);
    out.push_back(winchSpeed_);
    out.push_back(winchIntegral_);
}

bool Cable::restore(std::span<const double> in) {
    if (in.empty()) {
        return false;
    }
    const auto count = static_cast<std::size_t>(in[0]);
    if (count < 2 || count > x_.size() || in.size() != 1 + count * 6 + (count - 1) + 9) {
        return false;
    }
    std::size_t k = 1;
    count_ = count;
    for (std::size_t i = 0; i < count_; ++i) {
        for (double& c : x_[i]) {
            c = in[k++];
        }
        for (double& c : v_[i]) {
            c = in[k++];
        }
    }
    std::fill(rest_.begin(), rest_.end(), 0.0);
    for (std::size_t j = 0; j + 1 < count_; ++j) {
        rest_[j] = in[k++];
    }
    for (double& c : anchor_) {
        c = in[k++];
    }
    for (double& c : attach_) {
        c = in[k++];
    }
    anchorFilter_ = in[k++];
    winchSpeed_ = in[k++];
    winchIntegral_ = in[k++];
    updateMasses();
    return true;
}

void Cable::shift(const Vec3& by) {
    for (std::size_t i = 0; i < count_; ++i) {
        x_[i] = add(x_[i], by);
    }
    anchor_ = add(anchor_, by);
    attach_ = add(attach_, by);
}

}  // namespace lagrange::tether
