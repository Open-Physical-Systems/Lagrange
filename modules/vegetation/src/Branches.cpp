#include "lagrange/vegetation/Branches.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace lagrange::vegetation {

namespace {

using V3 = std::array<double, 3>;

V3 add(const V3& a, const V3& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}
V3 sub(const V3& a, const V3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
V3 scale(const V3& a, double s) {
    return {a[0] * s, a[1] * s, a[2] * s};
}
double dot(const V3& a, const V3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
V3 cross(const V3& a, const V3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
double norm(const V3& a) {
    return std::sqrt(dot(a, a));
}

/// A stub shorter than this is not cut again (m), as for the grass (kMinCutPiece).
constexpr double kMinPiece = 0.01;

}  // namespace

double discMeets(const V3& a, const V3& b, double d, const V3& centre, const V3& normal,
                 double radius) {
    const V3 ab = sub(b, a);
    const double length = norm(ab);
    if (!(length > 0.0)) {
        return -1.0;
    }
    const V3 u = scale(ab, 1.0 / length);
    const V3 w0 = sub(a, centre);
    double lo = 0.0;
    double hi = length;
    // Within the sheet: |n·(a + s u - c)| <= t.
    const double t = kBladeHalfThickness + 0.5 * d;
    const double h0 = dot(normal, w0);
    const double nu = dot(normal, u);
    if (std::abs(nu) < 1e-12) {
        if (std::abs(h0) > t) {
            return -1.0;
        }
    } else {
        const double s1 = (-t - h0) / nu;
        const double s2 = (t - h0) / nu;
        lo = std::max(lo, std::min(s1, s2));
        hi = std::min(hi, std::max(s1, s2));
    }
    // Within the radius: |w + s v|² <= R², w and v the parts in the disc's plane.
    const V3 w = sub(w0, scale(normal, h0));
    const V3 v = sub(u, scale(normal, nu));
    const double vv = dot(v, v);
    const double wv = dot(w, v);
    const double ww = dot(w, w) - radius * radius;
    if (vv < 1e-12) {
        if (ww > 0.0) {
            return -1.0;
        }
    } else {
        const double disc = wv * wv - vv * ww;
        if (disc < 0.0) {
            return -1.0;
        }
        const double root = std::sqrt(disc);
        lo = std::max(lo, (-wv - root) / vv);
        hi = std::min(hi, (-wv + root) / vv);
    }
    return lo <= hi ? lo : -1.0;
}

void Branches::build(const std::vector<TreeSpec>& trees) {
    wood_.build(trees);
    cuts_.clear();
    std::fill(cutValues_.begin(), cutValues_.end(), 0.0);
    cutVersion_ = 0.0;
    cutOut_ = SignalId{};
    rotors_.clear();
    twigsOut_.clear();
    twigs_.assign(trees.size(), 0.0);
}

void Branches::addRotor(SignalDeclarer& d, const std::string& joint,
                        const std::array<double, 3>& linkCom) {
    RotorState r;
    r.com = linkCom;
    if (!wood_.empty()) {
        SignalSpec damage;
        damage.name = "rotor/" + joint + "/damage";
        damage.type = SignalType::Scalar;
        damage.unit = units::one;
        r.damageOut = d.output(damage);
        SignalSpec strikes = damage;
        strikes.name = "rotor/" + joint + "/branch_strikes";
        r.strikesOut = d.output(strikes);
    }
    rotors_.push_back(r);
}

void Branches::addRotor(const std::array<double, 3>& linkCom) {
    RotorState r;
    r.com = linkCom;
    rotors_.push_back(r);
}

void Branches::declareTrees(const std::vector<TreeSpec>& trees, SignalDeclarer& d) {
    twigsOut_.clear();
    for (const TreeSpec& t : trees) {
        SignalSpec s;
        s.name = "tree/" + t.name + "/twigs_cut";
        s.type = SignalType::Scalar;
        s.unit = units::one;
        twigsOut_.push_back(d.output(s));
    }
    if (!trees.empty()) {
        SignalSpec cut;
        cut.name = std::string(kBranchCutSignal);
        cut.type = SignalType::ScalarArray;
        cut.unit = units::one;
        cut.arrayLength = kBranchCutLength;
        cutOut_ = d.output(cut);
    }
}

void Branches::init(SignalBus& bus) {
    wood_.reset();
    cuts_.clear();
    std::fill(cutValues_.begin(), cutValues_.end(), 0.0);
    cutVersion_ = -1.0;  // written by the first publish
    std::fill(twigs_.begin(), twigs_.end(), 0.0);
    for (RotorState& r : rotors_) {
        r.damage = 0.0;
        r.strikes = 0.0;
    }
    publish(bus);
}

double Branches::standing(std::size_t tree, std::size_t s) const {
    return wood_.standing(tree, s);
}

void Branches::apply(const Cut& cut) {
    wood_.cut(cut.tree, cut.segment, cut.share);
}

BranchHit Branches::sweep(std::size_t rotor, const std::vector<TreeSpec>& trees, const Pose& hub,
                          const std::array<double, 3>& axis, double radius,
                          const std::vector<std::array<double, 3>>& sways, double omega,
                          double inertia, double dt) {
    BranchHit hit;
    if (rotor >= rotors_.size() || wood_.empty()) {
        return hit;
    }
    std::uint32_t strikeTree = 0;
    std::uint32_t strikeSegment = 0;
    V3 strikeAt{};
    for (std::size_t k = 0; k < trees.size() && k < wood_.trees(); ++k) {
        const TreeSpec& tree = trees[k];
        const std::vector<BranchSegment>& segs = wood_.skeleton(k).segments;
        if (segs.empty()) {
            continue;
        }
        const V3 o = inCrown(tree, hub.position, sways[k]);
        const V3 n = rotate(conjugate(tree.pose.orientation), axis);
        if (norm(sub(o, segs[0].centre)) > segs[0].radius + radius + kBladeHalfThickness) {
            continue;  // far from this tree's wood
        }
        const auto standing = [&](std::size_t s) { return wood_.standing(k, s); };
        std::size_t i = 0;
        while (i < segs.size()) {
            const BranchSegment& s = segs[i];
            if (standing(i) <= 0.0) {
                i = s.end;
                continue;
            }
            const V3 dc = sub(s.centre, o);
            const double h = dot(n, dc);
            if (std::abs(h) > s.radius + kBladeHalfThickness ||
                norm(sub(dc, scale(n, h))) > s.radius + radius) {
                i = s.end;
                continue;
            }
            const V3 full = sub(s.to, s.from);
            const double length = norm(full);
            const double reach = standing(i) * length;
            const double at =
                discMeets(s.from, add(s.from, scale(full, standing(i))), s.diameter, o, n, radius);
            if (at >= 0.0 && at < reach - kMinPiece) {
                if (s.diameter <= tree.maxCutDiameter) {
                    const Cut cut{static_cast<std::uint32_t>(k), static_cast<std::uint32_t>(i),
                                  at / length};
                    record(cut);
                    hit.cutEnergy +=
                        tree.woodCutEnergy * std::numbers::pi * s.diameter * s.diameter / 4.0;
                    ++hit.twigsCut;
                    twigs_[k] += 1.0;
                    i = s.end;
                    continue;
                }
                if (s.diameter > hit.diameter) {
                    hit.diameter = s.diameter;
                    strikeTree = static_cast<std::uint32_t>(k);
                    strikeSegment = static_cast<std::uint32_t>(i);
                    strikeAt = add(s.from, scale(full, at / length));
                }
            }
            i = standing(i) < 1.0 ? s.end : i + 1;
        }
    }
    if (!(hit.diameter > 0.0) || !(inertia > 0.0) || !(dt > 0.0) || omega == 0.0) {
        return hit;
    }
    // The strike: back to the world.
    const TreeSpec& tree = trees[strikeTree];
    const BranchSegment& s = wood_.skeleton(strikeTree).segments[strikeSegment];
    const V3 up = rotate(tree.pose.orientation, {0.0, 0.0, tree.centreHeight()});
    const V3& sway = sways[strikeTree];
    const V3 point =
        add(add(add(tree.pose.position, up), sway), rotate(tree.pose.orientation, strikeAt));
    V3 u = rotate(tree.pose.orientation, sub(s.to, s.from));
    u = scale(u, 1.0 / norm(u));
    const V3 off = sub(point, hub.position);
    const V3 r = sub(off, scale(axis, dot(axis, off)));
    const double rn = norm(r);
    if (rn < 1e-3) {
        return hit;  // on the hub
    }
    const double spin = omega > 0.0 ? 1.0 : -1.0;
    const V3 tangent = scale(cross(axis, r), spin / rn);  // the blade's motion there
    // The branch pushes square to itself, against the blade's motion.
    V3 push = sub(tangent, scale(u, dot(tangent, u)));
    const double pushLength = norm(push);
    push = pushLength > 0.3 ? scale(push, -1.0 / pushLength) : scale(tangent, -1.0);
    double lever = dot(axis, cross(r, push));
    if (lever * omega >= 0.0 || std::abs(lever) < 0.3 * rn) {
        push = scale(tangent, -1.0);
        lever = dot(axis, cross(r, push));
    }
    // As hard as it takes to stop the propeller in the step: lever × F × dt = -I ω.
    const double force = -inertia * omega / (dt * lever);
    const V3 f = scale(push, force);
    const V3 com = compose(hub, Pose{rotors_[rotor].com, {1.0, 0.0, 0.0, 0.0}}).position;
    const V3 moment = cross(sub(point, com), f);
    hit.strike = true;
    hit.point = point;
    hit.wrench = {f[0], f[1], f[2], moment[0], moment[1], moment[2]};
    hit.energy = 0.5 * inertia * omega * omega;
    RotorState& rs = rotors_[rotor];
    rs.strikes += 1.0;
    if (breakEnergy > 0.0) {
        rs.damage = std::min(1.0, rs.damage + hit.energy / breakEnergy);
    }
    return hit;
}

void Branches::publish(SignalBus& bus) {
    for (const RotorState& r : rotors_) {
        if (r.damageOut.valid()) {
            bus.write(r.damageOut, r.damage);
            bus.write(r.strikesOut, r.strikes);
        }
    }
    for (std::size_t k = 0; k < twigsOut_.size() && k < twigs_.size(); ++k) {
        bus.write(twigsOut_[k], twigs_[k]);
    }
    if (cutOut_.valid()) {
        if (cutValues_[0] != cutVersion_) {
            bus.writeN(cutOut_, cutValues_);
            cutVersion_ = cutValues_[0];
        } else {
            bus.hold(cutOut_);
        }
    }
}

void Branches::record(const Cut& cut) {
    cuts_.push_back(cut);
    apply(cut);
    const std::size_t n = cuts_.size();
    cutValues_[0] = static_cast<double>(n);  // the version: the list only grows
    if (n <= kBranchCutCapacity) {
        const std::size_t at = kBranchCutHeader + (kBranchCutFields * (n - 1));
        cutValues_[1] = static_cast<double>(n);
        cutValues_[at] = cut.tree;
        cutValues_[at + 1] = cut.segment;
        cutValues_[at + 2] = cut.share;
    }
}

void Branches::save(StateWriter& writer) const {
    for (const RotorState& r : rotors_) {
        writer.put(r.damage);
        writer.put(r.strikes);
    }
    writer.putDoubles(twigs_);
    writer.put<std::uint64_t>(cuts_.size());
    for (const Cut& c : cuts_) {
        writer.put(c.tree);
        writer.put(c.segment);
        writer.put(c.share);
    }
}

bool Branches::restore(StateReader& reader) {
    for (RotorState& r : rotors_) {
        if (!reader.get(r.damage) || !reader.get(r.strikes)) {
            return false;
        }
    }
    if (!reader.getDoubles(twigs_)) {
        return false;
    }
    std::uint64_t count = 0;
    if (!reader.get(count)) {
        return false;
    }
    wood_.reset();
    cuts_.clear();
    std::fill(cutValues_.begin(), cutValues_.end(), 0.0);
    cutVersion_ = -1.0;
    for (std::uint64_t k = 0; k < count; ++k) {
        Cut c{};
        if (!reader.get(c.tree) || !reader.get(c.segment) || !reader.get(c.share) ||
            c.tree >= wood_.trees() || c.segment >= wood_.skeleton(c.tree).segments.size()) {
            return false;
        }
        record(c);
    }
    return true;
}

namespace {

/// x (world) in the frame of `pose`.
V3 inFrame(const Pose& pose, const V3& x) {
    return rotate(conjugate(pose.orientation), sub(x, pose.position));
}

/// The distance from p to the segment a-b.
double toSegment(const V3& p, const V3& a, const V3& b) {
    const V3 ab = sub(b, a);
    const double abab = dot(ab, ab);
    const double t = abab > 0.0 ? std::clamp(dot(sub(p, a), ab) / abab, 0.0, 1.0) : 0.0;
    return norm(sub(p, add(a, scale(ab, t))));
}

/// Golden-section search for the t in [0, length] where f is least; f is convex along it (a
/// convex shape's signed distance along a line).
template <class F>
double leastAlong(double length, const F& f) {
    constexpr int kIterations = 40;  // 0.618^40: 4e-9 of the length
    const double g = (std::sqrt(5.0) - 1.0) / 2.0;
    double lo = 0.0;
    double hi = length;
    double x1 = hi - g * (hi - lo);
    double x2 = lo + g * (hi - lo);
    double f1 = f(x1);
    double f2 = f(x2);
    for (int k = 0; k < kIterations; ++k) {
        if (f1 <= f2) {
            hi = x2;
            x2 = x1;
            f2 = f1;
            x1 = hi - g * (hi - lo);
            f1 = f(x1);
        } else {
            lo = x1;
            x1 = x2;
            f1 = f2;
            x2 = lo + g * (hi - lo);
            f2 = f(x2);
        }
    }
    return 0.5 * (lo + hi);
}

}  // namespace

bool woodShapeOf(const World& world, std::size_t c, WoodShape& out) {
    const World::Colliders& colliders = world.colliders;
    out = WoodShape{};
    out.type = colliders.shape.at(c);
    out.size = colliders.size.at(c);
    out.pose = colliders.poseInBody.at(c);
    const V3& s = out.size;
    switch (out.type) {
        case ShapeType::Sphere:
            out.bound = s[0];
            return s[0] > 0.0;
        case ShapeType::Capsule:
            out.bound = s[0] + s[1];
            return s[0] > 0.0;
        case ShapeType::Cylinder:
            out.bound = std::hypot(s[0], s[1]);
            return s[0] > 0.0;
        case ShapeType::Box:
            out.bound = norm(s);
            return out.bound > 0.0;
        case ShapeType::ConvexHull: {
            const std::uint32_t h = colliders.hull.at(c);
            if (h >= world.hulls.size() || world.hulls[h].faces.empty()) {
                return false;
            }
            out.faces = &world.hulls[h].faces;
            for (const V3& p : world.hulls[h].points) {
                out.bound = std::max(out.bound, norm(p));
            }
            return true;
        }
        case ShapeType::Plane:
        case ShapeType::HeightField:
            break;
    }
    return false;
}

double shapeDistance(const WoodShape& shape, const V3& q) {
    const V3& s = shape.size;
    switch (shape.type) {
        case ShapeType::Sphere:
            return norm(q) - s[0];
        case ShapeType::Capsule:
            return std::hypot(q[0], q[1], q[2] - std::clamp(q[2], -s[1], s[1])) - s[0];
        case ShapeType::Cylinder: {
            const double dr = std::hypot(q[0], q[1]) - s[0];
            const double dz = std::abs(q[2]) - s[1];
            return std::min(std::max(dr, dz), 0.0) +
                   std::hypot(std::max(dr, 0.0), std::max(dz, 0.0));
        }
        case ShapeType::Box: {
            const V3 d{std::abs(q[0]) - s[0], std::abs(q[1]) - s[1], std::abs(q[2]) - s[2]};
            const V3 out{std::max(d[0], 0.0), std::max(d[1], 0.0), std::max(d[2], 0.0)};
            return norm(out) + std::min(std::max({d[0], d[1], d[2]}), 0.0);
        }
        case ShapeType::ConvexHull: {
            if (shape.faces == nullptr || shape.faces->empty()) {
                return std::numeric_limits<double>::infinity();
            }
            double most = -std::numeric_limits<double>::infinity();
            for (const std::array<double, 4>& f : *shape.faces) {
                most = std::max(most, f[0] * q[0] + f[1] * q[1] + f[2] * q[2] - f[3]);
            }
            return most;
        }
        case ShapeType::Plane:
        case ShapeType::HeightField:
            break;
    }
    return std::numeric_limits<double>::infinity();
}

bool Branches::inFixed(const V3& point) const {
    for (const WoodShape& f : fixed_) {
        if (norm(sub(point, f.pose.position)) <= f.bound &&
            shapeDistance(f, inFrame(f.pose, point)) < 0.0) {
            return true;
        }
    }
    return false;
}

BodyContact Branches::touch(const std::vector<TreeSpec>& trees, std::span<const WoodShape> shapes,
                            double bound, const Pose& pose, const std::array<double, 6>& twist,
                            const V3& comLocal, double mass, const std::vector<V3>& sways,
                            const std::vector<V3>& crownVelocity, std::vector<V3>& crownPush) {
    BodyContact out;
    if (wood_.empty() || shapes.empty() || !(mass > 0.0) || !(contactTime > 0.0)) {
        return out;
    }
    // The penalty contact: critically damped at τ for the link's mass.
    const double stiffness = mass / (contactTime * contactTime);
    const double damping = 2.0 * mass / contactTime;
    const V3 v{twist[0], twist[1], twist[2]};
    const V3 w{twist[3], twist[4], twist[5]};
    const V3 com = compose(pose, Pose{comLocal, {1.0, 0.0, 0.0, 0.0}}).position;
    std::vector<Pose> placed(shapes.size());
    for (std::size_t j = 0; j < shapes.size(); ++j) {
        placed[j] = compose(pose, shapes[j].pose);
    }
    std::vector<V3> centres(shapes.size());
    V3 total{};
    for (std::size_t k = 0; k < trees.size() && k < wood_.trees(); ++k) {
        const std::vector<BranchSegment>& segs = wood_.skeleton(k).segments;
        if (segs.empty()) {
            continue;
        }
        const TreeSpec& tree = trees[k];
        const V3 o = inCrown(tree, pose.position, sways[k]);
        if (norm(sub(o, segs[0].centre)) > segs[0].radius + bound) {
            continue;  // far from this tree's wood
        }
        for (std::size_t j = 0; j < shapes.size(); ++j) {
            centres[j] = inCrown(tree, placed[j].position, sways[k]);
        }
        const V3 crown = add(
            add(tree.pose.position, rotate(tree.pose.orientation, {0.0, 0.0, tree.centreHeight()})),
            sways[k]);
        std::size_t i = 0;
        while (i < segs.size()) {
            const BranchSegment& s = segs[i];
            const double standing = wood_.standing(k, i);
            if (standing <= 0.0 || norm(sub(o, s.centre)) > s.radius + bound) {
                i = s.end;
                continue;
            }
            const double r = 0.5 * s.diameter;
            const V3 full = sub(s.to, s.from);
            const double length = norm(full);
            const double reach = standing * length;
            const V3 tip = add(s.from, scale(full, standing));
            bool broke = false;
            for (std::size_t j = 0; j < shapes.size() && length > 0.0; ++j) {
                const WoodShape& shape = shapes[j];
                if (toSegment(centres[j], s.from, tip) > shape.bound + r) {
                    continue;
                }
                const V3 aw = add(crown, rotate(tree.pose.orientation, s.from));
                const V3 uw = rotate(tree.pose.orientation, scale(full, 1.0 / length));
                const V3 as = inFrame(placed[j], aw);
                const V3 us = rotate(conjugate(placed[j].orientation), uw);
                const auto along = [&](double t) {
                    return shapeDistance(shape, add(as, scale(us, t)));
                };
                const double t = leastAlong(reach, along);
                const double distance = along(t);
                const double depth = r - distance;
                if (!(depth > 0.0)) {
                    continue;
                }
                const V3 x = add(aw, scale(uw, t));
                if (inFixed(x)) {
                    continue;  // the trunk the stage gives the tree: its collider's
                }
                if (s.diameter <= tree.maxCutDiameter) {
                    if (t < reach - kMinPiece) {  // a twig: it breaks
                        record(Cut{static_cast<std::uint32_t>(k), static_cast<std::uint32_t>(i),
                                   t / length});
                        twigs_[k] += 1.0;
                        ++out.broken;
                        broke = true;
                        break;
                    }
                    continue;
                }
                // The collider's outward normal at the wood's axis (central differences).
                const V3 xs = add(as, scale(us, t));
                constexpr double kH = 1e-6;  // m
                V3 grad{};
                for (std::size_t c = 0; c < 3; ++c) {
                    V3 hi = xs;
                    V3 lo = xs;
                    hi.at(c) += kH;
                    lo.at(c) -= kH;
                    grad.at(c) = shapeDistance(shape, hi) - shapeDistance(shape, lo);
                }
                const double gn = norm(grad);
                if (!(gn > 0.0)) {
                    continue;
                }
                const V3 n = rotate(placed[j].orientation, scale(grad, 1.0 / gn));
                const V3 p = sub(x, scale(n, distance));  // on the collider's surface
                const V3 vb = add(v, cross(w, sub(p, pose.position)));
                const V3& vc = crownVelocity[k];
                const V3 rel = sub(vb, vc);
                const double vn = dot(rel, n);  // > 0: into the wood
                const double fn = stiffness * depth + damping * vn;
                if (!(fn > 0.0)) {
                    continue;
                }
                V3 f = scale(n, -fn);
                const V3 vt = sub(rel, scale(n, vn));
                const double slide = norm(vt);
                if (slide > 0.0) {
                    const double ft = std::min(friction * fn, damping * slide);
                    f = sub(f, scale(vt, ft / slide));
                }
                const V3 m = cross(sub(p, com), f);
                for (std::size_t c = 0; c < 3; ++c) {
                    out.wrench.at(c) += f.at(c);
                    out.wrench.at(c + 3) += m.at(c);
                }
                total = add(total, f);
                crownPush[k][0] -= f[0];
                crownPush[k][1] -= f[1];
                out.depth = std::max(out.depth, depth);
                ++out.contacts;
            }
            i = broke || standing < 1.0 ? s.end : i + 1;
        }
    }
    out.force = norm(total);
    return out;
}

}  // namespace lagrange::vegetation
