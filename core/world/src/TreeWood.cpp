#include "lagrange/world/PoseMath.hpp"
#include "lagrange/world/TreeWood.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace lagrange {

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

/// v turned by `angle` about the unit axis k (Rodrigues).
V3 turn(const V3& v, const V3& k, double angle) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return add(add(scale(v, c), scale(cross(k, v), s)), scale(k, dot(k, v) * (1.0 - c)));
}

/// Distance from p (inside) along unit d to the ellipsoid x²/R² + y²/R² + z²/c² = 1.
double toSurface(const V3& p, const V3& d, double r, double c) {
    const double a = (d[0] * d[0] + d[1] * d[1]) / (r * r) + d[2] * d[2] / (c * c);
    const double b = 2.0 * ((p[0] * d[0] + p[1] * d[1]) / (r * r) + p[2] * d[2] / (c * c));
    const double k = (p[0] * p[0] + p[1] * p[1]) / (r * r) + p[2] * p[2] / (c * c) - 1.0;
    const double disc = b * b - 4.0 * a * k;
    if (!(a > 0.0) || disc < 0.0) {
        return 0.0;
    }
    return std::max(0.0, (-b + std::sqrt(disc)) / (2.0 * a));
}

class Grower {
public:
    Grower(Skeleton& out, double radius, double half) : out_(out), r_(radius), c_(half) {}

    std::uint32_t push(const V3& from, const V3& to, double d, std::int32_t generation) {
        BranchSegment s;
        s.from = from;
        s.to = to;
        s.diameter = d;
        s.generation = generation;
        out_.segments.push_back(s);
        return static_cast<std::uint32_t>(out_.segments.size() - 1);
    }
    void close(std::uint32_t i) {
        out_.segments[i].end = static_cast<std::uint32_t>(out_.segments.size());
    }

    /// A limb from `start` along frame (dir, p, q), base diameter d, reaching `length`.
    void limb(const V3& start, const V3& dir, const V3& p, double d, double length) {
        if (!(d >= kTwigDiameter) || !(length > 0.0)) {
            return;
        }
        int generations = 0;
        while (forkDiameter(d, generations) >= kTwigDiameter) {
            ++generations;
        }
        const double first =
            length * (1.0 - kForkLengthRatio) / (1.0 - std::pow(kForkLengthRatio, generations));
        grow(start, dir, p, cross(dir, p), first, d, 0, generations);
    }

private:
    void grow(const V3& start, const V3& dir, const V3& p, const V3& q, double length, double d,
              int g, int generations) {
        // A branch that would leave the crown ends at its edge, a tip.
        const double room = toSurface(start, dir, r_, c_);
        const bool edge = room < length;
        const V3 end = add(start, scale(dir, edge ? room : length));
        const std::uint32_t i = push(start, end, d, g);
        if (g + 1 < generations && !edge) {
            for (const double side : {1.0, -1.0}) {
                const double angle = side * kForkHalfAngle;
                const V3 cd = turn(dir, q, angle);
                const V3 cp = turn(p, q, angle);
                // The next fork's plane rolled 90° about the child: (p, q) -> (q, -p).
                grow(end, cd, q, scale(cp, -1.0), length * kForkLengthRatio,
                     d / std::numbers::sqrt2, g + 1, generations);
            }
        }
        close(i);
    }

    Skeleton& out_;
    double r_, c_;  // the crown's semi-axes
};

/// Whether the ray o + t d (unit d), t in [0, reach], passes within `radius` of `centre`.
bool raySphere(const V3& o, const V3& d, double reach, const V3& centre, double radius) {
    const V3 oc = sub(centre, o);
    const double t = std::clamp(dot(oc, d), 0.0, reach);
    const V3 off = sub(oc, scale(d, t));
    return dot(off, off) <= radius * radius;
}

}  // namespace

double limbDiameter(double trunkDiameter, double length, double totalLength) {
    if (!(totalLength > 0.0) || !(length > 0.0)) {
        return 0.0;
    }
    return trunkDiameter * std::sqrt(length / totalLength);
}

double forkDiameter(double diameter, int generation) {
    return diameter * std::pow(2.0, -0.5 * generation);
}

Skeleton skeletonOf(const TreeSpec& tree, std::uint32_t index) {
    Skeleton out;
    const double span = tree.height - tree.crownBase;
    const double c = tree.halfHeight();
    const double centre = tree.centreHeight();
    if (!(tree.trunkDiameter > 0.0) || !(span > 0.0) || !(tree.crownRadius > 0.0)) {
        return out;
    }
    const int n = std::max(0, tree.branchCount);
    const double top = tree.crownBase + kLimbSpan * span;
    std::vector<double> heights(static_cast<std::size_t>(n));
    std::vector<V3> dirs(static_cast<std::size_t>(n));
    double total = 0.0;
    for (int k = 0; k < n; ++k) {
        const auto ku = static_cast<std::size_t>(k);
        heights[ku] = tree.crownBase + kLimbSpan * span * (k + 0.5) / n;
        const double phi = kGoldenAngle * (k + 0.5 * index);
        dirs[ku] = {std::cos(kLimbElevation) * std::cos(phi),
                    std::cos(kLimbElevation) * std::sin(phi), std::sin(kLimbElevation)};
        out.limbs.push_back(
            toSurface({0.0, 0.0, heights[ku] - centre}, dirs[ku], tree.crownRadius, c));
        total += out.limbs.back();
    }
    out.limbs.push_back(tree.height - top);  // the leader
    total += out.limbs.back();
    out.trunkArea = std::numbers::pi * tree.trunkDiameter * tree.trunkDiameter / 4.0;

    Grower grow(out, tree.crownRadius, c);
    // The trunk through the crown, a segment from junction to junction; each holds what the
    // limbs below it did not take, and its subtree is everything above it.
    std::vector<std::uint32_t> open;
    double area = out.trunkArea;
    double from = tree.crownBase;
    for (int k = 0; k <= n; ++k) {
        const double to = k < n ? heights[static_cast<std::size_t>(k)] : top;
        open.push_back(grow.push({0.0, 0.0, from - centre}, {0.0, 0.0, to - centre},
                                 std::sqrt(4.0 * std::max(area, 0.0) / std::numbers::pi), -1));
        const double length = out.limbs[static_cast<std::size_t>(k)];
        const double d = limbDiameter(tree.trunkDiameter, length, total);
        if (k < n) {
            const V3& dir = dirs[static_cast<std::size_t>(k)];
            const V3 side = {-dir[1], dir[0], 0.0};
            const double l = norm(side);
            grow.limb({0.0, 0.0, to - centre}, dir, scale(side, 1.0 / l), d, length);
        } else {
            grow.limb({0.0, 0.0, to - centre}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, d, length);
        }
        area -= std::numbers::pi * d * d / 4.0;
        from = to;
    }
    for (auto it = open.rbegin(); it != open.rend(); ++it) {
        grow.close(*it);
    }
    // Spheres around each segment and its subtree, children first.
    std::vector<BranchSegment>& s = out.segments;
    for (std::size_t i = s.size(); i-- > 0;) {
        BranchSegment& b = s[i];
        b.centre = scale(add(b.from, b.to), 0.5);
        b.radius = 0.5 * norm(sub(b.to, b.from)) + 0.5 * b.diameter;
        for (std::size_t j = i + 1; j < b.end; j = s[j].end) {
            b.radius = std::max(b.radius, norm(sub(s[j].centre, b.centre)) + s[j].radius);
        }
    }
    return out;
}

double rayCapsule(const V3& o, const V3& d, const V3& a, const V3& b, double r, double range) {
    constexpr double kNever = std::numeric_limits<double>::infinity();
    const V3 axis = sub(b, a);
    const V3 rel = sub(o, a);
    const double axisSq = dot(axis, axis);
    // Inside already: the point of the axis nearest the origin is within r.
    const double along = axisSq > 0.0 ? std::clamp(dot(rel, axis) / axisSq, 0.0, 1.0) : 0.0;
    const V3 off = sub(rel, scale(axis, along));
    if (dot(off, off) <= r * r) {
        return 0.0;
    }
    double best = kNever;
    // The side: |(rel + t d) - ((rel + t d)·axis / axisSq) axis|² = r², the hit between the ends.
    if (axisSq > 0.0) {
        const double axisDir = dot(axis, d);
        const double axisRel = dot(axis, rel);
        const double qa = axisSq - axisDir * axisDir;
        const double qb = axisSq * dot(d, rel) - axisRel * axisDir;
        const double qc = axisSq * dot(rel, rel) - axisRel * axisRel - r * r * axisSq;
        const double h = qb * qb - qa * qc;
        if (qa > 1e-12 * axisSq && h >= 0.0) {
            const double t = (-qb - std::sqrt(h)) / qa;
            const double y = axisRel + t * axisDir;
            if (t >= 0.0 && y > 0.0 && y < axisSq) {
                best = t;
            }
        }
    }
    // The round ends.
    for (const V3* end : {&a, &b}) {
        const V3 oc = sub(o, *end);
        const double hb = dot(d, oc);
        const double hc = dot(oc, oc) - r * r;
        const double h = hb * hb - hc;
        if (h >= 0.0) {
            const double t = -hb - std::sqrt(h);
            if (t >= 0.0) {
                best = std::min(best, t);
            }
        }
    }
    return best <= range ? best : kNever;
}

void TreeWood::build(const std::vector<TreeSpec>& trees) {
    skeletons_.clear();
    for (std::size_t k = 0; k < trees.size(); ++k) {
        skeletons_.push_back(skeletonOf(trees[k], static_cast<std::uint32_t>(k)));
    }
    standing_.clear();
    for (const Skeleton& s : skeletons_) {
        standing_.emplace_back(s.segments.size(), 1.0);
    }
    version_ = 0.0;
}

void TreeWood::reset() {
    for (std::vector<double>& s : standing_) {
        std::fill(s.begin(), s.end(), 1.0);
    }
    version_ = 0.0;
}

void TreeWood::cut(std::uint32_t tree, std::uint32_t segment, double share) {
    if (tree >= standing_.size() || segment >= standing_[tree].size()) {
        return;
    }
    std::vector<double>& standing = standing_[tree];
    const BranchSegment& s = skeletons_[tree].segments[segment];
    standing[segment] = std::min(standing[segment], std::clamp(share, 0.0, 1.0));
    std::fill(standing.begin() + segment + 1, standing.begin() + s.end, 0.0);
}

void TreeWood::load(std::span<const double> values) {
    if (values.size() < kBranchCutHeader || values[0] == version_) {
        return;
    }
    reset();
    version_ = values[0];
    const auto count = std::min(static_cast<std::size_t>(std::max(0.0, values[1])),
                                (values.size() - kBranchCutHeader) / kBranchCutFields);
    for (std::size_t k = 0; k < count; ++k) {
        const std::size_t at = kBranchCutHeader + (kBranchCutFields * k);
        if (!(values[at] >= 0.0) || !(values[at + 1] >= 0.0)) {
            continue;
        }
        cut(static_cast<std::uint32_t>(values[at]), static_cast<std::uint32_t>(values[at + 1]),
            values[at + 2]);
    }
}

bool TreeWood::ray(const TreeSpec& tree, std::size_t index, const V3& origin, const V3& direction,
                   double range, const V3& sway, WoodHit& hit) const {
    if (index >= skeletons_.size()) {
        return false;
    }
    const std::vector<BranchSegment>& segs = skeletons_[index].segments;
    const std::vector<double>& standing = standing_[index];
    if (segs.empty() || !(range > 0.0)) {
        return false;
    }
    const V3 o = inCrown(tree, origin, sway);
    const V3 d = rotate(conjugate(tree.pose.orientation), direction);
    double best = range;
    bool found = false;
    std::size_t which = 0;
    std::size_t i = 0;
    while (i < segs.size()) {
        const BranchSegment& s = segs[i];
        if (standing[i] <= 0.0 || !raySphere(o, d, best, s.centre, s.radius)) {
            i = s.end;
            continue;
        }
        const V3 to = add(s.from, scale(sub(s.to, s.from), standing[i]));
        const double t = rayCapsule(o, d, s.from, to, 0.5 * s.diameter, best);
        if (t <= best) {
            best = t;
            found = true;
            which = i;
        }
        i = standing[i] < 1.0 ? s.end : i + 1;
    }
    if (!found) {
        return false;
    }
    const BranchSegment& s = segs[which];
    const V3 to = add(s.from, scale(sub(s.to, s.from), standing[which]));
    const V3 p = add(o, scale(d, best));
    const V3 axis = sub(to, s.from);
    const double axisSq = dot(axis, axis);
    const double along =
        axisSq > 0.0 ? std::clamp(dot(sub(p, s.from), axis) / axisSq, 0.0, 1.0) : 0.0;
    V3 n = sub(p, add(s.from, scale(axis, along)));
    const double length = norm(n);
    n = length > 0.0 && best > 0.0 ? scale(n, 1.0 / length) : scale(d, -1.0);
    hit.distance = best;
    hit.normal = rotate(tree.pose.orientation, n);
    hit.diameter = s.diameter;
    return true;
}

}  // namespace lagrange
