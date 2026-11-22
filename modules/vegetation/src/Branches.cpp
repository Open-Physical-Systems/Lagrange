#include "lagrange/vegetation/Branches.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
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

/// A stub shorter than this is not cut again (m), as for the grass (kMinCutPiece).
constexpr double kMinPiece = 0.01;

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
    skeletons_.clear();
    for (std::size_t k = 0; k < trees.size(); ++k) {
        skeletons_.push_back(skeletonOf(trees[k], static_cast<std::uint32_t>(k)));
    }
    standing_.clear();
    for (const Skeleton& s : skeletons_) {
        standing_.emplace_back(s.segments.size(), 1.0);
    }
    cuts_.clear();
    rotors_.clear();
    twigsOut_.clear();
    twigs_.assign(trees.size(), 0.0);
}

void Branches::addRotor(SignalDeclarer& d, const std::string& joint,
                        const std::array<double, 3>& linkCom) {
    RotorState r;
    r.com = linkCom;
    if (!skeletons_.empty()) {
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
}

void Branches::init(SignalBus& bus) {
    for (std::size_t k = 0; k < skeletons_.size(); ++k) {
        std::fill(standing_[k].begin(), standing_[k].end(), 1.0);
    }
    cuts_.clear();
    std::fill(twigs_.begin(), twigs_.end(), 0.0);
    for (RotorState& r : rotors_) {
        r.damage = 0.0;
        r.strikes = 0.0;
    }
    publish(bus);
}

double Branches::standing(std::size_t tree, std::size_t s) const {
    return standing_.at(tree).at(s);
}

void Branches::apply(const Cut& cut) {
    std::vector<double>& standing = standing_.at(cut.tree);
    const BranchSegment& s = skeletons_.at(cut.tree).segments.at(cut.segment);
    standing[cut.segment] = std::min(standing[cut.segment], cut.share);
    std::fill(standing.begin() + cut.segment + 1, standing.begin() + s.end, 0.0);
}

BranchHit Branches::sweep(std::size_t rotor, const std::vector<TreeSpec>& trees, const Pose& hub,
                          const std::array<double, 3>& axis, double radius,
                          const std::vector<std::array<double, 3>>& sways, double omega,
                          double inertia, double dt) {
    BranchHit hit;
    if (rotor >= rotors_.size() || skeletons_.empty()) {
        return hit;
    }
    std::uint32_t strikeTree = 0;
    std::uint32_t strikeSegment = 0;
    V3 strikeAt{};
    for (std::size_t k = 0; k < trees.size() && k < skeletons_.size(); ++k) {
        const TreeSpec& tree = trees[k];
        const std::vector<BranchSegment>& segs = skeletons_[k].segments;
        if (segs.empty()) {
            continue;
        }
        const V3 o = inCrown(tree, hub.position, sways[k]);
        const V3 n = rotate(conjugate(tree.pose.orientation), axis);
        if (norm(sub(o, segs[0].centre)) > segs[0].radius + radius + kBladeHalfThickness) {
            continue;  // far from this tree's wood
        }
        std::vector<double>& standing = standing_[k];
        std::size_t i = 0;
        while (i < segs.size()) {
            const BranchSegment& s = segs[i];
            if (standing[i] <= 0.0) {
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
            const double reach = standing[i] * length;
            const double at =
                discMeets(s.from, add(s.from, scale(full, standing[i])), s.diameter, o, n, radius);
            if (at >= 0.0 && at < reach - kMinPiece) {
                if (s.diameter <= tree.maxCutDiameter) {
                    const Cut cut{static_cast<std::uint32_t>(k), static_cast<std::uint32_t>(i),
                                  at / length};
                    cuts_.push_back(cut);
                    apply(cut);
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
            i = standing[i] < 1.0 ? s.end : i + 1;
        }
    }
    if (!(hit.diameter > 0.0) || !(inertia > 0.0) || !(dt > 0.0) || omega == 0.0) {
        return hit;
    }
    // The strike: back to the world.
    const TreeSpec& tree = trees[strikeTree];
    const BranchSegment& s = skeletons_[strikeTree].segments[strikeSegment];
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
    for (std::size_t k = 0; k < skeletons_.size(); ++k) {
        std::fill(standing_[k].begin(), standing_[k].end(), 1.0);
    }
    cuts_.clear();
    for (std::uint64_t k = 0; k < count; ++k) {
        Cut c{};
        if (!reader.get(c.tree) || !reader.get(c.segment) || !reader.get(c.share) ||
            c.tree >= skeletons_.size() || c.segment >= skeletons_[c.tree].segments.size()) {
            return false;
        }
        cuts_.push_back(c);
        apply(c);
    }
    return true;
}

}  // namespace lagrange::vegetation
