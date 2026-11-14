#pragma once
// Private to lagrange_sensors: the colliders a built-in kind casts rays against, placed at the
// body poses it reads from the bus as declared inputs (invariant 1), with a filter on which
// colliders count (a sensor skips its own body; a radio counts only walls).

#include "KindModule.hpp"
#include "lagrange/sensors/ShapeCaster.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <limits>
#include <span>
#include <string>
#include <vector>

namespace lagrange::sensors::kinds {
// Its own inline namespace: the kinds' private headers may name helpers alike (ODR).
inline namespace obstacle_scene {

using Vec3 = std::array<double, 3>;

/// Body poses (and twists) as inputs: rest poses for bodies that never move.
class BodyPoses {
public:
    /// Remembers the rest poses; call before watch().
    void reset(const World& world) {
        rest_ = restPoses(world);
        moving_ = movingBodies(world);
        names_ = &world.bodies.names;
        poses_.assign(rest_.size(), SignalId{});
        twists_.assign(rest_.size(), SignalId{});
    }

    /// Declares body/<b>/pose (and /twist with `twist`) when the body moves.
    void watch(SignalDeclarer& d, std::uint32_t body, bool twist = false) {
        if (body >= rest_.size() || !moving_[body]) {
            return;
        }
        const std::string base = "body/" + (*names_)[body];
        if (!poses_[body].valid()) {
            poses_[body] =
                d.input(kindSpec(base + "/pose", SignalType::Pose, units::m, "world_local"));
        }
        if (twist && !twists_[body].valid()) {
            twists_[body] = d.input(
                kindSpec(base + "/twist", SignalType::Wrench, units::m_per_s, "world_local"));
        }
    }

    [[nodiscard]] bool moving(std::uint32_t body) const {
        return body < moving_.size() && moving_[body];
    }

    [[nodiscard]] Pose pose(const SignalBus& bus, std::uint32_t body) const {
        if (!poses_[body].valid()) {
            return rest_[body];
        }
        const std::span<const double> v = bus.readN(poses_[body]);
        return {{v[0], v[1], v[2]}, {v[3], v[4], v[5], v[6]}};
    }

    /// The world velocity of point `p` of `body`: v + ω × (p − origin); zero for a still body.
    [[nodiscard]] Vec3 pointVelocity(const SignalBus& bus, std::uint32_t body,
                                     const Vec3& p) const {
        if (body >= twists_.size() || !twists_[body].valid()) {
            return {};
        }
        const std::span<const double> t = bus.readN(twists_[body]);
        const Pose at = pose(bus, body);
        const Vec3 r{p[0] - at.position[0], p[1] - at.position[1], p[2] - at.position[2]};
        return {t[0] + t[4] * r[2] - t[5] * r[1], t[1] + t[5] * r[0] - t[3] * r[2],
                t[2] + t[3] * r[1] - t[4] * r[0]};
    }

private:
    std::vector<Pose> rest_;
    std::vector<bool> moving_;
    const std::vector<std::string>* names_{nullptr};
    std::vector<SignalId> poses_, twists_;
};

/// The colliders that pass a filter, placed at the poses of a BodyPoses before each cast.
class ObstacleScene {
public:
    using Filter = std::function<bool(std::uint32_t collider)>;

    /// Picks the colliders and declares the poses of the moving bodies that own them.
    void declare(const World& world, BodyPoses& poses, SignalDeclarer& d, const Filter& keep) {
        world_ = &world;
        colliders_.clear();
        const World::Colliders& c = world.colliders;
        for (std::uint32_t k = 0; k < c.body.size(); ++k) {
            if (keep && !keep(k)) {
                continue;
            }
            colliders_.push_back(k);
            if (c.body[k] != kWorldBody) {
                poses.watch(d, c.body[k].index);
            }
        }
        shapes_.assign(colliders_.size(), {});
        placed_ = false;
    }

    /// Places the shapes at the poses on the bus now.
    void place(const SignalBus& bus, const BodyPoses& poses) {
        if (colliders_.empty()) {
            return;
        }
        const World::Colliders& c = world_->colliders;
        for (std::size_t i = 0; i < colliders_.size(); ++i) {
            const std::uint32_t k = colliders_[i];
            ShapeCaster::Shape& s = shapes_[i];
            s.type = c.shape[k];
            s.size = c.size[k];
            s.hull = c.shape[k] == ShapeType::ConvexHull ? &world_->hulls.at(c.hull[k]) : nullptr;
            s.field = c.shape[k] == ShapeType::HeightField ? &world_->heightfields.at(c.hull[k])
                                                           : nullptr;
            s.pose = c.body[k] == kWorldBody
                         ? c.poseInBody[k]
                         : compose(poses.pose(bus, c.body[k].index), c.poseInBody[k]);
        }
        caster_.update(shapes_);
        placed_ = true;
    }

    [[nodiscard]] bool empty() const { return colliders_.empty(); }
    [[nodiscard]] std::size_t size() const { return colliders_.size(); }
    /// The World::Colliders index of shape `i`.
    [[nodiscard]] std::uint32_t collider(std::uint32_t i) const { return colliders_[i]; }

    /// The nearest shape along the unit `dir` within `maxDistance`; no hit before place().
    [[nodiscard]] services::RayHit ray(const Vec3& origin, const Vec3& dir,
                                       double maxDistance) const {
        if (!placed_ || colliders_.empty()) {
            return {};
        }
        return caster_.ray(origin, dir, maxDistance);
    }

    /// The shapes within `radius` of `center` (touching counts), ascending; empty before place().
    [[nodiscard]] std::vector<std::uint32_t> overlap(const Vec3& center, double radius) const {
        if (!placed_ || colliders_.empty()) {
            return {};
        }
        return caster_.overlapSphere(center, radius);
    }

    /// Whether `p` is inside shape `i` (0 distance).
    [[nodiscard]] bool inside(std::uint32_t i, const Vec3& p) const {
        const ShapeCaster::Shape& s = shapes_[i];
        return distanceToShape(s.type, s.size, s.pose, p, s.hull) <= 0.0;
    }

    /// The shapes the straight segment a → b crosses: how many it enters and the length of the
    /// segment inside them (m). A shape the segment starts or ends in counts too.
    struct Crossing {
        int entered{0};
        double inside{0.0};
    };
    [[nodiscard]] Crossing crossing(const Vec3& a, const Vec3& b) const {
        Crossing out;
        const Vec3 ab{b[0] - a[0], b[1] - a[1], b[2] - a[2]};
        const double length = std::sqrt(ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2]);
        if (!placed_ || colliders_.empty() || length <= 0.0) {
            return out;
        }
        const Vec3 dir{ab[0] / length, ab[1] / length, ab[2] / length};
        constexpr double kStep = 1e-6;  // m past an entry or exit before the next ray
        constexpr int kMaxSurfaces = 256;
        double t = 0.0;
        double enteredAt = -1.0;  // where the segment went into a shape, while inside one
        const auto at = [&](double s) {
            return Vec3{a[0] + dir[0] * s, a[1] + dir[1] * s, a[2] + dir[2] * s};
        };
        for (int n = 0; n < kMaxSurfaces && t < length; ++n) {
            const services::RayHit hit = caster_.ray(at(t), dir, length - t);
            if (!hit.hit()) {
                break;
            }
            t += hit.distance + kStep;
            const bool nowInside = inside(hit.shape, at(t));
            if (nowInside && enteredAt < 0.0) {
                ++out.entered;
                enteredAt = t - kStep;
            } else if (!nowInside && enteredAt >= 0.0) {
                out.inside += t - kStep - enteredAt;
                enteredAt = -1.0;
            }
        }
        if (enteredAt >= 0.0) {
            out.inside += length - enteredAt;
        }
        return out;
    }

private:
    const World* world_{nullptr};
    std::vector<std::uint32_t> colliders_;
    std::vector<ShapeCaster::Shape> shapes_;
    ShapeCaster caster_;
    bool placed_{false};
};

/// A three-number text parameter, "x y z" (also comma separated), into `out`.
inline bool parseVec3(std::string_view text, Vec3& out) {
    std::string s(text);
    for (char& ch : s) {
        if (ch == ',') {
            ch = ' ';
        }
    }
    const char* p = s.c_str();
    for (double& v : out) {
        char* end = nullptr;
        v = std::strtod(p, &end);
        if (end == p) {
            return false;
        }
        p = end;
    }
    while (*p == ' ' || *p == '\t' || *p == '\n') {
        ++p;
    }
    return *p == '\0';
}

/// "x y z; x y z; ..." into points; false on a malformed one.
inline bool parsePoints(std::string_view text, std::vector<Vec3>& out) {
    std::size_t start = 0;
    while (start <= text.size()) {
        const std::size_t end = std::min(text.find(';', start), text.size());
        const std::string_view part = text.substr(start, end - start);
        if (part.find_first_not_of(" \t\n\r") != std::string_view::npos) {
            Vec3 v{};
            if (!parseVec3(part, v)) {
                return false;
            }
            out.push_back(v);
        }
        start = end + 1;
    }
    return true;
}

/// Space separated words of a text parameter.
inline std::vector<std::string> words(std::string_view text) {
    std::vector<std::string> out;
    std::size_t i = 0;
    while (i < text.size()) {
        while (i < text.size() &&
               (text[i] == ' ' || text[i] == ',' || text[i] == '\t' || text[i] == '\n')) {
            ++i;
        }
        std::size_t j = i;
        while (j < text.size() && text[j] != ' ' && text[j] != ',' && text[j] != '\t' &&
               text[j] != '\n') {
            ++j;
        }
        if (j > i) {
            out.emplace_back(text.substr(i, j - i));
        }
        i = j;
    }
    return out;
}

/// A point named in the world: on the world (`fixed`, `local` region-local) or in a body.
struct NamedPlace {
    bool fixed{true};
    std::uint32_t body{0};
    Vec3 local{};
};

/// Every prim and every body whose name starts with `prefix`, in prim order then body order; a
/// body's own prim is the body (at its origin), not a point at the prim table's pose.
inline std::vector<NamedPlace> namedPlaces(const World& world, std::string_view prefix) {
    std::vector<NamedPlace> out;
    std::vector<std::int64_t> bodyOfPrim(world.prims.instanceNames.size(), -1);
    for (std::size_t b = 0; b < world.bodies.prim.size(); ++b) {
        if (world.bodies.prim[b].index < bodyOfPrim.size()) {
            bodyOfPrim[world.bodies.prim[b].index] = static_cast<std::int64_t>(b);
        }
    }
    std::vector<bool> done(world.bodies.names.size(), false);
    for (std::uint32_t p = 0; p < world.prims.instanceNames.size(); ++p) {
        if (!world.prims.instanceNames[p].starts_with(prefix)) {
            continue;
        }
        if (bodyOfPrim[p] >= 0) {
            const auto b = static_cast<std::uint32_t>(bodyOfPrim[p]);
            done[b] = true;
            out.push_back({false, b, {0.0, 0.0, 0.0}});
            continue;
        }
        const BodyId b = world.prims.body[p];
        out.push_back(
            {b == kWorldBody, b == kWorldBody ? 0U : b.index, world.prims.poseInBody[p].position});
    }
    for (std::uint32_t b = 0; b < world.bodies.names.size(); ++b) {
        if (!done[b] && world.bodies.names[b].starts_with(prefix)) {
            out.push_back({false, b, {0.0, 0.0, 0.0}});
        }
    }
    return out;
}

inline constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

/// An optional input: unconnected (its domain off or absent) it reads `fallback`.
inline SignalSpec optionalKindSpec(std::string name, SignalType type, Unit unit,
                                   double fallback = kNaN, std::string frame = {}) {
    SignalSpec s = kindSpec(std::move(name), type, unit, std::move(frame));
    s.optional = true;
    s.defaultValue = fallback;
    return s;
}

}  // namespace obstacle_scene
}  // namespace lagrange::sensors::kinds
