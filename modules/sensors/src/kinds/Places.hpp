#pragma once
// Private to lagrange_sensors: what the vision and floor kinds (detections, landing_target,
// line_sensor, tag_reader; sensors.md §9) share. Things found by the name of a scene prim, body
// poses read from the bus, and a caster over every collider at its body's pose.

#include "KindModule.hpp"
#include "lagrange/sensors/ShapeCaster.hpp"
#include "lagrange/world/Model.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <array>
#include <cctype>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace lagrange::sensors::kinds {
// Its own inline namespace: the kinds' private headers may name helpers alike (ODR).
inline namespace places {

using Vec3 = std::array<double, 3>;

inline Vec3 add(const Vec3& a, const Vec3& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}
inline Vec3 sub(const Vec3& a, const Vec3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
inline Vec3 scaled(const Vec3& a, double k) {
    return {a[0] * k, a[1] * k, a[2] * k};
}
inline double dot(const Vec3& a, const Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
inline double norm(const Vec3& a) {
    return std::sqrt(dot(a, a));
}

/// The pose of frame b expressed in frame a (both in the same parent): a⁻¹ b.
inline Pose relative(const Pose& a, const Pose& b) {
    const std::array<double, 4> inv = conjugate(a.orientation);
    return {rotate(inv, sub(b.position, a.position)), multiply(inv, b.orientation)};
}

/// A point in a frame's coordinates (the inverse of placing it).
inline Vec3 toFrame(const Pose& frame, const Vec3& world) {
    return rotate(conjugate(frame.orientation), sub(world, frame.position));
}

/// A named place of the scene: a prim, where it is, and the box its meshes fill.
struct Place {
    std::string path;
    std::string name;
    std::uint32_t scenePrim{kNoIndex};
    std::uint32_t body{kNoIndex};  // World body it rides on; kNoIndex: fixed in the world
    Pose offset;                   // in the body, or in the world when fixed
    bool hasBox{false};
    Vec3 center{};  // the axis-aligned box, in the prim's frame, of its meshes and its children's
    Vec3 half{};    // half extents, m
};

/// Every prim of `scene` (with the boxes of its meshes and its descendants' on the same body), or
/// without a scene every prim World::prims knows (prims with an API schema; no boxes).
inline std::vector<Place> scenePlaces(const World& world, const Scene* scene) {
    std::vector<Place> out;
    if (scene == nullptr) {
        const World::Prims& prims = world.prims;
        for (std::size_t p = 0; p < prims.paths.size(); ++p) {
            Place place;
            place.path = prims.paths[p];
            place.name = prims.instanceNames[p];
            place.body = prims.body[p] == kWorldBody ? kNoIndex : prims.body[p].index;
            place.offset = prims.poseInBody[p];
            out.push_back(std::move(place));
        }
        return out;
    }
    out.resize(scene->prims.size());
    std::vector<Vec3> lo(out.size(), Vec3{1e300, 1e300, 1e300});
    std::vector<Vec3> hi(out.size(), Vec3{-1e300, -1e300, -1e300});
    for (std::uint32_t p = 0; p < scene->prims.size(); ++p) {
        const ScenePrim& prim = scene->prims[p];
        Place& place = out[p];
        place.path = prim.path;
        place.name = prim.name;
        place.scenePrim = p;
        place.body = prim.body;
        place.offset = prim.offset;
    }
    for (std::uint32_t p = 0; p < scene->prims.size(); ++p) {
        const ScenePrim& prim = scene->prims[p];
        if (prim.mesh == kNoIndex || prim.mesh >= scene->meshes.size()) {
            continue;
        }
        const std::vector<float>& points = scene->meshes[prim.mesh].positions;
        if (points.size() < 3) {
            continue;
        }
        Vec3 mlo{1e300, 1e300, 1e300};
        Vec3 mhi{-1e300, -1e300, -1e300};
        for (std::size_t i = 0; i + 2 < points.size(); i += 3) {
            for (std::size_t k = 0; k < 3; ++k) {
                const double v = static_cast<double>(points[i + k]) * prim.scale.at(k);
                mlo.at(k) = std::min(mlo.at(k), v);
                mhi.at(k) = std::max(mhi.at(k), v);
            }
        }
        // Into this prim and every ancestor on the same body: the 8 corners in its frame.
        for (std::uint32_t a = p; a != kNoIndex && a < scene->prims.size();
             a = scene->prims[a].parent) {
            if (scene->prims[a].body != prim.body) {
                break;
            }
            const Pose in = relative(scene->prims[a].offset, prim.offset);
            for (int c = 0; c < 8; ++c) {
                const Vec3 corner{(c & 1) != 0 ? mhi[0] : mlo[0], (c & 2) != 0 ? mhi[1] : mlo[1],
                                  (c & 4) != 0 ? mhi[2] : mlo[2]};
                const Vec3 q = add(in.position, rotate(in.orientation, corner));
                for (std::size_t k = 0; k < 3; ++k) {
                    lo[a].at(k) = std::min(lo[a].at(k), q.at(k));
                    hi[a].at(k) = std::max(hi[a].at(k), q.at(k));
                }
            }
            out[a].hasBox = true;
        }
    }
    for (std::size_t p = 0; p < out.size(); ++p) {
        if (out[p].hasBox) {
            for (std::size_t k = 0; k < 3; ++k) {
                out[p].center.at(k) = (lo[p].at(k) + hi[p].at(k)) / 2.0;
                out[p].half.at(k) = (hi[p].at(k) - lo[p].at(k)) / 2.0;
            }
        }
    }
    return out;
}

inline std::string lowerCase(std::string_view text) {
    std::string out(text);
    for (char& c : out) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return out;
}

/// The words of a name split at '_', '-', '.', ' ' and '/', lower case.
inline std::vector<std::string> words(std::string_view name) {
    std::vector<std::string> out;
    std::string word;
    for (const char c : name) {
        if (c == '_' || c == '-' || c == '.' || c == ' ' || c == '/') {
            if (!word.empty()) {
                out.push_back(lowerCase(word));
            }
            word.clear();
        } else {
            word.push_back(c);
        }
    }
    if (!word.empty()) {
        out.push_back(lowerCase(word));
    }
    return out;
}

/// `name` is the prim's name, its whole path, or the end of its path ("pad" never matches
/// "landing_pad"; "open_field/landing_pad" does).
inline bool namesPlace(const Place& place, std::string_view name) {
    if (name.empty()) {
        return false;
    }
    if (place.name == name || place.path == name) {
        return true;
    }
    return place.path.size() > name.size() &&
           std::string_view(place.path).substr(place.path.size() - name.size()) == name &&
           place.path[place.path.size() - name.size() - 1] == '/';
}

/// Body poses from the bus: `body/<b>/pose` for the bodies that move, the rest pose otherwise.
class BodyPoses {
public:
    void begin(const World& world) {
        moving_ = movingBodies(world);
        rest_ = restPoses(world);
        ids_.assign(moving_.size(), SignalId{});
        names_ = world.bodies.names;
    }
    /// Declares the pose input of body `b` if it moves (kNoIndex: nothing).
    void watch(std::uint32_t b, SignalDeclarer& d) {
        if (b < moving_.size() && moving_[b] && !ids_[b].valid()) {
            SignalSpec s =
                kindSpec("body/" + names_[b] + "/pose", SignalType::Pose, units::m, "world_local");
            ids_[b] = d.input(std::move(s));
        }
    }
    [[nodiscard]] Pose pose(const SignalBus& bus, std::uint32_t b) const {
        if (b >= ids_.size()) {
            return {};
        }
        if (!ids_[b].valid()) {
            return rest_[b];
        }
        const std::span<const double> v = bus.readN(ids_[b]);
        return {{v[0], v[1], v[2]}, {v[3], v[4], v[5], v[6]}};
    }
    /// The world pose of a place.
    [[nodiscard]] Pose place(const SignalBus& bus, const Place& p) const {
        return p.body == kNoIndex ? p.offset : compose(pose(bus, p.body), p.offset);
    }

private:
    std::vector<bool> moving_;
    std::vector<Pose> rest_;
    std::vector<SignalId> ids_;
    std::vector<std::string> names_;
};

/// Every collider at its body's pose, for ray casts (occlusion, the floor under a sensor). Exact
/// and deterministic (ShapeCaster, the CPU reference).
class Colliders {
public:
    /// Watches the bodies of the moving colliders; call in declare() after poses.begin().
    void declare(const World& world, BodyPoses& poses, SignalDeclarer& d) {
        world_ = &world;
        for (const BodyId owner : world.colliders.body) {
            if (owner != kWorldBody) {
                poses.watch(owner.index, d);
            }
        }
    }
    /// Places the shapes; call at every step before cast().
    void update(const SignalBus& bus, const BodyPoses& poses) {
        const World::Colliders& c = world_->colliders;
        shapes_.resize(c.body.size());
        for (std::size_t k = 0; k < c.body.size(); ++k) {
            const BodyId owner = c.body[k];
            ShapeCaster::Shape& s = shapes_[k];
            s.type = c.shape[k];
            s.size = c.size[k];
            s.pose = owner == kWorldBody ? c.poseInBody[k]
                                         : compose(poses.pose(bus, owner.index), c.poseInBody[k]);
            s.hull = c.shape[k] == ShapeType::ConvexHull ? &world_->hulls.at(c.hull[k]) : nullptr;
            s.field = c.shape[k] == ShapeType::HeightField ? &world_->heightfields.at(c.hull[k])
                                                           : nullptr;
        }
        caster_.update(shapes_);
    }
    /// The nearest collider along the unit `dir`: its index in World::colliders and distance.
    [[nodiscard]] services::RayHit cast(const Vec3& origin, const Vec3& dir) const {
        if (shapes_.empty()) {
            return {};
        }
        return caster_.castHit(origin, dir);
    }

    /// cast(), seeing through the colliders of `ownBody` (the sensor's housing and chassis).
    [[nodiscard]] services::RayHit castPast(const Vec3& origin, const Vec3& dir,
                                            std::uint32_t ownBody) const {
        constexpr int kMaxSkips = 8;
        constexpr double kStep = 1e-6;  // m past the surface left behind
        double travelled = 0.0;
        Vec3 from = origin;
        for (int k = 0; k < kMaxSkips; ++k) {
            services::RayHit hit = cast(from, dir);
            if (!hit.hit()) {
                return hit;
            }
            const BodyId owner = world_->colliders.body[hit.shape];
            if (owner == kWorldBody || owner.index != ownBody) {
                hit.distance += travelled;
                return hit;
            }
            travelled += hit.distance + kStep;
            from = add(from, scaled(dir, hit.distance + kStep));
        }
        return {};
    }

    /// Whether the line from `target` to `sensor` is clear. Cast from the target's side, starting
    /// `margin` out (the target's own collider does not hide it): hidden only when the first
    /// collider met is on another body than the sensor's (`ownBody`: a camera looks out of its
    /// own housing) and more than `margin` short of the sensor.
    [[nodiscard]] bool clear(const Vec3& target, const Vec3& sensor, double margin,
                             std::uint32_t ownBody) const {
        const Vec3 line = sub(sensor, target);
        const double distance = norm(line);
        if (distance <= 0.0 || shapes_.empty()) {
            return true;
        }
        // Start `margin` along the line, so that the target's own collider (its top face at the
        // target point) does not count.
        const Vec3 dir = scaled(line, 1.0 / distance);
        const Vec3 start = add(target, scaled(dir, std::min(margin, distance)));
        const services::RayHit hit = caster_.castHit(start, dir);
        if (!hit.hit() || hit.distance >= distance - margin) {
            return true;
        }
        const BodyId owner = world_->colliders.body[hit.shape];
        return owner != kWorldBody && owner.index == ownBody;
    }

private:
    const World* world_{nullptr};
    std::vector<ShapeCaster::Shape> shapes_;
    ShapeCaster caster_;
};

/// A module that refuses to start with `reason`: a kind whose parameters name nothing.
class RefuseKind final : public KindModule {
public:
    RefuseKind(const World& world, std::uint32_t sensor, std::string reason)
        : KindModule(world, sensor, false), reason_(std::move(reason)) {}
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer&) override {
        return Status::error(StatusCode::InvalidConfig, reason_);
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus&) override { return {}; }
    Status step(const StepContext&, SignalBus&) override { return {}; }

private:
    std::string reason_;
};

inline std::unique_ptr<IModule> refuseKind(const World& world, std::uint32_t sensor,
                                           std::string reason) {
    return std::make_unique<RefuseKind>(world, sensor, std::move(reason));
}

}  // namespace places
}  // namespace lagrange::sensors::kinds
