#pragma once

// Scene queries over the collision shapes (roadmap 0.2-C4): what a ray meets, and what lies
// within reach of a point. Sensors cast through it; contacts, comms and dose will query it.

#include <array>
#include <cstdint>
#include <limits>
#include <vector>

namespace lagrange::services {

/// The nearest shape along a ray.
struct RayHit {
    static constexpr std::uint32_t kNone = std::numeric_limits<std::uint32_t>::max();
    double distance{std::numeric_limits<double>::infinity()};  // m along the unit direction
    std::uint32_t shape{kNone};  // index in the implementation's shape list
    [[nodiscard]] bool hit() const { return shape != kNone; }
};

/// Read-only queries against a set of collision shapes in the region-local frame (m). The
/// shape indices are those of the implementation's shape list (World::Colliders order for the
/// lidar's ShapeCaster). Implementations are exact and deterministic: ties go to the lower
/// shape index. Safe to call from several threads at once between updates of the scene.
class SceneQuery {
public:
    SceneQuery() = default;
    SceneQuery(const SceneQuery&) = delete;
    SceneQuery& operator=(const SceneQuery&) = delete;
    SceneQuery(SceneQuery&&) = delete;
    SceneQuery& operator=(SceneQuery&&) = delete;
    virtual ~SceneQuery() = default;

    /// The first shape along the unit direction `dir` from `origin`, within `maxDistance`. A ray
    /// starting inside a shape reports where it leaves it.
    [[nodiscard]] virtual RayHit ray(const std::array<double, 3>& origin,
                                     const std::array<double, 3>& dir,
                                     double maxDistance) const = 0;

    /// Every shape within `radius` of `center` (touching counts), in ascending index order.
    [[nodiscard]] virtual std::vector<std::uint32_t> overlapSphere(
        const std::array<double, 3>& center, double radius) const = 0;
};

}  // namespace lagrange::services
