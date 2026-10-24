#pragma once

// Tier 0 ray casting against collision shapes (docs/design/domains/sensors.md §6).

#include "lagrange/world/World.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace lagrange::sensors {

/// The nearest hit of a ray among collision shapes, exactly as raycast() over every shape: the
/// same closed-form tests in double, so results are bit-identical to the brute-force reference
/// (invariant 7). Embree's bounding-volume hierarchy, over a conservative bounding sphere per
/// bounded shape, only skips the shapes a ray cannot reach. Unbounded planes are tested
/// directly. The hierarchy is refitted when shapes move (update()).
///
/// cast() may run on many threads at once between update() calls.
class ShapeCaster {
public:
    struct Shape {
        ShapeType type{ShapeType::Box};
        std::array<double, 3> size{};
        Pose pose;  // world_local
    };

    ShapeCaster();
    ~ShapeCaster();
    ShapeCaster(const ShapeCaster&) = delete;
    ShapeCaster& operator=(const ShapeCaster&) = delete;
    ShapeCaster(ShapeCaster&&) = delete;
    ShapeCaster& operator=(ShapeCaster&&) = delete;

    /// Places the shapes. The first call builds the hierarchy; later calls must pass the same
    /// shapes (types and sizes) in the same order and refit only the ones that moved.
    void update(std::span<const Shape> shapes);

    /// Distance along the unit direction `dir` from `origin` to the nearest shape, or kNoHit.
    [[nodiscard]] double cast(const std::array<double, 3>& origin,
                              const std::array<double, 3>& dir) const;

    struct Impl;

private:
    std::unique_ptr<Impl> impl_;
};

/// Radius of a sphere around the shape's origin that contains it (+inf for a plane).
[[nodiscard]] double boundingRadius(ShapeType type, const std::array<double, 3>& size);

}  // namespace lagrange::sensors
