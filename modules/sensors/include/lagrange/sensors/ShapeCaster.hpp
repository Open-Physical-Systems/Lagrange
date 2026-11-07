#pragma once

// Tier 0 ray casting against collision shapes (docs/design/domains/sensors.md §6).

#include "lagrange/services/SceneQuery.hpp"
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
class ShapeCaster final : public services::SceneQuery {
public:
    struct Shape {
        ShapeType type{ShapeType::Box};
        std::array<double, 3> size{};
        Pose pose;                          // world_local
        const ConvexHull* hull{nullptr};    // a ConvexHull's points; must outlive the caster
        const HeightField* field{nullptr};  // a HeightField's grid; must outlive the caster
    };

    ShapeCaster();
    ~ShapeCaster() override;
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

    /// cast() with the shape hit: the index in the list given to update(), the lowest on a tie.
    [[nodiscard]] services::RayHit castHit(const std::array<double, 3>& origin,
                                           const std::array<double, 3>& dir) const;

    // SceneQuery, over the shapes of the last update().
    [[nodiscard]] services::RayHit ray(const std::array<double, 3>& origin,
                                       const std::array<double, 3>& dir,
                                       double maxDistance) const override;
    [[nodiscard]] std::vector<std::uint32_t> overlapSphere(const std::array<double, 3>& center,
                                                           double radius) const override;

    struct Impl;

private:
    std::unique_ptr<Impl> impl_;
};

/// Distance (m) from `point` to the shape, 0 inside it. A plane is the half-space below it
/// (the ground): 0 under it, the height above it otherwise. For a convex hull it is the
/// distance to the farthest face plane, which is exact over a face and at most the true
/// distance near an edge or corner (overlap queries err on the side of reporting).
[[nodiscard]] double distanceToShape(ShapeType type, const std::array<double, 3>& size,
                                     const Pose& pose, const std::array<double, 3>& point,
                                     const ConvexHull* hull = nullptr);

/// The outward unit normal of the shape at a `point` on its surface. A height field's is its
/// slope there, a convex hull's the nearest face's.
[[nodiscard]] std::array<double, 3> surfaceNormal(ShapeType type, const std::array<double, 3>& size,
                                                  const Pose& pose,
                                                  const std::array<double, 3>& point,
                                                  const ConvexHull* hull = nullptr,
                                                  const HeightField* field = nullptr);

/// What a lidar gets back from a surface (docs/design/domains/sensors.md §2). The intensity is
/// reflected power relative to a white wall (reflectivity 1) head-on at kIntensityReferenceM,
/// capped at 1: ρ·((1−s)·cos θ + s·lobe(θ))·(r₀/r)², where s is the specular fraction and the lobe
/// is a mirror's, narrow about normal incidence. A retroreflector returns kRetroGain·ρ·(r₀/r)² at
/// any angle. A surface with a material gives no return when its power falls under what the
/// lidar needs at its range limit for a target of `referenceReflectivity`, so water and dark
/// surfaces drop out at a distance or a slant; without a material there is always a return.
inline constexpr double kIntensityReferenceM = 1.0;
inline constexpr double kRetroGain = 100.0;        // illustrative: a retroreflector's concentration
inline constexpr double kSpecularLobeRad = 0.035;  // about 2°, illustrative
struct LidarReturn {
    bool detected{true};
    double intensity{0.0};
};
[[nodiscard]] LidarReturn lidarReturn(const SurfaceMaterial& material, bool hasMaterial,
                                      bool wavelength1550, double cosIncidence, double range,
                                      double maxRange, double referenceReflectivity);

/// Radius of a sphere around the shape's origin that contains it (+inf for a plane).
[[nodiscard]] double boundingRadius(ShapeType type, const std::array<double, 3>& size);

}  // namespace lagrange::sensors
