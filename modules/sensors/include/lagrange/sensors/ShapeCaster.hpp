#pragma once

// Tier 0 ray casting against collision shapes (docs/design/domains/sensors.md §6).

#include "lagrange/services/SceneQuery.hpp"
#include "lagrange/world/CanopyCut.hpp"
#include "lagrange/world/Leaves.hpp"
#include "lagrange/world/Trees.hpp"
#include "lagrange/world/World.hpp"

#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <vector>

namespace lagrange::sensors {

/// The nearest hit of a ray among collision shapes, exactly as raycast() over every shape: the
/// same closed-form tests in double, so results are bit-identical to the brute-force reference
/// (invariant 7). Embree's bounding-volume hierarchy, over a conservative bounding sphere per
/// bounded shape, only skips the shapes a ray cannot reach. Unbounded planes are tested
/// directly. Shapes that have never moved sit in a hierarchy built once; the ones that have
/// moved in a second one, rebuilt when they move (update()).
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

    // SceneQuery, over the shapes of the last update(); ray() also gives the surface normal at
    // the hit (surfaceNormal()), which castHit() leaves out.
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

/// Lidar over grass (docs/physics/ground.md "Lidar over grass"): a surface with a canopy has
/// blades standing `canopyHeight` over the collider, which is the soil. A beam that reaches the
/// soil with incidence cosine c crossed L = min(h / c, range) of canopy. The blades are a turbid
/// medium: the chance of a free path s into it is exp(-mu s), mu = G LAI / h, the gap fraction
/// of Nilson (1971) per unit length (Beer-Lambert; G = 0.5 for a spherical leaf angle
/// distribution, Ross 1981). The return comes from the first blade, L - s before the soil, or
/// from the soil when s >= L. Grazing beams (long L) end in the blade tops, steep ones go deeper.
/// The mean shortening is L - (1 - exp(-mu L)) / mu (canopyMeanShortening).
inline constexpr double kCanopyProjection = 0.5;  // G, spherical leaf angles (Ross 1981)
/// One-sided leaf area per ground area of the blades. TODO(verify): a lawn's LAI against turf
/// measurements (e.g. Lee et al. 2011, Crop Sci.); 3 is an order of magnitude, not a measurement.
inline constexpr double kCanopyLeafAreaIndex = 3.0;
/// How far before the hit the return comes, for a uniform draw u in [0, 1): 0 when the beam
/// reaches the soil or the surface has no canopy.
[[nodiscard]] double canopyShortening(double canopyHeight, double cosIncidence, double range,
                                      double u);
/// The mean of canopyShortening over u, in closed form.
[[nodiscard]] double canopyMeanShortening(double canopyHeight, double cosIncidence, double range);
/// The same with the surface's own leaf area index (SurfaceMaterial::leafAreaIndex), mu =
/// G LAI / h; the overloads above take kCanopyLeafAreaIndex.
[[nodiscard]] double canopyShortening(double canopyHeight, double leafAreaIndex,
                                      double cosIncidence, double range, double u);
[[nodiscard]] double canopyMeanShortening(double canopyHeight, double leafAreaIndex,
                                          double cosIncidence, double range);

/// Reeds and shrubs (docs/physics/ground.md "Reeds and shrubs"): a canopy as a volume a beam can
/// cross without reaching the soil, e.g. a drone's lidar sweeping level over a reed bank. In the
/// collider's frame (z up) it is the box's top face extruded by the canopy height,
/// [-hx, hx] x [-hy, hy] x [hz, hz + h], or the slab 0 <= z <= h over a plane. Other shapes have
/// no volume here (they keep canopyShortening at their hit). canopyVolumeSpan gives the beam's
/// path [enter, leave] through it within [0, range] (origin and unit direction in the world,
/// `placed` the collider's pose), or false when it misses.
[[nodiscard]] bool canopyVolumeSpan(ShapeType type, const std::array<double, 3>& size,
                                    const Pose& placed, const std::array<double, 3>& origin,
                                    const std::array<double, 3>& direction, double canopyHeight,
                                    double range, double& enter, double& leave);
/// Where a beam crossing a canopy over [enter, leave] stops in it, for a uniform draw u: the
/// free path s has P(s > x) = exp(-mu x), mu = G LAI / h (the gap probability), so it stops at
/// enter + s when that is before `leave`, else never (+inf).
[[nodiscard]] double canopyVolumeStop(double enter, double leave, double canopyHeight,
                                      double leafAreaIndex, double u);
/// A leaf hit in a canopy volume faces the beam at random: the mean |cos| of a spherical leaf
/// angle distribution, which is G (Ross 1981), stands for the incidence cosine of its return.
inline constexpr double kCanopyLeafCosine = kCanopyProjection;

/// The canopy's shape (docs/physics/ground.md "The canopy's shape"). The functions above treat a
/// canopy as an even layer of one height, which a lidar sees as a box: a flat top at exactly
/// canopyHeight, walls straight up the collider's edges, points spread evenly through it. Real
/// plants are not like that, so the lidar uses the following instead:
///
/// 1. Height by place. The local height is h(x, y) = H f j g e: f = 1 + spread (2 n1 - 1) from a
///    smooth value noise n1 in [0, 1] at the form's patch size (tufts of grass, clumps of reeds,
///    bushes), j = 1 + jitter (2 n4 - 1) the same at the size of single stems (a ragged top, not
///    a smooth dome), g a bare-ground mask from a second noise (gaps between plants), e a taper at
///    a box's edges whose inner limit wanders with a third noise (a ragged border, not a wall). The
///    noise is a function of the collider-frame position only, so every run and every tier sees the
///    same plants. The leaf area per ground area scales with the height, LAI f j g e, so a taller
///    tuft holds more leaf and the bulk density stays G LAI / H.
/// 2. Leaf area by height. The leaf area density at height zeta = z / h(x, y) is
///    (LAI_local / h) p(zeta), with p integrating to 1 over [0, 1] (canopyLeafProfile): grass
///    dense low and thinning to the tips, reeds stems all the way with most leaf in the upper
///    half, shrubs a crown over bare stems. The beam's optical depth G a(x) ds is integrated
///    along it (midpoint steps a twelfth of the height or half a stem across, whichever is
///    shorter for the beam's slope), and it stops where it reaches -ln(1 - u).
/// Every number of the forms is TODO(verify) (canopyFormOf).
struct CanopyFormShape {
    double patch;   // m, the size of a tuft, clump or bush
    double spread;  // relative height variation of the patches, f in [1 - spread, 1 + spread]
    double gap;     // noise level below which the ground is bare (about the bare share)
    double edge;    // m, how far a box's border wanders inward
    double stem;    // m, the scale of single stems and blades within a patch
    double jitter;  // relative height variation from stem to stem, [1 - jitter, 1 + jitter]
};
[[nodiscard]] CanopyFormShape canopyFormOf(CanopyForm form);
/// The share of the leaf area at relative height zeta in [0, 1]; integrates to 1.
[[nodiscard]] double canopyLeafProfile(CanopyForm form, double zeta);
/// The plants' local height factor f j g e at (x, y) in the collider's frame; `edgeDistance` is how
/// far (x, y) lies inside the box's top face (+inf for a plane or a surface without edges).
[[nodiscard]] double canopyHeightFactor(const SurfaceMaterial& material, double x, double y,
                                        double edgeDistance);
/// The tallest plants: canopyHeight (1 + spread) (1 + jitter); the volume a beam can stop in is
/// this high.
[[nodiscard]] double canopyTallest(const SurfaceMaterial& material);
/// Where a beam crossing the canopy volume over [enter, leave] (canopyVolumeSpan with
/// canopyTallest) stops in the shaped canopy, for a uniform draw u: +inf when it goes through.
/// With `cuts`, the plants of collider `collider` stand no taller than its cut map says
/// (lagrange/world/CanopyCut.hpp; ground.md "Cutting the plants"): the stubble keeps the leaves
/// it had under the cut.
[[nodiscard]] double canopyShapedStop(const SurfaceMaterial& material, ShapeType type,
                                      const std::array<double, 3>& size, const Pose& placed,
                                      const std::array<double, 3>& origin,
                                      const std::array<double, 3>& direction, double enter,
                                      double leave, double u, const CanopyCuts* cuts = nullptr,
                                      std::uint32_t collider = 0);
/// canopyShortening in the shaped canopy, for a hit at (x, y) on a surface without a volume
/// (meshes, height fields): the local height and the form's leaf profile, under `cutTop` (m over
/// the soil; +inf uncut).
[[nodiscard]] double canopyShapedShortening(
    const SurfaceMaterial& material, double x, double y, double cosIncidence, double range,
    double u, double cutTop = std::numeric_limits<double>::infinity());

/// Trees (docs/physics/ground.md "Trees"): where a beam stops in a crown, a turbid ellipsoid of
/// leaves (lagrange/world/Trees.hpp), for a uniform draw u: its free path through the leaves
/// has P(s > x) = exp(-mu x), mu = G LAD (the gap probability), counted only through cells the
/// propellers have not cut away. The crown stands at its sway (world, m); the beam is marched in
/// steps of half a cut cell between where it enters and leaves the ellipsoid, within [0, range].
/// +inf when it goes through.
[[nodiscard]] double crownStop(const TreeSpec& tree, std::uint32_t index,
                               const std::array<double, 3>& origin,
                               const std::array<double, 3>& direction, double range,
                               const std::array<double, 3>& sway, const TreeCuts* cuts, double u);

/// Flying leaves (lagrange/world/Leaves.hpp, ground.md "Flying leaves"): the particles of the
/// vegetation domain's kLeavesSignal as the beams see them, each a disc of its size seen from any
/// side (a beam passing within size/2 of its centre meets it), with a bounding sphere round them
/// all so the beams that miss it skip the list.
class FlyingLeaves {
public:
    /// Takes the signal's value; rebuilds only when its version changed.
    void load(std::span<const double> values);
    [[nodiscard]] bool empty() const { return leaves_.empty(); }
    /// The range (m) along a beam (origin, unit direction) to the first leaf within [0, range];
    /// +inf when it meets none.
    [[nodiscard]] double stop(const std::array<double, 3>& origin,
                              const std::array<double, 3>& direction, double range) const;

private:
    double version_{-1.0};
    std::vector<std::array<double, 4>> leaves_;  // centre (world_local), radius
    std::array<double, 3> centre_{};
    double radius_{0.0};
};

/// Radius of a sphere around the shape's origin that contains it (+inf for a plane).
[[nodiscard]] double boundingRadius(ShapeType type, const std::array<double, 3>& size);

}  // namespace lagrange::sensors
