#pragma once

// What lidars and cameras see of a moving water surface (docs/physics/water.md "The surface under
// a rotor"; sensors.md): the hydrodynamics domain's patch of heights over the still level, met
// by a ray in double, so that a beam hits the water at η, not at the flat level. The same on
// every tier: the GPU paths cast against the flat box and this moves their hits onto the patch.

#include <array>
#include <cstdint>
#include <span>

namespace lagrange::sensors {

/// One body of water's moving surface, as water/<w>/surface and surface_origin give it.
struct MovingWater {
    double level{0.0};                 ///< m, the still surface's height (world)
    std::array<double, 2> lo{}, hi{};  ///< m, the water's extent (world x, y)
    std::uint32_t cells{0};            ///< the patch's side, cells; 0: none
    double cell{0.0};                  ///< m
    std::array<double, 2> corner{};    ///< m, the patch's lower corner (world x, y)
    std::span<const double> heights;   ///< m over the level, cells² in row order (y rows of x)
    double lowest{0.0}, highest{0.0};  ///< m, the least and the most of `heights` (and 0)

    /// Fills `lowest` and `highest` from `heights`; false when the patch is not usable (then the
    /// surface is the still level).
    bool prepare();
    /// η at a world point (bilinear between cell centres, as hydrodynamics::gridHeight); 0
    /// outside the patch or the water.
    [[nodiscard]] double heightAt(double x, double y) const;
    /// The surface's upward unit normal there, from η's slope.
    [[nodiscard]] std::array<double, 3> normalAt(double x, double y) const;
};

/// Where a ray from `origin` along `direction` (any length: distances are in its units) first
/// meets the moving surface, given what it hit before (`hit`, +inf for nothing). With
/// `hitIsWater` (the hit is the flat water itself) the answer may lie beyond `hit` (a trough);
/// otherwise it is `hit` unless a crest stands in front of it. Only from above the surface; a ray
/// that starts under it is left as it is. Solved in double by stepping half a cell at a
/// time between the crests' and the troughs' heights and bisecting the first crossing:
/// deterministic. `onWater` says whether the answer is the water's.
[[nodiscard]] double rayOntoWater(const MovingWater& water, const std::array<double, 3>& origin,
                                  const std::array<double, 3>& direction, double hit,
                                  bool hitIsWater, bool* onWater = nullptr);

}  // namespace lagrange::sensors
