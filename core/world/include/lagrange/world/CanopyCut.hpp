#pragma once

// Cut plants (docs/physics/ground.md "Cutting the plants"): where a propeller has swept through a
// canopy, the plants stand no taller than the blades went. The vegetation domain keeps that as a
// map of cells over each canopy collider's top (its own frame, x and y) and publishes it on the bus
// as one ScalarArray signal; the lidars read it to see the stubble. The layout is shared here so
// the two modules agree without including each other (architecture invariant 1):
//
//     [version, count, (collider, cellX, cellY, height) x count, unused...]
//
// version grows whenever a cell is cut lower, count is the cells in use, collider the index into
// World::Colliders, (cellX, cellY) the cell (x in [cellX, cellX + 1) kCanopyCutCell), height the
// plants' new top over the soil (m). A cell not listed is not cut.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <string_view>
#include <unordered_map>

namespace lagrange {

inline constexpr std::string_view kCanopyCutSignal = "vegetation/canopy_cut";
/// m, the side of a cut cell: a few blade widths, finer than any propeller's disc.
inline constexpr double kCanopyCutCell = 0.04;
/// Cells the map holds: 16384 cells of 4 cm are 26 m² of cut ground.
inline constexpr std::uint32_t kCanopyCutCapacity = 16384;
inline constexpr std::uint32_t kCanopyCutHeader = 2;
inline constexpr std::uint32_t kCanopyCutLength = kCanopyCutHeader + (4 * kCanopyCutCapacity);

/// The cell (cellX or cellY) of a coordinate in the collider's frame.
[[nodiscard]] inline std::int32_t canopyCutIndex(double coordinate) {
    return static_cast<std::int32_t>(std::floor(coordinate / kCanopyCutCell));
}

[[nodiscard]] inline std::uint64_t canopyCutKey(std::uint32_t collider, std::int32_t cellX,
                                                std::int32_t cellY) {
    constexpr std::uint64_t kMask = (1ULL << 22U) - 1U;  // ±2^21 cells: ±84 km
    constexpr std::int64_t kOffset = 1LL << 21U;
    return (static_cast<std::uint64_t>(collider) << 44U) |
           ((static_cast<std::uint64_t>(cellX + kOffset) & kMask) << 22U) |
           (static_cast<std::uint64_t>(cellY + kOffset) & kMask);
}

/// A reader's view of the map: the plants' top over the soil at a point, +inf where not cut.
class CanopyCuts {
public:
    /// Takes the signal's value; rebuilds only when its version changed.
    void load(std::span<const double> values) {
        if (values.size() < kCanopyCutHeader || values[0] == version_) {
            return;
        }
        version_ = values[0];
        cells_.clear();
        const auto count = static_cast<std::size_t>(std::max(0.0, values[1]));
        for (std::size_t k = 0; k < count; ++k) {
            const std::size_t at = kCanopyCutHeader + (4 * k);
            if (at + 3 >= values.size()) {
                break;
            }
            cells_[canopyCutKey(static_cast<std::uint32_t>(values[at]),
                                static_cast<std::int32_t>(values[at + 1]),
                                static_cast<std::int32_t>(values[at + 2]))] = values[at + 3];
        }
    }
    [[nodiscard]] bool empty() const { return cells_.empty(); }
    /// The cut top (m over the soil) at (x, y) in the collider's frame; +inf where not cut.
    [[nodiscard]] double top(std::uint32_t collider, double x, double y) const {
        if (cells_.empty()) {
            return std::numeric_limits<double>::infinity();
        }
        const auto it = cells_.find(canopyCutKey(collider, canopyCutIndex(x), canopyCutIndex(y)));
        return it == cells_.end() ? std::numeric_limits<double>::infinity() : it->second;
    }

private:
    double version_{0.0};
    std::unordered_map<std::uint64_t, double> cells_;
};

}  // namespace lagrange
