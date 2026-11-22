#pragma once

// Trees (docs/physics/ground.md "Trees"): a `TreeAPI_1` prim is a tree standing at the prim's
// origin, z up. Its trunk is whatever collider the stage gives it (a solid cylinder, usually); its
// crown is not a collider but a volume of leaves and twigs, an ellipsoid over the crown base
// centred at (0, 0, crownBase + (height - crownBase)/2) with semi-axes (R, R, (height -
// crownBase)/2). Bodies push through it (the vegetation domain's drag), the wind and the rotors'
// downwash sway it, propellers cut it, and lidars see it as a turbid volume.
//
// Shared here so that the modules agree without including each other (architecture invariant 1):
// the vegetation domain writes tree/<name>/sway (the crown's displacement, world, m) and the cut
// map kTreeCutSignal, the lidars and the viewer read them.
//
//     kTreeCutSignal: [version, count, (tree, cellX, cellY, cellZ) x count, unused...]
//
// tree is the index into treesOf(), (cellX, cellY, cellZ) a cell of kTreeCutCell in the crown's
// own frame (relative to its centre, unswayed). A listed cell has been cut away: no leaves there.

#include "lagrange/world/PoseMath.hpp"
#include "lagrange/world/World.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

namespace lagrange {

/// A tree's parameters (TreeAPI_1, formats.md §3.13) with their fallbacks applied.
struct TreeSpec {
    std::string name;  // the prim's signal instance name: tree/<name>/...
    PrimId prim{};
    Pose pose{};                   // region-local, the foot of the trunk
    double height{0.0};            // m
    double crownRadius{0.0};       // m
    double crownBase{0.0};         // m above the foot
    double trunkDiameter{0.0};     // m, for drawing
    double leafAreaDensity{0.0};   // m² of leaf per m³ of crown
    double dragCoefficient{0.0};   // of the crown's frontal area
    double naturalFrequency{0.0};  // Hz, the first sway mode
    double dampingRatio{0.0};
    double crownMass{0.0};  // kg, the sway mode's mass
    double cutEnergy{0.0};  // J per m³ of crown cut through
    double leafArea{0.0};   // m², one leaf

    [[nodiscard]] double halfHeight() const { return 0.5 * (height - crownBase); }
    /// The crown's centre over the foot, in the tree's frame.
    [[nodiscard]] double centreHeight() const { return crownBase + halfHeight(); }
    [[nodiscard]] double crownVolume() const {
        return 4.0 / 3.0 * 3.14159265358979323846 * crownRadius * crownRadius * halfHeight();
    }
    /// The crown's frontal area seen from the side, m²: the ellipse π R c.
    [[nodiscard]] double frontalArea() const {
        return 3.14159265358979323846 * crownRadius * halfHeight();
    }
};

/// Fallbacks of TreeAPI_1 that are drawing choices or typical values, every one TODO(verify)
/// against measurements of the species (docs/physics/ground.md "Trees", "Numbers and sources").
inline constexpr double kTreeCrownBaseShare = 0.3;    // crownBase = 0.3 height
inline constexpr double kTreeTrunkShare = 0.125;      // trunkDiameter = crownRadius / 8
inline constexpr double kTreeLeafAreaDensity = 1.0;   // m²/m³
inline constexpr double kTreeDragCoefficient = 0.3;   // crowns streamline in strong wind
inline constexpr double kTreeNaturalFrequency = 0.5;  // Hz, a tree of some 10 m
inline constexpr double kTreeDampingRatio = 0.1;
inline constexpr double kTreeCrownDensity = 2.0;  // kg/m³: crownMass = 2 × crown volume
inline constexpr double kTreeCutEnergy = 2000.0;  // J/m³ of crown (leaves and twigs)
inline constexpr double kTreeLeafArea = 0.003;    // m², a leaf of 5 cm by 6 cm

inline constexpr std::string_view kTreeCutSignal = "vegetation/tree_cut";
/// m, the side of a cut cell of a crown.
inline constexpr double kTreeCutCell = 0.1;
inline constexpr std::uint32_t kTreeCutCapacity = 16384;
inline constexpr std::uint32_t kTreeCutHeader = 2;
inline constexpr std::uint32_t kTreeCutLength = kTreeCutHeader + (4 * kTreeCutCapacity);

/// Every TreeAPI_1 prim of the world; `error` names the first that is wrong (and the list is
/// empty then).
[[nodiscard]] inline std::vector<TreeSpec> treesOf(const World& world, std::string& error) {
    std::vector<TreeSpec> out;
    for (const SchemaInstance& s : world.schemaInstances) {
        if (s.schema != "TreeAPI_1") {
            continue;
        }
        TreeSpec t;
        t.prim = s.prim;
        t.name = s.prim.index < world.prims.instanceNames.size()
                     ? world.prims.instanceNames[s.prim.index]
                     : std::string();
        t.pose = world.prims.poseInBody.at(s.prim.index);
        t.height = s.real("tree:height").value_or(0.0);
        t.crownRadius = s.real("tree:crownRadius").value_or(0.0);
        t.crownBase = s.real("tree:crownBase").value_or(kTreeCrownBaseShare * t.height);
        t.trunkDiameter = s.real("tree:trunkDiameter").value_or(kTreeTrunkShare * t.crownRadius);
        t.leafAreaDensity = s.real("tree:leafAreaDensity").value_or(kTreeLeafAreaDensity);
        t.dragCoefficient = s.real("tree:dragCoefficient").value_or(kTreeDragCoefficient);
        t.naturalFrequency = s.real("tree:naturalFrequency").value_or(kTreeNaturalFrequency);
        t.dampingRatio = s.real("tree:dampingRatio").value_or(kTreeDampingRatio);
        t.cutEnergy = s.real("tree:cutEnergy").value_or(kTreeCutEnergy);
        t.leafArea = s.real("tree:leafArea").value_or(kTreeLeafArea);
        if (!(t.height > 0.0) || !(t.crownRadius > 0.0) || !(t.crownBase >= 0.0) ||
            !(t.crownBase < t.height)) {
            error = "tree " + t.name +
                    ": tree:height and tree:crownRadius must be over 0, tree:crownBase from 0 to "
                    "under the height";
            return {};
        }
        t.crownMass = s.real("tree:crownMass").value_or(kTreeCrownDensity * t.crownVolume());
        for (const double v :
             {t.trunkDiameter, t.leafAreaDensity, t.dragCoefficient, t.naturalFrequency,
              t.dampingRatio, t.crownMass, t.cutEnergy, t.leafArea}) {
            if (!(v >= 0.0)) {
                error = "tree " + t.name + ": its numbers must not be negative";
                return {};
            }
        }
        if (!(t.naturalFrequency > 0.0) || !(t.crownMass > 0.0) || !(t.leafArea > 0.0)) {
            error = "tree " + t.name +
                    ": tree:naturalFrequency, tree:crownMass and tree:leafArea must be over 0";
            return {};
        }
        out.push_back(std::move(t));
    }
    return out;
}

/// A point (world) in the crown's frame: relative to its centre, the crown moved by `sway`
/// (world, m), the tree's orientation undone.
[[nodiscard]] inline std::array<double, 3> inCrown(const TreeSpec& t,
                                                   const std::array<double, 3>& point,
                                                   const std::array<double, 3>& sway) {
    const std::array<double, 3> up = rotate(t.pose.orientation, {0.0, 0.0, t.centreHeight()});
    const std::array<double, 3> d{point[0] - t.pose.position[0] - up[0] - sway[0],
                                  point[1] - t.pose.position[1] - up[1] - sway[1],
                                  point[2] - t.pose.position[2] - up[2] - sway[2]};
    return rotate(conjugate(t.pose.orientation), d);
}

/// Whether a point in the crown's frame lies in its ellipsoid.
[[nodiscard]] inline bool inEllipsoid(const TreeSpec& t, const std::array<double, 3>& q) {
    const double x = q[0] / t.crownRadius;
    const double y = q[1] / t.crownRadius;
    const double z = q[2] / t.halfHeight();
    return x * x + y * y + z * z <= 1.0;
}

[[nodiscard]] inline std::int32_t treeCutIndex(double coordinate) {
    return static_cast<std::int32_t>(std::floor(coordinate / kTreeCutCell));
}

[[nodiscard]] inline std::uint64_t treeCutKey(std::uint32_t tree, std::int32_t x, std::int32_t y,
                                              std::int32_t z) {
    constexpr std::uint64_t kMask = (1ULL << 16U) - 1U;  // ±32768 cells: ±3.2 km
    constexpr std::int64_t kOffset = 1LL << 15U;
    return (static_cast<std::uint64_t>(tree) << 48U) |
           ((static_cast<std::uint64_t>(x + kOffset) & kMask) << 32U) |
           ((static_cast<std::uint64_t>(y + kOffset) & kMask) << 16U) |
           (static_cast<std::uint64_t>(z + kOffset) & kMask);
}

/// A reader's view of the crowns' cut cells.
class TreeCuts {
public:
    /// Takes the signal's value; rebuilds only when its version changed.
    void load(std::span<const double> values) {
        if (values.size() < kTreeCutHeader || values[0] == version_) {
            return;
        }
        version_ = values[0];
        cells_.clear();
        const auto count = static_cast<std::size_t>(std::max(0.0, values[1]));
        for (std::size_t k = 0; k < count; ++k) {
            const std::size_t at = kTreeCutHeader + (4 * k);
            if (at + 3 >= values.size()) {
                break;
            }
            cells_.insert(treeCutKey(static_cast<std::uint32_t>(values[at]),
                                     static_cast<std::int32_t>(values[at + 1]),
                                     static_cast<std::int32_t>(values[at + 2]),
                                     static_cast<std::int32_t>(values[at + 3])));
        }
    }
    [[nodiscard]] bool empty() const { return cells_.empty(); }
    /// Whether the cell holding `q` (crown frame) of tree `tree` was cut away.
    [[nodiscard]] bool cut(std::uint32_t tree, const std::array<double, 3>& q) const {
        return !cells_.empty() &&
               cells_.contains(
                   treeCutKey(tree, treeCutIndex(q[0]), treeCutIndex(q[1]), treeCutIndex(q[2])));
    }

private:
    double version_{0.0};
    std::unordered_set<std::uint64_t> cells_;
};

/// Whether a point (world) is among a crown's leaves: in its ellipsoid at its sway, not cut away.
[[nodiscard]] inline bool inLeaves(const TreeSpec& t, std::uint32_t index,
                                   const std::array<double, 3>& point,
                                   const std::array<double, 3>& sway, const TreeCuts* cuts) {
    const std::array<double, 3> q = inCrown(t, point, sway);
    return inEllipsoid(t, q) && (cuts == nullptr || !cuts->cut(index, q));
}

}  // namespace lagrange
