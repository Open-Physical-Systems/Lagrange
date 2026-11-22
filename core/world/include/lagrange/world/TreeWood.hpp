#pragma once

// The wood in a tree's crown (docs/physics/ground.md "Branches"), shared so that the vegetation
// domain (which cuts it, and pushes bodies with it) and the lidars (which see it) grow the same
// branches without including each other (architecture invariant 1).
//
// Geometry, a pure function of the tree's parameters and index (no state, invariant 7):
//
//  - The trunk goes on through the crown from tree:crownBase to kLimbSpan of the crown's height.
//    tree:branchCount limbs leave it at even steps of height, each turned by the golden angle
//    from the last (plus a turn by the tree's index, so neighbouring trees differ), rising at
//    kLimbElevation, and long enough to reach the crown's ellipsoid; the leader goes on from the
//    trunk's top to the crown's top.
//  - Pipe model (Shinozaki, Yoda, Hozumi and Kira 1964, "A quantitative analysis of plant form -
//    the pipe model theory. I. Basic analyses", Japanese Journal of Ecology 14, 97-105): the wood's
//    cross-section carries the leaves beyond it. With the leaves spread along the limbs' length,
//    limb k of length L_k takes the share L_k / Λ of the trunk's cross-section, Λ the limbs' and
//    the leader's lengths together, so its base diameter is d_k = D √(L_k / Λ); the trunk above a
//    limb keeps what the limbs below it did not take, so the cross-sections add up at every
//    junction.
//  - Every limb forks in two, again and again, down to kTwigDiameter. Leonardo's rule (as stated
//    and argued in Eloy 2011, "Leonardo's rule, self-similarity, and wind-induced stresses in
//    trees", Phys. Rev. Lett. 107, 258101, doi:10.1103/PhysRevLett.107.258101): the two children's
//    cross-sections add up to their parent's, so each is d/√2 and generation g of a limb is
//    d_k 2^(-g/2). Each generation is kForkLengthRatio as long as its parent, the first sized so
//    that the straight line of forks reaches the limb's length; children turn ±kForkHalfAngle off
//    their parent, the fork's plane rolled 90° each generation.
//
// The broken wood (kBranchCutSignal, written by the vegetation domain):
//
//     [version, count, (tree, segment, share) x count, unused...]
//
// tree the index into treesOf(), segment the index into its skeleton's segments, share the part
// of the segment still standing from its base (0..1); everything beyond it (the segment's
// subtree) is gone. The list only grows; the version changes with it.

#include "lagrange/world/Trees.hpp"

#include <array>
#include <cstdint>
#include <numbers>
#include <span>
#include <string_view>
#include <vector>

namespace lagrange {

/// The limbs leave the trunk over the lower 70 % of the crown's height. TODO(verify).
inline constexpr double kLimbSpan = 0.7;
/// The limbs rise 30° over the horizontal. TODO(verify) against a species.
inline constexpr double kLimbElevation = 30.0 * std::numbers::pi / 180.0;
/// A fork's children turn 25° off their parent. TODO(verify).
inline constexpr double kForkHalfAngle = 25.0 * std::numbers::pi / 180.0;
/// Each generation of a limb is 0.8 as long as its parent. TODO(verify).
inline constexpr double kForkLengthRatio = 0.8;
/// Wood thinner than this is not drawn as branches: it is in the crown's leaves and twigs (m).
inline constexpr double kTwigDiameter = 0.004;
/// The golden angle (rad), between successive limbs.
inline constexpr double kGoldenAngle = std::numbers::pi * (3.0 - 2.23606797749978969640);

/// A piece of wood, a straight cylinder, in the crown's frame (relative to the crown's centre,
/// unswayed, the tree's orientation undone), in depth-first order.
struct BranchSegment {
    std::array<double, 3> from{};
    std::array<double, 3> to{};
    double diameter{0.0};            // m
    std::uint32_t end{0};            // one past the last segment of its subtree
    std::array<double, 3> centre{};  // a sphere around it and its subtree
    double radius{0.0};
    std::int32_t generation{0};  // -1 for the trunk and leader's base, else the fork generation
};

/// The wood of one tree.
struct Skeleton {
    std::vector<BranchSegment> segments;
    double trunkArea{0.0};      // m², π D² / 4
    std::vector<double> limbs;  // L_k, the limbs' lengths, then the leader's
};

/// The pipe model's limb diameter: D √(L / Λ).
[[nodiscard]] double limbDiameter(double trunkDiameter, double length, double totalLength);
/// Leonardo's rule over g forks: d 2^(-g/2).
[[nodiscard]] double forkDiameter(double diameter, int generation);
/// Tree `index`'s wood.
[[nodiscard]] Skeleton skeletonOf(const TreeSpec& tree, std::uint32_t index);

/// Where a ray o + t d (unit d) first meets a capsule round the segment a-b of radius r, for t
/// in [0, range]; +inf when it does not. A ray starting inside meets it at 0.
[[nodiscard]] double rayCapsule(const std::array<double, 3>& o, const std::array<double, 3>& d,
                                const std::array<double, 3>& a, const std::array<double, 3>& b,
                                double r, double range);

inline constexpr std::string_view kBranchCutSignal = "vegetation/branch_cut";
inline constexpr std::uint32_t kBranchCutCapacity = 4096;
inline constexpr std::uint32_t kBranchCutHeader = 2;
inline constexpr std::uint32_t kBranchCutFields = 3;
inline constexpr std::uint32_t kBranchCutLength =
    kBranchCutHeader + (kBranchCutFields * kBranchCutCapacity);

/// Where a ray met the wood.
struct WoodHit {
    double distance{0.0};            // m along the ray
    std::array<double, 3> normal{};  // world, unit, the bark's outward normal there
    double diameter{0.0};            // m, of the wood met
};

/// The trees' wood and how much of each segment still stands.
class TreeWood {
public:
    void build(const std::vector<TreeSpec>& trees);
    [[nodiscard]] bool empty() const { return skeletons_.empty(); }
    [[nodiscard]] std::size_t trees() const { return skeletons_.size(); }
    [[nodiscard]] const Skeleton& skeleton(std::size_t tree) const { return skeletons_.at(tree); }
    /// The share of segment `s` of tree `tree` still standing (1 whole, 0 gone).
    [[nodiscard]] double standing(std::size_t tree, std::size_t s) const {
        return standing_.at(tree).at(s);
    }
    /// Everything stands again.
    void reset();
    /// Breaks segment `segment` of tree `tree` at `share` of its length: what is beyond is gone.
    void cut(std::uint32_t tree, std::uint32_t segment, double share);
    /// Takes kBranchCutSignal's value; rebuilds only when its version changed.
    void load(std::span<const double> values);
    /// Where a ray (world, unit direction) first meets tree `tree`'s standing wood within
    /// [0, range], the crown moved by `sway` (world, m); false when it does not.
    bool ray(const TreeSpec& tree, std::size_t index, const std::array<double, 3>& origin,
             const std::array<double, 3>& direction, double range,
             const std::array<double, 3>& sway, WoodHit& hit) const;

private:
    std::vector<Skeleton> skeletons_;
    std::vector<std::vector<double>> standing_;
    double version_{0.0};
};

}  // namespace lagrange
