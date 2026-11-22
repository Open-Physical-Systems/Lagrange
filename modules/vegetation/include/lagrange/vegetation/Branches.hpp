#pragma once

// Branches (docs/physics/ground.md "Branches"): the wood in a tree's crown, which a propeller cuts
// when it is a twig and is stopped by when it is a branch.
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
// What a propeller does (a disc of the blades' radius, kBladeHalfThickness thick):
//  - wood it meets no thicker than tree:maxCutDiameter is cut where it meets it, everything beyond
//    falls (is gone), and the rotor pays E = σ π d² / 4 (σ tree:woodCutEnergy) as for the leaves;
//  - thicker wood is a strike: the blade stops on it. The branch pushes the blade at the contact,
//    against its motion and square to the branch, as hard as it takes to remove the propeller's
//    angular momentum I ω in the step (the push stands over the step); its push on the drone is
//    the same force through the hub. The strike takes the propeller's kinetic energy ½ I ω², and
//    a share of it, over kBladeBreakEnergy, breaks the blades: rotor/<j>/damage.

#include "lagrange/module/IModule.hpp"
#include "lagrange/module/StateIO.hpp"
#include "lagrange/world/Trees.hpp"

#include <array>
#include <cstdint>
#include <numbers>
#include <string>
#include <vector>

namespace lagrange::vegetation {

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
/// Half the thickness of the sheet a propeller's blades sweep (m). TODO(verify).
inline constexpr double kBladeHalfThickness = 0.005;
/// The strike energy that breaks a propeller's blades (J), rotor/<j>/damage = energy / this,
/// summed and held at 1. TODO(verify): no measurement behind it.
inline constexpr double kBladeBreakEnergy = 30.0;
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

/// Where a segment from `a` to `b` of diameter `d` meets a disc (centre, unit normal, radius,
/// kBladeHalfThickness thick): the distance along it from `a` to the first point of the
/// segment within the disc, or a negative number when it does not.
[[nodiscard]] double discMeets(const std::array<double, 3>& a, const std::array<double, 3>& b,
                               double d, const std::array<double, 3>& centre,
                               const std::array<double, 3>& normal, double radius);

/// What a rotor's disc did to the wood in one step.
struct BranchHit {
    double cutEnergy{0.0};  // J, twigs cut, for the rotor to pay
    int twigsCut{0};
    bool strike{false};
    double diameter{0.0};            // m, of the wood struck
    double energy{0.0};              // J, the rotor's kinetic energy the strike took
    std::array<double, 3> point{};   // world, where
    std::array<double, 6> wrench{};  // on the propeller's link, world, about its centre of mass
};

/// The trees' wood and what the rotors did to it. Signals per rotor rotor/<j>/damage (0..1) and
/// rotor/<j>/branch_strikes (count), per tree tree/<n>/twigs_cut (count).
class Branches {
public:
    void build(const std::vector<TreeSpec>& trees);
    /// Declares the rotor's signals (only when there are trees); call in the rotors' order.
    void addRotor(SignalDeclarer& declarer, const std::string& joint,
                  const std::array<double, 3>& linkCom);
    /// A rotor without signals (tests).
    void addRotor(const std::array<double, 3>& linkCom);
    void declareTrees(const std::vector<TreeSpec>& trees, SignalDeclarer& declarer);
    void init(SignalBus& bus);
    /// Rotor `rotor`'s disc (hub pose of its link, unit axis world, radius) at spin `omega` with
    /// `inertia` about its axis, over a step `dt`, through every crown at its sway.
    BranchHit sweep(std::size_t rotor, const std::vector<TreeSpec>& trees, const Pose& hub,
                    const std::array<double, 3>& axis, double radius,
                    const std::vector<std::array<double, 3>>& sways, double omega, double inertia,
                    double dt);
    void publish(SignalBus& bus);
    void save(StateWriter& writer) const;
    bool restore(StateReader& reader);

    [[nodiscard]] const Skeleton& skeleton(std::size_t tree) const { return skeletons_.at(tree); }
    /// The share of segment `s` of tree `tree` still standing (1 whole, 0 gone).
    [[nodiscard]] double standing(std::size_t tree, std::size_t s) const;
    [[nodiscard]] double damage(std::size_t rotor) const { return rotors_.at(rotor).damage; }
    double breakEnergy{kBladeBreakEnergy};

private:
    struct Cut {
        std::uint32_t tree, segment;
        double share;
    };
    struct RotorState {
        SignalId damageOut, strikesOut;
        std::array<double, 3> com{};
        double damage{0.0};
        double strikes{0.0};
    };
    void apply(const Cut& cut);
    std::vector<Skeleton> skeletons_;
    std::vector<std::vector<double>> standing_;
    std::vector<Cut> cuts_;  // in order, the saved state
    std::vector<RotorState> rotors_;
    std::vector<SignalId> twigsOut_;
    std::vector<double> twigs_;
};

}  // namespace lagrange::vegetation
