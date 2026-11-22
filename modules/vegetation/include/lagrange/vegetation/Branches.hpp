#pragma once

// Branches (docs/physics/ground.md "Branches"): the wood in a tree's crown, which a propeller cuts
// when it is a twig and is stopped by when it is a branch.
//
// The wood's geometry, a pure function of the tree's parameters and index, is shared with the
// lidars (lagrange/world/TreeWood.hpp: the trunk through the crown, the limbs by the pipe model,
// their forks by Leonardo's rule).
//
// What a propeller does (a disc of the blades' radius, kBladeHalfThickness thick):
//  - wood it meets no thicker than tree:maxCutDiameter is cut where it meets it, everything beyond
//    falls (is gone), and the rotor pays E = σ π d² / 4 (σ tree:woodCutEnergy) as for the leaves;
//  - thicker wood is a strike: the blade stops on it. The branch pushes the blade at the contact,
//    against its motion and square to the branch, as hard as it takes to remove the propeller's
//    angular momentum I ω in the step (the push stands over the step); its push on the drone is
//    the same force through the hub. The strike takes the propeller's kinetic energy ½ I ω², and
//    a share of it, over kBladeBreakEnergy, breaks the blades: rotor/<j>/damage.
//
// What a body does (its colliders against the wood, each segment a capsule of its diameter, at
// the crown's sway):
//  - wood no thicker than tree:maxCutDiameter that a collider reaches into breaks there: it is
//    cut as a propeller cuts a twig (the cut list, tree/<n>/twigs_cut), and pushes back nothing;
//  - thicker wood pushes the body out along the collider's surface normal by a penalty contact,
//    a spring and a damper on the overlap with Coulomb friction: F_n = max(0, k δ + c v_n),
//    |F_t| = min(μ F_n, c |v_t|) against the sliding, k = m/τ² and c = 2m/τ for the link's mass
//    m (the critically damped soft contact of MuJoCo's solref (τ, 1), whose default τ = 0.02 s
//    is kWoodContactTime; MuJoCo documentation, "Solver parameters"). The wood is rigid: no
//    branch bends. What pushes the body pushes the crown back, horizontally, into its sway.
//  - wood inside a fixed collider of the world (the trunk the stage gives the tree) is that
//    collider's: it does not push twice.

#include "lagrange/module/IModule.hpp"
#include "lagrange/module/StateIO.hpp"
#include "lagrange/world/TreeWood.hpp"
#include "lagrange/world/Trees.hpp"
#include "lagrange/world/World.hpp"

#include <array>
#include <cstdint>
#include <numbers>
#include <span>
#include <string>
#include <vector>

namespace lagrange::vegetation {

/// Half the thickness of the sheet a propeller's blades sweep (m). TODO(verify).
inline constexpr double kBladeHalfThickness = 0.005;
/// The strike energy that breaks a propeller's blades (J), rotor/<j>/damage = energy / this,
/// summed and held at 1. TODO(verify): no measurement behind it.
inline constexpr double kBladeBreakEnergy = 30.0;
/// The penalty contact's time constant τ (s): MuJoCo's default solref timeconst, 0.02 s, critically
/// damped. A numerical choice, not a property of wood.
inline constexpr double kWoodContactTime = 0.02;
/// Coulomb friction of bark on a robot's shell. TODO(verify): no measurement of bark on plastic or
/// aluminium behind it.
inline constexpr double kBarkFriction = 0.5;

using lagrange::BranchSegment;
using lagrange::forkDiameter;
using lagrange::kForkHalfAngle;
using lagrange::kForkLengthRatio;
using lagrange::kGoldenAngle;
using lagrange::kLimbElevation;
using lagrange::kLimbSpan;
using lagrange::kTwigDiameter;
using lagrange::limbDiameter;
using lagrange::Skeleton;
using lagrange::skeletonOf;

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

/// A collider as the wood touches it.
struct WoodShape {
    ShapeType type{ShapeType::Sphere};
    std::array<double, 3> size{};
    Pose pose{};  // in its body's frame (a fixed collider's: region-local)
    const std::vector<std::array<double, 4>>* faces{nullptr};  // a hull's planes
    double bound{0.0};  // m, a sphere round it about pose.position
};
/// The collider `c` of `world` as the wood touches it; false for a plane or a height field.
bool woodShapeOf(const World& world, std::size_t c, WoodShape& out);
/// The signed distance from q (the shape's frame) to its surface (m): negative inside. Exact for
/// spheres, capsules, cylinders and boxes; for a hull the farthest of its face planes (exact
/// inside and over a face, short of the truth past an edge).
[[nodiscard]] double shapeDistance(const WoodShape& shape, const std::array<double, 3>& q);

/// What the wood did to one body in one step.
struct BodyContact {
    std::array<double, 6> wrench{};  // N, N·m: world, about the centre of mass
    double force{0.0};               // N, the size of the total push
    double depth{0.0};               // m, the deepest overlap
    int contacts{0};
    int broken{0};  // twigs it broke
};

/// The trees' wood and what the rotors and bodies did to it. Signals per rotor rotor/<j>/damage
/// (0..1) and rotor/<j>/branch_strikes (count), per tree tree/<n>/twigs_cut (count, by rotors
/// and bodies), and kBranchCutSignal, the wood broken off.
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
    /// The wood's push on a body (pose of its link, world; twist: velocity of the link's origin
    /// and angular velocity, world) whose colliders are `shapes` within `bound` of its origin,
    /// centre of mass `comLocal` (link frame), mass `mass`. Breaks the twigs it reaches into.
    /// `crownVelocity` the crowns' sway velocities (world); the crowns' push back is added to
    /// `crownPush` (world, N, per tree).
    BodyContact touch(const std::vector<TreeSpec>& trees, std::span<const WoodShape> shapes,
                      double bound, const Pose& pose, const std::array<double, 6>& twist,
                      const std::array<double, 3>& comLocal, double mass,
                      const std::vector<std::array<double, 3>>& sways,
                      const std::vector<std::array<double, 3>>& crownVelocity,
                      std::vector<std::array<double, 3>>& crownPush);
    /// The world's fixed colliders: wood inside one is its, not pushed twice (the trunk).
    void setFixed(std::vector<WoodShape> fixed) { fixed_ = std::move(fixed); }
    void publish(SignalBus& bus);
    void save(StateWriter& writer) const;
    bool restore(StateReader& reader);

    [[nodiscard]] const Skeleton& skeleton(std::size_t tree) const { return wood_.skeleton(tree); }
    /// The share of segment `s` of tree `tree` still standing (1 whole, 0 gone).
    [[nodiscard]] double standing(std::size_t tree, std::size_t s) const;
    [[nodiscard]] const TreeWood& wood() const { return wood_; }
    [[nodiscard]] double damage(std::size_t rotor) const { return rotors_.at(rotor).damage; }
    /// kBranchCutSignal's value.
    [[nodiscard]] const std::vector<double>& cutValues() const { return cutValues_; }
    double breakEnergy{kBladeBreakEnergy};
    double contactTime{kWoodContactTime};  // s
    double friction{kBarkFriction};

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
    /// Appends a cut to the list and the signal's value.
    void record(const Cut& cut);
    [[nodiscard]] bool inFixed(const std::array<double, 3>& point) const;
    TreeWood wood_;
    std::vector<std::array<double, 3>> woodUp_;  // per tree, m (build)
    std::vector<Cut> cuts_;                      // in order, the saved state
    std::vector<double> cutValues_ = std::vector<double>(kBranchCutLength, 0.0);
    SignalId cutOut_;
    double cutVersion_{0.0};  // what publish last wrote
    std::vector<WoodShape> fixed_;
    std::vector<RotorState> rotors_;
    std::vector<SignalId> twigsOut_;
    std::vector<double> twigs_;
};

}  // namespace lagrange::vegetation
