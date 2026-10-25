#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace lagrange {
struct Model;
struct TriMesh;
}  // namespace lagrange

namespace lagrange::rotor_aero {

/// A section's aerodynamics over the full circle of angles of attack (aerodynamics.md §2.1):
/// attached flow is linear in lift, cl = liftSlope·(α − α0) with compressibility by the
/// Prandtl–Glauert factor when the speed of sound is known, and a parabolic drag polar
/// cd = cdMin + cdLift·(cl − clAtCdMin)²; beyond stall (cl outside [clMin, clMax]) the section
/// blends over kStallBlend into a flat plate (normal force 2·sin α, Hoerner), which also covers
/// reversed flow, so a propeller turning the wrong way or mounted the wrong way round behaves as
/// a real one: poorly.
struct Aerofoil {
    double liftSlope{0.0};      // 1/rad, two-dimensional
    double zeroLiftAngle{0.0};  // rad, against the section's reference line
    double clMax{0.0};
    double clMin{0.0};
    double cdMin{0.0};
    double clAtCdMin{0.0};
    double cdLift{0.0};
    // For drawing only: the section's mean-line camber (fraction of chord) and its position
    // (NACA four-digit mean line); the thickness is the station's.
    double camber{0.0};
    double camberPosition{0.4};
};

/// Lift and drag coefficient of `a` at angle of attack `alpha` (rad, any value) and Mach number
/// `mach` (0 for incompressible).
struct SectionCoefficients {
    double cl{0.0};
    double cd{0.0};
};
[[nodiscard]] SectionCoefficients sectionCoefficients(const Aerofoil& a, double alpha, double mach);

/// One blade station: radius fraction r/R, chord c/R, blade angle against the plane of rotation
/// (rad, the section's reference line), thickness t/c (for the drawn blade) and its aerofoil.
struct BladeStation {
    double radius{0.0};
    double chord{0.0};
    double twist{0.0};
    double thickness{0.0};
    Aerofoil aerofoil;
};

/// A propeller as built: blade geometry and sections, read from a propeller file
/// (`*.prop.yaml`, formats.md §3.3.1). `hand` is the sense of rotation about the propeller's own
/// thrust direction that makes thrust: "ccw" or "cw".
/// Folding blades: each blade outboard of `hinge` swings on a hinge whose axis is tangential,
/// coning forward under thrust, straightened by its spin, and folded back by gravity when the
/// propeller stops (FoldingBlade).
struct Folding {
    double hinge{0.0};      // r/R of the hinges
    double bladeMass{0.0};  // kg, one blade outboard of its hinge
    double limit{0.0};      // rad, the hinge's stop either way (folded flat against the motor)
};

struct Propeller {
    std::string name;
    double radius{0.0};  // m, tip
    int blades{0};
    std::string hand;
    std::vector<BladeStation> stations;  // by radius, the first at the root of the aerodynamic
                                         // blade (the hub's edge)
    std::optional<Folding> folding;
};

/// A folding blade's angle on its hinge (aerodynamics.md §2.1), quasi-static: the moments of
/// its thrust and its weight about the hinge balance the centrifugal moment of its mass,
/// Ω²·sin β·(r_h·S1 + cos β·S2), with S1 and S2 the blade's first and second mass moments about
/// the hinge (mass along the blade as chord²·thickness). β > 0 cones the blade towards the
/// front; stopped, a blade folds to its limit the way its weight pulls it.
class FoldingBlade {
public:
    FoldingBlade() = default;
    explicit FoldingBlade(const Propeller& p);
    /// β (rad) at speed of rotation `omega` (rad/s, either way) with the propeller's thrust
    /// `thrust` (N, all blades, along its thrust direction) acting at `thrustRadius` (m) and
    /// gravity's component along the thrust direction `gravityAlong` (m/s²).
    [[nodiscard]] double angle(double omega, double thrust, double thrustRadius,
                               double gravityAlong) const;
    [[nodiscard]] double centreOfMass() const { return centre_; }  // m from the axis

private:
    int blades_{1};
    double hinge_{0.0};   // m
    double mass_{0.0};    // kg
    double centre_{0.0};  // m from the axis
    double s1_{0.0};      // kg·m: ∫ m'·s ds, s from the hinge
    double s2_{0.0};      // kg·m²: ∫ m'·s² ds
    double limit_{0.0};   // rad
};

/// Reads a propeller file; on failure returns an empty propeller (radius 0) and sets `error`.
[[nodiscard]] Propeller loadPropeller(const std::string& path, std::string& error);

/// The same propeller with every blade angle turned by `delta` (rad): a different setting of a
/// variable-pitch or ground-adjustable hub.
[[nodiscard]] Propeller withBladeAngle(Propeller p, double delta);

/// Air at the propeller.
struct Air {
    double density{0.0};       // kg/m³
    double speedOfSound{0.0};  // m/s; 0: no compressibility correction
};

/// The propeller's steady load by blade-element momentum theory (aerodynamics.md §2.1): per
/// annulus, the axial and swirl induced velocities that make the blade-element forces equal the
/// momentum change of the air through it, with Prandtl tip and hub losses. `omega` (rad/s) is the
/// speed of rotation in the propeller's hand (negative: turning the wrong way) and `axialSpeed`
/// (m/s) the speed of the air arriving at the disc from the front, along the thrust direction
/// (flight speed; negative: the air comes from behind, as in a descent). `edgewiseSpeed` (m/s,
/// ≥ 0) is the air's speed across the disc (forward flight, side wind): the blades' loads are
/// averaged round the disc, each meeting ω·r + V_e·sin ψ, with Glauert's mass flow through each
/// annulus, which gives the in-plane H-force, the hub's rolling moment and the change of thrust.
///
/// Momentum theory has no steady solution in the vortex-ring state (descent at about the induced
/// velocity); there the annulus takes the blade-element forces with no induced velocity, the
/// common engineering treatment, and `vortexRing` is set.
struct BemLoad {
    double thrust{0.0};  // N, along the thrust direction
    double torque{0.0};  // N·m, the air's torque on the propeller about the thrust direction,
                         // positive against rotation in the hand (what the motor must give)
    double inducedVelocity{0.0};  // m/s, thrust-weighted mean axial induced velocity
    double thrustRadius{0.0};     // m, thrust-weighted mean radius: where the thrust acts
    bool vortexRing{false};
    // In edgewise flow, in the plane of the disc, along e1 (the way the edgewise air moves past
    // the propeller) and e2 = t × e1 with t the thrust direction, both in the propeller's hand
    // (a cw propeller's e2 is the mirror image): the in-plane force (N; the H-force along e1) and
    // the thrust's moment about the hub (N·m; the advancing blade's extra lift rolls it about e1).
    std::array<double, 2> inPlane{};
    std::array<double, 2> hubMoment{};
};
[[nodiscard]] BemLoad bemLoad(const Propeller& p, double omega, double axialSpeed, const Air& air,
                              double edgewiseSpeed = 0.0);

/// A propeller's loads tabulated once (aerodynamics.md §2.1): blade-element momentum theory over
/// the full circle of inflow angles θ = atan2(V, ΩR) at 0.25° steps (turning either way, stopped,
/// in climb, descent or windmilling), and, when `compressible`, over tip Mach numbers 0 to 0.9 at
/// 0.1 steps. Loads scale exactly with ρ·(Ω²R² + V²) (the induced velocities with the resultant
/// tip speed) in incompressible flow, so a lookup gives what bemLoad gives, to the interpolation:
/// what the rotor module steps with, at a fixed cost.
class PropellerTable {
public:
    [[nodiscard]] static PropellerTable build(const Propeller& p, bool compressible);
    /// The loads at `edgewiseSpeed` too: the edgewise table (inflow angle in 1° steps, edgewise
    /// angle atan2(V_e, √(Ω²R² + V_a²)) from 0 to 40° in 5° steps, held beyond) adds what edgewise
    /// flow changes to the axial loads, and gives the in-plane force and hub moment.
    [[nodiscard]] BemLoad at(double omega, double axialSpeed, const Air& air,
                             double edgewiseSpeed = 0.0) const;
    [[nodiscard]] bool empty() const { return rows_.empty(); }

private:
    struct Row {
        double thrust, torque, induced, thrustRadius;  // thrustRadius as a fraction of R
        bool vortexRing;
    };
    [[nodiscard]] BemLoad axial(double omega, double axialSpeed, const Air& air) const;
    struct EdgeRow {
        double thrust, torque, inPlane0, inPlane1, moment0, moment1;
    };
    double radius_{0.0};
    int machs_{1};
    std::vector<Row> rows_;       // per Mach number, per inflow angle
    std::vector<EdgeRow> edges_;  // per edgewise angle, per inflow angle (incompressible)
};

/// The propeller's blades and hub as a triangle mesh in its own frame: thrust along +z, the first
/// blade along +x, each blade's leading edge ahead in the propeller's hand and its cambered side
/// to the front, sections of NACA four-digit shape (the station's thickness, the aerofoil's
/// drawing camber).
[[nodiscard]] TriMesh propellerMesh(const Propeller& p);

/// For a folding propeller: the hub with the blade roots inboard of the hinges (the propeller's
/// frame, as propellerMesh), and one blade outboard of its hinge in a frame at the hinge: x
/// along the blade, z the thrust direction; turned by Ry(−β) for its angle β.
[[nodiscard]] TriMesh foldingHubMesh(const Propeller& p);
[[nodiscard]] TriMesh foldingBladeMesh(const Propeller& p);

/// Draws every rotor with a propeller file (aero:rotor:propeller) as its blades: the mesh on the
/// rotor's link, turned so the propeller's thrust direction is the rotor's (the joint axis, or
/// against it for a pusher), in place of the link's other visuals, which a propeller file
/// replaces. A file that cannot be read is noted in model.notes and the link keeps its visuals.
/// For a loaded model only: what an importer writes keeps the authored visuals.
void addPropellerBlades(Model& model);

/// Thrust and power coefficients of `p` at advance ratio J = V/(nD), as in propeller test reports:
/// CT = T/(ρn²D⁴), CP = P/(ρn³D⁵). Speed of rotation given by `tipSpeed` (m/s).
struct PropellerCoefficients {
    double ct{0.0};
    double cp{0.0};
};
[[nodiscard]] PropellerCoefficients coefficientsAt(const Propeller& p, double advanceRatio,
                                                   double tipSpeed, const Air& air);

}  // namespace lagrange::rotor_aero
