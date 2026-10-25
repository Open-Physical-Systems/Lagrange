#pragma once

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
struct Propeller {
    std::string name;
    double radius{0.0};  // m, tip
    int blades{0};
    std::string hand;
    std::vector<BladeStation> stations;  // by radius, the first at the root of the aerodynamic
                                         // blade (the hub's edge)
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
/// (flight speed; negative: the air comes from behind, as in a descent).
///
/// Momentum theory has no steady solution in the vortex-ring state (descent at about the induced
/// velocity); there the annulus takes the blade-element forces with no induced velocity, the
/// common engineering treatment, and `vortexRing` is set.
struct BemLoad {
    double thrust{0.0};  // N, along the thrust direction
    double torque{0.0};  // N·m, the air's torque on the propeller about the thrust direction,
                         // positive against rotation in the hand (what the motor must give)
    double inducedVelocity{0.0};  // m/s, thrust-weighted mean axial induced velocity
    bool vortexRing{false};
};
[[nodiscard]] BemLoad bemLoad(const Propeller& p, double omega, double axialSpeed, const Air& air);

/// A propeller's loads tabulated once (aerodynamics.md §2.1): blade-element momentum theory over
/// the full circle of inflow angles θ = atan2(V, ΩR) at 0.25° steps (turning either way, stopped,
/// in climb, descent or windmilling), and, when `compressible`, over tip Mach numbers 0 to 0.9 at
/// 0.1 steps. Loads scale exactly with ρ·(Ω²R² + V²) (the induced velocities with the resultant
/// tip speed) in incompressible flow, so a lookup gives what bemLoad gives, to the interpolation:
/// what the rotor module steps with, at a fixed cost.
class PropellerTable {
public:
    [[nodiscard]] static PropellerTable build(const Propeller& p, bool compressible);
    [[nodiscard]] BemLoad at(double omega, double axialSpeed, const Air& air) const;
    [[nodiscard]] bool empty() const { return rows_.empty(); }

private:
    struct Row {
        double thrust, torque, induced;
        bool vortexRing;
    };
    double radius_{0.0};
    int machs_{1};
    std::vector<Row> rows_;  // per Mach number, per inflow angle
};

/// The propeller's blades and hub as a triangle mesh in its own frame: thrust along +z, the first
/// blade along +x, each blade's leading edge ahead in the propeller's hand and its cambered side
/// to the front, sections of NACA four-digit shape (the station's thickness, the aerofoil's
/// drawing camber).
[[nodiscard]] TriMesh propellerMesh(const Propeller& p);

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
