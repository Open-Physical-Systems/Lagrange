#pragma once

// Tyre force models (domains/vehicle_dynamics.md §2): linear with a friction ellipse, and the
// Magic Formula of MF-Tyre 5.2 for pure and combined slip.

#include "lagrange/module/Expected.hpp"

#include <map>
#include <string>

namespace lagrange::vehicle_dynamics {

/// What a tyre gives at one contact: longitudinal force along the wheel's heading (positive
/// forward) and lateral force (positive to the wheel's left), N.
struct TyreForce {
    double fx{0.0};
    double fy{0.0};
};

/// The Magic Formula's curve D·sin(C·atan(B·x − E·(B·x − atan(B·x)))).
[[nodiscard]] double magicFormula(double x, double b, double c, double d, double e);

/// MF-Tyre 5.2 coefficients, as a .tir file names them (scaling factors λ = 1).
struct MagicFormula {
    double fz0{0.0};             // FNOMIN, N
    double unloadedRadius{0.0};  // UNLOADED_RADIUS, m (informative)
    // Longitudinal, pure slip.
    double pcx1{0.0}, pdx1{0.0}, pdx2{0.0}, pex1{0.0}, pex2{0.0}, pex3{0.0}, pex4{0.0};
    double pkx1{0.0}, pkx2{0.0}, pkx3{0.0};
    // Lateral, pure slip.
    double pcy1{0.0}, pdy1{0.0}, pdy2{0.0}, pey1{0.0}, pey2{0.0}, pey3{0.0};
    double pky1{0.0}, pky2{0.0};
    // Combined slip.
    double rbx1{0.0}, rbx2{0.0}, rcx1{1.0}, rex1{0.0}, rex2{0.0};
    double rby1{0.0}, rby2{0.0}, rby3{0.0}, rcy1{1.0}, rey1{0.0}, rey2{0.0};

    /// The coefficients of a .tir property file (TNO/TYDEX format: [SECTION] headers,
    /// `NAME = value $comment` lines); an error naming the first missing one.
    [[nodiscard]] static Expected<MagicFormula, std::string> fromTir(const std::string& path);
    [[nodiscard]] static Expected<MagicFormula, std::string> fromValues(
        const std::map<std::string, double>& values);
};

/// The forces at slip ratio `kappa` (wheel speed over ground speed minus one, positive when
/// driving), slip angle `alpha` (rad, positive when the contact slides to the wheel's left),
/// load `fz` (N) and friction scale `muScale` (the surface's and the weather's, 1 on the
/// reference surface).
///
/// Pure slip by MF-Tyre 5.2 with load dependence through dfz = (Fz − Fz0)/Fz0 and camber 0;
/// combined slip by its weighting functions Gxα and Gyκ. The curves are made odd: the shifts
/// (SH, SV), conicity and ply steer are not applied, and the forces oppose the slip whatever
/// the file's sign convention. No aligning moment, no turn slip.
[[nodiscard]] TyreForce pacejka(const MagicFormula& mf, double kappa, double alpha, double fz,
                                double muScale);

/// Linear tyre: Fx = Cκ·κ, Fy = −Cα·α, scaled back onto the friction ellipse of radius μ·Fz.
[[nodiscard]] TyreForce linearTyre(double longitudinalStiffness, double corneringStiffness,
                                   double mu, double kappa, double alpha, double fz);

}  // namespace lagrange::vehicle_dynamics
