// SPDX-License-Identifier: Apache-2.0
// What a surface's ground terms (SurfaceMaterial::frictionScale, rollingResistance) do to engine
// contacts, shared by the MuJoCo and Jolt backends so both engines feel grass the same way.
// docs/physics/ground.md "Engine contacts on grass".
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include <lagrange/world/World.hpp>

namespace lagrange {

/// The surface of collider `c`: its material, or the defaults (hard, bare ground).
[[nodiscard]] inline SurfaceMaterial colliderSurface(const World& world, std::size_t c) {
    const std::uint32_t m =
        c < world.colliders.material.size() ? world.colliders.material[c] : kNoMaterial;
    return m != kNoMaterial && m < world.surfaceMaterials.size() ? world.surfaceMaterials[m]
                                                                 : SurfaceMaterial{};
}

/// Collider `c`'s Coulomb coefficient as the engine model is built: its own times its surface's
/// frictionScale. The engines then combine a pair by the larger of the two (MuJoCo's rule for
/// geom friction, the Jolt backend's SetCombineFriction).
[[nodiscard]] inline double colliderFrictionOnSurface(const World& world, std::size_t c) {
    return world.colliders.friction[c] * colliderSurface(world, c).frictionScale;
}

/// Rolling resistance of one contact, as a torque on the rolling body (N·m, world frame), the
/// other body takes its opposite. The torque opposes the rolling part of the bodies' relative
/// spin (`omegaRel`, rad/s, the part about the contact normal `normal` is left alone) with
/// magnitude crr · normalForce · radius: a coefficient of rolling resistance crr is the
/// horizontal force over the load, and it acts at the rolling radius (Wong, Theory of Ground
/// Vehicles, 4th ed., ch. 1, TODO(verify) section). It is a Coulomb-like limit: never more than
/// stops the relative rolling spin in one step of `dt`, with `inertia` the roller's moment of
/// inertia about the contact point (kg·m²), so a roller at rest stays at rest and never rolls back.
[[nodiscard]] inline std::array<double, 3> rollingResistanceTorque(
    const std::array<double, 3>& omegaRel, const std::array<double, 3>& normal, double normalForce,
    double radius, double crr, double inertia, double dt) {
    if (crr <= 0.0 || normalForce <= 0.0 || radius <= 0.0) {
        return {0.0, 0.0, 0.0};
    }
    const double along =
        omegaRel[0] * normal[0] + omegaRel[1] * normal[1] + omegaRel[2] * normal[2];
    std::array<double, 3> rolling{};
    for (std::size_t k = 0; k < 3; ++k) {
        rolling.at(k) = omegaRel.at(k) - along * normal.at(k);
    }
    const double speed =
        std::sqrt(rolling[0] * rolling[0] + rolling[1] * rolling[1] + rolling[2] * rolling[2]);
    if (speed <= 0.0) {
        return {0.0, 0.0, 0.0};
    }
    double magnitude = crr * normalForce * radius;
    if (dt > 0.0 && inertia > 0.0) {
        magnitude = std::min(magnitude, inertia * speed / dt);
    }
    return {-magnitude * rolling[0] / speed, -magnitude * rolling[1] / speed,
            -magnitude * rolling[2] / speed};
}

}  // namespace lagrange
