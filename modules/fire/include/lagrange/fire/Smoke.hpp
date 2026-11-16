#pragma once

// Smoke (RFC 0022 §4): soot from every fire as Lagrangian puffs, rasterised into an extinction
// field for the cameras, the lidars and the visibility sensor. Pure: no bus, so its laws are
// tested alone (test_smoke.cpp); the fire module feeds it each step.
//
// - Production: ṁ_s = y_s·Q/ΔH_c (kg/s); every step each burning fire emits one puff of ṁ_s·dt
//   at its flame's tip.
// - Transport: a puff is a Gaussian (σ_h across, σ_v up) that rises at Heskestad's plume
//   centreline velocity u0 at its height above its fire's virtual origin and is carried by the
//   wind. While it rises its width follows the plume's, σ = (b/√(2 ln 2)), b = β·(z − z0). A ray
//   up that meets the fixed scenery within its next rise stops it under that ceiling; there it
//   keeps its depth and spreads sideways at the velocity it arrived with (a modelling choice; a
//   ceiling-jet law, Alpert 1972, is owed). It leaves beyond the smoke box. No deposition.
// - Optics: σ = K_m·ρ_soot, K_m = 8700 m²/kg at 633 nm (Mulholland and Croarkin, Fire and
//   Materials 24, 2000, as the FDS 6.11.1 guides state it). The sensors scale it to their band
//   and read visibility from it (sensors/WeatherEffects.hpp, sensors.md).
//
// Deterministic: fixed order, no random numbers (invariant 7).

#include "lagrange/fields/Field.hpp"
#include "lagrange/services/SceneQuery.hpp"

#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <vector>

namespace lagrange::fire {

/// The specific extinction coefficient of flame-generated smoke at 633 nm (m²/kg): Mulholland and
/// Croarkin, "Specific extinction coefficient of flame generated smoke", Fire and Materials 24
/// (2000) 227–230, as the FDS 6.11.1 User's and Technical Reference Guides give it ("8700 m²/kg
/// ± 1100 m²/kg at a wavelength of 633 nm").
inline constexpr double kSpecificExtinction = 8700.0;
/// σ (1/m) of a soot density (kg/m³) at 633 nm. The sensors scale it to their own band and turn
/// it into a visibility (sensors::smokeBandScale, smokeVisibility).
[[nodiscard]] double smokeExtinction(double sootDensity);

using Vec3 = std::array<double, 3>;

/// One fire's plume this step, as the smoke sees it.
struct PlumeSource {
    Vec3 base{};                      // m, world-local: the fuel surface's centre
    Vec3 tip{};                       // m: the flame's tip, where its soot leaves
    double hrr{0.0};                  // W
    double radiativeFraction{0.0};    // 1
    double virtualOriginHeight{0.0};  // m above the base (Heskestad's z0)
    double sootRate{0.0};             // kg/s
};

/// The air the puffs move in.
struct SmokeAir {
    Vec3 wind{};              // m/s, world-local
    double gravity{9.81};     // m/s²
    double temperature{0.0};  // K; 0: not known, the puffs do not rise
    double density{0.0};      // kg/m³; 0: not known
};

/// One puff.
struct Puff {
    Vec3 centre{};       // m, world-local
    double mass{0.0};    // kg of soot
    double sigmaH{0.0};  // m, across
    double sigmaV{0.0};  // m, up
    std::uint32_t fire{0};
    bool underCeiling{false};
    // m: the ceiling above it, nothing of it above that; +inf when there is none in reach.
    double top{std::numeric_limits<double>::infinity()};
};

struct SmokeSettings {
    double voxel{0.5};           // m, the field's voxel
    double reach{30.0};          // m: the smoke box, this far across from any fire's base
    double height{20.0};         // m: the box's top above the lowest fire's base
    std::size_t maxPuffs{2000};  // beyond: the two oldest puffs are merged
};

class Smoke {
public:
    /// The smoke box around the fires' bases (world-local, m).
    void configure(const SmokeSettings& settings, std::span<const Vec3> fireBases);

    /// No smoke (the run's start), the box kept.
    void clear() {
        puffs_.clear();
        left_ = 0.0;
    }

    /// Advances the puffs by `dt` (s) and emits this step's soot. `ceilings` (may be null): the
    /// fixed scenery a puff stops under.
    void step(double dt, std::span<const PlumeSource> fires, const SmokeAir& air,
              const services::SceneQuery* ceilings);

    /// The extinction field σ (1/m, at 633 nm) of the puffs now: each puff spread over the
    /// voxels within 3σ, its weights normalised so that its mass is kept exactly. Null when
    /// there is no smoke in the box.
    [[nodiscard]] std::shared_ptr<const fields::Field> raster() const;

    [[nodiscard]] std::span<const Puff> puffs() const { return puffs_; }
    /// Soot in the box (kg), and what has left it.
    [[nodiscard]] double mass() const;
    [[nodiscard]] double leftMass() const { return left_; }
    [[nodiscard]] const std::array<Vec3, 2>& box() const { return box_; }
    [[nodiscard]] const SmokeSettings& settings() const { return settings_; }

    void save(std::vector<double>& out) const;
    [[nodiscard]] bool restore(std::span<const double> in);

private:
    void mergeOldest();

    SmokeSettings settings_;
    std::array<Vec3, 2> box_{};
    std::vector<Puff> puffs_;  // oldest first
    double left_{0.0};
};

}  // namespace lagrange::fire
