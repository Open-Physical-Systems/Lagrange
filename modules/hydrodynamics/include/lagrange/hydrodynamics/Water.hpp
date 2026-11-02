#pragma once

// Water (docs/design/domains/hydrodynamics.md §9): a box of water. Buoyancy, quadratic drag and
// added mass on whatever moving bodies enter it, from cells sampled in their colliders, in a
// steady current and linear waves. Not Fossen's full manoeuvring model: the added mass is a
// sphere's, the same along every axis, and it carries no Coriolis terms.

#include "lagrange/module/IModule.hpp"
#include "lagrange/world/Model.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace lagrange::hydrodynamics {

struct WaterBody {
    std::string name{"water"};
    std::array<double, 3> surface{};              // the middle of the surface, world ENU
    double sizeX{10.0}, sizeY{10.0}, depth{2.0};  // m
    double density{1000.0};                       // kg/m³
    double drag{1.0};                             // 1, illustrative
    double reflectivity{0.02};                    // 1
};

/// The WaterAPI_1 prims of `model`, out of its schema instances (addWater puts them back).
[[nodiscard]] std::vector<WaterBody> takeWater(Model& model);

/// Adds a body of water: a drawn surface and a WaterAPI_1 instance that the module and the lidars
/// read. It has no collider: nothing stands on water, and a robot on the ground under it stays on
/// the ground.
void addWater(Model& model, const WaterBody& water);

/// The water bodies a world holds (for the module, and for the lidars' surface).
[[nodiscard]] std::vector<WaterBody> waterOf(const WorldView& world);

/// Writes body/<b>/wrench/water (world_local, about the centre of mass) for every moving,
/// non-kinematic body with colliders, and body/<b>/submerged (the fraction of its colliders'
/// volume under water), at 1 ms: buoyancy ρ g V at each submerged cell, and drag
/// ½ ρ Cd A |v| v from the cell's velocity through the water. Each collider is cut into cells
/// (kCells a side, kept when their centre is inside it, scaled to the shape's true volume).
///
/// The water moves (config "current_x", "current_y", "current_z", m/s; "wave_amplitude", m,
/// "wave_period", s, "wave_towards", degrees from +x): a steady current plus one train of linear
/// (Airy) waves, ω² = g·k·tanh(k·d) over the body's depth d, the surface at η = a·cos(k·x − ω·t)
/// and the water under it moving in the orbits of linear theory (Dean and Dalrymple, Water Wave
/// Mechanics for Engineers and Scientists, 1991, ch. 3). Cells are under the moving surface; the
/// drag is against the water's velocity there. A submerged body is also pushed by the water's
/// acceleration and has added mass (Morison's inertia terms): ρ·V·a_w + m_a·(a_w − a_b), with
/// m_a = ½·ρ·V the added mass of a sphere (potential flow), V the volume under water. a_b is
/// the body's acceleration over the last step, smoothed with gain 1/(1 + m_a/m) so a light body
/// (m_a > m) stays stable: for a steady push it settles in one step to the exact F/(m + m_a).
class Water final : public IModule {
public:
    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

private:
    struct Cell {
        std::array<double, 3> at;  // in the body's frame
        double volume;             // m³
        double area{0.0};          // m², its share of the frontal area, all under water
        double height;             // m, of the cell, for how much of it is under the surface
    };
    struct Body {
        SignalId pose, twist, wrench, submerged;
        std::array<double, 3> com{};
        std::vector<Cell> cells;
        double volume{0.0};
        double reach{0.0};  // m, of its farthest cell from its origin
        double mass{0.0};   // kg
        // For the added mass: the velocity of its origin at the last step, and its smoothed
        // acceleration (world).
        std::array<double, 3> lastVelocity{};
        std::array<double, 3> acceleration{};
        bool started{false};
    };
    /// The water's velocity and acceleration at a point of `water` (m/s, m/s²), at time `t`, and
    /// the surface's height there.
    struct Flow {
        std::array<double, 3> velocity{}, acceleration{};
        double surface{0.0};
    };
    [[nodiscard]] Flow flowAt(const WaterBody& water, double wavenumber,
                              const std::array<double, 3>& at, double t) const;
    std::vector<WaterBody> waters_;
    std::vector<double> wavenumbers_;  // per water body, rad/m (0: no waves)
    std::vector<Body> bodies_;
    std::array<double, 3> gravity_{};
    std::array<double, 3> current_{};
    double waveAmplitude_{0.0}, waveOmega_{0.0};
    std::array<double, 2> waveHeading_{1.0, 0.0};  // unit, horizontal
};

}  // namespace lagrange::hydrodynamics
