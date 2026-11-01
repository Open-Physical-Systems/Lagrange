#pragma once

// A wind tunnel: the lattice Boltzmann method on a uniform grid (D3Q19, BGK with a Smagorinsky
// eddy viscosity, half-way bounce-back on solids, forces by momentum exchange). See
// docs/design/domains/aerodynamics.md. Deterministic: threads only share out whole z slabs, and
// what they add up is added in slab order (invariant 7).

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace lagrange::aerodynamics {

struct LbmConfig {
    std::size_t nx{64}, ny{64}, nz{64};  ///< Cells.
    double cellSize{0.01};               ///< m
    double windSpeed{10.0};              ///< m/s, along +x, at the inlet
    double airDensity{1.2};              ///< kg/m³
    double kinematicViscosity{1.5e-5};   ///< m²/s, air's at 20 °C
    /// Lattice speed of the wind: sets the time step. Below about 0.1, so the flow stays well below
    /// the lattice's speed of sound (compressibility error goes as its square).
    double latticeSpeed{0.05};
    /// Smagorinsky constant of the eddy viscosity; 0: none, for flow the grid resolves.
    double smagorinsky{0.17};
    /// true: periodic on every side and no wind forced, for checking the solver itself. false: a
    /// tunnel along x: inlet at x = 0, outlet at the far end, periodic sideways.
    bool periodic{false};
};

class LbmTunnel {
public:
    /// Force in N and torque in N·m on a solid, the torque about the origin of the grid's frame.
    struct Load {
        std::array<double, 3> force{};
        std::array<double, 3> torque{};
    };

    explicit LbmTunnel(const LbmConfig& config);

    [[nodiscard]] const LbmConfig& config() const { return config_; }
    [[nodiscard]] double timeStep() const { return dt_; }  ///< s of air per step
    /// The viscosity the lattice has without the eddy part, in lattice units: τ = 3ν + ½.
    [[nodiscard]] double relaxationTime() const { return tau_; }
    [[nodiscard]] std::size_t cells() const { return config_.nx * config_.ny * config_.nz; }
    [[nodiscard]] std::size_t index(std::size_t x, std::size_t y, std::size_t z) const {
        return x + config_.nx * (y + config_.ny * z);
    }

    /// Marks a cell solid, as part of solid `solid` (0 or more), before stepping. Solids get loads.
    void setSolid(std::size_t x, std::size_t y, std::size_t z, std::uint16_t solid);
    [[nodiscard]] bool isSolid(std::size_t cell) const { return solid_[cell] != kFluid; }

    /// Sets a cell's flow directly, in lattice units (velocity as a share of a cell per step), at
    /// equilibrium. For starting from a known flow.
    void setLatticeFlow(std::size_t cell, double density, const std::array<double, 3>& velocity);

    /// Advances `steps` steps. Loads are the mean over those steps.
    void step(std::size_t steps);
    [[nodiscard]] const std::vector<Load>& loads() const { return loads_; }

    /// Velocity in m/s and pressure in Pa above the inlet's, at a cell; 0 in a solid.
    [[nodiscard]] std::array<double, 3> velocity(std::size_t cell) const;
    [[nodiscard]] double pressure(std::size_t cell) const;
    /// In lattice units.
    [[nodiscard]] std::array<double, 3> latticeVelocity(std::size_t cell) const;

    /// From now on adds up the flow every few steps, for the mean flow past a body whose wake
    /// never settles. meanVelocity and meanPressure read it; step's loads are means already.
    void startAveraging();
    [[nodiscard]] std::array<double, 3> meanVelocity(std::size_t cell) const;  ///< m/s
    [[nodiscard]] double meanPressure(std::size_t cell) const;                 ///< Pa

    /// Whether anything has gone non-finite: the flow is too fast or the grid too coarse.
    [[nodiscard]] bool diverged() const;

    static constexpr std::uint16_t kFluid = 0xFFFF;

private:
    void stepOnce(std::vector<Load>& sum);

    LbmConfig config_;
    double dt_{0.0};
    double tau_{0.5};
    double forceScale_{0.0};       // N per lattice unit of momentum per step
    std::vector<float> f_, next_;  // 19 per cell
    std::vector<std::uint16_t> solid_;
    std::vector<Load> loads_;
    bool averaging_{false};
    std::size_t sinceSample_{0};
    std::size_t samples_{0};
    std::vector<float> meanVelocity_;  // 3 per cell, m/s, summed
    std::vector<float> meanPressure_;  // per cell, Pa, summed
};

}  // namespace lagrange::aerodynamics
