#pragma once

// A box of air round a multicopter, by the lattice Boltzmann method (D3Q19, BGK with a
// Smagorinsky eddy viscosity, as the wind tunnel, Lbm.hpp), with the rotors as actuator discs
// pushing the air through Guo's forcing. The box's sides and top are open at ambient pressure;
// its bottom is either open too or a no-slip ground. See docs/design/domains/rotor_aero.md.
//
// The CPU path is deterministic (threads share out whole z slabs); the GPU path
// (gpu/kernels/fields/lbm_box.slang) runs the same update in float and is not (invariant 7).

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace lagrange::gpu {
class Device;
class LbmBoxStepper;
}  // namespace lagrange::gpu

namespace lagrange::aerodynamics {

struct LbmBoxConfig {
    std::size_t nx{40}, ny{40}, nz{40};  ///< Cells.
    double cellSize{0.05};               ///< m
    /// The fastest air expected, m/s; with latticeSpeed it sets the time step.
    double maxSpeed{15.0};
    /// Lattice speed of maxSpeed. Below about 0.15 so the flow stays well below the lattice's
    /// speed of sound (compressibility error goes as its square).
    double latticeSpeed{0.1};
    double airDensity{1.225};           ///< kg/m³
    double kinematicViscosity{1.5e-5};  ///< m²/s
    double smagorinsky{0.17};           ///< Smagorinsky constant; 0: none
    bool floor{false};                  ///< true: the bottom face is ground
};

class LbmBox {
public:
    /// A rotor as an actuator disc, in the box's frame (m from its corner).
    struct Rotor {
        std::array<double, 3> centre{};
        std::array<double, 3> axis{0.0, 0.0, 1.0};  ///< unit; thrust on the rotor along it
        double radius{0.1};                         ///< m
        double thrust{0.0};                         ///< N, on the rotor; the air gets −thrust·axis
    };

    explicit LbmBox(const LbmBoxConfig& config);
    ~LbmBox();
    LbmBox(const LbmBox&) = delete;
    LbmBox& operator=(const LbmBox&) = delete;

    /// Steps on this device from now on (call after the solids are set).
    void useGpu(const gpu::Device& device);
    [[nodiscard]] bool onGpu() const { return stepper_ != nullptr; }

    [[nodiscard]] const LbmBoxConfig& config() const { return config_; }
    [[nodiscard]] double timeStep() const { return dt_; }  ///< s of air per step
    [[nodiscard]] std::size_t cells() const { return config_.nx * config_.ny * config_.nz; }
    [[nodiscard]] std::size_t index(std::size_t x, std::size_t y, std::size_t z) const {
        return x + config_.nx * (y + config_.ny * z);
    }

    /// Marks a cell solid (a still wall, bounce-back) before stepping.
    void setSolid(std::size_t x, std::size_t y, std::size_t z);
    /// The rotors from the next step on.
    void setRotors(std::span<const Rotor> rotors);
    /// Advances `steps` steps.
    void step(std::size_t steps);
    /// On the GPU, copies the velocity back so velocity() and velocityAt() see the present flow.
    void syncVelocity();

    /// m/s at a cell; 0 in a solid.
    [[nodiscard]] std::array<double, 3> velocity(std::size_t cell) const;
    /// m/s at a point of the box (m from its corner), trilinear between cell centres, clamped to
    /// the box.
    [[nodiscard]] std::array<double, 3> velocityAt(const std::array<double, 3>& point) const;
    /// The lattice weight of each rotor's disc summed over the cells, for tests: the push is
    /// spread so the cells together get exactly the thrust.
    [[nodiscard]] double discWeightSum(const Rotor& rotor) const;

    [[nodiscard]] bool diverged() const;

private:
    void stepOnce();
    void pushDiscs();

    LbmBoxConfig config_;
    double dt_{0.0};
    double tau_{0.5};
    double forceScale_{0.0};       // N per lattice unit of force
    double speedScale_{0.0};       // m/s per lattice unit of speed
    std::vector<float> f_, next_;  // 19 per cell, one direction after another
    std::vector<float> velocity_;  // 3 per cell, one component after another, lattice units
    std::vector<std::uint8_t> solid_;
    struct Disc {
        std::array<double, 3> centre{};  // cells
        std::array<double, 3> axis{};
        double radius{0.0};            // cells
        std::array<double, 3> push{};  // lattice force per unit weight
    };
    std::vector<Disc> discs_;
    std::unique_ptr<gpu::LbmBoxStepper> stepper_;
};

}  // namespace lagrange::aerodynamics
