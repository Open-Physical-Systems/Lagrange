#pragma once

// The air box's step on the GPU: gpu/kernels/fields/lbm_box.slang and its host side. The solver's
// logic lives with the CPU reference in modules/aerodynamics (LbmBox); this only moves its grid
// along.

#include "lagrange/gpu/Device.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>

namespace lagrange::gpu {

class LbmBoxStepper {
public:
    struct Settings {
        std::size_t nx{1}, ny{1}, nz{1};
        bool floor{false};
        float tau{0.5F};
        float smagorinsky{0.0F};
    };
    /// One rotor, in cells from the box's corner, and the lattice force on the air per unit of
    /// the disc's weight (see lbm_box.slang).
    struct Disc {
        std::array<float, 4> centreRadius{};
        std::array<float, 4> axis{};
        std::array<float, 4> push{};
    };
    static constexpr std::size_t kMaxDiscs = 16;

    /// `f` holds 19 per cell, one direction after another; `solid` 1 per cell. Both are copied to
    /// the device, the populations in half precision. The box must be an even number of cells
    /// along x (else not valid()). `device` must outlive this.
    LbmBoxStepper(const Device& device, const Settings& settings, std::span<const float> f,
                  std::span<const std::uint32_t> solid);
    ~LbmBoxStepper();
    LbmBoxStepper(const LbmBoxStepper&) = delete;
    LbmBoxStepper& operator=(const LbmBoxStepper&) = delete;

    [[nodiscard]] bool valid() const;
    /// The wind round the box from the next step on, lattice units.
    void setAmbient(const std::array<float, 3>& wind);
    /// The rotors from the next step on; at most kMaxDiscs.
    void setDiscs(std::span<const Disc> discs);
    /// Advances `steps` steps and waits.
    void step(std::size_t steps);
    /// Copies out the populations (19 per cell) or the velocity (3 per cell, one component after
    /// another, lattice units).
    void readPopulations(std::span<float> f);
    void readVelocity(std::span<float> velocity);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace lagrange::gpu
