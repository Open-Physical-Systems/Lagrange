#pragma once

// The wind tunnel's step on the GPU: gpu/kernels/fields/lbm.slang and its host side. The solver's
// logic lives with the CPU reference in modules/aerodynamics; this only moves its grid along.

#include "lagrange/gpu/Device.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>

namespace lagrange::gpu {

class LbmStepper {
public:
    struct Settings {
        std::size_t nx{1}, ny{1}, nz{1};
        bool periodic{false};
        float tau{0.5F};
        float inletU{0.0F};
        float smagorinsky{0.0F};
    };
    /// `f` holds 19 per cell, `solid` 1 per cell; both are copied to the device. `device` must
    /// outlive this.
    LbmStepper(const Device& device, const Settings& settings, std::span<const float> f,
               std::span<const std::uint32_t> solid);
    ~LbmStepper();
    LbmStepper(const LbmStepper&) = delete;
    LbmStepper& operator=(const LbmStepper&) = delete;

    /// Whether the kernel could be built.
    [[nodiscard]] bool valid() const;
    /// Advances `steps` steps and waits.
    void step(std::size_t steps);
    /// Copies the present state out, 19 per cell.
    void read(std::span<float> f);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace lagrange::gpu
