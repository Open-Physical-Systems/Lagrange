#include "lagrange/gpu/Compute.hpp"
#include "lagrange/gpu/LbmBoxStepper.hpp"
#include "lagrange/gpu/Resources.hpp"
#include "shaders/kLbmBox.hpp"

#include <algorithm>
#include <cstring>
#include <vector>

namespace lagrange::gpu {

namespace {

constexpr std::uint32_t kGroupSize = 256;  // [numthreads(256, 1, 1)] in lbm_box.slang
constexpr std::size_t kStepsPerSubmit = 64;
constexpr std::uint32_t kBuffers = 5;
constexpr std::size_t kQ = 19;
constexpr std::array<float, kQ> kWeight{1.0F / 3.0F,  1.0F / 18.0F, 1.0F / 18.0F, 1.0F / 18.0F,
                                        1.0F / 18.0F, 1.0F / 18.0F, 1.0F / 18.0F, 1.0F / 36.0F,
                                        1.0F / 36.0F, 1.0F / 36.0F, 1.0F / 36.0F, 1.0F / 36.0F,
                                        1.0F / 36.0F, 1.0F / 36.0F, 1.0F / 36.0F, 1.0F / 36.0F,
                                        1.0F / 36.0F, 1.0F / 36.0F, 1.0F / 36.0F};

// IEEE 754 binary16, round to nearest even, as the kernel's f32tof16 (no infinities or NaNs
// expected: the values are populations less their rest weight, well under 1).
std::uint16_t toHalf(float value) {
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof bits);
    const std::uint32_t sign = (bits >> 16) & 0x8000U;
    const auto exponent = static_cast<std::int32_t>((bits >> 23) & 0xFFU) - 127 + 15;
    std::uint32_t mantissa = bits & 0x7FFFFFU;
    if (exponent <= 0) {
        if (exponent < -10) {
            return static_cast<std::uint16_t>(sign);
        }
        mantissa |= 0x800000U;
        const auto shift = static_cast<std::uint32_t>(14 - exponent);
        std::uint32_t half = mantissa >> shift;
        const std::uint32_t rest = mantissa & ((1U << shift) - 1U);
        const std::uint32_t midway = 1U << (shift - 1U);
        if (rest > midway || (rest == midway && (half & 1U) != 0)) {
            ++half;
        }
        return static_cast<std::uint16_t>(sign | half);
    }
    if (exponent >= 31) {
        return static_cast<std::uint16_t>(sign | 0x7BFFU);  // the largest finite
    }
    std::uint32_t half = (static_cast<std::uint32_t>(exponent) << 10) | (mantissa >> 13);
    const std::uint32_t rest = mantissa & 0x1FFFU;
    if (rest > 0x1000U || (rest == 0x1000U && (half & 1U) != 0)) {
        ++half;
    }
    return static_cast<std::uint16_t>(sign | half);
}

float fromHalf(std::uint16_t half) {
    const std::uint32_t sign = static_cast<std::uint32_t>(half & 0x8000U) << 16;
    std::uint32_t exponent = (half >> 10) & 0x1FU;
    std::uint32_t mantissa = half & 0x3FFU;
    std::uint32_t bits = 0;
    if (exponent == 0) {
        if (mantissa == 0) {
            bits = sign;
        } else {
            exponent = 127 - 15 + 1;
            while ((mantissa & 0x400U) == 0) {
                mantissa <<= 1;
                --exponent;
            }
            bits = sign | (exponent << 23) | ((mantissa & 0x3FFU) << 13);
        }
    } else {
        bits = sign | ((exponent - 15 + 127) << 23) | (mantissa << 13);
    }
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof value);
    return value;
}

// Populations (19 per cell, one direction after another) to the kernel's half pairs and back.
std::vector<std::uint32_t> pack(std::span<const float> f, std::size_t cells) {
    std::vector<std::uint32_t> out(kQ * cells / 2);
    for (std::size_t i = 0; i < kQ; ++i) {
        for (std::size_t c = 0; c < cells; ++c) {
            const std::uint32_t half = toHalf(f[i * cells + c] - kWeight.at(i));
            out[(i * cells + c) / 2] |= (c % 2 == 0) ? half : (half << 16);
        }
    }
    return out;
}

void unpack(std::span<const std::uint32_t> words, std::size_t cells, std::span<float> f) {
    for (std::size_t i = 0; i < kQ; ++i) {
        for (std::size_t c = 0; c < cells && i * cells + c < f.size(); ++c) {
            const std::uint32_t word = words[(i * cells + c) / 2];
            const auto half = static_cast<std::uint16_t>(c % 2 == 0 ? word & 0xFFFFU : word >> 16);
            f[i * cells + c] = fromHalf(half) + kWeight.at(i);
        }
    }
}

struct Params {
    std::int32_t nx, ny, nz, floor;
    float tau, smagorinsky2;
    std::int32_t discs;
    std::int32_t solids;
    std::array<float, 4> ambient;
};

void barrierAfter(VkCommandBuffer commands, VkPipelineStageFlags2 srcStage,
                  VkAccessFlags2 srcAccess, VkPipelineStageFlags2 dstStage,
                  VkAccessFlags2 dstAccess) {
    VkMemoryBarrier2 barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER_2};
    barrier.srcStageMask = srcStage;
    barrier.srcAccessMask = srcAccess;
    barrier.dstStageMask = dstStage;
    barrier.dstAccessMask = dstAccess;
    VkDependencyInfo dependency{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
    dependency.memoryBarrierCount = 1;
    dependency.pMemoryBarriers = &barrier;
    vkCmdPipelineBarrier2(commands, &dependency);
}

}  // namespace

struct LbmBoxStepper::Impl {
    const Device* device{nullptr};
    Params params{};
    std::uint32_t groups{0};
    std::array<Buffer, 2> grids;
    Buffer solid;
    Buffer velocity;
    Buffer discs;  // host visible
    Buffer staging;
    std::array<ComputeKernel, 2> kernels;
    std::size_t front{0};
    std::size_t cells{0};
    VkDeviceSize gridBytes{0};
    VkDeviceSize velocityBytes{0};
};

LbmBoxStepper::LbmBoxStepper(const Device& device, const Settings& settings,
                             std::span<const float> f, std::span<const std::uint32_t> solid)
    : impl_(std::make_unique<Impl>()) {
    Impl& s = *impl_;
    s.params = {
        static_cast<std::int32_t>(settings.nx),
        static_cast<std::int32_t>(settings.ny),
        static_cast<std::int32_t>(settings.nz),
        settings.floor ? 1 : 0,
        settings.tau,
        settings.smagorinsky * settings.smagorinsky,
        0,
        std::any_of(solid.begin(), solid.end(), [](std::uint32_t kind) { return kind != 0xFFFFU; })
            ? 1
            : 0,
        {0.0F, 0.0F, 0.0F, 0.0F}};
    const std::size_t cells = settings.nx * settings.ny * settings.nz;
    s.cells = cells;
    if (settings.nx % 2 != 0) {
        return;  // two cells to a word along x: not valid()
    }
    // One thread to two cells.
    s.groups = static_cast<std::uint32_t>((cells / 2 + kGroupSize - 1) / kGroupSize);
    const std::vector<std::uint32_t> halves = pack(f, cells);
    s.gridBytes = halves.size() * sizeof(std::uint32_t);
    s.device = &device;
    s.velocityBytes = cells * 3 * sizeof(float);
    constexpr VkBufferUsageFlags kUsage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                          VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
                                          VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    s.grids[0] = createBufferWithData(device, std::as_bytes(std::span(halves)), kUsage);
    s.grids[1] = createBufferWithData(device, std::as_bytes(std::span(halves)), kUsage);
    s.solid = createBufferWithData(device, std::as_bytes(solid), kUsage);
    const std::vector<float> still(cells * 3, 0.0F);
    s.velocity = createBufferWithData(device, std::as_bytes(std::span(still)), kUsage);
    s.discs =
        createBuffer(device, kMaxDiscs * sizeof(Disc), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, true);
    std::memset(s.discs.mapped, 0, kMaxDiscs * sizeof(Disc));
    s.staging = createBuffer(device, std::max(s.gridBytes, s.velocityBytes),
                             VK_BUFFER_USAGE_TRANSFER_DST_BIT, true);
    for (std::size_t k = 0; k < 2; ++k) {
        s.kernels.at(k) = createComputeKernel(device, kLbmBox, kBuffers, sizeof(Params));
        if (s.kernels.at(k).pipeline != VK_NULL_HANDLE) {
            const std::array<const Buffer*, kBuffers> bound{&s.grids.at(k), &s.grids.at(1 - k),
                                                            &s.solid, &s.velocity, &s.discs};
            bindBuffers(device, s.kernels.at(k), bound);
        }
    }
}

LbmBoxStepper::~LbmBoxStepper() {
    Impl& s = *impl_;
    if (s.device == nullptr) {
        return;
    }
    for (ComputeKernel& kernel : s.kernels) {
        destroy(*s.device, kernel);
    }
    for (Buffer& grid : s.grids) {
        destroy(*s.device, grid);
    }
    destroy(*s.device, s.solid);
    destroy(*s.device, s.velocity);
    destroy(*s.device, s.discs);
    destroy(*s.device, s.staging);
}

bool LbmBoxStepper::valid() const {
    return impl_->kernels[0].pipeline != VK_NULL_HANDLE &&
           impl_->kernels[1].pipeline != VK_NULL_HANDLE;
}

void LbmBoxStepper::setAmbient(const std::array<float, 3>& wind) {
    impl_->params.ambient = {wind[0], wind[1], wind[2], 0.0F};
}

void LbmBoxStepper::setDiscs(std::span<const Disc> discs) {
    Impl& s = *impl_;
    const std::size_t count = std::min(discs.size(), kMaxDiscs);
    std::memcpy(s.discs.mapped, discs.data(), count * sizeof(Disc));
    s.params.discs = static_cast<std::int32_t>(count);
}

void LbmBoxStepper::step(std::size_t steps) {
    Impl& s = *impl_;
    while (steps > 0) {
        const std::size_t now = std::min(steps, kStepsPerSubmit);
        steps -= now;
        s.device->submitAndWait([&s, now](VkCommandBuffer commands) {
            for (std::size_t i = 0; i < now; ++i) {
                recordDispatch(commands, s.kernels.at(s.front),
                               std::as_bytes(std::span(&s.params, 1)), s.groups);
                barrierAfter(
                    commands, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                    VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                    VK_ACCESS_2_SHADER_STORAGE_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);
                s.front = 1 - s.front;
            }
        });
    }
}

namespace {

void copyOut(const Device& device, const Buffer& from, const Buffer& staging, VkDeviceSize bytes,
             std::span<float> out) {
    device.submitAndWait([&](VkCommandBuffer commands) {
        VkBufferCopy region{};
        region.size = bytes;
        vkCmdCopyBuffer(commands, from.buffer, staging.buffer, 1, &region);
        barrierAfter(commands, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT,
                     VK_PIPELINE_STAGE_2_HOST_BIT, VK_ACCESS_2_HOST_READ_BIT);
    });
    std::memcpy(out.data(), staging.mapped, std::min<std::size_t>(bytes, out.size_bytes()));
}

}  // namespace

void LbmBoxStepper::readPopulations(std::span<float> f) {
    Impl& s = *impl_;
    std::vector<std::uint32_t> words(s.gridBytes / sizeof(std::uint32_t));
    copyOut(*s.device, s.grids.at(s.front), s.staging, s.gridBytes,
            std::span(reinterpret_cast<float*>(words.data()), words.size()));
    unpack(words, s.cells, f);
}

void LbmBoxStepper::readVelocity(std::span<float> velocity) {
    Impl& s = *impl_;
    copyOut(*s.device, s.velocity, s.staging, s.velocityBytes, velocity);
}

}  // namespace lagrange::gpu
