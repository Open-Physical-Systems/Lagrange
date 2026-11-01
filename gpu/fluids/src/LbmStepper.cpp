#include "lagrange/gpu/Compute.hpp"
#include "lagrange/gpu/LbmStepper.hpp"
#include "lagrange/gpu/Resources.hpp"
#include "shaders/kLbm.hpp"

#include <algorithm>
#include <array>
#include <cstring>

namespace lagrange::gpu {

namespace {

constexpr std::uint32_t kGroupSize = 256;  // [numthreads(256, 1, 1)] in lbm.slang
constexpr std::size_t kStepsPerSubmit = 32;

struct Params {
    std::int32_t nx, ny, nz, periodic;
    float tau, inletU, smagorinsky2, pad;
};

}  // namespace

struct LbmStepper::Impl {
    const Device* device{nullptr};
    Params params{};
    std::uint32_t groups{0};
    std::array<Buffer, 2> grids;
    Buffer solid;
    Buffer staging;
    // One kernel per direction: each reads one grid and writes the other.
    std::array<ComputeKernel, 2> kernels;
    std::size_t front{0};
    VkDeviceSize bytes{0};
};

LbmStepper::LbmStepper(const Device& device, const Settings& settings, std::span<const float> f,
                       std::span<const std::uint32_t> solid)
    : impl_(std::make_unique<Impl>()) {
    Impl& s = *impl_;
    s.device = &device;
    s.params = {static_cast<std::int32_t>(settings.nx),
                static_cast<std::int32_t>(settings.ny),
                static_cast<std::int32_t>(settings.nz),
                settings.periodic ? 1 : 0,
                settings.tau,
                settings.inletU,
                settings.smagorinsky * settings.smagorinsky,
                0.0F};
    const std::size_t cells = settings.nx * settings.ny * settings.nz;
    s.groups = static_cast<std::uint32_t>((cells + kGroupSize - 1) / kGroupSize);
    s.bytes = f.size_bytes();
    constexpr VkBufferUsageFlags kUsage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                          VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
                                          VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    s.grids[0] = createBufferWithData(device, std::as_bytes(f), kUsage);
    s.grids[1] = createBufferWithData(device, std::as_bytes(f), kUsage);
    s.solid = createBufferWithData(device, std::as_bytes(solid), kUsage);
    s.staging = createBuffer(device, s.bytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT, true);
    for (std::size_t k = 0; k < 2; ++k) {
        s.kernels.at(k) = createComputeKernel(device, kLbm, 3, sizeof(Params));
        if (s.kernels.at(k).pipeline != VK_NULL_HANDLE) {
            const std::array<const Buffer*, 3> bound{&s.grids.at(k), &s.grids.at(1 - k), &s.solid};
            bindBuffers(device, s.kernels.at(k), bound);
        }
    }
}

LbmStepper::~LbmStepper() {
    Impl& s = *impl_;
    for (ComputeKernel& kernel : s.kernels) {
        destroy(*s.device, kernel);
    }
    for (Buffer& grid : s.grids) {
        destroy(*s.device, grid);
    }
    destroy(*s.device, s.solid);
    destroy(*s.device, s.staging);
}

bool LbmStepper::valid() const {
    return impl_->kernels[0].pipeline != VK_NULL_HANDLE &&
           impl_->kernels[1].pipeline != VK_NULL_HANDLE;
}

void LbmStepper::step(std::size_t steps) {
    Impl& s = *impl_;
    while (steps > 0) {
        const std::size_t now = std::min(steps, kStepsPerSubmit);
        steps -= now;
        s.device->submitAndWait([&s, now](VkCommandBuffer commands) {
            for (std::size_t i = 0; i < now; ++i) {
                recordDispatch(commands, s.kernels.at(s.front),
                               std::as_bytes(std::span(&s.params, 1)), s.groups);
                // The next step reads what this one wrote.
                VkMemoryBarrier2 barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER_2};
                barrier.srcStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
                barrier.srcAccessMask = VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
                barrier.dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
                barrier.dstAccessMask =
                    VK_ACCESS_2_SHADER_STORAGE_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
                VkDependencyInfo dependency{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
                dependency.memoryBarrierCount = 1;
                dependency.pMemoryBarriers = &barrier;
                vkCmdPipelineBarrier2(commands, &dependency);
                s.front = 1 - s.front;
            }
        });
    }
}

void LbmStepper::read(std::span<float> f) {
    Impl& s = *impl_;
    s.device->submitAndWait([&s](VkCommandBuffer commands) {
        VkBufferCopy region{};
        region.size = s.bytes;
        vkCmdCopyBuffer(commands, s.grids.at(s.front).buffer, s.staging.buffer, 1, &region);
        VkMemoryBarrier2 barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER_2};
        barrier.srcStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
        barrier.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
        barrier.dstStageMask = VK_PIPELINE_STAGE_2_HOST_BIT;
        barrier.dstAccessMask = VK_ACCESS_2_HOST_READ_BIT;
        VkDependencyInfo dependency{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
        dependency.memoryBarrierCount = 1;
        dependency.pMemoryBarriers = &barrier;
        vkCmdPipelineBarrier2(commands, &dependency);
    });
    std::memcpy(f.data(), s.staging.mapped, std::min<std::size_t>(s.bytes, f.size_bytes()));
}

}  // namespace lagrange::gpu
