#pragma once

#include "lagrange/gpu/Resources.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

#include <vulkan/vulkan.h>

namespace lagrange::gpu {

class Device;

/// A compute shader with `storageBuffers` storage buffers at bindings 0..n-1 of set 0 and an
/// optional push-constant block. It owns one descriptor set, so one dispatch at a time: rebind
/// and dispatch under the caller's lock, and wait before the next.
struct ComputeKernel {
    VkDescriptorSetLayout setLayout{VK_NULL_HANDLE};
    VkPipelineLayout layout{VK_NULL_HANDLE};
    VkPipeline pipeline{VK_NULL_HANDLE};
    VkDescriptorPool pool{VK_NULL_HANDLE};
    VkDescriptorSet set{VK_NULL_HANDLE};
    std::uint32_t storageBuffers{0};
    std::uint32_t pushConstantBytes{0};
};

/// Builds the pipeline from SPIR-V (entry point "main", the name slangc gives a single entry).
/// Returns an empty kernel (pipeline VK_NULL_HANDLE) on failure.
[[nodiscard]] ComputeKernel createComputeKernel(const Device& device,
                                                std::span<const std::uint32_t> spirv,
                                                std::uint32_t storageBuffers,
                                                std::uint32_t pushConstantBytes);
void destroy(const Device& device, ComputeKernel& kernel);

/// Points binding i at the whole of buffers[i]; buffers.size() must be storageBuffers.
void bindBuffers(const Device& device, const ComputeKernel& kernel,
                 std::span<const Buffer* const> buffers);

/// Records the dispatch of `groups` workgroups along x, with `pushConstants` (pushConstantBytes
/// long), followed by a barrier that makes its writes visible to host reads.
void recordDispatch(VkCommandBuffer commands, const ComputeKernel& kernel,
                    std::span<const std::byte> pushConstants, std::uint32_t groups);

}  // namespace lagrange::gpu
