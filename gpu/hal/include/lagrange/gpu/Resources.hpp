#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include <vulkan/vulkan.h>

struct VmaAllocation_T;
using VmaAllocation = VmaAllocation_T*;

namespace lagrange::gpu {

class Device;

struct Buffer {
    VkBuffer buffer{VK_NULL_HANDLE};
    VmaAllocation allocation{nullptr};
    VkDeviceSize size{0};
    void* mapped{nullptr};  ///< Non-null for host-visible buffers.
};

struct Image {
    VkImage image{VK_NULL_HANDLE};
    VkImageView view{VK_NULL_HANDLE};
    VmaAllocation allocation{nullptr};
    VkFormat format{VK_FORMAT_UNDEFINED};
    VkExtent2D extent{};
};

[[nodiscard]] Buffer createBuffer(const Device& device, VkDeviceSize size, VkBufferUsageFlags usage,
                                  bool hostVisible);
/// A device-local buffer holding `data`, uploaded through a temporary staging buffer.
[[nodiscard]] Buffer createBufferWithData(const Device& device, std::span<const std::byte> data,
                                          VkBufferUsageFlags usage);
void destroy(const Device& device, Buffer& buffer);
/// Makes host writes to a host-visible buffer visible to the GPU (needed on non-coherent memory).
void flush(const Device& device, const Buffer& buffer);

[[nodiscard]] Image createImage2D(const Device& device, VkExtent2D extent, VkFormat format,
                                  VkImageUsageFlags usage, VkImageAspectFlags aspect,
                                  VkSampleCountFlagBits samples = VK_SAMPLE_COUNT_1_BIT);
void destroy(const Device& device, Image& image);

[[nodiscard]] VkShaderModule createShaderModule(const Device& device,
                                                std::span<const std::uint32_t> spirv);

/// Records a layout transition for a whole single-mip, single-layer image.
void transitionImage(VkCommandBuffer commands, VkImage image, VkImageAspectFlags aspect,
                     VkImageLayout from, VkImageLayout to);

/// Copies an RGBA8 colour image to host memory (row-major, 4 bytes per pixel). The image must
/// be in `layout` and is returned to it. Waits for the GPU.
[[nodiscard]] std::vector<std::uint8_t> readbackRgba8(const Device& device, const Image& image,
                                                      VkImageLayout layout);

/// Writes an 8-bit RGBA PNG. Returns false if the file cannot be written.
[[nodiscard]] bool writePng(const std::string& path, std::uint32_t width, std::uint32_t height,
                            std::span<const std::uint8_t> rgba);

}  // namespace lagrange::gpu
