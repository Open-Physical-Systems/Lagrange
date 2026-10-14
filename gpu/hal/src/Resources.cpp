#include "lagrange/gpu/Device.hpp"
#include "lagrange/gpu/Resources.hpp"

#include <array>
#include <cstdio>
#include <cstring>
#include <memory>
#include <zlib.h>

#include <vk_mem_alloc.h>

namespace lagrange::gpu {

Buffer createBuffer(const Device& device, VkDeviceSize size, VkBufferUsageFlags usage,
                    bool hostVisible) {
    VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    info.size = size;
    info.usage = usage;
    VmaAllocationCreateInfo allocation{};
    allocation.usage = VMA_MEMORY_USAGE_AUTO;
    if (hostVisible) {
        allocation.flags =
            VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT;
    }
    Buffer buffer;
    buffer.size = size;
    VmaAllocationInfo result{};
    vmaCreateBuffer(device.allocator(), &info, &allocation, &buffer.buffer, &buffer.allocation,
                    &result);
    buffer.mapped = hostVisible ? result.pMappedData : nullptr;
    return buffer;
}

void flush(const Device& device, const Buffer& buffer) {
    vmaFlushAllocation(device.allocator(), buffer.allocation, 0, VK_WHOLE_SIZE);
}

Buffer createBufferWithData(const Device& device, std::span<const std::byte> data,
                            VkBufferUsageFlags usage) {
    Buffer staging = createBuffer(device, data.size(), VK_BUFFER_USAGE_TRANSFER_SRC_BIT, true);
    std::memcpy(staging.mapped, data.data(), data.size());
    vmaFlushAllocation(device.allocator(), staging.allocation, 0, VK_WHOLE_SIZE);
    Buffer buffer =
        createBuffer(device, data.size(), usage | VK_BUFFER_USAGE_TRANSFER_DST_BIT, false);
    device.submitAndWait([&](VkCommandBuffer commands) {
        const VkBufferCopy region{0, 0, data.size()};
        vkCmdCopyBuffer(commands, staging.buffer, buffer.buffer, 1, &region);
    });
    destroy(device, staging);
    return buffer;
}

void destroy(const Device& device, Buffer& buffer) {
    if (buffer.buffer != VK_NULL_HANDLE) {
        vmaDestroyBuffer(device.allocator(), buffer.buffer, buffer.allocation);
    }
    buffer = {};
}

Image createImage2D(const Device& device, VkExtent2D extent, VkFormat format,
                    VkImageUsageFlags usage, VkImageAspectFlags aspect,
                    VkSampleCountFlagBits samples) {
    VkImageCreateInfo info{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
    info.imageType = VK_IMAGE_TYPE_2D;
    info.format = format;
    info.extent = {extent.width, extent.height, 1};
    info.mipLevels = 1;
    info.arrayLayers = 1;
    info.samples = samples;
    info.tiling = VK_IMAGE_TILING_OPTIMAL;
    info.usage = usage;
    VmaAllocationCreateInfo allocation{};
    allocation.usage = VMA_MEMORY_USAGE_AUTO;

    Image image;
    image.format = format;
    image.extent = extent;
    vmaCreateImage(device.allocator(), &info, &allocation, &image.image, &image.allocation,
                   nullptr);

    VkImageViewCreateInfo viewInfo{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
    viewInfo.image = image.image;
    viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
    viewInfo.format = format;
    viewInfo.subresourceRange = {aspect, 0, 1, 0, 1};
    vkCreateImageView(device.device(), &viewInfo, nullptr, &image.view);
    return image;
}

void destroy(const Device& device, Image& image) {
    if (image.view != VK_NULL_HANDLE) {
        vkDestroyImageView(device.device(), image.view, nullptr);
    }
    if (image.image != VK_NULL_HANDLE) {
        vmaDestroyImage(device.allocator(), image.image, image.allocation);
    }
    image = {};
}

VkShaderModule createShaderModule(const Device& device, std::span<const std::uint32_t> spirv) {
    VkShaderModuleCreateInfo info{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
    info.codeSize = spirv.size_bytes();
    info.pCode = spirv.data();
    VkShaderModule module = VK_NULL_HANDLE;
    vkCreateShaderModule(device.device(), &info, nullptr, &module);
    return module;
}

void transitionImage(VkCommandBuffer commands, VkImage image, VkImageAspectFlags aspect,
                     VkImageLayout from, VkImageLayout to) {
    VkImageMemoryBarrier2 barrier{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2};
    // Conservative masks: correct for every transition, at the cost of some stalling. Fine for
    // the handful of transitions per frame the viewer does.
    barrier.srcStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
    barrier.srcAccessMask = VK_ACCESS_2_MEMORY_WRITE_BIT;
    barrier.dstStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
    barrier.dstAccessMask = VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT;
    barrier.oldLayout = from;
    barrier.newLayout = to;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.image = image;
    barrier.subresourceRange = {aspect, 0, 1, 0, 1};
    VkDependencyInfo dependency{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
    dependency.imageMemoryBarrierCount = 1;
    dependency.pImageMemoryBarriers = &barrier;
    vkCmdPipelineBarrier2(commands, &dependency);
}

std::vector<std::uint8_t> readbackRgba8(const Device& device, const Image& image,
                                        VkImageLayout layout) {
    const VkDeviceSize bytes = VkDeviceSize{image.extent.width} * image.extent.height * 4;
    Buffer staging = createBuffer(device, bytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT, true);
    device.submitAndWait([&](VkCommandBuffer commands) {
        transitionImage(commands, image.image, VK_IMAGE_ASPECT_COLOR_BIT, layout,
                        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
        VkBufferImageCopy region{};
        region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
        region.imageExtent = {image.extent.width, image.extent.height, 1};
        vkCmdCopyImageToBuffer(commands, image.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                               staging.buffer, 1, &region);
        transitionImage(commands, image.image, VK_IMAGE_ASPECT_COLOR_BIT,
                        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, layout);
    });
    vmaInvalidateAllocation(device.allocator(), staging.allocation, 0, VK_WHOLE_SIZE);
    std::vector<std::uint8_t> pixels(bytes);
    std::memcpy(pixels.data(), staging.mapped, bytes);
    destroy(device, staging);
    if (image.format == VK_FORMAT_B8G8R8A8_UNORM || image.format == VK_FORMAT_B8G8R8A8_SRGB) {
        for (std::size_t i = 0; i + 3 < pixels.size(); i += 4) {
            std::swap(pixels[i], pixels[i + 2]);
        }
    }
    return pixels;
}

namespace {

void appendChunk(std::vector<std::uint8_t>& out, const char* type,
                 std::span<const std::uint8_t> data) {
    const auto length = static_cast<std::uint32_t>(data.size());
    for (int shift = 24; shift >= 0; shift -= 8) {
        out.push_back(static_cast<std::uint8_t>(length >> shift));
    }
    const std::size_t start = out.size();
    out.insert(out.end(), type, type + 4);
    out.insert(out.end(), data.begin(), data.end());
    const auto crc = static_cast<std::uint32_t>(
        crc32(0, out.data() + start, static_cast<uInt>(out.size() - start)));
    for (int shift = 24; shift >= 0; shift -= 8) {
        out.push_back(static_cast<std::uint8_t>(crc >> shift));
    }
}

}  // namespace

bool writePng(const std::string& path, std::uint32_t width, std::uint32_t height,
              std::span<const std::uint8_t> rgba) {
    if (rgba.size() != std::size_t{width} * height * 4 || width == 0 || height == 0) {
        return false;
    }
    // Each scanline is prefixed with filter type 0 (none).
    std::vector<std::uint8_t> raw;
    raw.reserve((std::size_t{width} * 4 + 1) * height);
    for (std::uint32_t y = 0; y < height; ++y) {
        raw.push_back(0);
        const auto* row = rgba.data() + std::size_t{y} * width * 4;
        raw.insert(raw.end(), row, row + std::size_t{width} * 4);
    }
    uLongf compressedSize = compressBound(static_cast<uLong>(raw.size()));
    std::vector<std::uint8_t> compressed(compressedSize);
    if (compress2(compressed.data(), &compressedSize, raw.data(), static_cast<uLong>(raw.size()),
                  6) != Z_OK) {
        return false;
    }
    compressed.resize(compressedSize);

    std::vector<std::uint8_t> file = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
    std::array<std::uint8_t, 13> header{};
    for (int i = 0; i < 4; ++i) {
        header.at(static_cast<std::size_t>(i)) = static_cast<std::uint8_t>(width >> (24 - 8 * i));
        header.at(static_cast<std::size_t>(4 + i)) =
            static_cast<std::uint8_t>(height >> (24 - 8 * i));
    }
    header[8] = 8;  // bit depth
    header[9] = 6;  // colour type: RGBA
    appendChunk(file, "IHDR", header);
    appendChunk(file, "IDAT", compressed);
    appendChunk(file, "IEND", {});

    const std::unique_ptr<std::FILE, int (*)(std::FILE*)> out(std::fopen(path.c_str(), "wb"),
                                                              &std::fclose);
    return out != nullptr && std::fwrite(file.data(), 1, file.size(), out.get()) == file.size();
}

}  // namespace lagrange::gpu
