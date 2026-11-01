#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <vulkan/vulkan.h>

struct VmaAllocator_T;
using VmaAllocator = VmaAllocator_T*;

namespace lagrange::gpu {

/// Hardware capability tiers (RFC 0004). Every GPU feature defines its behaviour on each.
enum class CapabilityTier : std::uint8_t {
    Tier0 = 0,  ///< CPU only; a software Vulkan device may be used for raster.
    Tier1 = 1,  ///< Hardware Vulkan 1.3 device without hardware ray tracing.
    Tier2 = 2,  ///< Tier 1 plus ray query, acceleration structures and at least 6 GiB.
};

/// What one Vulkan physical device offers.
struct DeviceInfo {
    std::string name;    ///< "NVIDIA GeForce RTX 4060 Laptop GPU"
    std::string driver;  ///< "NVIDIA 595.91.07", "Mesa 26.0.8 (llvmpipe)"
    std::uint32_t vendorId{0};
    std::uint32_t deviceId{0};
    std::uint32_t apiMajor{0};
    std::uint32_t apiMinor{0};
    bool isSoftware{false};
    bool isDiscrete{false};
    std::uint64_t deviceMemoryBytes{0};  ///< Sum of device-local heaps.
    bool rayQuery{false};
    bool accelerationStructure{false};
    bool usable{false};  ///< Meets the minimum the HAL needs (Vulkan 1.3, dynamic rendering).
    CapabilityTier tier{CapabilityTier::Tier0};
    std::vector<std::string> reasons;  ///< Why not usable, or why not a higher tier.
};

/// Minimum device memory for Tier 2 (RFC 0004; provisional).
inline constexpr std::uint64_t kTier2MinimumMemoryBytes = 6ULL * 1024 * 1024 * 1024;

/// Classifies a device. Pure function of the facts in `info`; fills tier, usable and reasons.
void classify(DeviceInfo& info);

struct DeviceDesc {
    std::string applicationName{"lagrange"};
    /// Instance extensions the window system needs (from SDL). Empty for headless use.
    std::vector<std::string> instanceExtensions;
    /// Called once the instance exists. The returned surface is owned by the Device, and the
    /// chosen physical device is required to present to it.
    std::function<VkSurfaceKHR(VkInstance)> createSurface;
    bool validation{false};  ///< Enable VK_LAYER_KHRONOS_validation if installed.
    /// Device to use: an index into the enumeration, or a case-insensitive part of its name.
    /// Empty: the LAGRANGE_GPU environment variable, else discrete > integrated > software.
    std::string preferredDevice;
    /// Lower the effective tier. Empty: the LAGRANGE_TIER environment variable, else no limit.
    std::optional<CapabilityTier> maximumTier;
};

/// A Vulkan instance, one logical device with one graphics+compute queue, and an allocator.
/// All Vulkan use in the project goes through this class (invariant 8).
class Device {
public:
    /// Creates the device. On failure returns nullptr and puts a one-line reason in `error`.
    [[nodiscard]] static std::unique_ptr<Device> create(const DeviceDesc& desc, std::string& error);
    ~Device();
    Device(const Device&) = delete;
    Device& operator=(const Device&) = delete;

    [[nodiscard]] VkInstance instance() const { return instance_; }
    [[nodiscard]] VkPhysicalDevice physicalDevice() const { return physicalDevice_; }
    [[nodiscard]] VkDevice device() const { return device_; }
    [[nodiscard]] VkQueue queue() const { return queue_; }
    [[nodiscard]] std::uint32_t queueFamily() const { return queueFamily_; }
    [[nodiscard]] VmaAllocator allocator() const { return allocator_; }
    [[nodiscard]] VkSurfaceKHR surface() const { return surface_; }
    [[nodiscard]] VkCommandPool commandPool() const { return commandPool_; }

    [[nodiscard]] const DeviceInfo& info() const { return info_; }
    /// The tier in effect: the device's tier, lowered by DeviceDesc::maximumTier.
    [[nodiscard]] CapabilityTier tier() const { return effectiveTier_; }
    /// Every device the instance reported, classified. For diagnostics.
    [[nodiscard]] const std::vector<DeviceInfo>& allDevices() const { return allDevices_; }
    [[nodiscard]] bool validationEnabled() const { return validation_; }
    /// Whether pipelines may draw polygons as lines (VkPhysicalDeviceFeatures::fillModeNonSolid,
    /// enabled when the device has it).
    [[nodiscard]] bool nonSolidFill() const { return nonSolidFill_; }
    [[nodiscard]] bool samplerAnisotropy() const { return anisotropy_; }
    /// Number of validation errors reported so far (0 when validation is off).
    [[nodiscard]] std::uint32_t validationErrorCount() const;

    /// Records commands with `record`, submits them and waits for completion. For uploads,
    /// readbacks and tests; not for per-frame rendering.
    void submitAndWait(const std::function<void(VkCommandBuffer)>& record) const;

private:
    Device() = default;

    VkInstance instance_{VK_NULL_HANDLE};
    VkDebugUtilsMessengerEXT messenger_{VK_NULL_HANDLE};
    VkSurfaceKHR surface_{VK_NULL_HANDLE};
    VkPhysicalDevice physicalDevice_{VK_NULL_HANDLE};
    VkDevice device_{VK_NULL_HANDLE};
    VkQueue queue_{VK_NULL_HANDLE};
    std::uint32_t queueFamily_{0};
    VkCommandPool commandPool_{VK_NULL_HANDLE};
    VmaAllocator allocator_{nullptr};
    DeviceInfo info_;
    CapabilityTier effectiveTier_{CapabilityTier::Tier0};
    std::vector<DeviceInfo> allDevices_;
    bool validation_{false};
    bool nonSolidFill_{false};
    bool anisotropy_{false};
};

[[nodiscard]] const char* toString(CapabilityTier tier);

}  // namespace lagrange::gpu
