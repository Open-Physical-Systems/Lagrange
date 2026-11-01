#include "lagrange/gpu/Device.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <vk_mem_alloc.h>

namespace lagrange::gpu {

namespace {

std::atomic<std::uint32_t> gValidationErrors{0};

VKAPI_ATTR VkBool32 VKAPI_CALL onDebugMessage(VkDebugUtilsMessageSeverityFlagBitsEXT severity,
                                              VkDebugUtilsMessageTypeFlagsEXT /*types*/,
                                              const VkDebugUtilsMessengerCallbackDataEXT* data,
                                              void* /*user*/) {
    if ((severity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT) != 0) {
        gValidationErrors.fetch_add(1, std::memory_order_relaxed);
    }
    std::fprintf(stderr, "[vulkan] %s\n", data->pMessage);
    return VK_FALSE;
}

std::string lower(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text;
}

std::string environment(const char* name) {
    // NOLINTNEXTLINE(concurrency-mt-unsafe): read during start-up, before threads exist.
    const char* value = std::getenv(name);
    return value != nullptr ? std::string(value) : std::string();
}

bool hasExtension(const std::vector<VkExtensionProperties>& available, const char* name) {
    return std::any_of(available.begin(), available.end(), [name](const auto& e) {
        return std::strcmp(static_cast<const char*>(e.extensionName), name) == 0;
    });
}

DeviceInfo describe(VkPhysicalDevice physical) {
    DeviceInfo info;
    VkPhysicalDeviceDriverProperties driver{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DRIVER_PROPERTIES};
    VkPhysicalDeviceProperties2 properties{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2};
    properties.pNext = &driver;
    vkGetPhysicalDeviceProperties2(physical, &properties);
    const VkPhysicalDeviceProperties& p = properties.properties;
    info.name = static_cast<const char*>(p.deviceName);
    info.driver = std::string(static_cast<const char*>(driver.driverName)) + " " +
                  static_cast<const char*>(driver.driverInfo);
    info.vendorId = p.vendorID;
    info.deviceId = p.deviceID;
    info.apiMajor = VK_API_VERSION_MAJOR(p.apiVersion);
    info.apiMinor = VK_API_VERSION_MINOR(p.apiVersion);
    info.isSoftware = p.deviceType == VK_PHYSICAL_DEVICE_TYPE_CPU;
    info.isDiscrete = p.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU;

    VkPhysicalDeviceMemoryProperties memory{};
    vkGetPhysicalDeviceMemoryProperties(physical, &memory);
    for (std::uint32_t i = 0; i < memory.memoryHeapCount; ++i) {
        if ((memory.memoryHeaps[i].flags & VK_MEMORY_HEAP_DEVICE_LOCAL_BIT) != 0) {
            info.deviceMemoryBytes += memory.memoryHeaps[i].size;
        }
    }

    std::uint32_t count = 0;
    vkEnumerateDeviceExtensionProperties(physical, nullptr, &count, nullptr);
    std::vector<VkExtensionProperties> extensions(count);
    vkEnumerateDeviceExtensionProperties(physical, nullptr, &count, extensions.data());
    info.rayQuery = hasExtension(extensions, VK_KHR_RAY_QUERY_EXTENSION_NAME);
    info.accelerationStructure =
        hasExtension(extensions, VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME);

    classify(info);
    if (info.usable) {
        VkPhysicalDeviceVulkan13Features features13{
            VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES};
        VkPhysicalDeviceVulkan11Features features11{
            VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_1_FEATURES};
        features13.pNext = &features11;
        VkPhysicalDeviceFeatures2 features{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2};
        features.pNext = &features13;
        vkGetPhysicalDeviceFeatures2(physical, &features);
        if (features13.dynamicRendering == VK_FALSE || features13.synchronization2 == VK_FALSE ||
            features11.shaderDrawParameters == VK_FALSE) {
            info.usable = false;
            info.reasons.insert(
                info.reasons.begin(),
                "dynamic rendering, synchronization2 or shaderDrawParameters not supported");
        }
    }
    return info;
}

}  // namespace

const char* toString(CapabilityTier tier) {
    switch (tier) {
        case CapabilityTier::Tier0:
            return "Tier 0 (CPU)";
        case CapabilityTier::Tier1:
            return "Tier 1 (raster GPU)";
        case CapabilityTier::Tier2:
            return "Tier 2 (full, hardware ray tracing)";
    }
    return "unknown";
}

void classify(DeviceInfo& info) {
    info.reasons.clear();
    info.usable = info.apiMajor > 1 || (info.apiMajor == 1 && info.apiMinor >= 3);
    if (!info.usable) {
        info.reasons.emplace_back("Vulkan 1.3 is required, the device offers " +
                                  std::to_string(info.apiMajor) + "." +
                                  std::to_string(info.apiMinor));
    }
    if (info.isSoftware) {
        info.tier = CapabilityTier::Tier0;
        info.reasons.emplace_back("software device");
        return;
    }
    info.tier = CapabilityTier::Tier2;
    if (!info.rayQuery) {
        info.reasons.emplace_back("no VK_KHR_ray_query");
        info.tier = CapabilityTier::Tier1;
    }
    if (!info.accelerationStructure) {
        info.reasons.emplace_back("no VK_KHR_acceleration_structure");
        info.tier = CapabilityTier::Tier1;
    }
    if (info.deviceMemoryBytes < kTier2MinimumMemoryBytes) {
        info.reasons.emplace_back(std::to_string(info.deviceMemoryBytes / (1024 * 1024)) +
                                  " MiB of device memory, Tier 2 needs " +
                                  std::to_string(kTier2MinimumMemoryBytes / (1024 * 1024)) +
                                  " MiB");
        info.tier = CapabilityTier::Tier1;
    }
}

std::uint32_t Device::validationErrorCount() const {
    return validation_ ? gValidationErrors.load(std::memory_order_relaxed) : 0;
}

std::unique_ptr<Device> Device::create(const DeviceDesc& desc, std::string& error) {
    std::unique_ptr<Device> self(new Device());  // private constructor: no make_unique
    error.clear();

    // --- instance ---------------------------------------------------------------------------
    std::uint32_t layerCount = 0;
    vkEnumerateInstanceLayerProperties(&layerCount, nullptr);
    std::vector<VkLayerProperties> layers(layerCount);
    vkEnumerateInstanceLayerProperties(&layerCount, layers.data());
    const char* validationLayer = "VK_LAYER_KHRONOS_validation";
    const bool layerInstalled =
        std::any_of(layers.begin(), layers.end(), [&](const VkLayerProperties& l) {
            return std::strcmp(static_cast<const char*>(l.layerName), validationLayer) == 0;
        });
    self->validation_ = desc.validation && layerInstalled;

    std::vector<const char*> extensions;
    extensions.reserve(desc.instanceExtensions.size() + 1);
    for (const std::string& name : desc.instanceExtensions) {
        extensions.push_back(name.c_str());
    }
    if (self->validation_) {
        extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
    }

    VkApplicationInfo application{VK_STRUCTURE_TYPE_APPLICATION_INFO};
    application.pApplicationName = desc.applicationName.c_str();
    application.pEngineName = "lagrange";
    application.apiVersion = VK_API_VERSION_1_3;

    VkInstanceCreateInfo instanceInfo{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
    instanceInfo.pApplicationInfo = &application;
    instanceInfo.enabledExtensionCount = static_cast<std::uint32_t>(extensions.size());
    instanceInfo.ppEnabledExtensionNames = extensions.data();
    if (self->validation_) {
        instanceInfo.enabledLayerCount = 1;
        instanceInfo.ppEnabledLayerNames = &validationLayer;
    }
    if (vkCreateInstance(&instanceInfo, nullptr, &self->instance_) != VK_SUCCESS) {
        error = "cannot create a Vulkan instance (is a Vulkan driver installed?)";
        return nullptr;
    }

    if (self->validation_) {
        VkDebugUtilsMessengerCreateInfoEXT messengerInfo{
            VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT};
        messengerInfo.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
                                        VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
        messengerInfo.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
                                    VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
                                    VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
        messengerInfo.pfnUserCallback = onDebugMessage;
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast): Vulkan's extension lookup.
        auto createMessenger = reinterpret_cast<PFN_vkCreateDebugUtilsMessengerEXT>(
            vkGetInstanceProcAddr(self->instance_, "vkCreateDebugUtilsMessengerEXT"));
        if (createMessenger != nullptr) {
            createMessenger(self->instance_, &messengerInfo, nullptr, &self->messenger_);
        }
    }

    if (desc.createSurface) {
        self->surface_ = desc.createSurface(self->instance_);
        if (self->surface_ == VK_NULL_HANDLE) {
            error = "cannot create a window surface";
            return nullptr;
        }
    }

    // --- physical device --------------------------------------------------------------------
    std::uint32_t deviceCount = 0;
    vkEnumeratePhysicalDevices(self->instance_, &deviceCount, nullptr);
    std::vector<VkPhysicalDevice> physicals(deviceCount);
    vkEnumeratePhysicalDevices(self->instance_, &deviceCount, physicals.data());
    if (physicals.empty()) {
        error = "no Vulkan device found";
        return nullptr;
    }

    struct Candidate {
        VkPhysicalDevice physical;
        std::uint32_t queueFamily;
        bool presentable;
    };
    std::vector<Candidate> candidates;
    for (VkPhysicalDevice physical : physicals) {
        self->allDevices_.push_back(describe(physical));
        std::uint32_t familyCount = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(physical, &familyCount, nullptr);
        std::vector<VkQueueFamilyProperties> families(familyCount);
        vkGetPhysicalDeviceQueueFamilyProperties(physical, &familyCount, families.data());
        Candidate candidate{physical, 0, false};
        for (std::uint32_t f = 0; f < familyCount; ++f) {
            const VkQueueFlags wanted = VK_QUEUE_GRAPHICS_BIT | VK_QUEUE_COMPUTE_BIT;
            if ((families[f].queueFlags & wanted) != wanted) {
                continue;
            }
            VkBool32 present = VK_TRUE;
            if (self->surface_ != VK_NULL_HANDLE) {
                vkGetPhysicalDeviceSurfaceSupportKHR(physical, f, self->surface_, &present);
            }
            if (present == VK_TRUE) {
                candidate.queueFamily = f;
                candidate.presentable = true;
                break;
            }
        }
        if (!candidate.presentable) {
            DeviceInfo& info = self->allDevices_.back();
            info.usable = false;
            info.reasons.insert(info.reasons.begin(), self->surface_ != VK_NULL_HANDLE
                                                          ? "cannot present to the window"
                                                          : "no graphics+compute queue");
        }
        candidates.push_back(candidate);
    }

    const std::string wanted =
        lower(desc.preferredDevice.empty() ? environment("LAGRANGE_GPU") : desc.preferredDevice);
    int chosen = -1;
    if (!wanted.empty()) {
        const bool isIndex = std::all_of(wanted.begin(), wanted.end(),
                                         [](unsigned char c) { return std::isdigit(c) != 0; });
        for (std::size_t i = 0; i < candidates.size(); ++i) {
            const bool match =
                isIndex ? std::to_string(i) == wanted
                        : lower(self->allDevices_[i].name).find(wanted) != std::string::npos;
            if (match) {
                chosen = static_cast<int>(i);
                break;
            }
        }
        if (chosen < 0) {
            error = "no Vulkan device matches '" + wanted + "'";
            return nullptr;
        }
        if (!self->allDevices_[static_cast<std::size_t>(chosen)].usable) {
            error = "device '" + self->allDevices_[static_cast<std::size_t>(chosen)].name +
                    "' cannot be used: " +
                    self->allDevices_[static_cast<std::size_t>(chosen)].reasons.front();
            return nullptr;
        }
    } else {
        // Discrete before integrated before software; never software when hardware exists.
        const auto rank = [&](std::size_t i) {
            const DeviceInfo& d = self->allDevices_[i];
            return !d.usable ? -1 : d.isDiscrete ? 3 : d.isSoftware ? 1 : 2;
        };
        for (std::size_t i = 0; i < candidates.size(); ++i) {
            if (rank(i) > (chosen < 0 ? 0 : rank(static_cast<std::size_t>(chosen)))) {
                chosen = static_cast<int>(i);
            }
        }
        if (chosen < 0) {
            error = "no usable Vulkan device: " + self->allDevices_.front().name + ": " +
                    (self->allDevices_.front().reasons.empty()
                         ? "unknown reason"
                         : self->allDevices_.front().reasons.front());
            return nullptr;
        }
    }
    const Candidate& pick = candidates[static_cast<std::size_t>(chosen)];
    self->physicalDevice_ = pick.physical;
    self->queueFamily_ = pick.queueFamily;
    self->info_ = self->allDevices_[static_cast<std::size_t>(chosen)];

    // --- effective tier ---------------------------------------------------------------------
    std::optional<CapabilityTier> limit = desc.maximumTier;
    if (!limit) {
        const std::string text = environment("LAGRANGE_TIER");
        if (text == "0" || text == "1" || text == "2") {
            limit = static_cast<CapabilityTier>(text[0] - '0');
        } else if (!text.empty()) {
            error = "LAGRANGE_TIER must be 0, 1 or 2";
            return nullptr;
        }
    }
    self->effectiveTier_ = self->info_.tier;
    if (limit) {
        if (*limit > self->info_.tier) {
            error = std::string("requested ") + toString(*limit) + ", but '" + self->info_.name +
                    "' is " + toString(self->info_.tier);
            return nullptr;
        }
        self->effectiveTier_ = *limit;
    }

    // --- logical device ---------------------------------------------------------------------
    const float priority = 1.0F;
    VkDeviceQueueCreateInfo queueInfo{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
    queueInfo.queueFamilyIndex = self->queueFamily_;
    queueInfo.queueCount = 1;
    queueInfo.pQueuePriorities = &priority;

    VkPhysicalDeviceVulkan13Features features13{
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES};
    features13.dynamicRendering = VK_TRUE;
    features13.synchronization2 = VK_TRUE;
    // Slang maps SV_VertexID to the SPIR-V DrawParameters capability.
    VkPhysicalDeviceVulkan11Features features11{
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_1_FEATURES};
    features11.shaderDrawParameters = VK_TRUE;
    features13.pNext = &features11;

    std::vector<const char*> deviceExtensions;
    if (self->surface_ != VK_NULL_HANDLE) {
        deviceExtensions.push_back(VK_KHR_SWAPCHAIN_EXTENSION_NAME);
    }
    VkPhysicalDeviceFeatures supported{};
    vkGetPhysicalDeviceFeatures(self->physicalDevice_, &supported);
    VkPhysicalDeviceFeatures core{};
    core.fillModeNonSolid = supported.fillModeNonSolid;  // the viewer's wireframe shading
    self->nonSolidFill_ = supported.fillModeNonSolid == VK_TRUE;
    core.samplerAnisotropy = supported.samplerAnisotropy;  // textures seen at a grazing angle
    self->anisotropy_ = supported.samplerAnisotropy == VK_TRUE;
    VkDeviceCreateInfo deviceInfo{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
    deviceInfo.pNext = &features13;
    deviceInfo.pEnabledFeatures = &core;
    deviceInfo.queueCreateInfoCount = 1;
    deviceInfo.pQueueCreateInfos = &queueInfo;
    deviceInfo.enabledExtensionCount = static_cast<std::uint32_t>(deviceExtensions.size());
    deviceInfo.ppEnabledExtensionNames = deviceExtensions.data();
    if (vkCreateDevice(self->physicalDevice_, &deviceInfo, nullptr, &self->device_) != VK_SUCCESS) {
        error = "cannot create a logical device on '" + self->info_.name + "'";
        return nullptr;
    }
    vkGetDeviceQueue(self->device_, self->queueFamily_, 0, &self->queue_);

    VkCommandPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    poolInfo.queueFamilyIndex = self->queueFamily_;
    if (vkCreateCommandPool(self->device_, &poolInfo, nullptr, &self->commandPool_) != VK_SUCCESS) {
        error = "cannot create a command pool";
        return nullptr;
    }

    VmaAllocatorCreateInfo allocatorInfo{};
    allocatorInfo.instance = self->instance_;
    allocatorInfo.physicalDevice = self->physicalDevice_;
    allocatorInfo.device = self->device_;
    allocatorInfo.vulkanApiVersion = VK_API_VERSION_1_3;
    if (vmaCreateAllocator(&allocatorInfo, &self->allocator_) != VK_SUCCESS) {
        error = "cannot create the memory allocator";
        return nullptr;
    }
    return self;
}

Device::~Device() {
    if (device_ != VK_NULL_HANDLE) {
        vkDeviceWaitIdle(device_);
    }
    if (allocator_ != nullptr) {
        vmaDestroyAllocator(allocator_);
    }
    if (commandPool_ != VK_NULL_HANDLE) {
        vkDestroyCommandPool(device_, commandPool_, nullptr);
    }
    if (device_ != VK_NULL_HANDLE) {
        vkDestroyDevice(device_, nullptr);
    }
    if (surface_ != VK_NULL_HANDLE) {
        vkDestroySurfaceKHR(instance_, surface_, nullptr);
    }
    if (messenger_ != VK_NULL_HANDLE) {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast): Vulkan's extension lookup.
        auto destroyMessenger = reinterpret_cast<PFN_vkDestroyDebugUtilsMessengerEXT>(
            vkGetInstanceProcAddr(instance_, "vkDestroyDebugUtilsMessengerEXT"));
        if (destroyMessenger != nullptr) {
            destroyMessenger(instance_, messenger_, nullptr);
        }
    }
    if (instance_ != VK_NULL_HANDLE) {
        vkDestroyInstance(instance_, nullptr);
    }
}

void Device::submitAndWait(const std::function<void(VkCommandBuffer)>& record) const {
    VkCommandBufferAllocateInfo allocateInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    allocateInfo.commandPool = commandPool_;
    allocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocateInfo.commandBufferCount = 1;
    VkCommandBuffer commands = VK_NULL_HANDLE;
    vkAllocateCommandBuffers(device_, &allocateInfo, &commands);

    VkCommandBufferBeginInfo beginInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(commands, &beginInfo);
    record(commands);
    vkEndCommandBuffer(commands);

    VkCommandBufferSubmitInfo commandInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO};
    commandInfo.commandBuffer = commands;
    VkSubmitInfo2 submit{VK_STRUCTURE_TYPE_SUBMIT_INFO_2};
    submit.commandBufferInfoCount = 1;
    submit.pCommandBufferInfos = &commandInfo;

    VkFenceCreateInfo fenceInfo{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
    VkFence fence = VK_NULL_HANDLE;
    vkCreateFence(device_, &fenceInfo, nullptr, &fence);
    vkQueueSubmit2(queue_, 1, &submit, fence);
    vkWaitForFences(device_, 1, &fence, VK_TRUE, UINT64_MAX);
    vkDestroyFence(device_, fence, nullptr);
    vkFreeCommandBuffers(device_, commandPool_, 1, &commands);
}

}  // namespace lagrange::gpu
