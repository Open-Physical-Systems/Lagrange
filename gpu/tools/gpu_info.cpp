// Prints the Vulkan devices on this machine and the capability tier of each.
// Bug reports ask for this output.

#include <cstdio>
#include <memory>
#include <string>

#include "lagrange/gpu/Device.hpp"

int main() {
    std::string error;
    const std::unique_ptr<lagrange::gpu::Device> device =
        lagrange::gpu::Device::create(lagrange::gpu::DeviceDesc{}, error);
    if (!device) {
        std::printf("No usable Vulkan device: %s\nCapability: %s\n", error.c_str(),
                    lagrange::gpu::toString(lagrange::gpu::CapabilityTier::Tier0));
        return 1;
    }
    std::size_t index = 0;
    for (const lagrange::gpu::DeviceInfo& info : device->allDevices()) {
        const bool selected = info.name == device->info().name;
        std::printf("%c [%zu] %s\n", selected ? '*' : ' ', index++, info.name.c_str());
        std::printf("      driver   %s\n", info.driver.c_str());
        std::printf("      vulkan   %u.%u\n", info.apiMajor, info.apiMinor);
        std::printf("      memory   %llu MiB\n",
                    static_cast<unsigned long long>(info.deviceMemoryBytes / (1024 * 1024)));
        std::printf("      tier     %s%s\n", lagrange::gpu::toString(info.tier),
                    info.usable ? "" : "  (not usable)");
        for (const std::string& reason : info.reasons) {
            std::printf("      note     %s\n", reason.c_str());
        }
    }
    std::printf("\nSelected: %s\nCapability in effect: %s\n", device->info().name.c_str(),
                lagrange::gpu::toString(device->tier()));
    std::printf("Override with LAGRANGE_GPU=<index or part of the name> and LAGRANGE_TIER=<0|1|2>.\n");
    return 0;
}
