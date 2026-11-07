#pragma once

// The realistic renderer (gpu/kernels/camera/realistic.slang), one code for the viewer's viewport
// and the camera sensor (invariant 13): the sun's shadow map, the scene in HDR with sky, materials
// and water, then the final image.

#include "lagrange/gpu/Raster.hpp"
#include "lagrange/gpu/Resources.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string_view>

#include <vulkan/vulkan.h>

namespace lagrange::gpu {

class Device;

/// The surface pattern value (realistic.slang's surfacePattern) of a material named `name`, from a
/// word of the name; 0: nothing of its own.
[[nodiscard]] float surfacePatternOf(std::string_view name);

/// One thing to draw, at a world pose.
struct RealisticDraw {
    std::uint32_t mesh{0};
    std::array<double, 3> position{};
    std::array<float, 4> rotation{1.0F, 0.0F, 0.0F, 0.0F};  // w, x, y, z
    std::array<float, 3> scale{1.0F, 1.0F, 1.0F};
    std::array<float, 4> baseColor{0.8F, 0.8F, 0.8F, 1.0F};
    float metallic{0.0F};
    float roughness{0.6F};
    float pattern{0.0F};
    std::array<float, 3> emissive{0.0F, 0.0F, 0.0F};
    bool ground{false};          // the bare ground: drawn out to the haze, casting nothing
    bool water{false};           // a body of water: its box, drawn as a surface
    std::array<float, 6> box{};  // water only: the mesh's bounds, min then max
};

/// Where it is seen from. Screen y runs down; depth is reversed with no far plane.
struct RealisticView {
    std::array<float, 16> viewProjection{};  // relative to the eye
    std::array<double, 3> eye{};
    std::array<double, 3> right{}, up{}, forward{};  // unit, world
    std::array<double, 3> focus{};                   // world: what the shadows centre on
    double focusDistance{10.0};                      // m
    double tanHalfX{1.0}, tanHalfY{1.0};
    double near{0.05};  // m
};

/// What does not come from the scene.
struct RealisticSettings {
    int season{0};          // 0 summer, 1 fall, 2 winter, 3 rainy
    float windSpeed{0.0F};  // m/s: how rough water is drawn
    double seconds{0.0};    // sim time
    // What falls, drawn in the world: rain in mm/h, snow in mm/h of water, carried by the wind
    // (m/s, world x and y, where it blows to).
    float rainMmPerH{0.0F};
    float snowMmPerH{0.0F};
    std::array<float, 2> wind{};
};

/// The photographs of surfaces, shared by the renderers on a device.
class RealisticTextures {
public:
    explicit RealisticTextures(const Device& device);
    ~RealisticTextures();
    RealisticTextures(const RealisticTextures&) = delete;
    RealisticTextures& operator=(const RealisticTextures&) = delete;
    [[nodiscard]] bool complete() const { return complete_; }

private:
    friend class RealisticRenderer;
    const Device& device_;
    Image albedo_, normal_;
    bool complete_{false};
};

class RealisticRenderer {
public:
    RealisticRenderer(const Device& device, std::shared_ptr<const RealisticTextures> textures);
    ~RealisticRenderer();
    RealisticRenderer(const RealisticRenderer&) = delete;
    RealisticRenderer& operator=(const RealisticRenderer&) = delete;

    /// Makes its targets for this size; `depth` is the caller's depth image, sampled here.
    void resize(VkExtent2D extent, const Image& depth);

    /// Records the passes outside any rendering scope. On entry `color` and `depth` are
    /// attachments; on return `color` holds the final image, still an attachment, and `depth` is
    /// DEPTH_READ_ONLY_OPTIMAL. Returns the number of draws.
    std::uint32_t record(VkCommandBuffer commands, std::span<const RealisticDraw> draws,
                         const RealisticView& view, const RealisticSettings& settings,
                         const MeshSet& meshes, const Image& color, const Image& depth);

private:
    /// What a water prim occupies, relative to the eye.
    struct Water {
        std::size_t draw{0};
        std::array<double, 2> center{};
        std::array<double, 2> half{};
        double top{0.0};
        double bottom{0.0};
    };
    void pipelines();
    void descriptors(const Image& depth);

    const Device& device_;
    std::shared_ptr<const RealisticTextures> textures_;
    Image shadow_;
    Image hdr_;
    Image hdrCopy_;
    Image mirror_;
    Image mirrorDepth_;
    Buffer frames_;  // host-visible: the main Frame, then the mirror pass's
    VkSampler linear_{VK_NULL_HANDLE};
    VkSampler nearest_{VK_NULL_HANDLE};
    VkSampler repeat_{VK_NULL_HANDLE};
    VkDescriptorSetLayout setLayout_{VK_NULL_HANDLE};
    VkDescriptorPool pool_{VK_NULL_HANDLE};
    VkDescriptorSet mainSet_{VK_NULL_HANDLE};
    VkDescriptorSet mirrorSet_{VK_NULL_HANDLE};
    VkDescriptorSet compositeSet_{VK_NULL_HANDLE};

    struct Pipeline {
        VkPipelineLayout layout{VK_NULL_HANDLE};
        VkPipeline pipeline{VK_NULL_HANDLE};
    };
    Pipeline shadowPipe_;
    Pipeline opaque_;
    Pipeline transparent_;
    Pipeline sky_;
    Pipeline waterPipe_;
    Pipeline composite_;
};

}  // namespace lagrange::gpu
