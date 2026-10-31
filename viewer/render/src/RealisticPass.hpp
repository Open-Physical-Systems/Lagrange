#pragma once

#include "lagrange/gpu/Raster.hpp"
#include "lagrange/gpu/Resources.hpp"
#include "lagrange/viewer/Camera.hpp"
#include "lagrange/viewer/RenderOptions.hpp"
#include "lagrange/viewer/SceneStore.hpp"

#include <array>
#include <cstdint>
#include <optional>

#include <vulkan/vulkan.h>

namespace lagrange::gpu {
class Device;
}  // namespace lagrange::gpu

namespace lagrange::viewer {

/// Shading::Realistic: everything that is drawn before the overlays
/// (gpu/kernels/camera/realistic.slang). The sun's shadow map, the scene in HDR with sky, materials
/// and water, then the final image.
class RealisticPass {
public:
    explicit RealisticPass(const gpu::Device& device);
    ~RealisticPass();
    RealisticPass(const RealisticPass&) = delete;
    RealisticPass& operator=(const RealisticPass&) = delete;

    /// Makes its targets for this size; `depth` is the renderer's depth image, sampled here.
    void resize(VkExtent2D extent, const gpu::Image& depth);

    /// Records the passes outside any rendering scope. On entry `color` and `depth` are
    /// attachments; on return `color` holds the final image, still an attachment, and `depth` is
    /// DEPTH_READ_ONLY_OPTIMAL. Returns the number of draws.
    std::uint32_t record(VkCommandBuffer commands, const SceneFrame* frame, const Camera& camera,
                         const RenderOptions& options, gpu::MeshSet& meshes,
                         const gpu::Image& color, const gpu::Image& depth);

private:
    /// What a water prim occupies, relative to the eye.
    struct Water {
        std::size_t prim{0};
        std::array<double, 2> center{};
        std::array<double, 2> half{};
        double top{0.0};
        double bottom{0.0};
    };
    [[nodiscard]] std::optional<Water> findWater(const SceneFrame& frame,
                                                 const std::array<double, 3>& eye) const;
    void pipelines();
    void descriptors(const gpu::Image& depth);

    const gpu::Device& device_;
    gpu::Image shadow_;
    gpu::Image hdr_;
    gpu::Image hdrCopy_;
    gpu::Image mirror_;
    gpu::Image mirrorDepth_;
    gpu::Buffer frames_;  // host-visible: the main Frame, then the mirror pass's
    VkSampler linear_{VK_NULL_HANDLE};
    VkSampler nearest_{VK_NULL_HANDLE};
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
    bool first_{true};
};

}  // namespace lagrange::viewer
