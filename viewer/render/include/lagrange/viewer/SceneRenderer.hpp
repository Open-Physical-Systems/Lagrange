#pragma once

#include "lagrange/gpu/Raster.hpp"
#include "lagrange/gpu/Resources.hpp"
#include "lagrange/viewer/Camera.hpp"
#include "lagrange/viewer/RenderOptions.hpp"
#include "lagrange/viewer/SceneStore.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

#include <vulkan/vulkan.h>

namespace lagrange::gpu {
class Device;
}  // namespace lagrange::gpu

namespace lagrange::viewer {

class RealisticPass;

/// Lines that change every frame (sensor overlays): pairs of points relative to `origin`,
/// drawn unlit in `color` (display values, like the design tokens).
struct OverlayLines {
    std::array<double, 3> origin{0.0, 0.0, 0.0};  ///< World position, kept in double.
    std::array<float, 4> color{1.0F, 1.0F, 1.0F, 1.0F};
    std::vector<float> vertices;  ///< x, y, z per point; two points per segment.
};

/// Draws a SceneFrame into an offscreen colour image that the UI shows as a texture.
class SceneRenderer {
public:
    explicit SceneRenderer(const gpu::Device& device);
    ~SceneRenderer();
    SceneRenderer(const SceneRenderer&) = delete;
    SceneRenderer& operator=(const SceneRenderer&) = delete;

    /// Makes the target `extent` pixels. Returns true if the target was recreated, in which
    /// case any texture handle made from colorView() is stale. Waits for the GPU when it
    /// recreates.
    bool resize(VkExtent2D extent);

    /// Records the scene pass. `frame` may be null (draws the grid only). Leaves the colour
    /// image in SHADER_READ_ONLY_OPTIMAL.
    /// `overlays` are drawn after the meshes, depth-tested. The buffer that holds them is
    /// rewritten every call, so the previous frame must have finished on the GPU.
    void render(VkCommandBuffer commands, const SceneFrame* frame, const Camera& camera,
                const RenderOptions& options = {}, std::span<const OverlayLines> overlays = {});

    [[nodiscard]] VkImageView colorView() const { return color_.view; }
    [[nodiscard]] const gpu::Image& color() const { return color_; }
    [[nodiscard]] VkExtent2D extent() const { return color_.extent; }
    [[nodiscard]] std::uint32_t lastDrawCount() const { return lastDrawCount_; }
    /// Whether Shading::Wireframe draws edges (the device can fill polygons as lines); without
    /// it wireframe shading draws unlit.
    [[nodiscard]] bool wireframeSupported() const { return wireframePipeline_ != VK_NULL_HANDLE; }

private:
    void createPipelines();
    std::unique_ptr<RealisticPass> realistic_;  // made when first used
    void uploadMeshes(const SceneStatics& statics);

    const gpu::Device& device_;
    gpu::Image color_;
    gpu::Image depth_;
    bool colorInitialised_{false};

    VkPipelineLayout meshLayout_{VK_NULL_HANDLE};
    VkPipeline meshPipeline_{VK_NULL_HANDLE};
    VkPipelineLayout unlitLayout_{VK_NULL_HANDLE};
    VkPipeline unlitPipeline_{VK_NULL_HANDLE};
    VkPipelineLayout wireframeLayout_{VK_NULL_HANDLE};
    VkPipeline wireframePipeline_{VK_NULL_HANDLE};
    VkPipelineLayout lineLayout_{VK_NULL_HANDLE};
    VkPipeline linePipeline_{VK_NULL_HANDLE};

    gpu::MeshSet meshes_;
    std::uint64_t meshGeneration_{0};  // SceneStatics::generation the meshes were built from
    gpu::Buffer gridMinor_;
    gpu::Buffer gridMajor_;
    gpu::Buffer axes_;
    gpu::Buffer overlay_;  // host-visible, grown as needed
    std::uint32_t gridMinorCount_{0};
    std::uint32_t gridMajorCount_{0};
    std::uint32_t lastDrawCount_{0};
};

}  // namespace lagrange::viewer
