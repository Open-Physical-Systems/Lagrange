#pragma once

#include "lagrange/gpu/Resources.hpp"
#include "lagrange/viewer/Camera.hpp"
#include "lagrange/viewer/SceneStore.hpp"

#include <cstdint>
#include <memory>
#include <vector>

#include <vulkan/vulkan.h>

namespace lagrange::gpu {
class Device;
}  // namespace lagrange::gpu

namespace lagrange::viewer {

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
    void render(VkCommandBuffer commands, const SceneFrame* frame, const Camera& camera);

    [[nodiscard]] VkImageView colorView() const { return color_.view; }
    [[nodiscard]] const gpu::Image& color() const { return color_; }
    [[nodiscard]] VkExtent2D extent() const { return color_.extent; }
    [[nodiscard]] std::uint32_t lastDrawCount() const { return lastDrawCount_; }

private:
    struct GpuMesh {
        gpu::Buffer vertices;
        gpu::Buffer indices;
        std::uint32_t indexCount{0};
    };

    void createPipelines();
    void uploadMeshes(const SceneStatics& statics);
    void destroyMeshes();

    const gpu::Device& device_;
    gpu::Image color_;
    gpu::Image depth_;
    bool colorInitialised_{false};

    VkPipelineLayout meshLayout_{VK_NULL_HANDLE};
    VkPipeline meshPipeline_{VK_NULL_HANDLE};
    VkPipelineLayout lineLayout_{VK_NULL_HANDLE};
    VkPipeline linePipeline_{VK_NULL_HANDLE};

    std::vector<GpuMesh> meshes_;
    std::uint64_t meshGeneration_{0};  // SceneStatics::generation the meshes were built from
    gpu::Buffer gridMinor_;
    gpu::Buffer gridMajor_;
    gpu::Buffer axes_;
    std::uint32_t gridMinorCount_{0};
    std::uint32_t gridMajorCount_{0};
    std::uint32_t lastDrawCount_{0};
};

}  // namespace lagrange::viewer
