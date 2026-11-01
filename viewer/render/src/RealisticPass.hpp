#pragma once

#include "lagrange/gpu/Realistic.hpp"
#include "lagrange/viewer/Camera.hpp"
#include "lagrange/viewer/RenderOptions.hpp"
#include "lagrange/viewer/SceneStore.hpp"

#include <cstdint>
#include <memory>
#include <vector>

namespace lagrange::viewer {

/// Shading::Realistic: what the viewport hands to the renderer it shares with the camera sensor.
class RealisticPass {
public:
    explicit RealisticPass(const gpu::Device& device);

    /// Makes its targets for this size; `depth` is the renderer's depth image, sampled here.
    void resize(VkExtent2D extent, const gpu::Image& depth) { renderer_.resize(extent, depth); }

    /// As gpu::RealisticRenderer::record, for what a frame of the scene shows.
    std::uint32_t record(VkCommandBuffer commands, const SceneFrame* frame, const Camera& camera,
                         const RenderOptions& options, const gpu::MeshSet& meshes,
                         const gpu::Image& color, const gpu::Image& depth);

private:
    gpu::RealisticRenderer renderer_;
    std::vector<gpu::RealisticDraw> draws_;
    decltype(SceneStatics::generation) patternsOf_{};
    std::vector<float> patterns_;  // per material of that generation
};

}  // namespace lagrange::viewer
