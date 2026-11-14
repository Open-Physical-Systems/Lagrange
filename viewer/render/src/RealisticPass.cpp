#include "RealisticPass.hpp"

#include <algorithm>
#include <cmath>

namespace lagrange::viewer {

RealisticPass::RealisticPass(const gpu::Device& device)
    : renderer_(device, std::make_shared<gpu::RealisticTextures>(device)) {}

std::uint32_t RealisticPass::record(VkCommandBuffer commands, const SceneFrame* frame,
                                    const Camera& camera, const RenderOptions& options,
                                    const gpu::MeshSet& meshes, const gpu::Image& color,
                                    const gpu::Image& depth) {
    draws_.clear();
    if (frame != nullptr && frame->statics) {
        const SceneStatics& statics = *frame->statics;
        if (staticsOf_ != frame->statics) {
            staticsOf_ = frame->statics;
            std::vector<float> patterns;
            patterns.reserve(statics.materials.size());
            for (const MaterialData& material : statics.materials) {
                patterns.push_back(gpu::surfacePatternOf(material.name));
            }
            looks_.clear();
            for (std::size_t i = 0; i < statics.prims.size(); ++i) {
                const PrimData& prim = statics.prims[i];
                if (!prim.visible || prim.mesh == kNone) {
                    continue;
                }
                gpu::RealisticDraw draw;
                draw.mesh = prim.mesh;
                if (prim.material != kNone) {
                    const MaterialData& material = statics.materials[prim.material];
                    draw.baseColor = material.baseColor;
                    draw.metallic = material.metallic;
                    draw.roughness = material.roughness;
                    draw.pattern = patterns[prim.material];
                    draw.emissive = material.emissive;
                }
                draw.ground = prim.name == "ground";
                draw.water = prim.mesh < statics.meshes.size() &&
                             std::find(prim.apiSchemas.begin(), prim.apiSchemas.end(),
                                       "WaterAPI_1") != prim.apiSchemas.end();
                if (draw.water) {
                    draw.box = statics.meshes[prim.mesh].aabb;
                }
                looks_.emplace_back(static_cast<std::uint32_t>(i), draw);
            }
        }
        draws_.reserve(looks_.size());
        for (const auto& [i, look] : looks_) {
            gpu::RealisticDraw& draw = draws_.emplace_back(look);
            const Transform& at = frame->transforms[i];
            draw.position = at.position;
            draw.rotation = at.rotation;
            draw.scale = at.scale;
        }
    }
    const VkExtent2D extent = color.extent;
    const double aspect = static_cast<double>(extent.width) / extent.height;
    const std::array<std::array<double, 3>, 3> axes = camera.axes();
    gpu::RealisticView view;
    view.viewProjection = camera.viewProjection(aspect);
    view.eye = camera.eye();
    view.right = axes[0];
    view.up = axes[1];
    view.forward = axes[2];
    view.focus = camera.target;
    view.focusDistance = camera.distance;
    view.tanHalfY = std::tan(camera.verticalFov / 2.0);
    view.tanHalfX = view.tanHalfY * aspect;
    view.near = camera.nearPlane;
    gpu::RealisticSettings settings;
    settings.season = static_cast<int>(effectiveSeason(options));
    settings.windSpeed = options.windSpeed;
    settings.rainMmPerH = options.rainMmPerH;
    settings.snowMmPerH = options.snowMmPerH;
    settings.wind = options.wind;
    settings.seconds = frame != nullptr ? static_cast<double>(frame->simTimeNs) * 1e-9 : 0.0;
    return renderer_.record(commands, draws_, view, settings, meshes, color, depth);
}

}  // namespace lagrange::viewer
