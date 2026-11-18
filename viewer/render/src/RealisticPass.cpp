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
    lamps_.clear();
    for (const RenderOptions::Lamp& from : options.lamps) {
        gpu::RealisticLamp& lamp = lamps_.emplace_back();
        lamp.position = from.position;
        lamp.axis = from.axis;
        lamp.intensity = from.intensity;
        lamp.beamAngle = from.beamAngle;
        lamp.fieldAngle = from.fieldAngle;
        lamp.point = from.point;
        if (from.colorTemperature > 0.0) {
            lamp.color = gpu::colorOfTemperature(from.colorTemperature);
        } else {
            // lamp:color, normalised to luminance 1 (Rec. 709 weights, sRGB's Y row).
            const std::array<float, 3>& c = from.color;
            const float y = 0.2126F * c[0] + 0.7152F * c[1] + 0.0722F * c[2];
            lamp.color = y > 0.0F ? std::array<float, 3>{c[0] / y, c[1] / y, c[2] / y}
                                  : std::array<float, 3>{0.0F, 0.0F, 0.0F};
        }
    }
    settings.lamps = lamps_;
    settings.skyIlluminance = options.skyIlluminance;
    settings.waterCdom = options.waterCdom;
    if (const auto& moving = options.waterSurface) {
        settings.waterSurface.cells = moving->cells;
        settings.waterSurface.cell = moving->cell;
        settings.waterSurface.first = moving->first;
        settings.waterSurface.heights = moving->heights;
        settings.waterSurface.foam = moving->foam;
        settings.waterSurface.friction = moving->friction;
        settings.waterSurface.relief = options.waterRelief;
    }
    if (const auto& spray = options.spray3d) {
        // The drops' extinction per kg/m³ of water: Q_ext = 2 for drops much larger than the
        // wavelength (van de Hulst 1957, §8.42), 3/(ρ_w·D) per kg/m³, over the aerodynamics
        // domain's equal shares of 50, 150 and 450 µm drops (downwash.md §5b).
        constexpr double kWaterDensity = 1000.0;                            // kg/m³
        constexpr std::array<double, 3> kDiameters{50e-6, 150e-6, 450e-6};  // m
        double perKg = 0.0;
        for (const double d : kDiameters) {
            perKg += 3.0 / (kWaterDensity * d) / static_cast<double>(kDiameters.size());
        }
        settings.spray.cells = spray->cells;
        settings.spray.cell = spray->cell;
        settings.spray.first = spray->first;
        settings.spray.water = spray->water;
        settings.spray.extinctionPerKg = perKg;
    }
    return renderer_.record(commands, draws_, view, settings, meshes, color, depth);
}

}  // namespace lagrange::viewer
