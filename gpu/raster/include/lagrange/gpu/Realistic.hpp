#pragma once

// The realistic renderer (gpu/kernels/camera/realistic.slang), one code for the viewer's viewport
// and the camera sensor (invariant 13): the sun's shadow map, the scene in HDR with sky, materials
// and water, then the final image.

#include "lagrange/gpu/Raster.hpp"
#include "lagrange/gpu/Resources.hpp"

#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string_view>
#include <utility>
#include <vector>

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

/// A lamp as the renderer draws it (RFC 0021 §4): what LampAPI_1 and lamp/<n>/intensity give.
struct RealisticLamp {
    std::array<double, 3> position{};              // world
    std::array<double, 3> axis{1.0, 0.0, 0.0};     // world, unit: where the beam points
    double intensity{0.0};                         // cd on the axis, now
    double beamAngle{0.0};                         // rad, full angle to 50 % (spot)
    double fieldAngle{0.0};                        // rad, full angle to 10 %, dark beyond (spot)
    bool point{false};                             // the same all round
    std::array<float, 3> color{1.0F, 1.0F, 1.0F};  // linear sRGB of luminance 1
};

/// Lamps drawn per frame at most (RFC 0021 open question 5): the brightest at the eye, by I/d².
inline constexpr std::size_t kMaxRealisticLamps = 64;

/// The renderer's units per lux (RFC 0021 §4): the drawn sun, kSunStrength, is the IES clear-sky
/// direct normal illuminance of the light_sensor (sensors.md §9) at the drawn sun's elevation on
/// the March equinox, so a Lambertian surface of albedo ρ under E lx is drawn ρ·E/π times this.
[[nodiscard]] double realisticUnitsPerLux();
/// The drawn sun's direct normal illuminance (lx), and the day's horizontal illuminance (lx,
/// direct on a level surface plus the clear sky's diffuse): what the drawn day stands for.
[[nodiscard]] double realisticSunIlluminance();
[[nodiscard]] double realisticDaylight();
/// The unit vector towards the drawn sun (world, z up).
[[nodiscard]] std::array<double, 3> realisticSunDirection();
/// How much the picture is brightened for a sky of `skyIlluminance` lx (horizontal): 1 by day,
/// up to kMaxAdaptation in the dark, as eyes and cameras adapt (a drawing choice). A camera
/// sensor divides its own exposure by it (CameraModel), so its electrons stay physical.
[[nodiscard]] double realisticAdaptation(double skyIlluminance);
/// Linear sRGB of luminance 1 of a black body at `kelvin`: Planck's law weighted by the CIE 1931
/// 2° colour-matching functions in the analytic fit of Wyman, Sloan and Shirley (JCGT 2(2), 2013),
/// then IEC 61966-2-1's XYZ to linear sRGB; negative components clipped.
[[nodiscard]] std::array<float, 3> colorOfTemperature(double kelvin);

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
    /// Lamps (RFC 0021 §4), any number: the kMaxRealisticLamps brightest at the eye are drawn.
    std::span<const RealisticLamp> lamps;
    /// The sky's horizontal illuminance (lx): the sun and the sky are drawn at its share of
    /// realisticDaylight(), and the picture adapts (realisticAdaptation). NaN: the drawn day.
    double skyIlluminance{std::numeric_limits<double>::quiet_NaN()};
    /// The final image is the scene's linear HDR value, clamped to 1, with no exposure, tone
    /// mapping, bloom or occlusion: for calibration tests.
    bool linear{false};
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
    Buffer lamps_;   // host-visible: up to kMaxRealisticLamps lamps (realistic.slang's Lamp)
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
    Pipeline depthOnly_;    // the opaque meshes' depth, before they are shaded
    Pipeline opaqueShade_;  // opaque_ on that depth: no writes, so each pixel is shaded once
    Pipeline transparent_;
    // Per frame: each opaque item and its push constants (Push in Realistic.cpp), kept to
    // reuse its capacity.
    std::vector<std::pair<std::size_t, std::array<float, 24>>> pushes_;
    // Per frame: each item's bounding ball relative to the eye (x, y, z, radius); radius < 0
    // for an item that is never culled.
    std::vector<std::array<float, 4>> balls_;
    // Per frame: the lamps by brightness at the eye (−I/d², index), kept to reuse its capacity.
    std::vector<std::pair<double, std::size_t>> lampOrder_;
    Pipeline sky_;
    Pipeline waterPipe_;
    Pipeline composite_;
};

}  // namespace lagrange::gpu
