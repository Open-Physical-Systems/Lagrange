#include "SurfaceTextures.hpp"
#include "lagrange/gpu/Device.hpp"
#include "lagrange/gpu/Realistic.hpp"
#include "lagrange/gpu/WaterOptics.hpp"
#include "shaders/kRealCompositeFragment.hpp"
#include "shaders/kRealMeshFragment.hpp"
#include "shaders/kRealMeshVertex.hpp"
#include "shaders/kRealScreenVertex.hpp"
#include "shaders/kRealShadowFragment.hpp"
#include "shaders/kRealShadowVertex.hpp"
#include "shaders/kRealSkyFragment.hpp"
#include "shaders/kRealWaterFragment.hpp"
#include "shaders/kRealWaterVertex.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <numbers>
#include <string_view>
#include <utility>
#include <vector>

namespace lagrange::gpu {

namespace {

constexpr VkFormat kHdrFormat = VK_FORMAT_R16G16B16A16_SFLOAT;
constexpr VkFormat kFinalFormat = VK_FORMAT_R8G8B8A8_UNORM;
constexpr std::uint32_t kShadowSize = 2048;  // per cascade; the atlas is two side by side
constexpr VkDeviceSize kFrameStride = 512;   // at least any device's uniform buffer alignment
constexpr std::uint32_t kWaterGrid = 256;    // kGrid in realistic.slang

// Must match Frame and Push in realistic.slang.
struct Frame {
    std::array<float, 16> viewProjection;
    std::array<float, 4> sunDirection;
    std::array<float, 4> sunColor;
    std::array<float, 4> worldWrap;
    std::array<float, 4> cameraRight;
    std::array<float, 4> cameraUp;
    std::array<float, 4> cameraForward;
    std::array<float, 12> shadow0;
    std::array<float, 12> shadow1;
    std::array<float, 4> water;
    std::array<float, 4> waterRect;
    std::array<float, 4> screen;
    std::array<float, 4> mode;
    std::array<float, 4> weather;  // rain mm/h, snow mm/h, wind x, y m/s
    std::array<float, 4> light;    // daylight share, lamps, linear output, unused
    std::array<float, 4> surface;  // first cell's centre relative to the eye, cell, cells a side
    std::array<float, 4> surfaceLook;  // relief, the wind's ripple slope variance, its direction
    std::array<float, 4> sprayBox;     // first cell's centre relative to the eye, cell
    std::array<float, 4> sprayDims;    // cells, extinction per kg/m³
    std::array<float, 4> waterOptics;  // CDOM absorption at 620, 550, 465 nm (1/m), unused
};
static_assert(sizeof(Frame) == 432);

// Must match Lamp in realistic.slang.
struct GpuLamp {
    std::array<float, 4> position;  // relative to the eye; w: intensity · units per lux
    std::array<float, 4> axis;      // w: 1 for a point lamp
    std::array<float, 4> cone;      // half beam, half field (rad), near limit (m)
    std::array<float, 4> color;
};
static_assert(sizeof(GpuLamp) == 64);

struct Push {
    std::array<float, 12> model;
    std::array<float, 4> baseColor;
    std::array<float, 4> surface;
    std::array<float, 4> emissive;
};
static_assert(sizeof(Push) == 96);
static_assert(sizeof(Push) == sizeof(std::array<float, 24>));

// The sun: mid-afternoon, high enough for the whole scene and low enough for long shadows. A
// drawing choice, not a site parameter: the simulation has no time of day.
constexpr std::array<double, 3> kSun{0.52, 0.34, 0.78};
constexpr std::array<float, 3> kSunColor{1.0F, 0.95F, 0.87F};
constexpr float kSunStrength = 4.2F;
constexpr float kExposure = 0.72F;

// The light_sensor's clear-sky daylight (modules/sensors EnvironmentSensors.cpp, iesDaylight and
// airMass; sensors.md §9), for the calibration of RFC 0021 §4. A sensors test checks that these
// give its numbers.
constexpr double kExtraterrestrialLux = 127.5e3;  // lx, IES (1984)
constexpr double kEccentricity = 0.033;           // of the extraterrestrial illuminance
constexpr double kClearSkyExtinction = 0.21;      // per air mass, IES clear sky
constexpr int kEquinoxDay = 80;                   // the light_sensor's day without a date
constexpr double kDaysPerYear = 365.0;
// The darkest a picture is brightened for (realisticAdaptation): 2^10, a drawing choice, enough
// for a lamp's pool of a few hundred lux to read at night while the 0.8 mlx night sky stays black.
constexpr double kMaxAdaptation = 1024.0;
constexpr double kLampNear = 0.02;  // m: lamp light is not made stronger nearer than this

// The bare ground's colour: weathered concrete, not the flat design colour of the Lit view.
constexpr std::array<float, 4> kGroundColor{0.24F, 0.235F, 0.225F, 1.0F};
constexpr float kGroundReach = 40.0F;  // the ground runs on to the haze, as drawn only

/// Kasten and Young's (1989) relative air mass at zenith angle `zenith` (rad).
double airMass(double zenith) {
    constexpr double kDeg = std::numbers::pi / 180.0;
    const double z = std::min(zenith / kDeg, 90.0);
    return 1.0 / (std::cos(z * kDeg) + 0.50572 * std::pow(96.07995 - z, -1.6364));
}

/// Analytic CIE 1931 2° colour-matching functions: Wyman, Sloan and Shirley, "Simple Analytic
/// Approximations to the CIE XYZ Color Matching Functions", JCGT 2(2), 2013, the multi-lobe fit.
std::array<double, 3> cieXyz(double nm) {
    const auto g = [nm](double mu, double below, double above) {
        const double t = (nm - mu) / (nm < mu ? below : above);
        return std::exp(-0.5 * t * t);
    };
    return {
        1.056 * g(599.8, 37.9, 31.0) + 0.362 * g(442.0, 16.0, 26.7) - 0.065 * g(501.1, 20.4, 26.2),
        0.821 * g(568.8, 46.9, 40.5) + 0.286 * g(530.9, 16.3, 31.1),
        1.217 * g(437.0, 11.8, 36.0) + 0.681 * g(459.0, 26.0, 13.8)};
}

std::array<float, 12> rows(const RealisticDraw& t, const std::array<double, 3>& eye) {
    const float w = t.rotation[0];
    const float x = t.rotation[1];
    const float y = t.rotation[2];
    const float z = t.rotation[3];
    const std::array<float, 9> r = {
        1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y),
        2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
        2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y),
    };
    std::array<float, 12> m{};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            m.at(row * 4 + column) = r.at(row * 3 + column) * t.scale.at(column);
        }
        m.at(row * 4 + 3) = static_cast<float>(t.position.at(row) - eye.at(row));
    }
    return m;
}

// The sun's frame: `a` applied after `model` (3x4 each).
std::array<float, 12> times(const std::array<float, 12>& a, const std::array<float, 12>& model) {
    std::array<float, 12> m{};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t c = 0; c < 4; ++c) {
            float sum = 0.0F;
            for (std::size_t k = 0; k < 3; ++k) {
                sum += a.at(row * 4 + k) * model.at(k * 4 + c);
            }
            m.at(row * 4 + c) = sum + (c == 3 ? a.at(row * 4 + 3) : 0.0F);
        }
    }
    return m;
}

struct Cascade {
    std::array<float, 12> clip;   // relative to the eye -> clip x, y and reversed depth
    std::array<float, 12> atlas;  // relative to the eye -> atlas u, v and reversed depth
};

// An orthographic view along the sun of a ball round `center` (relative to the eye), moved in whole
// texels so that shadow edges do not crawl as the camera moves. `left`: which half of the atlas.
Cascade cascade(const std::array<double, 3>& center, const std::array<double, 3>& eye,
                double radius, bool left) {
    const std::array<double, 3>& s = kSun;
    const double norm = std::sqrt(s[0] * s[0] + s[1] * s[1] + s[2] * s[2]);
    const std::array<double, 3> z{s[0] / norm, s[1] / norm, s[2] / norm};
    std::array<double, 3> x{z[1], -z[0], 0.0};
    const double xn = std::sqrt(x[0] * x[0] + x[1] * x[1]);
    x = {x[0] / xn, x[1] / xn, 0.0};
    const std::array<double, 3> y{z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2],
                                  z[0] * x[1] - z[1] * x[0]};
    const auto dot = [](const std::array<double, 3>& a, const std::array<double, 3>& b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    };
    const double texel = 2.0 * radius / kShadowSize;
    // Snapped in world coordinates, kept in double.
    const std::array<double, 3> world{center[0] + eye[0], center[1] + eye[1], center[2] + eye[2]};
    const double cx = std::round(dot(world, x) / texel) * texel - dot(eye, x);
    const double cy = std::round(dot(world, y) / texel) * texel - dot(eye, y);
    const double reach = radius + 300.0;  // what stands between the sun and the ball casts
    const double cz = dot(center, z);
    Cascade out{};
    const auto f = [](double v) { return static_cast<float>(v); };
    out.clip = {f(x[0] / radius), f(x[1] / radius), f(x[2] / radius), f(-cx / radius),
                f(y[0] / radius), f(y[1] / radius), f(y[2] / radius), f(-cy / radius),
                // Reversed: 1 at the sun's end, 0 at the far end.
                f(z[0] / (2.0 * reach)), f(z[1] / (2.0 * reach)), f(z[2] / (2.0 * reach)),
                f(0.5 - cz / (2.0 * reach))};
    out.atlas = out.clip;
    for (std::size_t c = 0; c < 4; ++c) {
        out.atlas.at(c) = out.clip.at(c) * 0.25F;
        out.atlas.at(4 + c) = out.clip.at(4 + c) * 0.5F;
    }
    out.atlas[3] += left ? 0.25F : 0.75F;
    out.atlas[7] += 0.5F;
    return out;
}

}  // namespace

std::array<double, 3> realisticSunDirection() {
    const double n = std::sqrt(kSun[0] * kSun[0] + kSun[1] * kSun[1] + kSun[2] * kSun[2]);
    return {kSun[0] / n, kSun[1] / n, kSun[2] / n};
}

double realisticSunIlluminance() {
    const double elevation = std::asin(realisticSunDirection()[2]);
    const double extraterrestrial =
        kExtraterrestrialLux *
        (1.0 + kEccentricity * std::cos(2.0 * std::numbers::pi * kEquinoxDay / kDaysPerYear));
    return extraterrestrial *
           std::exp(-kClearSkyExtinction * airMass(std::numbers::pi / 2.0 - elevation));
}

double realisticDaylight() {
    const double up = realisticSunDirection()[2];
    // The IES (1984) clear sky's diffuse horizontal illuminance, (0.8 + 15.5 √sin α) klx.
    const double diffuse = (0.8 + 15.5 * std::sqrt(up)) * 1e3;
    return realisticSunIlluminance() * up + diffuse;
}

double realisticUnitsPerLux() {
    return kSunStrength / realisticSunIlluminance();
}

double realisticAdaptation(double skyIlluminance) {
    if (!std::isfinite(skyIlluminance)) {
        return 1.0;
    }
    const double share = std::max(skyIlluminance, 0.0) / realisticDaylight();
    return std::clamp(1.0 / std::max(share, 1.0 / kMaxAdaptation), 1.0, kMaxAdaptation);
}

std::array<float, 3> colorOfTemperature(double kelvin) {
    // Planck's spectral radiance (relative), summed over 360-830 nm in 1 nm steps.
    constexpr double kC2 = 1.438776877e-2;  // m·K, the second radiation constant (CODATA 2018)
    std::array<double, 3> xyz{};
    for (int nm = 360; nm <= 830; ++nm) {
        const double lambda = nm * 1e-9;
        const double planck = 1.0 / (std::pow(lambda, 5) * std::expm1(kC2 / (lambda * kelvin)));
        const std::array<double, 3> cmf = cieXyz(nm);
        for (std::size_t k = 0; k < 3; ++k) {
            xyz.at(k) += planck * cmf.at(k);
        }
    }
    if (!(xyz[1] > 0.0)) {
        return {1.0F, 1.0F, 1.0F};
    }
    const double x = xyz[0] / xyz[1];
    const double z = xyz[2] / xyz[1];
    // IEC 61966-2-1 (sRGB), XYZ (D65) to linear RGB.
    std::array<double, 3> rgb{3.2406 * x - 1.5372 - 0.4986 * z, -0.9689 * x + 1.8758 + 0.0415 * z,
                              0.0557 * x - 0.2040 + 1.0570 * z};
    for (double& c : rgb) {
        c = std::max(c, 0.0);
    }
    // Back to luminance 1 after the clip (Rec. 709 luminance weights, as sRGB's Y row).
    const double y = 0.2126 * rgb[0] + 0.7152 * rgb[1] + 0.0722 * rgb[2];
    return {static_cast<float>(rgb[0] / y), static_cast<float>(rgb[1] / y),
            static_cast<float>(rgb[2] / y)};
}

float surfacePatternOf(std::string_view name) {
    static constexpr std::array<std::pair<std::string_view, float>, 11> kWords{
        {{"brick", 1.0F},
         {"asphalt", 2.0F},
         {"pavement", 3.0F},
         {"plaster", 4.0F},
         {"concrete", 5.0F},
         {"lawn", 6.0F},
         {"leaves", 7.0F},
         {"road_paint", 8.0F},
         {"window_glass", 9.0F},
         {"bark", 10.0F},
         {"roof_felt", 11.0F}}};
    for (const auto& [word, value] : kWords) {
        if (name.find(word) != std::string_view::npos) {
            return value;
        }
    }
    return 0.0F;
}

RealisticTextures::RealisticTextures(const Device& device) : device_(device) {
    // Without the photographs a grey layer, and the shader's own patterns.
    SurfaceTextures textures = loadSurfaceTextures(surfaceTextureDirectory());
    complete_ = textures.complete;
    if (complete_) {
        albedo_ = createTextureArray(device_, SurfaceTextures::kSize, VK_FORMAT_R8G8B8A8_SRGB,
                                     textures.albedo);
        normal_ = createTextureArray(device_, SurfaceTextures::kSize, VK_FORMAT_R8G8B8A8_UNORM,
                                     textures.normal);
    } else {
        const std::vector<std::vector<std::uint8_t>> grey{{128, 128, 255, 255}};
        albedo_ = createTextureArray(device_, 1, VK_FORMAT_R8G8B8A8_SRGB, grey);
        normal_ = createTextureArray(device_, 1, VK_FORMAT_R8G8B8A8_UNORM, grey);
    }
}

RealisticTextures::~RealisticTextures() {
    destroy(device_, albedo_);
    destroy(device_, normal_);
}

RealisticRenderer::RealisticRenderer(const Device& device,
                                     std::shared_ptr<const RealisticTextures> textures)
    : device_(device), textures_(std::move(textures)) {
    VkDevice d = device_.device();
    VkSamplerCreateInfo sampler{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
    sampler.magFilter = VK_FILTER_LINEAR;
    sampler.minFilter = VK_FILTER_LINEAR;
    sampler.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler.maxLod = 0.0F;
    vkCreateSampler(d, &sampler, nullptr, &linear_);
    sampler.magFilter = VK_FILTER_NEAREST;
    sampler.minFilter = VK_FILTER_NEAREST;
    vkCreateSampler(d, &sampler, nullptr, &nearest_);
    sampler.magFilter = VK_FILTER_LINEAR;
    sampler.minFilter = VK_FILTER_LINEAR;
    sampler.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    sampler.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    sampler.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    sampler.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    sampler.maxLod = VK_LOD_CLAMP_NONE;
    sampler.anisotropyEnable = device_.samplerAnisotropy() ? VK_TRUE : VK_FALSE;
    sampler.maxAnisotropy = 8.0F;
    vkCreateSampler(d, &sampler, nullptr, &repeat_);

    std::array<VkDescriptorSetLayoutBinding, 11> bindings{};
    for (std::uint32_t b = 0; b < bindings.size(); ++b) {
        bindings.at(b).binding = b;
        bindings.at(b).descriptorCount = 1;
        bindings.at(b).descriptorType = b == 0 ? VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER
                                        : b >= 7 && b <= 9
                                            ? VK_DESCRIPTOR_TYPE_STORAGE_BUFFER
                                            : VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        bindings.at(b).stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    }
    VkDescriptorSetLayoutCreateInfo layout{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layout.bindingCount = bindings.size();
    layout.pBindings = bindings.data();
    vkCreateDescriptorSetLayout(d, &layout, nullptr, &setLayout_);

    const std::array<VkDescriptorPoolSize, 3> sizes{
        {{VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 3},
         {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 21},
         {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 9}}};
    VkDescriptorPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    poolInfo.maxSets = 3;
    poolInfo.poolSizeCount = sizes.size();
    poolInfo.pPoolSizes = sizes.data();
    vkCreateDescriptorPool(d, &poolInfo, nullptr, &pool_);
    const std::array<VkDescriptorSetLayout, 3> layouts{setLayout_, setLayout_, setLayout_};
    VkDescriptorSetAllocateInfo allocate{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
    allocate.descriptorPool = pool_;
    allocate.descriptorSetCount = 3;
    allocate.pSetLayouts = layouts.data();
    std::array<VkDescriptorSet, 3> sets{};
    vkAllocateDescriptorSets(d, &allocate, sets.data());
    mainSet_ = sets[0];
    mirrorSet_ = sets[1];
    compositeSet_ = sets[2];

    frames_ = createBuffer(device_, 2 * kFrameStride, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, true);
    lamps_ = createBuffer(device_, kMaxRealisticLamps * sizeof(GpuLamp),
                          VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, true);
    surface_ = createBuffer(device_,
                            static_cast<VkDeviceSize>(kMaxRealisticSurfaceCells) *
                                kMaxRealisticSurfaceCells * 8 * sizeof(float),
                            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, true);
    spray_ = createBuffer(device_, kMaxRealisticSprayCells * sizeof(float),
                          VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, true);
    shadow_ =
        createImage2D(device_, {2 * kShadowSize, kShadowSize}, kDepthFormat,
                      VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                      VK_IMAGE_ASPECT_DEPTH_BIT);
    pipelines();
}

RealisticRenderer::~RealisticRenderer() {
    VkDevice d = device_.device();
    vkDeviceWaitIdle(d);
    for (Pipeline* p : {&shadowPipe_, &opaque_, &depthOnly_, &opaqueShade_, &transparent_, &sky_,
                        &waterPipe_, &composite_}) {
        vkDestroyPipeline(d, p->pipeline, nullptr);
        vkDestroyPipelineLayout(d, p->layout, nullptr);
    }
    vkDestroyDescriptorPool(d, pool_, nullptr);
    vkDestroySampler(d, repeat_, nullptr);
    vkDestroyDescriptorSetLayout(d, setLayout_, nullptr);
    vkDestroySampler(d, linear_, nullptr);
    vkDestroySampler(d, nearest_, nullptr);
    destroy(device_, frames_);
    destroy(device_, lamps_);
    destroy(device_, surface_);
    destroy(device_, spray_);
    destroy(device_, shadow_);
    destroy(device_, hdr_);
    destroy(device_, hdrCopy_);
    destroy(device_, mirror_);
    destroy(device_, mirrorDepth_);
}

void RealisticRenderer::pipelines() {
    PipelineSpec mesh;
    mesh.vertex = kRealMeshVertex;
    mesh.fragment = kRealMeshFragment;
    mesh.pushSize = sizeof(Push);
    mesh.colorFormats = std::span{&kHdrFormat, 1};
    mesh.blend = false;
    mesh.setLayout = setLayout_;
    buildPipeline(device_, mesh, opaque_.layout, opaque_.pipeline);

    // Depth first, then shading where a mesh is the nearest (reversed depth, greater or equal):
    // the costly surface shader runs once per pixel instead of once per covering triangle. The
    // shading pass writes no depth, which keeps the early depth test on despite its discard.
    PipelineSpec depthOnly = mesh;
    depthOnly.fragment = kRealShadowFragment;  // empty
    depthOnly.colorFormats = {};
    buildPipeline(device_, depthOnly, depthOnly_.layout, depthOnly_.pipeline);
    PipelineSpec shade = mesh;
    shade.depthWrite = false;
    buildPipeline(device_, shade, opaqueShade_.layout, opaqueShade_.pipeline);

    PipelineSpec glass = mesh;
    glass.blend = true;
    glass.depthWrite = false;
    buildPipeline(device_, glass, transparent_.layout, transparent_.pipeline);

    PipelineSpec sun = mesh;
    sun.vertex = kRealShadowVertex;
    sun.fragment = kRealShadowFragment;
    sun.colorFormats = {};
    buildPipeline(device_, sun, shadowPipe_.layout, shadowPipe_.pipeline);

    PipelineSpec sky = mesh;
    sky.vertex = kRealScreenVertex;
    sky.fragment = kRealSkyFragment;
    sky.stride = 0;
    sky.depthWrite = false;
    buildPipeline(device_, sky, sky_.layout, sky_.pipeline);

    PipelineSpec water = sky;
    water.vertex = kRealWaterVertex;
    water.fragment = kRealWaterFragment;
    buildPipeline(device_, water, waterPipe_.layout, waterPipe_.pipeline);

    PipelineSpec composite = sky;
    composite.fragment = kRealCompositeFragment;
    composite.colorFormats = std::span{&kFinalFormat, 1};
    composite.depthTest = false;
    buildPipeline(device_, composite, composite_.layout, composite_.pipeline);
}

void RealisticRenderer::resize(VkExtent2D extent, const Image& depth) {
    vkDeviceWaitIdle(device_.device());
    destroy(device_, hdr_);
    destroy(device_, hdrCopy_);
    destroy(device_, mirror_);
    destroy(device_, mirrorDepth_);
    const VkImageUsageFlags attachment =
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    hdr_ = createImage2D(device_, extent, kHdrFormat, attachment | VK_IMAGE_USAGE_TRANSFER_SRC_BIT,
                         VK_IMAGE_ASPECT_COLOR_BIT);
    hdrCopy_ = createImage2D(device_, extent, kHdrFormat,
                             VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                             VK_IMAGE_ASPECT_COLOR_BIT);
    const VkExtent2D half{std::max<std::uint32_t>(extent.width / 2, 1),
                          std::max<std::uint32_t>(extent.height / 2, 1)};
    mirror_ = createImage2D(device_, half, kHdrFormat, attachment, VK_IMAGE_ASPECT_COLOR_BIT);
    mirrorDepth_ =
        createImage2D(device_, half, kDepthFormat,
                      VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                      VK_IMAGE_ASPECT_DEPTH_BIT);
    descriptors(depth);
}

void RealisticRenderer::descriptors(const Image& depth) {
    const auto write = [&](VkDescriptorSet set, VkDeviceSize offset, const Image& scene) {
        const VkDescriptorBufferInfo uniform{frames_.buffer, offset, sizeof(Frame)};
        const VkDescriptorBufferInfo lampData{lamps_.buffer, 0, VK_WHOLE_SIZE};
        const VkDescriptorBufferInfo surfaceData{surface_.buffer, 0, VK_WHOLE_SIZE};
        const VkDescriptorBufferInfo sprayData{spray_.buffer, 0, VK_WHOLE_SIZE};
        const VkImageLayout read = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        const VkImageLayout readDepth = VK_IMAGE_LAYOUT_DEPTH_READ_ONLY_OPTIMAL;
        const std::array<VkDescriptorImageInfo, 6> images{{
            {nearest_, shadow_.view, readDepth},
            {linear_, scene.view, read},
            {nearest_, depth.view, readDepth},
            {linear_, mirror_.view, read},
            {repeat_, textures_->albedo_.view, read},
            {repeat_, textures_->normal_.view, read},
        }};
        // How far what the water reflects stands from it (realistic.slang, mirrorDepth).
        const VkDescriptorImageInfo mirrorDepthInfo{nearest_, mirrorDepth_.view, readDepth};
        std::array<VkWriteDescriptorSet, 11> writes{};
        for (std::uint32_t b = 0; b < writes.size(); ++b) {
            VkWriteDescriptorSet& w = writes.at(b);
            w.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            w.dstSet = set;
            w.dstBinding = b;
            w.descriptorCount = 1;
            if (b == 0) {
                w.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                w.pBufferInfo = &uniform;
            } else if (b >= 7 && b <= 9) {
                w.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                w.pBufferInfo = b == 7 ? &lampData : b == 8 ? &surfaceData : &sprayData;
            } else if (b == 10) {
                w.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                w.pImageInfo = &mirrorDepthInfo;
            } else {
                w.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                w.pImageInfo = &images.at(b - 1);
            }
        }
        vkUpdateDescriptorSets(device_.device(), writes.size(), writes.data(), 0, nullptr);
    };
    write(mainSet_, 0, hdrCopy_);
    write(mirrorSet_, kFrameStride, hdrCopy_);
    write(compositeSet_, 0, hdr_);
}

std::uint32_t RealisticRenderer::record(VkCommandBuffer commands,
                                        std::span<const RealisticDraw> items,
                                        const RealisticView& view,
                                        const RealisticSettings& settings, const MeshSet& meshes,
                                        const Image& color, const Image& depth) {
    const VkExtent2D extent = color.extent;
    const std::array<double, 3>& eye = view.eye;
    const std::array<double, 3> toTarget{view.focus[0] - eye[0], view.focus[1] - eye[1],
                                         view.focus[2] - eye[2]};
    const double near = std::clamp(view.focusDistance * 1.8, 8.0, 60.0);
    const Cascade nearCascade = cascade(toTarget, eye, near, true);
    const Cascade farCascade = cascade(toTarget, eye, near * 7.0, false);
    const auto f = [](double v) { return static_cast<float>(v); };

    // The first body of water is drawn as water; any others as what they are made of.
    std::optional<Water> water;
    for (std::size_t i = 0; i < items.size() && !water; ++i) {
        if (!items[i].water) {
            continue;
        }
        const RealisticDraw& at = items[i];
        const std::array<float, 6>& box = at.box;
        Water found;
        found.draw = i;
        for (std::size_t k = 0; k < 2; ++k) {
            const double middle = (box.at(k) + box.at(k + 3)) / 2.0 * at.scale.at(k);
            found.center.at(k) = at.position.at(k) + middle - eye.at(k);
            found.half.at(k) = (box.at(k + 3) - box.at(k)) / 2.0 * at.scale.at(k);
        }
        found.top = at.position[2] + box[5] * at.scale[2] - eye[2];
        found.bottom = at.position[2] + box[2] * at.scale[2] - eye[2];
        water = found;
    }

    Frame main{};
    main.viewProjection = view.viewProjection;
    const double sunNorm = std::sqrt(kSun[0] * kSun[0] + kSun[1] * kSun[1] + kSun[2] * kSun[2]);
    main.sunDirection = {f(kSun[0] / sunNorm), f(kSun[1] / sunNorm), f(kSun[2] / sunNorm),
                         kSunStrength};
    main.sunColor = {kSunColor[0], kSunColor[1], kSunColor[2], kExposure};
    // Sun and exposure through the year, by eye: a drawing choice as kSun is.
    const float season = static_cast<float>(settings.season);
    switch (settings.season) {
        case 1:  // fall
            main.sunDirection[3] = 3.4F;
            main.sunColor = {1.0F, 0.84F, 0.64F, kExposure};
            break;
        case 2:  // winter
            main.sunDirection[3] = 1.1F;
            main.sunColor = {0.95F, 0.96F, 1.0F, kExposure * 0.85F};
            break;
        case 3:  // the rainy season
            main.sunDirection[3] = 0.25F;
            main.sunColor = {0.85F, 0.9F, 1.0F, kExposure * 1.05F};
            break;
        default:
            break;
    }
    // Night (RFC 0021 §6): the sun and the sky at the sky's share of the drawn day, and the
    // picture brightened as an eye or a camera adapts.
    const bool night = std::isfinite(settings.skyIlluminance);
    const double daylight =
        night ? std::max(settings.skyIlluminance, 0.0) / realisticDaylight() : 1.0;
    main.sunDirection[3] *= f(daylight);
    main.sunColor[3] *= f(realisticAdaptation(settings.skyIlluminance));
    // The lamps (RFC 0021 §4): the brightest at the eye, by I/d², up to the buffer's size.
    lampOrder_.clear();
    for (std::size_t i = 0; i < settings.lamps.size(); ++i) {
        const RealisticLamp& lamp = settings.lamps[i];
        if (!(lamp.intensity > 0.0)) {
            continue;
        }
        const std::array<double, 3> d{lamp.position[0] - eye[0], lamp.position[1] - eye[1],
                                      lamp.position[2] - eye[2]};
        const double d2 = std::max(d[0] * d[0] + d[1] * d[1] + d[2] * d[2], kLampNear * kLampNear);
        lampOrder_.emplace_back(-lamp.intensity / d2, i);
    }
    const std::size_t lampCount = std::min(lampOrder_.size(), kMaxRealisticLamps);
    std::partial_sort(lampOrder_.begin(),
                      lampOrder_.begin() + static_cast<std::ptrdiff_t>(lampCount),
                      lampOrder_.end());
    auto* lampData = static_cast<GpuLamp*>(lamps_.mapped);
    const double unitsPerLux = realisticUnitsPerLux();
    for (std::size_t k = 0; k < lampCount; ++k) {
        const RealisticLamp& lamp = settings.lamps[lampOrder_[k].second];
        GpuLamp& g = lampData[k];
        g.position = {f(lamp.position[0] - eye[0]), f(lamp.position[1] - eye[1]),
                      f(lamp.position[2] - eye[2]), f(lamp.intensity * unitsPerLux)};
        g.axis = {f(lamp.axis[0]), f(lamp.axis[1]), f(lamp.axis[2]), lamp.point ? 1.0F : 0.0F};
        g.cone = {f(lamp.beamAngle / 2.0), f(lamp.fieldAngle / 2.0), f(kLampNear), 0.0F};
        g.color = {lamp.color[0], lamp.color[1], lamp.color[2], 0.0F};
    }
    if (lampCount != 0) {
        flush(device_, lamps_);
    }
    main.light = {f(daylight), static_cast<float>(lampCount), settings.linear ? 1.0F : 0.0F, 0.0F};
    main.mode[2] = season;
    main.mode[3] = textures_->complete() ? 1.0F : 0.0F;
    main.weather = {settings.rainMmPerH, settings.snowMmPerH, settings.wind[0], settings.wind[1]};
    const double seconds = settings.seconds;
    // Wrapped so that a float holds it: noise and waves repeat every 1024 m and 17 minutes.
    main.worldWrap = {f(std::fmod(eye[0], 1024.0)), f(std::fmod(eye[1], 1024.0)),
                      f(std::fmod(eye[2], 1024.0)), f(std::fmod(seconds, 1024.0))};
    main.cameraRight = {f(view.right[0]), f(view.right[1]), f(view.right[2]), f(view.tanHalfX)};
    main.cameraUp = {f(view.up[0]), f(view.up[1]), f(view.up[2]), f(view.tanHalfY)};
    main.cameraForward = {f(view.forward[0]), f(view.forward[1]), f(view.forward[2]), f(view.near)};
    main.shadow0 = nearCascade.atlas;
    main.shadow1 = farCascade.atlas;
    if (water) {
        main.water = {f(water->top), settings.windSpeed, 1.0F, f(water->bottom)};
        main.waterRect = {f(water->center[0]), f(water->center[1]), f(water->half[0]),
                          f(water->half[1])};
    }
    // The ripples the wind raises below what any grid holds (water_optics): their slope
    // variance and the way it blows, everywhere on the water.
    const double windSpeed = std::max(static_cast<double>(settings.windSpeed), 0.0);
    const double windFriction = water_optics::frictionVelocityOf(windSpeed);
    const double windNorm = std::hypot(settings.wind[0], settings.wind[1]);
    const std::array<double, 2> windTo =
        windNorm > 1e-6
            ? std::array<double, 2>{settings.wind[0] / windNorm, settings.wind[1] / windNorm}
            : std::array<double, 2>{1.0, 0.0};
    main.surfaceLook = {1.0F, f(water_optics::rippleSlopeVariance(windFriction)), f(windTo[0]),
                        f(windTo[1])};
    {
        // CDOM's absorption a(λ) = a(440)·exp(−S·(λ − 440)), S = 0.014 /nm (Bricaud et al.).
        const auto cdom = [&](double nm) {
            return f(std::max(settings.waterCdom, 0.0F) * std::exp(-0.014 * (nm - 440.0)));
        };
        main.waterOptics = {cdom(620.0), cdom(550.0), cdom(465.0), 0.0F};
    }
    if (water) {
        const RealisticWaterSurface& moving = settings.waterSurface;
        const std::size_t n = moving.cells;
        if (n >= 2 && n <= kMaxRealisticSurfaceCells && moving.cell > 0.0 &&
            moving.heights.size() == n * n) {
            // Per cell: η, its slope by central differences (one-sided at the rim), the
            // whitecaps; then the ripples' slope variance the fetch has grown under the air's
            // friction there and the wind's (water_optics::fetchLimitedRipples), and the way
            // the stress runs.
            auto* cells = static_cast<std::array<float, 4>*>(surface_.mapped);
            const bool rubbed = moving.friction.size() == 2 * n * n;
            const std::array<double, 2> windU{windFriction * windTo[0], windFriction * windTo[1]};
            water_optics::fetchLimitedRipples(n, moving.cell, moving.friction, windU,
                                              rippleVariance_, rippleDirection_);
            const double onset = water_optics::kelvinHelmholtzOnset();
            const auto eta = [&](std::size_t x, std::size_t y) {
                const float v = moving.heights[y * n + x];
                return std::isfinite(v) ? v : 0.0F;
            };
            const auto cell = static_cast<float>(moving.cell);
            for (std::size_t y = 0; y < n; ++y) {
                for (std::size_t x = 0; x < n; ++x) {
                    const std::size_t i = y * n + x;
                    const std::size_t x0 = x == 0 ? 0 : x - 1;
                    const std::size_t x1 = x + 1 == n ? x : x + 1;
                    const std::size_t y0 = y == 0 ? 0 : y - 1;
                    const std::size_t y1 = y + 1 == n ? y : y + 1;
                    float foam =
                        moving.foam.size() == n * n ? std::clamp(moving.foam[i], 0.0F, 1.0F) : 0.0F;
                    double outwash = 0.0;
                    if (rubbed && std::isfinite(moving.friction[i]) &&
                        std::isfinite(moving.friction[n * n + i])) {
                        outwash = std::hypot(moving.friction[i], moving.friction[n * n + i]);
                    }
                    foam =
                        std::max(foam, f(water_optics::whitecapOf(outwash, windFriction, onset)));
                    cells[2 * i] = {
                        eta(x, y), (eta(x1, y) - eta(x0, y)) / (static_cast<float>(x1 - x0) * cell),
                        (eta(x, y1) - eta(x, y0)) / (static_cast<float>(y1 - y0) * cell), foam};
                    cells[2 * i + 1] = {f(rippleVariance_[i]), f(rippleDirection_[i]),
                                        f(rippleDirection_[n * n + i]), 0.0F};
                }
            }
            flush(device_, surface_);
            main.surface = {f(moving.first[0] - eye[0]), f(moving.first[1] - eye[1]), cell,
                            static_cast<float>(n)};
            main.surfaceLook[0] = moving.relief;
        }
    }
    {
        const RealisticSpray& spray = settings.spray;
        const std::size_t count =
            static_cast<std::size_t>(spray.cells[0]) * spray.cells[1] * spray.cells[2];
        if (spray.cells[0] >= 2 && spray.cells[1] >= 2 && spray.cells[2] >= 2 &&
            count <= kMaxRealisticSprayCells && spray.water.size() == count && spray.cell > 0.0 &&
            spray.extinctionPerKg > 0.0) {
            std::memcpy(spray_.mapped, spray.water.data(), count * sizeof(float));
            flush(device_, spray_);
            main.sprayBox = {f(spray.first[0] - eye[0]), f(spray.first[1] - eye[1]),
                             f(spray.first[2] - eye[2]), f(spray.cell)};
            main.sprayDims = {static_cast<float>(spray.cells[0]),
                              static_cast<float>(spray.cells[1]),
                              static_cast<float>(spray.cells[2]), f(spray.extinctionPerKg)};
        }
    }
    main.screen = {static_cast<float>(extent.width), static_cast<float>(extent.height),
                   1.0F / static_cast<float>(extent.width),
                   1.0F / static_cast<float>(extent.height)};
    Frame mirrorFrame = main;
    mirrorFrame.mode = {1.0F, water ? f(water->top) : 0.0F, season,
                        textures_->complete() ? 1.0F : 0.0F};
    mirrorFrame.sunDirection[2] = -main.sunDirection[2];
    auto* mapped = static_cast<std::byte*>(frames_.mapped);
    std::memcpy(mapped, &main, sizeof(Frame));
    std::memcpy(mapped + kFrameStride, &mirrorFrame, sizeof(Frame));
    flush(device_, frames_);

    std::uint32_t draws = 0;
    const VkShaderStageFlags stages = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

    const auto bind = [&](const Pipeline& pipe, VkDescriptorSet set) {
        vkCmdBindPipeline(commands, VK_PIPELINE_BIND_POINT_GRAPHICS, pipe.pipeline);
        vkCmdBindDescriptorSets(commands, VK_PIPELINE_BIND_POINT_GRAPHICS, pipe.layout, 0, 1, &set,
                                0, nullptr);
    };
    const auto region = [&](VkExtent2D size, VkOffset2D at = {0, 0}) {
        const VkViewport viewport{static_cast<float>(at.x),
                                  static_cast<float>(at.y),
                                  static_cast<float>(size.width),
                                  static_cast<float>(size.height),
                                  0.0F,
                                  1.0F};
        const VkRect2D scissor{at, size};
        vkCmdSetViewport(commands, 0, 1, &viewport);
        vkCmdSetScissor(commands, 0, 1, &scissor);
    };
    const auto begin = [&](const Image* target, VkImageView depthView, VkImageLayout depthLayout,
                           VkAttachmentLoadOp depthLoad, VkExtent2D size, VkAttachmentLoadOp load,
                           std::array<float, 4> clear = {}) {
        VkRenderingAttachmentInfo colorInfo{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
        if (target != nullptr) {
            colorInfo.imageView = target->view;
            colorInfo.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
            colorInfo.loadOp = load;
            colorInfo.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
            std::copy(clear.begin(), clear.end(), colorInfo.clearValue.color.float32);
        }
        VkRenderingAttachmentInfo depthInfo{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
        depthInfo.imageView = depthView;
        depthInfo.imageLayout = depthLayout;
        depthInfo.loadOp = depthLoad;
        depthInfo.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        depthInfo.clearValue.depthStencil = {0.0F, 0};
        VkRenderingInfo rendering{VK_STRUCTURE_TYPE_RENDERING_INFO};
        rendering.renderArea = {{0, 0}, size};
        rendering.layerCount = 1;
        rendering.colorAttachmentCount = target != nullptr ? 1 : 0;
        rendering.pColorAttachments = &colorInfo;
        rendering.pDepthAttachment = &depthInfo;
        vkCmdBeginRendering(commands, &rendering);
        region(size);
    };

    // What is drawn as a mesh, opaque or see-through, with its look.
    const auto pushFor = [&](std::size_t i, std::array<float, 12> model) {
        const RealisticDraw& item = items[i];
        Push push{};
        push.model = model;
        push.baseColor = item.baseColor;
        push.surface = {item.metallic, item.roughness, 0.0F, item.pattern};
        push.emissive = {item.emissive[0], item.emissive[1], item.emissive[2], 0.0F};
        if (item.ground) {
            push.baseColor = kGroundColor;
            for (std::size_t row = 0; row < 3; ++row) {
                push.model.at(row * 4) *= kGroundReach;
                push.model.at(row * 4 + 1) *= kGroundReach;
            }
        }
        // Grain where rough, none where polished, little on metal.
        const float rough = std::clamp((push.surface[1] - 0.45F) / 0.4F, 0.0F, 1.0F);
        push.surface[2] = push.surface[3] > 0.5F ? rough * 0.35F  // the pattern is the detail
                                                 : rough * (1.0F - 0.7F * push.surface[0]);
        return push;
    };
    const auto drawable = [&](std::size_t i) { return !(water && water->draw == i); };
    const auto opaque = [&](std::size_t i) { return items[i].baseColor[3] >= 1.0F; };
    const std::size_t primCount = items.size();

    // Bounding balls, relative to the eye, for leaving out what a pass cannot see (the meshes'
    // bounds come from their vertices, in MeshSet).
    balls_.resize(primCount);
    for (std::size_t i = 0; i < primCount; ++i) {
        const RealisticDraw& item = items[i];
        std::array<float, 4>& ball = balls_[i];
        ball = {0.0F, 0.0F, 0.0F, -1.0F};
        const std::array<float, 6>* box = meshes.bounds(item.mesh);
        if (box == nullptr || item.ground) {
            continue;  // the ground is drawn far past its mesh (kGroundReach)
        }
        const std::array<float, 12> m = rows(item, eye);
        const std::array<float, 6>& b = *box;
        float half = 0.0F;  // half the box's diagonal, scaled by the largest scale
        for (std::size_t k = 0; k < 3; ++k) {
            const float c = (b.at(k) + b.at(k + 3)) * 0.5F;
            ball[0] += m.at(k) * c;
            ball[1] += m.at(4 + k) * c;
            ball[2] += m.at(8 + k) * c;
            const float h = (b.at(k + 3) - b.at(k)) * 0.5F;
            half += h * h;
        }
        ball[0] += m[3];
        ball[1] += m[7];
        ball[2] += m[11];
        const float scale =
            std::max({std::abs(item.scale[0]), std::abs(item.scale[1]), std::abs(item.scale[2])});
        ball[3] = std::sqrt(half) * scale * 1.01F + 0.01F;  // a little margin for rounding
    }
    // The view's four sides and its near plane, as inward normals (unnormalised) through the eye.
    std::array<std::array<double, 3>, 4> sides{};
    for (std::size_t k = 0; k < 3; ++k) {
        sides[0].at(k) = view.forward.at(k) * view.tanHalfX - view.right.at(k);
        sides[1].at(k) = view.forward.at(k) * view.tanHalfX + view.right.at(k);
        sides[2].at(k) = view.forward.at(k) * view.tanHalfY - view.up.at(k);
        sides[3].at(k) = view.forward.at(k) * view.tanHalfY + view.up.at(k);
    }
    std::array<double, 4> sideNorms{};
    for (std::size_t s = 0; s < 4; ++s) {
        sideNorms.at(s) =
            std::sqrt(sides.at(s)[0] * sides.at(s)[0] + sides.at(s)[1] * sides.at(s)[1] +
                      sides.at(s)[2] * sides.at(s)[2]);
    }
    const auto inView = [&](std::size_t i) {
        const std::array<float, 4>& ball = balls_[i];
        if (ball[3] < 0.0F) {
            return true;
        }
        const double r = ball[3];
        const double ahead =
            ball[0] * view.forward[0] + ball[1] * view.forward[1] + ball[2] * view.forward[2];
        if (ahead < view.near - r) {
            return false;
        }
        for (std::size_t s = 0; s < 4; ++s) {
            const std::array<double, 3>& n = sides.at(s);
            if (ball[0] * n[0] + ball[1] * n[1] + ball[2] * n[2] < -r * sideNorms.at(s)) {
                return false;
            }
        }
        return true;
    };
    // A cascade's clip rows are orthonormal over its radius, so a ball keeps its shape there.
    const auto inCascade = [&](std::size_t i, const std::array<float, 12>& clip) {
        const std::array<float, 4>& ball = balls_[i];
        if (ball[3] < 0.0F) {
            return true;
        }
        const std::array<float, 3> at{
            clip[0] * ball[0] + clip[1] * ball[1] + clip[2] * ball[2] + clip[3],
            clip[4] * ball[0] + clip[5] * ball[1] + clip[6] * ball[2] + clip[7],
            clip[8] * ball[0] + clip[9] * ball[1] + clip[10] * ball[2] + clip[11]};
        const float across = ball[3] * std::hypot(clip[0], clip[1], clip[2]);
        const float along = ball[3] * std::hypot(clip[8], clip[9], clip[10]);
        return std::abs(at[0]) <= 1.0F + across && std::abs(at[1]) <= 1.0F + across &&
               at[2] >= -along && at[2] <= 1.0F + along;
    };

    // --- the sun's depth ---------------------------------------------------------------------
    transitionImage(commands, shadow_.image, VK_IMAGE_ASPECT_DEPTH_BIT, VK_IMAGE_LAYOUT_UNDEFINED,
                    VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL);
    begin(nullptr, shadow_.view, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
          VK_ATTACHMENT_LOAD_OP_CLEAR, shadow_.extent, VK_ATTACHMENT_LOAD_OP_DONT_CARE);
    bind(shadowPipe_, mainSet_);
    for (const bool left : {true, false}) {
        region({kShadowSize, kShadowSize}, {left ? 0 : static_cast<std::int32_t>(kShadowSize), 0});
        const std::array<float, 12>& sun = left ? nearCascade.clip : farCascade.clip;
        for (std::size_t i = 0; i < primCount; ++i) {
            if (!drawable(i) || !opaque(i) || items[i].ground || !inCascade(i, sun)) {
                continue;
            }
            Push push = pushFor(i, times(sun, rows(items[i], eye)));
            vkCmdPushConstants(commands, shadowPipe_.layout, stages, 0, sizeof(push), &push);
            meshes.draw(commands, items[i].mesh);
        }
    }
    vkCmdEndRendering(commands);
    transitionImage(commands, shadow_.image, VK_IMAGE_ASPECT_DEPTH_BIT,
                    VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
                    VK_IMAGE_LAYOUT_DEPTH_READ_ONLY_OPTIMAL);

    // --- what the water reflects: the scene turned over its surface ---------------------------
    transitionImage(commands, mirror_.image, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED,
                    VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
    transitionImage(commands, mirrorDepth_.image, VK_IMAGE_ASPECT_DEPTH_BIT,
                    VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL);
    begin(&mirror_, mirrorDepth_.view, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
          VK_ATTACHMENT_LOAD_OP_CLEAR, mirror_.extent, VK_ATTACHMENT_LOAD_OP_CLEAR);
    if (water) {
        bind(opaque_, mirrorSet_);
        const float plane = f(water->top);
        for (std::size_t i = 0; i < primCount; ++i) {
            if (!drawable(i) || !opaque(i) || items[i].ground) {
                continue;
            }
            std::array<float, 12> model = rows(items[i], eye);
            for (std::size_t c = 0; c < 4; ++c) {
                model.at(8 + c) = -model.at(8 + c);
            }
            model[11] += 2.0F * plane;
            Push push = pushFor(i, model);
            vkCmdPushConstants(commands, opaque_.layout, stages, 0, sizeof(push), &push);
            if (meshes.draw(commands, items[i].mesh)) {
                ++draws;
            }
        }
    }
    vkCmdEndRendering(commands);
    transitionImage(commands, mirror_.image, VK_IMAGE_ASPECT_COLOR_BIT,
                    VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    transitionImage(commands, mirrorDepth_.image, VK_IMAGE_ASPECT_DEPTH_BIT,
                    VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
                    VK_IMAGE_LAYOUT_DEPTH_READ_ONLY_OPTIMAL);

    // --- the scene ---------------------------------------------------------------------------
    // The opaque meshes' pushes, made once for both passes.
    pushes_.clear();
    for (std::size_t i = 0; i < primCount; ++i) {
        if (drawable(i) && opaque(i) && inView(i)) {
            const Push push = pushFor(i, rows(items[i], eye));
            auto& [item, bytes] = pushes_.emplace_back();
            item = i;
            std::memcpy(bytes.data(), &push, sizeof(push));
        }
    }
    begin(nullptr, depth.view, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
          VK_ATTACHMENT_LOAD_OP_CLEAR, extent, VK_ATTACHMENT_LOAD_OP_DONT_CARE);
    bind(depthOnly_, mainSet_);
    for (const auto& [i, push] : pushes_) {
        vkCmdPushConstants(commands, depthOnly_.layout, stages, 0, sizeof(push), push.data());
        meshes.draw(commands, items[i].mesh);
    }
    vkCmdEndRendering(commands);
    {
        // The depth just written is tested by the shading pass.
        VkMemoryBarrier2 barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER_2};
        barrier.srcStageMask = VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT;
        barrier.srcAccessMask = VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
        barrier.dstStageMask = VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT |
                               VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT;
        barrier.dstAccessMask = VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT |
                                VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
        VkDependencyInfo dependency{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
        dependency.memoryBarrierCount = 1;
        dependency.pMemoryBarriers = &barrier;
        vkCmdPipelineBarrier2(commands, &dependency);
    }
    transitionImage(commands, hdr_.image, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED,
                    VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
    begin(&hdr_, depth.view, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL, VK_ATTACHMENT_LOAD_OP_LOAD,
          extent, VK_ATTACHMENT_LOAD_OP_CLEAR);
    bind(opaqueShade_, mainSet_);
    for (const auto& [i, push] : pushes_) {
        vkCmdPushConstants(commands, opaqueShade_.layout, stages, 0, sizeof(push), push.data());
        if (meshes.draw(commands, items[i].mesh)) {
            ++draws;
        }
    }
    bind(sky_, mainSet_);
    vkCmdDraw(commands, 3, 1, 0, 0);
    vkCmdEndRendering(commands);
    transitionImage(commands, depth.image, VK_IMAGE_ASPECT_DEPTH_BIT,
                    VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
                    VK_IMAGE_LAYOUT_DEPTH_READ_ONLY_OPTIMAL);

    // --- water: reads what is below it, so from a copy ----------------------------------------
    const auto toRead = [&](VkImageLayout from) {
        transitionImage(commands, hdr_.image, VK_IMAGE_ASPECT_COLOR_BIT, from,
                        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
    };
    if (water) {
        toRead(VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
        transitionImage(commands, hdrCopy_.image, VK_IMAGE_ASPECT_COLOR_BIT,
                        VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
        VkImageCopy copy{};
        copy.srcSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
        copy.dstSubresource = copy.srcSubresource;
        copy.extent = {extent.width, extent.height, 1};
        vkCmdCopyImage(commands, hdr_.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, hdrCopy_.image,
                       VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);
        transitionImage(commands, hdrCopy_.image, VK_IMAGE_ASPECT_COLOR_BIT,
                        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        transitionImage(commands, hdr_.image, VK_IMAGE_ASPECT_COLOR_BIT,
                        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                        VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
    }
    begin(&hdr_, depth.view, VK_IMAGE_LAYOUT_DEPTH_READ_ONLY_OPTIMAL, VK_ATTACHMENT_LOAD_OP_LOAD,
          extent, VK_ATTACHMENT_LOAD_OP_LOAD);
    if (water) {
        bind(waterPipe_, mainSet_);
        vkCmdDraw(commands, (kWaterGrid + 2) * (kWaterGrid + 2) * 6, 1, 0, 0);
        ++draws;
    }
    // See-through things, farthest first.
    std::vector<std::pair<double, std::size_t>> glass;
    for (std::size_t i = 0; i < primCount; ++i) {
        if (drawable(i) && !opaque(i) && inView(i)) {
            const std::array<double, 3>& p = items[i].position;
            const std::array<double, 3> d{p[0] - eye[0], p[1] - eye[1], p[2] - eye[2]};
            glass.emplace_back(-(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]), i);
        }
    }
    std::sort(glass.begin(), glass.end());
    if (!glass.empty()) {
        bind(transparent_, mainSet_);
        for (const auto& [distance, i] : glass) {
            Push push = pushFor(i, rows(items[i], eye));
            vkCmdPushConstants(commands, transparent_.layout, stages, 0, sizeof(push), &push);
            if (meshes.draw(commands, items[i].mesh)) {
                ++draws;
            }
        }
    }
    vkCmdEndRendering(commands);
    transitionImage(commands, hdr_.image, VK_IMAGE_ASPECT_COLOR_BIT,
                    VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

    // --- the final image ---------------------------------------------------------------------
    begin(&color, depth.view, VK_IMAGE_LAYOUT_DEPTH_READ_ONLY_OPTIMAL, VK_ATTACHMENT_LOAD_OP_LOAD,
          extent, VK_ATTACHMENT_LOAD_OP_DONT_CARE);
    bind(composite_, compositeSet_);
    vkCmdDraw(commands, 3, 1, 0, 0);
    vkCmdEndRendering(commands);
    return draws;
}

}  // namespace lagrange::gpu
