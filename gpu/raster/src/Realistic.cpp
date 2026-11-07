#include "SurfaceTextures.hpp"
#include "lagrange/gpu/Device.hpp"
#include "lagrange/gpu/Realistic.hpp"
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
};
static_assert(sizeof(Frame) == 336);

struct Push {
    std::array<float, 12> model;
    std::array<float, 4> baseColor;
    std::array<float, 4> surface;
    std::array<float, 4> emissive;
};
static_assert(sizeof(Push) == 96);

// The sun: mid-afternoon, high enough for the whole scene and low enough for long shadows. A
// drawing choice, not a site parameter: the simulation has no time of day.
constexpr std::array<double, 3> kSun{0.52, 0.34, 0.78};
constexpr std::array<float, 3> kSunColor{1.0F, 0.95F, 0.87F};
constexpr float kSunStrength = 4.2F;
constexpr float kExposure = 0.72F;

// The bare ground's colour: weathered concrete, not the flat design colour of the Lit view.
constexpr std::array<float, 4> kGroundColor{0.24F, 0.235F, 0.225F, 1.0F};
constexpr float kGroundReach = 40.0F;  // the ground runs on to the haze, as drawn only

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

    std::array<VkDescriptorSetLayoutBinding, 7> bindings{};
    for (std::uint32_t b = 0; b < bindings.size(); ++b) {
        bindings.at(b).binding = b;
        bindings.at(b).descriptorCount = 1;
        bindings.at(b).descriptorType =
            b == 0 ? VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER : VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        bindings.at(b).stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    }
    VkDescriptorSetLayoutCreateInfo layout{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layout.bindingCount = bindings.size();
    layout.pBindings = bindings.data();
    vkCreateDescriptorSetLayout(d, &layout, nullptr, &setLayout_);

    const std::array<VkDescriptorPoolSize, 2> sizes{
        {{VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 3}, {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 18}}};
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
    shadow_ =
        createImage2D(device_, {2 * kShadowSize, kShadowSize}, kDepthFormat,
                      VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                      VK_IMAGE_ASPECT_DEPTH_BIT);
    pipelines();
}

RealisticRenderer::~RealisticRenderer() {
    VkDevice d = device_.device();
    vkDeviceWaitIdle(d);
    for (Pipeline* p : {&shadowPipe_, &opaque_, &transparent_, &sky_, &waterPipe_, &composite_}) {
        vkDestroyPipeline(d, p->pipeline, nullptr);
        vkDestroyPipelineLayout(d, p->layout, nullptr);
    }
    vkDestroyDescriptorPool(d, pool_, nullptr);
    vkDestroySampler(d, repeat_, nullptr);
    vkDestroyDescriptorSetLayout(d, setLayout_, nullptr);
    vkDestroySampler(d, linear_, nullptr);
    vkDestroySampler(d, nearest_, nullptr);
    destroy(device_, frames_);
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
        createImage2D(device_, half, kDepthFormat, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
                      VK_IMAGE_ASPECT_DEPTH_BIT);
    descriptors(depth);
}

void RealisticRenderer::descriptors(const Image& depth) {
    const auto write = [&](VkDescriptorSet set, VkDeviceSize offset, const Image& scene) {
        const VkDescriptorBufferInfo uniform{frames_.buffer, offset, sizeof(Frame)};
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
        std::array<VkWriteDescriptorSet, 7> writes{};
        for (std::uint32_t b = 0; b < writes.size(); ++b) {
            VkWriteDescriptorSet& w = writes.at(b);
            w.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            w.dstSet = set;
            w.dstBinding = b;
            w.descriptorCount = 1;
            if (b == 0) {
                w.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                w.pBufferInfo = &uniform;
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
            if (!drawable(i) || !opaque(i) || items[i].ground) {
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

    // --- the scene ---------------------------------------------------------------------------
    transitionImage(commands, hdr_.image, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED,
                    VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
    begin(&hdr_, depth.view, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL, VK_ATTACHMENT_LOAD_OP_CLEAR,
          extent, VK_ATTACHMENT_LOAD_OP_CLEAR);
    bind(opaque_, mainSet_);
    for (std::size_t i = 0; i < primCount; ++i) {
        if (!drawable(i) || !opaque(i)) {
            continue;
        }
        Push push = pushFor(i, rows(items[i], eye));
        vkCmdPushConstants(commands, opaque_.layout, stages, 0, sizeof(push), &push);
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
        if (drawable(i) && !opaque(i)) {
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
