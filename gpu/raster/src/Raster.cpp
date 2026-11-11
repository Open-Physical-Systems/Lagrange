#include "lagrange/gpu/Compute.hpp"
#include "lagrange/gpu/Device.hpp"
#include "lagrange/gpu/Raster.hpp"
#include "lagrange/gpu/Realistic.hpp"
#include "shaders/kCameraModel.hpp"
#include "shaders/kMeshVertex.hpp"
#include "shaders/kSensorColor.hpp"
#include "shaders/kSensorLabels.hpp"
#include "shaders/kThermalPalette.hpp"
#include "shaders/kThermalTemperature.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <mutex>
#include <utility>

namespace lagrange::gpu {

namespace {

using Matrix = std::array<double, 9>;  // row-major

/// Rotation matrix of a unit quaternion (w, x, y, z).
Matrix matrixOf(const std::array<double, 4>& q) {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    return {1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y),
            2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
            2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)};
}

// The camera sensor's targets: the colour pass (multisampled, averaged on resolve) and the
// labels pass (linear depth, instance id; one sample).
constexpr VkFormat kColorFormat = VK_FORMAT_R8G8B8A8_UNORM;
constexpr std::array<VkFormat, 2> kLabelFormats = {VK_FORMAT_R32_SFLOAT, VK_FORMAT_R32_UINT};
constexpr std::size_t kSensorBytesPerPixel = 4 + 4 + 4;

// Background where nothing is drawn: the shader's sky colour (0.55, 0.60, 0.70), linear,
// encoded for display as the shader encodes its output.
constexpr std::array<float, 3> kSkyLinear = {0.55F, 0.60F, 0.70F};

}  // namespace

std::array<float, 12> modelRows(const std::array<double, 3>& position,
                                const std::array<double, 4>& rotation,
                                const std::array<float, 3>& scale,
                                const std::array<double, 3>& eye) {
    const Matrix r = matrixOf(rotation);
    std::array<float, 12> m{};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            m.at(row * 4 + column) = static_cast<float>(r.at(row * 3 + column) * scale.at(column));
        }
        m.at(row * 4 + 3) = static_cast<float>(position.at(row) - eye.at(row));
    }
    return m;
}

void buildPipeline(const Device& device, const PipelineSpec& spec, VkPipelineLayout& layout,
                   VkPipeline& pipeline) {
    VkPushConstantRange range{VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0,
                              spec.pushSize};
    VkPipelineLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    layoutInfo.pushConstantRangeCount = 1;
    layoutInfo.pPushConstantRanges = &range;
    if (spec.setLayout != VK_NULL_HANDLE) {
        layoutInfo.setLayoutCount = 1;
        layoutInfo.pSetLayouts = &spec.setLayout;
    }
    vkCreatePipelineLayout(device.device(), &layoutInfo, nullptr, &layout);

    VkShaderModule vertex = createShaderModule(device, spec.vertex);
    VkShaderModule fragment = createShaderModule(device, spec.fragment);
    std::array<VkPipelineShaderStageCreateInfo, 2> stages{};
    stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
    stages[0].module = vertex;
    stages[0].pName = "main";
    stages[1] = stages[0];
    stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    stages[1].module = fragment;

    const VkVertexInputBindingDescription binding{0, spec.stride, VK_VERTEX_INPUT_RATE_VERTEX};
    const std::array<VkVertexInputAttributeDescription, 2> attributes = {{
        {0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},
        {1, 0, VK_FORMAT_R32G32B32_SFLOAT, 12},
    }};
    VkPipelineVertexInputStateCreateInfo vertexInput{
        VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    vertexInput.vertexBindingDescriptionCount = spec.stride != 0 ? 1 : 0;  // 0: no vertex buffer
    vertexInput.pVertexBindingDescriptions = &binding;
    vertexInput.vertexAttributeDescriptionCount = spec.stride == 0 ? 0 : (spec.hasNormal ? 2 : 1);
    vertexInput.pVertexAttributeDescriptions = attributes.data();

    VkPipelineInputAssemblyStateCreateInfo assembly{
        VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    assembly.topology = spec.topology;
    VkPipelineViewportStateCreateInfo viewport{
        VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    viewport.viewportCount = 1;
    viewport.scissorCount = 1;
    VkPipelineRasterizationStateCreateInfo raster{
        VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    raster.polygonMode = spec.polygonMode;
    raster.cullMode = VK_CULL_MODE_NONE;  // imported meshes do not have reliable winding yet
    raster.lineWidth = 1.0F;
    VkPipelineMultisampleStateCreateInfo multisample{
        VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
    multisample.rasterizationSamples = spec.samples;
    VkPipelineDepthStencilStateCreateInfo depth{
        VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
    depth.depthTestEnable = spec.depthTest ? VK_TRUE : VK_FALSE;
    depth.depthWriteEnable = spec.depthWrite ? VK_TRUE : VK_FALSE;
    depth.depthCompareOp = VK_COMPARE_OP_GREATER_OR_EQUAL;  // reversed depth
    std::vector<VkPipelineColorBlendAttachmentState> blendAttachments(spec.colorFormats.size());
    for (VkPipelineColorBlendAttachmentState& attachment : blendAttachments) {
        attachment.colorWriteMask = 0xF;
    }
    if (spec.blend && !blendAttachments.empty()) {
        VkPipelineColorBlendAttachmentState& first = blendAttachments.front();
        first.blendEnable = VK_TRUE;
        first.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
        first.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        first.colorBlendOp = VK_BLEND_OP_ADD;
        first.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        first.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        first.alphaBlendOp = VK_BLEND_OP_MAX;
    }
    VkPipelineColorBlendStateCreateInfo blend{
        VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    blend.attachmentCount = static_cast<std::uint32_t>(blendAttachments.size());
    blend.pAttachments = blendAttachments.data();
    const std::array<VkDynamicState, 2> dynamicStates = {VK_DYNAMIC_STATE_VIEWPORT,
                                                         VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dynamic{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    dynamic.dynamicStateCount = static_cast<std::uint32_t>(dynamicStates.size());
    dynamic.pDynamicStates = dynamicStates.data();
    VkPipelineRenderingCreateInfo rendering{VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO};
    rendering.colorAttachmentCount = static_cast<std::uint32_t>(spec.colorFormats.size());
    rendering.pColorAttachmentFormats = spec.colorFormats.data();
    rendering.depthAttachmentFormat = kDepthFormat;

    VkGraphicsPipelineCreateInfo info{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    info.pNext = &rendering;
    info.stageCount = static_cast<std::uint32_t>(stages.size());
    info.pStages = stages.data();
    info.pVertexInputState = &vertexInput;
    info.pInputAssemblyState = &assembly;
    info.pViewportState = &viewport;
    info.pRasterizationState = &raster;
    info.pMultisampleState = &multisample;
    info.pDepthStencilState = &depth;
    info.pColorBlendState = &blend;
    info.pDynamicState = &dynamic;
    info.layout = layout;
    vkCreateGraphicsPipelines(device.device(), VK_NULL_HANDLE, 1, &info, nullptr, &pipeline);

    vkDestroyShaderModule(device.device(), vertex, nullptr);
    vkDestroyShaderModule(device.device(), fragment, nullptr);
}

// --- MeshSet ----------------------------------------------------------------------------------

void MeshSet::clear() {
    destroy(device_, vertices_);
    destroy(device_, indices_);
    meshes_.clear();
}

void MeshSet::upload(std::span<const MeshSource> meshes) {
    clear();
    meshes_.resize(meshes.size());
    // One buffer for all vertices and one for all indices: two transfers in all, instead of two
    // per mesh (a town has thousands of meshes, and each transfer waited for the queue).
    std::size_t vertexTotal = 0;
    std::size_t indexTotal = 0;
    for (const MeshSource& source : meshes) {
        if (source.positions.size() / 3 != 0 && !source.indices.empty()) {
            vertexTotal += source.positions.size() / 3;
            indexTotal += source.indices.size();
        }
    }
    if (vertexTotal == 0 || indexTotal == 0) {
        return;
    }
    std::vector<float> interleaved(vertexTotal * 6);
    std::vector<std::uint32_t> indices;
    indices.reserve(indexTotal);
    std::size_t vertexAt = 0;
    for (std::size_t i = 0; i < meshes.size(); ++i) {
        const MeshSource& source = meshes[i];
        const std::size_t vertexCount = source.positions.size() / 3;
        if (vertexCount == 0 || source.indices.empty()) {
            continue;
        }
        float* out = &interleaved[vertexAt * 6];
        std::array<float, 6>& box = meshes_[i].box;
        box = {INFINITY, INFINITY, INFINITY, -INFINITY, -INFINITY, -INFINITY};
        for (std::size_t v = 0; v < vertexCount; ++v) {
            std::memcpy(&out[v * 6], &source.positions[v * 3], 12);
            for (std::size_t a = 0; a < 3; ++a) {
                box.at(a) = std::min(box.at(a), source.positions[v * 3 + a]);
                box.at(a + 3) = std::max(box.at(a + 3), source.positions[v * 3 + a]);
            }
            if (source.normals.size() < source.positions.size()) {
                out[v * 6 + 5] = 1.0F;  // no normals: light it as if facing up
            } else {
                std::memcpy(&out[v * 6 + 3], &source.normals[v * 3], 12);
            }
        }
        meshes_[i].firstIndex = static_cast<std::uint32_t>(indices.size());
        meshes_[i].vertexOffset = static_cast<std::int32_t>(vertexAt);
        meshes_[i].indexCount = static_cast<std::uint32_t>(source.indices.size());
        indices.insert(indices.end(), source.indices.begin(), source.indices.end());
        vertexAt += vertexCount;
    }
    vertices_ = createBufferWithData(device_, std::as_bytes(std::span{interleaved}),
                                     VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    indices_ = createBufferWithData(device_, std::as_bytes(std::span{indices}),
                                    VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
}

bool MeshSet::draw(VkCommandBuffer commands, std::size_t index) const {
    if (index >= meshes_.size() || meshes_[index].indexCount == 0) {
        return false;
    }
    const GpuMesh& mesh = meshes_[index];
    const VkDeviceSize zero = 0;
    vkCmdBindVertexBuffers(commands, 0, 1, &vertices_.buffer, &zero);
    vkCmdBindIndexBuffer(commands, indices_.buffer, 0, VK_INDEX_TYPE_UINT32);
    vkCmdDrawIndexed(commands, mesh.indexCount, 1, mesh.firstIndex, mesh.vertexOffset, 0);
    return true;
}

// --- RasterScene ------------------------------------------------------------------------------

struct Pipeline {
    VkPipelineLayout layout{VK_NULL_HANDLE};
    VkPipeline pipeline{VK_NULL_HANDLE};
};

struct RasterScene::Impl {
    std::shared_ptr<Device> device;  // shared with the scenes made from this one (create's reuse)
    std::unique_ptr<MeshSet> meshes;
    Pipeline color;   // multisampled: the image
    Pipeline labels;  // one sample: depth and instance id, never blended across an edge
    VkSampleCountFlagBits samples{VK_SAMPLE_COUNT_1_BIT};
    std::shared_ptr<const RealisticTextures> textures;  // with the realistic look
    int season{0};
    // One render at a time on the queue: shared, like `device`, with the scenes made from this.
    std::shared_ptr<std::mutex> queueMutex = std::make_shared<std::mutex>();
};

namespace {

/// The most samples, up to 8, that colour and depth targets both support.
VkSampleCountFlagBits sampleCount(VkPhysicalDevice physical) {
    VkPhysicalDeviceProperties properties{};
    vkGetPhysicalDeviceProperties(physical, &properties);
    const VkSampleCountFlags supported = properties.limits.framebufferColorSampleCounts &
                                         properties.limits.framebufferDepthSampleCounts;
    for (const VkSampleCountFlagBits count :
         {VK_SAMPLE_COUNT_8_BIT, VK_SAMPLE_COUNT_4_BIT, VK_SAMPLE_COUNT_2_BIT}) {
        if ((supported & count) != 0) {
            return count;
        }
    }
    return VK_SAMPLE_COUNT_1_BIT;
}

void destroy(VkDevice device, Pipeline& pipeline) {
    vkDestroyPipeline(device, pipeline.pipeline, nullptr);
    vkDestroyPipelineLayout(device, pipeline.layout, nullptr);
    pipeline = {};
}

}  // namespace

RasterScene::RasterScene() : impl_(std::make_unique<Impl>()) {}

RasterScene::~RasterScene() {
    if (!impl_->device) {
        return;
    }
    VkDevice device = impl_->device->device();
    vkDeviceWaitIdle(device);
    impl_->meshes.reset();
    impl_->textures.reset();
    destroy(device, impl_->color);
    destroy(device, impl_->labels);
}

namespace {
/// LAGRANGE_TIMING=1: how long each step of making a camera scene took, on stderr.
void gpuTiming(const char* stage, std::chrono::steady_clock::time_point& since) {
    const auto now = std::chrono::steady_clock::now();
    static const bool on = std::getenv("LAGRANGE_TIMING") != nullptr;
    if (on) {
        std::fprintf(stderr, "[timing] gpu %s %.1f ms\n", stage,
                     std::chrono::duration<double, std::milli>(now - since).count());
    }
    since = now;
}
}  // namespace

std::shared_ptr<RasterScene> RasterScene::create(const std::string& preferredDevice,
                                                 std::span<const MeshSource> meshes,
                                                 std::string& error, RasterLook look,
                                                 const std::shared_ptr<const RasterScene>& reuse) {
    auto t = std::chrono::steady_clock::now();
    std::shared_ptr<Device> device;
    if (reuse && reuse->impl_->device) {
        device = reuse->impl_->device;
    } else {
        DeviceDesc desc;
        desc.applicationName = "lagrange_camera";
        desc.preferredDevice = preferredDevice;
        device = Device::create(desc, error);
    }
    gpuTiming("device", t);
    if (!device) {
        return nullptr;
    }
    std::shared_ptr<RasterScene> scene(new RasterScene());
    Impl& impl = *scene->impl_;
    if (reuse && reuse->impl_->device) {
        impl.queueMutex = reuse->impl_->queueMutex;  // one queue: one lock
    }
    impl.samples = sampleCount(device->physicalDevice());
    PipelineSpec spec;
    spec.vertex = kMeshVertex;
    spec.fragment = kSensorColor;
    spec.colorFormats = std::span{&kColorFormat, 1};
    spec.blend = false;
    spec.samples = impl.samples;
    buildPipeline(*device, spec, impl.color.layout, impl.color.pipeline);
    spec.fragment = kSensorLabels;
    spec.colorFormats = kLabelFormats;
    spec.samples = VK_SAMPLE_COUNT_1_BIT;
    buildPipeline(*device, spec, impl.labels.layout, impl.labels.pipeline);
    gpuTiming("pipelines", t);
    if (impl.color.pipeline == VK_NULL_HANDLE || impl.labels.pipeline == VK_NULL_HANDLE) {
        error = "cannot build the camera pipelines on " + device->info().name;
        destroy(device->device(), impl.color);
        destroy(device->device(), impl.labels);
        return nullptr;
    }
    impl.meshes = std::make_unique<MeshSet>(*device);
    impl.meshes->upload(meshes);
    gpuTiming("meshes", t);
    impl.device = std::move(device);
    if (look.realistic && !impl.device->info().isSoftware) {
        impl.textures = reuse && reuse->impl_->textures && reuse->impl_->device == impl.device
                            ? reuse->impl_->textures
                            : std::make_shared<RealisticTextures>(*impl.device);
        impl.season = look.season;
    }
    gpuTiming("textures", t);
    return scene;
}

const std::string& RasterScene::deviceName() const {
    return impl_->device->info().name;
}

bool RasterScene::isSoftware() const {
    return impl_->device->info().isSoftware;
}

std::uint32_t RasterScene::samples() const {
    return static_cast<std::uint32_t>(impl_->samples);
}

bool RasterScene::realistic() const {
    return impl_->textures != nullptr;
}

// --- RasterCamera -----------------------------------------------------------------------------

struct RasterCamera::Impl {
    std::shared_ptr<RasterScene> scene;
    VkExtent2D extent{};
    Image color;                  // multisampled (when the scene has more than one sample)
    Image colorResolved;          // one sample per pixel: read back
    Image colorDepth;             // depth buffer of the colour pass
    std::array<Image, 2> labels;  // linear depth, instance id: read back
    Image labelDepth;             // depth buffer of the labels pass
    Buffer readback;
    Image realisticDepth;  // with the realistic look: sampled by it
    std::unique_ptr<RealisticRenderer> realistic;
    std::vector<RealisticDraw> draws;
    bool colour{true};  // false: only depth and ids are drawn (setColour)
    // The sensor model on the GPU (setEffects): its kernel, inputs and output.
    ComputeKernel effects;
    CameraEffects effectsSetup;
    Buffer effectsIn;  // the render's images for the camera model, on the device
    Buffer effectsRays;
    Buffer effectsTables;
    Buffer effectsOut;
    Device::Submission inFlight;  // begin()'s commands, until finish()
    bool inFlightModel{false};    // they end with the model (its output is what to read)
    bool inFlightThermal{false};  // they end with the thermal image
    // The thermal image on the GPU (setThermal): its kernels, inputs and output.
    ComputeKernel thermalTemperature, thermalPalette;
    ThermalImageSetup thermalSetup;
    Buffer thermalIn;     // the render's images, on the device
    Buffer thermalTable;  // K per scene prim, written each frame (host-visible)
    Buffer thermalRange;  // the frame's lowest and highest temperature
    Buffer thermalOut;    // palette, depth, ids, temperatures (host-visible)
};

namespace {

// gpu/kernels/camera/camera_model.slang's push constants (std430).
struct EffectsParams {
    std::array<float, 4> omega;  // xyz rad/s, w rolling shutter (s)
    std::uint32_t width, height, renderWidth, renderHeight;
    std::array<float, 4> intrinsics;
    std::uint32_t exposure;
    float electronsPerUnit, gain, fullWell, readNoise, fullScale;
    std::uint32_t key0, key1, step0, step1, entity, pad;
};
static_assert(sizeof(EffectsParams) == 96);
constexpr std::uint32_t kEffectsGroup = 64;  // numthreads of applyModel

// gpu/kernels/camera/thermal_camera.slang's push constants.
struct ThermalParams {
    std::uint32_t width, height, prims, palette;
    float sky, netd, displayMin, displayMax, span;
    std::uint32_t key0, key1, step0, step1, entity, pad0, pad1;
};
static_assert(sizeof(ThermalParams) == 64);
constexpr std::uint32_t kThermalGroup = 64;                   // numthreads of both kernels
constexpr std::uint32_t kPositiveInfinityBits = 0x7F800000U;  // the range's starting minimum

/// Makes the compute work before it visible to the compute work after.
void computeToCompute(VkCommandBuffer commands) {
    VkMemoryBarrier2 barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER_2};
    barrier.srcStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
    barrier.srcAccessMask = VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
    barrier.dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
    barrier.dstAccessMask =
        VK_ACCESS_2_SHADER_STORAGE_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
    VkDependencyInfo dependency{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
    dependency.memoryBarrierCount = 1;
    dependency.pMemoryBarriers = &barrier;
    vkCmdPipelineBarrier2(commands, &dependency);
}

}  // namespace

RasterCamera::RasterCamera(std::shared_ptr<RasterScene> scene, std::uint32_t width,
                           std::uint32_t height)
    : impl_(std::make_unique<Impl>()) {
    impl_->scene = std::move(scene);
    impl_->extent = {std::max(width, 1U), std::max(height, 1U)};
    const Device& device = *impl_->scene->impl_->device;
    const VkSampleCountFlagBits samples = impl_->scene->impl_->samples;
    const VkExtent2D extent = impl_->extent;
    constexpr VkImageUsageFlags kReadBack =
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    impl_->colorResolved =
        createImage2D(device, extent, kColorFormat, kReadBack, VK_IMAGE_ASPECT_COLOR_BIT);
    if (samples != VK_SAMPLE_COUNT_1_BIT) {
        impl_->color =
            createImage2D(device, extent, kColorFormat, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
                          VK_IMAGE_ASPECT_COLOR_BIT, samples);
    }
    impl_->colorDepth =
        createImage2D(device, extent, kDepthFormat, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
                      VK_IMAGE_ASPECT_DEPTH_BIT, samples);
    for (std::size_t k = 0; k < kLabelFormats.size(); ++k) {
        impl_->labels.at(k) = createImage2D(device, extent, kLabelFormats.at(k), kReadBack,
                                            VK_IMAGE_ASPECT_COLOR_BIT);
    }
    impl_->labelDepth =
        createImage2D(device, extent, kDepthFormat, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
                      VK_IMAGE_ASPECT_DEPTH_BIT);
    const VkDeviceSize pixels = VkDeviceSize{extent.width} * extent.height;
    impl_->readback =
        createBuffer(device, pixels * kSensorBytesPerPixel,
                     VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, true);
    if (impl_->scene->impl_->textures) {
        impl_->realisticDepth =
            createImage2D(device, extent, kDepthFormat,
                          VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                          VK_IMAGE_ASPECT_DEPTH_BIT);
        auto t = std::chrono::steady_clock::now();
        impl_->realistic =
            std::make_unique<RealisticRenderer>(device, impl_->scene->impl_->textures);
        gpuTiming("realistic renderer", t);
        impl_->realistic->resize(extent, impl_->realisticDepth);
        gpuTiming("realistic resize", t);
    }
}

RasterCamera::~RasterCamera() {
    const Device& device = *impl_->scene->impl_->device;
    const std::lock_guard<std::mutex> lock(*impl_->scene->impl_->queueMutex);
    vkDeviceWaitIdle(device.device());
    device.wait(impl_->inFlight);  // a begin() never finished
    impl_->realistic.reset();
    destroy(device, impl_->realisticDepth);
    for (Image* image : {&impl_->color, &impl_->colorResolved, &impl_->colorDepth,
                         &impl_->labels[0], &impl_->labels[1], &impl_->labelDepth}) {
        destroy(device, *image);
    }
    destroy(device, impl_->readback);
    destroy(device, impl_->effects);
    for (Buffer* buffer :
         {&impl_->effectsIn, &impl_->effectsRays, &impl_->effectsTables, &impl_->effectsOut,
          &impl_->thermalIn, &impl_->thermalTable, &impl_->thermalRange, &impl_->thermalOut}) {
        if (buffer->buffer != VK_NULL_HANDLE) {
            destroy(device, *buffer);
        }
    }
    destroy(device, impl_->thermalTemperature);
    destroy(device, impl_->thermalPalette);
}

void RasterCamera::setColour(bool drawn) {
    impl_->colour = drawn;
}

bool RasterCamera::setEffects(const CameraEffects& effects, std::string& error) {
    const Device& device = *impl_->scene->impl_->device;
    const std::size_t pixels = std::size_t{effects.width} * effects.height;
    if (pixels == 0 || effects.rays.size() != pixels) {
        error = "the camera model's rays do not match its size";
        return false;
    }
    const std::lock_guard<std::mutex> lock(*impl_->scene->impl_->queueMutex);
    if (impl_->effects.pipeline == VK_NULL_HANDLE) {
        impl_->effects = createComputeKernel(device, kCameraModel, 4,
                                             static_cast<std::uint32_t>(sizeof(EffectsParams)));
        if (impl_->effects.pipeline == VK_NULL_HANDLE) {
            error = "cannot build the camera model kernel on " + device.info().name;
            return false;
        }
    }
    for (Buffer* buffer :
         {&impl_->effectsIn, &impl_->effectsRays, &impl_->effectsTables, &impl_->effectsOut}) {
        if (buffer->buffer != VK_NULL_HANDLE) {
            destroy(device, *buffer);
        }
    }
    impl_->effectsRays = createBufferWithData(device, std::as_bytes(std::span(effects.rays)),
                                              VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    std::vector<std::uint32_t> tables(256 + effects.encoded.size());
    std::memcpy(tables.data(), effects.decode.data(), 256 * sizeof(float));
    std::copy(effects.encoded.begin(), effects.encoded.end(), tables.begin() + 256);
    impl_->effectsTables = createBufferWithData(device, std::as_bytes(std::span(tables)),
                                                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    // The render's images stay on the device for the model (the host reads only its output):
    // a third of the bus traffic of copying them to host memory and reading them back from it.
    impl_->effectsIn =
        createBuffer(device, impl_->readback.size,
                     VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, false);
    impl_->effectsOut = createBuffer(device, pixels * 4 * sizeof(std::uint32_t),
                                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, true);
    impl_->effectsSetup = effects;
    impl_->effectsSetup.rays.clear();  // on the GPU now
    const std::array<const Buffer*, 4> bound{&impl_->effectsIn, &impl_->effectsRays,
                                             &impl_->effectsTables, &impl_->effectsOut};
    bindBuffers(device, impl_->effects, bound);
    return true;
}

void RasterCamera::render(std::span<const RasterInstance> instances, const PinholeCamera& camera,
                          RasterImages& out, double seconds) {
    begin(instances, camera, nullptr, seconds);
    finish(out, nullptr);
}

void RasterCamera::render(std::span<const RasterInstance> instances, const PinholeCamera& camera,
                          const CameraEffectsFrame& frame, RasterImages& out,
                          std::vector<float>& signal, double seconds) {
    begin(instances, camera, &frame, seconds);
    finish(out, &signal);
}

void RasterCamera::begin(std::span<const RasterInstance> instances, const PinholeCamera& camera,
                         const CameraEffectsFrame* frame, double seconds) {
    if (pending()) {
        RasterImages discarded;
        std::vector<float> signal;
        finish(discarded, &signal);
    }
    draw(instances, camera, seconds, frame);
}

bool RasterCamera::pending() const {
    return impl_->inFlight.fence != VK_NULL_HANDLE;
}

bool RasterCamera::setThermal(const ThermalImageSetup& setup, std::string& error) {
    const Device& device = *impl_->scene->impl_->device;
    const std::lock_guard<std::mutex> lock(*impl_->scene->impl_->queueMutex);
    if (impl_->thermalTemperature.pipeline == VK_NULL_HANDLE) {
        impl_->thermalTemperature = createComputeKernel(
            device, kThermalTemperature, 4, static_cast<std::uint32_t>(sizeof(ThermalParams)));
        impl_->thermalPalette = createComputeKernel(
            device, kThermalPalette, 4, static_cast<std::uint32_t>(sizeof(ThermalParams)));
        if (impl_->thermalTemperature.pipeline == VK_NULL_HANDLE ||
            impl_->thermalPalette.pipeline == VK_NULL_HANDLE) {
            error = "cannot build the thermal image kernels on " + device.info().name;
            return false;
        }
    }
    for (Buffer* buffer :
         {&impl_->thermalIn, &impl_->thermalTable, &impl_->thermalRange, &impl_->thermalOut}) {
        if (buffer->buffer != VK_NULL_HANDLE) {
            destroy(device, *buffer);
        }
    }
    const VkDeviceSize pixels = VkDeviceSize{impl_->extent.width} * impl_->extent.height;
    impl_->thermalIn =
        createBuffer(device, impl_->readback.size,
                     VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, false);
    impl_->thermalTable =
        createBuffer(device, std::max<VkDeviceSize>(setup.prims, 1) * sizeof(float),
                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, true);
    impl_->thermalRange =
        createBuffer(device, 2 * sizeof(std::uint32_t),
                     VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, false);
    impl_->thermalOut = createBuffer(device, pixels * 4 * sizeof(std::uint32_t),
                                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, true);
    const std::array<const Buffer*, 4> bound{&impl_->thermalIn, &impl_->thermalTable,
                                             &impl_->thermalRange, &impl_->thermalOut};
    bindBuffers(device, impl_->thermalTemperature, bound);
    bindBuffers(device, impl_->thermalPalette, bound);
    impl_->thermalSetup = setup;
    return true;
}

void RasterCamera::beginThermal(std::span<const RasterInstance> instances,
                                const PinholeCamera& camera, const ThermalImageFrame& frame,
                                double seconds) {
    if (pending()) {
        RasterImages discarded;
        std::vector<float> signal;
        finish(discarded, &signal);
    }
    draw(instances, camera, seconds, nullptr, &frame);
}

void RasterCamera::finish(RasterImages& out, std::vector<float>* signal) {
    if (!pending()) {
        return;
    }
    RasterScene::Impl& scene = *impl_->scene->impl_;
    const Device& device = *scene.device;
    {
        const std::lock_guard<std::mutex> lock(*scene.queueMutex);
        device.wait(impl_->inFlight);
    }
    if (impl_->inFlightThermal) {
        invalidate(device, impl_->thermalOut);
        const VkExtent2D extent = impl_->extent;
        const std::size_t pixels = std::size_t{extent.width} * extent.height;
        const auto* bytes = static_cast<const std::byte*>(impl_->thermalOut.mapped);
        out.rgba.resize(pixels * 4);
        out.depth.resize(pixels);
        out.instance.resize(pixels);
        std::memcpy(out.rgba.data(), bytes, pixels * 4);
        std::memcpy(out.depth.data(), bytes + pixels * 4, pixels * 4);
        std::memcpy(out.instance.data(), bytes + pixels * 8, pixels * 4);
        if (signal != nullptr) {
            signal->resize(pixels);
            std::memcpy(signal->data(), bytes + pixels * 12, pixels * 4);
        }
        return;
    }
    if (!impl_->inFlightModel) {
        const VkExtent2D extent = impl_->extent;
        invalidate(device, impl_->readback);
        const std::size_t pixels = std::size_t{extent.width} * extent.height;
        const auto* bytes = static_cast<const std::byte*>(impl_->readback.mapped);
        out.rgba.resize(pixels * 4);
        out.depth.resize(pixels);
        out.instance.resize(pixels);
        std::memcpy(out.rgba.data(), bytes, pixels * 4);
        std::memcpy(out.depth.data(), bytes + pixels * 4, pixels * 4);
        std::memcpy(out.instance.data(), bytes + pixels * 8, pixels * 4);
        return;
    }
    invalidate(device, impl_->effectsOut);
    const std::size_t pixels = std::size_t{impl_->effectsSetup.width} * impl_->effectsSetup.height;
    const auto* bytes = static_cast<const std::byte*>(impl_->effectsOut.mapped);
    out.rgba.resize(pixels * 4);
    out.depth.resize(pixels);
    out.instance.resize(pixels);
    std::memcpy(out.rgba.data(), bytes, pixels * 4);
    std::memcpy(out.depth.data(), bytes + pixels * 4, pixels * 4);
    std::memcpy(out.instance.data(), bytes + pixels * 8, pixels * 4);
    if (signal != nullptr) {
        signal->resize(pixels);
        std::memcpy(signal->data(), bytes + pixels * 12, pixels * 4);
    }
}

void RasterCamera::draw(std::span<const RasterInstance> instances, const PinholeCamera& camera,
                        double seconds, const CameraEffectsFrame* frame,
                        const ThermalImageFrame* thermal) {
    RasterScene::Impl& scene = *impl_->scene->impl_;
    const Device& device = *scene.device;
    const VkExtent2D extent = impl_->extent;
    const auto w = static_cast<double>(extent.width);
    const auto h = static_cast<double>(extent.height);

    // View: the optical frame's axes in world coordinates are the columns of R, so the view
    // rotation's rows are those columns. Projection: OpenCV pixel centres at integers, Vulkan
    // framebuffer centres at +0.5, y down in both; reversed depth near / z with no far plane.
    const Matrix r = matrixOf(camera.orientation);
    const auto axis = [&r](std::size_t c) {
        return std::array<double, 3>{r[c], r[3 + c], r[6 + c]};
    };
    const std::array<double, 3> xAxis = axis(0), yAxis = axis(1), zAxis = axis(2);
    const double px = 2.0 * camera.fx / w, ox = 2.0 * (camera.cx + 0.5) / w - 1.0;
    const double py = 2.0 * camera.fy / h, oy = 2.0 * (camera.cy + 0.5) / h - 1.0;
    MeshPush push{};
    for (std::size_t i = 0; i < 3; ++i) {
        push.viewProjection[0].at(i) = static_cast<float>(px * xAxis.at(i) + ox * zAxis.at(i));
        push.viewProjection[1].at(i) = static_cast<float>(py * yAxis.at(i) + oy * zAxis.at(i));
        push.viewProjection[3].at(i) = static_cast<float>(zAxis.at(i));
    }
    push.viewProjection[2] = {0.0F, 0.0F, 0.0F, static_cast<float>(camera.near)};

    const auto attachment = [](const Image& image, VkClearValue clear) {
        VkRenderingAttachmentInfo info{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
        info.imageView = image.view;
        info.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        info.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        info.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        info.clearValue = clear;
        return info;
    };
    const auto depthAttachment = [](const Image& image) {
        VkRenderingAttachmentInfo info{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
        info.imageView = image.view;
        info.imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
        info.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        info.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        info.clearValue.depthStencil = {0.0F, 0};  // reversed: 0 is infinitely far
        return info;
    };
    const auto pass = [&](VkCommandBuffer commands,
                          std::span<const VkRenderingAttachmentInfo> colors,
                          const VkRenderingAttachmentInfo& depth, const Pipeline& pipeline) {
        VkRenderingInfo rendering{VK_STRUCTURE_TYPE_RENDERING_INFO};
        rendering.renderArea = {{0, 0}, extent};
        rendering.layerCount = 1;
        rendering.colorAttachmentCount = static_cast<std::uint32_t>(colors.size());
        rendering.pColorAttachments = colors.data();
        rendering.pDepthAttachment = &depth;
        vkCmdBeginRendering(commands, &rendering);
        const VkViewport viewport{0.0F, 0.0F, static_cast<float>(w), static_cast<float>(h),
                                  0.0F, 1.0F};
        const VkRect2D scissor{{0, 0}, extent};
        vkCmdSetViewport(commands, 0, 1, &viewport);
        vkCmdSetScissor(commands, 0, 1, &scissor);
        vkCmdBindPipeline(commands, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline.pipeline);
        for (const RasterInstance& instance : instances) {
            push.model =
                modelRows(instance.position, instance.rotation, instance.scale, camera.position);
            // The id travels as a float, exact up to 2^24 instances.
            push.baseColor = {instance.color[0], instance.color[1], instance.color[2],
                              static_cast<float>(instance.id)};
            vkCmdPushConstants(commands, pipeline.layout,
                               VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0,
                               sizeof(push), &push);
            scene.meshes->draw(commands, instance.mesh);
        }
        vkCmdEndRendering(commands);
    };

    const std::lock_guard<std::mutex> lock(*scene.queueMutex);
    impl_->inFlightModel = frame != nullptr;
    impl_->inFlightThermal = thermal != nullptr && frame == nullptr;
    if (impl_->inFlightThermal) {
        // This frame's table, read by the kernel (one frame in flight per camera).
        auto* table = static_cast<float*>(impl_->thermalTable.mapped);
        const std::size_t count =
            std::min<std::size_t>(thermal->apparent.size(), impl_->thermalSetup.prims);
        for (std::size_t k = 0; k < count; ++k) {
            table[k] = static_cast<float>(thermal->apparent[k]);
        }
        flush(device, impl_->thermalTable);
    }
    impl_->inFlight = device.submit([&](VkCommandBuffer commands) {
        const bool multisampled = scene.samples != VK_SAMPLE_COUNT_1_BIT;
        std::vector<const Image*> targets{&impl_->colorResolved, &impl_->labels[0],
                                          &impl_->labels[1]};
        if (multisampled) {
            targets.push_back(&impl_->color);
        }
        for (const Image* target : targets) {
            transitionImage(commands, target->image, VK_IMAGE_ASPECT_COLOR_BIT,
                            VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
        }
        for (const Image* buffer : {&impl_->colorDepth, &impl_->labelDepth}) {
            transitionImage(commands, buffer->image, VK_IMAGE_ASPECT_DEPTH_BIT,
                            VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL);
        }

        if (impl_->realistic && impl_->colour) {
            impl_->draws.clear();
            for (const RasterInstance& instance : instances) {
                RealisticDraw draw;
                draw.mesh = instance.mesh;
                draw.position = instance.position;
                for (std::size_t k = 0; k < 4; ++k) {
                    draw.rotation.at(k) = static_cast<float>(instance.rotation.at(k));
                }
                draw.scale = instance.scale;
                draw.baseColor = instance.color;
                draw.metallic = instance.metallic;
                draw.roughness = instance.roughness;
                draw.pattern = instance.pattern;
                draw.emissive = instance.emissive;
                draw.ground = instance.ground;
                draw.water = instance.water;
                draw.box = instance.box;
                impl_->draws.push_back(draw);
            }
            RealisticView view;
            for (std::size_t row = 0; row < 4; ++row) {
                for (std::size_t column = 0; column < 4; ++column) {
                    view.viewProjection.at(row * 4 + column) =
                        push.viewProjection.at(row).at(column);
                }
            }
            view.eye = camera.position;
            view.right = xAxis;
            view.up = {-yAxis[0], -yAxis[1], -yAxis[2]};
            view.forward = zAxis;
            constexpr double kFocus = 12.0;  // m ahead: where the sharpest shadows are
            view.focus = {camera.position[0] + kFocus * zAxis[0],
                          camera.position[1] + kFocus * zAxis[1],
                          camera.position[2] + kFocus * zAxis[2]};
            view.focusDistance = kFocus;
            view.tanHalfX = w / (2.0 * camera.fx);
            view.tanHalfY = h / (2.0 * camera.fy);
            view.near = camera.near;
            RealisticSettings settings;
            settings.season = scene.season;
            settings.seconds = seconds;
            transitionImage(commands, impl_->realisticDepth.image, VK_IMAGE_ASPECT_DEPTH_BIT,
                            VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL);
            impl_->realistic->record(commands, impl_->draws, view, settings, *scene.meshes,
                                     impl_->colorResolved, impl_->realisticDepth);
        }

        VkClearValue sky{};
        for (std::size_t c = 0; c < 3; ++c) {
            sky.color.float32[c] = std::pow(kSkyLinear.at(c), 1.0F / 2.2F);  // as the shader
        }
        sky.color.float32[3] = 1.0F;
        VkRenderingAttachmentInfo color =
            attachment(multisampled ? impl_->color : impl_->colorResolved, sky);
        if (multisampled) {
            color.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
            color.resolveMode = VK_RESOLVE_MODE_AVERAGE_BIT;
            color.resolveImageView = impl_->colorResolved.view;
            color.resolveImageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        }
        if (!impl_->realistic && impl_->colour) {
            pass(commands, std::span{&color, 1}, depthAttachment(impl_->colorDepth), scene.color);
        }

        VkClearValue far{};
        far.color.float32[0] = std::numeric_limits<float>::infinity();
        VkClearValue none{};
        none.color.uint32[0] = kNoInstance;
        const std::array<VkRenderingAttachmentInfo, 2> labels = {
            attachment(impl_->labels[0], far), attachment(impl_->labels[1], none)};
        pass(commands, labels, depthAttachment(impl_->labelDepth), scene.labels);

        // With the model or the thermal image, the images go to their device-local input.
        const VkBuffer images = frame != nullptr          ? impl_->effectsIn.buffer
                                : impl_->inFlightThermal ? impl_->thermalIn.buffer
                                                         : impl_->readback.buffer;
        VkDeviceSize offset = 0;
        for (const Image* target : {&impl_->colorResolved, &impl_->labels[0], &impl_->labels[1]}) {
            if (target == &impl_->colorResolved && !impl_->colour) {
                // Not drawn: the colour reads back as zeros.
                const VkDeviceSize bytes = VkDeviceSize{extent.width} * extent.height * 4;
                vkCmdFillBuffer(commands, images, 0, bytes, 0);
                offset += bytes;
                continue;
            }
            transitionImage(commands, target->image, VK_IMAGE_ASPECT_COLOR_BIT,
                            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                            VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
            VkBufferImageCopy region{};
            region.bufferOffset = offset;
            region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
            region.imageExtent = {extent.width, extent.height, 1};
            vkCmdCopyImageToBuffer(commands, target->image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                                   images, 1, &region);
            offset += VkDeviceSize{extent.width} * extent.height * 4;
        }
        if (impl_->inFlightThermal) {
            // The range starts empty; the copies and that, then the two passes.
            vkCmdFillBuffer(commands, impl_->thermalRange.buffer, 0, sizeof(std::uint32_t),
                            kPositiveInfinityBits);
            vkCmdFillBuffer(commands, impl_->thermalRange.buffer, sizeof(std::uint32_t),
                            sizeof(std::uint32_t), 0);
            VkMemoryBarrier2 copied{VK_STRUCTURE_TYPE_MEMORY_BARRIER_2};
            copied.srcStageMask = VK_PIPELINE_STAGE_2_COPY_BIT | VK_PIPELINE_STAGE_2_CLEAR_BIT;
            copied.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
            copied.dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
            copied.dstAccessMask =
                VK_ACCESS_2_SHADER_STORAGE_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
            VkDependencyInfo dependency{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
            dependency.memoryBarrierCount = 1;
            dependency.pMemoryBarriers = &copied;
            vkCmdPipelineBarrier2(commands, &dependency);
            const ThermalImageSetup& setup = impl_->thermalSetup;
            ThermalParams params{};
            params.width = extent.width;
            params.height = extent.height;
            params.prims = setup.prims;
            params.palette = setup.palette;
            params.sky = static_cast<float>(thermal->sky);
            params.netd = static_cast<float>(setup.netd);
            params.displayMin = static_cast<float>(setup.displayMin);
            params.displayMax = static_cast<float>(setup.displayMax);
            params.span = static_cast<float>(setup.span);
            params.key0 = static_cast<std::uint32_t>(thermal->seed);
            params.key1 = static_cast<std::uint32_t>(thermal->seed >> 32U);
            params.step0 = static_cast<std::uint32_t>(thermal->stepIndex);
            params.step1 = static_cast<std::uint32_t>(thermal->stepIndex >> 32U);
            params.entity = thermal->entity;
            const std::uint32_t pixels = extent.width * extent.height;
            const std::uint32_t groups = (pixels + kThermalGroup - 1) / kThermalGroup;
            recordDispatch(commands, impl_->thermalTemperature,
                           std::as_bytes(std::span(&params, 1)), groups);
            computeToCompute(commands);
            recordDispatch(commands, impl_->thermalPalette, std::as_bytes(std::span(&params, 1)),
                           groups);
        }
        if (frame != nullptr) {
            // The copies, then the model reading them.
            VkMemoryBarrier2 copied{VK_STRUCTURE_TYPE_MEMORY_BARRIER_2};
            copied.srcStageMask = VK_PIPELINE_STAGE_2_COPY_BIT;
            copied.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
            copied.dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
            copied.dstAccessMask = VK_ACCESS_2_SHADER_STORAGE_READ_BIT;
            VkDependencyInfo dependency{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
            dependency.memoryBarrierCount = 1;
            dependency.pMemoryBarriers = &copied;
            vkCmdPipelineBarrier2(commands, &dependency);
            const CameraEffects& setup = impl_->effectsSetup;
            const bool exposure = !setup.encoded.empty();
            EffectsParams params{};
            params.omega = {
                static_cast<float>(frame->omega[0]), static_cast<float>(frame->omega[1]),
                static_cast<float>(frame->omega[2]), static_cast<float>(frame->rollingShutterTime)};
            params.width = setup.width;
            params.height = setup.height;
            params.renderWidth = extent.width;
            params.renderHeight = extent.height;
            params.intrinsics = {static_cast<float>(setup.fx), static_cast<float>(setup.fy),
                                 static_cast<float>(setup.cx), static_cast<float>(setup.cy)};
            params.exposure = exposure ? 1U : 0U;
            params.electronsPerUnit = static_cast<float>(frame->electronsPerUnit);
            params.gain = static_cast<float>(frame->gain);
            params.fullWell = static_cast<float>(frame->fullWell);
            params.readNoise = static_cast<float>(frame->readNoise);
            params.fullScale = exposure ? static_cast<float>(setup.encoded.size() - 1) : 255.0F;
            params.key0 = static_cast<std::uint32_t>(frame->seed);
            params.key1 = static_cast<std::uint32_t>(frame->seed >> 32U);
            params.step0 = static_cast<std::uint32_t>(frame->stepIndex);
            params.step1 = static_cast<std::uint32_t>(frame->stepIndex >> 32U);
            params.entity = frame->entity;
            const std::uint32_t pixels = setup.width * setup.height;
            recordDispatch(commands, impl_->effects, std::as_bytes(std::span(&params, 1)),
                           (pixels + kEffectsGroup - 1) / kEffectsGroup);
        }
    });
}

}  // namespace lagrange::gpu
