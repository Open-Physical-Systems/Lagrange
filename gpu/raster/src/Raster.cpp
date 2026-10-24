#include "lagrange/gpu/Device.hpp"
#include "lagrange/gpu/Raster.hpp"
#include "shaders/kMeshVertex.hpp"
#include "shaders/kSensorColor.hpp"
#include "shaders/kSensorLabels.hpp"

#include <algorithm>
#include <cmath>
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
    vertexInput.vertexBindingDescriptionCount = 1;
    vertexInput.pVertexBindingDescriptions = &binding;
    vertexInput.vertexAttributeDescriptionCount = spec.hasNormal ? 2 : 1;
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
    depth.depthTestEnable = VK_TRUE;
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
    for (GpuMesh& mesh : meshes_) {
        destroy(device_, mesh.vertices);
        destroy(device_, mesh.indices);
    }
    meshes_.clear();
}

void MeshSet::upload(std::span<const MeshSource> meshes) {
    clear();
    meshes_.resize(meshes.size());
    for (std::size_t i = 0; i < meshes.size(); ++i) {
        const MeshSource& source = meshes[i];
        const std::size_t vertexCount = source.positions.size() / 3;
        if (vertexCount == 0 || source.indices.empty()) {
            continue;
        }
        std::vector<float> interleaved(vertexCount * 6);
        for (std::size_t v = 0; v < vertexCount; ++v) {
            std::memcpy(&interleaved[v * 6], &source.positions[v * 3], 12);
            if (source.normals.size() < source.positions.size()) {
                interleaved[v * 6 + 5] = 1.0F;  // no normals: light it as if facing up
            } else {
                std::memcpy(&interleaved[v * 6 + 3], &source.normals[v * 3], 12);
            }
        }
        meshes_[i].vertices = createBufferWithData(device_, std::as_bytes(std::span{interleaved}),
                                                   VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
        meshes_[i].indices = createBufferWithData(device_, std::as_bytes(source.indices),
                                                  VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
        meshes_[i].indexCount = static_cast<std::uint32_t>(source.indices.size());
    }
}

bool MeshSet::draw(VkCommandBuffer commands, std::size_t index) const {
    if (index >= meshes_.size() || meshes_[index].indexCount == 0) {
        return false;
    }
    const GpuMesh& mesh = meshes_[index];
    const VkDeviceSize zero = 0;
    vkCmdBindVertexBuffers(commands, 0, 1, &mesh.vertices.buffer, &zero);
    vkCmdBindIndexBuffer(commands, mesh.indices.buffer, 0, VK_INDEX_TYPE_UINT32);
    vkCmdDrawIndexed(commands, mesh.indexCount, 1, 0, 0, 0);
    return true;
}

// --- RasterScene ------------------------------------------------------------------------------

struct Pipeline {
    VkPipelineLayout layout{VK_NULL_HANDLE};
    VkPipeline pipeline{VK_NULL_HANDLE};
};

struct RasterScene::Impl {
    std::unique_ptr<Device> device;
    std::unique_ptr<MeshSet> meshes;
    Pipeline color;   // multisampled: the image
    Pipeline labels;  // one sample: depth and instance id, never blended across an edge
    VkSampleCountFlagBits samples{VK_SAMPLE_COUNT_1_BIT};
    std::mutex mutex;  // one render at a time on the queue
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
    destroy(device, impl_->color);
    destroy(device, impl_->labels);
}

std::shared_ptr<RasterScene> RasterScene::create(const std::string& preferredDevice,
                                                 std::span<const MeshSource> meshes,
                                                 std::string& error) {
    DeviceDesc desc;
    desc.applicationName = "lagrange_camera";
    desc.preferredDevice = preferredDevice;
    std::unique_ptr<Device> device = Device::create(desc, error);
    if (!device) {
        return nullptr;
    }
    std::shared_ptr<RasterScene> scene(new RasterScene());
    Impl& impl = *scene->impl_;
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
    if (impl.color.pipeline == VK_NULL_HANDLE || impl.labels.pipeline == VK_NULL_HANDLE) {
        error = "cannot build the camera pipelines on " + device->info().name;
        destroy(device->device(), impl.color);
        destroy(device->device(), impl.labels);
        return nullptr;
    }
    impl.meshes = std::make_unique<MeshSet>(*device);
    impl.meshes->upload(meshes);
    impl.device = std::move(device);
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
};

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
        createBuffer(device, pixels * kSensorBytesPerPixel, VK_BUFFER_USAGE_TRANSFER_DST_BIT, true);
}

RasterCamera::~RasterCamera() {
    const Device& device = *impl_->scene->impl_->device;
    const std::lock_guard<std::mutex> lock(impl_->scene->impl_->mutex);
    vkDeviceWaitIdle(device.device());
    for (Image* image : {&impl_->color, &impl_->colorResolved, &impl_->colorDepth,
                         &impl_->labels[0], &impl_->labels[1], &impl_->labelDepth}) {
        destroy(device, *image);
    }
    destroy(device, impl_->readback);
}

void RasterCamera::render(std::span<const RasterInstance> instances, const PinholeCamera& camera,
                          RasterImages& out) {
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

    const std::lock_guard<std::mutex> lock(scene.mutex);
    device.submitAndWait([&](VkCommandBuffer commands) {
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
        pass(commands, std::span{&color, 1}, depthAttachment(impl_->colorDepth), scene.color);

        VkClearValue far{};
        far.color.float32[0] = std::numeric_limits<float>::infinity();
        VkClearValue none{};
        none.color.uint32[0] = kNoInstance;
        const std::array<VkRenderingAttachmentInfo, 2> labels = {
            attachment(impl_->labels[0], far), attachment(impl_->labels[1], none)};
        pass(commands, labels, depthAttachment(impl_->labelDepth), scene.labels);

        VkDeviceSize offset = 0;
        for (const Image* target : {&impl_->colorResolved, &impl_->labels[0], &impl_->labels[1]}) {
            transitionImage(commands, target->image, VK_IMAGE_ASPECT_COLOR_BIT,
                            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                            VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
            VkBufferImageCopy region{};
            region.bufferOffset = offset;
            region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
            region.imageExtent = {extent.width, extent.height, 1};
            vkCmdCopyImageToBuffer(commands, target->image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                                   impl_->readback.buffer, 1, &region);
            offset += VkDeviceSize{extent.width} * extent.height * 4;
        }
    });
    invalidate(device, impl_->readback);
    const std::size_t pixels = std::size_t{extent.width} * extent.height;
    const auto* bytes = static_cast<const std::byte*>(impl_->readback.mapped);
    out.rgba.resize(pixels * 4);
    out.depth.resize(pixels);
    out.instance.resize(pixels);
    std::memcpy(out.rgba.data(), bytes, pixels * 4);
    std::memcpy(out.depth.data(), bytes + pixels * 4, pixels * 4);
    std::memcpy(out.instance.data(), bytes + pixels * 8, pixels * 4);
}

}  // namespace lagrange::gpu
