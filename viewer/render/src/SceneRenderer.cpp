#include "lagrange/gpu/Device.hpp"
#include "lagrange/viewer/SceneRenderer.hpp"
#include "shaders/kLineFragment.hpp"
#include "shaders/kLineVertex.hpp"
#include "shaders/kMeshFragment.hpp"
#include "shaders/kMeshVertex.hpp"

#include <array>
#include <cmath>
#include <cstring>

namespace lagrange::viewer {

namespace {

constexpr VkFormat kColorFormat = VK_FORMAT_R8G8B8A8_UNORM;
constexpr VkFormat kDepthFormat = VK_FORMAT_D32_SFLOAT;
constexpr int kGridHalfCells = 20;  // lines every metre, 20 m to each side
constexpr int kGridMajorEvery = 5;

struct MeshPush {
    Mat4 viewProjection;
    std::array<float, 12> model;
    std::array<float, 4> baseColor;
};
static_assert(sizeof(MeshPush) == 128, "must match MeshPush in mesh.slang");

struct LinePush {
    Mat4 viewProjection;
    std::array<float, 4> offset;
    std::array<float, 4> color;
};
static_assert(sizeof(LinePush) == 96, "must match LinePush in line.slang");

// Rows of the 3x4 matrix for rotation (unit quaternion w, x, y, z), scale and translation.
std::array<float, 12> modelRows(const Transform& t, const std::array<double, 3>& eye) {
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
        // Subtract in double, convert last: the GPU never sees a large coordinate.
        m.at(row * 4 + 3) = static_cast<float>(t.position.at(row) - eye.at(row));
    }
    return m;
}

struct PipelineSpec {
    std::span<const std::uint32_t> vertex;
    std::span<const std::uint32_t> fragment;
    VkPrimitiveTopology topology;
    std::uint32_t stride;
    bool hasNormal;
    std::uint32_t pushSize;
    bool depthWrite;
};

void buildPipeline(const gpu::Device& device, const PipelineSpec& spec, VkPipelineLayout& layout,
                   VkPipeline& pipeline) {
    VkPushConstantRange range{VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0,
                              spec.pushSize};
    VkPipelineLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    layoutInfo.pushConstantRangeCount = 1;
    layoutInfo.pPushConstantRanges = &range;
    vkCreatePipelineLayout(device.device(), &layoutInfo, nullptr, &layout);

    VkShaderModule vertex = gpu::createShaderModule(device, spec.vertex);
    VkShaderModule fragment = gpu::createShaderModule(device, spec.fragment);
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
    raster.polygonMode = VK_POLYGON_MODE_FILL;
    raster.cullMode = VK_CULL_MODE_NONE;  // imported meshes do not have reliable winding yet
    raster.lineWidth = 1.0F;
    VkPipelineMultisampleStateCreateInfo multisample{
        VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
    multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
    VkPipelineDepthStencilStateCreateInfo depth{
        VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
    depth.depthTestEnable = VK_TRUE;
    depth.depthWriteEnable = spec.depthWrite ? VK_TRUE : VK_FALSE;
    depth.depthCompareOp = VK_COMPARE_OP_GREATER_OR_EQUAL;  // reversed depth
    VkPipelineColorBlendAttachmentState blendAttachment{};
    blendAttachment.colorWriteMask = 0xF;
    blendAttachment.blendEnable = VK_TRUE;
    blendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
    blendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    blendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
    blendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    blendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    blendAttachment.alphaBlendOp = VK_BLEND_OP_MAX;
    VkPipelineColorBlendStateCreateInfo blend{
        VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    blend.attachmentCount = 1;
    blend.pAttachments = &blendAttachment;
    const std::array<VkDynamicState, 2> dynamicStates = {VK_DYNAMIC_STATE_VIEWPORT,
                                                         VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dynamic{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    dynamic.dynamicStateCount = static_cast<std::uint32_t>(dynamicStates.size());
    dynamic.pDynamicStates = dynamicStates.data();
    VkPipelineRenderingCreateInfo rendering{VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO};
    rendering.colorAttachmentCount = 1;
    rendering.pColorAttachmentFormats = &kColorFormat;
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

gpu::Buffer lineBuffer(const gpu::Device& device, const std::vector<float>& vertices) {
    return gpu::createBufferWithData(device, std::as_bytes(std::span{vertices}),
                                     VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
}

}  // namespace

SceneRenderer::SceneRenderer(const gpu::Device& device) : device_(device) {
    createPipelines();

    // Grid on the z = 0 plane, centred on its own origin; drawn snapped under the camera target.
    std::vector<float> minor;
    std::vector<float> major;
    const auto extent = static_cast<float>(kGridHalfCells);
    for (int i = -kGridHalfCells; i <= kGridHalfCells; ++i) {
        const auto v = static_cast<float>(i);
        std::vector<float>& lines = i % kGridMajorEvery == 0 ? major : minor;
        lines.insert(lines.end(), {v, -extent, 0, v, extent, 0, -extent, v, 0, extent, v, 0});
    }
    gridMinorCount_ = static_cast<std::uint32_t>(minor.size() / 3);
    gridMajorCount_ = static_cast<std::uint32_t>(major.size() / 3);
    gridMinor_ = lineBuffer(device_, minor);
    gridMajor_ = lineBuffer(device_, major);
    axes_ = lineBuffer(device_, {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1});
}

SceneRenderer::~SceneRenderer() {
    vkDeviceWaitIdle(device_.device());
    destroyMeshes();
    gpu::destroy(device_, gridMinor_);
    gpu::destroy(device_, gridMajor_);
    gpu::destroy(device_, axes_);
    gpu::destroy(device_, color_);
    gpu::destroy(device_, depth_);
    vkDestroyPipeline(device_.device(), meshPipeline_, nullptr);
    vkDestroyPipelineLayout(device_.device(), meshLayout_, nullptr);
    vkDestroyPipeline(device_.device(), linePipeline_, nullptr);
    vkDestroyPipelineLayout(device_.device(), lineLayout_, nullptr);
}

void SceneRenderer::createPipelines() {
    buildPipeline(device_,
                  {kMeshVertex, kMeshFragment, VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, 24, true,
                   sizeof(MeshPush), true},
                  meshLayout_, meshPipeline_);
    buildPipeline(device_,
                  {kLineVertex, kLineFragment, VK_PRIMITIVE_TOPOLOGY_LINE_LIST, 12, false,
                   sizeof(LinePush), false},
                  lineLayout_, linePipeline_);
}

bool SceneRenderer::resize(VkExtent2D extent) {
    extent.width = std::max<std::uint32_t>(extent.width, 1);
    extent.height = std::max<std::uint32_t>(extent.height, 1);
    if (color_.image != VK_NULL_HANDLE && color_.extent.width == extent.width &&
        color_.extent.height == extent.height) {
        return false;
    }
    vkDeviceWaitIdle(device_.device());
    gpu::destroy(device_, color_);
    gpu::destroy(device_, depth_);
    color_ = gpu::createImage2D(device_, extent, kColorFormat,
                                VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT |
                                    VK_IMAGE_USAGE_TRANSFER_SRC_BIT,
                                VK_IMAGE_ASPECT_COLOR_BIT);
    depth_ =
        gpu::createImage2D(device_, extent, kDepthFormat,
                           VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT, VK_IMAGE_ASPECT_DEPTH_BIT);
    colorInitialised_ = false;
    return true;
}

void SceneRenderer::destroyMeshes() {
    for (GpuMesh& mesh : meshes_) {
        gpu::destroy(device_, mesh.vertices);
        gpu::destroy(device_, mesh.indices);
    }
    meshes_.clear();
}

void SceneRenderer::uploadMeshes(const SceneStatics& statics) {
    vkDeviceWaitIdle(device_.device());
    destroyMeshes();
    meshes_.resize(statics.meshes.size());
    for (std::size_t i = 0; i < statics.meshes.size(); ++i) {
        const MeshData& source = statics.meshes[i];
        const std::size_t vertexCount = source.positions.size() / 3;
        if (vertexCount == 0 || source.indices.empty()) {
            continue;
        }
        std::vector<float> interleaved(vertexCount * 6);
        for (std::size_t v = 0; v < vertexCount; ++v) {
            std::memcpy(&interleaved[v * 6], &source.positions[v * 3], 12);
            if (source.normals.empty()) {
                interleaved[v * 6 + 5] = 1.0F;  // no normals: light it as if facing up
            } else {
                std::memcpy(&interleaved[v * 6 + 3], &source.normals[v * 3], 12);
            }
        }
        meshes_[i].vertices = gpu::createBufferWithData(
            device_, std::as_bytes(std::span{interleaved}), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
        meshes_[i].indices = gpu::createBufferWithData(
            device_, std::as_bytes(std::span{source.indices}), VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
        meshes_[i].indexCount = static_cast<std::uint32_t>(source.indices.size());
    }
    meshGeneration_ = statics.generation;
}

void SceneRenderer::render(VkCommandBuffer commands, const SceneFrame* frame, const Camera& camera,
                           const RenderOptions& options) {
    if (frame != nullptr && frame->statics && frame->statics->generation != meshGeneration_) {
        uploadMeshes(*frame->statics);
    }
    const VkExtent2D extent = color_.extent;
    const Mat4 viewProjection =
        camera.viewProjection(static_cast<double>(extent.width) / extent.height);
    const std::array<double, 3> eye = camera.eye();

    gpu::transitionImage(
        commands, color_.image, VK_IMAGE_ASPECT_COLOR_BIT,
        colorInitialised_ ? VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL : VK_IMAGE_LAYOUT_UNDEFINED,
        VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
    gpu::transitionImage(commands, depth_.image, VK_IMAGE_ASPECT_DEPTH_BIT,
                         VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL);

    VkRenderingAttachmentInfo colorAttachment{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
    colorAttachment.imageView = color_.view;
    colorAttachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    // Design (.vp): #091120. The target is UNORM and shaders encode for display, so the
    // clear colour is given in display values.
    colorAttachment.clearValue.color = {{9.0F / 255.0F, 17.0F / 255.0F, 32.0F / 255.0F, 1.0F}};
    VkRenderingAttachmentInfo depthAttachment{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
    depthAttachment.imageView = depth_.view;
    depthAttachment.imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
    depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depthAttachment.clearValue.depthStencil = {0.0F, 0};  // reversed depth: far is 0
    VkRenderingInfo rendering{VK_STRUCTURE_TYPE_RENDERING_INFO};
    rendering.renderArea = {{0, 0}, extent};
    rendering.layerCount = 1;
    rendering.colorAttachmentCount = 1;
    rendering.pColorAttachments = &colorAttachment;
    rendering.pDepthAttachment = &depthAttachment;
    vkCmdBeginRendering(commands, &rendering);

    const VkViewport viewport{
        0.0F, 0.0F, static_cast<float>(extent.width), static_cast<float>(extent.height),
        0.0F, 1.0F};
    const VkRect2D scissor{{0, 0}, extent};
    vkCmdSetViewport(commands, 0, 1, &viewport);
    vkCmdSetScissor(commands, 0, 1, &scissor);

    lastDrawCount_ = 0;
    const VkShaderStageFlags stages = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    const VkDeviceSize zero = 0;

    // --- meshes ---------------------------------------------------------------------------
    if (frame != nullptr && frame->statics) {
        vkCmdBindPipeline(commands, VK_PIPELINE_BIND_POINT_GRAPHICS, meshPipeline_);
        const SceneStatics& statics = *frame->statics;
        for (std::size_t i = 0; i < statics.prims.size(); ++i) {
            const PrimData& prim = statics.prims[i];
            if (!prim.visible || prim.mesh == kNone || prim.mesh >= meshes_.size() ||
                meshes_[prim.mesh].indexCount == 0) {
                continue;
            }
            MeshPush push{};
            push.viewProjection = viewProjection;
            push.model = modelRows(frame->transforms[i], eye);
            push.baseColor = prim.material != kNone ? statics.materials[prim.material].baseColor
                                                    : std::array<float, 4>{0.8F, 0.8F, 0.8F, 1.0F};
            vkCmdPushConstants(commands, meshLayout_, stages, 0, sizeof(push), &push);
            const GpuMesh& mesh = meshes_[prim.mesh];
            vkCmdBindVertexBuffers(commands, 0, 1, &mesh.vertices.buffer, &zero);
            vkCmdBindIndexBuffer(commands, mesh.indices.buffer, 0, VK_INDEX_TYPE_UINT32);
            vkCmdDrawIndexed(commands, mesh.indexCount, 1, 0, 0, 0);
            ++lastDrawCount_;
        }
    }

    // --- grid and axes --------------------------------------------------------------------
    vkCmdBindPipeline(commands, VK_PIPELINE_BIND_POINT_GRAPHICS, linePipeline_);
    const auto drawLines = [&](const gpu::Buffer& buffer, std::uint32_t count,
                               const std::array<double, 3>& origin,
                               const std::array<float, 4>& color) {
        LinePush push{};
        push.viewProjection = viewProjection;
        for (std::size_t k = 0; k < 3; ++k) {
            push.offset.at(k) = static_cast<float>(origin.at(k) - eye.at(k));
        }
        push.color = color;
        vkCmdPushConstants(commands, lineLayout_, stages, 0, sizeof(push), &push);
        vkCmdBindVertexBuffers(commands, 0, 1, &buffer.buffer, &zero);
        vkCmdDraw(commands, count, 1, 0, 0);
    };
    // Snap the grid to whole major cells under the target so that it appears endless.
    const double cell = kGridMajorEvery;
    const std::array<double, 3> gridOrigin{std::floor(camera.target[0] / cell) * cell,
                                           std::floor(camera.target[1] / cell) * cell, 0.002};
    // Grid in the design's line colours: #1B293D minor, #2A3F5C major.
    if (options.grid) {
        drawLines(gridMinor_, gridMinorCount_, gridOrigin, {0.106F, 0.161F, 0.239F, 1.0F});
        drawLines(gridMajor_, gridMajorCount_, gridOrigin, {0.165F, 0.247F, 0.361F, 1.0F});
    }
    const std::array<double, 3> worldOrigin{0.0, 0.0, 0.004};
    LinePush axisPush{};
    axisPush.viewProjection = viewProjection;
    for (std::size_t k = 0; k < 3; ++k) {
        axisPush.offset.at(k) = static_cast<float>(worldOrigin.at(k) - eye.at(k));
    }
    vkCmdBindVertexBuffers(commands, 0, 1, &axes_.buffer, &zero);
    const std::array<std::array<float, 4>, 3> axisColors = {{
        // Design tokens, color.axis.
        {0.941F, 0.376F, 0.365F, 1.0F},  // X #F0605D
        {0.310F, 0.788F, 0.478F, 1.0F},  // Y #4FC97A
        {0.353F, 0.635F, 1.000F, 1.0F},  // Z #5AA2FF
    }};
    for (std::uint32_t axis = 0; options.axes && axis < 3; ++axis) {
        axisPush.color = axisColors.at(axis);
        vkCmdPushConstants(commands, lineLayout_, stages, 0, sizeof(axisPush), &axisPush);
        vkCmdDraw(commands, 2, 1, axis * 2, 0);
    }

    vkCmdEndRendering(commands);
    gpu::transitionImage(commands, color_.image, VK_IMAGE_ASPECT_COLOR_BIT,
                         VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                         VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    colorInitialised_ = true;
}

}  // namespace lagrange::viewer
