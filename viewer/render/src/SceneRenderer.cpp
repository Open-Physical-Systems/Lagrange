#include "lagrange/gpu/Device.hpp"
#include "lagrange/gpu/Raster.hpp"
#include "lagrange/viewer/SceneRenderer.hpp"
#include "shaders/kLineFragment.hpp"
#include "shaders/kLineVertex.hpp"
#include "shaders/kMeshFragment.hpp"
#include "shaders/kMeshUnlitFragment.hpp"
#include "shaders/kMeshVertex.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

namespace lagrange::viewer {

namespace {

constexpr VkFormat kColorFormat = VK_FORMAT_R8G8B8A8_UNORM;
using gpu::kDepthFormat;
constexpr int kGridHalfCells = 20;  // lines every metre, 20 m to each side
constexpr int kGridMajorEvery = 5;
constexpr float kNoTemperature = 0.02F;  // linear grey of meshes without a thermal node

// gpu::MeshPush, with the viewer's matrix type.
struct MeshPush {
    Mat4 viewProjection;
    std::array<float, 12> model;
    std::array<float, 4> baseColor;
};
static_assert(sizeof(MeshPush) == sizeof(gpu::MeshPush), "must match MeshPush in mesh.slang");

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

gpu::Buffer lineBuffer(const gpu::Device& device, const std::vector<float>& vertices) {
    return gpu::createBufferWithData(device, std::as_bytes(std::span{vertices}),
                                     VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
}

}  // namespace

std::array<float, 3> thermalColor(float t) {
    // Design tokens colormap.thermal ("inferno"), sRGB.
    static constexpr std::array<std::array<int, 3>, 6> kStops{{{0x00, 0x00, 0x04},
                                                               {0x42, 0x0A, 0x68},
                                                               {0x93, 0x26, 0x67},
                                                               {0xDD, 0x51, 0x3A},
                                                               {0xFC, 0xA5, 0x0A},
                                                               {0xFC, 0xFF, 0xA4}}};
    const float x = std::clamp(t, 0.0F, 1.0F) * static_cast<float>(kStops.size() - 1);
    const auto k = std::min(static_cast<std::size_t>(x), kStops.size() - 2);
    const float f = x - static_cast<float>(k);
    std::array<float, 3> out{};
    for (std::size_t c = 0; c < 3; ++c) {
        // Blend in sRGB, where the map's stops are even, then make it linear.
        const float srgb = (1.0F - f) * static_cast<float>(kStops[k][c]) +
                           f * static_cast<float>(kStops[k + 1][c]);
        out[c] = std::pow(srgb / 255.0F, 2.2F);
    }
    return out;
}

SceneRenderer::SceneRenderer(const gpu::Device& device) : device_(device), meshes_(device) {
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
    meshes_.clear();
    gpu::destroy(device_, gridMinor_);
    gpu::destroy(device_, gridMajor_);
    gpu::destroy(device_, axes_);
    gpu::destroy(device_, overlay_);
    gpu::destroy(device_, color_);
    gpu::destroy(device_, depth_);
    vkDestroyPipeline(device_.device(), meshPipeline_, nullptr);
    vkDestroyPipelineLayout(device_.device(), meshLayout_, nullptr);
    vkDestroyPipeline(device_.device(), unlitPipeline_, nullptr);
    vkDestroyPipelineLayout(device_.device(), unlitLayout_, nullptr);
    vkDestroyPipeline(device_.device(), linePipeline_, nullptr);
    vkDestroyPipelineLayout(device_.device(), lineLayout_, nullptr);
}

void SceneRenderer::createPipelines() {
    gpu::PipelineSpec mesh;
    mesh.vertex = kMeshVertex;
    mesh.fragment = kMeshFragment;
    mesh.pushSize = sizeof(MeshPush);
    mesh.colorFormats = std::span{&kColorFormat, 1};
    gpu::buildPipeline(device_, mesh, meshLayout_, meshPipeline_);
    gpu::PipelineSpec unlit = mesh;
    unlit.fragment = kMeshUnlitFragment;
    gpu::buildPipeline(device_, unlit, unlitLayout_, unlitPipeline_);
    gpu::PipelineSpec line = mesh;
    line.vertex = kLineVertex;
    line.fragment = kLineFragment;
    line.topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;
    line.stride = 12;
    line.hasNormal = false;
    line.pushSize = sizeof(LinePush);
    line.depthWrite = false;
    gpu::buildPipeline(device_, line, lineLayout_, linePipeline_);
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

void SceneRenderer::uploadMeshes(const SceneStatics& statics) {
    vkDeviceWaitIdle(device_.device());
    std::vector<gpu::MeshSource> sources;
    sources.reserve(statics.meshes.size());
    for (const MeshData& mesh : statics.meshes) {
        sources.push_back({mesh.positions, mesh.normals, mesh.indices});
    }
    meshes_.upload(sources);
    meshGeneration_ = statics.generation;
}

void SceneRenderer::render(VkCommandBuffer commands, const SceneFrame* frame, const Camera& camera,
                           const RenderOptions& options, std::span<const OverlayLines> overlays) {
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
        const bool lit = options.shading == Shading::Lit;
        vkCmdBindPipeline(commands, VK_PIPELINE_BIND_POINT_GRAPHICS,
                          lit ? meshPipeline_ : unlitPipeline_);
        const VkPipelineLayout layout = lit ? meshLayout_ : unlitLayout_;
        const SceneStatics& statics = *frame->statics;
        for (std::size_t i = 0; i < statics.prims.size(); ++i) {
            const PrimData& prim = statics.prims[i];
            if (!prim.visible || prim.mesh == kNone) {
                continue;
            }
            MeshPush push{};
            push.viewProjection = viewProjection;
            push.model = modelRows(frame->transforms[i], eye);
            push.baseColor = prim.material != kNone ? statics.materials[prim.material].baseColor
                                                    : std::array<float, 4>{0.8F, 0.8F, 0.8F, 1.0F};
            if (options.shading == Shading::Thermal) {
                const float t = i < options.temperature.size() ? options.temperature[i] : NAN;
                if (std::isnan(t)) {
                    push.baseColor = {kNoTemperature, kNoTemperature, kNoTemperature, 1.0F};
                } else {
                    const float span =
                        std::max(options.temperatureHigh - options.temperatureLow, 1e-3F);
                    const std::array<float, 3> rgb =
                        thermalColor((t - options.temperatureLow) / span);
                    push.baseColor = {rgb[0], rgb[1], rgb[2], 1.0F};
                }
            }
            vkCmdPushConstants(commands, layout, stages, 0, sizeof(push), &push);
            if (meshes_.draw(commands, prim.mesh)) {
                ++lastDrawCount_;
            }
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

    // --- overlays -------------------------------------------------------------------------
    std::size_t floats = 0;
    for (const OverlayLines& batch : overlays) {
        floats += batch.vertices.size();
    }
    if (floats > 0) {
        const VkDeviceSize bytes = floats * sizeof(float);
        if (overlay_.size < bytes) {
            gpu::destroy(device_, overlay_);  // the previous frame has finished (see header)
            overlay_ = gpu::createBuffer(device_, std::max<VkDeviceSize>(bytes * 2, 64 * 1024),
                                         VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, true);
        }
        auto* mapped = static_cast<float*>(overlay_.mapped);
        std::size_t written = 0;
        for (const OverlayLines& batch : overlays) {
            std::copy(batch.vertices.begin(), batch.vertices.end(), mapped + written);
            written += batch.vertices.size();
        }
        gpu::flush(device_, overlay_);
        std::size_t first = 0;
        for (const OverlayLines& batch : overlays) {
            LinePush push{};
            push.viewProjection = viewProjection;
            for (std::size_t k = 0; k < 3; ++k) {
                push.offset.at(k) = static_cast<float>(batch.origin.at(k) - eye.at(k));
            }
            push.color = batch.color;
            vkCmdPushConstants(commands, lineLayout_, stages, 0, sizeof(push), &push);
            const VkDeviceSize offset = first * sizeof(float);
            vkCmdBindVertexBuffers(commands, 0, 1, &overlay_.buffer, &offset);
            vkCmdDraw(commands, static_cast<std::uint32_t>(batch.vertices.size() / 3), 1, 0, 0);
            ++lastDrawCount_;
            first += batch.vertices.size();
        }
    }

    vkCmdEndRendering(commands);
    gpu::transitionImage(commands, color_.image, VK_IMAGE_ASPECT_COLOR_BIT,
                         VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                         VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    colorInitialised_ = true;
}

}  // namespace lagrange::viewer
