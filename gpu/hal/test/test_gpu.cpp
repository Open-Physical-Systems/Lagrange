#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "lagrange/gpu/Device.hpp"
#include "lagrange/gpu/Resources.hpp"
#include "shaders/kTriangleFragment.hpp"
#include "shaders/kTriangleVertex.hpp"

namespace {

using namespace lagrange::gpu;

DeviceInfo hardware(std::uint64_t memoryGiB, bool rayQuery, bool accel) {
    DeviceInfo info;
    info.name = "test";
    info.apiMajor = 1;
    info.apiMinor = 3;
    info.deviceMemoryBytes = memoryGiB * 1024 * 1024 * 1024;
    info.rayQuery = rayQuery;
    info.accelerationStructure = accel;
    classify(info);
    return info;
}

TEST(Classify, SoftwareDeviceIsTier0WhateverItSupports) {
    DeviceInfo info = hardware(64, true, true);
    info.isSoftware = true;
    classify(info);
    EXPECT_EQ(info.tier, CapabilityTier::Tier0);
    EXPECT_TRUE(info.usable);
}

TEST(Classify, RayTracingWithEnoughMemoryIsTier2) {
    const DeviceInfo info = hardware(8, true, true);
    EXPECT_EQ(info.tier, CapabilityTier::Tier2);
    EXPECT_TRUE(info.reasons.empty());
}

TEST(Classify, MissingRayQueryOrAccelerationStructureIsTier1WithAReason) {
    EXPECT_EQ(hardware(8, false, true).tier, CapabilityTier::Tier1);
    EXPECT_EQ(hardware(8, true, false).tier, CapabilityTier::Tier1);
    EXPECT_FALSE(hardware(8, false, true).reasons.empty());
}

TEST(Classify, RayTracingWithLittleMemoryIsTier1) {
    const DeviceInfo info = hardware(4, true, true);
    EXPECT_EQ(info.tier, CapabilityTier::Tier1);
    ASSERT_EQ(info.reasons.size(), 1U);
    EXPECT_NE(info.reasons[0].find("4096 MiB"), std::string::npos);
}

TEST(Classify, OldVulkanIsNotUsable) {
    DeviceInfo info = hardware(8, true, true);
    info.apiMinor = 2;
    classify(info);
    EXPECT_FALSE(info.usable);
    EXPECT_NE(info.reasons.front().find("1.2"), std::string::npos);
}

std::unique_ptr<Device> makeDevice(DeviceDesc desc = {}) {
    desc.validation = true;
    std::string error;
    auto device = Device::create(desc, error);
    EXPECT_NE(device, nullptr) << error;
    return device;
}

TEST(Device, CreatesOnTheDefaultDeviceAndReportsItsTier) {
    const auto device = makeDevice();
    ASSERT_NE(device, nullptr);
    EXPECT_FALSE(device->info().name.empty());
    EXPECT_TRUE(device->info().usable);
    EXPECT_LE(device->tier(), device->info().tier);
    EXPECT_FALSE(device->allDevices().empty());
    std::printf("device: %s, %s\n", device->info().name.c_str(), toString(device->tier()));
}

TEST(Device, UnknownDeviceNameFailsWithAMessage) {
    DeviceDesc desc;
    desc.preferredDevice = "no-such-gpu-anywhere";
    std::string error;
    EXPECT_EQ(Device::create(desc, error), nullptr);
    EXPECT_NE(error.find("no-such-gpu-anywhere"), std::string::npos);
}

TEST(Device, TierCanBeLoweredButNotRaised) {
    const auto reference = makeDevice();
    ASSERT_NE(reference, nullptr);
    const CapabilityTier actual = reference->info().tier;

    DeviceDesc lower;
    lower.preferredDevice = reference->info().name;
    lower.maximumTier = CapabilityTier::Tier0;
    const auto lowered = makeDevice(lower);
    ASSERT_NE(lowered, nullptr);
    EXPECT_EQ(lowered->tier(), CapabilityTier::Tier0);
    EXPECT_EQ(lowered->info().tier, actual);

    if (actual != CapabilityTier::Tier2) {
        DeviceDesc higher;
        higher.preferredDevice = reference->info().name;
        higher.maximumTier = CapabilityTier::Tier2;
        std::string error;
        EXPECT_EQ(Device::create(higher, error), nullptr);
        EXPECT_NE(error.find("Tier 2"), std::string::npos);
    }
}

TEST(Device, RendersATriangleOffscreenAndReadsItBack) {
    const auto device = makeDevice();
    ASSERT_NE(device, nullptr);
    const VkExtent2D extent{64, 64};
    const VkFormat format = VK_FORMAT_R8G8B8A8_UNORM;
    Image target = createImage2D(
        *device, extent, format,
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT,
        VK_IMAGE_ASPECT_COLOR_BIT);

    VkShaderModule vertex = createShaderModule(*device, kTriangleVertex);
    VkShaderModule fragment = createShaderModule(*device, kTriangleFragment);
    VkPipelineShaderStageCreateInfo stages[2] = {};
    stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
    stages[0].module = vertex;
    stages[0].pName = "main";
    stages[1] = stages[0];
    stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    stages[1].module = fragment;

    VkPipelineLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    VkPipelineLayout layout = VK_NULL_HANDLE;
    ASSERT_EQ(vkCreatePipelineLayout(device->device(), &layoutInfo, nullptr, &layout), VK_SUCCESS);

    VkPipelineVertexInputStateCreateInfo vertexInput{
        VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    VkPipelineInputAssemblyStateCreateInfo assembly{
        VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    assembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    VkPipelineViewportStateCreateInfo viewportState{
        VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    viewportState.viewportCount = 1;
    viewportState.scissorCount = 1;
    VkPipelineRasterizationStateCreateInfo raster{
        VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    raster.polygonMode = VK_POLYGON_MODE_FILL;
    raster.cullMode = VK_CULL_MODE_NONE;
    raster.lineWidth = 1.0F;
    VkPipelineMultisampleStateCreateInfo multisample{
        VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
    multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
    VkPipelineColorBlendAttachmentState blendAttachment{};
    blendAttachment.colorWriteMask = 0xF;
    VkPipelineColorBlendStateCreateInfo blend{
        VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    blend.attachmentCount = 1;
    blend.pAttachments = &blendAttachment;
    const VkDynamicState dynamicStates[2] = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dynamic{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    dynamic.dynamicStateCount = 2;
    dynamic.pDynamicStates = dynamicStates;
    VkPipelineRenderingCreateInfo rendering{VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO};
    rendering.colorAttachmentCount = 1;
    rendering.pColorAttachmentFormats = &format;

    VkGraphicsPipelineCreateInfo pipelineInfo{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    pipelineInfo.pNext = &rendering;
    pipelineInfo.stageCount = 2;
    pipelineInfo.pStages = stages;
    pipelineInfo.pVertexInputState = &vertexInput;
    pipelineInfo.pInputAssemblyState = &assembly;
    pipelineInfo.pViewportState = &viewportState;
    pipelineInfo.pRasterizationState = &raster;
    pipelineInfo.pMultisampleState = &multisample;
    pipelineInfo.pColorBlendState = &blend;
    pipelineInfo.pDynamicState = &dynamic;
    pipelineInfo.layout = layout;
    VkPipeline pipeline = VK_NULL_HANDLE;
    ASSERT_EQ(vkCreateGraphicsPipelines(device->device(), VK_NULL_HANDLE, 1, &pipelineInfo,
                                        nullptr, &pipeline),
              VK_SUCCESS);

    device->submitAndWait([&](VkCommandBuffer commands) {
        transitionImage(commands, target.image, VK_IMAGE_ASPECT_COLOR_BIT,
                        VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
        VkRenderingAttachmentInfo color{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
        color.imageView = target.view;
        color.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        color.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        color.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        color.clearValue.color = {{0.0F, 0.0F, 1.0F, 1.0F}};  // blue background
        VkRenderingInfo info{VK_STRUCTURE_TYPE_RENDERING_INFO};
        info.renderArea = {{0, 0}, extent};
        info.layerCount = 1;
        info.colorAttachmentCount = 1;
        info.pColorAttachments = &color;
        vkCmdBeginRendering(commands, &info);
        const VkViewport viewport{0, 0, 64.0F, 64.0F, 0.0F, 1.0F};
        const VkRect2D scissor{{0, 0}, extent};
        vkCmdSetViewport(commands, 0, 1, &viewport);
        vkCmdSetScissor(commands, 0, 1, &scissor);
        vkCmdBindPipeline(commands, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
        vkCmdDraw(commands, 3, 1, 0, 0);
        vkCmdEndRendering(commands);
    });

    const std::vector<std::uint8_t> pixels =
        readbackRgba8(*device, target, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
    ASSERT_EQ(pixels.size(), 64U * 64U * 4U);
    const auto pixel = [&](std::uint32_t x, std::uint32_t y) { return &pixels[(y * 64 + x) * 4]; };
    EXPECT_EQ(pixel(32, 40)[0], 255);  // inside the triangle: red
    EXPECT_EQ(pixel(32, 40)[1], 0);
    EXPECT_EQ(pixel(32, 40)[2], 0);
    EXPECT_EQ(pixel(2, 2)[0], 0);  // corner: blue background
    EXPECT_EQ(pixel(2, 2)[2], 255);
    EXPECT_EQ(pixel(2, 2)[3], 255);

    const std::string path =
        (std::filesystem::temp_directory_path() / "lagrange_test_triangle.png").string();
    ASSERT_TRUE(writePng(path, 64, 64, pixels));
    std::ifstream file(path, std::ios::binary);
    char signature[8] = {};
    file.read(signature, 8);
    EXPECT_EQ(std::string(signature, 8), std::string("\x89PNG\r\n\x1a\n", 8));

    vkDestroyPipeline(device->device(), pipeline, nullptr);
    vkDestroyPipelineLayout(device->device(), layout, nullptr);
    vkDestroyShaderModule(device->device(), vertex, nullptr);
    vkDestroyShaderModule(device->device(), fragment, nullptr);
    destroy(*device, target);
    EXPECT_EQ(device->validationErrorCount(), 0U) << "Vulkan validation reported errors";
}

TEST(WritePng, RejectsWrongSizes) {
    const std::vector<std::uint8_t> pixels(16);
    EXPECT_FALSE(writePng("/tmp/never-written.png", 4, 4, pixels));  // needs 64 bytes
    EXPECT_FALSE(writePng("/tmp/never-written.png", 0, 0, {}));
    EXPECT_FALSE(writePng("/nonexistent-dir/x.png", 2, 2, pixels));
}

}  // namespace
