#include "lagrange/gpu/Compute.hpp"
#include "lagrange/gpu/Device.hpp"

#include <vector>

namespace lagrange::gpu {

ComputeKernel createComputeKernel(const Device& device, std::span<const std::uint32_t> spirv,
                                  std::uint32_t storageBuffers, std::uint32_t pushConstantBytes) {
    ComputeKernel kernel;
    kernel.storageBuffers = storageBuffers;
    kernel.pushConstantBytes = pushConstantBytes;
    const VkDevice vk = device.device();

    std::vector<VkDescriptorSetLayoutBinding> bindings(storageBuffers);
    for (std::uint32_t i = 0; i < storageBuffers; ++i) {
        bindings[i].binding = i;
        bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        bindings[i].descriptorCount = 1;
        bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    }
    VkDescriptorSetLayoutCreateInfo setInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    setInfo.bindingCount = storageBuffers;
    setInfo.pBindings = bindings.data();
    if (vkCreateDescriptorSetLayout(vk, &setInfo, nullptr, &kernel.setLayout) != VK_SUCCESS) {
        destroy(device, kernel);
        return kernel;
    }

    const VkPushConstantRange range{VK_SHADER_STAGE_COMPUTE_BIT, 0, pushConstantBytes};
    VkPipelineLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    layoutInfo.setLayoutCount = 1;
    layoutInfo.pSetLayouts = &kernel.setLayout;
    layoutInfo.pushConstantRangeCount = pushConstantBytes > 0 ? 1 : 0;
    layoutInfo.pPushConstantRanges = &range;
    if (vkCreatePipelineLayout(vk, &layoutInfo, nullptr, &kernel.layout) != VK_SUCCESS) {
        destroy(device, kernel);
        return kernel;
    }

    const VkShaderModule module = createShaderModule(device, spirv);
    VkComputePipelineCreateInfo pipelineInfo{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
    pipelineInfo.stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    pipelineInfo.stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    pipelineInfo.stage.module = module;
    pipelineInfo.stage.pName = "main";
    pipelineInfo.layout = kernel.layout;
    const VkResult made =
        vkCreateComputePipelines(vk, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &kernel.pipeline);
    vkDestroyShaderModule(vk, module, nullptr);
    if (made != VK_SUCCESS) {
        kernel.pipeline = VK_NULL_HANDLE;
        destroy(device, kernel);
        return kernel;
    }

    const VkDescriptorPoolSize size{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, storageBuffers};
    VkDescriptorPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    poolInfo.maxSets = 1;
    poolInfo.poolSizeCount = 1;
    poolInfo.pPoolSizes = &size;
    VkDescriptorSetAllocateInfo allocateInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
    allocateInfo.descriptorSetCount = 1;
    allocateInfo.pSetLayouts = &kernel.setLayout;
    if (vkCreateDescriptorPool(vk, &poolInfo, nullptr, &kernel.pool) != VK_SUCCESS ||
        (allocateInfo.descriptorPool = kernel.pool,
         vkAllocateDescriptorSets(vk, &allocateInfo, &kernel.set) != VK_SUCCESS)) {
        destroy(device, kernel);
    }
    return kernel;
}

void destroy(const Device& device, ComputeKernel& kernel) {
    const VkDevice vk = device.device();
    if (kernel.pool != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(vk, kernel.pool, nullptr);  // frees the set
    }
    if (kernel.pipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(vk, kernel.pipeline, nullptr);
    }
    if (kernel.layout != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(vk, kernel.layout, nullptr);
    }
    if (kernel.setLayout != VK_NULL_HANDLE) {
        vkDestroyDescriptorSetLayout(vk, kernel.setLayout, nullptr);
    }
    kernel = {};
}

void bindBuffers(const Device& device, const ComputeKernel& kernel,
                 std::span<const Buffer* const> buffers) {
    std::vector<VkDescriptorBufferInfo> infos(buffers.size());
    std::vector<VkWriteDescriptorSet> writes(buffers.size());
    for (std::size_t i = 0; i < buffers.size(); ++i) {
        infos[i] = {buffers[i]->buffer, 0, VK_WHOLE_SIZE};
        writes[i] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        writes[i].dstSet = kernel.set;
        writes[i].dstBinding = static_cast<std::uint32_t>(i);
        writes[i].descriptorCount = 1;
        writes[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[i].pBufferInfo = &infos[i];
    }
    vkUpdateDescriptorSets(device.device(), static_cast<std::uint32_t>(writes.size()),
                           writes.data(), 0, nullptr);
}

void recordDispatch(VkCommandBuffer commands, const ComputeKernel& kernel,
                    std::span<const std::byte> pushConstants, std::uint32_t groups) {
    vkCmdBindPipeline(commands, VK_PIPELINE_BIND_POINT_COMPUTE, kernel.pipeline);
    vkCmdBindDescriptorSets(commands, VK_PIPELINE_BIND_POINT_COMPUTE, kernel.layout, 0, 1,
                            &kernel.set, 0, nullptr);
    if (!pushConstants.empty()) {
        vkCmdPushConstants(commands, kernel.layout, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                           static_cast<std::uint32_t>(pushConstants.size()), pushConstants.data());
    }
    vkCmdDispatch(commands, groups, 1, 1);
    VkMemoryBarrier2 barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER_2};
    barrier.srcStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
    barrier.srcAccessMask = VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
    barrier.dstStageMask = VK_PIPELINE_STAGE_2_HOST_BIT;
    barrier.dstAccessMask = VK_ACCESS_2_HOST_READ_BIT;
    VkDependencyInfo dependency{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
    dependency.memoryBarrierCount = 1;
    dependency.pMemoryBarriers = &barrier;
    vkCmdPipelineBarrier2(commands, &dependency);
}

}  // namespace lagrange::gpu
