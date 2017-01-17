/* Copyright (c) 2016-2017, NVIDIA CORPORATION. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*  * Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*  * Neither the name of NVIDIA CORPORATION nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
* EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
* PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
* CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
* EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
* PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
* OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <vkhlf/Buffer.h>
#include <vkhlf/BufferView.h>
#include <vkhlf/CommandBuffer.h>
#include <vkhlf/CommandPool.h>
#include <vkhlf/DescriptorPool.h>
#include <vkhlf/DescriptorSet.h>
#include <vkhlf/Device.h>
#include <vkhlf/DeviceMemoryAllocator.h>
#include <vkhlf/Event.h>
#include <vkhlf/Fence.h>
#include <vkhlf/Framebuffer.h>
#include <vkhlf/Image.h>
#include <vkhlf/PipelineCache.h>
#include <vkhlf/PhysicalDevice.h>
#include <vkhlf/QueryPool.h>
#include <vkhlf/Queue.h>
#include <vkhlf/RenderPass.h>
#include <vkhlf/Semaphore.h>
#include <vkhlf/Swapchain.h>
#include <vector>

namespace vkhlf
{

  DeviceQueueCreateInfo::DeviceQueueCreateInfo(uint32_t queueFamilyIndex_, vk::ArrayProxy<const float> queuePriorities_)
    : queueFamilyIndex(queueFamilyIndex_)
    , queuePriorities(queuePriorities_.begin(), queuePriorities_.end())
  {}


  DeviceQueueCreateInfo::DeviceQueueCreateInfo(DeviceQueueCreateInfo const& rhs)
    : DeviceQueueCreateInfo(rhs.queueFamilyIndex, rhs.queuePriorities)
  {}

  DeviceQueueCreateInfo & DeviceQueueCreateInfo::operator = (DeviceQueueCreateInfo const& rhs)
  {
    queueFamilyIndex  = rhs.queueFamilyIndex;
    queuePriorities   = rhs.queuePriorities;
    return *this;
  }

  Device::Device(std::shared_ptr<PhysicalDevice> const& physicalDevice, vk::ArrayProxy<const vkhlf::DeviceQueueCreateInfo> queueCreateInfos, vk::ArrayProxy<const std::string> enabledLayerNames,
                 vk::ArrayProxy<const std::string> enabledExtensionNames, vk::PhysicalDeviceFeatures const& enabledFeatures, std::shared_ptr<Allocator> const& allocator)
    : Reference(physicalDevice, allocator)
  {
#if !defined(NDEBUG)
    std::vector<vk::QueueFamilyProperties> queueFamilyProperties = get<PhysicalDevice>()->getQueueFamilyProperties();
    assert(queueCreateInfos.size() <= queueFamilyProperties.size());
#endif

    std::vector<vk::DeviceQueueCreateInfo> queueCIs;
    queueCIs.reserve(queueCreateInfos.size());
    for (auto const& ci : queueCreateInfos)
    {
      queueCIs.push_back(vk::DeviceQueueCreateInfo({}, ci.queueFamilyIndex, vkhlf::checked_cast<uint32_t>(ci.queuePriorities.size()), ci.queuePriorities.data()));
    }

    std::vector<char const*> layers;
    layers.reserve(enabledLayerNames.size());
    for (auto const& s : enabledLayerNames)
    {
      layers.push_back(s.c_str());
    }

    std::vector<char const*> extensions;
    extensions.reserve(enabledExtensionNames.size());
    for (auto const& s : enabledExtensionNames)
    {
      extensions.push_back(s.c_str());
    }

    vk::DeviceCreateInfo createInfo({}, vkhlf::checked_cast<uint32_t>(queueCIs.size()), queueCIs.data(), vkhlf::checked_cast<uint32_t>(layers.size()), layers.data(),
                                    vkhlf::checked_cast<uint32_t>(extensions.size()), extensions.data());
    m_device = static_cast<vk::PhysicalDevice>(*get<PhysicalDevice>()).createDevice(createInfo, *get<Allocator>());

    m_queues.resize(createInfo.queueCreateInfoCount);
    for (uint32_t i = 0; i<createInfo.queueCreateInfoCount; i++)
    {
      m_queues[i].resize(createInfo.pQueueCreateInfos[i].queueCount);
    }
  }

  Device::~Device( )
  {
    m_queues.clear();
    m_device.destroy(*get<Allocator>());
  }

  PFN_vkVoidFunction Device::getProcAddress(std::string const& name) const
  {
    assert(!name.empty());
    return m_device.getProcAddr(name);
  }

  std::shared_ptr<vkhlf::Queue> Device::getQueue(uint32_t familyIndex, uint32_t queueIndex)
  {
    assert((familyIndex < m_queues.size()) && (queueIndex<m_queues[familyIndex].size()));

    std::shared_ptr<vkhlf::Queue> queue = m_queues[familyIndex][queueIndex].lock();
    if (!queue)
    {
      queue = std::make_shared<vkhlf::Queue>(shared_from_this(), familyIndex, queueIndex);
      m_queues[familyIndex][queueIndex] = queue;
    }
    return queue;
  }

  size_t Device::getQueueCount(uint32_t familyIndex) const
  {
    assert(familyIndex < m_queues.size());
    return m_queues[familyIndex].size();
  }

  size_t Device::getQueueFamilyCount() const
  {
    return m_queues.size();
  }

  vk::Result Device::waitForFences(vk::ArrayProxy<const std::shared_ptr<Fence>> fences, bool waitAll, uint32_t timeout) const
  {
    std::vector<vk::Fence> vkFences;
    vkFences.reserve(fences.size());
    for (auto const& f : fences)
    {
      vkFences.push_back(*f);
    }
    return m_device.waitForFences(vkFences, waitAll, timeout);
  }

  void Device::waitIdle() const
  {
    m_device.waitIdle();
  }

  std::shared_ptr<vkhlf::Image> Device::createImage(vk::ImageCreateFlags createFlags, vk::ImageType type, vk::Format format, vk::Extent3D const& extent, uint32_t mipLevels, uint32_t arraySize,
                                                  vk::SampleCountFlagBits samples, vk::ImageTiling tiling, vk::ImageUsageFlags usageFlags, vk::SharingMode sharingMode,
                                                  std::vector<uint32_t> const& queueFamilyIndices, vk::ImageLayout initialLayout, vk::MemoryPropertyFlags memoryPropertyFlags,
                                                  std::shared_ptr<DeviceMemoryAllocator> const& deviceMemoryAllocator, std::shared_ptr<Allocator> const& imageAllocator)
  {
    return std::make_shared<Image>(shared_from_this(), createFlags, type, format, extent, mipLevels, arraySize, samples, tiling, usageFlags, sharingMode, queueFamilyIndices, initialLayout,
                                   memoryPropertyFlags, deviceMemoryAllocator, imageAllocator);
  }

  std::shared_ptr<vkhlf::Buffer> Device::createBuffer(vk::BufferCreateFlags createFlags, vk::DeviceSize size, vk::BufferUsageFlags usageFlags, vk::SharingMode sharingMode,
                                                    vk::ArrayProxy<const uint32_t> queueFamilyIndices, vk::MemoryPropertyFlags memoryPropertyFlags,
                                                    std::shared_ptr<DeviceMemoryAllocator> const& deviceMemoryAllocator, std::shared_ptr<Allocator> const& bufferAllocator)
  {
    return std::make_shared<Buffer>(shared_from_this(), createFlags, size, usageFlags, sharingMode, queueFamilyIndices, memoryPropertyFlags, deviceMemoryAllocator, bufferAllocator);
  }

  std::shared_ptr<vkhlf::Buffer> Device::createBuffer(vk::DeviceSize size, vk::BufferUsageFlags usageFlags, vk::SharingMode sharingMode, vk::ArrayProxy<const uint32_t> queueFamilyIndices,
                                                    vk::MemoryPropertyFlags memoryPropertyFlags, std::shared_ptr<DeviceMemoryAllocator> const& deviceMemoryAllocator,
                                                    std::shared_ptr<Allocator> const& bufferAllocator)
  {
    return std::make_shared<Buffer>(shared_from_this(), vk::BufferCreateFlags(), size, usageFlags, sharingMode, queueFamilyIndices, memoryPropertyFlags, deviceMemoryAllocator, bufferAllocator);
  }

  std::shared_ptr<vkhlf::CommandPool> Device::createCommandPool(vk::CommandPoolCreateFlags flags, uint32_t familyIndex, std::shared_ptr<Allocator> const& allocator)
  {
    return std::make_shared<CommandPool>(shared_from_this(), flags, familyIndex, allocator);
  }

  std::shared_ptr<vkhlf::DescriptorPool> Device::createDescriptorPool(vk::DescriptorPoolCreateFlags flags, uint32_t maxSets, vk::ArrayProxy<const vk::DescriptorPoolSize> poolSizes,
                                                                    std::shared_ptr<Allocator> const& allocator)
  {
    return std::make_shared<DescriptorPool>(shared_from_this(), flags, maxSets, poolSizes, allocator);
  }

  std::shared_ptr<vkhlf::DescriptorSet> Device::allocateDescriptorSet(std::shared_ptr<vkhlf::DescriptorPool> const& pool, std::shared_ptr<vkhlf::DescriptorSetLayout> const& layout)
  {
    return std::make_shared<DescriptorSet>(shared_from_this(), pool, layout);
  }

  void Device::updateDescriptorSets(vk::ArrayProxy<const WriteDescriptorSet> descriptorWrites, vk::ArrayProxy<const CopyDescriptorSet> descriptorCopies)
  {
    std::vector<std::unique_ptr<vk::DescriptorImageInfo>> diis;
    diis.reserve(descriptorWrites.size());

    std::vector<std::unique_ptr<vk::DescriptorBufferInfo>> dbis;
    dbis.reserve(descriptorWrites.size());

    std::vector<vk::WriteDescriptorSet> writes;
    writes.reserve(descriptorWrites.size());
    for (auto const& w : descriptorWrites)
    {
      diis.push_back(std::unique_ptr<vk::DescriptorImageInfo>(w.imageInfo ? new vk::DescriptorImageInfo(w.imageInfo->sampler ? static_cast<vk::Sampler>(*w.imageInfo->sampler) : nullptr,
                                                                                                        w.imageInfo->imageView ? static_cast<vk::ImageView>(*w.imageInfo->imageView) : nullptr,
                                                                                                        w.imageInfo->imageLayout)
                                                                          : nullptr));
      dbis.push_back(std::unique_ptr<vk::DescriptorBufferInfo>(w.bufferInfo ? new vk::DescriptorBufferInfo(w.bufferInfo->buffer ? static_cast<vk::Buffer>(*w.bufferInfo->buffer) : nullptr,
                                                                                                           w.bufferInfo->offset, w.bufferInfo->range)
                                                                            : nullptr));
      vk::WriteDescriptorSet write(w.dstSet ? static_cast<vk::DescriptorSet>(*w.dstSet) : nullptr, w.dstBinding, w.dstArrayElement, w.descriptorCount, w.descriptorType, diis.back().get(), dbis.back().get());
      if (w.texelBufferView)
      {
        vk::BufferView bufferView = static_cast<vk::BufferView>(*w.texelBufferView);
        write.setPTexelBufferView(&bufferView);
      }
      writes.push_back(std::move(write));
    }

    std::vector<vk::CopyDescriptorSet> copies;
    copies.reserve(descriptorCopies.size());
    for (auto const& c : descriptorCopies)
    {
      copies.push_back(vk::CopyDescriptorSet(c.srcSet ? static_cast<vk::DescriptorSet>(*c.srcSet) : nullptr, c.srcBinding, c.srcArrayElement,
                                             c.dstSet ? static_cast<vk::DescriptorSet>(*c.dstSet) : nullptr, c.dstBinding, c.dstArrayElement, c.descriptorCount));
    }

    m_device.updateDescriptorSets(writes, copies);
  }

  std::shared_ptr<vkhlf::DescriptorSetLayout> Device::createDescriptorSetLayout(vk::ArrayProxy<const DescriptorSetLayoutBinding> bindings, std::shared_ptr<Allocator> const& allocator)
  {
    return std::make_shared<DescriptorSetLayout>(shared_from_this(), bindings, allocator);
  }

  std::shared_ptr<vkhlf::DeviceMemory> Device::allocateMemory(vk::DeviceSize allocationSize, uint32_t memoryTypeIndex, std::shared_ptr<DeviceMemoryAllocator> const& deviceMemoryAllocator)
  {
    assert(deviceMemoryAllocator);
    return deviceMemoryAllocator->allocate(allocationSize, memoryTypeIndex);
  }

  std::shared_ptr<vkhlf::Event> Device::createEvent(std::shared_ptr<Allocator> const& allocator)
  {
    return std::make_shared<Event>(shared_from_this(), allocator);
  }

  std::shared_ptr<vkhlf::Fence> Device::createFence(bool signaled, std::shared_ptr<Allocator> const& allocator)
  {
    return std::make_shared<Fence>(shared_from_this(), signaled, allocator);
  }

  std::shared_ptr<vkhlf::QueryPool> Device::createOcclusionQuery(uint32_t entryCount, std::shared_ptr<Allocator> const& allocator)
  {
    return std::make_shared<QueryPool>(shared_from_this(), vk::QueryPoolCreateFlags(), vk::QueryType::eOcclusion, entryCount, vk::QueryPipelineStatisticFlags(), allocator);
  }

  std::shared_ptr<vkhlf::QueryPool> Device::createPipelineStatisticsQuery(uint32_t entryCount, vk::QueryPipelineStatisticFlags flags, std::shared_ptr<Allocator> const& allocator)
  {
    return std::make_shared<QueryPool>(shared_from_this(), vk::QueryPoolCreateFlags(), vk::QueryType::ePipelineStatistics, entryCount, flags, allocator);
  }

  std::shared_ptr<vkhlf::Sampler> Device::createSampler(vk::Filter magFilter, vk::Filter minFilter, vk::SamplerMipmapMode mipmapMode, vk::SamplerAddressMode addressModeU,
                                                      vk::SamplerAddressMode addressModeV, vk::SamplerAddressMode addressModeW, float mipLodBias, bool anisotropyEnable, float maxAnisotropy,
                                                      bool compareEnable, vk::CompareOp compareOp, float minLod, float maxLod, vk::BorderColor borderColor, bool unnormalizedCoordinates,
                                                      std::shared_ptr<Allocator> const& allocator)
  {
    return std::make_shared<Sampler>(shared_from_this(), magFilter, minFilter, mipmapMode, addressModeU, addressModeV, addressModeW, mipLodBias, anisotropyEnable, maxAnisotropy, compareEnable,
                                     compareOp, minLod, maxLod, borderColor, unnormalizedCoordinates, allocator);
  }

  std::shared_ptr<vkhlf::Semaphore> Device::createSemaphore(std::shared_ptr<Allocator> const& allocator)
  {
    return std::make_shared<Semaphore>(shared_from_this(), allocator);
  }

  std::shared_ptr<vkhlf::ShaderModule> Device::createShaderModule(vk::ArrayProxy<const uint32_t> code, std::shared_ptr<Allocator> const& allocator)
  {
    return std::make_shared<ShaderModule>(shared_from_this(), code, allocator);
  }

  std::shared_ptr<vkhlf::RenderPass> Device::createRenderPass(vk::ArrayProxy<const vk::AttachmentDescription> attachments, vk::ArrayProxy<const vk::SubpassDescription> subpasses,
                                                            vk::ArrayProxy<const vk::SubpassDependency> dependencies, std::shared_ptr<Allocator> const& allocator)
  {
    return std::make_shared<RenderPass>(shared_from_this(), attachments, subpasses, dependencies, allocator);
  }

  std::shared_ptr<vkhlf::Pipeline> Device::createComputePipeline(std::shared_ptr<vkhlf::PipelineCache> const & pipelineCache, vk::PipelineCreateFlags flags, PipelineShaderStageCreateInfo const& stage,
                                                               std::shared_ptr<PipelineLayout> const& layout, std::shared_ptr<Pipeline> const& basePipelineHandle, int32_t basePipelineIndex,
                                                               std::shared_ptr<Allocator> const& allocator)
  {
    return std::make_shared<ComputePipeline>(shared_from_this(), pipelineCache, flags, stage, layout, basePipelineHandle, basePipelineIndex, allocator);
  }

  std::shared_ptr<vkhlf::Pipeline> Device::createGraphicsPipeline(std::shared_ptr<PipelineCache> const& pipelineCache, vk::PipelineCreateFlags flags,
                                                                vk::ArrayProxy<const PipelineShaderStageCreateInfo> stages, vk::Optional<const PipelineVertexInputStateCreateInfo> vertexInputState,
                                                                vk::Optional<const vk::PipelineInputAssemblyStateCreateInfo> inputAssemblyState,
                                                                vk::Optional<const vk::PipelineTessellationStateCreateInfo> tessellationState,
                                                                vk::Optional<const PipelineViewportStateCreateInfo> viewportState,
                                                                vk::Optional<const vk::PipelineRasterizationStateCreateInfo> rasterizationState,
                                                                vk::Optional<const PipelineMultisampleStateCreateInfo> multisampleState,
                                                                vk::Optional<const vk::PipelineDepthStencilStateCreateInfo> depthStencilState,
                                                                vk::Optional<const PipelineColorBlendStateCreateInfo> colorBlendState,
                                                                vk::Optional<const PipelineDynamicStateCreateInfo> dynamicState, std::shared_ptr<PipelineLayout> const& pipelineLayout,
                                                                std::shared_ptr<RenderPass> const& renderPass, uint32_t subpass, std::shared_ptr<Pipeline> const& basePipelineHandle,
                                                                uint32_t basePipelineIndex, std::shared_ptr<Allocator> const& allocator)
  {
    return std::make_shared<GraphicsPipeline>(shared_from_this(), pipelineCache, flags, stages, vertexInputState, inputAssemblyState, tessellationState, viewportState, rasterizationState,
                                              multisampleState, depthStencilState, colorBlendState, dynamicState, pipelineLayout, renderPass, subpass, basePipelineHandle, basePipelineIndex,
                                              allocator);
  }

  std::shared_ptr<vkhlf::PipelineCache> Device::createPipelineCache(size_t initialSize, void const* initialData, std::shared_ptr<Allocator> const& allocator)
  {
    return std::make_shared<PipelineCache>(shared_from_this(), vk::PipelineCacheCreateFlags(), initialSize, initialData, allocator);
  }

  std::shared_ptr<PipelineLayout> Device::createPipelineLayout(vk::ArrayProxy<const std::shared_ptr<DescriptorSetLayout>> setLayouts, vk::ArrayProxy<const vk::PushConstantRange> pushConstantRanges,
                                                               std::shared_ptr<Allocator> const& allocator)
  {
    return std::make_shared<PipelineLayout>(shared_from_this(), setLayouts, pushConstantRanges, allocator);
  }

  std::shared_ptr<vkhlf::Framebuffer> Device::createFramebuffer(std::shared_ptr<vkhlf::RenderPass> const & renderPass, vk::ArrayProxy<const std::shared_ptr<vkhlf::ImageView>> attachments,
                                                              vk::Extent2D const& extent, uint32_t layers, std::shared_ptr<Allocator> const& allocator)
  {
    return std::make_shared<Framebuffer>(shared_from_this(), renderPass, attachments, extent, layers, allocator);
  }

  std::shared_ptr<Swapchain> Device::createSwapchain(std::shared_ptr<Surface> const& surface, uint32_t minImageCount, vk::Format imageFormat, vk::Extent2D const& extent, uint32_t imageArrayLayers,
                                                     vk::ImageUsageFlags imageUsage, vk::SharingMode imageSharingMode, vk::ArrayProxy<const uint32_t> queueFamilyIndices,
                                                     vk::SurfaceTransformFlagBitsKHR preTransform, vk::CompositeAlphaFlagBitsKHR compositeAlpha, vk::PresentModeKHR presentMode, bool clipped,
                                                     std::shared_ptr<Swapchain> const& oldSwapchain, std::shared_ptr<Allocator> const& allocator)
  {
    return std::make_shared<Swapchain>(shared_from_this(), surface, minImageCount, imageFormat, extent, imageArrayLayers, imageUsage, imageSharingMode, queueFamilyIndices, preTransform,
                                       compositeAlpha, presentMode, clipped, oldSwapchain, allocator);
  }

  FramebufferData::FramebufferData(std::shared_ptr<Device> const& device, std::shared_ptr<Surface> const& surface, vk::Format depthFormat, std::shared_ptr<CommandBuffer> const& commandBuffer,
                                   std::shared_ptr<RenderPass> const& renderPass, std::shared_ptr<DeviceMemoryAllocator> const& deviceMemoryAllocator, std::shared_ptr<Allocator> const& swapchainAllocator,
                                   std::shared_ptr<Allocator> const& imageAllocator, std::shared_ptr<Allocator> const& imageViewAllocator)
  {
    vk::SurfaceCapabilitiesKHR surfaceCapabilities = device->get<PhysicalDevice>()->getSurfaceCapabilities(surface);
    assert(surfaceCapabilities.currentExtent.width != -1);
    m_extent = surfaceCapabilities.currentExtent;

    std::vector<vk::PresentModeKHR> presentModes = device->get<PhysicalDevice>()->getSurfacePresentModes(surface);

    // If mailbox mode is available, use it, as is the lowest-latency non-tearing mode.  If not, try IMMEDIATE which will usually be available,
    // and is fastest (though it tears).  If not, fall back to FIFO which is always available.
    vk::PresentModeKHR swapchainPresentMode = vk::PresentModeKHR::eFifo;
#ifndef __ANDROID__
    for (size_t i = 0; i < presentModes.size(); i++)
    {
      if (presentModes[i] == vk::PresentModeKHR::eMailbox)
      {
        swapchainPresentMode = vk::PresentModeKHR::eMailbox;
        break;
      }
      else if ((swapchainPresentMode != vk::PresentModeKHR::eMailbox) && (presentModes[i] == vk::PresentModeKHR::eImmediate))
      {
        swapchainPresentMode = vk::PresentModeKHR::eImmediate;
      }
    }
#endif

    // Determine the number of images to use in the swap chain (we desire to own only 1 image at a time, besides the images being displayed and queued for display):
    uint32_t desiredNumberOfSwapChainImages = surfaceCapabilities.minImageCount + 1;
    if ((0 < surfaceCapabilities.maxImageCount) && (surfaceCapabilities.maxImageCount < desiredNumberOfSwapChainImages))
    {
      // Application must settle for fewer images than desired:
      desiredNumberOfSwapChainImages = surfaceCapabilities.maxImageCount;
    }

    std::vector<vk::SurfaceFormatKHR> surfaceFormats = device->get<PhysicalDevice>()->getSurfaceFormats(surface);
    assert(!surfaceFormats.empty());

    // If the format list includes just one entry of VK_FORMAT_UNDEFINED, the surface has no preferred format.  Otherwise, at least one supported format will be returned.
    vk::Format surfaceFormat = ((surfaceFormats.size() == 1) && (surfaceFormats[0].format == vk::Format::eUndefined)) ? vk::Format::eB8G8R8A8Unorm : surfaceFormats[0].format;

#ifndef __ANDROID__
    bool clipped = true;
#else
    bool clipped = false;
#endif

    vk::SurfaceTransformFlagBitsKHR preTransform = (surfaceCapabilities.supportedTransforms & vk::SurfaceTransformFlagBitsKHR::eIdentity)
                                                 ? vk::SurfaceTransformFlagBitsKHR::eIdentity
                                                 : surfaceCapabilities.currentTransform;

    m_swapchain = device->createSwapchain(surface, desiredNumberOfSwapChainImages, surfaceFormat, m_extent, 1,
                                          vk::ImageUsageFlagBits::eColorAttachment | vk::ImageUsageFlagBits::eTransferSrc, vk::SharingMode::eExclusive, {}, preTransform,
                                          vk::CompositeAlphaFlagBitsKHR::eOpaque, swapchainPresentMode, clipped, nullptr, swapchainAllocator);

    m_colorImages = m_swapchain->getImages();

    m_colorViews.reserve(m_colorImages.size());
    for (size_t i = 0; i < m_colorImages.size(); i++)
    {
      m_colorViews.push_back(m_colorImages[i]->createImageView(vk::ImageViewType::e2D, surfaceFormat,
                             {vk::ComponentSwizzle::eR, vk::ComponentSwizzle::eG, vk::ComponentSwizzle::eB, vk::ComponentSwizzle::eA}, {vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1}, imageViewAllocator));
    }

    // depth buffer
    // assert that a depth format is requested
    assert((depthFormat == vk::Format::eD16Unorm) ||
           (depthFormat == vk::Format::eD16UnormS8Uint) ||
           (depthFormat == vk::Format::eD24UnormS8Uint) ||
           (depthFormat == vk::Format::eD32Sfloat) ||
           (depthFormat == vk::Format::eD32SfloatS8Uint));

    vk::FormatProperties formatProperties = device->get<PhysicalDevice>()->getFormatProperties(depthFormat);
    assert((formatProperties.linearTilingFeatures & vk::FormatFeatureFlagBits::eDepthStencilAttachment) ||
           (formatProperties.optimalTilingFeatures & vk::FormatFeatureFlagBits::eDepthStencilAttachment));

    vk::ImageTiling tiling = (formatProperties.linearTilingFeatures & vk::FormatFeatureFlagBits::eDepthStencilAttachment) ? vk::ImageTiling::eLinear : vk::ImageTiling::eOptimal;

    m_depthImage = device->createImage({}, vk::ImageType::e2D, depthFormat, vk::Extent3D(m_extent.width, m_extent.height, 1), 1, 1, vk::SampleCountFlagBits::e1, tiling,
                                       vk::ImageUsageFlagBits::eDepthStencilAttachment, vk::SharingMode::eExclusive, {}, vk::ImageLayout::eUndefined, {} /* No requirements */, deviceMemoryAllocator,
                                       imageAllocator);

    vk::ImageAspectFlags aspectMask = vk::ImageAspectFlagBits::eDepth;
    if ((depthFormat == vk::Format::eD16UnormS8Uint) ||
        (depthFormat == vk::Format::eD24UnormS8Uint) ||
        (depthFormat == vk::Format::eD32SfloatS8Uint))
    {
      aspectMask |= vk::ImageAspectFlagBits::eStencil;
    }

    commandBuffer->begin();
    setImageLayout(commandBuffer, m_depthImage, aspectMask, vk::ImageLayout::eUndefined, vk::ImageLayout::eDepthStencilAttachmentOptimal);
    commandBuffer->end();

    m_depthView = m_depthImage->createImageView(vk::ImageViewType::e2D, depthFormat,
                                                vk::ComponentMapping(vk::ComponentSwizzle::eR, vk::ComponentSwizzle::eG, vk::ComponentSwizzle::eB, vk::ComponentSwizzle::eA),
                                                vk::ImageSubresourceRange(aspectMask, 0, 1, 0, 1), imageViewAllocator);

    // finally: the framebuffers!
    m_framebuffers.reserve(m_colorViews.size());
    for (size_t i = 0; i < m_colorViews.size(); i++)
    {
      m_framebuffers.push_back(device->createFramebuffer(renderPass, { m_colorViews[i], m_depthView }, m_extent, 1));
    }

    submitAndWait(device->getQueue(0, 0), commandBuffer);
  }

  std::vector<std::shared_ptr<Image>> const& FramebufferData::getColorImages() const
  {
    return m_colorImages;
  }

  std::vector<std::shared_ptr<ImageView>> const& FramebufferData::getColorViews() const
  {
    return m_colorViews;
  }

  std::shared_ptr<Image> const& FramebufferData::getDepthImage() const
  {
    return m_depthImage;
  }

  std::shared_ptr<ImageView> const& FramebufferData::getDepthView() const
  {
    return m_depthView;
  }

  vk::Extent2D const& FramebufferData::getExtent() const
  {
    return m_extent;
  }

  std::vector<std::shared_ptr<Framebuffer>> const& FramebufferData::getFramebuffers() const
  {
    return m_framebuffers;
  }

  std::shared_ptr<Swapchain> const& FramebufferData::getSwapchain() const
  {
    return m_swapchain;
  }

} // namespace vkh
