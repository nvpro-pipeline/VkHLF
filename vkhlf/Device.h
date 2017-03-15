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


#pragma once

#include <vkhlf/Config.h>
#include <vkhlf/DescriptorSetLayout.h>
#include <vkhlf/Pipeline.h>
#include <vkhlf/PipelineLayout.h>
#include <vkhlf/Sampler.h>
#include <vkhlf/Types.h>
#include <map>

namespace vkhlf
{
  struct CopyDescriptorSet;
  struct WriteDescriptorSet;

  struct DeviceQueueCreateInfo
  {
    VKHLF_API DeviceQueueCreateInfo(uint32_t queueFamilyIndex, vk::ArrayProxy<const float> queuePriorities);
    VKHLF_API DeviceQueueCreateInfo(DeviceQueueCreateInfo const& rhs);
    VKHLF_API DeviceQueueCreateInfo & operator=(DeviceQueueCreateInfo const& rhs);

    uint32_t            queueFamilyIndex;
    std::vector<float>  queuePriorities;
  };

  class Device : public Reference<PhysicalDevice, Allocator>, public std::enable_shared_from_this<Device>
  {
    public:
      VKHLF_API static std::shared_ptr<Device> create(std::shared_ptr<PhysicalDevice> const& physicalDevice,
                                                      vk::ArrayProxy<const DeviceQueueCreateInfo> queueCreateInfos,
                                                      vk::ArrayProxy<const std::string> enabledLayerNames,
                                                      vk::ArrayProxy<const std::string> enabledExtensionNames,
                                                      vk::PhysicalDeviceFeatures const& enabledFeatures,
                                                      std::shared_ptr<Allocator> const& allocator);

      VKHLF_API virtual ~Device();

      // allocate DeviceMemory
      VKHLF_API std::shared_ptr<DeviceMemory> allocateMemory(vk::DeviceSize allocationSize, uint32_t memoryTypeIndex, std::shared_ptr<DeviceMemoryAllocator> const& deviceMemoryAllocator);

      // create buffer
      VKHLF_API std::shared_ptr<Buffer> createBuffer(vk::BufferCreateFlags createFlags, vk::DeviceSize size, vk::BufferUsageFlags usageFlags = vk::BufferUsageFlagBits::eTransferDst,
                                                  vk::SharingMode sharingMode = vk::SharingMode::eExclusive, vk::ArrayProxy<const uint32_t> queueFamilyIndices = nullptr,
                                                  vk::MemoryPropertyFlags memoryPropertyFlags = vk::MemoryPropertyFlagBits::eDeviceLocal,
                                                  std::shared_ptr<DeviceMemoryAllocator> const& deviceMemoryAllocator = nullptr, std::shared_ptr<Allocator> const& bufferAllocator = nullptr);
      VKHLF_API std::shared_ptr<Buffer> createBuffer(vk::DeviceSize size, vk::BufferUsageFlags usageFlags = vk::BufferUsageFlagBits::eTransferDst, vk::SharingMode sharingMode = vk::SharingMode::eExclusive,
                                                  vk::ArrayProxy<const uint32_t> queueFamilyIndices = nullptr, vk::MemoryPropertyFlags memoryPropertyFlags = vk::MemoryPropertyFlagBits::eDeviceLocal,
                                                  std::shared_ptr<DeviceMemoryAllocator> const& deviceMemoryAllocator = nullptr, std::shared_ptr<Allocator> const& bufferAllocator = nullptr);

      // create CommandPool
      VKHLF_API std::shared_ptr<CommandPool> createCommandPool(vk::CommandPoolCreateFlags flags = {}, uint32_t familyIndex = 0, std::shared_ptr<Allocator> const& allocator = nullptr);

      // create DescriptorPool
      VKHLF_API std::shared_ptr<DescriptorPool> createDescriptorPool(vk::DescriptorPoolCreateFlags flags, uint32_t maxSets, vk::ArrayProxy<const vk::DescriptorPoolSize> poolSizes,
                                                                  std::shared_ptr<Allocator> const& allocator = nullptr);

      // DescriptorSet
      VKHLF_API std::shared_ptr<DescriptorSet> allocateDescriptorSet(std::shared_ptr<DescriptorPool> const& pool, std::shared_ptr<DescriptorSetLayout> const& layout);
      VKHLF_API void updateDescriptorSets(vk::ArrayProxy<const WriteDescriptorSet> descriptorWrites, vk::ArrayProxy<const CopyDescriptorSet> descriptorCopies);

      // create DescriptorSetLayout
      VKHLF_API std::shared_ptr<DescriptorSetLayout> createDescriptorSetLayout(vk::ArrayProxy<const DescriptorSetLayoutBinding> bindings, std::shared_ptr<Allocator> const& allocator = nullptr);

      // create Event
      VKHLF_API std::shared_ptr<Event> createEvent(std::shared_ptr<Allocator> const& allocator = nullptr);

      // create Fence
      VKHLF_API std::shared_ptr<Fence> createFence(bool signaled, std::shared_ptr<Allocator> const& allocator = nullptr);

      // create Framebuffer
      VKHLF_API std::shared_ptr<Framebuffer> createFramebuffer(std::shared_ptr<RenderPass> const & renderPass, vk::ArrayProxy<const std::shared_ptr<ImageView>> attachments, vk::Extent2D const& extent,
                                                            uint32_t layers, std::shared_ptr<Allocator> const& allocator = nullptr);

      // create Image
      VKHLF_API std::shared_ptr<Image> createImage(vk::ImageCreateFlags createFlags, vk::ImageType type, vk::Format format, vk::Extent3D const& extent, uint32_t mipLevels, uint32_t arraySize,
                                                vk::SampleCountFlagBits samples, vk::ImageTiling tiling, vk::ImageUsageFlags usageFlags, vk::SharingMode sharingMode = vk::SharingMode::eExclusive,
                                                std::vector<uint32_t> const& queueFamilyIndices = {}, vk::ImageLayout initialLayout = vk::ImageLayout::eUndefined,
                                                vk::MemoryPropertyFlags memoryPropertyFlags = vk::MemoryPropertyFlagBits::eDeviceLocal,
                                                std::shared_ptr<DeviceMemoryAllocator> const& deviceMemoryAllocator = nullptr, std::shared_ptr<Allocator> const& imageAllocator = nullptr);

      // create OcclusionQuery
      VKHLF_API std::shared_ptr<QueryPool> createOcclusionQuery(uint32_t entryCount, std::shared_ptr<Allocator> const& allocator = nullptr);

      // create Pipeline
      VKHLF_API std::shared_ptr<Pipeline> createComputePipeline(std::shared_ptr<PipelineCache> const & pipelineCache, vk::PipelineCreateFlags flags, PipelineShaderStageCreateInfo const& stage,
                                                             std::shared_ptr<PipelineLayout> const& layout, std::shared_ptr<Pipeline> const& basePipelineHandle, int32_t basePipelineIndex,
                                                             std::shared_ptr<Allocator> const& allocator = nullptr);
      VKHLF_API std::shared_ptr<Pipeline> createGraphicsPipeline(std::shared_ptr<PipelineCache> const& pipelineCache, vk::PipelineCreateFlags flags, vk::ArrayProxy<const PipelineShaderStageCreateInfo> stages,
                                                              vk::Optional<const PipelineVertexInputStateCreateInfo> vertexInputState,
                                                              vk::Optional<const vk::PipelineInputAssemblyStateCreateInfo> inputAssemblyState,
                                                              vk::Optional<const vk::PipelineTessellationStateCreateInfo> tessellationState,
                                                              vk::Optional<const PipelineViewportStateCreateInfo> viewportState,
                                                              vk::Optional<const vk::PipelineRasterizationStateCreateInfo> rasterizationState,
                                                              vk::Optional<const PipelineMultisampleStateCreateInfo> multisampleState,
                                                              vk::Optional<const vk::PipelineDepthStencilStateCreateInfo> depthStencilState,
                                                              vk::Optional<const PipelineColorBlendStateCreateInfo> colorBlendState,
                                                              vk::Optional<const PipelineDynamicStateCreateInfo> dynamicState,
                                                              std::shared_ptr<PipelineLayout> const& pipelineLayout, std::shared_ptr<RenderPass> const& renderPass, uint32_t subpass = 0,
                                                              std::shared_ptr<Pipeline> const& basePipelineHandle = {}, uint32_t basePipelineIndex = 0,
                                                              std::shared_ptr<Allocator> const& allocator = nullptr);

      // create PipelineCache
      VKHLF_API std::shared_ptr<PipelineCache> createPipelineCache(size_t initialSize, void const* initialData, std::shared_ptr<Allocator> const& allocator = nullptr);

      // create PipelineLayout
      VKHLF_API std::shared_ptr<PipelineLayout> createPipelineLayout(vk::ArrayProxy<const std::shared_ptr<DescriptorSetLayout>> setLayouts,
                                                                  vk::ArrayProxy<const vk::PushConstantRange> pushConstantRanges, std::shared_ptr<Allocator> const& allocator = nullptr);

      // create PipelineStatisticsQuery
      VKHLF_API std::shared_ptr<QueryPool> createPipelineStatisticsQuery(uint32_t entryCount, vk::QueryPipelineStatisticFlags flags, std::shared_ptr<Allocator> const& allocator = nullptr);

      // create RenderPass
      VKHLF_API std::shared_ptr<RenderPass> createRenderPass(vk::ArrayProxy<const vk::AttachmentDescription> attachments, vk::ArrayProxy<const vk::SubpassDescription> subpasses,
                                                          vk::ArrayProxy<const vk::SubpassDependency> dependencies, std::shared_ptr<Allocator> const& allocator = nullptr);

      // create Sampler
      VKHLF_API std::shared_ptr<Sampler> createSampler(vk::Filter magFilter, vk::Filter minFilter, vk::SamplerMipmapMode mipmapMode, vk::SamplerAddressMode addressModeU, vk::SamplerAddressMode addressModeV,
                                                    vk::SamplerAddressMode addressModeW, float mipLodBias, bool anisotropyEnable, float maxAnisotropy, bool compareEnable, vk::CompareOp compareOp,
                                                    float minLod, float maxLod, vk::BorderColor borderColor, bool unnormalizedCoordinates, std::shared_ptr<Allocator> const& allocator = nullptr);

      // create Semaphore
      VKHLF_API std::shared_ptr<Semaphore> createSemaphore(std::shared_ptr<Allocator> const& allocator = nullptr);

      // create ShaderModule
      VKHLF_API std::shared_ptr<ShaderModule> createShaderModule(vk::ArrayProxy<const uint32_t> code, std::shared_ptr<Allocator> const& allocator = nullptr);

      // create Swapchain
      VKHLF_API std::shared_ptr<Swapchain> createSwapchain(std::shared_ptr<Surface> const& surface, uint32_t minImageCount, vk::Format imageFormat, vk::Extent2D const& extent, uint32_t imageArrayLayers,
                                                        vk::ImageUsageFlags imageUsage, vk::SharingMode imageSharingMode, vk::ArrayProxy<const uint32_t> queueFamilyIndices,
                                                        vk::SurfaceTransformFlagBitsKHR preTransform, vk::CompositeAlphaFlagBitsKHR compositeAlpha, vk::PresentModeKHR presentMode, bool clipped,
                                                        std::shared_ptr<Swapchain> const& oldSwapchain = nullptr, std::shared_ptr<Allocator> const& allocator = nullptr);

      VKHLF_API PFN_vkVoidFunction                     getProcAddress(std::string const& name) const;
      VKHLF_API std::shared_ptr<Queue>                 getQueue(uint32_t familyIndex, uint32_t queueIndex);
      VKHLF_API size_t                                 getQueueCount(uint32_t familyIndex) const;
      VKHLF_API size_t                                 getQueueFamilyCount() const;
      VKHLF_API vk::Result                             waitForFences(vk::ArrayProxy<const std::shared_ptr<Fence>> fences, bool waitAll, uint32_t timeout) const;
      VKHLF_API void                                   waitIdle() const;

      VKHLF_API operator vk::Device() const;

      Device(Device const& rhs) = delete;
      Device & operator=(Device const& rhs) = delete;

    protected:
      VKHLF_API Device(std::shared_ptr<PhysicalDevice> const& physicalDevice, std::shared_ptr<Allocator> const& allocator);

    private:
      VKHLF_API void init(vk::ArrayProxy<const vkhlf::DeviceQueueCreateInfo> queueCreateInfos, vk::ArrayProxy<const std::string> enabledLayerNames,
                          vk::ArrayProxy<const std::string> enabledExtensionNames, vk::PhysicalDeviceFeatures const& enabledFeatures);

      vk::Device                                                     m_device;
      std::map<uint32_t, std::vector<std::unique_ptr<vkhlf::Queue>>> m_queues; // key is queueFamilyIndex
  };

  class FramebufferData
  {
    public:
      VKHLF_API FramebufferData(std::shared_ptr<Device> const& device, std::shared_ptr<Surface> const& surface, vk::Format depthFormat, std::shared_ptr<CommandBuffer> const& commandBuffer,
                             std::shared_ptr<RenderPass> const& renderPass, std::shared_ptr<DeviceMemoryAllocator> const& deviceMemoryAllocator = nullptr,
                             std::shared_ptr<Allocator> const& swapchainAllocator = nullptr, std::shared_ptr<Allocator> const& imageAllocator = nullptr,
                             std::shared_ptr<Allocator> const& imageViewAllocator = nullptr);

      VKHLF_API std::vector<std::shared_ptr<Image>> const&       getColorImages() const;
      VKHLF_API std::vector<std::shared_ptr<ImageView>> const&   getColorViews() const;
      VKHLF_API std::shared_ptr<Image> const&                    getDepthImage() const;
      VKHLF_API std::shared_ptr<ImageView> const&                getDepthView() const;
      VKHLF_API vk::Extent2D const&                              getExtent() const;
      VKHLF_API std::vector<std::shared_ptr<Framebuffer>> const& getFramebuffers() const;
      VKHLF_API std::shared_ptr<Swapchain> const&                getSwapchain() const;

    private:
      vk::Extent2D                              m_extent;
      std::shared_ptr<Swapchain>                m_swapchain;
      std::vector<std::shared_ptr<Image>>       m_colorImages;
      std::vector<std::shared_ptr<ImageView>>   m_colorViews;
      std::shared_ptr<Image>                    m_depthImage;
      std::shared_ptr<DeviceMemory>             m_depthMemory;
      std::shared_ptr<ImageView>                m_depthView;
      std::vector<std::shared_ptr<Framebuffer>> m_framebuffers;
  };

  inline Device::operator vk::Device() const
  {
    return m_device;
  }

  // TODO: wrapper function for vkUpdateDescriptorSets

} // namespace vk
