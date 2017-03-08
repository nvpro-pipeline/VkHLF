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
#include <vkhlf/CommandBuffer.h>
#include <vkhlf/CommandPool.h>
#include <vkhlf/DescriptorSet.h>
#include <vkhlf/Device.h>
#include <vkhlf/Event.h>
#include <vkhlf/Framebuffer.h>
#include <vkhlf/Image.h>
#include <vkhlf/QueryPool.h>
#include <vkhlf/RenderPass.h>

namespace vkhlf
{

  ImageMemoryBarrier::ImageMemoryBarrier(vk::AccessFlags srcAccessMask_, vk::AccessFlags dstAccessMask_, vk::ImageLayout oldLayout_, vk::ImageLayout newLayout_, uint32_t srcQueueFamilyIndex_,
                                         uint32_t dstQueueFamilyIndex_, std::shared_ptr<Image> const& image_, vk::ImageSubresourceRange const& subresourceRange_)
    : srcAccessMask(srcAccessMask_)
    , dstAccessMask(dstAccessMask_)
    , oldLayout(oldLayout_)
    , newLayout(newLayout_)
    , srcQueueFamilyIndex(srcQueueFamilyIndex_)
    , dstQueueFamilyIndex(dstQueueFamilyIndex_)
    , image(image_)
    , subresourceRange(subresourceRange_)
  {}

  ImageMemoryBarrier::ImageMemoryBarrier(ImageMemoryBarrier const& rhs)
    : ImageMemoryBarrier(rhs.srcAccessMask, rhs.dstAccessMask, rhs.oldLayout, rhs.newLayout, rhs.srcQueueFamilyIndex, rhs.dstQueueFamilyIndex, rhs.image, rhs.subresourceRange)
  {}

  ImageMemoryBarrier & ImageMemoryBarrier::operator=(ImageMemoryBarrier const& rhs)
  {
    srcAccessMask       = rhs.srcAccessMask;
    dstAccessMask       = rhs.dstAccessMask;
    oldLayout           = rhs.oldLayout;
    newLayout           = rhs.newLayout;
    srcQueueFamilyIndex = rhs.srcQueueFamilyIndex;
    dstQueueFamilyIndex = rhs.dstQueueFamilyIndex;
    image               = rhs.image;
    subresourceRange    = rhs.subresourceRange;
    return *this;
  }


  CommandBuffer::CommandBuffer(std::shared_ptr<vkhlf::CommandPool> const &commandPool, vk::CommandBufferLevel level)
    : Reference(commandPool)
#if !defined(NDEBUG)
    , m_inRenderPass(false)
    , m_isRecording(false)
    , m_isResetFromCommandPool(true)
    , m_level(level)
#endif
  {
#if !defined(NDEBUG)
    assert(get<CommandPool>());

    for ( size_t i=0 ; i<=VK_QUERY_TYPE_RANGE_SIZE ; i++ )
    {
      m_queryInfo[i].active = false;
    }
#endif

    vk::CommandBufferAllocateInfo info(*get<CommandPool>(), level, 1);
    std::vector<vk::CommandBuffer> commandBuffers = static_cast<vk::Device>(*commandPool->get<Device>()).allocateCommandBuffers(info);
    assert(!commandBuffers.empty());
    m_commandBuffer = commandBuffers[0];
  }

  CommandBuffer::~CommandBuffer()
  {
#if !defined(NDEBUG)
    assert(!m_resourceTracker || !m_resourceTracker->isUsed());
    get<CommandPool>()->onDeleteCommandBuffer(this);
#endif
    static_cast<vk::Device>(*get<CommandPool>()->get<Device>()).freeCommandBuffers(*get<CommandPool>(), m_commandBuffer);
  }

  void CommandBuffer::begin(vk::CommandBufferUsageFlags               flags,
                            std::shared_ptr<vkhlf::RenderPass> const&   renderPass,
                            uint32_t                                  subPass,
                            std::shared_ptr<vkhlf::Framebuffer> const&  framebuffer,
                            vk::Bool32                                occlusionQueryEnable,
                            vk::QueryControlFlags                     queryFlags,
                            vk::QueryPipelineStatisticFlags           pipelineStatistics)
  {
#if !defined(NDEBUG)
    // TBD: should we introduce some FirstLevelCommandBuffer and SecondLevelCommandBuffer, where FirstLevel can have a much simpler begin function?
    //      renderPass and subPass are meaningless for FirstLevelCommandBuffer
    //      occlusionQueryEnable, queryFlags, and pipelineStatistics are ignored for FirstLevelCommandBuffer
    assert(!m_isRecording);
    assert(get<CommandPool>()->individuallyResetCommandBuffers() || m_isResetFromCommandPool);
    // From the spec:
    //    If commandBuffer is a secondary command buffer and either the occlusionQueryEnable member of 
    //    pBeginInfo is VK_FALSE, or the precise occlusion queries feature is not enabled, the queryFlags member
    //    of pBeginInfo must not contain VK_QUERY_CONTROL_PRECISE_BIT
    // -> what's "precise occlusion queries feature" ??
    assert(!renderPass || ((m_level == vk::CommandBufferLevel::eSecondary) && (flags & vk::CommandBufferUsageFlagBits::eRenderPassContinue)));
    // From the spec:
    //    subpass is the index of the subpass within renderPass that the VkCommandBuffer will be rendering against if
    //    this is a secondary command buffer that was allocated with the VK_COMMAND_BUFFER_USAGE_RENDER_
    //    PASS_CONTINUE_BIT set.
    // -> where's the subpass within renderPass ??
    assert(!framebuffer || (flags & vk::CommandBufferUsageFlagBits::eRenderPassContinue));
    assert(!(renderPass && framebuffer) || (renderPass->get<Device>() == framebuffer->get<Device>()));
    // From the spec:
    //    If the inherited queries feature is not enabled, occlusionQueryEnable must be VK_FALSE
    //    If the inherited queries feature is enabled, queryFlags must be a valid combination of VkQueryControlFlagBits values
    // -> where's that inherited queries feature? The VkPhysicalDeviceFeatures I currently have doesn't have that entry
    // From the spec:
    //    If flags contains VK_COMMAND_BUFFER_USAGE_RENDER_PASS_CONTINUE_BIT, renderpass
    //    must not be VK_NULL_HANDLE, and must be compatible with the render pass for the render pass instance
    //    which this secondary command buffer will be executed in - see Section 8.2
    // -> just check first half, as the compatibility statement needs some extra knowledge (?)
    assert(!(flags & vk::CommandBufferUsageFlagBits::eRenderPassContinue) || renderPass);
    // From the spec:
    //    If flags contains VK_COMMAND_BUFFER_USAGE_RENDER_PASS_CONTINUE_BIT, and
    //    framebuffer is not VK_NULL_HANDLE, framebuffer must match the VkFramebuffer that is specified
    //    by vkCmdBeginRenderPass for the render pass instance which this secondary command buffer will be
    //    executed in
    // -> needs a check in beginRenderPass ?
    assert(!((flags & vk::CommandBufferUsageFlagBits::eRenderPassContinue) && framebuffer) || (renderPass && framebuffer->get<RenderPass>()->isCompatible(renderPass)));
    // From the spec:
    //    If renderPass is not VK_NULL_HANDLE, subpass must refer to a valid subpass index within
    //    renderPass, specifically the index of the subpass which this secondary command buffer will be executed in
    //  -> VkRenderPass has no subpass index ?!?

    m_flags = flags;
    m_isRecording = true;
    m_isResetFromCommandPool = false;
    m_occlusionQueryEnable = occlusionQueryEnable;
    m_stageFlags = vk::PipelineStageFlags();
#endif

    m_resourceTracker = vkhlf::ResourceTrackerAll::create();
    m_resourceTracker->track(renderPass);
    m_resourceTracker->track(framebuffer);
    m_renderPass = renderPass;
    m_framebuffer = framebuffer;

    vk::CommandBufferInheritanceInfo inheritanceInfo;
    vk::CommandBufferBeginInfo beginInfo(flags, &inheritanceInfo);

    inheritanceInfo.renderPass            = renderPass ? *renderPass : vk::RenderPass();
    inheritanceInfo.subpass               = subPass;
    inheritanceInfo.framebuffer           = framebuffer ? *framebuffer : vk::Framebuffer();
    inheritanceInfo.occlusionQueryEnable  = occlusionQueryEnable;
    inheritanceInfo.queryFlags            = queryFlags;
    inheritanceInfo.pipelineStatistics    = pipelineStatistics;

    m_commandBuffer.begin(beginInfo);
  }

  void CommandBuffer::beginQuery(std::shared_ptr<vkhlf::QueryPool> const& queryPool, uint32_t slot, vk::QueryControlFlags flags)
  {
#if !defined(NDEBUG)
    int queryInfoIndex = int(queryPool->getQueryType())-VK_QUERY_TYPE_BEGIN_RANGE;
    assert(!m_queryInfo[queryInfoIndex].active);
    m_queryInfo[queryInfoIndex].active = true;
    m_queryInfo[queryInfoIndex].contained = true;
    m_queryInfo[queryInfoIndex].flags = flags;
#endif
    m_commandBuffer.beginQuery(*queryPool, slot, flags);
  }

  void CommandBuffer::beginRenderPass(vk::RenderPassBeginInfo const & beginInfo, vk::SubpassContents contents)
  {
#if !defined(NDEBUG)
    assert(m_level == vk::CommandBufferLevel::ePrimary);
    m_inRenderPass = true;
#endif
    m_commandBuffer.beginRenderPass(beginInfo, contents);
  }

  void CommandBuffer::beginRenderPass( std::shared_ptr<vkhlf::RenderPass> const & renderPass, std::shared_ptr<vkhlf::Framebuffer> const & framebuffer, vk::Rect2D const & area, vk::ArrayProxy<const vk::ClearValue> clearValues, vk::SubpassContents contents)
  {
#if !defined(NDEBUG)
    assert(m_level == vk::CommandBufferLevel::ePrimary);
    m_inRenderPass = true;
#endif
    m_resourceTracker->track(renderPass);
    m_resourceTracker->track(framebuffer);
    m_renderPass = renderPass;
    m_framebuffer = framebuffer;

    vk::RenderPassBeginInfo renderPassBeginInfo;

    renderPassBeginInfo.renderPass      = *renderPass;
    renderPassBeginInfo.framebuffer     = *framebuffer;
    renderPassBeginInfo.renderArea      = area;
    renderPassBeginInfo.clearValueCount = static_cast<uint32_t>(clearValues.size());
    renderPassBeginInfo.pClearValues    = reinterpret_cast<vk::ClearValue const*>(clearValues.data());

    m_commandBuffer.beginRenderPass(renderPassBeginInfo, contents);
  }

  void CommandBuffer::bindDescriptorSets(vk::PipelineBindPoint pipelineBindPoint, std::shared_ptr<vkhlf::PipelineLayout> const& pipelineLayout, uint32_t firstSet, vk::ArrayProxy<const std::shared_ptr<vkhlf::DescriptorSet>> descriptorSets, vk::ArrayProxy<const uint32_t> dynamicOffsets)
  {
    m_bindDescriptorSets.clear();
    for(auto & descriptor : descriptorSets)
    {
      m_resourceTracker->track(descriptor);
      m_bindDescriptorSets.push_back(*descriptor);
    }

    m_commandBuffer.bindDescriptorSets(pipelineBindPoint, *pipelineLayout, firstSet, m_bindDescriptorSets, dynamicOffsets);
  }

  void CommandBuffer::bindIndexBuffer(std::shared_ptr<vkhlf::Buffer> const& buffer, vk::DeviceSize offset, vk::IndexType indexType)
  {
    m_resourceTracker->track(buffer);
    m_commandBuffer.bindIndexBuffer(*buffer, offset, indexType);
  }

  void CommandBuffer::bindPipeline(vk::PipelineBindPoint bindingPoint, std::shared_ptr<vkhlf::Pipeline> const& pipeline)
  {
    m_commandBuffer.bindPipeline(bindingPoint, *pipeline);
  }

  void CommandBuffer::bindVertexBuffer(uint32_t startBinding, std::shared_ptr<vkhlf::Buffer> const& buffer, vk::DeviceSize offset)
  {
    m_resourceTracker->track(buffer);
    m_commandBuffer.bindVertexBuffers(startBinding, static_cast<vk::Buffer>(*buffer), offset);
  }

  void CommandBuffer::bindVertexBuffers(uint32_t startBinding, vk::ArrayProxy<const std::shared_ptr<vkhlf::Buffer>> buffers, vk::ArrayProxy<const vk::DeviceSize> offsets)
  {
    assert(buffers.size() == offsets.size());

    m_bindVertexBuffers.clear();
    for(auto & buffer : buffers)
    {
      m_resourceTracker->track(buffer);
      m_bindVertexBuffers.push_back(*buffer);
    }

    m_commandBuffer.bindVertexBuffers(startBinding, m_bindVertexBuffers, offsets);
  }

  void CommandBuffer::blitImage(std::shared_ptr<vkhlf::Image> const& srcImage, vk::ImageLayout srcImageLayout, std::shared_ptr<vkhlf::Image> const& dstImage, vk::ImageLayout dstImageLayout, vk::ArrayProxy<const vk::ImageBlit> regions, vk::Filter filter)
  {
    m_resourceTracker->track(srcImage);
    m_commandBuffer.blitImage(*srcImage, srcImageLayout, *dstImage, dstImageLayout, regions, filter);
  }

  void CommandBuffer::clearAttachments(vk::ArrayProxy<const vk::ClearAttachment> attachments, vk::ArrayProxy<const vk::ClearRect> rects)
  {
    m_commandBuffer.clearAttachments(attachments, rects);
  }

  void CommandBuffer::clearColorImage(std::shared_ptr<vkhlf::Image> const & image, vk::ImageLayout imageLayout, vk::ClearColorValue const & color, vk::ArrayProxy<const vk::ImageSubresourceRange> ranges)
  {
    m_resourceTracker->track(image);
    m_commandBuffer.clearColorImage(*image, imageLayout, color, ranges);
  }

  void CommandBuffer::clearDepthStencilImage(std::shared_ptr<vkhlf::Image> const& image, vk::ImageLayout imageLayout, float depth, uint32_t stencil, vk::ArrayProxy<const vk::ImageSubresourceRange> ranges)
  {
    m_resourceTracker->track(image);
    vk::ClearDepthStencilValue depthStencil {depth, stencil};
    m_commandBuffer.clearDepthStencilImage(*image, imageLayout, depthStencil, ranges);
  }

  void CommandBuffer::copyBuffer(std::shared_ptr<vkhlf::Buffer> const& srcBuffer, std::shared_ptr<vkhlf::Buffer> const& dstBuffer, vk::ArrayProxy<const vk::BufferCopy> regions)
  {
    m_resourceTracker->track(srcBuffer);
    m_resourceTracker->track(dstBuffer);
    m_commandBuffer.copyBuffer(*srcBuffer, *dstBuffer, regions);
  }

  void CommandBuffer::copyBufferToImage(std::shared_ptr<vkhlf::Buffer> const& srcBuffer, std::shared_ptr<vkhlf::Image> const& dstImage, vk::ImageLayout dstImageLayout, vk::ArrayProxy<const vk::BufferImageCopy> regions)
  {
    m_resourceTracker->track(srcBuffer);
    m_resourceTracker->track(dstImage);
    m_commandBuffer.copyBufferToImage(*srcBuffer, *dstImage, dstImageLayout, regions);
  }

  void CommandBuffer::copyImage(std::shared_ptr<vkhlf::Image> const& srcImage, vk::ImageLayout srcImageLayout, std::shared_ptr<vkhlf::Image> const& dstImage, vk::ImageLayout dstImageLayout, vk::ArrayProxy<const vk::ImageCopy> regions)
  {
    m_resourceTracker->track(srcImage);
    m_resourceTracker->track(dstImage);
    m_commandBuffer.copyImage(*srcImage, srcImageLayout, *dstImage, dstImageLayout, regions);
  }

  void CommandBuffer::copyImageToBuffer(std::shared_ptr<vkhlf::Image> const& srcImage, vk::ImageLayout srcImageLayout, std::shared_ptr<vkhlf::Buffer> const& dstBuffer, vk::ArrayProxy<const vk::BufferImageCopy> regions)
  {
    m_resourceTracker->track(srcImage);
    m_resourceTracker->track(dstBuffer);
    m_commandBuffer.copyImageToBuffer(*srcImage, srcImageLayout, *dstBuffer, regions);
  }

  void CommandBuffer::copyQueryPoolResults(std::shared_ptr<vkhlf::QueryPool> const& queryPool, uint32_t startQuery, uint32_t queryCount, std::shared_ptr<vkhlf::Buffer> const& dstBuffer, vk::DeviceSize dstOffset, vk::DeviceSize dstStride, vk::QueryResultFlags flags)
  {
    m_resourceTracker->track(dstBuffer);
    m_commandBuffer.copyQueryPoolResults(*queryPool, startQuery, queryCount, *dstBuffer, dstOffset, dstStride, flags);
  }

  void CommandBuffer::dispatch(uint32_t x, uint32_t y, uint32_t z)
  {
    m_commandBuffer.dispatch(x, y, z);
  }

  void CommandBuffer::dispatchIndirect(std::shared_ptr<vkhlf::Buffer> const& buffer, vk::DeviceSize offset)
  {
    m_resourceTracker->track(buffer);
    m_commandBuffer.dispatchIndirect(*buffer, offset);
  }

  void CommandBuffer::draw(uint32_t vertexCount, uint32_t instanceCount, uint32_t firstVertex, uint32_t firstInstance)
  {
    m_commandBuffer.draw(vertexCount, instanceCount, firstVertex, firstInstance);
  }

  void CommandBuffer::drawIndexed(uint32_t indexCount, uint32_t instanceCount, uint32_t firstIndex, int32_t vertexOffset, uint32_t firstInstance)
  {
    m_commandBuffer.drawIndexed(indexCount, instanceCount, firstIndex, vertexOffset, firstInstance);
  }

  void CommandBuffer::drawIndexedIndirect(std::shared_ptr<vkhlf::Buffer> const& buffer, vk::DeviceSize offset, uint32_t count, uint32_t stride)
  {
    m_resourceTracker->track(buffer);
    m_commandBuffer.drawIndexedIndirect(*buffer, offset, count, stride);
  }

  void CommandBuffer::drawIndirect(std::shared_ptr<vkhlf::Buffer> const& buffer, vk::DeviceSize offset, uint32_t count, uint32_t stride)
  {
    m_resourceTracker->track(buffer);
    m_commandBuffer.drawIndirect(*buffer, offset, count, stride);
  }

  void CommandBuffer::end()
  {
#if !defined(NDEBUG)
    assert(m_isRecording);
    m_isRecording = false;
    // From the spec:
    //    vkEndCommandBuffer must not be called inside a render pass instance
    //    All queries made active during the recording of commandBuffer must have been made inactive
#endif

    m_commandBuffer.end();
  }

  void CommandBuffer::endQuery(std::shared_ptr<vkhlf::QueryPool> const& queryPool, uint32_t slot)
  {
#if !defined(NDEBUG)
    assert(m_queryInfo[int(queryPool->getQueryType())-VK_QUERY_TYPE_BEGIN_RANGE].active);
    m_queryInfo[int(queryPool->getQueryType())-VK_QUERY_TYPE_BEGIN_RANGE].active = false;
#endif
    m_commandBuffer.endQuery(*queryPool, slot);
  }

  void CommandBuffer::endRenderPass()
  {
#if !defined(NDEBUG)
    assert(m_level == vk::CommandBufferLevel::ePrimary);
    m_inRenderPass = false;
#endif
    // TODO actually a commandBuffer should keep a std::vector for those
    m_renderPass.reset();
    m_framebuffer.reset();

    m_commandBuffer.endRenderPass();
  }

  void CommandBuffer::executeCommands(vk::ArrayProxy<const std::shared_ptr<vkhlf::CommandBuffer>> commands)
  {
    assert(m_isRecording);
    assert(get<CommandPool>()->supportsCompute() || get<CommandPool>()->supportsGraphics() || get<CommandPool>()->supportsTransfer());
    assert(m_level == vk::CommandBufferLevel::ePrimary);
    assert(!commands.empty());
    // From the Spec:
    //    If the inherited queries feature is not enabled, commandBuffer must not have any queries active
    // -> where's that inherited queries feature? The VkPhysicalDeviceFeatures I currently have doesn't have that entry

    m_secondaryCommandBuffers.insert(m_secondaryCommandBuffers.end(), commands.begin(), commands.end());

    std::vector<vk::CommandBuffer> c;
    for (auto const& it : commands)
    {
      assert(get<CommandPool>()->get<Device>() == it->get<CommandPool>()->get<Device>());
      assert(it->m_level == vk::CommandBufferLevel::eSecondary);
      assert((it->m_flags & vk::CommandBufferUsageFlagBits::eSimultaneousUse) || (std::count(m_secondaryCommandBuffers.begin(), m_secondaryCommandBuffers.end(), it) == 1));
      assert(!it->getPrimaryCommandBuffer());
      assert(!it->isRecording());
      assert(!m_inRenderPass || (it->m_flags & vk::CommandBufferUsageFlagBits::eRenderPassContinue));
      assert(!m_queryInfo[VK_QUERY_TYPE_OCCLUSION].active || (it->m_occlusionQueryEnable && (it->m_queryInfo[VK_QUERY_TYPE_OCCLUSION].flags == m_queryInfo[VK_QUERY_TYPE_OCCLUSION].flags)));
      assert(!m_queryInfo[VK_QUERY_TYPE_PIPELINE_STATISTICS].active || (it->m_queryInfo[VK_QUERY_TYPE_PIPELINE_STATISTICS].flags == m_queryInfo[VK_QUERY_TYPE_PIPELINE_STATISTICS].flags));
#if !defined(NDEBUG)
      for (size_t i = 0; i<VK_QUERY_TYPE_RANGE_SIZE; i++)
      {
        assert(!m_queryInfo[i].active || !it->m_queryInfo[i].contained);
      }
#endif

      c.push_back(*it);

#if !defined(NDEBUG)
      it->setPrimaryCommandBuffer(shared_from_this());
#endif
    }
    m_commandBuffer.executeCommands(c);
  }

  void CommandBuffer::fillBuffer(std::shared_ptr<vkhlf::Buffer> const& dstBuffer, vk::DeviceSize dstOffset, vk::DeviceSize fillSize, uint32_t data)
  {
    m_resourceTracker->track(dstBuffer);
    m_commandBuffer.fillBuffer(*dstBuffer, dstOffset, fillSize, data);
  }

  void CommandBuffer::nextSubpass(vk::SubpassContents contents)
  {
    assert(m_level == vk::CommandBufferLevel::ePrimary);
    m_commandBuffer.nextSubpass(contents);
  }

  void CommandBuffer::pipelineBarrier(vk::PipelineStageFlags srcStageMask, vk::PipelineStageFlags destStageMask, vk::DependencyFlags dependencyFlags,
                                      vk::ArrayProxy<const vk::MemoryBarrier> barriers, vk::ArrayProxy<const vk::BufferMemoryBarrier> bufferMemoryBarriers,
                                      vk::ArrayProxy<const ImageMemoryBarrier> imageMemoryBarriers)
  {
    std::vector<vk::ImageMemoryBarrier> imbs;
    imbs.reserve(imageMemoryBarriers.size());
    for (auto const& imb : imageMemoryBarriers)
    {
      m_resourceTracker->track(imb.image);
      imbs.push_back(vk::ImageMemoryBarrier(imb.srcAccessMask, imb.dstAccessMask, imb.oldLayout, imb.newLayout, imb.srcQueueFamilyIndex, imb.dstQueueFamilyIndex,
                                            imb.image ? static_cast<vk::Image>(*imb.image) : nullptr, imb.subresourceRange));
    }

    m_commandBuffer.pipelineBarrier(srcStageMask, destStageMask, dependencyFlags, barriers, bufferMemoryBarriers, imbs);
  }

  void CommandBuffer::reset( vk::CommandBufferResetFlags flags )
  {
    assert(get<CommandPool>()->individuallyResetCommandBuffers());
    assert(!m_resourceTracker->isUsed());
    assert(!isRecording());

    m_commandBuffer.reset(flags);
    m_resourceTracker.reset();
    m_renderPass.reset();
    m_framebuffer.reset();
    m_secondaryCommandBuffers.clear();
#if !defined(NDEBUG)
    m_stageFlags = vk::PipelineStageFlags();
#endif
  }

  void CommandBuffer::resetEvent(std::shared_ptr<vkhlf::Event> const& event, vk::PipelineStageFlags stageMask)
  {
    assert(!!stageMask);
    assert(m_isRecording);
    assert(get<CommandPool>()->supportsGraphics() || get<CommandPool>()->supportsCompute());
    assert(!m_inRenderPass);
    assert(get<CommandPool>()->get<Device>() == event->get<Device>());
    // From the spec:
    //    If the geometry shaders feature is not enabled, stageMask must not contain VK_PIPELINE_STAGE_GEOMETRY_SHADER_BIT
    //    If the tessellation shaders feature is not enabled, stageMask must not contain VK_PIPELINE_STAGE_
    //    TESSELLATION_CONTROL_SHADER_BIT or VK_PIPELINE_STAGE_TESSELLATION_EVALUATION_SHADER_BIT
    m_resourceTracker->track(event);
    m_commandBuffer.resetEvent(*event, stageMask);
  }

  void CommandBuffer::resetQueryPool(std::shared_ptr<vkhlf::QueryPool> const& queryPool, uint32_t startQuery, uint32_t queryCount)
  {
    m_commandBuffer.resetQueryPool(*queryPool, startQuery, queryCount);
  }

  void CommandBuffer::resolveImage(std::shared_ptr<vkhlf::Image> const& srcImage, vk::ImageLayout srcImageLayout, std::shared_ptr<vkhlf::Image> const& dstImage, vk::ImageLayout dstImageLayout, vk::ArrayProxy<const vk::ImageResolve> regions)
  {
    m_resourceTracker->track(srcImage);
    m_resourceTracker->track(dstImage);
    m_commandBuffer.resolveImage(*srcImage, srcImageLayout, *dstImage, dstImageLayout, regions);
  }

  void CommandBuffer::setBlendConstants(const float blendConst[4])
  {
    m_commandBuffer.setBlendConstants(blendConst);
  }

  void CommandBuffer::setDepthBias(float depthBias, float depthBiasClamp, float slopeScaledDepthBias)
  {
    m_commandBuffer.setDepthBias(depthBias, depthBiasClamp, slopeScaledDepthBias);
  }

  void CommandBuffer::setDepthBounds(float minDepthBounds, float maxDepthBounds)
  {
    m_commandBuffer.setDepthBounds(minDepthBounds, maxDepthBounds);
  }

  void CommandBuffer::setEvent(std::shared_ptr<vkhlf::Event> const& event, vk::PipelineStageFlags stageMask)
  {
    assert(!!stageMask);
    assert(m_isRecording);
    assert(get<CommandPool>()->supportsGraphics() || get<CommandPool>()->supportsCompute());
    assert(!m_inRenderPass);
    assert(get<CommandPool>()->get<Device>() == event->get<Device>());
    // From the spec:
    //    If the geometry shaders feature is not enabled, stageMask must not contain VK_PIPELINE_STAGE_GEOMETRY_SHADER_BIT
    //    If the tessellation shaders feature is not enabled, stageMask must not contain VK_PIPELINE_STAGE_
    //    TESSELLATION_CONTROL_SHADER_BIT or VK_PIPELINE_STAGE_TESSELLATION_EVALUATION_SHADER_BIT
#if !defined(NDEBUG)
    m_stageFlags |= stageMask;
#endif
    m_resourceTracker->track(event);
    m_commandBuffer.setEvent(*event, stageMask);
  }

  void CommandBuffer::setLineWidth(float lineWidth)
  {
    m_commandBuffer.setLineWidth(lineWidth);
  }

  void CommandBuffer::setScissor(uint32_t first, vk::ArrayProxy<const vk::Rect2D> scissors)
  {
    m_commandBuffer.setScissor(first, scissors);
  }

  void CommandBuffer::setStencilCompareMask(vk::StencilFaceFlags faceMask, uint32_t stencilCompareMask)
  {
    m_commandBuffer.setStencilCompareMask(faceMask, stencilCompareMask);
  }

  void CommandBuffer::setStencilReference(vk::StencilFaceFlags faceMask, uint32_t stencilReference)
  {
    m_commandBuffer.setStencilReference(faceMask, stencilReference);
  }

  void CommandBuffer::setStencilWriteMask(vk::StencilFaceFlags faceMask, uint32_t stencilWriteMask)
  {
    m_commandBuffer.setStencilWriteMask(faceMask, stencilWriteMask);
  }

  void CommandBuffer::setViewport(uint32_t first, vk::ArrayProxy<const vk::Viewport> viewports)
  {
    m_commandBuffer.setViewport(first, viewports);
  }

  void CommandBuffer::waitEvents(vk::ArrayProxy<const std::shared_ptr<vkhlf::Event>> events, vk::PipelineStageFlags srcStageMask, vk::PipelineStageFlags dstStageMask,
                                 vk::ArrayProxy<const vk::MemoryBarrier> memoryBarriers, vk::ArrayProxy<const vk::BufferMemoryBarrier> bufferMemoryBarriers,
                                 vk::ArrayProxy<const vk::ImageMemoryBarrier> imageMemoryBarriers)
  {
    assert(!!srcStageMask);
    assert(!!dstStageMask);
    assert(m_isRecording);
    assert(get<CommandPool>()->supportsGraphics() || get<CommandPool>()->supportsCompute());
    assert(!events.empty());
    assert(m_stageFlags == srcStageMask);
    // From the spec:
    //    If vkSetEvent was used to signal any of the events in pEvents, srcStageMask must include the VK_
    //    PIPELINE_STAGE_HOST_BIT flag
    // -> is there any constraint on when that vkSetEvent had happened?? would need to add VK_PIPELINE_STAGE_HOST_BIT to m_stageFlags
    // From the spec:
    //    If the geometry shaders feature is not enabled, srcStageMask must not contain VK_PIPELINE_STAGE_GEOMETRY_SHADER_BIT
    //    If the geometry shaders feature is not enabled, dstStageMask must not contain VK_PIPELINE_STAGE_GEOMETRY_SHADER_BIT
    //    If the tessellation shaders feature is not enabled, srcStageMask must not contain VK_PIPELINE_STAGE_
    //    TESSELLATION_CONTROL_SHADER_BIT or VK_PIPELINE_STAGE_TESSELLATION_EVALUATION_SHADER_BIT
    //    If the tessellation shaders feature is not enabled, dstStageMask must not contain VK_PIPELINE_STAGE_
    //    TESSELLATION_CONTROL_SHADER_BIT or VK_PIPELINE_STAGE_TESSELLATION_EVALUATION_SHADER_BIT
    // From the spec:
    //    If the value of memoryBarrierCount is not 0, any given element of ppMemoryBarriers must point to a
    //    valid vk::MemoryBarrier, vk::BufferMemoryBarrier or vk::ImageMemoryBarrier structure
    // From the spec:
    //    If pEvents includes one or more events that will be signaled by vkSetEvent after commandBuffer has
    //    been submitted to a queue, then vkCmdWaitEvents must not be called inside a render pass instance

    std::vector<vk::Event> e;
    for(auto const & event : events)
    {
      m_resourceTracker->track(event);
      assert(get<CommandPool>()->get<Device>() == event->get<Device>());
      e.push_back(*event);
    }

    m_commandBuffer.waitEvents(e, srcStageMask, dstStageMask, memoryBarriers, bufferMemoryBarriers, imageMemoryBarriers);
  }

  void CommandBuffer::writeTimestamp(vk::PipelineStageFlagBits pipelineStage, std::shared_ptr<vkhlf::QueryPool> const& queryPool, uint32_t entry)
  {
    m_commandBuffer.writeTimestamp(pipelineStage, *queryPool, entry);
  }

  void CommandBuffer::onReset()
  {
    assert(!m_resourceTracker->isUsed());
    m_renderPass.reset();
    m_framebuffer.reset();
    m_secondaryCommandBuffers.clear();

#if !defined(NDEBUG)
    m_isResetFromCommandPool = true;
#endif
  }

  std::shared_ptr<ResourceTracker> CommandBuffer::getResourceTracker() const
  {
    return m_resourceTracker;
  }

#if !defined(NDEBUG)
  void CommandBuffer::setPrimaryCommandBuffer(std::shared_ptr<vkhlf::CommandBuffer> const& primaryCommandBuffer)
  {
    m_primaryCommandBuffer = primaryCommandBuffer;
  }

  std::shared_ptr<vkhlf::CommandBuffer> CommandBuffer::getPrimaryCommandBuffer() const
  {
    return m_primaryCommandBuffer.lock();
  }

  bool CommandBuffer::isOneTimeSubmit() const
  {
    return !!(m_flags & vk::CommandBufferUsageFlagBits::eOneTimeSubmit);
  }

  bool CommandBuffer::isRecording() const
  {
    return m_isRecording;
  }

  bool CommandBuffer::isSimultaneousUsageAllowed() const
  {
    return !!(m_flags & vk::CommandBufferUsageFlagBits::eSimultaneousUse);
  }
#endif

  static vk::AccessFlags determineAccessFlags(vk::ImageLayout layout)
  {
    vk::AccessFlags accessFlags;
    switch (layout)
    {
      case vk::ImageLayout::eColorAttachmentOptimal:
        accessFlags = vk::AccessFlagBits::eColorAttachmentWrite;
        break;
      case vk::ImageLayout::eDepthStencilAttachmentOptimal:
        accessFlags = vk::AccessFlagBits::eDepthStencilAttachmentWrite;
        break;
      case vk::ImageLayout::eDepthStencilReadOnlyOptimal:
        accessFlags = vk::AccessFlagBits::eDepthStencilAttachmentRead;
        break;
      case vk::ImageLayout::eShaderReadOnlyOptimal:
        accessFlags = vk::AccessFlagBits::eShaderRead;
        break;
      case vk::ImageLayout::eTransferSrcOptimal:
        accessFlags = vk::AccessFlagBits::eTransferRead;
        break;
      case vk::ImageLayout::eTransferDstOptimal:
        accessFlags = vk::AccessFlagBits::eTransferWrite;
        break;
      case vk::ImageLayout::ePreinitialized:
        accessFlags = vk::AccessFlagBits::eHostWrite;
        break;
      // for all other cases: keep accessFlags == 0
      case vk::ImageLayout::eUndefined:
      case vk::ImageLayout::eGeneral:
      case vk::ImageLayout::ePresentSrcKHR:
      default:
        break;
    }
    return accessFlags;
  }

  void setImageLayout(std::shared_ptr<vkhlf::CommandBuffer> const& commandBuffer, std::shared_ptr<vkhlf::Image> const& image, vk::ImageSubresourceRange const& subresourceRange, vk::ImageLayout oldImageLayout, vk::ImageLayout newImageLayout)
  {
    vk::AccessFlags srcAccessMask = determineAccessFlags(oldImageLayout);
    vk::AccessFlags dstAccessMask = determineAccessFlags(newImageLayout);

    vkhlf::ImageMemoryBarrier imageMemoryBarrier(srcAccessMask, dstAccessMask, oldImageLayout, newImageLayout, VK_QUEUE_FAMILY_IGNORED, VK_QUEUE_FAMILY_IGNORED, image, subresourceRange);

    assert(commandBuffer->isRecording());
    commandBuffer->pipelineBarrier(vk::PipelineStageFlagBits::eTopOfPipe, vk::PipelineStageFlagBits::eTopOfPipe, {}, nullptr, nullptr, imageMemoryBarrier);
  }

} // namespace vkhlf
