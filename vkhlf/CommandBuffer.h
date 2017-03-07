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
#include <vkhlf/Reference.h>
#include <vkhlf/ResourceTrackerAll.h>
#include <vkhlf/Types.h>
#include <memory>
#include <set>

namespace vkhlf
{

  struct ImageMemoryBarrier
  {
    VKHLF_API ImageMemoryBarrier(vk::AccessFlags srcAccessMask, vk::AccessFlags dstAccessMask, vk::ImageLayout oldLayout, vk::ImageLayout newLayout, uint32_t srcQueueFamilyIndex, uint32_t dstQueueFamilyIndex,
                              std::shared_ptr<Image> const& image, vk::ImageSubresourceRange const& subresourceRange);
    VKHLF_API ImageMemoryBarrier(ImageMemoryBarrier const& rhs);
    VKHLF_API ImageMemoryBarrier & operator=(ImageMemoryBarrier const& rhs);

    vk::AccessFlags           srcAccessMask;
    vk::AccessFlags           dstAccessMask;
    vk::ImageLayout           oldLayout;
    vk::ImageLayout           newLayout;
    uint32_t                  srcQueueFamilyIndex;
    uint32_t                  dstQueueFamilyIndex;
    std::shared_ptr<Image>    image;
    vk::ImageSubresourceRange subresourceRange;
  };

#undef MemoryBarrier

  class CommandBuffer : public Reference<CommandPool>
#if !defined(NDEBUG)
    , public std::enable_shared_from_this<CommandBuffer>
#endif
  {
    public:
      VKHLF_API CommandBuffer(std::shared_ptr<vkhlf::CommandPool> const &commandPool, vk::CommandBufferLevel level = vk::CommandBufferLevel::ePrimary);
      VKHLF_API virtual ~CommandBuffer();

      VKHLF_API void begin(vk::CommandBufferUsageFlags               flags = vk::CommandBufferUsageFlags(),
                        std::shared_ptr<vkhlf::RenderPass> const&   renderPass = std::shared_ptr<vkhlf::RenderPass>(),
                        uint32_t                                  subpass = 0,
                        std::shared_ptr<vkhlf::Framebuffer> const&  framebuffer = std::shared_ptr<vkhlf::Framebuffer>(),
                        vk::Bool32                                occlusionQueryEnable = false,
                        vk::QueryControlFlags                     queryFlags = vk::QueryControlFlags(),
                        vk::QueryPipelineStatisticFlags           pipelineStatistics = vk::QueryPipelineStatisticFlags());
      VKHLF_API void beginQuery(std::shared_ptr<vkhlf::QueryPool> const& queryPool, uint32_t slot, vk::QueryControlFlags flags);
      VKHLF_API void beginRenderPass(vk::RenderPassBeginInfo const & beginInfo, vk::SubpassContents contents);
      VKHLF_API void beginRenderPass(std::shared_ptr<vkhlf::RenderPass> const & renderPass, std::shared_ptr<vkhlf::Framebuffer> const & framebuffer, vk::Rect2D const & area, vk::ArrayProxy<const vk::ClearValue> clearValues, vk::SubpassContents contents);
      VKHLF_API void bindDescriptorSets(vk::PipelineBindPoint pipelineBindPoint, std::shared_ptr<vkhlf::PipelineLayout> const& pipelineLayout, uint32_t firstSet, vk::ArrayProxy<const std::shared_ptr<vkhlf::DescriptorSet>> descriptorSets, vk::ArrayProxy<const uint32_t> dynamicOffsets);
      VKHLF_API void bindIndexBuffer(std::shared_ptr<vkhlf::Buffer> const& buffer, vk::DeviceSize offset, vk::IndexType indexType);
      VKHLF_API void bindPipeline(vk::PipelineBindPoint bindingPoint, std::shared_ptr<vkhlf::Pipeline> const& pipeline);
      VKHLF_API void bindVertexBuffer(uint32_t startBinding, std::shared_ptr<vkhlf::Buffer> const& buffer, vk::DeviceSize offset);
      VKHLF_API void bindVertexBuffers(uint32_t startBinding, vk::ArrayProxy<const std::shared_ptr<vkhlf::Buffer>> buffers, vk::ArrayProxy<const vk::DeviceSize> offsets);
      VKHLF_API void blitImage(std::shared_ptr<vkhlf::Image> const& srcImage, vk::ImageLayout srcImageLayout, std::shared_ptr<vkhlf::Image> const& dstImage, vk::ImageLayout dstImageLayout, vk::ArrayProxy<const vk::ImageBlit> regions, vk::Filter filter);
      VKHLF_API void clearAttachments(vk::ArrayProxy< const vk::ClearAttachment> attachments, vk::ArrayProxy<const vk::ClearRect> rects);
      VKHLF_API void clearColorImage(std::shared_ptr<vkhlf::Image> const & image, vk::ImageLayout imageLayout, vk::ClearColorValue const & color, vk::ArrayProxy<const vk::ImageSubresourceRange> ranges = { { vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1 } });
      VKHLF_API void clearDepthStencilImage(std::shared_ptr<vkhlf::Image> const& image, vk::ImageLayout imageLayout, float depth, uint32_t stencil, vk::ArrayProxy<const vk::ImageSubresourceRange> ranges);
      VKHLF_API void copyBuffer(std::shared_ptr<vkhlf::Buffer> const& srcBuffer, std::shared_ptr<vkhlf::Buffer> const& dstBuffer, vk::ArrayProxy<const vk::BufferCopy> regions);
      VKHLF_API void copyBufferToImage(std::shared_ptr<vkhlf::Buffer> const& srcBuffer, std::shared_ptr<vkhlf::Image> const& dstImage, vk::ImageLayout dstImageLayout, vk::ArrayProxy<const vk::BufferImageCopy> regions);
      VKHLF_API void copyImage(std::shared_ptr<vkhlf::Image> const& srcImage, vk::ImageLayout srcImageLayout, std::shared_ptr<vkhlf::Image> const& dstImage, vk::ImageLayout dstImageLayout, vk::ArrayProxy<const vk::ImageCopy> regions);
      VKHLF_API void copyImageToBuffer(std::shared_ptr<vkhlf::Image> const& srcImage, vk::ImageLayout srcImageLayout, std::shared_ptr<vkhlf::Buffer> const& dstBuffer, vk::ArrayProxy<const vk::BufferImageCopy> regions);
      VKHLF_API void copyQueryPoolResults(std::shared_ptr<vkhlf::QueryPool> const& queryPool, uint32_t startQuery, uint32_t queryCount, std::shared_ptr<vkhlf::Buffer> const& dstBuffer, vk::DeviceSize dstOffset, vk::DeviceSize dstStride, vk::QueryResultFlags flags);
      VKHLF_API void dispatch(uint32_t x, uint32_t y, uint32_t z);
      VKHLF_API void dispatchIndirect(std::shared_ptr<vkhlf::Buffer> const& buffer, vk::DeviceSize offset);
      VKHLF_API void draw(uint32_t vertexCount, uint32_t instanceCount, uint32_t firstVertex, uint32_t firstInstance);
      VKHLF_API void drawIndexed(uint32_t indexCount, uint32_t instanceCount, uint32_t firstIndex, int32_t vertexOffset, uint32_t firstInstance);
      VKHLF_API void drawIndexedIndirect(std::shared_ptr<vkhlf::Buffer> const& buffer, vk::DeviceSize offset, uint32_t count, uint32_t stride);
      VKHLF_API void drawIndirect(std::shared_ptr<vkhlf::Buffer> const& buffer, vk::DeviceSize offset, uint32_t count, uint32_t stride);
      VKHLF_API void end();
      VKHLF_API void endQuery(std::shared_ptr<vkhlf::QueryPool> const& queryPool, uint32_t slot);
      VKHLF_API void endRenderPass();
      VKHLF_API void executeCommands(vk::ArrayProxy<const std::shared_ptr<vkhlf::CommandBuffer>> commands);
      VKHLF_API void fillBuffer(std::shared_ptr<vkhlf::Buffer> const& dstBuffer, vk::DeviceSize dstOffset, vk::DeviceSize fillSize, uint32_t data);
      VKHLF_API void nextSubpass(vk::SubpassContents contents);
      VKHLF_API void pipelineBarrier(vk::PipelineStageFlags srcStageMask, vk::PipelineStageFlags destStageMask, vk::DependencyFlags dependencyFlags, vk::ArrayProxy<const vk::MemoryBarrier> barriers, vk::ArrayProxy<const vk::BufferMemoryBarrier> bufferMemoryBarriers, vk::ArrayProxy<const ImageMemoryBarrier> imageMemoryBarriers);
      template <typename T> void pushConstants(vk::PipelineLayout layout, vk::ShaderStageFlags stageFlags, uint32_t start, vk::ArrayProxy<const T> values);
      VKHLF_API void reset(vk::CommandBufferResetFlags flags = {});
      VKHLF_API void resetEvent(std::shared_ptr<vkhlf::Event> const& event, vk::PipelineStageFlags stageMask);
      VKHLF_API void resetQueryPool(std::shared_ptr<vkhlf::QueryPool> const& queryPool, uint32_t startQuery, uint32_t queryCount);
      VKHLF_API void resolveImage(std::shared_ptr<vkhlf::Image> const& srcImage, vk::ImageLayout srcImageLayout, std::shared_ptr<vkhlf::Image> const& dstImage, vk::ImageLayout dstImageLayout, vk::ArrayProxy<const vk::ImageResolve> regions);
      VKHLF_API void setBlendConstants(const float blendConst[4]);
      VKHLF_API void setDepthBias(float depthBias, float depthBiasClamp, float slopeScaledDepthBias);
      VKHLF_API void setDepthBounds(float minDepthBounds, float maxDepthBounds);
      VKHLF_API void setEvent(std::shared_ptr<vkhlf::Event> const& event, vk::PipelineStageFlags stageMask);
      VKHLF_API void setLineWidth(float lineWidth);
      VKHLF_API void setScissor(uint32_t first, vk::ArrayProxy<const vk::Rect2D> scissors);
      VKHLF_API void setStencilCompareMask(vk::StencilFaceFlags faceMask, uint32_t stencilCompareMask);
      VKHLF_API void setStencilReference(vk::StencilFaceFlags faceMask, uint32_t stencilReference);
      VKHLF_API void setStencilWriteMask(vk::StencilFaceFlags faceMask, uint32_t stecilWriteMask);
      VKHLF_API void setViewport(uint32_t first, vk::ArrayProxy<const vk::Viewport> viewports);
      template <typename T> void updateBuffer(std::shared_ptr<vkhlf::Buffer> const & destBuffer, vk::DeviceSize destOffset, vk::ArrayProxy<const T> data);
      VKHLF_API void waitEvents(vk::ArrayProxy<const std::shared_ptr<vkhlf::Event>> events, vk::PipelineStageFlags srcStageMask, vk::PipelineStageFlags dstStageMask, vk::ArrayProxy<const vk::MemoryBarrier> memoryBarriers, vk::ArrayProxy<const vk::BufferMemoryBarrier> bufferMemoryBarriers, vk::ArrayProxy<const vk::ImageMemoryBarrier> imageMemoryBarriers);
      VKHLF_API void writeTimestamp(vk::PipelineStageFlagBits pipelineStage, std::shared_ptr<vkhlf::QueryPool> const& queryPool, uint32_t entry);

      VKHLF_API std::shared_ptr<ResourceTracker> getResourceTracker() const;
      VKHLF_API std::vector<std::shared_ptr<vkhlf::CommandBuffer>> const& getSecondaryCommandBuffers() const;

      VKHLF_API operator vk::CommandBuffer() const;

      VKHLF_API std::shared_ptr<vkhlf::RenderPass> getRenderPass() const { return m_renderPass; }
      VKHLF_API std::shared_ptr<vkhlf::Framebuffer> getFramebuffer() const { return m_framebuffer; }

#if !defined(NDEBUG)
      VKHLF_API std::shared_ptr<vkhlf::CommandBuffer> getPrimaryCommandBuffer() const;     // this belongs into a SecondaryCommandBuffer only
      VKHLF_API bool isOneTimeSubmit() const;
      VKHLF_API bool isRecording() const;
      VKHLF_API bool isSimultaneousUsageAllowed() const;
#endif

      CommandBuffer(CommandBuffer const& rhs) = delete;
      CommandBuffer & operator=(CommandBuffer const& rhs) = delete;

    protected:
      friend class CommandPool;
      friend class Queue;

      VKHLF_API void onReset();

#if !defined(NDEBUG)
    private:
      void setPrimaryCommandBuffer(std::shared_ptr<vkhlf::CommandBuffer> const& primaryCommandBuffer);   // this belongs into a SecondayCommandBuffer only
#endif

    private:
      struct QueryInfo
      {
        bool                  active;
        bool                  contained;
        vk::QueryControlFlags flags;
      };

    private:
      vk::CommandBuffer                                  m_commandBuffer;
      std::shared_ptr<vkhlf::RenderPass>                 m_renderPass;
      std::shared_ptr<vkhlf::Framebuffer>                m_framebuffer;
      std::vector<std::shared_ptr<vkhlf::CommandBuffer>> m_secondaryCommandBuffers;    // this belongs into a PrimaryCommandBuffer only

      // scratch std::vectors for bind operations
      std::vector<::vk::DescriptorSet> m_bindDescriptorSets;
      std::vector<::vk::Buffer>        m_bindVertexBuffers;

      std::shared_ptr<ResourceTracker>  m_resourceTracker;
#if !defined(NDEBUG)
      vk::CommandBufferUsageFlags       m_flags;
      bool                              m_inRenderPass;
      bool                              m_isRecording;
      bool                              m_isResetFromCommandPool;
      vk::CommandBufferLevel            m_level;
      vk::Bool32                        m_occlusionQueryEnable;
      std::weak_ptr<vkhlf::CommandBuffer> m_primaryCommandBuffer;       // this belongs into a SecondayCommandBuffer only
      QueryInfo                         m_queryInfo[VK_QUERY_TYPE_RANGE_SIZE];
      vk::PipelineStageFlags            m_stageFlags;
#endif
  };

  VKHLF_API void setImageLayout(std::shared_ptr<vkhlf::CommandBuffer> const& commandBuffer, std::shared_ptr<vkhlf::Image> const& image, vk::ImageSubresourceRange const& subresourceRange, vk::ImageLayout oldImageLayout, vk::ImageLayout newImageLayout);
  VKHLF_API void setImageLayout(std::shared_ptr<vkhlf::CommandBuffer> const& commandBuffer, std::shared_ptr<vkhlf::Image> const& image, vk::ImageAspectFlags aspectMask, vk::ImageLayout oldImageLayout, vk::ImageLayout newImageLayout);


  inline std::vector<std::shared_ptr<vkhlf::CommandBuffer>> const& CommandBuffer::getSecondaryCommandBuffers() const
  {
    return m_secondaryCommandBuffers;
  }

  inline CommandBuffer::operator vk::CommandBuffer() const
  {
    return m_commandBuffer;
  }

  template <typename T>
  void CommandBuffer::pushConstants(vk::PipelineLayout layout, vk::ShaderStageFlags stageFlags, uint32_t start, vk::ArrayProxy<const T> values)
  {
    m_commandBuffer.pushConstants<T>(layout, stageFlags, start, values);
  }

  template <typename T>
  inline void CommandBuffer::updateBuffer(std::shared_ptr<vkhlf::Buffer> const & destBuffer, vk::DeviceSize destOffset, vk::ArrayProxy<const T> data)
  {
    m_resourceTracker->track(destBuffer);
    m_commandBuffer.updateBuffer<T>(*destBuffer, destOffset, data);
  }


  inline void setImageLayout(std::shared_ptr<vkhlf::CommandBuffer> const& commandBuffer, std::shared_ptr<vkhlf::Image> const& image, vk::ImageAspectFlags aspectMask, vk::ImageLayout oldImageLayout, vk::ImageLayout newImageLayout)
  {
    setImageLayout(commandBuffer, image, vk::ImageSubresourceRange(aspectMask, 0, 1, 0, 1), oldImageLayout, newImageLayout);
  }

} // namespace vk
