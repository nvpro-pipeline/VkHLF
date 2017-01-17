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
#include <vkhlf/Types.h>
#include <set>
#include <map>

namespace vkhlf
{
  struct SparseMemoryBind
  {
    VKHLF_API SparseMemoryBind(vk::DeviceSize resourceOffset, vk::DeviceSize size, std::shared_ptr<DeviceMemory> const& memory, vk::DeviceSize memoryOffset, vk::SparseMemoryBindFlags flags);
    VKHLF_API SparseMemoryBind(SparseMemoryBind const& rhs);
    VKHLF_API SparseMemoryBind & operator=(SparseMemoryBind const& rhs);

    vk::DeviceSize                resourceOffset;
    vk::DeviceSize                size;
    std::shared_ptr<DeviceMemory> memory;
    vk::DeviceSize                memoryOffset;
    vk::SparseMemoryBindFlags     flags;
  };

  struct SparseBufferMemoryBindInfo
  {
    VKHLF_API SparseBufferMemoryBindInfo(std::shared_ptr<Buffer> const& buffer, vk::ArrayProxy<const SparseMemoryBind> binds);
    VKHLF_API SparseBufferMemoryBindInfo(SparseBufferMemoryBindInfo const& rhs);
    VKHLF_API SparseBufferMemoryBindInfo & operator=(SparseBufferMemoryBindInfo const& rhs);

    std::shared_ptr<Buffer>       buffer;
    std::vector<SparseMemoryBind> binds;
  };

  struct SparseImageOpaqueMemoryBindInfo
  {
    VKHLF_API SparseImageOpaqueMemoryBindInfo(std::shared_ptr<Image> const& image, vk::ArrayProxy<const SparseMemoryBind> binds);
    VKHLF_API SparseImageOpaqueMemoryBindInfo(SparseImageOpaqueMemoryBindInfo const& rhs);
    VKHLF_API SparseImageOpaqueMemoryBindInfo & operator=(SparseImageOpaqueMemoryBindInfo const& rhs);

    std::shared_ptr<Image>        image;
    std::vector<SparseMemoryBind> binds;
  };

  struct SparseImageMemoryBind
  {
    VKHLF_API SparseImageMemoryBind(vk::ImageSubresource subresource, vk::Offset3D const& offset, vk::Extent3D const& extent, std::shared_ptr<DeviceMemory> const& memory, vk::DeviceSize memoryOffset,
                                 vk::SparseMemoryBindFlags flags);
    VKHLF_API SparseImageMemoryBind(SparseImageMemoryBind const& rhs);
    VKHLF_API SparseImageMemoryBind & operator=(SparseImageMemoryBind const& rhs);

    vk::ImageSubresource          subresource;
    vk::Offset3D                  offset;
    vk::Extent3D                  extent;
    std::shared_ptr<DeviceMemory> memory;
    vk::DeviceSize                memoryOffset;
    vk::SparseMemoryBindFlags     flags;
  };

  struct SparseImageMemoryBindInfo
  {
    VKHLF_API SparseImageMemoryBindInfo(std::shared_ptr<Image> const& image, vk::ArrayProxy<const SparseImageMemoryBind> binds);
    VKHLF_API SparseImageMemoryBindInfo(SparseImageMemoryBindInfo const& rhs);
    VKHLF_API SparseImageMemoryBindInfo & operator=(SparseImageMemoryBindInfo const& rhs);

    std::shared_ptr<Image>              image;
    std::vector<SparseImageMemoryBind>  binds;
  };

  struct BindSparseInfo
  {
    VKHLF_API BindSparseInfo(vk::ArrayProxy<const std::shared_ptr<Semaphore>> waitSemaphores, vk::ArrayProxy<const SparseBufferMemoryBindInfo> bufferBinds,
                          vk::ArrayProxy<const SparseImageOpaqueMemoryBindInfo> imageOpaqueBinds, vk::ArrayProxy<const SparseImageMemoryBindInfo> imageBinds,
                          vk::ArrayProxy<const std::shared_ptr<Semaphore>> signalSemaphores);
    VKHLF_API BindSparseInfo(BindSparseInfo const& rhs);
    VKHLF_API BindSparseInfo & operator=(BindSparseInfo const& rhs);

    std::vector<std::shared_ptr<Semaphore>>       waitSemaphores;
    std::vector<SparseBufferMemoryBindInfo>       bufferBinds;
    std::vector<SparseImageOpaqueMemoryBindInfo>  imageOpaqueBinds;
    std::vector<SparseImageMemoryBindInfo>        imageBinds;
    std::vector<std::shared_ptr<Semaphore>>       signalSemaphores;
  };

  struct SubmitInfo
  {
    VKHLF_API SubmitInfo(vk::ArrayProxy<const std::shared_ptr<Semaphore>> const& waitSemaphores, vk::ArrayProxy<const vk::PipelineStageFlags> waitDstStageMasks,
                      vk::ArrayProxy<const std::shared_ptr<CommandBuffer>> const& commandBuffers, vk::ArrayProxy<const std::shared_ptr<Semaphore>> signalSemaphores);
    VKHLF_API SubmitInfo(SubmitInfo const& rhs);
    VKHLF_API SubmitInfo & operator=(SubmitInfo const& rhs);

    std::vector<std::shared_ptr<Semaphore>>     waitSemaphores;
    std::vector<vk::PipelineStageFlags>         waitDstStageMasks;
    std::vector<std::shared_ptr<CommandBuffer>> commandBuffers;
    std::vector<std::shared_ptr<Semaphore>>     signalSemaphores;
  };

  class Queue : public Reference<Device>
  {
    public:
      VKHLF_API Queue(std::shared_ptr<vkhlf::Device> const& device, uint32_t queueFamilyIndex, uint32_t queueIndex);
      VKHLF_API virtual ~Queue();

      VKHLF_API void bindSparse(vk::ArrayProxy<const BindSparseInfo> bindInfos, std::shared_ptr<Fence> const& fence);
      VKHLF_API std::vector<vk::Result> present(vk::ArrayProxy<const std::shared_ptr<Semaphore>> waitSemaphores, vk::ArrayProxy<const std::shared_ptr<Swapchain>> swapchains,
                                             vk::ArrayProxy<const uint32_t> imageIndices);
      VKHLF_API void submit(vk::ArrayProxy<const SubmitInfo> submitInfos, std::shared_ptr<Fence> const& fence = std::shared_ptr<Fence>());
      
      VKHLF_API void submit(std::shared_ptr<CommandBuffer> const& commandBuffer, std::shared_ptr<Fence> const& fence = std::shared_ptr<Fence>());

      VKHLF_API void waitIdle();

      VKHLF_API operator vk::Queue() const;

      Queue(Queue const& rhs) = delete;
      Queue & operator=(Queue const& rhs) = delete;

    private:
      VKHLF_API void releaseResources();

    private:
      vk::Queue                                                     m_queue;
      std::map<std::shared_ptr<Fence>, std::vector<SubmitInfo>>     m_submitInfos;
      std::map<std::shared_ptr<Fence>, std::vector<BindSparseInfo>> m_bindSparseInfos;

#if !defined(NDEBUG)
      uint32_t  m_familyIndex;
#endif
  };

  VKHLF_API void submitAndWait(std::shared_ptr<Queue> const& queue, std::shared_ptr<CommandBuffer> const& commandBuffer, std::shared_ptr<Allocator> const& fenceAllocator = nullptr);

  inline Queue::operator vk::Queue() const
  {
    return m_queue;
  }

} // namespace vk
