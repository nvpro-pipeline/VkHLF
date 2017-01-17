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
#include <vkhlf/Device.h>
#include <vkhlf/DeviceMemoryChunk.h>
#include <vkhlf/Fence.h>
#include <vkhlf/Image.h>
#include <vkhlf/Queue.h>
#include <vkhlf/ResourceTracker.h>
#include <vkhlf/Semaphore.h>
#include <vkhlf/Swapchain.h>
#include <tuple>

namespace vkhlf
{
  SparseMemoryBind::SparseMemoryBind(vk::DeviceSize resourceOffset_, vk::DeviceSize size_, std::shared_ptr<DeviceMemory> const& memory_, vk::DeviceSize memoryOffset_, vk::SparseMemoryBindFlags flags_)
    : resourceOffset(resourceOffset_)
    , size(size_)
    , memory(memory_)
    , memoryOffset(memoryOffset_)
    , flags(flags_)
  {}
  
  SparseMemoryBind::SparseMemoryBind(SparseMemoryBind const& rhs)
    : SparseMemoryBind(rhs.resourceOffset, rhs.size, rhs.memory, rhs.memoryOffset, rhs.flags)
  {}

  SparseMemoryBind & SparseMemoryBind::operator = (SparseMemoryBind const& rhs)
  {
    resourceOffset  = rhs.resourceOffset;
    size            = rhs.size;
    memory          = rhs.memory;
    memoryOffset    = rhs.memoryOffset;
    flags           = rhs.flags;
    return *this;
  }


  SparseBufferMemoryBindInfo::SparseBufferMemoryBindInfo(std::shared_ptr<Buffer> const& buffer_, vk::ArrayProxy<const SparseMemoryBind> binds_)
    : buffer(buffer_)
    , binds(binds_.begin(), binds_.end())
  {}

  SparseBufferMemoryBindInfo::SparseBufferMemoryBindInfo(SparseBufferMemoryBindInfo const& rhs)
    : SparseBufferMemoryBindInfo(rhs.buffer, rhs.binds)
  {}

  SparseBufferMemoryBindInfo & SparseBufferMemoryBindInfo::operator = (SparseBufferMemoryBindInfo const& rhs)
  {
    buffer  = rhs.buffer;
    binds   = rhs.binds;
    return *this;
  }


  SparseImageOpaqueMemoryBindInfo::SparseImageOpaqueMemoryBindInfo(std::shared_ptr<Image> const& image_, vk::ArrayProxy<const SparseMemoryBind> binds_)
    : image(image_)
    , binds(binds_.begin(), binds_.end())
  {}

  SparseImageOpaqueMemoryBindInfo::SparseImageOpaqueMemoryBindInfo(SparseImageOpaqueMemoryBindInfo const& rhs)
    : SparseImageOpaqueMemoryBindInfo(rhs.image, rhs.binds)
  {}

  SparseImageOpaqueMemoryBindInfo & SparseImageOpaqueMemoryBindInfo::operator = (SparseImageOpaqueMemoryBindInfo const& rhs)
  {
    image = rhs.image;
    binds = rhs.binds;
    return *this;
  }


  SparseImageMemoryBind::SparseImageMemoryBind(vk::ImageSubresource subresource_, vk::Offset3D const& offset_, vk::Extent3D const& extent_, std::shared_ptr<DeviceMemory> const& memory_,
                                               vk::DeviceSize memoryOffset_, vk::SparseMemoryBindFlags flags_)
    : subresource(subresource_)
    , offset(offset_)
    , extent(extent_)
    , memory(memory_)
    , memoryOffset(memoryOffset_)
    , flags(flags_)
  {}

  SparseImageMemoryBind::SparseImageMemoryBind(SparseImageMemoryBind const& rhs)
    : SparseImageMemoryBind(rhs.subresource, rhs.offset, rhs.extent, rhs.memory, rhs.memoryOffset, rhs.flags)
  {}

  SparseImageMemoryBind & SparseImageMemoryBind::operator = (SparseImageMemoryBind const& rhs)
  {
    subresource   = rhs.subresource;
    offset        = rhs.offset;
    extent        = rhs.extent;
    memory        = rhs.memory;
    memoryOffset  = rhs.memoryOffset;
    flags         = rhs.flags;
    return *this;
  }


  SparseImageMemoryBindInfo::SparseImageMemoryBindInfo(std::shared_ptr<Image> const& image_, vk::ArrayProxy<const SparseImageMemoryBind> binds_)
    : image(image_)
    , binds(binds_.begin(), binds_.end())
  {}

  SparseImageMemoryBindInfo::SparseImageMemoryBindInfo(SparseImageMemoryBindInfo const& rhs)
    : SparseImageMemoryBindInfo(rhs.image, rhs.binds)
  {}

  SparseImageMemoryBindInfo & SparseImageMemoryBindInfo::operator = (SparseImageMemoryBindInfo const& rhs)
  {
    image = rhs.image;
    binds = rhs.binds;
    return *this;
  }


  BindSparseInfo::BindSparseInfo(vk::ArrayProxy<const std::shared_ptr<Semaphore>> waitSemaphores_, vk::ArrayProxy<const SparseBufferMemoryBindInfo> bufferBinds_,
                                 vk::ArrayProxy<const SparseImageOpaqueMemoryBindInfo> imageOpaqueBinds_, vk::ArrayProxy<const SparseImageMemoryBindInfo> imageBinds_,
                                 vk::ArrayProxy<const std::shared_ptr<Semaphore>> signalSemaphores_)
    : waitSemaphores(waitSemaphores_.begin(), waitSemaphores_.end())
    , bufferBinds(bufferBinds_.begin(), bufferBinds_.end())
    , imageOpaqueBinds(imageOpaqueBinds_.begin(), imageOpaqueBinds_.end())
    , imageBinds(imageBinds_.begin(), imageBinds_.end())
    , signalSemaphores(signalSemaphores_.begin(), signalSemaphores_.end())
  {}

  BindSparseInfo::BindSparseInfo(BindSparseInfo const& rhs)
    : BindSparseInfo(rhs.waitSemaphores, rhs.bufferBinds, rhs.imageOpaqueBinds, rhs.imageBinds, rhs.signalSemaphores)
  {}

  BindSparseInfo & BindSparseInfo::operator = (BindSparseInfo const& rhs)
  {
    waitSemaphores    = rhs.waitSemaphores;
    bufferBinds       = rhs.bufferBinds;
    imageOpaqueBinds  = rhs.imageOpaqueBinds;
    imageBinds        = rhs.imageBinds;
    signalSemaphores  = rhs.signalSemaphores;
    return *this;
  }


  SubmitInfo::SubmitInfo(vk::ArrayProxy<const std::shared_ptr<Semaphore>> const& waitSemaphores_, vk::ArrayProxy<const vk::PipelineStageFlags> waitDstStageMasks_,
                         vk::ArrayProxy<const std::shared_ptr<CommandBuffer>> const& commandBuffers_, vk::ArrayProxy<const std::shared_ptr<Semaphore>> signalSemaphores_)
    : waitSemaphores(waitSemaphores_.begin(), waitSemaphores_.end())
    , waitDstStageMasks(waitDstStageMasks_.begin(), waitDstStageMasks_.end())
    , commandBuffers(commandBuffers_.begin(), commandBuffers_.end())
    , signalSemaphores(signalSemaphores_.begin(), signalSemaphores_.end())
  {
    assert(waitSemaphores.size() == waitDstStageMasks.size());
  }

  SubmitInfo::SubmitInfo(SubmitInfo const& rhs)
    : SubmitInfo(rhs.waitSemaphores, rhs.waitDstStageMasks, rhs.commandBuffers, rhs.signalSemaphores)
  {}

  SubmitInfo & SubmitInfo::operator=(SubmitInfo const& rhs)
  {
    waitSemaphores    = rhs.waitSemaphores;
    waitDstStageMasks = rhs.waitDstStageMasks;
    commandBuffers    = rhs.commandBuffers;
    signalSemaphores  = rhs.signalSemaphores;
    return *this;
  }


  Queue::Queue(std::shared_ptr<vkhlf::Device> const& device, vk::Queue queue)
    : m_device(device)
    , m_queue(queue)
  {

  }

  Queue::~Queue( )
  {
    releaseResources();
  }

  void Queue::bindSparse(vk::ArrayProxy<const BindSparseInfo> bindSparseInfos, std::shared_ptr<vkhlf::Fence> const& fenceIn)
  {
    // if this Queue has been used before release resources for completed fence. It's a requirement of vulkan not to reuse
    // a fence which hasn't been completed. Thus there shouldn't be an entry in the map using the specified fence.
    releaseResources();

    std::shared_ptr<Fence> fence = fenceIn ? fenceIn : getDevice()->createFence(false);
#if !defined(NDEBUG)
    assert(!fence->isSignaled());
    assert(m_bindSparseInfos.find(fence) == m_bindSparseInfos.end());
#endif

    m_bindSparseInfos.insert(std::make_pair(fence, std::vector<BindSparseInfo>(bindSparseInfos.begin(), bindSparseInfos.end())));

    std::vector<std::vector<vk::Semaphore>> waitSemaphores;
    waitSemaphores.reserve(bindSparseInfos.size());

    std::vector<std::vector<std::vector<vk::SparseMemoryBind>>> bufferBindBinds;
    bufferBindBinds.reserve(bindSparseInfos.size());

    std::vector<std::vector<vk::SparseBufferMemoryBindInfo>> bufferBinds;
    bufferBinds.reserve(bindSparseInfos.size());

    std::vector<std::vector<std::vector<vk::SparseMemoryBind>>> imageOpaqueBindBinds;
    imageOpaqueBindBinds.reserve(bindSparseInfos.size());

    std::vector<std::vector<vk::SparseImageOpaqueMemoryBindInfo>> imageOpaqueBinds;
    imageOpaqueBinds.reserve(bindSparseInfos.size());

    std::vector<std::vector<std::vector<vk::SparseImageMemoryBind>>> imageBindBinds;
    imageBindBinds.reserve(bindSparseInfos.size());

    std::vector<std::vector<vk::SparseImageMemoryBindInfo>> imageBinds;
    imageBinds.reserve(bindSparseInfos.size());

    std::vector<std::vector<vk::Semaphore>> signalSemaphores;
    signalSemaphores.reserve(bindSparseInfos.size());

    std::vector<vk::BindSparseInfo> vkBindSparseInfos;
    vkBindSparseInfos.reserve(bindSparseInfos.size());
    for (auto const& bsi : bindSparseInfos)
    {
      waitSemaphores.push_back(std::vector<vk::Semaphore>());
      waitSemaphores.back().reserve(bsi.waitSemaphores.size());
      for (auto const& s : bsi.waitSemaphores)
      {
        waitSemaphores.back().push_back(s ? static_cast<vk::Semaphore>(*s) : nullptr);
      }

      bufferBindBinds.push_back(std::vector<std::vector<vk::SparseMemoryBind>>());
      bufferBindBinds.back().reserve(bsi.bufferBinds.size());
      bufferBinds.push_back(std::vector<vk::SparseBufferMemoryBindInfo>());
      bufferBinds.back().reserve(bsi.bufferBinds.size());
      for (auto const& b : bsi.bufferBinds)
      {
        bufferBindBinds.back().push_back(std::vector<vk::SparseMemoryBind>());
        bufferBindBinds.back().back().reserve(b.binds.size());
        for (auto const& bb : b.binds)
        {
          bufferBindBinds.back().back().push_back(vk::SparseMemoryBind(bb.resourceOffset, bb.size, bb.memory ? static_cast<vk::DeviceMemory>(*bb.memory->get<DeviceMemoryChunk>()) : nullptr,
                                                                       bb.memoryOffset + (bb.memory ? bb.memory->getOffset() : 0), bb.flags));
        }
        bufferBinds.back().push_back(vk::SparseBufferMemoryBindInfo(b.buffer ? static_cast<vk::Buffer>(*b.buffer) : nullptr, vkhlf::checked_cast<uint32_t>(bufferBindBinds.back().back().size()),
                                                                    bufferBindBinds.back().back().data()));
      }

      imageOpaqueBindBinds.push_back(std::vector<std::vector<vk::SparseMemoryBind>>());
      imageOpaqueBindBinds.back().reserve(bsi.imageOpaqueBinds.size());
      imageOpaqueBinds.push_back(std::vector<vk::SparseImageOpaqueMemoryBindInfo>());
      imageOpaqueBinds.back().reserve(bsi.imageOpaqueBinds.size());
      for (auto const& b : bsi.imageOpaqueBinds)
      {
        imageOpaqueBindBinds.back().push_back(std::vector<vk::SparseMemoryBind>());
        imageOpaqueBindBinds.back().back().reserve(b.binds.size());
        for (auto const& bb : b.binds)
        {
          imageOpaqueBindBinds.back().back().push_back(vk::SparseMemoryBind(bb.resourceOffset, bb.size, bb.memory ? static_cast<vk::DeviceMemory>(*bb.memory->get<DeviceMemoryChunk>()) : nullptr,
                                                                            bb.memoryOffset + (bb.memory ? bb.memory->getOffset() : 0), bb.flags));
        }
        imageOpaqueBinds.back().push_back(vk::SparseImageOpaqueMemoryBindInfo(b.image ? static_cast<vk::Image>(*b.image) : nullptr, vkhlf::checked_cast<uint32_t>(imageOpaqueBindBinds.back().back().size()),
                                                                              imageOpaqueBindBinds.back().back().data()));
      }

      imageBindBinds.push_back(std::vector<std::vector<vk::SparseImageMemoryBind>>());
      imageBindBinds.back().reserve(bsi.imageBinds.size());
      imageBinds.push_back(std::vector<vk::SparseImageMemoryBindInfo>());
      imageBinds.back().reserve(bsi.imageBinds.size());
      for (auto const& b : bsi.imageBinds)
      {
        imageBindBinds.back().push_back(std::vector<vk::SparseImageMemoryBind>());
        imageBindBinds.back().back().reserve(b.binds.size());
        for (auto const& bb : b.binds)
        {
          imageBindBinds.back().back().push_back(vk::SparseImageMemoryBind(bb.subresource, bb.offset, bb.extent, bb.memory ? static_cast<vk::DeviceMemory>(*bb.memory->get<DeviceMemoryChunk>()) : nullptr,
                                                                           bb.memoryOffset + (bb.memory ? bb.memory->getOffset() : 0), bb.flags));
        }
        imageBinds.back().push_back(vk::SparseImageMemoryBindInfo(b.image ? static_cast<vk::Image>(*b.image) : nullptr, vkhlf::checked_cast<uint32_t>(imageBindBinds.back().back().size()),
                                                                  imageBindBinds.back().back().data()));
      }

      signalSemaphores.push_back(std::vector<vk::Semaphore>());
      signalSemaphores.back().reserve(bsi.signalSemaphores.size());
      for (auto const& s : bsi.signalSemaphores)
      {
        signalSemaphores.back().push_back(s ? static_cast<vk::Semaphore>(*s) : nullptr);
      }

      vkBindSparseInfos.push_back(vk::BindSparseInfo(vkhlf::checked_cast<uint32_t>(waitSemaphores.back().size()), waitSemaphores.back().data(),
                                                     vkhlf::checked_cast<uint32_t>(bufferBinds.back().size()), bufferBinds.back().data(),
                                                     vkhlf::checked_cast<uint32_t>(imageOpaqueBinds.back().size()), imageOpaqueBinds.back().data(),
                                                     vkhlf::checked_cast<uint32_t>(imageBinds.back().size()), imageBinds.back().data(),
                                                     vkhlf::checked_cast<uint32_t>(signalSemaphores.back().size()), signalSemaphores.back().data()));
    }

    m_queue.bindSparse(vkBindSparseInfos, *fence);
  }

  static std::shared_ptr<vkhlf::Device> const& findADevice(SubmitInfo const& si)
  {
    if (!si.waitSemaphores.empty())
    {
      return si.waitSemaphores.front()->get<Device>();
    }
    if (!si.commandBuffers.empty())
    {
      return si.commandBuffers.front()->get<CommandPool>()->get<Device>();
    }
    if (!si.signalSemaphores.empty())
    {
      return si.signalSemaphores.front()->get<Device>();
    }

    static const std::shared_ptr<vkhlf::Device> dummy;
    return dummy;
  }

  std::vector<vk::Result> Queue::present(vk::ArrayProxy<const std::shared_ptr<Semaphore>> waitSemaphores, vk::ArrayProxy<const std::shared_ptr<Swapchain>> swapchains,
                                         vk::ArrayProxy<const uint32_t> imageIndices)
  {
    assert(swapchains.size() == imageIndices.size());

    std::vector<vk::Semaphore> waitSemaphoreData;
    waitSemaphoreData.reserve(waitSemaphores.size());
    for (auto const& s : waitSemaphores)
    {
      waitSemaphoreData.push_back(*s);
    }

    std::vector<vk::SwapchainKHR> swapchainData;
    swapchainData.reserve(swapchains.size());
    for (auto const& s : swapchains)
    {
      swapchainData.push_back(static_cast<vk::SwapchainKHR>(*s));
    }

    std::vector<vk::Result> results(swapchains.size());
    m_queue.presentKHR(vk::PresentInfoKHR(vkhlf::checked_cast<uint32_t>(waitSemaphoreData.size()), waitSemaphoreData.data(), vkhlf::checked_cast<uint32_t>(swapchainData.size()), swapchainData.data(),
                                          imageIndices.data(), results.data()));
    return results;
  }

  void Queue::submit(vk::ArrayProxy<const SubmitInfo> submitInfos, std::shared_ptr<vkhlf::Fence> const& fenceIn)
  {
    // if this Queue has beed used before, then there is already a Fence -> wait 'til it's complete to free the previous resources first
    releaseResources();

    // create a new fence if none has been passed to track completion of the submit.
    std::shared_ptr<Fence> fence = fenceIn ? fenceIn : getDevice()->createFence(false);

    m_submitInfos.insert(std::make_pair(fence, std::vector<SubmitInfo>(submitInfos.begin(), submitInfos.end())));

    // TBD: this queue should be passed to a worker thread
    //      that worker thread polls the status of the fence,
    //      when the fence has been signaled, all the corresponding commandBuffers are signaled as not pending any more
#if !defined(NDEBUG)
    // From the spec:
    //    pWaitDstStageMask is a pointer to an array of pipeline stages at which each corresponding semaphore wait will occur.
    // -> I don't see any pWaitDstStageMask in our VkSubmitInfo !?
    for (auto const& si : submitInfos)
    {
      std::shared_ptr<vkhlf::Device> const& device = findADevice(si);
      if (device)
      {
        for ( std::vector<std::shared_ptr<vkhlf::Semaphore>>::const_iterator it = si.waitSemaphores.begin() ; it != si.waitSemaphores.end() ; ++it )
        {
          assert(device == (*it)->get<Device>());
        }
        for ( std::vector<std::shared_ptr<vkhlf::CommandBuffer>>::const_iterator it = si.commandBuffers.begin() ; it != si.commandBuffers.end() ; ++it )
        {
          assert(device == (*it)->get<CommandPool>()->get<Device>());
          assert((*it)->isSimultaneousUsageAllowed() || !(*it)->getResourceTracker()->isUsed());
          assert(!(*it)->isRecording());
          for ( std::vector<std::shared_ptr<vkhlf::CommandBuffer>>::const_iterator sit = (*it)->getSecondaryCommandBuffers().begin() ; sit != (*it)->getSecondaryCommandBuffers().end() ; ++sit )
          {
            assert((*sit)->isSimultaneousUsageAllowed() || !(*it)->getResourceTracker()->isUsed());
            assert((*sit)->getPrimaryCommandBuffer() == *it);
          }
        }
        for ( std::vector<std::shared_ptr<vkhlf::Semaphore>>::const_iterator it = si.signalSemaphores.begin() ; it != si.signalSemaphores.end() ; ++it )
        {
          assert(device == (*it)->get<Device>());
          assert(!(*it)->isSignalled());
        }
      }
    }
    // From the spec:
    //    Any given element of pCommandBuffers must either have been recorded with the VK_COMMAND_
    //    BUFFER_USAGE_SIMULTANEOUS_USE_BIT, or not currently be executing on the device
    //    If any given element of pCommandBuffers contains references to secondary command buffers, those
    //    secondary command buffers must have been recorded with the VK_COMMAND_BUFFER_USAGE_
    //    SIMULTANEOUS_USE_BIT, or not currently be executing on the device
    // -> how should the execution state on the device be checked? And is execution on an other device allowed?
    // From the spec:
    //    Any given element of VkSemaphore in pWaitSemaphores must refer to a prior signal of that VkSemaphore
    //    that won’t be consumed by any other wait on that semaphore
    // -> what does that mean?
    // From the spec: two more things referring to pWaitDstStageMask:
    //    If the geometry shaders feature is not enabled, any given element of pWaitDstStageMask must not contain
    //    VK_PIPELINE_STAGE_GEOMETRY_SHADER_BIT
    //    If the tessellation shaders feature is not enabled, any given element of pWaitDstStageMask must not contain
    //    VK_PIPELINE_STAGE_TESSELLATION_CONTROL_SHADER_BIT or VK_PIPELINE_STAGE_
    //    TESSELLATION_EVALUATION_SHADER_BIT
#endif

    std::vector<std::vector<vk::Semaphore>> waitSemaphores;
    waitSemaphores.reserve(submitInfos.size());

    std::vector<std::vector<vk::CommandBuffer>> commandBuffers;
    commandBuffers.reserve(submitInfos.size());

    std::vector<std::vector<vk::Semaphore>> signalSemaphores;
    signalSemaphores.reserve(submitInfos.size());

    std::vector<vk::SubmitInfo> vkSubmitInfos;
    vkSubmitInfos.reserve(submitInfos.size());
    for (auto const& si : submitInfos)
    {
      assert(si.waitSemaphores.size() == si.waitDstStageMasks.size());

      waitSemaphores.push_back(std::vector<vk::Semaphore>());
      waitSemaphores.back().reserve(si.waitSemaphores.size());
      for (auto const& s : si.waitSemaphores)
      {
        waitSemaphores.back().push_back(s ? static_cast<vk::Semaphore>(*s) : nullptr);
      }

      commandBuffers.push_back(std::vector<vk::CommandBuffer>());
      commandBuffers.back().reserve(si.commandBuffers.size());
      for (auto const& cb : si.commandBuffers)
      {
        commandBuffers.back().push_back(cb ? static_cast<vk::CommandBuffer>(*cb) : nullptr);
      }

      signalSemaphores.push_back(std::vector<vk::Semaphore>());
      signalSemaphores.back().reserve(si.signalSemaphores.size());
      for (auto const& s : si.signalSemaphores)
      {
        signalSemaphores.back().push_back(s ? static_cast<vk::Semaphore>(*s) : nullptr);
      }

      vkSubmitInfos.push_back(vk::SubmitInfo(vkhlf::checked_cast<uint32_t>(waitSemaphores.back().size()), waitSemaphores.back().data(), si.waitDstStageMasks.data(),
                                             vkhlf::checked_cast<uint32_t>(commandBuffers.back().size()), commandBuffers.back().data(),
                                             vkhlf::checked_cast<uint32_t>(signalSemaphores.back().size()), signalSemaphores.back().data()));
    }

    m_queue.submit(vkSubmitInfos, *fence);
  }

  void Queue::submit(std::shared_ptr<vkhlf::CommandBuffer> const& commandBuffer, std::shared_ptr<vkhlf::Fence> const& fence)
  {
    submit(SubmitInfo(nullptr, nullptr, commandBuffer, nullptr), fence);
  }

  void Queue::waitIdle()
  {
    m_queue.waitIdle();
    releaseResources();
  }

  void Queue::releaseResources()
  {
    // remove completed fences from submitInfo map and update attached CommandBuffers
    std::vector<std::shared_ptr<Fence>> completedFences;
    for (auto const & submitInfoEntry : m_submitInfos)
    {
      if (submitInfoEntry.first->isSignaled())
      {
        for (auto const & submitInfo : submitInfoEntry.second)
        {
          for (auto const & commandBuffer : submitInfo.commandBuffers)
          {
            // isUsed removes completed fences
            commandBuffer->getResourceTracker()->isUsed();
          }
        }
        completedFences.push_back(submitInfoEntry.first);
      }
    }
    
    for (auto const & fence : completedFences)
    {
      m_submitInfos.erase(fence);
    }

    // remove completed fences from sparseInfo map
    completedFences.clear();
    for (auto const & bindSparseInfo : m_bindSparseInfos)
    {
      if (bindSparseInfo.first->isSignaled())
      {
        completedFences.push_back(bindSparseInfo.first);
      }
    }

    for (auto const & fence : completedFences)
    {
      m_submitInfos.erase(fence);
    }
  }

  void submitAndWait(std::shared_ptr<Queue> const& queue, std::shared_ptr<CommandBuffer> const& commandBuffer, std::shared_ptr<Allocator> const& fenceAllocator)
  {
    std::shared_ptr<Fence> fence = queue->getDevice()->createFence(false, fenceAllocator);
    queue->submit(commandBuffer, fence);

    vk::Result vkRes;
    do
    {
      vkRes = queue->getDevice()->waitForFences(fence, true, 0);    // some timeout ?
    } while (vkRes == vk::Result::eTimeout);
    assert(vkRes == vk::Result::eSuccess);
  }

} // namespace vk

