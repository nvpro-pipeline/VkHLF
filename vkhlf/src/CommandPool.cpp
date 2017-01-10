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


#include <vkhlf/Allocator.h>
#include <vkhlf/CommandPool.h>
#include <vkhlf/CommandBuffer.h>
#include <vkhlf/Device.h>
#include <vkhlf/PhysicalDevice.h>

namespace vkhlf
{

  CommandPool::CommandPool(std::shared_ptr<Device> const & device, vk::CommandPoolCreateFlags flags, uint32_t familyIndex, std::shared_ptr<Allocator> const& allocator)
    : Reference(device, allocator)
#if !defined(NDEBUG)
    , m_createFlags(flags)
    , m_familyIndex(familyIndex)
#endif
  {
    assert(familyIndex < device->getQueueFamilyCount());

    vk::CommandPoolCreateInfo info(flags, familyIndex);
    m_commandPool = static_cast<vk::Device>(*get<Device>()).createCommandPool(info, *get<Allocator>());
  }

  CommandPool::~CommandPool()
  {
    assert( m_commandBuffers.empty() );   // due to our ref-counting, this CommandPool can only go when all its CommandBuffers are already gone!
                                          // that, means, none of the CommandBuffers are pending execution !!
    static_cast<vk::Device>(*get<Device>()).destroyCommandPool(m_commandPool, *get<Allocator>());
  }

  std::shared_ptr<vkhlf::CommandBuffer> CommandPool::allocateCommandBuffer(vk::CommandBufferLevel level)
  {
    std::shared_ptr<vkhlf::CommandBuffer> commandBuffer = std::make_shared<CommandBuffer>(shared_from_this(), level);
#if !defined(NDEBUG)
    m_commandBuffers.push_back( commandBuffer.get() );
#endif
    return( commandBuffer );
  }

  void CommandPool::reset(vk::CommandPoolResetFlags flags)
  {
#if !defined(NDEBUG)
    for ( std::vector<vkhlf::CommandBuffer*>::iterator it = m_commandBuffers.begin() ; it != m_commandBuffers.end() ; ++it )
    {
      (*it)->onReset();
    }
#endif
    static_cast<vk::Device>(*get<Device>()).resetCommandPool(m_commandPool, flags);
  }

#if !defined(NDEBUG)
  uint32_t CommandPool::getFamilyIndex() const
  {
    return m_familyIndex;
  }

  bool CommandPool::individuallyResetCommandBuffers() const
  {
    return !!(m_createFlags & vk::CommandPoolCreateFlagBits::eResetCommandBuffer);
  }

  bool CommandPool::shortLivedCommandBuffers() const
  {
    return !!(m_createFlags & vk::CommandPoolCreateFlagBits::eTransient);
  }

  bool CommandPool::supportsCompute() const
  {
    return !!(get<Device>()->get<PhysicalDevice>()->getQueueFamilyProperties()[m_familyIndex].queueFlags & vk::QueueFlagBits::eCompute);
  }

  bool CommandPool::supportsGraphics() const
  {
    return !!(get<Device>()->get<PhysicalDevice>()->getQueueFamilyProperties()[m_familyIndex].queueFlags & vk::QueueFlagBits::eGraphics);
  }

  bool CommandPool::supportsTransfer() const
  {
    return !!(get<Device>()->get<PhysicalDevice>()->getQueueFamilyProperties()[m_familyIndex].queueFlags & vk::QueueFlagBits::eTransfer);
  }

  void CommandPool::onDeleteCommandBuffer( vkhlf::CommandBuffer const* commandBuffer )
  {
    std::vector<vkhlf::CommandBuffer*>::iterator it = std::find( m_commandBuffers.begin(), m_commandBuffers.end(), commandBuffer );
    assert(it != m_commandBuffers.end());
    m_commandBuffers.erase(it);
  }

#endif

} // namespace vk
