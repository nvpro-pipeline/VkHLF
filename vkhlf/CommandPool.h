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
#include <vkhlf/Device.h>
#if !defined(NDEBUG)
#include <vector>
#endif

namespace vkhlf
{

  class CommandPool : public Reference<Device, Allocator>, public std::enable_shared_from_this<CommandPool>
  {
    public:
      VKHLF_API CommandPool(std::shared_ptr<Device> const & device, vk::CommandPoolCreateFlags flags = vk::CommandPoolCreateFlags(), uint32_t familyIndex = 0,
                         std::shared_ptr<Allocator> const& allocator = nullptr);
      VKHLF_API virtual ~CommandPool();

      VKHLF_API std::shared_ptr<vkhlf::CommandBuffer>  allocateCommandBuffer(vk::CommandBufferLevel level = vk::CommandBufferLevel::ePrimary);
      VKHLF_API void                                 reset(vk::CommandPoolResetFlags flags);

      VKHLF_API operator vk::CommandPool() const;

      CommandPool(CommandPool const& rhs) = delete;
      CommandPool & operator=(CommandPool const& rhs) = delete;

#if !defined(NDEBUG)
      VKHLF_API uint32_t getFamilyIndex() const;
      VKHLF_API bool individuallyResetCommandBuffers() const;
      VKHLF_API bool shortLivedCommandBuffers() const;
      VKHLF_API bool supportsCompute() const;
      VKHLF_API bool supportsGraphics() const;
      VKHLF_API bool supportsTransfer() const;

    private:
      friend class CommandBuffer;

      VKHLF_API void onDeleteCommandBuffer( vkhlf::CommandBuffer const* commandBuffer );
#endif

    private:
      vk::CommandPool m_commandPool;

#if !defined(NDEBUG)
      std::vector<vkhlf::CommandBuffer*>  m_commandBuffers;
      vk::CommandPoolCreateFlags        m_createFlags;
      uint32_t                          m_familyIndex;
#endif
  };

  inline CommandPool::operator vk::CommandPool() const
  {
    return m_commandPool;
  }

} // namespace vk
