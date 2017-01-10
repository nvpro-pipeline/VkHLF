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
#include <vkhlf/Device.h>
#include <vkhlf/Fence.h>

namespace vkhlf
{

  Fence::Fence( std::shared_ptr<Device> const& device, bool signaled, std::shared_ptr<Allocator> const& allocator )
    : Reference(device, allocator)
  {
    vk::FenceCreateInfo fenceCreateInfo(signaled ? vk::FenceCreateFlagBits::eSignaled : vk::FenceCreateFlags());

    m_fence = static_cast<vk::Device>(*get<Device>()).createFence(fenceCreateInfo, *get<Allocator>());
  }

  Fence::~Fence( )
  {
    // From the spec:
    //    fence must not be associated with any queue command that has not yet completed execution on that queue
    static_cast<vk::Device>(*get<Device>()).destroyFence(m_fence, *get<Allocator>());
  }

  bool Fence::isSignaled() const
  {
    vk::Result result = static_cast<vk::Device>(*get<Device>()).getFenceStatus(m_fence);
    assert( ( result == vk::Result::eSuccess ) || ( result == vk::Result::eNotReady ) );
    return(result == vk::Result::eSuccess);
  }

  void Fence::reset()
  {
    static_cast<vk::Device>(*get<Device>()).resetFences(m_fence);
  }

  void Fence::wait( uint64_t timeout ) const
  {
    static_cast<vk::Device>(*get<Device>()).waitForFences(m_fence, true, timeout);
  }


  void resetFences(vk::ArrayProxy<const std::shared_ptr<vkhlf::Fence>> fences)
  {
    if (!fences.empty())
    {
      std::vector <vk::Fence> fencesArray;
      for (std::shared_ptr<vkhlf::Fence> const& fence : fences)
      {
        assert(fences.front()->get<Device>() == fence->get<Device>());
        fencesArray.push_back(*fence);
      }
      static_cast<vk::Device>(*fences.front()->get<Device>()).resetFences(fencesArray);
    }
  }

  void waitForFences(vk::ArrayProxy<const std::shared_ptr<vkhlf::Fence>> fences, bool all, uint32_t timeout)
  {
    if (!fences.empty())
    {
      std::vector <vk::Fence> fencesArray;
      for (std::shared_ptr<vkhlf::Fence> const& fence : fences)
      {
        assert(fences.front()->get<Device>() == fence->get<Device>());
        fencesArray.push_back(*fence);
      }
      static_cast<vk::Device>(*fences.front()->get<Device>()).waitForFences(fencesArray, all, timeout);
    }
  }

} // namespace vk
