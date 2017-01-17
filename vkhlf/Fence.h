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
#include <vkhlf/Reference.h>
#include <vector>

namespace vkhlf
{

  class Fence : public Reference<Device,Allocator>
  {
    public:
      VKHLF_API Fence(std::shared_ptr<Device> const& device, bool signaled, std::shared_ptr<Allocator> const& allocator);
      VKHLF_API virtual ~Fence();

      VKHLF_API bool isSignaled() const;
      VKHLF_API void reset();
      VKHLF_API void wait(uint64_t timeout) const;

      VKHLF_API operator vk::Fence() const;

      Fence(Fence const& rhs) = delete;
      Fence & operator=(Fence const& rhs) = delete;

    private:
      vk::Fence m_fence;
  };

  // Wait for all fences in the vector. All fences must be part of the same device.
  VKHLF_API void waitForFences(vk::ArrayProxy<const std::shared_ptr<vkhlf::Fence>> fences, bool all, uint32_t timeout);
  VKHLF_API void resetFences(vk::ArrayProxy<const std::shared_ptr<vkhlf::Fence>> fences);

  inline Fence::operator vk::Fence() const
  {
    return m_fence;
  }

} // namespace vk
