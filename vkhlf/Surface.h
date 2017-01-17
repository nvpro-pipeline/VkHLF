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

#include <vkhlf/Allocator.h>
#include <vkhlf/Config.h>
#include <vkhlf/Reference.h>
#include <vkhlf/Types.h>

namespace vkhlf
{
  class Surface : public Reference<Instance, Allocator>
  {
    public:
      VKHLF_API Surface(std::shared_ptr<Instance> const& instance, vk::SurfaceKHR const& surface, std::shared_ptr<Allocator> const& allocator);
      VKHLF_API Surface(std::shared_ptr<Instance> const& instance, std::shared_ptr<DisplayMode> const& displayMode, uint32_t planeIndex, uint32_t planeStackIndex,
                     vk::SurfaceTransformFlagBitsKHR transform, float globalAlpha, vk::DisplayPlaneAlphaFlagBitsKHR alphaMode, vk::Extent2D const& imageExtent,
                     std::shared_ptr<Allocator> const& allocator);
      VKHLF_API ~Surface();

      VKHLF_API operator vk::SurfaceKHR() const;

      Surface(Surface const& rhs) = delete;
      Surface & operator=(Surface const& rhs) = delete;

    private:
    vk::SurfaceKHR  m_surface;
  };

  inline Surface::operator vk::SurfaceKHR() const
  {
    return m_surface;
  }

} // namespace vk
