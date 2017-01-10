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


#include <vkhlf/Device.h>
#include <vkhlf/DisplayMode.h>
#include <vkhlf/Fence.h>
#include <vkhlf/Image.h>
#include <vkhlf/Instance.h>
#include <vkhlf/Semaphore.h>
#include <vkhlf/Surface.h>

namespace vkhlf
{
  Surface::Surface(std::shared_ptr<Instance> const& instance, vk::SurfaceKHR const& surface, std::shared_ptr<Allocator> const& allocator)
    : Reference(instance, allocator)
    , m_surface(surface)
  {}

  Surface::Surface(std::shared_ptr<Instance> const& instance, std::shared_ptr<DisplayMode> const& displayMode, uint32_t planeIndex, uint32_t planeStackIndex,
                   vk::SurfaceTransformFlagBitsKHR transform, float globalAlpha, vk::DisplayPlaneAlphaFlagBitsKHR alphaMode, vk::Extent2D const& imageExtent,
                   std::shared_ptr<Allocator> const& allocator)
    : Reference(instance, allocator)
  {
    vk::DisplaySurfaceCreateInfoKHR createInfo({}, static_cast<vk::DisplayModeKHR>(*displayMode), planeIndex, planeStackIndex, transform, globalAlpha, alphaMode, imageExtent);
    m_surface = static_cast<vk::Instance>(*get<Instance>()).createDisplayPlaneSurfaceKHR(createInfo, *get<Allocator>());
  }

  Surface::~Surface()
  {
    static_cast<vk::Instance>(*get<Instance>()).destroySurfaceKHR(m_surface, *get<Allocator>());
  }

} // namespace vk
