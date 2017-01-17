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
#include <vkhlf/PhysicalDevice.h>

namespace vkhlf
{

  class DisplayMode : public Reference<Display>
  {
    public:
      VKHLF_API DisplayMode(std::shared_ptr<Display> const& display, vk::DisplayModeKHR const& displayMode);
      VKHLF_API DisplayMode(std::shared_ptr<Display> const& display, vk::Extent2D const& visibleRegion, uint32_t refreshRate);
      VKHLF_API virtual ~DisplayMode();

      VKHLF_API vk::DisplayPlaneCapabilitiesKHR getPlaneCapabilities(uint32_t planeIndex) const;

      VKHLF_API operator vk::DisplayModeKHR() const;

      DisplayMode(DisplayMode const& rhs) = delete;
      DisplayMode & operator=(DisplayMode const& rhs) = delete;

    private:
      vk::DisplayModeKHR  m_displayMode;
  };

  inline DisplayMode::operator vk::DisplayModeKHR() const
  {
    return m_displayMode;
  }

} // namespace vk
