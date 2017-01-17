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

namespace vkhlf
{
  class Swapchain : public Reference<Device, Allocator>
  {
    public:
      VKHLF_API Swapchain(std::shared_ptr<Device> const & device, std::shared_ptr<Surface> const& surface, uint32_t minImageCount, vk::Format imageFormat, vk::Extent2D const& imageExtent,
                       uint32_t imageArrayLayers, vk::ImageUsageFlags imageUsage, vk::SharingMode imageSharingMode, vk::ArrayProxy<const uint32_t> queueFamilyIndices,
                       vk::SurfaceTransformFlagBitsKHR preTransform, vk::CompositeAlphaFlagBitsKHR compositeAlpha, vk::PresentModeKHR presentMode, bool clipped,
                       std::shared_ptr<Swapchain> const& oldSwapchain, std::shared_ptr<Allocator> const& allocator);
      VKHLF_API virtual ~Swapchain();

      VKHLF_API uint32_t acquireNextImage(uint64_t timeout = UINT64_MAX, std::shared_ptr<Fence> const& fence = {});

      VKHLF_API std::vector<std::shared_ptr<vkhlf::Image>> const& getImages() const;
      VKHLF_API std::vector<std::shared_ptr<vkhlf::Semaphore>> const& getPresentCompleteSemaphores() const { return m_presentCompleteSemaphores; }

      VKHLF_API operator vk::SwapchainKHR() const;

      Swapchain(Swapchain const& rhs) = delete;
      Swapchain & operator=(Swapchain const& rhs) = delete;

    private:
      std::vector<std::shared_ptr<Image>>     m_images;
      std::vector<std::shared_ptr<Semaphore>> m_presentCompleteSemaphores;
      // this is the semaphore to use when acquiring a new image
      // the semaphore at the given image index will be swapped with free semaphore
      std::shared_ptr<Semaphore>              m_freeSemaphore;
      vk::SwapchainKHR                        m_swapchain;
  };

  inline std::vector<std::shared_ptr<vkhlf::Image>> const& Swapchain::getImages() const
  {
    return m_images;
  }

  inline Swapchain::operator vk::SwapchainKHR() const
  {
    return m_swapchain;
  }

} // namespace vk
