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
#include <vkhlf/Fence.h>
#include <vkhlf/Image.h>
#include <vkhlf/Semaphore.h>
#include <vkhlf/Surface.h>
#include <vkhlf/Swapchain.h>

namespace vkhlf
{

  Swapchain::Swapchain(std::shared_ptr<Device> const& device, std::shared_ptr<Surface> const& surface, uint32_t minImageCount, vk::Format imageFormat, vk::Extent2D const& imageExtent,
                       uint32_t imageArrayLayers, vk::ImageUsageFlags imageUsage, vk::SharingMode imageSharingMode, vk::ArrayProxy<const uint32_t> queueFamilyIndices,
                       vk::SurfaceTransformFlagBitsKHR preTransform, vk::CompositeAlphaFlagBitsKHR compositeAlpha, vk::PresentModeKHR presentMode, bool clipped,
                       std::shared_ptr<Swapchain> const& oldSwapchain, std::shared_ptr<Allocator> const& allocator)
    : Reference(device, allocator)
  {
    static_assert(VK_COLOR_SPACE_RANGE_SIZE_KHR == 1, "add argument 'colorSpace' when this assertion fires");
    vk::SwapchainCreateInfoKHR createInfo({}, static_cast<vk::SurfaceKHR>(*surface), minImageCount, imageFormat, vk::ColorSpaceKHR::eSrgbNonlinear, imageExtent, imageArrayLayers, imageUsage,
                                          imageSharingMode, vkhlf::checked_cast<uint32_t>(queueFamilyIndices.size()), queueFamilyIndices.data(), preTransform, compositeAlpha, presentMode, clipped,
                                          oldSwapchain ? static_cast<vk::SwapchainKHR>(*oldSwapchain) : nullptr);
    m_swapchain = static_cast<vk::Device>(*get<Device>()).createSwapchainKHR(createInfo, *get<Allocator>());

    std::vector<vk::Image> images = static_cast<vk::Device>(*get<Device>()).getSwapchainImagesKHR(m_swapchain);
    m_images.reserve(images.size());
    m_presentCompleteSemaphores.reserve(images.size() + 1);
    for (size_t i = 0; i < images.size(); i++)
    {
      m_images.push_back(std::make_shared<Image>(get<Device>(), images[i]));
      m_presentCompleteSemaphores.push_back(get<Device>()->createSemaphore()); 
    }
    m_freeSemaphore = get<Device>()->createSemaphore();
  }

  Swapchain::~Swapchain()
  {
    m_images.clear();
    static_cast<vk::Device>(*get<Device>()).destroySwapchainKHR(m_swapchain, *get<Allocator>());
  }

  uint32_t Swapchain::acquireNextImage(uint64_t timeout, std::shared_ptr<Fence> const& fence)
  {
    vk::ResultValue<uint32_t> result = static_cast<vk::Device>(*get<Device>()).acquireNextImageKHR(m_swapchain, timeout, *m_freeSemaphore, fence ? static_cast<vk::Fence>(*fence) : nullptr);
    assert(result.result == vk::Result::eSuccess);  // need to handle timeout, not ready, and suboptimal
    // put the semaphore at the correct index and use the semaphore from the new index as next free semaphore
    std::swap(m_freeSemaphore, m_presentCompleteSemaphores[result.value]);
    return result.value;
  }

} // namespace vk
