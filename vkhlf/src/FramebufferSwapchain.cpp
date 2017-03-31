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


#include <vkhlf/CommandBuffer.h>
#include <vkhlf/FramebufferSwapchain.h>
#include <vkhlf/Image.h>
#include <vkhlf/PhysicalDevice.h>

#include <algorithm>
#include <functional>

namespace vkhlf {

    FramebufferSwapchain::FramebufferSwapchain(std::shared_ptr<Device> const& device,
        std::shared_ptr<Surface> const& surface,
        vk::Format surfaceFormat,
        vk::Format depthFormat,
        std::shared_ptr<RenderPass> const& renderPass,
        std::shared_ptr<DeviceMemoryAllocator> const& deviceMemoryAllocator,
        std::shared_ptr<Allocator> const& swapchainAllocator,
        std::shared_ptr<Allocator> const& imageAllocator,
        std::shared_ptr<Allocator> const& imageViewAllocator)
      : m_swapchainIndex(0)
    {
        vk::SurfaceCapabilitiesKHR surfaceCapabilities = device->get<PhysicalDevice>()->getSurfaceCapabilities(surface);
        assert(surfaceCapabilities.currentExtent.width != -1);
        m_extent = surfaceCapabilities.currentExtent;

        std::vector<vk::PresentModeKHR> presentModes = device->get<PhysicalDevice>()->getSurfacePresentModes(surface);

        static std::initializer_list<vk::PresentModeKHR> const presentModeOrder{ vk::PresentModeKHR::eMailbox, vk::PresentModeKHR::eFifo, vk::PresentModeKHR::eImmediate };
        auto itPresentModeOrder = std::find_first_of(presentModeOrder.begin(), presentModeOrder.end(), presentModes.begin(), presentModes.end());
        if (itPresentModeOrder == presentModeOrder.end())
        {
            throw std::runtime_error("Failed to find a valid presentMode");
        }
        vk::PresentModeKHR swapchainPresentMode = *itPresentModeOrder;

        // Request at least one more image than minImageCount to ensure that double-buffering does work if maxImageCount does allow it.
        uint32_t desiredNumberOfSwapChainImages = (surfaceCapabilities.maxImageCount == 0) ? surfaceCapabilities.minImageCount + 1 : std::min(surfaceCapabilities.minImageCount + 1, surfaceCapabilities.maxImageCount);

        std::vector<vk::SurfaceFormatKHR> surfaceFormats = device->get<PhysicalDevice>()->getSurfaceFormats(surface);
        auto itSurfaceFormats = std::find(surfaceFormats.begin(), surfaceFormats.end(), vk::SurfaceFormatKHR{ surfaceFormat, vk::ColorSpaceKHR::eSrgbNonlinear });
        if (itSurfaceFormats == surfaceFormats.end())
        {
            throw std::runtime_error("surface does not support surfaceFormat");
        }

        vk::SurfaceTransformFlagBitsKHR preTransform = (surfaceCapabilities.supportedTransforms & vk::SurfaceTransformFlagBitsKHR::eIdentity)
            ? vk::SurfaceTransformFlagBitsKHR::eIdentity
            : surfaceCapabilities.currentTransform;

        m_swapchain = device->createSwapchain(surface, desiredNumberOfSwapChainImages, surfaceFormat, m_extent, 1,
            vk::ImageUsageFlagBits::eColorAttachment | vk::ImageUsageFlagBits::eTransferSrc, vk::SharingMode::eExclusive, {}, preTransform,
            vk::CompositeAlphaFlagBitsKHR::eOpaque, swapchainPresentMode, true, nullptr, swapchainAllocator);

        m_colorImages = m_swapchain->getImages();

        m_colorViews.reserve(m_colorImages.size());
        for (size_t i = 0; i < m_colorImages.size(); i++)
        {
            m_colorViews.push_back(m_colorImages[i]->createImageView(vk::ImageViewType::e2D, surfaceFormat,
            { vk::ComponentSwizzle::eR, vk::ComponentSwizzle::eG, vk::ComponentSwizzle::eB, vk::ComponentSwizzle::eA }, { vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1 }, imageViewAllocator));
        }

        // depth/stencil buffer
        // assert that a depth and/or stencil format is requested
        vk::FormatProperties formatProperties = device->get<PhysicalDevice>()->getFormatProperties(depthFormat);
        assert((formatProperties.linearTilingFeatures & vk::FormatFeatureFlagBits::eDepthStencilAttachment) ||
               (formatProperties.optimalTilingFeatures & vk::FormatFeatureFlagBits::eDepthStencilAttachment));

        vk::ImageTiling tiling = (formatProperties.optimalTilingFeatures & vk::FormatFeatureFlagBits::eDepthStencilAttachment) ? vk::ImageTiling::eOptimal : vk::ImageTiling::eLinear;

        m_depthImage = device->createImage({}, vk::ImageType::e2D, depthFormat, vk::Extent3D(m_extent.width, m_extent.height, 1), 1, 1, vk::SampleCountFlagBits::e1, tiling,
            vk::ImageUsageFlagBits::eDepthStencilAttachment, vk::SharingMode::eExclusive, {}, vk::ImageLayout::eUndefined, {} /* No requirements */,
            deviceMemoryAllocator, imageAllocator);

        // determine ImageAspect based on format
        vk::ImageAspectFlags aspectMask;
        if (depthFormat != vk::Format::eS8Uint)
        {
            aspectMask |= vk::ImageAspectFlagBits::eDepth;
        }

        // add eStencil if image contains stencil
        static std::initializer_list<vk::Format> const stencilFormats{ vk::Format::eD16UnormS8Uint, vk::Format::eD24UnormS8Uint, vk::Format::eD32SfloatS8Uint, vk::Format::eS8Uint };
        if (std::find(stencilFormats.begin(), stencilFormats.end(), depthFormat) != stencilFormats.end())
        {
            aspectMask |= vk::ImageAspectFlagBits::eStencil;
        }

        m_depthView = m_depthImage->createImageView(vk::ImageViewType::e2D, depthFormat,
            vk::ComponentMapping(vk::ComponentSwizzle::eR, vk::ComponentSwizzle::eG, vk::ComponentSwizzle::eB, vk::ComponentSwizzle::eA),
            vk::ImageSubresourceRange(aspectMask, 0, 1, 0, 1), imageViewAllocator);

        // finally: the framebuffers!
        m_framebuffers.reserve(m_colorViews.size());
        for (size_t i = 0; i < m_colorViews.size(); i++)
        {
            m_framebuffers.push_back(device->createFramebuffer(renderPass, { m_colorViews[i], m_depthView }, m_extent, 1));
        }
    }
} // namespace vkhlf
