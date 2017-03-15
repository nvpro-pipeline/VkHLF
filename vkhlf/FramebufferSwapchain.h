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
#include <vkhlf/Swapchain.h>
#include <vkhlf/Queue.h>

namespace vkhlf {

    class FramebufferSwapchain
    {
    public:
        VKHLF_API FramebufferSwapchain(std::shared_ptr<Device> const& device,
            std::shared_ptr<Surface> const& surface,
            vk::Format surfaceFormat,
            vk::Format depthFormat,
            std::shared_ptr<RenderPass> const& renderPass,
            std::shared_ptr<DeviceMemoryAllocator> const& deviceMemoryAllocator = nullptr,
            std::shared_ptr<Allocator> const& swapchainAllocator = nullptr,
            std::shared_ptr<Allocator> const& imageAllocator = nullptr,
            std::shared_ptr<Allocator> const& imageViewAllocator = nullptr);

        void acquireNextFrame(uint64_t timeout = UINT64_MAX, std::shared_ptr<Fence> const& fence = {}) { m_swapchainIndex = m_swapchain->acquireNextImage(timeout, fence); }

        std::shared_ptr<Framebuffer> const& getFramebuffer() const { return m_framebuffers[m_swapchainIndex]; }

        std::shared_ptr<Image> const&       getColorImage() const { return m_colorImages[m_swapchainIndex]; }
        std::shared_ptr<ImageView> const&   getColorImageView() const { return m_colorViews[m_swapchainIndex]; }

        std::shared_ptr<Image> const&       getDepthImage() const { return m_depthImage; }
        std::shared_ptr<ImageView> const&   getDepthView() const { return m_depthView; }

        vk::Extent2D const&                 getExtent() const { return m_extent; }
        std::shared_ptr<Swapchain> const&   getSwapchain() const { return m_swapchain; }
        uint32_t                            getSwapchainIndex() const { return m_swapchainIndex; }

        // wait for this semaphore before rendering
        std::shared_ptr<Semaphore> const&   getPresentSemaphore() const { return m_swapchain->getPresentCompleteSemaphores()[m_swapchainIndex]; }

        void present(std::shared_ptr<Queue> const& queue, vk::ArrayProxy<const std::shared_ptr<Semaphore>> waitSemaphores = {})
        {
            queue->present(waitSemaphores, m_swapchain, m_swapchainIndex);
        }

    private:
        vk::Extent2D                              m_extent;
        std::shared_ptr<Swapchain>                m_swapchain;
        uint32_t                                  m_swapchainIndex; // current swapchain index
        std::vector<std::shared_ptr<Image>>       m_colorImages;
        std::vector<std::shared_ptr<ImageView>>   m_colorViews;
        std::shared_ptr<Image>                    m_depthImage;
        std::shared_ptr<DeviceMemory>             m_depthMemory;
        std::shared_ptr<ImageView>                m_depthView;
        std::vector<std::shared_ptr<Framebuffer>> m_framebuffers;
    };

} // namespace vkhlf
