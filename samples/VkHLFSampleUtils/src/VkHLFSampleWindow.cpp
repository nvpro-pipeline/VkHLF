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

// Windows #defines MemoryBarrier which collides with vk::MemoryBarrier
// #undef here...
#include <Windows.h>
#if defined(MemoryBarrier)
# undef MemoryBarrier
#endif

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <vulkan/vulkan.hpp>

#include <cassert>
#include <iostream>
#include <memory>
#include <system_error>
#include <vector>

#include <VkHLFSampleWindow.h>

VkHLFSampleWindow::VkHLFSampleWindow(char const * title, int width, int height)
  : m_window(nullptr, glfwDestroyWindow)
{
  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
  m_window.reset(glfwCreateWindow(width, height, "VkHLF - 01_window", NULL, NULL));
  if (!m_window)
  {
    throw std::runtime_error("failed to create window");
  }

  std::vector<std::string>  enabledExtensions, enabledLayers;

  // Put instance extensions required by glfw into enabled extensions list
  uint32_t count;
  const char** extensions = glfwGetRequiredInstanceExtensions(&count);
  std::copy(extensions, extensions + count, std::back_inserter(enabledExtensions));

#if !defined(NDEBUG)
  // Enable standard validation layer to find as much errors as possible!
  enabledLayers.push_back("VK_LAYER_LUNARG_standard_validation");
#endif

  // Create a new vulkan instance using the required extensions
  std::shared_ptr<vkhlf::Instance> instance = vkhlf::Instance::create(title, 1, enabledLayers, enabledExtensions);

#if !defined(NDEBUG)
  // The validation layers send a lot of information to the debug report callback. Register one
  vk::DebugReportFlagsEXT flags(vk::DebugReportFlagBitsEXT::eWarning | vk::DebugReportFlagBitsEXT::ePerformanceWarning | vk::DebugReportFlagBitsEXT::eError | vk::DebugReportFlagBitsEXT::eDebug);
  m_debugReportCallback = instance->createDebugReportCallback(flags, &vkhlf::debugReportCallback);
#endif

  // Find a physical device with presentation support
  assert(instance->getPhysicalDeviceCount() != 0);
  for (size_t index = 0; index < instance->getPhysicalDeviceCount(); ++index)
  {
    // need to get the QueueFamilyProperties before asking for presentation support !
    std::shared_ptr<vkhlf::PhysicalDevice> physicalDevice = instance->getPhysicalDevice(index);
    std::vector<vk::QueueFamilyProperties> properties = physicalDevice->getQueueFamilyProperties();
    assert(!properties.empty());

    if (glfwGetPhysicalDevicePresentationSupport(static_cast<vk::Instance>(*instance), static_cast<vk::PhysicalDevice>(*physicalDevice), 0))
    {
      m_physicalDevice = physicalDevice;
      break;
    }
  }
  if (!m_physicalDevice)
  {
    throw std::runtime_error("Failed to find a device with presentation support");
  }

  m_surface = instance->createSurface(m_window.get());

  // Search for a graphics queue and a present queue in the array of queue families, try to find one that supports both
  std::vector<uint32_t> queueFamilyIndices = vkhlf::getGraphicsPresentQueueFamilyIndices(m_physicalDevice, m_surface);
  assert(!queueFamilyIndices.empty());
  m_queueFamilyIndex = queueFamilyIndices[0];

  std::vector<vk::SurfaceFormatKHR> surfaceFormats = m_physicalDevice->getSurfaceFormats(m_surface);
  assert(!surfaceFormats.empty());

  // If the format list includes just one entry of VK_FORMAT_UNDEFINED, the surface has no preferred format.  Otherwise, at least one supported format will be returned.
  m_colorFormat = ((surfaceFormats.size() == 1) && (surfaceFormats[0].format == vk::Format::eUndefined)) ? vk::Format::eB8G8R8A8Unorm : surfaceFormats[0].format;
  m_depthFormat = vk::Format::eD24UnormS8Uint;

  // Create a new device with the VK_KHR_SWAPCHAIN_EXTENSION enabled.
  m_device = m_physicalDevice->createDevice(vkhlf::DeviceQueueCreateInfo(m_queueFamilyIndex, 0.0f), nullptr, { VK_KHR_SWAPCHAIN_EXTENSION_NAME });

  m_graphicsQueue = m_device->getQueue(m_queueFamilyIndex, 0);

  // init render pass
  vk::AttachmentReference colorReference(0, vk::ImageLayout::eColorAttachmentOptimal);
  vk::AttachmentReference depthReference(1, vk::ImageLayout::eDepthStencilAttachmentOptimal);

  m_renderPass = m_device->createRenderPass(
    { 
      vk::AttachmentDescription( // attachment 0
        {}, m_colorFormat, vk::SampleCountFlagBits::e1,
        vk::AttachmentLoadOp::eClear, vk::AttachmentStoreOp::eStore, // color
        vk::AttachmentLoadOp::eDontCare, vk::AttachmentStoreOp::eDontCare, // stencil
        vk::ImageLayout::eUndefined, vk::ImageLayout::ePresentSrcKHR
      ),
      vk::AttachmentDescription( // attachment 1
        {}, m_depthFormat, vk::SampleCountFlagBits::e1,
        vk::AttachmentLoadOp::eClear, vk::AttachmentStoreOp::eStore, // depth
        vk::AttachmentLoadOp::eDontCare, vk::AttachmentStoreOp::eDontCare, // stencil
        vk::ImageLayout::eUndefined,vk::ImageLayout::eDepthStencilAttachmentOptimal
      )
    },
    vk::SubpassDescription( {}, vk::PipelineBindPoint::eGraphics, 0, nullptr, 1, &colorReference, nullptr, &depthReference, 0, nullptr), nullptr
  );

  m_renderCompleteSemaphore = m_device->createSemaphore();

  // create Framebuffer & Swapchain
  resize(width, height);

  if (m_window)
  {
    glfwSetWindowUserPointer(m_window.get(), this);

    glfwSetFramebufferSizeCallback(m_window.get(), VkHLFSampleWindow::resizeCallback);
    glfwSetWindowRefreshCallback(m_window.get(), VkHLFSampleWindow::paintCallback);
    glfwSetKeyCallback(m_window.get(), VkHLFSampleWindow::keyCallback);
    glfwSetMouseButtonCallback(m_window.get(), VkHLFSampleWindow::mouseButtonCallback);
    glfwSetCursorPosCallback(m_window.get(), VkHLFSampleWindow::cursorPosCallback);
  }
}

VkHLFSampleWindow::~VkHLFSampleWindow()
{
  m_framebufferSwapchain.reset();
}

void VkHLFSampleWindow::resize(int width, int height)
{
    assert((0 <= width) && (0 <= height));

    m_framebufferSwapchain.reset();    // need to be reset, before creating a new one!!
    m_framebufferSwapchain.reset(new vkhlf::FramebufferSwapchain(m_device, m_surface, m_colorFormat, m_depthFormat, m_renderPass));
    assert(m_framebufferSwapchain->getExtent() == vk::Extent2D(width, height));

    doResize(width, height);
}

void VkHLFSampleWindow::doResize(int width, int height)
{
}

void VkHLFSampleWindow::paint()
{
  // Get the index of the next available swapchain image:
  m_framebufferSwapchain->acquireNextFrame();
  doPaint();
  m_framebufferSwapchain->present(m_graphicsQueue, m_renderCompleteSemaphore);
}

void VkHLFSampleWindow::doPaint()
{
  // create a command pool for command buffer allocation
  std::shared_ptr<vkhlf::CommandPool> commandPool = m_device->createCommandPool(vk::CommandPoolCreateFlagBits::eResetCommandBuffer, m_queueFamilyIndex);
  std::shared_ptr<vkhlf::CommandBuffer> commandBuffer = commandPool->allocateCommandBuffer();

  std::array<float, 4> ccv = { 0.462745f, 0.72549f, 0.0f };
  commandBuffer->begin();
  commandBuffer->beginRenderPass(m_renderPass, m_framebufferSwapchain->getFramebuffer(), vk::Rect2D({ 0, 0 }, m_framebufferSwapchain->getExtent()),
                                 { vk::ClearValue(ccv), vk::ClearValue(vk::ClearDepthStencilValue(1.0f, 0)) }, vk::SubpassContents::eInline);
  commandBuffer->setViewport(0, vk::Viewport(0.0f, 0.0f, (float)m_framebufferSwapchain->getExtent().width, (float)m_framebufferSwapchain->getExtent().height, 0.0f, 1.0f));
  commandBuffer->setScissor(0, vk::Rect2D({ 0, 0 }, m_framebufferSwapchain->getExtent()));
  commandBuffer->endRenderPass();
  commandBuffer->end();

  m_graphicsQueue->submit(vkhlf::SubmitInfo{ { m_framebufferSwapchain->getPresentSemaphore() },{ vk::PipelineStageFlagBits::eColorAttachmentOutput }, commandBuffer, m_renderCompleteSemaphore });
}

void VkHLFSampleWindow::paintCallback(GLFWwindow * window)
{
  VkHLFSampleWindow * wd = reinterpret_cast<VkHLFSampleWindow*>(glfwGetWindowUserPointer(window));
  wd->paint();
}

void VkHLFSampleWindow::resizeCallback(GLFWwindow *window, int width, int height)
{
    VkHLFSampleWindow * wd = reinterpret_cast<VkHLFSampleWindow*>(glfwGetWindowUserPointer(window));
    wd->resize(width, height);
}

void VkHLFSampleWindow::cursorPosCallback(GLFWwindow * window, double xPos, double yPos)
{
  VkHLFSampleWindow * wd = reinterpret_cast<VkHLFSampleWindow*>(glfwGetWindowUserPointer(window));
  wd->cursorPosEvent(xPos, yPos);
}

void VkHLFSampleWindow::keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
  VkHLFSampleWindow * wd = reinterpret_cast<VkHLFSampleWindow*>(glfwGetWindowUserPointer(window));
  wd->keyEvent(key, scancode, action, mods);
}

void VkHLFSampleWindow::mouseButtonCallback(GLFWwindow * window, int button, int action, int mods)
{
  VkHLFSampleWindow * wd = reinterpret_cast<VkHLFSampleWindow*>(glfwGetWindowUserPointer(window));
  wd->mouseButtonEvent(button, action, mods);
}

void VkHLFSampleWindow::cursorPosEvent(double xPos, double yPos)
{
}

void VkHLFSampleWindow::keyEvent(int key, int scancode, int action, int mods)
{
}

void VkHLFSampleWindow::mouseButtonEvent(int button, int action, int mods)
{
}

