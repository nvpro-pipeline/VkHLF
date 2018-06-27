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

#include <vkhlf/vkhlf.h>

class VkHLFSampleWindow
{
public:
  VkHLFSampleWindow(char const * title, int width, int height);
  virtual ~VkHLFSampleWindow();

  void paint();

  GLFWwindow *getWindow() const { return m_window.get(); } 
protected:
  virtual void resize(int width, int height);

  virtual void doResize(int width, int height);
  virtual void doPaint();

  virtual void cursorPosEvent(double xPos, double yPos);
  virtual void keyEvent(int key, int scancode, int action, int mods);
  virtual void mouseButtonEvent(int button, int action, int mods);
  virtual void scrollEvent(double offset);

  std::shared_ptr<vkhlf::CommandPool>          const &getCommandPool()             const { return m_commandPool; }
  std::shared_ptr<vkhlf::PhysicalDevice>       const &getPhysicalDevice()          const { return m_physicalDevice; }
  std::shared_ptr<vkhlf::Device>               const &getDevice()                  const { return m_device; }
  std::unique_ptr<vkhlf::FramebufferSwapchain> const &getFramebufferSwapchain()    const { return m_framebufferSwapchain; }
  std::shared_ptr<vkhlf::Queue>                const &getGraphicsQueue()           const { return m_graphicsQueue; }
  std::shared_ptr<vkhlf::RenderPass>           const &getRenderPass()              const { return m_renderPass; }
  std::shared_ptr<vkhlf::Surface>              const &getSurface()                 const { return m_surface; }
  vk::Format                                          getColorFormat()             const { return m_colorFormat; }
  vk::Format                                          getDepthFormat()             const { return m_depthFormat; }
  std::shared_ptr<vkhlf::Semaphore>            const &getRenderCompleteSemaphore() const { return m_renderCompleteSemaphore; }
  uint32_t                                            getQueueFamilyIndex()        const { return m_queueFamilyIndex; }

private:
  static void cursorPosCallback(GLFWwindow * window, double xPos, double yPos);
  static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
  static void paintCallback(GLFWwindow *window);
  static void mouseButtonCallback(GLFWwindow * window, int button, int action, int mods);
  static void resizeCallback(GLFWwindow *window, int width, int height);
  static void scrollCallback(GLFWwindow * window, double xOffset, double yOffset);

private:
  std::unique_ptr<GLFWwindow, void(*)(GLFWwindow*)> m_window;
#if !defined(NDEBUG)
  std::shared_ptr<vkhlf::DebugReportCallback>  m_debugReportCallback;
#endif
  std::shared_ptr<vkhlf::PhysicalDevice>       m_physicalDevice;
  std::shared_ptr<vkhlf::Device>               m_device;
  std::unique_ptr<vkhlf::FramebufferSwapchain> m_framebufferSwapchain;
  std::shared_ptr<vkhlf::Queue>                m_graphicsQueue;
  std::shared_ptr<vkhlf::RenderPass>           m_renderPass;
  std::shared_ptr<vkhlf::Surface>              m_surface;
  vk::Format                                   m_colorFormat;
  vk::Format                                   m_depthFormat;
  std::shared_ptr<vkhlf::Semaphore>            m_renderCompleteSemaphore;
  uint32_t                                     m_queueFamilyIndex;
  std::shared_ptr<vkhlf::CommandPool>          m_commandPool;
};