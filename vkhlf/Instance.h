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
#include <vkhlf/Surface.h>
#include <memory>
#include <vector>

namespace vkhlf
{

  VKHLF_API VkBool32 VKAPI_CALL debugReportCallback(VkDebugReportFlagsEXT flags, VkDebugReportObjectTypeEXT objectType, uint64_t object, size_t location, int32_t messageCode, const char* pLayerPrefix,
                                                 const char* pMessage, void* pUserData);

  class Instance : public Reference<Allocator>, public std::enable_shared_from_this<Instance>
  {
    public:
      VKHLF_API static std::shared_ptr<vkhlf::Instance> create(std::string const&                appName,
                                                          uint32_t                          appVersion,
                                                          vk::ArrayProxy<const std::string> enabledLayers = nullptr,
                                                          vk::ArrayProxy<const std::string> enabledExtensions = nullptr,
                                                          std::shared_ptr<Allocator> const& allocator = nullptr);
      VKHLF_API Instance(vk::InstanceCreateInfo const& createInfo, std::shared_ptr<Allocator> const& allocator);
      VKHLF_API virtual ~Instance();

      VKHLF_API std::shared_ptr<DebugReportCallback> createDebugReportCallback(vk::DebugReportFlagsEXT flags, PFN_vkDebugReportCallbackEXT callback, void * pUserData = nullptr,
                                                                            std::shared_ptr<Allocator> const& allocator = nullptr);
#ifdef VK_USE_PLATFORM_ANDROID_KHR
             std::shared_ptr<Surface>             createSurface(ANativeWindow * window, std::shared_ptr<Allocator> const& allocator = nullptr);
#endif
#ifdef VK_USE_PLATFORM_MIR_KHR
             std::shared_ptr<Surface>             createSurface(MirConnection * connection, MirSurface * surface, std::shared_ptr<Allocator> const& allocator = nullptr);
#endif
#ifdef VK_USE_PLATFORM_WAYLAND_KHR
             std::shared_ptr<Surface>             createSurface(struct wl_display * display, struct wl_surface * surface, std::shared_ptr<Allocator> const& allocator = nullptr);
#endif
#ifdef VK_USE_PLATFORM_WIN32_KHR
             std::shared_ptr<Surface>             createSurface(HINSTANCE hinstance, HWND hwnd, std::shared_ptr<Allocator> const& allocator = nullptr);
#endif
#ifdef VK_USE_PLATFORM_XLIB_KHR
             std::shared_ptr<Surface>             createSurface(Display * dpy, Window window, std::shared_ptr<Allocator> const& allocator = nullptr);
#endif
#ifdef VK_USE_PLATFORM_XCB_KHR
             std::shared_ptr<Surface>             createSurface(xcb_connection_t * connection, xcb_window_t window, std::shared_ptr<Allocator> const& allocator = nullptr);
#endif
#ifdef GLFW_INCLUDE_VULKAN
             std::shared_ptr<Surface>             createSurface(GLFWwindow * window, std::shared_ptr<Allocator> const& allocator = nullptr);
#endif
      VKHLF_API size_t                               getPhysicalDeviceCount() const;
      VKHLF_API std::shared_ptr<vkhlf::PhysicalDevice> getPhysicalDevice(size_t index);
      VKHLF_API PFN_vkVoidFunction                   getProcAddress(std::string const& name) const;

      VKHLF_API operator vk::Instance();

      Instance(Instance const& rhs) = delete;
      Instance & operator=(Instance const& rhs) = delete;

    private:
      std::shared_ptr<Allocator>                      m_allocator;
      std::vector<vk::PhysicalDevice>                 m_physicalDevices;
      std::vector<std::weak_ptr<vkhlf::PhysicalDevice>> m_physicalDevicesCache;
      vk::Instance                                    m_instance;
  };

  inline Instance::operator vk::Instance()
  {
    return m_instance;
  }

#ifdef VK_USE_PLATFORM_ANDROID_KHR
  inline std::shared_ptr<Surface> Instance::createSurface(ANativeWindow * window, std::shared_ptr<Allocator> const& allocator)
  {
    return std::make_shared<Surface>(shared_from_this(), m_instance.createAndroidSurfaceKHR(vk::AndroidSurfaceCreateInfoKHR({}, window), *allocator), allocator);
  }
#endif
  
#ifdef VK_USE_PLATFORM_MIR_KHR
  inline std::shared_ptr<Surface> Instance::createSurface(MirConnection * connection, MirSurface * surface, std::shared_ptr<Allocator> const& allocator)
  {
    return std::make_shared<Surface>(shared_from_this(), m_instance.createMirSurfaceKHR(vk::MirSurfaceCreateInfoKHR({}, connection, surface), *allocator), allocator);
  }
#endif
  
#ifdef VK_USE_PLATFORM_WAYLAND_KHR
  inline std::shared_ptr<Surface> Instance::createSurface(struct wl_display * display, struct wl_surface * surface, std::shared_ptr<Allocator> const& allocator)
  {
    return std::make_shared<Surface>(shared_from_this(), m_instance.createWaylandSurfaceKHR(vk::WaylandSurfaceCreateInfoKHR({}, display, surface), *allocator), allocator);
  }
#endif
  
#ifdef VK_USE_PLATFORM_WIN32_KHR
  inline std::shared_ptr<Surface> Instance::createSurface(HINSTANCE hinstance, HWND hwnd, std::shared_ptr<Allocator> const& allocator)
  {
    return std::make_shared<Surface>(shared_from_this(), m_instance.createWin32SurfaceKHR(vk::Win32SurfaceCreateInfoKHR({}, hinstance, hwnd), *allocator), allocator);
  }
#endif
  
#ifdef VK_USE_PLATFORM_XLIB_KHR
  inline std::shared_ptr<Surface> Instance::createSurface(Display * dpy, Window window, std::shared_ptr<Allocator> const& allocator)
  {
    return std::make_shared<Surface>(shared_from_this(), m_instance.createXlibSurfaceKHR(vk::XlibSurfaceCreateInfoKHR({}, dpy, window), *allocator), allocator);
  }
#endif
  
#ifdef VK_USE_PLATFORM_XCB_KHR
  inline std::shared_ptr<Surface> Instance::createSurface(xcb_connection_t * connection, xcb_window_t window, std::shared_ptr<Allocator> const& allocator)
  {
    return std::make_shared<Surface>(shared_from_this(), m_instance.createXcbSurfaceKHR(vk::XcbSurfaceCreateInfoKHR({}, connection, window), *allocator), allocator);
  }
#endif

#ifdef GLFW_INCLUDE_VULKAN
  inline std::shared_ptr<Surface> Instance::createSurface(GLFWwindow * window, std::shared_ptr<Allocator> const& allocator)
  {
    VkSurfaceKHR surface;
    VkResult res = glfwCreateWindowSurface(m_instance, window, NULL, &surface);
    assert(res == VK_SUCCESS);
    return std::make_shared<Surface>(shared_from_this(), surface, allocator);
  }
#endif

} // namespace vk
