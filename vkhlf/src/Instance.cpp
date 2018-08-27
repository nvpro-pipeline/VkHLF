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


#include <vkhlf/Config.h>
#include <vkhlf/DebugReportCallback.h>
#include <vkhlf/Instance.h>
#include <vkhlf/PhysicalDevice.h>

#ifdef VK_OS_WINDOWS
#include <windows.h>
#else
#include <dlfcn.h>
#endif

#include <iostream>

PFN_vkCreateDebugReportCallbackEXT  pfnVkCreateDebugReportCallbackEXT;
PFN_vkDestroyDebugReportCallbackEXT pfnVkDestroyDebugReportCallbackEXT;

VKAPI_ATTR VkResult VKAPI_CALL vkCreateDebugReportCallbackEXT(VkInstance instance, const VkDebugReportCallbackCreateInfoEXT* pCreateInfo, const VkAllocationCallbacks* pAllocator, VkDebugReportCallbackEXT* pCallback)
{
  return pfnVkCreateDebugReportCallbackEXT(instance, pCreateInfo, pAllocator, pCallback);
}

VKAPI_ATTR void VKAPI_CALL vkDestroyDebugReportCallbackEXT(VkInstance instance, VkDebugReportCallbackEXT callback, const VkAllocationCallbacks* pAllocator)
{
  pfnVkDestroyDebugReportCallbackEXT(instance, callback, pAllocator);
}

namespace vkhlf
{
  VKAPI_ATTR VkBool32 VKAPI_CALL debugReportCallback(VkDebugReportFlagsEXT flags, VkDebugReportObjectTypeEXT objectType, uint64_t object, size_t location, int32_t messageCode, const char* pLayerPrefix, const char* pMessage, void* pUserData)
  {
    switch (flags)
    {
      case VK_DEBUG_REPORT_INFORMATION_BIT_EXT :
        std::cout << "INFORMATION: " << pMessage << std::endl;
        return VK_FALSE;
      case VK_DEBUG_REPORT_WARNING_BIT_EXT :
        std::cerr << "WARNING: ";
        break;
      case VK_DEBUG_REPORT_PERFORMANCE_WARNING_BIT_EXT:
        std::cerr << "PERFORMANCE WARNING: ";
        break;
      case VK_DEBUG_REPORT_ERROR_BIT_EXT:
        std::cerr << "ERROR: ";
        break;
      case VK_DEBUG_REPORT_DEBUG_BIT_EXT:
        std::cout << "DEBUG: " << pMessage << std::endl;
        return VK_FALSE;
      default:
        std::cerr << "unknown flag (" << flags << "): ";
        break;
    }
    std::cerr << pMessage << std::endl;
    return VK_TRUE;
  }

  std::shared_ptr<vkhlf::Instance> Instance::create(std::string const& appName, uint32_t appVersion, vk::ArrayProxy<const std::string> enabledLayers, vk::ArrayProxy<const std::string> enabledExtensions,
                                                  std::shared_ptr<Allocator> const & allocator)
  {
    vk::ApplicationInfo applicationInfo(appName.c_str(), appVersion, "DP::VHK", 1, VK_MAKE_VERSION(1,1,0));

    std::vector<char const*> layers;
    layers.reserve(enabledLayers.size());
    for (auto const& s : enabledLayers)
    {
      layers.push_back(s.c_str());
    }

    std::vector<char const*> extensions;
    extensions.reserve(enabledExtensions.size());
    for (auto const& s : enabledExtensions)
    {
      extensions.push_back(s.c_str());
    }
#if !defined(NDEBUG)
    std::vector<vk::ExtensionProperties> extensionProperties = vk::enumerateInstanceExtensionProperties();
    for (auto const& ee : enabledExtensions)
    {
      assert(std::find_if(extensionProperties.begin(), extensionProperties.end(), [ee](vk::ExtensionProperties const& ep) { return ee == ep.extensionName; }) != extensionProperties.end());
    }

# if !defined(VK_OS_MAC) && !defined(VK_OS_IOS)
    if (std::find(enabledExtensions.begin(), enabledExtensions.end(), VK_EXT_DEBUG_REPORT_EXTENSION_NAME) == enabledExtensions.end())
    {
      assert(std::find_if(extensionProperties.begin(), extensionProperties.end(), [](vk::ExtensionProperties const& lp) { return strcmp(lp.extensionName, VK_EXT_DEBUG_REPORT_EXTENSION_NAME) == 0; }) != extensionProperties.end());
      extensions.push_back(VK_EXT_DEBUG_REPORT_EXTENSION_NAME);
    }
# endif
#endif

    vk::InstanceCreateInfo instanceInfo({}, &applicationInfo, vkhlf::checked_cast<uint32_t>(layers.size()), layers.data(), vkhlf::checked_cast<uint32_t>(extensions.size()), extensions.data());

    return std::make_shared<Instance>(instanceInfo, allocator);
  }

  Instance::Instance(vk::InstanceCreateInfo const& createInfo, std::shared_ptr<Allocator> const& allocator)
    : Reference(allocator)
  {
    m_instance = vk::createInstance(createInfo, *get<Allocator>());
    m_physicalDevices = m_instance.enumeratePhysicalDevices();
    m_physicalDevicesCache.resize(m_physicalDevices.size());

    static bool initialized = false;
    if (!initialized)
    {
      pfnVkCreateDebugReportCallbackEXT = reinterpret_cast<PFN_vkCreateDebugReportCallbackEXT>(m_instance.getProcAddr("vkCreateDebugReportCallbackEXT"));
      pfnVkDestroyDebugReportCallbackEXT = reinterpret_cast<PFN_vkDestroyDebugReportCallbackEXT>(m_instance.getProcAddr("vkDestroyDebugReportCallbackEXT"));
      initialized = true;
    }
  }

  Instance::~Instance()
  {
    assert(m_instance);
    m_physicalDevices.clear();
    m_instance.destroy(*m_allocator);
  }

  std::shared_ptr<DebugReportCallback> Instance::createDebugReportCallback(vk::DebugReportFlagsEXT flags, PFN_vkDebugReportCallbackEXT callback, void * pUserData, std::shared_ptr<Allocator> const& allocator)
  {
    return std::make_shared<DebugReportCallback>(shared_from_this(), flags, callback, pUserData, allocator);
  }

  size_t Instance::getPhysicalDeviceCount() const
  {
    return(m_physicalDevices.size());
  }

  std::shared_ptr<vkhlf::PhysicalDevice> Instance::getPhysicalDevice(size_t index)
  {
    assert(index < m_physicalDevices.size());

    std::shared_ptr<vkhlf::PhysicalDevice> physicalDevice = m_physicalDevicesCache[index].lock();
    if ( !physicalDevice )
    {
      physicalDevice = std::make_shared<vkhlf::PhysicalDevice>(shared_from_this(), m_physicalDevices[index]);
      m_physicalDevicesCache[index] = physicalDevice;
    }
    return physicalDevice;
  }

  PFN_vkVoidFunction Instance::getProcAddress(std::string const& name) const
  {
    assert(!name.empty());
    return m_instance.getProcAddr(name);
  }
} // namespace vk
