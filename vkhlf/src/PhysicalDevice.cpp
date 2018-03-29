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
#include <vkhlf/Device.h>
#include <vkhlf/Display.h>
#include <vkhlf/PhysicalDevice.h>
#include <vkhlf/Surface.h>

#if !defined(NDEBUG)
#include <set>
#endif

namespace vkhlf
{
  PhysicalDevice::PhysicalDevice(std::shared_ptr<vkhlf::Instance> const& instance, vk::PhysicalDevice const& physicalDevice)
    : Reference(instance)
    , m_physicalDevice(physicalDevice)
  {
  }

  PhysicalDevice::~PhysicalDevice( )
  {
  }

  std::shared_ptr<Device> PhysicalDevice::createDevice(vk::ArrayProxy<const DeviceQueueCreateInfo>  queueCreateInfos,
                                                       vk::ArrayProxy<const std::string>            enabledLayerNames,
                                                       vk::ArrayProxy<const std::string>            enabledExtensionNames,
                                                       vk::PhysicalDeviceFeatures const&            enabledFeatures,
                                                       std::shared_ptr<Allocator> const&            allocator)
  {
    assert(!queueCreateInfos.empty());
#if !defined(NDEBUG)
    std::set<uint32_t> familyIndices;
    for (auto const& ci : queueCreateInfos)
    {
      assert(!ci.queuePriorities.empty());
      assert(familyIndices.insert(ci.queueFamilyIndex).second);   // queueFamilyIndex must be unique within the queueCreateInfos
    }
#endif
    // TODO: assert that for each DeviceQueueCreateInfo
    // - queueFamilyIndex is less than pQueueFamilyPropertyCount returned by vkGetPhysicalDeviceQueueFamilyProperties
    // - queueCount is less than or equal to the value of the queueCount member of the VkQueueFamilyProperties structure, as returned by vkGetPhysicalDeviceQueueFamilyProperties in the pQueueFamilyProperties[queueFamilyIndex]

#if !defined(NDEBUG)
    if (std::find(enabledLayerNames.begin(), enabledLayerNames.end(), "VK_LAYER_LUNARG_standard_validation") == enabledLayerNames.end())
    {
      std::vector<std::string> eln(enabledLayerNames.begin(), enabledLayerNames.end());
      eln.push_back("VK_LAYER_LUNARG_standard_validation");
      return Device::create(shared_from_this(), queueCreateInfos, eln, enabledExtensionNames, enabledFeatures, allocator);
    }
#endif
    return Device::create(shared_from_this(), queueCreateInfos, enabledLayerNames, enabledExtensionNames, enabledFeatures, allocator);
  }

  std::vector<DisplayPlaneProperties> PhysicalDevice::getDisplayPlaneProperties()
  {
    std::vector<vk::DisplayPlanePropertiesKHR> vkDisplayPlaneProperties
#if !defined(VK_OS_MAC) && !defined(VK_OS_IOS)
    = m_physicalDevice.getDisplayPlanePropertiesKHR()
#endif
    ;
    std::vector<DisplayPlaneProperties> displayPlaneProperties;
    displayPlaneProperties.reserve(vkDisplayPlaneProperties.size());
    for (auto const& dpp : vkDisplayPlaneProperties)
    {
      displayPlaneProperties.push_back(DisplayPlaneProperties(std::make_shared<Display>(shared_from_this(), dpp.currentDisplay), dpp.currentStackIndex));
    }
    return displayPlaneProperties;
  }

  std::vector<DisplayProperties> PhysicalDevice::getDisplayProperties()
  {
    std::vector<vk::DisplayPropertiesKHR> vkDisplayProperties
#if !defined(VK_OS_MAC) && !defined(VK_OS_IOS)
    = m_physicalDevice.getDisplayPropertiesKHR()
#endif
    ;
    std::vector<DisplayProperties> displayProperties;
    displayProperties.reserve(vkDisplayProperties.size());
    for (auto const& dp : vkDisplayProperties)
    {
      displayProperties.push_back(DisplayProperties(std::make_shared<Display>(shared_from_this(), dp.display), dp.displayName, dp.physicalDimensions, dp.physicalResolution, dp.supportedTransforms,
                                                    !!dp.planeReorderPossible, !!dp.persistentContent));
    }
    return displayProperties;
  }

  std::vector<vk::ExtensionProperties> PhysicalDevice::getExtensionProperties(std::string const& layerName) const
  {
    return m_physicalDevice.enumerateDeviceExtensionProperties(layerName);
  }

  vk::PhysicalDeviceFeatures PhysicalDevice::getFeatures() const
  {
    return m_physicalDevice.getFeatures();
  }

  vk::FormatProperties PhysicalDevice::getFormatProperties(vk::Format format) const
  {
    return m_physicalDevice.getFormatProperties(format);
  }

  vk::ImageFormatProperties PhysicalDevice::getImageFormatProperties(vk::Format format, vk::ImageType type, vk::ImageTiling tiling, vk::ImageUsageFlags usage, vk::ImageCreateFlags flags) const
  {
    return m_physicalDevice.getImageFormatProperties(format, type, tiling, usage, flags);
  }

  std::vector<vk::LayerProperties> PhysicalDevice::getLayerProperties() const
  {
    return m_physicalDevice.enumerateDeviceLayerProperties();
  }

  vk::PhysicalDeviceMemoryProperties PhysicalDevice::getMemoryProperties() const
  {
    return m_physicalDevice.getMemoryProperties();
  }

  vk::PhysicalDeviceProperties PhysicalDevice::getProperties() const
  {
    return m_physicalDevice.getProperties();
  }

  std::vector<vk::QueueFamilyProperties> PhysicalDevice::getQueueFamilyProperties() const
  {
    return m_physicalDevice.getQueueFamilyProperties();
  }

  std::vector<vk::SparseImageFormatProperties> PhysicalDevice::getSparseImageFormatProperties(vk::Format format, vk::ImageType type, vk::SampleCountFlagBits samples, vk::ImageUsageFlags usage, vk::ImageTiling tiling) const
  {
    return m_physicalDevice.getSparseImageFormatProperties(format, type, samples, usage, tiling);
  }

  vk::SurfaceCapabilitiesKHR PhysicalDevice::getSurfaceCapabilities(std::shared_ptr<Surface> const& surface) const
  {
    return m_physicalDevice.getSurfaceCapabilitiesKHR(static_cast<vk::SurfaceKHR>(*surface));
  }

  std::vector<vk::SurfaceFormatKHR> PhysicalDevice::getSurfaceFormats(std::shared_ptr<Surface> const& surface) const
  {
    return m_physicalDevice.getSurfaceFormatsKHR(static_cast<vk::SurfaceKHR>(*surface));
  }

  std::vector<vk::PresentModeKHR> PhysicalDevice::getSurfacePresentModes(std::shared_ptr<Surface> const& surface) const
  {
    return m_physicalDevice.getSurfacePresentModesKHR(static_cast<vk::SurfaceKHR>(*surface));
  }

  bool PhysicalDevice::getSurfaceSupport(uint32_t queueFamilyIndex, std::shared_ptr<Surface> const& surface) const
  {
    return !!m_physicalDevice.getSurfaceSupportKHR(queueFamilyIndex, static_cast<vk::SurfaceKHR>(*surface));
  }

  std::vector<std::shared_ptr<Display>> PhysicalDevice::getSupportedDisplays(uint32_t planeIndex)
  {
    std::vector<vk::DisplayKHR> vkDisplays
#if !defined(VK_OS_MAC) && !defined(VK_OS_IOS)
    = m_physicalDevice.getDisplayPlaneSupportedDisplaysKHR(planeIndex)
#endif    
    ;
    std::vector<std::shared_ptr<Display>> displays;
    displays.reserve(vkDisplays.size());
    for (auto const& d : vkDisplays)
    {
      displays.push_back(std::make_shared<Display>(shared_from_this(), d));
    }
    return displays;
  }

  std::vector<uint32_t> getGraphicsPresentQueueFamilyIndices(std::shared_ptr<PhysicalDevice> const& physicalDevice, std::shared_ptr<Surface> const& surface)
  {
    std::vector<vk::QueueFamilyProperties> props = physicalDevice->getQueueFamilyProperties();
    assert(!props.empty());

    std::vector<uint32_t> indices;
    for (size_t i = 0; i < props.size(); i++)
    {
      if ((props[i].queueFlags & vk::QueueFlagBits::eGraphics) && physicalDevice->getSurfaceSupport(vkhlf::checked_cast<uint32_t>(i), surface))
      {
        indices.push_back(vkhlf::checked_cast<uint32_t>(i));
      }
    }
    return indices;
  }

  std::vector<uint32_t> getQueueFamilyIndices(std::shared_ptr<PhysicalDevice> const& physicalDevice, vk::QueueFlags queueFlags)
  {
    std::vector<vk::QueueFamilyProperties> props = physicalDevice->getQueueFamilyProperties();
    assert(!props.empty());

    std::vector<uint32_t> indices;
    for (size_t i = 0; i < props.size(); i++)
    {
      if ((props[i].queueFlags & queueFlags) == queueFlags)
      {
        indices.push_back(vkhlf::checked_cast<uint32_t>(i));
      }
    }
    return indices;
  }

} // namespace vk
