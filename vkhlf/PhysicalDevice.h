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
#include <vkhlf/Device.h>
#include <vkhlf/Types.h>
#include <memory>

namespace vkhlf
{

  struct DisplayPlaneProperties
  {
    DisplayPlaneProperties(std::shared_ptr<Display> const& cd, uint32_t csi)
      : currentDisplay(cd)
      , currentStackIndex(csi)
    {}

    std::shared_ptr<Display>  currentDisplay;
    uint32_t                  currentStackIndex;
  };

  struct DisplayProperties
  {
    DisplayProperties(std::shared_ptr<Display> const& d, std::string const& dn, vk::Extent2D const& pd, vk::Extent2D const& pr, vk::SurfaceTransformFlagsKHR st, bool prp, bool pc)
      : display(d)
      , displayName(dn)
      , physicalDimensions(pd)
      , physicalResolution(pr)
      , supportedTransforms(st)
      , planeReorderPossible(prp)
      , persistentContent(pc)
    {}

    std::shared_ptr<Display>      display;
    std::string                   displayName;
    vk::Extent2D                  physicalDimensions;
    vk::Extent2D                  physicalResolution;
    vk::SurfaceTransformFlagsKHR  supportedTransforms;
    bool                          planeReorderPossible;
    bool                          persistentContent;
  };

  class PhysicalDevice : public Reference<Instance>, public std::enable_shared_from_this<PhysicalDevice>
  {
    public:
      VKHLF_API PhysicalDevice(std::shared_ptr<vkhlf::Instance> const& instance, vk::PhysicalDevice const& physicalDevice);
      VKHLF_API virtual ~PhysicalDevice();

    public:
      VKHLF_API std::shared_ptr<Device>                      createDevice(vk::ArrayProxy<const DeviceQueueCreateInfo> queueCreateInfos      = DeviceQueueCreateInfo(0, 1.0f),
                                                                       vk::ArrayProxy<const std::string>           enabledLayerNames     = nullptr,
                                                                       vk::ArrayProxy<const std::string>           enabledExtensionNames = nullptr,
                                                                       vk::PhysicalDeviceFeatures const&           enabledFeatures       = {},
                                                                       std::shared_ptr<Allocator> const&           allocator             = nullptr);
      VKHLF_API std::vector<DisplayPlaneProperties>          getDisplayPlaneProperties();
      VKHLF_API std::vector<DisplayProperties>               getDisplayProperties();
      VKHLF_API std::vector<vk::ExtensionProperties>         getExtensionProperties(std::string const& layerName) const;
      VKHLF_API vk::PhysicalDeviceFeatures                   getFeatures() const;
      VKHLF_API vk::FormatProperties                         getFormatProperties(vk::Format format) const;
      VKHLF_API vk::ImageFormatProperties                    getImageFormatProperties(vk::Format format, vk::ImageType type, vk::ImageTiling tiling, vk::ImageUsageFlags usage, vk::ImageCreateFlags flags) const;
      VKHLF_API std::vector<vk::LayerProperties>             getLayerProperties() const;
      VKHLF_API vk::PhysicalDeviceMemoryProperties           getMemoryProperties() const;
      VKHLF_API vk::PhysicalDeviceProperties                 getProperties() const;
      VKHLF_API std::vector<vk::QueueFamilyProperties>       getQueueFamilyProperties() const;
      VKHLF_API std::vector<vk::SparseImageFormatProperties> getSparseImageFormatProperties(vk::Format format, vk::ImageType type, vk::SampleCountFlagBits samples, vk::ImageUsageFlags usage, vk::ImageTiling tiling) const;
      VKHLF_API vk::SurfaceCapabilitiesKHR                   getSurfaceCapabilities(std::shared_ptr<Surface> const& surface) const;
      VKHLF_API std::vector<vk::SurfaceFormatKHR>            getSurfaceFormats(std::shared_ptr<Surface> const& surface) const;
      VKHLF_API std::vector<vk::PresentModeKHR>              getSurfacePresentModes(std::shared_ptr<Surface> const& surface) const;
      VKHLF_API bool                                         getSurfaceSupport(uint32_t queueFamilyIndex, std::shared_ptr<Surface> const& surface) const;
      VKHLF_API std::vector<std::shared_ptr<Display>>        getSupportedDisplays(uint32_t planeIndex);

      VKHLF_API operator vk::PhysicalDevice() const;

      PhysicalDevice(PhysicalDevice const& rhs) = delete;
      PhysicalDevice & operator=(PhysicalDevice const& rhs) = delete;

    private:
      vk::PhysicalDevice  m_physicalDevice;
  };

  VKHLF_API std::vector<uint32_t> getGraphicsPresentQueueFamilyIndices(std::shared_ptr<PhysicalDevice> const& physicalDevice, std::shared_ptr<Surface> const& surface);
  VKHLF_API std::vector<uint32_t> getQueueFamilyIndices(std::shared_ptr<PhysicalDevice> const& physicalDevice, vk::QueueFlags queueFlags);

  inline PhysicalDevice::operator vk::PhysicalDevice() const
  {
    return m_physicalDevice;
  }

} // namespace vk
