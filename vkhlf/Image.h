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
#include <vkhlf/DeviceMemory.h>
#include <vector>

namespace vkhlf
{

  /************************************************************************/
  /* Image                                                                */
  /************************************************************************/
  class Image : public Reference<Device, DeviceMemory, Allocator>, public std::enable_shared_from_this<Image>
  {
    public:
      VKHLF_API Image(std::shared_ptr<Device> const& device, vk::Image const& image, std::shared_ptr<Allocator> const& allocator = nullptr);
      VKHLF_API Image(std::shared_ptr<Device> const& device, vk::ImageCreateFlags createFlags, vk::ImageType type, vk::Format format, vk::Extent3D extent, uint32_t mipLevels, uint32_t arrayLayers,
                   vk::SampleCountFlagBits samples, vk::ImageTiling tiling, vk::ImageUsageFlags usageFlags, vk::SharingMode sharingMode, std::vector<uint32_t> const& queueFamilyIndices,
                   vk::ImageLayout initialLayout, vk::MemoryPropertyFlags memoryPropertyFlags, std::shared_ptr<DeviceMemoryAllocator> const& deviceMemoryAllocator,
                   std::shared_ptr<Allocator> const& imageAllocator);
      VKHLF_API virtual ~Image();

      VKHLF_API std::shared_ptr<vkhlf::ImageView>                createImageView(vk::ImageViewType viewType, vk::Format format,
                                                                            vk::ComponentMapping components = { vk::ComponentSwizzle::eR, vk::ComponentSwizzle::eG, vk::ComponentSwizzle::eB, vk::ComponentSwizzle::eA },
                                                                            vk::ImageSubresourceRange subresourceRange = { vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1 },
                                                                            std::shared_ptr<Allocator> const& allocator = nullptr);
      VKHLF_API uint32_t                                       getArrayLayers() const;
      VKHLF_API vk::Extent3D const&                            getExtent() const;
      VKHLF_API vk::Format                                     getFormat() const;
      VKHLF_API vk::ImageMemoryBarrier                         getImageLayoutBarrier(vk::ImageLayout layout);
      VKHLF_API vk::MemoryPropertyFlags                        getMemoryPropertyFlags() const;
      VKHLF_API vk::MemoryRequirements                         getMemoryRequirements() const;
      VKHLF_API uint32_t                                       getMipLevels() const;
      VKHLF_API std::vector<uint32_t> const&                   getQueueFamilyIndices() const;
      VKHLF_API vk::SampleCountFlagBits                        getSamples() const;
      VKHLF_API vk::SharingMode                                getSharingMode() const;
      VKHLF_API std::vector<vk::SparseImageMemoryRequirements> getSparseImageMemoryRequirements() const;
      VKHLF_API vk::SubresourceLayout                          getSubresourceLayout(vk::ImageAspectFlags aspect, uint32_t mipLevel, uint32_t arrayLayer) const;
      VKHLF_API vk::ImageTiling                                getTiling() const;
      VKHLF_API vk::ImageType                                  getType() const;

      VKHLF_API operator vk::Image () const;

      Image(Image const& rhs) = delete;
      Image & operator=(Image const& rhs) = delete;

    private:
      uint32_t                m_arrayLayers;
      vk::Extent3D            m_extent;
      vk::Format              m_format;
      vk::Image               m_image;
      bool                    m_managed;
      vk::MemoryPropertyFlags m_memoryPropertyFlags;
      uint32_t                m_mipLevels;
      std::vector<uint32_t>   m_queueFamilyIndices;
      vk::SampleCountFlagBits m_samples;
      vk::SharingMode         m_sharingMode;
      vk::ImageTiling         m_tiling;
      vk::ImageType           m_type;
  };

  class MappedImage
  {
    public:
      VKHLF_API MappedImage(std::shared_ptr<Image> const& image, std::shared_ptr<CommandBuffer> const& commandBuffer, vk::DeviceSize offset = 0, vk::DeviceSize size = VK_WHOLE_SIZE);
      VKHLF_API ~MappedImage();

      VKHLF_API void *                 getPointer();
      VKHLF_API vk::SubresourceLayout  getSubresourceLayout(vk::ImageAspectFlags aspect, uint32_t mipLevel, uint32_t arrayLayer) const;

    private:
      std::shared_ptr<CommandBuffer>  m_commandBuffer;
      void *                          m_pData;
      std::shared_ptr<Image>          m_image;
      std::shared_ptr<Image>          m_mappingImage;
      vk::DeviceSize                  m_offset;
      vk::DeviceSize                  m_size;
  };


  inline Image::operator vk::Image() const
  {
    return m_image;
  }

} // namespace vkh
