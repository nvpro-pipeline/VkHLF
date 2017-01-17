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


#include <vkhlf/Allocator.h>
#include <vkhlf/CommandBuffer.h>
#include <vkhlf/Device.h>
#include <vkhlf/DeviceMemoryAllocator.h>
#include <vkhlf/DeviceMemoryChunk.h>
#include <vkhlf/Image.h>
#include <vkhlf/ImageView.h>
#include <vkhlf/PhysicalDevice.h>
#include <vkhlf/Queue.h>

namespace vkhlf
{

  Image::Image(std::shared_ptr<Device> const& device, vk::Image const& image, std::shared_ptr<Allocator> const& allocator)
    : Reference(device, nullptr, allocator)
    , m_image(image)
    , m_managed(false)
  {
  }

  Image::Image(std::shared_ptr<Device> const& device, vk::ImageCreateFlags createFlags, vk::ImageType type, vk::Format format, vk::Extent3D extent, uint32_t mipLevels, uint32_t arrayLayers,
               vk::SampleCountFlagBits samples, vk::ImageTiling tiling, vk::ImageUsageFlags usageFlags, vk::SharingMode sharingMode, std::vector<uint32_t> const& queueFamilyIndices,
               vk::ImageLayout initialLayout, vk::MemoryPropertyFlags memoryPropertyFlags, std::shared_ptr<DeviceMemoryAllocator> const& deviceMemoryAllocator,
               std::shared_ptr<Allocator> const& imageAllocator)
    : Reference(device, nullptr, imageAllocator)
    , m_arrayLayers(arrayLayers)
    , m_extent(extent)
    , m_format(format)
    , m_managed(true)
    , m_memoryPropertyFlags(memoryPropertyFlags)
    , m_mipLevels(mipLevels)
    , m_queueFamilyIndices(queueFamilyIndices)
    , m_samples(samples)
    , m_sharingMode(sharingMode)
    , m_tiling(tiling)
    , m_type(type)
  {
    vk::ImageCreateInfo createInfo(createFlags, m_type, m_format, m_extent, m_mipLevels, m_arrayLayers, m_samples, m_tiling, usageFlags, m_sharingMode,
                                   vkhlf::checked_cast<uint32_t>(m_queueFamilyIndices.size()), m_queueFamilyIndices.data(), initialLayout);
    m_image = static_cast<vk::Device>(*get<Device>()).createImage(createInfo, *get<Allocator>());

    vk::MemoryRequirements memReqs = getMemoryRequirements();
    uint32_t memoryTypeIndex = determineMemoryTypeIndex(get<Device>()->get<PhysicalDevice>()->getMemoryProperties(), memReqs.memoryTypeBits, m_memoryPropertyFlags);
    assert(memoryTypeIndex != ~0);
    set<DeviceMemory>(get<Device>()->allocateMemory(memReqs.size, memoryTypeIndex, deviceMemoryAllocator ? deviceMemoryAllocator : std::make_shared<DeviceMemoryAllocator>(get<Device>(), 0, nullptr)));
    static_cast<vk::Device>(*get<Device>()).bindImageMemory(m_image, static_cast<vk::DeviceMemory>(*get<DeviceMemory>()->get<DeviceMemoryChunk>()), get<DeviceMemory>()->getOffset());
  }

  Image::~Image( )
  {
    if (m_managed)
    {
      static_cast<vk::Device>(*get<Device>()).destroyImage(m_image, *get<Allocator>());
    }
  }

  vk::MemoryPropertyFlags Image::getMemoryPropertyFlags() const
  {
    return m_memoryPropertyFlags;
  }

  vk::MemoryRequirements Image::getMemoryRequirements() const
  {
    return static_cast<vk::Device>(*get<Device>()).getImageMemoryRequirements(m_image);
  }

  uint32_t Image::getMipLevels() const
  {
    return m_mipLevels;
  }

  std::vector<uint32_t> const& Image::getQueueFamilyIndices() const
  {
    return m_queueFamilyIndices;
  }

  vk::SampleCountFlagBits Image::getSamples() const
  {
    return m_samples;
  }

  vk::SharingMode Image::getSharingMode() const
  {
    return m_sharingMode;
  }

  std::vector<vk::SparseImageMemoryRequirements> Image::getSparseImageMemoryRequirements() const
  {
    return static_cast<vk::Device>(*get<Device>()).getImageSparseMemoryRequirements(m_image);
  }

  vk::SubresourceLayout Image::getSubresourceLayout(vk::ImageAspectFlags aspect, uint32_t mipLevel, uint32_t arrayLayer) const
  {
    vk::ImageSubresource subresource(aspect, mipLevel, arrayLayer);
    return static_cast<vk::Device>(*get<Device>()).getImageSubresourceLayout(m_image, subresource);
  }

  vk::ImageTiling Image::getTiling() const
  {
    return m_tiling;
  }

  vk::ImageType Image::getType() const
  {
    return m_type;
  }

  std::shared_ptr<vkhlf::ImageView> Image::createImageView(vk::ImageViewType viewType, vk::Format format, vk::ComponentMapping components, vk::ImageSubresourceRange subresourceRange,
                                                         std::shared_ptr<Allocator> const& allocator)
  {
    return std::make_shared<ImageView>(shared_from_this(), vk::ImageViewCreateFlags(), viewType, format, components, subresourceRange, allocator);
  }

  uint32_t Image::getArrayLayers() const
  {
    return m_arrayLayers;
  }

  vk::Extent3D const& Image::getExtent() const
  {
    assert(m_managed);
    return m_extent;
  }

  vk::Format Image::getFormat() const
  {
    return m_format;
  }

  vk::ImageMemoryBarrier Image::getImageLayoutBarrier(vk::ImageLayout layout)
  {
    // TODO support depth images!
    // TODO support arrays/mip mapping!
    vk::ImageSubresourceRange subresourceRange(vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1);
    vk::ImageMemoryBarrier memBarrier;

    memBarrier.srcAccessMask    = vk::AccessFlagBits::eColorAttachmentRead;
    memBarrier.dstAccessMask    = vk::AccessFlagBits::eColorAttachmentWrite;
    memBarrier.oldLayout        = vk::ImageLayout::eGeneral;
    memBarrier.newLayout        = vk::ImageLayout::eColorAttachmentOptimal;
    memBarrier.image            = m_image;
    memBarrier.subresourceRange = subresourceRange;

    return memBarrier;
  }

  // Keeping this piece of code since it'll be required in the queue/cmdbuffer/image/device. Not sure where to put it now.
#if 0
  // image transition to attachment layout
  {
    cmdBuffer = cmdPool->createCmdBuffer();
    cmdBuffer->begin(VK_CMD_BUFFER_OPTIMIZE_ONE_TIME_SUBMIT_BIT);

    VkImageMemoryBarrier imageBarrier = colorImage->getImageLayoutBarrier(VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
    cmdBuffer->pipelineBarrier(VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, VK_FALSE, &imageBarrier);
    cmdBuffer->end();
    submitAndWait(queue, cmdBuffer);
  }
#endif

  MappedImage::MappedImage(std::shared_ptr<Image> const& image, std::shared_ptr<CommandBuffer> const& commandBuffer, vk::DeviceSize offset, vk::DeviceSize size)
    : m_commandBuffer(commandBuffer)
    , m_image(image)
    , m_offset(offset)
    , m_size((size == VK_WHOLE_SIZE) ? image->get<DeviceMemory>()->getSize() - offset : size)
  {
    if (m_image->getMemoryPropertyFlags() & vk::MemoryPropertyFlagBits::eHostVisible)
    {
      m_pData = m_image->get<DeviceMemory>()->map(m_offset, m_size);
    }
    else
    {
      m_mappingImage = m_image->get<Device>()->createImage({}, m_image->getType(), m_image->getFormat(), m_image->getExtent(), m_image->getMipLevels(), m_image->getArrayLayers(), m_image->getSamples(),
                                                           vk::ImageTiling::eLinear, vk::ImageUsageFlagBits::eTransferSrc, m_image->getSharingMode(), m_image->getQueueFamilyIndices(),
                                                           vk::ImageLayout::ePreinitialized, vk::MemoryPropertyFlagBits::eHostVisible, nullptr, m_image->get<Allocator>());
      setImageLayout(m_commandBuffer, m_mappingImage, vk::ImageAspectFlagBits::eColor, vk::ImageLayout::ePreinitialized, vk::ImageLayout::eGeneral);
      m_pData = m_mappingImage->get<DeviceMemory>()->map(m_offset, m_size);
    }
  }

  MappedImage::~MappedImage()
  {
    if (m_image->getMemoryPropertyFlags() & vk::MemoryPropertyFlagBits::eHostVisible)
    {
      if (!(m_image->getMemoryPropertyFlags() & vk::MemoryPropertyFlagBits::eHostCoherent))
      {
        m_image->get<DeviceMemory>()->flush(m_offset, m_size);
      }
      m_image->get<DeviceMemory>()->unmap();
    }
    else
    {
      m_mappingImage->get<DeviceMemory>()->flush(m_offset, m_size);
      m_mappingImage->get<DeviceMemory>()->unmap();

      setImageLayout(m_commandBuffer, m_mappingImage, vk::ImageAspectFlagBits::eColor, vk::ImageLayout::eGeneral, vk::ImageLayout::eTransferSrcOptimal);
      setImageLayout(m_commandBuffer, m_image, vk::ImageAspectFlagBits::eColor, vk::ImageLayout::eUndefined, vk::ImageLayout::eTransferDstOptimal);
      m_commandBuffer->copyImage(m_mappingImage, vk::ImageLayout::eTransferSrcOptimal, m_image, vk::ImageLayout::eTransferDstOptimal,
                                 vk::ImageCopy(vk::ImageSubresourceLayers(vk::ImageAspectFlagBits::eColor, 0, 0, 1), vk::Offset3D(0, 0, 0),
                                               vk::ImageSubresourceLayers(vk::ImageAspectFlagBits::eColor, 0, 0, 1), vk::Offset3D(0, 0, 0), m_image->getExtent()));
      setImageLayout(m_commandBuffer, m_image, vk::ImageAspectFlagBits::eColor, vk::ImageLayout::eTransferDstOptimal, vk::ImageLayout::eShaderReadOnlyOptimal);
    }
  }

  void * MappedImage::getPointer()
  {
    return m_pData;
  }

  vk::SubresourceLayout MappedImage::getSubresourceLayout(vk::ImageAspectFlags aspect, uint32_t mipLevel, uint32_t arrayLayer) const
  {
    if (m_image->getMemoryPropertyFlags() & vk::MemoryPropertyFlagBits::eHostVisible)
    {
      return m_image->getSubresourceLayout(aspect, mipLevel, arrayLayer);
    }
    else
    {
      return m_mappingImage->getSubresourceLayout(aspect, mipLevel, arrayLayer);
    }
  }

} // namespace vkh
