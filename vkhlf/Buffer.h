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
#include <vkhlf/CommandBuffer.h>
#include <vkhlf/Device.h>
#include <vkhlf/DeviceMemory.h>

namespace vkhlf
{
  class Buffer : public Reference<Device,DeviceMemory,Allocator>, public std::enable_shared_from_this<Buffer>
  {
    public:
      VKHLF_API Buffer(std::shared_ptr<Device> const & device, vk::BufferCreateFlags createFlags, vk::DeviceSize size, vk::BufferUsageFlags usageFlags, vk::SharingMode sharingMode,
                    vk::ArrayProxy<const uint32_t> queueFamilyIndices, vk::MemoryPropertyFlags memoryPropertyFlags, std::shared_ptr<DeviceMemoryAllocator> const& deviceMemoryAllocator,
                    std::shared_ptr<Allocator> const& bufferAllocator);
      VKHLF_API virtual ~Buffer();

      VKHLF_API std::shared_ptr<vkhlf::BufferView>     createBufferView(vk::Format format, vk::DeviceSize offset = 0, vk::DeviceSize range = ~0, std::shared_ptr<Allocator> const& allocator = nullptr);
      VKHLF_API vk::DeviceSize                       getSize() const;
      VKHLF_API vk::MemoryPropertyFlags              getMemoryPropertyFlags() const;
      VKHLF_API vk::MemoryRequirements               getMemoryRequirements() const;
      template <typename T> void                  update(vk::DeviceSize offset, vk::ArrayProxy<const T> data, std::shared_ptr<CommandBuffer> const& commandBuffer);

      VKHLF_API operator vk::Buffer() const;

      Buffer(Buffer const& rhs) = delete;
      Buffer & operator=(Buffer const& rhs) = delete;

    private:
      vk::Buffer              m_buffer;
      vk::MemoryPropertyFlags m_memoryPropertyFlags;
      vk::DeviceSize          m_size;
  };

  inline vk::DeviceSize Buffer::getSize() const
  {
    return m_size;
  }

  inline Buffer::operator vk::Buffer() const
  {
    return m_buffer;
  }

  template <typename T>
  inline void Buffer::update(vk::DeviceSize offset, vk::ArrayProxy<const T> data, std::shared_ptr<CommandBuffer> const& commandBuffer)
  {
    size_t size = data.size() * sizeof(T);
    if (((offset & 0x3) == 0) && (size < 64 * 1024) && ((size & 0x3) == 0))
    {
      commandBuffer->updateBuffer(shared_from_this(), offset, data);
    }
    else if (getMemoryPropertyFlags() & vk::MemoryPropertyFlagBits::eHostVisible)
    {
      void * pData = get<DeviceMemory>()->map(offset, size);
      memcpy(pData, data.data(), size);
      if (!(getMemoryPropertyFlags() & vk::MemoryPropertyFlagBits::eHostCoherent))
      {
        get<DeviceMemory>()->flush(offset, size);
      }
      get<DeviceMemory>()->unmap();
    }
    else
    {
      std::shared_ptr<Buffer> mappingBuffer = get<Device>()->createBuffer(m_size, vk::BufferUsageFlagBits::eTransferSrc, vk::SharingMode::eExclusive, nullptr, vk::MemoryPropertyFlagBits::eHostVisible,
                                                                          nullptr, get<Allocator>());
      void * pData = mappingBuffer->get<DeviceMemory>()->map(offset, size);
      memcpy(pData, data.data(), size);
      mappingBuffer->get<DeviceMemory>()->flush(offset, size);
      mappingBuffer->get<DeviceMemory>()->unmap();
      commandBuffer->copyBuffer(mappingBuffer, shared_from_this(), vk::BufferCopy(0, 0, size));
    }
  }


} // namespace vk
