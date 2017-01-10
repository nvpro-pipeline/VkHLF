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
#include <vkhlf/Device.h>
#include <vkhlf/DeviceMemory.h>
#include <vkhlf/DeviceMemoryChunk.h>
#include <vkhlf/PhysicalDevice.h>

namespace vkhlf
{

  uint32_t determineMemoryTypeIndex(vk::PhysicalDeviceMemoryProperties const& memoryProperties, uint32_t typeBits, vk::MemoryPropertyFlags requirements_mask)
  {
    // Search memtypes to find first index with those properties
    for (uint32_t i = 0; i < memoryProperties.memoryTypeCount; i++)
    {
      if ((typeBits & 1) == 1)
      {
        // Type is available, does it match user properties?
        if ((memoryProperties.memoryTypes[i].propertyFlags & requirements_mask) == requirements_mask)
        {
          return i;
        }
      }
      typeBits >>= 1;
    }
    // No memory types matched, return failure
    return ~0;
  }

  DeviceMemory::DeviceMemory(std::shared_ptr<DeviceMemoryChunk> const& chunk, vk::DeviceSize offset, vk::DeviceSize size)
    : Reference(chunk)
    , m_offset(offset)
    , m_size(size)
  {}

  DeviceMemory::~DeviceMemory( )
  {}

  void DeviceMemory::flush(vk::DeviceSize offset, vk::DeviceSize size) const
  {
    assert( offset + size <= m_size );
    get<DeviceMemoryChunk>()->flush(m_offset + offset, size);
  }

  vk::DeviceSize DeviceMemory::getCommitment() const
  {
    return get<DeviceMemoryChunk>()->getCommitment();
  }

  void DeviceMemory::invalidate(vk::DeviceSize offset, vk::DeviceSize size) const
  {
    assert( offset + size <= m_size );
    get<DeviceMemoryChunk>()->invalidate(m_offset + offset, size);
  }

  void * DeviceMemory::map(vk::DeviceSize offset, vk::DeviceSize size)
  {
    assert(offset + size <= m_size);
    return get<DeviceMemoryChunk>()->map(m_offset + offset, size);
  }

  void DeviceMemory::unmap()
  {
    get<DeviceMemoryChunk>()->unmap();
  }

  MappedDeviceMemory::MappedDeviceMemory( std::shared_ptr<vkhlf::DeviceMemory> const& deviceMemory, vk::DeviceSize offset, vk::DeviceSize size )
    : m_deviceMemory( deviceMemory )
    , m_data( nullptr )
    , m_size(size)
  {
    m_data = m_deviceMemory->map( offset, size );
  }

  MappedDeviceMemory::~MappedDeviceMemory()
  {
    m_deviceMemory->unmap();
  }

  void MappedDeviceMemory::set(vk::DeviceSize offset, vk::DeviceSize size, void const* pData)
  {
    assert(offset + size <= m_size);
    memcpy(reinterpret_cast<char*>(m_data) + offset, pData, size);
  }

} // namespace vkh
