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
#include <vkhlf/DeviceMemoryAllocator.h>
#include <vkhlf/DeviceMemoryChunk.h>
#include <vkhlf/PhysicalDevice.h>

namespace vkhlf
{

  DeviceMemoryAllocator::DeviceMemoryAllocator(std::shared_ptr<Device> const& device, vk::DeviceSize chunkSize, std::shared_ptr<Allocator> const& hostAllocator)
    : Reference(device, hostAllocator)
    , m_chunkSize(chunkSize)
  {}

  DeviceMemoryAllocator::~DeviceMemoryAllocator()
  {}

  std::shared_ptr<DeviceMemory> DeviceMemoryAllocator::allocate(vk::DeviceSize allocationSize, uint32_t memoryTypeIndex)
  {
    if (m_chunkSize < allocationSize)
    {
      std::shared_ptr<DeviceMemoryChunk> chunk = std::make_shared<DeviceMemoryChunk>(get<Device>(), allocationSize, memoryTypeIndex, get<Allocator>());
      return std::make_shared<DeviceMemory>(chunk, 0, allocationSize);
    }

    auto chunkIt = m_chunks.find(memoryTypeIndex);
    if (chunkIt == m_chunks.end())
    {
      chunkIt = m_chunks.insert(std::make_pair(memoryTypeIndex, ChunkData(std::make_shared<DeviceMemoryChunk>(get<Device>(), m_chunkSize, memoryTypeIndex, get<Allocator>()), 0))).first;
    }
    if ( m_chunkSize < chunkIt->second.offset + allocationSize)
    {
      chunkIt->second.chunk = std::make_shared<DeviceMemoryChunk>(get<Device>(), m_chunkSize, memoryTypeIndex, get<Allocator>());
      chunkIt->second.offset = 0;
    }
    chunkIt->second.offset += allocationSize;

    return std::make_shared<DeviceMemory>(chunkIt->second.chunk, chunkIt->second.offset - allocationSize, allocationSize);
  }

} // namespace vkh
