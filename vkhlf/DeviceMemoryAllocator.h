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
#include <vkhlf/Reference.h>

namespace vkhlf
{
  class DeviceMemoryAllocator : public Reference<Device, Allocator>
  {
    public:
      VKHLF_API DeviceMemoryAllocator(std::shared_ptr<Device> const& device, vk::DeviceSize chunkSize, std::shared_ptr<Allocator> const& hostAllocator);
      VKHLF_API virtual ~DeviceMemoryAllocator();

      VKHLF_API std::shared_ptr<DeviceMemory> allocate(vk::DeviceSize allocationSize, uint32_t memoryTypeIndex);

      DeviceMemoryAllocator(DeviceMemoryAllocator const& rhs) = delete;
      DeviceMemoryAllocator & operator=(DeviceMemoryAllocator const& rhs) = delete;

    private:
      struct ChunkData
      {
        ChunkData(std::shared_ptr<DeviceMemoryChunk> const& c, vk::DeviceSize o)
          : chunk(c)
          , offset(o)
        {}

        std::shared_ptr<DeviceMemoryChunk>  chunk;
        vk::DeviceSize                      offset;
      };

      std::map<uint32_t, ChunkData> m_chunks;       // map from memoryTypeIndex to current chunk
      vk::DeviceSize                m_chunkSize;
      std::shared_ptr<Device>       m_device;
  };

} // namespace vkh
