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
  VKHLF_API uint32_t determineMemoryTypeIndex(vk::PhysicalDeviceMemoryProperties const& memoryProperties, uint32_t typeBits, vk::MemoryPropertyFlags requirements_mask);

  class DeviceMemory : public Reference<DeviceMemoryChunk>
  {
    public:
      VKHLF_API DeviceMemory(std::shared_ptr<DeviceMemoryChunk> const& chunk, vk::DeviceSize offset, vk::DeviceSize size);
      VKHLF_API virtual ~DeviceMemory();

      VKHLF_API void           flush(vk::DeviceSize offset, vk::DeviceSize size) const;
      VKHLF_API vk::DeviceSize getCommitment() const;
      VKHLF_API vk::DeviceSize getOffset() const;
      VKHLF_API vk::DeviceSize getSize() const;
      VKHLF_API void           invalidate(vk::DeviceSize offset, vk::DeviceSize size) const;
      VKHLF_API void *         map(vk::DeviceSize offset, vk::DeviceSize size);
      VKHLF_API void           unmap();

      DeviceMemory(DeviceMemory const& rhs) = delete;
      DeviceMemory & operator=(DeviceMemory const& rhs) = delete;

    private:
      vk::DeviceSize  m_offset;
      vk::DeviceSize  m_size;
  };

  inline vk::DeviceSize DeviceMemory::getOffset() const
  {
    return m_offset;
  }

  inline vk::DeviceSize DeviceMemory::getSize() const
  {
    return m_size;
  }

  struct MappedDeviceMemory
  {
    public:
      VKHLF_API MappedDeviceMemory(std::shared_ptr<vkhlf::DeviceMemory> const& deviceMemory, vk::DeviceSize offset, vk::DeviceSize size);
      VKHLF_API ~MappedDeviceMemory();

      VKHLF_API void set(vk::DeviceSize offset, vk::DeviceSize size, void const* pData);

    private:
      std::shared_ptr<vkhlf::DeviceMemory>  m_deviceMemory;
      void *                              m_data;
      vk::DeviceSize                      m_size;
  };

} // namespace vkh
