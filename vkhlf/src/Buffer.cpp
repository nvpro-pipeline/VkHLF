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


#include <vkhlf/Buffer.h>
#include <vkhlf/BufferView.h>
#include <vkhlf/CommandBuffer.h>
#include <vkhlf/CommandPool.h>
#include <vkhlf/Device.h>
#include <vkhlf/DeviceMemoryAllocator.h>
#include <vkhlf/DeviceMemoryChunk.h>
#include <vkhlf/PhysicalDevice.h>
#include <vkhlf/Queue.h>

namespace vkhlf
{

  Buffer::Buffer(std::shared_ptr<Device> const& device, vk::BufferCreateFlags createFlags, vk::DeviceSize size, vk::BufferUsageFlags usageFlags, vk::SharingMode sharingMode,
                 vk::ArrayProxy<const uint32_t> queueFamilyIndices, vk::MemoryPropertyFlags memoryPropertyFlags, std::shared_ptr<DeviceMemoryAllocator> const& deviceMemoryAllocator,
                 std::shared_ptr<Allocator> const& bufferAllocator)
    : Reference(device, nullptr, bufferAllocator)
    , m_memoryPropertyFlags(memoryPropertyFlags)
    , m_size(size)
  {
    vk::BufferCreateInfo createInfo(createFlags, size, usageFlags, sharingMode, vkhlf::checked_cast<uint32_t>(queueFamilyIndices.size()), queueFamilyIndices.data());
    m_buffer = static_cast<vk::Device>(*get<Device>()).createBuffer(createInfo, *get<Allocator>());

    vk::MemoryRequirements memReqs = getMemoryRequirements();
    uint32_t memoryTypeIndex = determineMemoryTypeIndex(get<Device>()->get<PhysicalDevice>()->getMemoryProperties(), memReqs.memoryTypeBits, m_memoryPropertyFlags);
    assert(memoryTypeIndex != ~0 && "No mappable, coherent memory");
    set<DeviceMemory>(get<Device>()->allocateMemory(memReqs.size, memoryTypeIndex, deviceMemoryAllocator ? deviceMemoryAllocator : std::make_shared<DeviceMemoryAllocator>(get<Device>(), 0, nullptr)));
    static_cast<vk::Device>(*get<Device>()).bindBufferMemory(m_buffer, static_cast<vk::DeviceMemory>(*get<DeviceMemory>()->get<DeviceMemoryChunk>()), get<DeviceMemory>()->getOffset());
  }

  Buffer::~Buffer( )
  {
    static_cast<vk::Device>(*get<Device>()).destroyBuffer(m_buffer, *get<Allocator>());
  }

  std::shared_ptr<BufferView> Buffer::createBufferView(vk::Format format, vk::DeviceSize offset, vk::DeviceSize range, std::shared_ptr<Allocator> const& allocator)
  {
    return std::make_shared<BufferView>(shared_from_this(), format, offset, range, allocator);
  }

  vk::MemoryPropertyFlags Buffer::getMemoryPropertyFlags() const
  {
    return m_memoryPropertyFlags;
  }

  vk::MemoryRequirements Buffer::getMemoryRequirements() const
  {
    return static_cast<vk::Device>(*get<Device>()).getBufferMemoryRequirements(m_buffer);
  }

} // namespace vk
