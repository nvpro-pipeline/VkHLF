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


#include <vkhlf/BufferView.h>
#include <vkhlf/DescriptorPool.h>
#include <vkhlf/DescriptorSet.h>
#include <vkhlf/Device.h>

namespace vkhlf
{
  CopyDescriptorSet::CopyDescriptorSet(std::shared_ptr<DescriptorSet> const& srcSet_, uint32_t srcBinding_, uint32_t srcArrayElement_, std::shared_ptr<DescriptorSet> const& dstSet_, uint32_t dstBinding_,
                                       uint32_t dstArrayElement_, uint32_t descriptorCount_)
    : srcSet(srcSet_)
    , srcBinding(srcBinding_)
    , srcArrayElement(srcArrayElement_)
    , dstSet(dstSet_)
    , dstBinding(dstBinding_)
    , dstArrayElement(dstArrayElement_)
    , descriptorCount(descriptorCount_)
  {}

  CopyDescriptorSet::CopyDescriptorSet(CopyDescriptorSet const& rhs)
    : CopyDescriptorSet(rhs.srcSet, rhs.srcBinding, rhs.srcArrayElement, rhs.dstSet, rhs.dstBinding, rhs.dstArrayElement, rhs.descriptorCount)
  {}

  CopyDescriptorSet & CopyDescriptorSet::operator=(CopyDescriptorSet const& rhs)
  {
    srcSet          = rhs.srcSet;
    srcBinding      = rhs.srcBinding;
    srcArrayElement = rhs.srcArrayElement;
    dstSet          = rhs.dstSet;
    dstBinding      = rhs.dstBinding;
    dstArrayElement = rhs.dstArrayElement;
    descriptorCount = rhs.descriptorCount;
    return *this;
  }


  DescriptorBufferInfo::DescriptorBufferInfo(std::shared_ptr<Buffer> const& buffer_, vk::DeviceSize offset_, vk::DeviceSize range_)
    : buffer(buffer_)
    , offset(offset_)
    , range(range_)
  {
  }

  DescriptorBufferInfo::DescriptorBufferInfo(DescriptorBufferInfo const& rhs)
    : DescriptorBufferInfo(rhs.buffer, rhs.offset, rhs.range)
  {}

  DescriptorBufferInfo & DescriptorBufferInfo::operator=(DescriptorBufferInfo const& rhs)
  {
    buffer  = rhs.buffer;
    offset  = rhs.offset;
    range   = rhs.range;
    return *this;
  }


  DescriptorImageInfo::DescriptorImageInfo(std::shared_ptr<Sampler> const& sampler_, std::shared_ptr<ImageView> const& imageView_, vk::ImageLayout imageLayout_)
    : sampler(sampler_)
    , imageView(imageView_)
    , imageLayout(imageLayout_)
  {
  }

  DescriptorImageInfo::DescriptorImageInfo(DescriptorImageInfo const& rhs)
    : DescriptorImageInfo(rhs.sampler, rhs.imageView, rhs.imageLayout)
  {}

  DescriptorImageInfo & DescriptorImageInfo::operator=(DescriptorImageInfo const& rhs)
  {
    sampler     = rhs.sampler;
    imageView   = rhs.imageView;
    imageLayout = rhs.imageLayout;
    return *this;
  }


  WriteDescriptorSet::WriteDescriptorSet(std::shared_ptr<DescriptorSet> const& dstSet_, uint32_t dstBinding_, uint32_t dstArrayElement_, uint32_t descriptorCount_, vk::DescriptorType descriptorType_,
                                         vk::Optional<const DescriptorImageInfo> imageInfo_, vk::Optional<const DescriptorBufferInfo> bufferInfo_, std::shared_ptr<BufferView> const& texelBufferView_)
    : dstSet(dstSet_)
    , dstBinding(dstBinding_)
    , dstArrayElement(dstArrayElement_)
    , descriptorCount(descriptorCount_)
    , descriptorType(descriptorType_)
    , imageInfo(imageInfo_ ? new DescriptorImageInfo(*imageInfo_) : nullptr)
    , bufferInfo(bufferInfo_ ? new DescriptorBufferInfo(*bufferInfo_) : nullptr)
    , texelBufferView(texelBufferView_)
  {
  }

  WriteDescriptorSet::WriteDescriptorSet(WriteDescriptorSet const& rhs)
    : WriteDescriptorSet(rhs.dstSet, rhs.dstBinding, rhs.dstArrayElement, rhs.descriptorCount, rhs.descriptorType,
                         rhs.imageInfo.get() ? vk::Optional<const DescriptorImageInfo>(*rhs.imageInfo) : vk::Optional<const DescriptorImageInfo>(nullptr),
                         rhs.bufferInfo.get() ? vk::Optional<const DescriptorBufferInfo>(*rhs.bufferInfo) : vk::Optional<const DescriptorBufferInfo>(nullptr), rhs.texelBufferView)
  {}

  WriteDescriptorSet & WriteDescriptorSet::operator=(WriteDescriptorSet const& rhs)
  {
    dstSet          = rhs.dstSet;
    dstBinding      = rhs.dstBinding;
    dstArrayElement = rhs.dstArrayElement;
    descriptorCount = rhs.descriptorCount;
    descriptorType  = rhs.descriptorType;
    imageInfo.reset(rhs.imageInfo ? new DescriptorImageInfo(*rhs.imageInfo) : nullptr);
    bufferInfo.reset(rhs.bufferInfo ? new DescriptorBufferInfo(*rhs.bufferInfo) : nullptr);
    texelBufferView = rhs.texelBufferView;
    return *this;
  }


  DescriptorSet::DescriptorSet(std::shared_ptr<Device> const & device, std::shared_ptr<vkhlf::DescriptorPool> const& descriptorPool, std::shared_ptr<vkhlf::DescriptorSetLayout> const& layout)
    : Reference(device, descriptorPool, layout)
  {
    vk::DescriptorSetLayout vkLayout = *layout;
    vk::DescriptorSetAllocateInfo allocateInfo(*get<DescriptorPool>(), 1, &vkLayout);

    std::vector<vk::DescriptorSet> vds = static_cast<vk::Device>(*get<Device>()).allocateDescriptorSets(allocateInfo);
    assert(vds.size() == 1);
    m_descriptorSet = vds[0];
  }

  DescriptorSet::~DescriptorSet()
  {
    static_cast<vk::Device>(*get<Device>()).freeDescriptorSets(*get<DescriptorPool>(), m_descriptorSet);
  }

} // namespace vk
