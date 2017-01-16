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
#include <vkhlf/Buffer.h>
#include <vkhlf/ImageView.h>
#include <vkhlf/Sampler.h>

namespace vkhlf
{

  struct CopyDescriptorSet
  {
    VKHLF_API CopyDescriptorSet(std::shared_ptr<DescriptorSet> const& srcSet, uint32_t srcBinding, uint32_t srcArrayElement, std::shared_ptr<DescriptorSet> const& dstSet, uint32_t dstBinding,
                              uint32_t dstArrayElement, uint32_t descriptorCount);
    VKHLF_API CopyDescriptorSet(CopyDescriptorSet const& rhs);
    VKHLF_API CopyDescriptorSet & operator=(CopyDescriptorSet const& rhs);

    std::shared_ptr<DescriptorSet>  srcSet;
    uint32_t                        srcBinding;
    uint32_t                        srcArrayElement;
    std::shared_ptr<DescriptorSet>  dstSet;
    uint32_t                        dstBinding;
    uint32_t                        dstArrayElement;
    uint32_t                        descriptorCount;
  };

  struct DescriptorBufferInfo
  {
    VKHLF_API DescriptorBufferInfo(std::shared_ptr<Buffer> const& buffer, vk::DeviceSize offset, vk::DeviceSize range);
    VKHLF_API DescriptorBufferInfo(DescriptorBufferInfo const& rhs);
    VKHLF_API DescriptorBufferInfo & operator=(DescriptorBufferInfo const& rhs);

    std::shared_ptr<Buffer>   buffer;
    vk::DeviceSize            offset;
    vk::DeviceSize            range;
  };

  struct DescriptorImageInfo
  {
    VKHLF_API DescriptorImageInfo(std::shared_ptr<Sampler> const& sampler, std::shared_ptr<ImageView> const& imageView, vk::ImageLayout imageLayout);
    VKHLF_API DescriptorImageInfo(DescriptorImageInfo const& rhs);
    VKHLF_API DescriptorImageInfo & operator=(DescriptorImageInfo const& rhs);

    std::shared_ptr<Sampler>    sampler;
    std::shared_ptr<ImageView>  imageView;
    vk::ImageLayout             imageLayout;
  };


  struct WriteDescriptorSet
  {
    VKHLF_API WriteDescriptorSet(WriteDescriptorSet const& rhs);
    VKHLF_API WriteDescriptorSet(std::shared_ptr<DescriptorSet> const& dstSet, uint32_t dstBinding, uint32_t dstArrayElement, uint32_t descriptorCount, vk::DescriptorType descriptorType,
                              vk::Optional<const DescriptorImageInfo> imageInfo, vk::Optional<const DescriptorBufferInfo> bufferInfo, std::shared_ptr<BufferView> const& texelBufferView = {});
    VKHLF_API WriteDescriptorSet & operator=(WriteDescriptorSet const& rhs);

    std::shared_ptr<DescriptorSet>        dstSet;
    uint32_t                              dstBinding;
    uint32_t                              dstArrayElement;
    uint32_t                              descriptorCount;
    vk::DescriptorType                    descriptorType;
    std::unique_ptr<DescriptorImageInfo>  imageInfo;
    std::unique_ptr<DescriptorBufferInfo> bufferInfo;
    std::shared_ptr<BufferView>           texelBufferView;
  };


  class DescriptorSet : public Reference<Device, DescriptorPool, DescriptorSetLayout>
  {
    public:
      VKHLF_API DescriptorSet(std::shared_ptr<Device> const & device, std::shared_ptr<vkhlf::DescriptorPool> const& descriptorPool, std::shared_ptr<vkhlf::DescriptorSetLayout> const& layout);
      VKHLF_API ~DescriptorSet();

      VKHLF_API operator vk::DescriptorSet() const;

      DescriptorSet(DescriptorSet const& rhs) = delete;
      DescriptorSet & operator=(DescriptorSet const& rhs) = delete;

    private:
      vk::DescriptorSet m_descriptorSet;
  };

  inline DescriptorSet::operator vk::DescriptorSet() const
  {
    return m_descriptorSet;
  }

} // namespace vk
