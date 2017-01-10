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
#include <vkhlf/DescriptorSetLayout.h>
#include <vkhlf/Device.h>

namespace vkhlf
{
  DescriptorSetLayoutBinding::DescriptorSetLayoutBinding(uint32_t binding_, vk::DescriptorType descriptorType_, vk::ShaderStageFlags stageFlags_, vk::ArrayProxy<const std::shared_ptr<vkhlf::Sampler>> immutableSamplers_)
    : binding(binding_)
    , descriptorType(descriptorType_)
    , stageFlags(stageFlags_)
    , immutableSamplers(immutableSamplers_.begin(), immutableSamplers_.end())
  {}

  DescriptorSetLayoutBinding::DescriptorSetLayoutBinding(DescriptorSetLayoutBinding const& rhs)
    : DescriptorSetLayoutBinding(rhs.binding, rhs.descriptorType, rhs.stageFlags, rhs.immutableSamplers)
  {}

  DescriptorSetLayoutBinding & DescriptorSetLayoutBinding::operator=(DescriptorSetLayoutBinding const& rhs)
  {
    binding           = rhs.binding;
    descriptorType    = rhs.descriptorType;
    stageFlags        = rhs.stageFlags;
    immutableSamplers = rhs.immutableSamplers;
    return *this;
  }


  DescriptorSetLayout::DescriptorSetLayout(std::shared_ptr<Device> const & device, vk::ArrayProxy<const DescriptorSetLayoutBinding> bindings, std::shared_ptr<Allocator> const& allocator)
    : Reference(device, allocator)
    , m_bindings(bindings.begin(), bindings.end())
  {
    std::vector<std::vector<vk::Sampler>> samplers;
    samplers.reserve(m_bindings.size());

    std::vector<vk::DescriptorSetLayoutBinding> dslb;
    dslb.reserve(m_bindings.size());
    for ( auto b : m_bindings )
    {
      samplers.push_back(std::vector<vk::Sampler>());
      samplers.back().reserve(b.immutableSamplers.size());
      for (auto s : b.immutableSamplers)
      {
        samplers.back().push_back(static_cast<vk::Sampler>(*s));
      }

      uint32_t desciptorCount = 1;
      if (b.descriptorType == vk::DescriptorType::eSampler)
      {
        desciptorCount = vkhlf::checked_cast<uint32_t>(samplers.back().size());
      }

      dslb.push_back(vk::DescriptorSetLayoutBinding(b.binding, b.descriptorType, desciptorCount, b.stageFlags, samplers.back().data()));
    }

    vk::DescriptorSetLayoutCreateInfo createInfo({}, vkhlf::checked_cast<uint32_t>(dslb.size()), dslb.data());

    m_descriptorSetLayout = static_cast<vk::Device>(*get<Device>()).createDescriptorSetLayout(createInfo, *get<Allocator>());
  }

  DescriptorSetLayout::~DescriptorSetLayout()
  {
    static_cast<vk::Device>(*get<Device>()).destroyDescriptorSetLayout(m_descriptorSetLayout, *get<Allocator>());
  }

} // namespace vk
