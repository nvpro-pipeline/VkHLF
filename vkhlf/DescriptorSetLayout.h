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

#include <vkhlf/Allocator.h>
#include <vkhlf/Config.h>
#include <vkhlf/Reference.h>
#include <vkhlf/Types.h>

namespace vkhlf
{

  struct DescriptorSetLayoutBinding
  {
    VKHLF_API DescriptorSetLayoutBinding(uint32_t binding, vk::DescriptorType descriptorType, vk::ShaderStageFlags stageFlags, vk::ArrayProxy<const std::shared_ptr<vkhlf::Sampler>> immutableSamplers);
    VKHLF_API DescriptorSetLayoutBinding(DescriptorSetLayoutBinding const& rhs);
    VKHLF_API DescriptorSetLayoutBinding & operator=(DescriptorSetLayoutBinding const& rhs);

    uint32_t                                      binding;
    vk::DescriptorType                            descriptorType;
    vk::ShaderStageFlags                          stageFlags;
    std::vector<std::shared_ptr<vkhlf::Sampler>>  immutableSamplers;
  };

  class DescriptorSetLayout : public Reference<Device, Allocator>
  {
    public:
      VKHLF_API DescriptorSetLayout(std::shared_ptr<Device> const & device, vk::ArrayProxy<const DescriptorSetLayoutBinding> bindings, std::shared_ptr<Allocator> const& allocator);
      VKHLF_API ~DescriptorSetLayout();

      std::vector<DescriptorSetLayoutBinding> const& getBindings() { return m_bindings; }

      VKHLF_API operator vk::DescriptorSetLayout() const;

      DescriptorSetLayout(DescriptorSetLayout const& rhs) = delete;
      DescriptorSetLayout & operator=(DescriptorSetLayout const& rhs) = delete;

    private:
      std::vector<DescriptorSetLayoutBinding> m_bindings;
      vk::DescriptorSetLayout                 m_descriptorSetLayout;
  };

  inline DescriptorSetLayout::operator vk::DescriptorSetLayout() const
  {
    return m_descriptorSetLayout;
  }

} // namespace vk
