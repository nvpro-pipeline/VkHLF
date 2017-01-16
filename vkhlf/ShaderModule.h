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
#include <vkhlf/Types.h>
#include <vkhlf/Reference.h>
#include <vulkan/../../glslang/glslang/Public/ShaderLang.h>
#include <vulkan/../../glslang/SPIRV/GlslangToSpv.h>
#include <vector>

namespace vkhlf
{

  class ShaderModule : public Reference<Device, Allocator>, public std::enable_shared_from_this<ShaderModule>
  {
    public:
      VKHLF_API ShaderModule(std::shared_ptr<Device> const & device, vk::ArrayProxy<const uint32_t> code, std::shared_ptr<Allocator> const& allocator);
      VKHLF_API ~ShaderModule();

      VKHLF_API operator vk::ShaderModule() const;

      ShaderModule(ShaderModule const& rhs) = delete;
      ShaderModule & operator=(ShaderModule const& rhs) = delete;

    private:
      vk::ShaderModule m_shaderModule;
  };

  class GLSLToSpearVConverter
  {
    public:
      VKHLF_API GLSLToSpearVConverter();
      VKHLF_API ~GLSLToSpearVConverter();

      VKHLF_API std::vector<uint32_t> convert(vk::ShaderStageFlagBits stage, std::string const & source);

    private:
      TBuiltInResource  m_resource;
  };

  inline ShaderModule::operator vk::ShaderModule() const
  {
    return m_shaderModule;
  }

} // namespace vk
