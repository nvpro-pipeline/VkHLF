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


#include <vkhlf/Device.h>
#include <vkhlf/Sampler.h>

namespace vkhlf
{
  Sampler::Sampler(std::shared_ptr<Device> const & device, vk::Filter magFilter, vk::Filter minFilter, vk::SamplerMipmapMode mipmapMode, vk::SamplerAddressMode addressModeU,
                   vk::SamplerAddressMode addressModeV, vk::SamplerAddressMode addressModeW, float mipLodBias, bool anisotropyEnable, float maxAnisotropy, bool compareEnable,
                   vk::CompareOp compareOp, float minLod, float maxLod, vk::BorderColor borderColor, bool unnormalizedCoordinates, std::shared_ptr<Allocator> const& allocator)
    : Reference(device, allocator)
  {
    vk::SamplerCreateInfo ci({}, magFilter, minFilter, mipmapMode, addressModeU, addressModeV, addressModeW, mipLodBias, anisotropyEnable, maxAnisotropy, compareEnable, compareOp, minLod, maxLod,
                             borderColor, unnormalizedCoordinates);
    m_sampler = static_cast<vk::Device>(*get<Device>()).createSampler(ci, *get<Allocator>());
  }

  Sampler::~Sampler()
  {
    static_cast<vk::Device>(*get<Device>()).destroySampler(m_sampler, *get<Allocator>());
  }

} // namespace vk
