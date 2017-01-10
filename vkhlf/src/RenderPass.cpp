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
#include <vkhlf/RenderPass.h>

namespace vkhlf
{

  RenderPass::RenderPass(std::shared_ptr<Device> const& device, vk::ArrayProxy<const vk::AttachmentDescription> attachments, vk::ArrayProxy<const vk::SubpassDescription> subpasses,
                         vk::ArrayProxy<const vk::SubpassDependency> dependencies, std::shared_ptr<Allocator> const& allocator)
    : Reference(device, allocator)
  {
    vk::RenderPassCreateInfo createInfo({}, vkhlf::checked_cast<uint32_t>(attachments.size()), attachments.data(), vkhlf::checked_cast<uint32_t>(subpasses.size()), subpasses.data(),
                                        vkhlf::checked_cast<uint32_t>(dependencies.size()), dependencies.data());
    m_renderPass = static_cast<vk::Device>(*get<Device>()).createRenderPass(createInfo, *get<Allocator>());
  }

  RenderPass::~RenderPass()
  {
    static_cast<vk::Device>(*get<Device>()).destroyRenderPass(m_renderPass, *get<Allocator>());
  }

  VkExtent2D RenderPass::getRenderAreaGranularity() const
  {
    return static_cast<vk::Device>(*get<Device>()).getRenderAreaGranularity(m_renderPass);
  }

#if !defined(NDEBUG)
  bool RenderPass::isCompatible(std::shared_ptr<vkhlf::RenderPass> const& otherPass) const
  {
    // TODO: determine, what compatibility means here !!
    return true;
  }
#endif

} // namespace vk
