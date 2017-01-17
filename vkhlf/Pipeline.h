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
#include <vkhlf/Reference.h>
#include <vkhlf/ShaderModule.h>

namespace vkhlf
{

  struct SpecializationInfo
  {
    VKHLF_API SpecializationInfo(vk::ArrayProxy<const vk::SpecializationMapEntry> mapEntries, vk::ArrayProxy<const uint8_t> data);
    VKHLF_API SpecializationInfo(SpecializationInfo const& rhs);
    VKHLF_API SpecializationInfo & operator=(SpecializationInfo const& rhs);

    std::vector<vk::SpecializationMapEntry> mapEntries;
    std::vector<uint8_t>                    data;
  };

  struct PipelineColorBlendStateCreateInfo
  {
    VKHLF_API PipelineColorBlendStateCreateInfo(bool logicEnable, vk::LogicOp logicOp, vk::ArrayProxy<const vk::PipelineColorBlendAttachmentState> attachments, std::array<float,4> const& blendConstants);
    VKHLF_API PipelineColorBlendStateCreateInfo(PipelineColorBlendStateCreateInfo const& rhs);
    VKHLF_API PipelineColorBlendStateCreateInfo & operator=(PipelineColorBlendStateCreateInfo const& rhs);

    bool                                                logicEnable;
    vk::LogicOp                                         logicOp;
    std::vector<vk::PipelineColorBlendAttachmentState>  attachments;
    std::array<float, 4>                                blendConstants;
  };

  struct PipelineDynamicStateCreateInfo
  {
    VKHLF_API PipelineDynamicStateCreateInfo(vk::ArrayProxy<const vk::DynamicState> dynamicStates);
    VKHLF_API PipelineDynamicStateCreateInfo(PipelineDynamicStateCreateInfo const& rhs);
    VKHLF_API PipelineDynamicStateCreateInfo & operator=(PipelineDynamicStateCreateInfo const& rhs);

    std::vector<vk::DynamicState> dynamicStates;
  };

  struct PipelineMultisampleStateCreateInfo
  {
    // TODO: find a way to keep rasterizationSamples and sampleMasks in sync!!
    VKHLF_API PipelineMultisampleStateCreateInfo(vk::SampleCountFlagBits rasterizationSamples, bool sampleShadingEnable, float minSampleShading, vk::ArrayProxy<const vk::SampleMask> sampleMasks,
                                              bool alphaToCoverageEnable, bool alphaToOneEnable);
    VKHLF_API PipelineMultisampleStateCreateInfo(PipelineMultisampleStateCreateInfo const &rhs);
    VKHLF_API PipelineMultisampleStateCreateInfo & operator=(PipelineMultisampleStateCreateInfo const& rhs);

    vk::SampleCountFlagBits     rasterizationSamples;
    bool                        sampleShadingEnable;
    float                       minSampleShading;
    std::vector<vk::SampleMask> sampleMasks;
    bool                        alphaToCoverageEnable;
    bool                        alphaToOneEnable;
  };

  struct PipelineShaderStageCreateInfo
  {
    VKHLF_API PipelineShaderStageCreateInfo(vk::ShaderStageFlagBits stage, std::shared_ptr<ShaderModule> const& module, std::string const& name,
                                         vk::Optional<const SpecializationInfo> specializationInfo = nullptr);
    VKHLF_API PipelineShaderStageCreateInfo(PipelineShaderStageCreateInfo const& rhs);
    VKHLF_API PipelineShaderStageCreateInfo & operator=(PipelineShaderStageCreateInfo const& rhs);

    vk::ShaderStageFlagBits             stage;
    std::shared_ptr<ShaderModule>       module;
    std::string                         name;
    std::unique_ptr<SpecializationInfo> specializationInfo;
  };

  struct PipelineVertexInputStateCreateInfo
  {
    VKHLF_API PipelineVertexInputStateCreateInfo(vk::ArrayProxy<const vk::VertexInputBindingDescription> vertexBindingDescriptions,
                                              vk::ArrayProxy<const vk::VertexInputAttributeDescription> vertexAttributeDesriptions);
    VKHLF_API PipelineVertexInputStateCreateInfo(PipelineVertexInputStateCreateInfo const& rhs);
    VKHLF_API PipelineVertexInputStateCreateInfo & operator=(PipelineVertexInputStateCreateInfo const& rhs);

    std::vector<vk::VertexInputBindingDescription>    vertexBindingDescriptions;
    std::vector<vk::VertexInputAttributeDescription>  vertexAttributeDesriptions;
  };

  struct PipelineViewportStateCreateInfo
  {
    VKHLF_API PipelineViewportStateCreateInfo(vk::ArrayProxy<const vk::Viewport> viewports_, vk::ArrayProxy<const vk::Rect2D> scissors_);
    VKHLF_API PipelineViewportStateCreateInfo(PipelineViewportStateCreateInfo const& rhs);
    VKHLF_API PipelineViewportStateCreateInfo & operator=(PipelineViewportStateCreateInfo const& rhs);

    std::vector<vk::Viewport> viewports;
    std::vector<vk::Rect2D>   scissors;
  };


  class Pipeline : public Reference<Device, Allocator>
  {
    public:
      VKHLF_API ~Pipeline();

      VKHLF_API operator vk::Pipeline() const;

      Pipeline(Pipeline const& rhs) = delete;
      Pipeline & operator=(Pipeline const& rhs) = delete;

    protected:
      VKHLF_API Pipeline(std::shared_ptr<Device> const& device, std::shared_ptr<Allocator> const& allocator);
      VKHLF_API void setPipeline(vk::Pipeline const& pipeline);

    private:
      vk::Pipeline m_pipeline;
  };

  inline Pipeline::operator vk::Pipeline() const
  {
    return m_pipeline;
  }

  class ComputePipeline : public Pipeline
  {
    public:
      VKHLF_API ComputePipeline(std::shared_ptr<Device> const & device, std::shared_ptr<vkhlf::PipelineCache> const & pipelineCache, vk::PipelineCreateFlags flags,
                             PipelineShaderStageCreateInfo const& stage, std::shared_ptr<PipelineLayout> const& layout, std::shared_ptr<Pipeline> const& basePipelineHandle,
                             int32_t basePipelineIndex, std::shared_ptr<Allocator> const& allocator);
  };

  class GraphicsPipeline : public Pipeline
  {
    public:
      VKHLF_API GraphicsPipeline(std::shared_ptr<Device> const & device, std::shared_ptr<PipelineCache> const& pipelineCache, vk::PipelineCreateFlags flags,
                              vk::ArrayProxy<const PipelineShaderStageCreateInfo> stages, vk::Optional<const PipelineVertexInputStateCreateInfo> vertexInputState,
                              vk::Optional<const vk::PipelineInputAssemblyStateCreateInfo> inputAssemblyState, vk::Optional<const vk::PipelineTessellationStateCreateInfo> tessellationState,
                              vk::Optional<const PipelineViewportStateCreateInfo> viewportState, vk::Optional<const vk::PipelineRasterizationStateCreateInfo> rasterizationState,
                              vk::Optional<const PipelineMultisampleStateCreateInfo> multisampleState, vk::Optional<const vk::PipelineDepthStencilStateCreateInfo> depthStencilState,
                              vk::Optional<const PipelineColorBlendStateCreateInfo> colorBlendState, vk::Optional<const PipelineDynamicStateCreateInfo> dynamicState,
                              std::shared_ptr<PipelineLayout> const& pipelineLayout, std::shared_ptr<RenderPass> const& renderPass, uint32_t subpass,
                              std::shared_ptr<Pipeline> const& basePipelineHandle, uint32_t basePipelineIndex, std::shared_ptr<Allocator> const& allocator);
  };

} // namespace vk
