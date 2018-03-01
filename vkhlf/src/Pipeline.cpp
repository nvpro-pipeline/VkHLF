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
#include <vkhlf/Pipeline.h>
#include <vkhlf/PipelineCache.h>
#include <vkhlf/RenderPass.h>
#include <array>
#include <cmath>

namespace vkhlf
{
  SpecializationInfo::SpecializationInfo(vk::ArrayProxy<const vk::SpecializationMapEntry> mapEntries_, vk::ArrayProxy<const uint8_t> data_)
    : mapEntries(mapEntries_.begin(), mapEntries_.end())
    , data(data_.begin(), data_.end())
  {}

  SpecializationInfo::SpecializationInfo(SpecializationInfo const& rhs)
    : SpecializationInfo(rhs.mapEntries, rhs.data)
  {}

  SpecializationInfo & SpecializationInfo::operator=(SpecializationInfo const& rhs)
  {
    mapEntries = rhs.mapEntries;
    return *this;
  }


  PipelineColorBlendStateCreateInfo::PipelineColorBlendStateCreateInfo(bool logicEnable_, vk::LogicOp logicOp_, vk::ArrayProxy<const vk::PipelineColorBlendAttachmentState> attachments_, std::array<float, 4> const& blendConstants_)
    : logicEnable(logicEnable_)
    , logicOp(logicOp_)
    , attachments(attachments_.begin(), attachments_.end())
    , blendConstants(blendConstants_)
  {}

  PipelineColorBlendStateCreateInfo::PipelineColorBlendStateCreateInfo(PipelineColorBlendStateCreateInfo const& rhs)
    : PipelineColorBlendStateCreateInfo(rhs.logicEnable, rhs.logicOp, rhs.attachments, rhs.blendConstants)
  {}

  PipelineColorBlendStateCreateInfo & PipelineColorBlendStateCreateInfo::operator=(PipelineColorBlendStateCreateInfo const& rhs)
  {
    logicEnable     = rhs.logicEnable;
    logicOp         = rhs.logicOp;
    attachments     = rhs.attachments;
    blendConstants  = rhs.blendConstants;
    return *this;
  }


  PipelineDynamicStateCreateInfo::PipelineDynamicStateCreateInfo(vk::ArrayProxy<const vk::DynamicState> dynamicStates_)
    : dynamicStates(dynamicStates_.begin(), dynamicStates_.end())
  {}

  PipelineDynamicStateCreateInfo::PipelineDynamicStateCreateInfo(PipelineDynamicStateCreateInfo const& rhs)
    : PipelineDynamicStateCreateInfo(rhs.dynamicStates)
  {}

  PipelineDynamicStateCreateInfo & PipelineDynamicStateCreateInfo::operator=(PipelineDynamicStateCreateInfo const& rhs)
  {
    dynamicStates = rhs.dynamicStates;
    return *this;
  }


  PipelineMultisampleStateCreateInfo::PipelineMultisampleStateCreateInfo(vk::SampleCountFlagBits rasterizationSamples_, bool sampleShadingEnable_, float minSampleShading_,
                                                                         vk::ArrayProxy<const vk::SampleMask> sampleMasks_, bool alphaToCoverageEnable_, bool alphaToOneEnable_)
    : rasterizationSamples(rasterizationSamples_)
    , sampleShadingEnable(sampleShadingEnable_)
    , minSampleShading(minSampleShading_)
    , sampleMasks(sampleMasks_.begin(), sampleMasks_.end())
    , alphaToCoverageEnable(alphaToCoverageEnable_)
    , alphaToOneEnable(alphaToOneEnable_)
  {
    assert(sampleMasks.empty() || (std::ceil(static_cast<uint32_t>(sampleShadingEnable) / 32) <= sampleMasks.size()));
  }

  PipelineMultisampleStateCreateInfo::PipelineMultisampleStateCreateInfo(PipelineMultisampleStateCreateInfo const& rhs)
    : PipelineMultisampleStateCreateInfo(rhs.rasterizationSamples, rhs.sampleShadingEnable, rhs.minSampleShading, rhs.sampleMasks, rhs.alphaToCoverageEnable, rhs.alphaToOneEnable)
  {}

  PipelineMultisampleStateCreateInfo & PipelineMultisampleStateCreateInfo::operator=(PipelineMultisampleStateCreateInfo const& rhs)
  {
    rasterizationSamples  = rhs.rasterizationSamples;
    sampleShadingEnable   = rhs.sampleShadingEnable;
    minSampleShading      = rhs.minSampleShading;
    sampleMasks           = rhs.sampleMasks;
    alphaToCoverageEnable = rhs.alphaToCoverageEnable;
    alphaToOneEnable      = rhs.alphaToOneEnable;
    return *this;
  }


  PipelineShaderStageCreateInfo::PipelineShaderStageCreateInfo(vk::ShaderStageFlagBits stage_, std::shared_ptr<ShaderModule> const& module_, std::string const& name_,
                                                               vk::Optional<const SpecializationInfo> specializationInfo_)
    : stage(stage_)
    , module(module_)
    , name(name_)
    , specializationInfo(specializationInfo_ ? new SpecializationInfo(*specializationInfo_) : nullptr)
  {}

  PipelineShaderStageCreateInfo::PipelineShaderStageCreateInfo(PipelineShaderStageCreateInfo const& rhs)
    : PipelineShaderStageCreateInfo(rhs.stage, rhs.module, rhs.name, rhs.specializationInfo ? vk::Optional<const SpecializationInfo>(*rhs.specializationInfo.get()) : nullptr)
  {}

  PipelineShaderStageCreateInfo & PipelineShaderStageCreateInfo::operator=(PipelineShaderStageCreateInfo const& rhs)
  {
    stage = rhs.stage;
    module = rhs.module;
    name = rhs.name;
    specializationInfo.reset(rhs.specializationInfo ? new SpecializationInfo(*rhs.specializationInfo) : nullptr);
    return *this;
  }


  PipelineVertexInputStateCreateInfo::PipelineVertexInputStateCreateInfo(vk::ArrayProxy<const vk::VertexInputBindingDescription> vertexBindingDescriptions_,
                                                                         vk::ArrayProxy<const vk::VertexInputAttributeDescription> vertexAttributeDesriptions_)
    : vertexBindingDescriptions(vertexBindingDescriptions_.begin(), vertexBindingDescriptions_.end())
    , vertexAttributeDesriptions(vertexAttributeDesriptions_.begin(), vertexAttributeDesriptions_.end())
  {}

  PipelineVertexInputStateCreateInfo::PipelineVertexInputStateCreateInfo(PipelineVertexInputStateCreateInfo const& rhs)
    : PipelineVertexInputStateCreateInfo(rhs.vertexBindingDescriptions, rhs.vertexAttributeDesriptions)
  {}

  PipelineVertexInputStateCreateInfo & PipelineVertexInputStateCreateInfo::operator=(PipelineVertexInputStateCreateInfo const& rhs)
  {
    vertexBindingDescriptions = rhs.vertexBindingDescriptions;
    vertexAttributeDesriptions = rhs.vertexAttributeDesriptions;
    return *this;
  }


  PipelineViewportStateCreateInfo::PipelineViewportStateCreateInfo(vk::ArrayProxy<const vk::Viewport> viewports_, vk::ArrayProxy<const vk::Rect2D> scissors_)
    : viewports(viewports_.begin(), viewports_.end())
    , scissors(scissors_.begin(), scissors_.end())
  {}

  PipelineViewportStateCreateInfo::PipelineViewportStateCreateInfo(PipelineViewportStateCreateInfo const& rhs)
    : PipelineViewportStateCreateInfo(rhs.viewports, rhs.scissors)
  {}

  PipelineViewportStateCreateInfo & PipelineViewportStateCreateInfo::operator=(PipelineViewportStateCreateInfo const& rhs)
  {
    viewports = rhs.viewports;
    scissors = rhs.scissors;
    return *this;
  }


  /************************************************************************/
  /* Pipeline                                                             */
  /************************************************************************/
  Pipeline::Pipeline(std::shared_ptr<Device> const& device, std::shared_ptr<Allocator> const& allocator)
    : Reference(device, allocator)
  {}

  void Pipeline::setPipeline(vk::Pipeline const& pipeline)
  {
    m_pipeline = pipeline;
  }

  Pipeline::~Pipeline()
  {
    static_cast<vk::Device>(*get<Device>()).destroyPipeline(m_pipeline, *get<Allocator>());
  }

  ComputePipeline::ComputePipeline(std::shared_ptr<Device> const & device, std::shared_ptr<vkhlf::PipelineCache> const & pipelineCache, vk::PipelineCreateFlags flags,
                                   PipelineShaderStageCreateInfo const& stage, std::shared_ptr<PipelineLayout> const& layout, std::shared_ptr<Pipeline> const& basePipelineHandle,
                                   int32_t basePipelineIndex, std::shared_ptr<Allocator> const& allocator)
    : Pipeline(device, allocator)
  {
    vk::SpecializationInfo vkSpecializationInfo(vkhlf::checked_cast<uint32_t>(stage.specializationInfo->mapEntries.size()), stage.specializationInfo->mapEntries.data(),
                                                vkhlf::checked_cast<uint32_t>(stage.specializationInfo->data.size()), stage.specializationInfo->data.data());
    vk::PipelineShaderStageCreateInfo vkStage({}, stage.stage, stage.module ? static_cast<vk::ShaderModule>(*stage.module) : nullptr, stage.name.data(), &vkSpecializationInfo);

    vk::ComputePipelineCreateInfo createInfo(flags, vkStage, layout ? static_cast<vk::PipelineLayout>(*layout) : nullptr,
                                             basePipelineHandle ? static_cast<vk::Pipeline>(*basePipelineHandle) : nullptr, basePipelineIndex);

    setPipeline(static_cast<vk::Device>(*get<Device>()).createComputePipeline(pipelineCache ? static_cast<vk::PipelineCache>(*pipelineCache) : nullptr, createInfo, *get<Allocator>()));
  }

  GraphicsPipeline::GraphicsPipeline(std::shared_ptr<Device> const & device, std::shared_ptr<PipelineCache> const& pipelineCache, vk::PipelineCreateFlags flags,
                                     vk::ArrayProxy<const PipelineShaderStageCreateInfo> stages, vk::Optional<const PipelineVertexInputStateCreateInfo> vertexInputState,
                                     vk::Optional<const vk::PipelineInputAssemblyStateCreateInfo> inputAssemblyState, vk::Optional<const vk::PipelineTessellationStateCreateInfo> tessellationState,
                                     vk::Optional<const PipelineViewportStateCreateInfo> viewportState, vk::Optional<const vk::PipelineRasterizationStateCreateInfo> rasterizationState,
                                     vk::Optional<const PipelineMultisampleStateCreateInfo> multisampleState, vk::Optional<const vk::PipelineDepthStencilStateCreateInfo> depthStencilState,
                                     vk::Optional<const PipelineColorBlendStateCreateInfo> colorBlendState, vk::Optional<const PipelineDynamicStateCreateInfo> dynamicState,
                                     std::shared_ptr<PipelineLayout> const& pipelineLayout, std::shared_ptr<RenderPass> const& renderPass, uint32_t subpass,
                                     std::shared_ptr<Pipeline> const& basePipelineHandle, uint32_t basePipelineIndex, std::shared_ptr<Allocator> const& allocator)
    : Pipeline(device, allocator)
  {
    std::vector<vk::SpecializationInfo> specializationInfos;
    specializationInfos.reserve(stages.size());

    std::vector<vk::PipelineShaderStageCreateInfo> vkStages;
    vkStages.reserve(stages.size());
    for (auto const& s : stages)
    {
      if (s.specializationInfo)
      {
        specializationInfos.push_back(vk::SpecializationInfo(vkhlf::checked_cast<uint32_t>(s.specializationInfo->mapEntries.size()), s.specializationInfo->mapEntries.data(),
                                                             vkhlf::checked_cast<uint32_t>(s.specializationInfo->data.size()), s.specializationInfo->data.data()));
      }
      vkStages.push_back(vk::PipelineShaderStageCreateInfo({}, s.stage, s.module ? static_cast<vk::ShaderModule>(*s.module) : nullptr, s.name.data(),
                                                           s.specializationInfo ? &specializationInfos.back() : nullptr));
    }

    vk::PipelineVertexInputStateCreateInfo vkVertexInputState;
    if (vertexInputState)
    {
      vkVertexInputState = vk::PipelineVertexInputStateCreateInfo({}, vkhlf::checked_cast<uint32_t>(vertexInputState->vertexBindingDescriptions.size()), vertexInputState->vertexBindingDescriptions.data(),
                                                                  vkhlf::checked_cast<uint32_t>(vertexInputState->vertexAttributeDesriptions.size()), vertexInputState->vertexAttributeDesriptions.data());
    }

    vk::PipelineViewportStateCreateInfo vkViewportState;
    if (viewportState)
    {
      vkViewportState = vk::PipelineViewportStateCreateInfo({}, vkhlf::checked_cast<uint32_t>(viewportState->viewports.size()), viewportState->viewports.data(),
                                                            vkhlf::checked_cast<uint32_t>(viewportState->scissors.size()), viewportState->scissors.data());
    }

    vk::PipelineMultisampleStateCreateInfo vkMultisampleState;
    if (multisampleState)
    {
      vkMultisampleState = vk::PipelineMultisampleStateCreateInfo({}, multisampleState->rasterizationSamples, multisampleState->sampleShadingEnable, multisampleState->minSampleShading,
                                                                  multisampleState->sampleMasks.empty() ? nullptr : multisampleState->sampleMasks.data(), multisampleState->alphaToCoverageEnable,
                                                                  multisampleState->alphaToOneEnable);
    }

    vk::PipelineColorBlendStateCreateInfo vkColorBlendState;
    if (colorBlendState)
    {
      vkColorBlendState = vk::PipelineColorBlendStateCreateInfo({}, colorBlendState->logicEnable, colorBlendState->logicOp, vkhlf::checked_cast<uint32_t>(colorBlendState->attachments.size()),
                                                                colorBlendState->attachments.data(), colorBlendState->blendConstants);
    }

    vk::PipelineDynamicStateCreateInfo vkDynamicState;
    if (dynamicState)
    {
      vkDynamicState = vk::PipelineDynamicStateCreateInfo({}, vkhlf::checked_cast<uint32_t>(dynamicState->dynamicStates.size()), dynamicState->dynamicStates.data());
    }

    vk::GraphicsPipelineCreateInfo createInfo(flags, vkhlf::checked_cast<uint32_t>(vkStages.size()), vkStages.data(), vertexInputState ? &vkVertexInputState : nullptr, inputAssemblyState, tessellationState,
                                              viewportState ? &vkViewportState : nullptr, rasterizationState, multisampleState ? &vkMultisampleState : nullptr, depthStencilState,
                                              colorBlendState ? &vkColorBlendState : nullptr, dynamicState ? &vkDynamicState : nullptr, pipelineLayout ? *pipelineLayout : vk::PipelineLayout(),
                                              renderPass ? *renderPass : vk::RenderPass(), subpass, basePipelineHandle ? *basePipelineHandle : vk::Pipeline(), basePipelineIndex);
    setPipeline(static_cast<vk::Device>(*get<Device>()).createGraphicsPipeline(pipelineCache ? *pipelineCache : vk::PipelineCache(), createInfo, *get<Allocator>()));
  }

} // namespace vk
