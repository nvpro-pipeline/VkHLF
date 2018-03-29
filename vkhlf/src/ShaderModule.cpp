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
#include <vkhlf/ShaderModule.h>
#include <glslang/SPIRV/GlslangToSpv.h>
#include <map>

namespace vkhlf
{

  ShaderModule::ShaderModule(std::shared_ptr<Device> const & device, vk::ArrayProxy<const uint32_t> code, std::shared_ptr<Allocator> const& allocator)
    : Reference(device, allocator)
  {
    vk::ShaderModuleCreateInfo createInfo(vk::ShaderModuleCreateFlags(), 4 * code.size(), code.data());
    m_shaderModule = static_cast<vk::Device>(*get<Device>()).createShaderModule(createInfo, *get<Allocator>());
  }

  ShaderModule::~ShaderModule()
  {
    static_cast<vk::Device>(*get<Device>()).destroyShaderModule(m_shaderModule, *get<Allocator>());
  }


  class GLSLToSPIRVCompiler
  {
  public:
    VKHLF_API GLSLToSPIRVCompiler();
    VKHLF_API ~GLSLToSPIRVCompiler();

    VKHLF_API std::vector<uint32_t> compile(vk::ShaderStageFlagBits stage, std::string const & source) const;

  private:
    TBuiltInResource  m_resource;
  };

  GLSLToSPIRVCompiler::GLSLToSPIRVCompiler()
  {
#ifndef __ANDROID__
    glslang::InitializeProcess();
#endif

    m_resource.maxLights = 32;
    m_resource.maxClipPlanes = 6;
    m_resource.maxTextureUnits = 32;
    m_resource.maxTextureCoords = 32;
    m_resource.maxVertexAttribs = 64;
    m_resource.maxVertexUniformComponents = 4096;
    m_resource.maxVaryingFloats = 64;
    m_resource.maxVertexTextureImageUnits = 32;
    m_resource.maxCombinedTextureImageUnits = 80;
    m_resource.maxTextureImageUnits = 32;
    m_resource.maxFragmentUniformComponents = 4096;
    m_resource.maxDrawBuffers = 32;
    m_resource.maxVertexUniformVectors = 128;
    m_resource.maxVaryingVectors = 8;
    m_resource.maxFragmentUniformVectors = 16;
    m_resource.maxVertexOutputVectors = 16;
    m_resource.maxFragmentInputVectors = 15;
    m_resource.minProgramTexelOffset = -8;
    m_resource.maxProgramTexelOffset = 7;
    m_resource.maxClipDistances = 8;
    m_resource.maxComputeWorkGroupCountX = 65535;
    m_resource.maxComputeWorkGroupCountY = 65535;
    m_resource.maxComputeWorkGroupCountZ = 65535;
    m_resource.maxComputeWorkGroupSizeX = 1024;
    m_resource.maxComputeWorkGroupSizeY = 1024;
    m_resource.maxComputeWorkGroupSizeZ = 64;
    m_resource.maxComputeUniformComponents = 1024;
    m_resource.maxComputeTextureImageUnits = 16;
    m_resource.maxComputeImageUniforms = 8;
    m_resource.maxComputeAtomicCounters = 8;
    m_resource.maxComputeAtomicCounterBuffers = 1;
    m_resource.maxVaryingComponents = 60;
    m_resource.maxVertexOutputComponents = 64;
    m_resource.maxGeometryInputComponents = 64;
    m_resource.maxGeometryOutputComponents = 128;
    m_resource.maxFragmentInputComponents = 128;
    m_resource.maxImageUnits = 8;
    m_resource.maxCombinedImageUnitsAndFragmentOutputs = 8;
    m_resource.maxCombinedShaderOutputResources = 8;
    m_resource.maxImageSamples = 0;
    m_resource.maxVertexImageUniforms = 0;
    m_resource.maxTessControlImageUniforms = 0;
    m_resource.maxTessEvaluationImageUniforms = 0;
    m_resource.maxGeometryImageUniforms = 0;
    m_resource.maxFragmentImageUniforms = 8;
    m_resource.maxCombinedImageUniforms = 8;
    m_resource.maxGeometryTextureImageUnits = 16;
    m_resource.maxGeometryOutputVertices = 256;
    m_resource.maxGeometryTotalOutputComponents = 1024;
    m_resource.maxGeometryUniformComponents = 1024;
    m_resource.maxGeometryVaryingComponents = 64;
    m_resource.maxTessControlInputComponents = 128;
    m_resource.maxTessControlOutputComponents = 128;
    m_resource.maxTessControlTextureImageUnits = 16;
    m_resource.maxTessControlUniformComponents = 1024;
    m_resource.maxTessControlTotalOutputComponents = 4096;
    m_resource.maxTessEvaluationInputComponents = 128;
    m_resource.maxTessEvaluationOutputComponents = 128;
    m_resource.maxTessEvaluationTextureImageUnits = 16;
    m_resource.maxTessEvaluationUniformComponents = 1024;
    m_resource.maxTessPatchComponents = 120;
    m_resource.maxPatchVertices = 32;
    m_resource.maxTessGenLevel = 64;
    m_resource.maxViewports = 16;
    m_resource.maxVertexAtomicCounters = 0;
    m_resource.maxTessControlAtomicCounters = 0;
    m_resource.maxTessEvaluationAtomicCounters = 0;
    m_resource.maxGeometryAtomicCounters = 0;
    m_resource.maxFragmentAtomicCounters = 8;
    m_resource.maxCombinedAtomicCounters = 8;
    m_resource.maxAtomicCounterBindings = 1;
    m_resource.maxVertexAtomicCounterBuffers = 0;
    m_resource.maxTessControlAtomicCounterBuffers = 0;
    m_resource.maxTessEvaluationAtomicCounterBuffers = 0;
    m_resource.maxGeometryAtomicCounterBuffers = 0;
    m_resource.maxFragmentAtomicCounterBuffers = 1;
    m_resource.maxCombinedAtomicCounterBuffers = 1;
    m_resource.maxAtomicCounterBufferSize = 16384;
    m_resource.maxTransformFeedbackBuffers = 4;
    m_resource.maxTransformFeedbackInterleavedComponents = 64;
    m_resource.maxCullDistances = 8;
    m_resource.maxCombinedClipAndCullDistances = 8;
    m_resource.maxSamples = 4;
    m_resource.limits.nonInductiveForLoops = 1;
    m_resource.limits.whileLoops = 1;
    m_resource.limits.doWhileLoops = 1;
    m_resource.limits.generalUniformIndexing = 1;
    m_resource.limits.generalAttributeMatrixVectorIndexing = 1;
    m_resource.limits.generalVaryingIndexing = 1;
    m_resource.limits.generalSamplerIndexing = 1;
    m_resource.limits.generalVariableIndexing = 1;
    m_resource.limits.generalConstantMatrixVectorIndexing = 1;
  }

  GLSLToSPIRVCompiler::~GLSLToSPIRVCompiler()
  {
#ifndef __ANDROID__
    glslang::FinalizeProcess();
#endif
  }

  std::vector<uint32_t> GLSLToSPIRVCompiler::compile(vk::ShaderStageFlagBits stage, std::string const & source) const
  {
    static const std::map<vk::ShaderStageFlagBits, EShLanguage> stageToLanguageMap
    {
      {vk::ShaderStageFlagBits::eVertex, EShLangVertex},
      {vk::ShaderStageFlagBits::eTessellationControl, EShLangTessControl},
      {vk::ShaderStageFlagBits::eTessellationEvaluation, EShLangTessEvaluation},
      {vk::ShaderStageFlagBits::eGeometry, EShLangGeometry},
      {vk::ShaderStageFlagBits::eFragment, EShLangFragment},
      {vk::ShaderStageFlagBits::eCompute, EShLangCompute}
    };

    std::map<vk::ShaderStageFlagBits, EShLanguage>::const_iterator stageIt = stageToLanguageMap.find(stage);
    assert( stageIt != stageToLanguageMap.end());
    glslang::TShader shader(stageIt->second);

    const char *shaderStrings[1];
    shaderStrings[0] = source.c_str();
    shader.setStrings(shaderStrings, 1);

    // Enable SPIR-V and Vulkan rules when parsing GLSL
    EShMessages messages = (EShMessages)(EShMsgSpvRules | EShMsgVulkanRules);

    if (!shader.parse(&m_resource, 100, false, messages))
    {
      std::string infoLog = shader.getInfoLog();
      std::string infoDebugLog = shader.getInfoDebugLog();
      assert(false);
    }

    glslang::TProgram program;
    program.addShader(&shader);

    if (!program.link(messages))
    {
      std::string infoLog = program.getInfoLog();
      std::string infoDebugLog = program.getInfoDebugLog();
      assert(false);
    }

    std::vector<uint32_t> code;
    glslang::GlslangToSpv(*program.getIntermediate(stageIt->second), code);

    return code;
  }

  std::vector<uint32_t> compileGLSLToSPIRV(vk::ShaderStageFlagBits stage, std::string const & source)
  {
    static GLSLToSPIRVCompiler compiler;
    return compiler.compile(stage, source);
  }

} // namespace vkhlf
