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

#include <vulkan/vulkan.hpp>

#include <iostream>
#include <vkhlf/Instance.h>
#include <vkhlf/PhysicalDevice.h>

int main(int argc, char *argv[])
{
  try
  {
    std::vector<vk::LayerProperties> layerProperties = vk::enumerateInstanceLayerProperties();
    std::cout << "Layers : " << layerProperties.size() << std::endl;
    for (size_t i = 0; i < layerProperties.size(); i++)
    {
      std::cout << "Layer " << i << ":" << std::endl;
      std::cout << "\tlayerName             : " << layerProperties[i].layerName << std::endl;
      std::cout << "\tspecVersion           : " << layerProperties[i].specVersion << std::endl;
      std::cout << "\timplementationVersion : " << layerProperties[i].implementationVersion << std::endl;
      std::cout << "\tdescription           : " << layerProperties[i].description << std::endl;
    }

    std::vector<vk::ExtensionProperties> instanceExtensions = vk::enumerateInstanceExtensionProperties();
    std::cout << "InstanceExtensions : " << instanceExtensions.size() << std::endl;
    for (size_t i = 0; i < instanceExtensions.size(); i++)
    {
      std::cout << "\t" << instanceExtensions[i].extensionName << " (Version " << instanceExtensions[i].specVersion << ")" << std::endl;
    }

    std::shared_ptr<vkhlf::Instance> instance = vkhlf::Instance::create("GLUTInfo", 0);

#if !defined(NDEBUG)
    vk::DebugReportFlagsEXT flags(vk::DebugReportFlagBitsEXT::eWarning | vk::DebugReportFlagBitsEXT::ePerformanceWarning | vk::DebugReportFlagBitsEXT::eError | vk::DebugReportFlagBitsEXT::eDebug);
    std::shared_ptr<vkhlf::DebugReportCallback> debugReportCallback = instance->createDebugReportCallback(flags, &vkhlf::debugReportCallback);
#endif

    size_t pdCount = instance->getPhysicalDeviceCount();
    std::cout << "PhysicalDeviceCount : " << pdCount << std::endl;
    for (size_t i = 0; i < pdCount; i++)
    {
      std::shared_ptr<vkhlf::PhysicalDevice> pd = instance->getPhysicalDevice(i);
      vk::PhysicalDeviceProperties pdProperties = pd->getProperties();

      std::cout << "Device " << i << ":" << std::endl;
      std::cout << "\tAPI Version    : " << pdProperties.apiVersion << std::endl;
      std::cout << "\tDriver Version : " << pdProperties.driverVersion << std::endl;
      std::cout << "\tVendor ID      : " << pdProperties.vendorID << std::endl;
      std::cout << "\tDevice ID      : " << pdProperties.deviceID << std::endl;
      std::cout << "\tDevice Type    : " << vk::to_string(pdProperties.deviceType) << std::endl;
      std::cout << "\tDevice Name    : " << pdProperties.deviceName << std::endl;
      std::cout << "\tUUID           : ";
      for (int i = 0; i < VK_UUID_SIZE; i++)
      {
        std::cout << pdProperties.pipelineCacheUUID[i] << " ";
      }
      std::cout << std::endl;
      std::cout << "\tlimits: " << std::endl;
      std::cout << "\t\tMax Image Dimension 1D                                : " << pdProperties.limits.maxImageDimension1D << std::endl;
      std::cout << "\t\tMax Image Dimension 2D                                : " << pdProperties.limits.maxImageDimension2D << std::endl;
      std::cout << "\t\tMax Image Dimension 3D                                : " << pdProperties.limits.maxImageDimension3D << std::endl;
      std::cout << "\t\tMax Image Dimension Cube                              : " << pdProperties.limits.maxImageDimensionCube << std::endl;
      std::cout << "\t\tMax Image Array Layers                                : " << pdProperties.limits.maxImageArrayLayers << std::endl;
      std::cout << "\t\tMax Texel Buffer Elements                             : " << pdProperties.limits.maxTexelBufferElements << std::endl;
      std::cout << "\t\tMax Uniform Buffer Range                              : " << pdProperties.limits.maxUniformBufferRange << std::endl;
      std::cout << "\t\tMax Storage Buffer Range                              : " << pdProperties.limits.maxStorageBufferRange << std::endl;
      std::cout << "\t\tMax Push Constant Size                                : " << pdProperties.limits.maxPushConstantsSize << std::endl;
      std::cout << "\t\tMax Memory Allocation Count                           : " << pdProperties.limits.maxMemoryAllocationCount << std::endl;
      std::cout << "\t\tMax Sampler Allocation Count                          : " << pdProperties.limits.maxSamplerAllocationCount << std::endl;
      std::cout << "\t\tBuffer Image Granularity                              : " << pdProperties.limits.bufferImageGranularity << std::endl;
      std::cout << "\t\tSparse Address Space Size                             : " << pdProperties.limits.sparseAddressSpaceSize << std::endl;
      std::cout << "\t\tMax Bound Descriptor Sets                             : " << pdProperties.limits.maxBoundDescriptorSets << std::endl;
      std::cout << "\t\tMax Per Stage Descriptor Samplers                     : " << pdProperties.limits.maxPerStageDescriptorSamplers << std::endl;
      std::cout << "\t\tMax Per Stage Descriptor Uniform Buffers              : " << pdProperties.limits.maxPerStageDescriptorUniformBuffers << std::endl;
      std::cout << "\t\tMax Per Stage Descriptor Storage Buffers              : " << pdProperties.limits.maxPerStageDescriptorStorageBuffers << std::endl;
      std::cout << "\t\tMax Per Stage Descriptor Sampled Images               : " << pdProperties.limits.maxPerStageDescriptorSampledImages << std::endl;
      std::cout << "\t\tMax Per Stage Descriptor Storage Images               : " << pdProperties.limits.maxPerStageDescriptorStorageImages << std::endl;
      std::cout << "\t\tMax Per Stage Descriptor Input Attachments            : " << pdProperties.limits.maxPerStageDescriptorInputAttachments << std::endl;
      std::cout << "\t\tMax Per Stage Resources                               : " << pdProperties.limits.maxPerStageResources << std::endl;
      std::cout << "\t\tMax DescriptorSet Samplers                            : " << pdProperties.limits.maxDescriptorSetSamplers << std::endl;
      std::cout << "\t\tMax DescriptorSet Uniform Buffers                     : " << pdProperties.limits.maxDescriptorSetUniformBuffers << std::endl;
      std::cout << "\t\tMax DescriptorSet Uniform Buffers Dynamic             : " << pdProperties.limits.maxDescriptorSetUniformBuffersDynamic << std::endl;
      std::cout << "\t\tMax DescriptorSet Storage Buffers                     : " << pdProperties.limits.maxDescriptorSetStorageBuffers << std::endl;
      std::cout << "\t\tMax DescriptorSet Storage Buffers Dynamic             : " << pdProperties.limits.maxDescriptorSetStorageBuffersDynamic << std::endl;
      std::cout << "\t\tMax DescriptorSet Sampled Images                      : " << pdProperties.limits.maxDescriptorSetSampledImages << std::endl;
      std::cout << "\t\tMax DescriptorSet Storage Images                      : " << pdProperties.limits.maxDescriptorSetStorageImages << std::endl;
      std::cout << "\t\tMax DescriptorSet Input Attachments                   : " << pdProperties.limits.maxDescriptorSetInputAttachments << std::endl;
      std::cout << "\t\tMax Vertex Input Attributes                           : " << pdProperties.limits.maxVertexInputAttributes << std::endl;
      std::cout << "\t\tMax Vertex Input Bindings                             : " << pdProperties.limits.maxVertexInputBindings << std::endl;
      std::cout << "\t\tMax Vertex Input Attribute Offset                     : " << pdProperties.limits.maxVertexInputAttributeOffset << std::endl;
      std::cout << "\t\tMax Vertex Input Binding Stride                       : " << pdProperties.limits.maxVertexInputBindingStride << std::endl;
      std::cout << "\t\tMax Vertex Output Components                          : " << pdProperties.limits.maxVertexOutputComponents << std::endl;
      std::cout << "\t\tMax Tessellation Generation Level                     : " << pdProperties.limits.maxTessellationGenerationLevel << std::endl;
      std::cout << "\t\tMax Tessellation Patch Size                           : " << pdProperties.limits.maxTessellationPatchSize << std::endl;
      std::cout << "\t\tMax Tessellation Control Per Vertex Input Components  : " << pdProperties.limits.maxTessellationControlPerVertexInputComponents << std::endl;
      std::cout << "\t\tMax Tessellation Control Per Vertex Output Components : " << pdProperties.limits.maxTessellationControlPerVertexOutputComponents << std::endl;
      std::cout << "\t\tMax Tessellation Control Per Vertex Output Components : " << pdProperties.limits.maxTessellationControlPerVertexOutputComponents << std::endl;
      std::cout << "\t\tMax Tessellation Control Per Patch Output Components  : " << pdProperties.limits.maxTessellationControlPerPatchOutputComponents << std::endl;
      std::cout << "\t\tMax Tessellation Control Total Output Components      : " << pdProperties.limits.maxTessellationControlTotalOutputComponents << std::endl;
      std::cout << "\t\tMax Tessellation Evaluation Input Components          : " << pdProperties.limits.maxTessellationEvaluationInputComponents << std::endl;
      std::cout << "\t\tMax Tessellation Evaluation Output Components         : " << pdProperties.limits.maxTessellationEvaluationOutputComponents << std::endl;
      std::cout << "\t\tMax Geometry Shader Invocations                       : " << pdProperties.limits.maxGeometryShaderInvocations << std::endl;
      std::cout << "\t\tMax Geometry Input Components                         : " << pdProperties.limits.maxGeometryInputComponents << std::endl;
      std::cout << "\t\tMax Geometry Output Components                        : " << pdProperties.limits.maxGeometryOutputComponents << std::endl;
      std::cout << "\t\tMax Geometry Output Vertices                          : " << pdProperties.limits.maxGeometryOutputVertices << std::endl;
      std::cout << "\t\tMax Geometry Total Output Components                  : " << pdProperties.limits.maxGeometryTotalOutputComponents << std::endl;
      std::cout << "\t\tMax Fragment Input Components                         : " << pdProperties.limits.maxFragmentInputComponents << std::endl;
      std::cout << "\t\tMax Fragment Output Components                        : " << pdProperties.limits.maxFragmentOutputAttachments << std::endl;
      std::cout << "\t\tMax Fragment Dual Source Attachments                  : " << pdProperties.limits.maxFragmentDualSrcAttachments << std::endl;
      std::cout << "\t\tMax Fragment Combined Output Resources                : " << pdProperties.limits.maxFragmentCombinedOutputResources << std::endl;
      std::cout << "\t\tMax Compute Shared Memory Size                        : " << pdProperties.limits.maxComputeSharedMemorySize << std::endl;
      std::cout << "\t\tMax Compute Work Group Count                          : " << pdProperties.limits.maxComputeWorkGroupCount[0] << ", " << pdProperties.limits.maxComputeWorkGroupCount[1] << ", " << pdProperties.limits.maxComputeWorkGroupCount[2] << std::endl;
      std::cout << "\t\tMax Compute Work Group Invocations                    : " << pdProperties.limits.maxComputeWorkGroupInvocations << std::endl;
      std::cout << "\t\tMax Compute Work Group Size                           : " << pdProperties.limits.maxComputeWorkGroupSize[0] << ", " << pdProperties.limits.maxComputeWorkGroupSize[1] << ", " << pdProperties.limits.maxComputeWorkGroupSize[2] << std::endl;
      std::cout << "\t\tSub Pixel Precision Bits                              : " << pdProperties.limits.subPixelPrecisionBits << std::endl;
      std::cout << "\t\tSub Texel Precision Bits                              : " << pdProperties.limits.subTexelPrecisionBits << std::endl;
      std::cout << "\t\tMipmap Precision Bits                                 : " << pdProperties.limits.mipmapPrecisionBits << std::endl;
      std::cout << "\t\tMax DrawIndexed Index Value                           : " << pdProperties.limits.maxDrawIndexedIndexValue << std::endl;
      std::cout << "\t\tMax DrawIndirect Count                                : " << pdProperties.limits.maxDrawIndirectCount << std::endl;
      std::cout << "\t\tMax Sampler LOD Bias                                  : " << pdProperties.limits.maxSamplerLodBias << std::endl;
      std::cout << "\t\tMax Sampler Anisotropy                                : " << pdProperties.limits.maxSamplerAnisotropy << std::endl;
      std::cout << "\t\tMax Viewports                                         : " << pdProperties.limits.maxViewports << std::endl;
      std::cout << "\t\tMax Viewport Dimensions                               : " << pdProperties.limits.maxViewportDimensions[0] << ", " << pdProperties.limits.maxViewportDimensions[1] << std::endl;
      std::cout << "\t\tMax Bounds Range                                      : " << pdProperties.limits.viewportBoundsRange[0] << ", " << pdProperties.limits.viewportBoundsRange[1] << std::endl;
      std::cout << "\t\tViewport SubPixel Bits                                : " << pdProperties.limits.viewportSubPixelBits << std::endl;
      std::cout << "\t\tMin Memory Map Alignment                              : " << pdProperties.limits.minMemoryMapAlignment << std::endl;
      std::cout << "\t\tMin Texel Buffer Offset Alignment                     : " << pdProperties.limits.minTexelBufferOffsetAlignment << std::endl;
      std::cout << "\t\tMin Uniform Buffer Offset Alignment                   : " << pdProperties.limits.minUniformBufferOffsetAlignment << std::endl;
      std::cout << "\t\tMin Storage Buffer Offset Alignment                   : " << pdProperties.limits.minStorageBufferOffsetAlignment << std::endl;
      std::cout << "\t\tMin Texel Offset                                      : " << pdProperties.limits.minTexelOffset << std::endl;
      std::cout << "\t\tMax Texel Offset                                      : " << pdProperties.limits.maxTexelOffset << std::endl;
      std::cout << "\t\tMin Texel Gather Offset                               : " << pdProperties.limits.minTexelGatherOffset << std::endl;
      std::cout << "\t\tMax Texel Gather Offset                               : " << pdProperties.limits.maxTexelGatherOffset << std::endl;
      std::cout << "\t\tMin Interpolation Offset                              : " << pdProperties.limits.minInterpolationOffset << std::endl;
      std::cout << "\t\tMax Interpolation Offset                              : " << pdProperties.limits.maxInterpolationOffset << std::endl;
      std::cout << "\t\tSubPixel Interpolation Offset Bits                    : " << pdProperties.limits.subPixelInterpolationOffsetBits << std::endl;
      std::cout << "\t\tMax Framebuffer Width                                 : " << pdProperties.limits.maxFramebufferWidth << std::endl;
      std::cout << "\t\tMax Framebuffer Height                                : " << pdProperties.limits.maxFramebufferHeight << std::endl;
      std::cout << "\t\tMax Framebuffer Layers                                : " << pdProperties.limits.maxFramebufferLayers << std::endl;
      std::cout << "\t\tFramebuffer Color Sample Counts                       : " << to_string(pdProperties.limits.framebufferColorSampleCounts) << std::endl;
      std::cout << "\t\tFramebuffer Depth Sample Counts                       : " << to_string(pdProperties.limits.framebufferDepthSampleCounts) << std::endl;
      std::cout << "\t\tFramebuffer Stencil Sample Counts                     : " << to_string(pdProperties.limits.framebufferStencilSampleCounts) << std::endl;
      std::cout << "\t\tFramebuffer NoAttachments Sample Counts               : " << to_string(pdProperties.limits.framebufferNoAttachmentsSampleCounts) << std::endl;
      std::cout << "\t\tMax Color Attachments                                 : " << pdProperties.limits.maxColorAttachments << std::endl;
      std::cout << "\t\tSampled Image Color Sample Counts                     : " << to_string(pdProperties.limits.sampledImageColorSampleCounts) << std::endl;
      std::cout << "\t\tSampled Image Integer Sample Counts                   : " << to_string(pdProperties.limits.sampledImageIntegerSampleCounts) << std::endl;
      std::cout << "\t\tSampled Image Depth Sample Counts                     : " << to_string(pdProperties.limits.sampledImageDepthSampleCounts) << std::endl;
      std::cout << "\t\tSampled Image Stencil Sample Counts                   : " << to_string(pdProperties.limits.sampledImageStencilSampleCounts) << std::endl;
      std::cout << "\t\tStorage Image Sample Counts                           : " << to_string(pdProperties.limits.storageImageSampleCounts) << std::endl;
      std::cout << "\t\tMax Sample Mask Words                                 : " << pdProperties.limits.maxSampleMaskWords << std::endl;
      std::cout << "\t\tTimestamp Period                                      : " << pdProperties.limits.timestampPeriod << std::endl;
      std::cout << "\t\tMax Clip Distances                                    : " << pdProperties.limits.maxClipDistances << std::endl;
      std::cout << "\t\tMax Cull Distances                                    : " << pdProperties.limits.maxCullDistances << std::endl;
      std::cout << "\t\tMax Combined Clip And Cull Distances                  : " << pdProperties.limits.maxCombinedClipAndCullDistances << std::endl;
      std::cout << "\t\tDiscrete Queue Priorities                             : " << pdProperties.limits.discreteQueuePriorities << std::endl;
      std::cout << "\t\tPoint Size Range                                      : " << pdProperties.limits.pointSizeRange[0] << ", " << pdProperties.limits.pointSizeRange[1] << std::endl;
      std::cout << "\t\tLine Width Range                                      : " << pdProperties.limits.lineWidthRange[0] << ", " << pdProperties.limits.lineWidthRange[1] << std::endl;
      std::cout << "\t\tPoint Size Granularity                                : " << pdProperties.limits.pointSizeGranularity << std::endl;
      std::cout << "\t\tLine Width Granularity                                : " << pdProperties.limits.lineWidthGranularity << std::endl;
      std::cout << "\t\tStrict Lines                                          : " << (pdProperties.limits.strictLines ? "TRUE" : "FALSE") << std::endl;
      std::cout << "\t\tStandard Sample Locations                             : " << (pdProperties.limits.standardSampleLocations ? "TRUE" : "FALSE") << std::endl;
      std::cout << "\t\tOptimal Buffer Copy Offset Alignment                  : " << pdProperties.limits.optimalBufferCopyOffsetAlignment << std::endl;
      std::cout << "\t\tOptimal Buffer Copy Rpw Pitch Alignment               : " << pdProperties.limits.optimalBufferCopyRowPitchAlignment << std::endl;
      std::cout << "\t\tNon Coherent Atom Size                                : " << pdProperties.limits.nonCoherentAtomSize << std::endl;
      std::cout << "\tSpares Properties:" << std::endl;
      std::cout << "\t\tResidency Standard 2D Block Shape             : " << (pdProperties.sparseProperties.residencyStandard2DBlockShape ? "TRUE" : "FALSE") << std::endl;
      std::cout << "\t\tResidency Standard 2D Multisample Block Shape : " << (pdProperties.sparseProperties.residencyStandard2DMultisampleBlockShape ? "TRUE" : "FALSE") << std::endl;
      std::cout << "\t\tResidency Standard 3D Block Shape             : " << (pdProperties.sparseProperties.residencyStandard3DBlockShape ? "TRUE" : "FALSE") << std::endl;
      std::cout << "\t\tResidency Aligned Mip Size                    : " << (pdProperties.sparseProperties.residencyAlignedMipSize ? "TRUE" : "FALSE") << std::endl;
      std::cout << "\t\tResidency NonResident Strict                  : " << (pdProperties.sparseProperties.residencyNonResidentStrict ? "TRUE" : "FALSE") << std::endl;

      std::vector<vk::QueueFamilyProperties> qfProperties = pd->getQueueFamilyProperties();
      std::cout << "\tQueue Family Property Count : " << qfProperties.size() << std::endl;
      for (size_t j = 0; j < qfProperties.size(); j++)
      {
        std::cout << "\t\tFamily " << j << ":" << std::endl;
        std::cout << "\t\t\tQueue Flags                    : " << vk::to_string(qfProperties[j].queueFlags) << std::endl;
        std::cout << "\t\t\tQueue Count                    : " << qfProperties[j].queueCount << std::endl;
        std::cout << "\t\t\tTimestampe Valid Bits          : " << qfProperties[j].timestampValidBits << std::endl;
        std::cout << "\t\t\tMin Image Transfer Granularity : " << qfProperties[j].minImageTransferGranularity.width << ", " << qfProperties[j].minImageTransferGranularity.height << ", " << qfProperties[j].minImageTransferGranularity.depth << std::endl;
      }

      std::vector<vk::ExtensionProperties> deviceExtensions = pd->getExtensionProperties(std::string());
      std::cout << "\tDeviceExtensions : " << deviceExtensions.size() << std::endl;
      for (size_t i = 0; i < deviceExtensions.size(); i++)
      {
        std::cout << "\t\t" << deviceExtensions[i].extensionName << " (Version " << deviceExtensions[i].specVersion << ")" << std::endl;
      }
    }
  }
  catch (std::system_error systemError)
  {
    std::cout << "System Error: " << systemError.what() << std::endl;
  }
}
