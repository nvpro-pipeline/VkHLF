# Copyright (c) 2016-2017, NVIDIA CORPORATION. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of NVIDIA CORPORATION nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
# OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 
 cmake_minimum_required(VERSION 2.8)

#includes
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

#definitions
add_definitions(
  -DNOMINMAX
  -DVKHLF_EXPORTS
)

#sources
set(SOURCES
  src/Buffer.cpp
  src/BufferView.cpp
  src/CommandBuffer.cpp
  src/CommandPool.cpp
  src/DebugReportCallback.cpp
  src/DescriptorPool.cpp
  src/DescriptorSet.cpp
  src/DescriptorSetLayout.cpp
  src/Device.cpp
  src/DeviceMemory.cpp
  src/DeviceMemoryAllocator.cpp
  src/DeviceMemoryChunk.cpp
  src/Display.cpp
  src/DisplayMode.cpp
  src/Event.cpp
  src/Fence.cpp
  src/Framebuffer.cpp
  src/FramebufferSwapchain.cpp
  src/Image.cpp
  src/ImageView.cpp
  src/Instance.cpp
  src/Pipeline.cpp
  src/PipelineCache.cpp
  src/PipelineLayout.cpp
  src/PhysicalDevice.cpp
  src/QueryPool.cpp
  src/Queue.cpp
  src/RenderPass.cpp
  src/ResourceTracker.cpp
  src/ResourceTrackerAll.cpp
  src/Sampler.cpp
  src/Semaphore.cpp
  src/ShaderModule.cpp
  src/Swapchain.cpp
  src/Surface.cpp
)

set(PUBLIC_HEADERS
  Allocator.h
  Buffer.h
  BufferView.h
  CommandBuffer.h
  CommandPool.h
  Config.h
  DebugReportCallback.h
  DescriptorPool.h
  DescriptorSet.h
  DescriptorSetLayout.h
  Device.h
  DeviceMemory.h
  DeviceMemoryAllocator.h
  DeviceMemoryChunk.h
  Display.h
  DisplayMode.h
  Event.h
  Fence.h
  Framebuffer.h
  FramebufferSwapchain.h
  Image.h
  ImageView.h
  Instance.h
  Pipeline.h
  PipelineCache.h
  PipelineLayout.h
  PhysicalDevice.h
  QueryPool.h
  Queue.h
  Reference.h
  RenderPass.h
  ResourceTracker.h
  ResourceTrackerAll.h
  Sampler.h
  Semaphore.h
  ShaderModule.h
  Swapchain.h
  Surface.h
  Types.h
)

set(PRIVATE
)

source_group(sources FILES ${SOURCES})
source_group(headers FILES ${PUBLIC_HEADERS})
source_group(private FILES ${PRIVATE})

#target
add_library( VkHLF SHARED
  ${SOURCES}
  ${PUBLIC_HEADERS}
  ${PRIVATE}
)

target_include_directories(VkHLF
  PUBLIC    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/private>
  PUBLIC    "$ENV{VK_SDK_PATH}/include"
  PUBLIC    "${CMAKE_SOURCE_DIR}/3rdparty"
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/private>
  INTERFACE "$ENV{VK_SDK_PATH}/include"
  INTERFACE  "${PROJECT_SOURCE_DIR}"
)


target_link_libraries( VkHLF
  ${OPENGL_gl_LIBRARY}
  vulkan
  glslang
  OGLCompiler
  OSDependent
  HLSL
  SPIRV
)
