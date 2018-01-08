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

#define VK_USE_PLATFORM_WIN32_KHR
#include <Windows.h>
#if defined(MemoryBarrier)
# undef MemoryBarrier
#endif

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <assert.h>
#include <iostream>
#include <memory>
#include <system_error>
#include <vector>

#include <VkHLFSampleWindow.h>

#define FENCE_TIMEOUT 100000000

class Window : public VkHLFSampleWindow
{
public:
  Window(char const* title, int width, int height);
  ~Window();

protected:
  virtual void doPaint() override;

private:
  std::shared_ptr<vkhlf::DeviceMemoryAllocator> m_deviceMemoryAllocatorBuffer;
  std::shared_ptr<vkhlf::DeviceMemoryAllocator> m_deviceMemoryAllocatorImage;
  std::shared_ptr<vkhlf::Pipeline>              m_pipeline;
  std::shared_ptr<vkhlf::PipelineLayout>        m_pipelineLayout;
  std::shared_ptr<vkhlf::ImageView>             m_textureImageView;
  std::shared_ptr<vkhlf::Sampler>               m_textureSampler;
  std::shared_ptr<vkhlf::Buffer>                m_uniformBufferMVP;
  std::shared_ptr<vkhlf::Buffer>                m_vertexBuffer;
};

// shaders
static char const *vertShaderText =
R"(#version 430
layout(location = 0) in vec2 inVertex;
layout(location = 1) in vec4 inColor;
layout(location = 0) out vec4 outColor;
out gl_PerVertex
{
  vec4 gl_Position;
};
void main()
{
  outColor = inColor;
  gl_Position = vec4(inVertex, 0, 1);
}
)";

static char const *fragShaderText = 
R"(#version 430
layout(location = 0) in vec4 inColor;
layout(location = 0) out vec4 outColor;
void main()
{
  outColor = inColor;
}
)";

// geometry
struct Vertex
{
  float   position[2];
  uint8_t color[4];
};

const std::vector<Vertex> vertices =
{
  { { -0.5f, -0.5f },{ 0xFF, 0x00, 0x00, 0xFF }, },
  { { 0.5f, 0.0f },{ 0x00, 0xFF, 0x00, 0xFF }, },
  { { 0.0f, 0.5f },{ 0x00, 0x00, 0xFF, 0xFF }, },
};

Window::Window(char const* title, int width, int height)
  : VkHLFSampleWindow(title, width, height)
{
  // create a command pool for command buffer allocation
  std::shared_ptr<vkhlf::CommandPool> commandPool = getDevice()->createCommandPool(vk::CommandPoolCreateFlagBits::eResetCommandBuffer, getQueueFamilyIndex());

  // VkHLF has mostly transparent support for suballocators. Create two device memory heaps, one with a chunk size of 64kb and another one with a chunk size of 128kb
  m_deviceMemoryAllocatorBuffer.reset(new vkhlf::DeviceMemoryAllocator(getDevice(), 64 * 1024, nullptr));
  m_deviceMemoryAllocatorImage.reset(new vkhlf::DeviceMemoryAllocator(getDevice(), 128 * 1024, nullptr));

  // init descriptor and pipeline layouts
  std::vector<vkhlf::DescriptorSetLayoutBinding> dslbs;
  std::shared_ptr<vkhlf::DescriptorSetLayout> descriptorSetLayout = getDevice()->createDescriptorSetLayout(dslbs);
  m_pipelineLayout = getDevice()->createPipelineLayout(descriptorSetLayout, nullptr);

  // init vertex buffer

  std::shared_ptr<vkhlf::CommandBuffer> commandBuffer = commandPool->allocateCommandBuffer();
  commandBuffer->begin();

  m_vertexBuffer = getDevice()->createBuffer(vertices.size() * sizeof(Vertex), vk::BufferUsageFlagBits::eTransferDst | vk::BufferUsageFlagBits::eVertexBuffer, vk::SharingMode::eExclusive, nullptr,
    vk::MemoryPropertyFlagBits::eDeviceLocal, m_deviceMemoryAllocatorBuffer);
  m_vertexBuffer->update<Vertex>(0, { uint32_t(vertices.size()), vertices.data() }, commandBuffer);
  commandBuffer->end();
  vkhlf::submitAndWait(getGraphicsQueue(), commandBuffer);

  // init shaders
  std::shared_ptr<vkhlf::ShaderModule> vertexShaderModule = getDevice()->createShaderModule(vkhlf::compileGLSLToSPIRV(vk::ShaderStageFlagBits::eVertex, vertShaderText));
  std::shared_ptr<vkhlf::ShaderModule> fragmentShaderModule = getDevice()->createShaderModule(vkhlf::compileGLSLToSPIRV(vk::ShaderStageFlagBits::eFragment, fragShaderText));

  // init pipeline
  std::shared_ptr<vkhlf::PipelineCache> pipelineCache = getDevice()->createPipelineCache(0, nullptr);
  vkhlf::PipelineShaderStageCreateInfo vertexStage(vk::ShaderStageFlagBits::eVertex, vertexShaderModule, "main");
  vkhlf::PipelineShaderStageCreateInfo fragmentStage(vk::ShaderStageFlagBits::eFragment, fragmentShaderModule, "main");
  vk::VertexInputBindingDescription binding(0, sizeof(Vertex), vk::VertexInputRate::eVertex);
  vk::VertexInputAttributeDescription attrib0(0, 0, vk::Format::eR32G32B32A32Sfloat, offsetof(Vertex, position));
  vk::VertexInputAttributeDescription attrib1(1, 0, vk::Format::eR8G8B8A8Unorm, offsetof(Vertex, color));
  vkhlf::PipelineVertexInputStateCreateInfo vertexInput(binding, { attrib0, attrib1 });
  vk::PipelineInputAssemblyStateCreateInfo assembly({}, vk::PrimitiveTopology::eTriangleList, VK_FALSE);
  vkhlf::PipelineViewportStateCreateInfo viewport({ {} }, { {} });   // one dummy viewport and scissor, as dynamic state sets them
  vk::PipelineRasterizationStateCreateInfo rasterization({}, false, false, vk::PolygonMode::eFill, vk::CullModeFlagBits::eBack, vk::FrontFace::eClockwise, false, 0.0f, 0.0f, 0.0f, 1.0f);
  vkhlf::PipelineMultisampleStateCreateInfo multisample(vk::SampleCountFlagBits::e1, false, 0.0f, nullptr, false, false);
  vk::StencilOpState stencilOpState(vk::StencilOp::eKeep, vk::StencilOp::eKeep, vk::StencilOp::eKeep, vk::CompareOp::eAlways, 0, 0, 0);
  vk::PipelineDepthStencilStateCreateInfo depthStencil({}, true, true, vk::CompareOp::eLessOrEqual, false, false, stencilOpState, stencilOpState, 0.0f, 0.0f);
  vk::PipelineColorBlendAttachmentState colorBlendAttachment(false, vk::BlendFactor::eZero, vk::BlendFactor::eZero, vk::BlendOp::eAdd, vk::BlendFactor::eZero, vk::BlendFactor::eZero, vk::BlendOp::eAdd,
    vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG | vk::ColorComponentFlagBits::eB | vk::ColorComponentFlagBits::eA);
  vkhlf::PipelineColorBlendStateCreateInfo colorBlend(false, vk::LogicOp::eNoOp, colorBlendAttachment, { 1.0f, 1.0f, 1.0f, 1.0f });
  vkhlf::PipelineDynamicStateCreateInfo dynamic({ vk::DynamicState::eViewport, vk::DynamicState::eScissor });

  m_pipeline = getDevice()->createGraphicsPipeline(pipelineCache, {}, { vertexStage, fragmentStage }, vertexInput, assembly, nullptr, viewport, rasterization, multisample, depthStencil, colorBlend, dynamic,
    m_pipelineLayout, getRenderPass());
}

Window::~Window()
{
  // TODO Resource tracking for DescriptorSets has not yet been implemented. Wait for the device being idle to ensure that the CommandBuffer can be released.
  getDevice()->waitIdle();
}


void Window::doPaint()
{
  // create a command pool for command buffer allocation
  std::shared_ptr<vkhlf::CommandPool> commandPool = getDevice()->createCommandPool(vk::CommandPoolCreateFlagBits::eResetCommandBuffer, getQueueFamilyIndex());
  std::shared_ptr<vkhlf::CommandBuffer> commandBuffer = commandPool->allocateCommandBuffer();

  commandBuffer->begin();

  std::array<float, 4> ccv = { 0.462745f, 0.72549f, 0.0f };
  commandBuffer->beginRenderPass(getRenderPass(), getFramebufferSwapchain()->getFramebuffer(), vk::Rect2D({ 0, 0 }, getFramebufferSwapchain()->getExtent()),
  { vk::ClearValue(ccv), vk::ClearValue(vk::ClearDepthStencilValue(1.0f, 0)) }, vk::SubpassContents::eInline);
  commandBuffer->bindPipeline(vk::PipelineBindPoint::eGraphics, m_pipeline);
  commandBuffer->bindVertexBuffer(0, m_vertexBuffer, 0);
  commandBuffer->setViewport(0, vk::Viewport(0.0f, 0.0f, (float)getFramebufferSwapchain()->getExtent().width, (float)getFramebufferSwapchain()->getExtent().height, 0.0f, 1.0f));
  commandBuffer->setScissor(0, vk::Rect2D({ 0, 0 }, getFramebufferSwapchain()->getExtent()));
  commandBuffer->draw(uint32_t(vertices.size()), 1, 0, 0);
  commandBuffer->endRenderPass();

  commandBuffer->end();

  getGraphicsQueue()->submit(vkhlf::SubmitInfo{ { getFramebufferSwapchain()->getPresentSemaphore() },{ vk::PipelineStageFlagBits::eColorAttachmentOutput }, commandBuffer, getRenderCompleteSemaphore() });
}

GLFWwindow * initWindow(int width, int height, std::string const& title)
{
  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
  GLFWwindow* window = glfwCreateWindow(width, height, title.c_str(), NULL, NULL);
  return window;
}

void errorCallback(int error, const char* description)
{
  fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

int main(int argc, char *argv[])
{
  try
  {
    glfwSetErrorCallback(errorCallback);

    if (glfwInit())
    {
      Window window(argv[0], 640, 480);

      // This is the actual renderloop
      while (!glfwWindowShouldClose(window.getWindow()))
      {
        glfwWaitEvents();
      }

      glfwTerminate();
    }
  }
  catch (std::system_error systemError)
  {
    std::cout << "System Error: " << systemError.what() << std::endl;
  }
}
