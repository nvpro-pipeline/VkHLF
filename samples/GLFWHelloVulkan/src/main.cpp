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

#include <VkHLFSampleWindow.h>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <assert.h>
#include <iostream>
#include <memory>
#include <system_error>
#include <vector>

class Window : public VkHLFSampleWindow
{
public:
  Window(char const* title, int width, int height);
  ~Window();


protected:
  virtual void doPaint() override;
  virtual void doResize(int width, int height) override;
  virtual void cursorPosEvent(double xPos, double yPos) override;
  virtual void keyEvent(int key, int scancode, int action, int mods) override;
  virtual void mouseButtonEvent(int button, int action, int mods) override;

  void updateCameraInformation(int width, int height);

private:
  std::shared_ptr<vkhlf::DescriptorSet>         m_descriptorSet;
  std::shared_ptr<vkhlf::DeviceMemoryAllocator> m_deviceMemoryAllocatorBuffer;
  std::shared_ptr<vkhlf::DeviceMemoryAllocator> m_deviceMemoryAllocatorImage;
  std::shared_ptr<vkhlf::Pipeline>              m_pipeline;
  std::shared_ptr<vkhlf::PipelineLayout>        m_pipelineLayout;
  std::shared_ptr<vkhlf::ImageView>             m_textureImageView;
  std::shared_ptr<vkhlf::Sampler>               m_textureSampler;
  std::shared_ptr<vkhlf::Buffer>                m_uniformBufferMVP;
  std::shared_ptr<vkhlf::Buffer>                m_vertexBuffer;

  float                                       m_fovy;
  glm::vec3                                   m_eyePos;
  glm::vec3                                   m_centerPos;
  glm::vec3                                   m_upDir;
  glm::mat4                                   m_view;
  bool                                        m_tracking;
  double                                      m_trackStartX, m_trackStartY;
  glm::vec3                                   m_trackStartEyePos;
  glm::vec3                                   m_trackStartCenterPos;
  glm::vec2                                   m_trackFactor;
};

static const char *vertShaderText =
"#version 420\n"
"layout (std140, binding = 0) uniform bufferVals\n"
"{\n"
"  mat4 mvp;\n"
"} myBufferVals;\n"
"layout (location = 0) in vec4 pos;\n"
"layout (location = 1) in vec2 inTexCoord;\n"
"layout (location = 0) out vec2 outTexCoord;\n"
"out gl_PerVertex\n"
"{ \n"
"  vec4 gl_Position;\n"
"};\n"
"void main()\n"
"{\n"
"  outTexCoord = inTexCoord;\n"
"  gl_Position = myBufferVals.mvp * pos;\n"
"}\n";

static const char *fragShaderText =
"#version 420\n"
"layout (binding = 1) uniform sampler2D tex;\n"
"layout (location = 0) in vec2 texCoord;\n"
"layout (location = 0) out vec4 outColor;\n"
"void main()\n"
"{\n"
"  outColor = textureLod(tex, texCoord, 0.0);\n"
"}\n";

struct Vertex
{
  float x, y, z, w; // Position data
  float u, v;       // texture u,v
};

static const Vertex vertexData[] = {
  // Position                   Texture
  {-1.0f, -1.0f, -1.0f,  1.0f, 0.0f, 0.0f},
  {-1.0f,  1.0f,  1.0f,  1.0f, 1.0f, 1.0f},
  {-1.0f, -1.0f,  1.0f,  1.0f, 1.0f, 0.0f},
  {-1.0f,  1.0f,  1.0f,  1.0f, 1.0f, 1.0f},
  {-1.0f, -1.0f, -1.0f,  1.0f, 0.0f, 0.0f},
  {-1.0f,  1.0f, -1.0f,  1.0f, 0.0f, 1.0f},

  {-1.0f, -1.0f, -1.0f,  1.0f, 1.0f, 0.0f},
  { 1.0f, -1.0f, -1.0f,  1.0f, 0.0f, 0.0f},
  { 1.0f,  1.0f, -1.0f,  1.0f, 0.0f, 1.0f},
  {-1.0f, -1.0f, -1.0f,  1.0f, 1.0f, 0.0f},
  { 1.0f,  1.0f, -1.0f,  1.0f, 0.0f, 1.0f},
  {-1.0f,  1.0f, -1.0f,  1.0f, 1.0f, 1.0f},

  {-1.0f, -1.0f, -1.0f,  1.0f, 1.0f, 1.0f},
  { 1.0f, -1.0f,  1.0f,  1.0f, 0.0f, 0.0f},
  { 1.0f, -1.0f, -1.0f,  1.0f, 1.0f, 0.0f},
  {-1.0f, -1.0f, -1.0f,  1.0f, 1.0f, 1.0f},
  {-1.0f, -1.0f,  1.0f,  1.0f, 0.0f, 1.0f},
  { 1.0f, -1.0f,  1.0f,  1.0f, 0.0f, 0.0f},

  {-1.0f,  1.0f, -1.0f,  1.0f, 1.0f, 1.0f},
  { 1.0f,  1.0f,  1.0f,  1.0f, 0.0f, 0.0f},
  {-1.0f,  1.0f,  1.0f,  1.0f, 0.0f, 1.0f},
  {-1.0f,  1.0f, -1.0f,  1.0f, 1.0f, 1.0f},
  { 1.0f,  1.0f, -1.0f,  1.0f, 1.0f, 0.0f},
  { 1.0f,  1.0f,  1.0f,  1.0f, 0.0f, 0.0f},

  { 1.0f,  1.0f, -1.0f,  1.0f, 1.0f, 1.0f},
  { 1.0f, -1.0f,  1.0f,  1.0f, 0.0f, 0.0f},
  { 1.0f,  1.0f,  1.0f,  1.0f, 0.0f, 1.0f},
  { 1.0f, -1.0f,  1.0f,  1.0f, 0.0f, 0.0f},
  { 1.0f,  1.0f, -1.0f,  1.0f, 1.0f, 1.0f},
  { 1.0f, -1.0f, -1.0f,  1.0f, 1.0f, 0.0f},

  {-1.0f,  1.0f,  1.0f,  1.0f, 0.0f, 1.0f},
  { 1.0f,  1.0f,  1.0f,  1.0f, 1.0f, 1.0f},
  {-1.0f, -1.0f,  1.0f,  1.0f, 0.0f, 0.0f},
  {-1.0f, -1.0f,  1.0f,  1.0f, 0.0f, 0.0f},
  { 1.0f,  1.0f,  1.0f,  1.0f, 1.0f, 1.0f},
  { 1.0f, -1.0f,  1.0f,  1.0f, 1.0f, 0.0f},
};

Window::Window(char const* title, int width, int height)
  : VkHLFSampleWindow(title, width, height)
  , m_tracking(false)
  , m_eyePos(5.0f, 3.0f, 10.0f) // Camera is at (5,3,10), in World Space
  , m_centerPos(0, 0, 0)        // and looks at the origin
  , m_upDir(0, -1, 0)          // Head is up (set to 0,-1,0 to look upside-down)
  , m_view(glm::lookAt(m_eyePos, m_centerPos, m_upDir))
{

  // VkHLF has mostly transparent support for suballocators. Create two device memory heaps, one with a chunk size of 64kb and another one with a chunk size of 128kb
  m_deviceMemoryAllocatorBuffer.reset(new vkhlf::DeviceMemoryAllocator(getDevice(), 64 * 1024, nullptr));
  m_deviceMemoryAllocatorImage.reset(new vkhlf::DeviceMemoryAllocator(getDevice(), 128 * 1024, nullptr));

  // create a command pool for command buffer allocation
  std::shared_ptr<vkhlf::CommandPool> commandPool = getDevice()->createCommandPool(vk::CommandPoolCreateFlagBits::eResetCommandBuffer, getQueueFamilyIndex());
  std::shared_ptr<vkhlf::CommandBuffer> commandBuffer = commandPool->allocateCommandBuffer();

  // create a uniform buffer for the model view projection matrix
  m_uniformBufferMVP = getDevice()->createBuffer(sizeof(glm::mat4), vk::BufferUsageFlagBits::eTransferDst | vk::BufferUsageFlagBits::eUniformBuffer, vk::SharingMode::eExclusive, nullptr,
                                              vk::MemoryPropertyFlagBits::eDeviceLocal, m_deviceMemoryAllocatorBuffer);

  // create a small 8x8 RGBA8 texture with a simple color pattern
  // 1. init image
  const vk::Format format = vk::Format::eR8G8B8A8Unorm;
  vk::FormatProperties imageFormatProperties = getPhysicalDevice()->getFormatProperties(format);
  assert((imageFormatProperties.linearTilingFeatures & vk::FormatFeatureFlagBits::eSampledImage) || (imageFormatProperties.optimalTilingFeatures & vk::FormatFeatureFlagBits::eSampledImage));

  const uint32_t imageWidth = 8;
  const uint32_t imageHeight = 8;
  std::shared_ptr<vkhlf::Image> image = getDevice()->createImage({}, vk::ImageType::e2D, format, vk::Extent3D(imageWidth, imageHeight, 1), 1, 1, vk::SampleCountFlagBits::e1, vk::ImageTiling::eOptimal,
                                                            vk::ImageUsageFlagBits::eTransferDst | vk::ImageUsageFlagBits::eSampled, vk::SharingMode::eExclusive, {}, vk::ImageLayout::eUndefined,
                                                            vk::MemoryPropertyFlagBits::eDeviceLocal, m_deviceMemoryAllocatorImage);

  // TODO the position of begin() seems a little bit random.
  commandBuffer->begin();
  {
    // create a temporary upload image and fill it with pixel data. The destructor of MappedImage will put the transfer into the command buffer.
    vkhlf::MappedImage mi(image, commandBuffer, 0, 256);
    vk::SubresourceLayout layout = mi.getSubresourceLayout(vk::ImageAspectFlagBits::eColor, 0, 0);
    uint8_t * data = reinterpret_cast<uint8_t*>(mi.getPointer());
    for (size_t y = 0; y < imageHeight; y++)
    {
      uint8_t * rowPtr = data;
      for (size_t x = 0; x < imageWidth; x++)
      {
        rowPtr[0] = ((x ^ y) & 1) ? 255 : 0;
        rowPtr[1] = ((x ^ y) & 2) ? 255 : 0;
        rowPtr[3] = ((x ^ y) & 4) ? 255 : 0;
        rowPtr[4] = 255;  // alpha == 1
        rowPtr += 4;
      }
      data += layout.rowPitch;
    }
  }

  m_textureImageView = image->createImageView(vk::ImageViewType::e2D, format);

  // 2. init sampler
  m_textureSampler = getDevice()->createSampler(vk::Filter::eNearest, vk::Filter::eNearest, vk::SamplerMipmapMode::eNearest, vk::SamplerAddressMode::eClampToEdge, vk::SamplerAddressMode::eClampToEdge,
                                             vk::SamplerAddressMode::eClampToEdge, 0.0f, false, 0.0f, false, vk::CompareOp::eNever, 0.0f, 0.0f, vk::BorderColor::eFloatOpaqueWhite, false);

  // init descriptor and pipeline layouts
  std::vector<vkhlf::DescriptorSetLayoutBinding> dslbs;
  dslbs.push_back(vkhlf::DescriptorSetLayoutBinding(0, vk::DescriptorType::eUniformBuffer, vk::ShaderStageFlagBits::eVertex, nullptr));
  dslbs.push_back(vkhlf::DescriptorSetLayoutBinding(1, vk::DescriptorType::eCombinedImageSampler, vk::ShaderStageFlagBits::eFragment, nullptr));
  std::shared_ptr<vkhlf::DescriptorSetLayout> descriptorSetLayout = getDevice()->createDescriptorSetLayout(dslbs);
  m_pipelineLayout = getDevice()->createPipelineLayout(descriptorSetLayout, nullptr);

  std::shared_ptr<vkhlf::DescriptorPool> descriptorPool = getDevice()->createDescriptorPool({}, 1, {{vk::DescriptorType::eUniformBuffer, 1}, {vk::DescriptorType::eCombinedImageSampler, 1}});

  // init descriptor set
  m_descriptorSet = getDevice()->allocateDescriptorSet(descriptorPool, descriptorSetLayout);
  std::vector<vkhlf::WriteDescriptorSet> wdss;
  wdss.push_back(vkhlf::WriteDescriptorSet(m_descriptorSet, 0, 0, 1, vk::DescriptorType::eUniformBuffer, nullptr, vkhlf::DescriptorBufferInfo(m_uniformBufferMVP, 0, sizeof(glm::mat4))));
  wdss.push_back(vkhlf::WriteDescriptorSet(m_descriptorSet, 1, 0, 1, vk::DescriptorType::eCombinedImageSampler, vkhlf::DescriptorImageInfo(m_textureSampler, m_textureImageView, vk::ImageLayout::eGeneral), nullptr));
  getDevice()->updateDescriptorSets(wdss, nullptr);

  // init vertex buffer
  m_vertexBuffer = getDevice()->createBuffer(sizeof(vertexData), vk::BufferUsageFlagBits::eTransferDst | vk::BufferUsageFlagBits::eVertexBuffer, vk::SharingMode::eExclusive, nullptr,
                                          vk::MemoryPropertyFlagBits::eDeviceLocal, m_deviceMemoryAllocatorBuffer);
  m_vertexBuffer->update<Vertex>(0, { sizeof(vertexData) / sizeof(Vertex), vertexData }, commandBuffer);
  commandBuffer->end();
  vkhlf::submitAndWait(getGraphicsQueue(), commandBuffer);

  // init shaders
  std::shared_ptr<vkhlf::ShaderModule> vertexShaderModule = getDevice()->createShaderModule(vkhlf::compileGLSLToSPIRV(vk::ShaderStageFlagBits::eVertex, vertShaderText));
  std::shared_ptr<vkhlf::ShaderModule> fragmentShaderModule = getDevice()->createShaderModule(vkhlf::compileGLSLToSPIRV(vk::ShaderStageFlagBits::eFragment, fragShaderText));

  // init pipeline
  std::shared_ptr<vkhlf::PipelineCache> pipelineCache = getDevice()->createPipelineCache(0, nullptr);
  vkhlf::PipelineShaderStageCreateInfo vertexStage(vk::ShaderStageFlagBits::eVertex, vertexShaderModule, "main");
  vkhlf::PipelineShaderStageCreateInfo fragmentStage(vk::ShaderStageFlagBits::eFragment, fragmentShaderModule, "main");
  vk::VertexInputBindingDescription binding(0, sizeof(vertexData[0]), vk::VertexInputRate::eVertex);
  vk::VertexInputAttributeDescription attrib0(0, 0, vk::Format::eR32G32B32A32Sfloat, 0);
  vk::VertexInputAttributeDescription attrib1(1, 0, vk::Format::eR32G32Sfloat, 16);
  vkhlf::PipelineVertexInputStateCreateInfo vertexInput(binding, {attrib0, attrib1});
  vk::PipelineInputAssemblyStateCreateInfo assembly({}, vk::PrimitiveTopology::eTriangleList, VK_FALSE);
  vkhlf::PipelineViewportStateCreateInfo viewport({{}}, {{}});   // one dummy viewport and scissor, as dynamic state sets them
  vk::PipelineRasterizationStateCreateInfo rasterization({}, true, false, vk::PolygonMode::eFill, vk::CullModeFlagBits::eBack, vk::FrontFace::eClockwise, false, 0.0f, 0.0f, 0.0f, 1.0f);
  vkhlf::PipelineMultisampleStateCreateInfo multisample(vk::SampleCountFlagBits::e1, false, 0.0f, nullptr, false, false);
  vk::StencilOpState stencilOpState(vk::StencilOp::eKeep, vk::StencilOp::eKeep, vk::StencilOp::eKeep, vk::CompareOp::eAlways, 0, 0, 0);
  vk::PipelineDepthStencilStateCreateInfo depthStencil({}, true, true, vk::CompareOp::eLessOrEqual, false, false, stencilOpState, stencilOpState, 0.0f, 0.0f);
  vk::PipelineColorBlendAttachmentState colorBlendAttachment(false, vk::BlendFactor::eZero, vk::BlendFactor::eZero, vk::BlendOp::eAdd, vk::BlendFactor::eZero, vk::BlendFactor::eZero, vk::BlendOp::eAdd,
                                                             vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG | vk::ColorComponentFlagBits::eB | vk::ColorComponentFlagBits::eA);
  vkhlf::PipelineColorBlendStateCreateInfo colorBlend(false, vk::LogicOp::eNoOp, colorBlendAttachment, {1.0f, 1.0f, 1.0f, 1.0f});
  vkhlf::PipelineDynamicStateCreateInfo dynamic({vk::DynamicState::eViewport, vk::DynamicState::eScissor});

  m_pipeline = getDevice()->createGraphicsPipeline(pipelineCache, {}, {vertexStage, fragmentStage}, vertexInput, assembly, nullptr, viewport, rasterization, multisample, depthStencil, colorBlend, dynamic,
                                                   m_pipelineLayout, getRenderPass());
  // update 
  updateCameraInformation(width, height);
}

Window::~Window()
{
  // TODO Resource tracking for DescriptorSets has not yet been implemented. Wait for the device being idle to ensure that the CommandBuffer can be released.
  getDevice()->waitIdle();
}

void Window::cursorPosEvent(double xPos, double yPos)
{
  if (m_tracking)
  {
    glm::vec3 delta = - m_trackFactor.x * float(xPos - m_trackStartX) * glm::vec3(m_view[0]) + m_trackFactor.y * float(yPos - m_trackStartY) * glm::vec3(m_view[1]);
    m_eyePos = m_trackStartEyePos + delta;
    m_centerPos = m_trackStartCenterPos + delta;
    paint();
  }
}

void Window::updateCameraInformation(int width, int height)
{
  m_fovy = glm::radians(45.0f);
  if (getFramebufferSwapchain()->getExtent().width > getFramebufferSwapchain()->getExtent().height)
  {
    m_fovy *= static_cast<float>(getFramebufferSwapchain()->getExtent().height) / static_cast<float>(getFramebufferSwapchain()->getExtent().width);
  }

  float fovx = m_fovy * getFramebufferSwapchain()->getExtent().width / getFramebufferSwapchain()->getExtent().height;
  float d = glm::length(m_eyePos - m_centerPos);
  m_trackFactor = glm::vec2(d * tan(fovx) / getFramebufferSwapchain()->getExtent().width, d * tan(m_fovy) / getFramebufferSwapchain()->getExtent().height);
}

void Window::doResize(int width, int height)
{
  assert((0 <= width) && (0 <= height));
  updateCameraInformation(width, height);
}

void Window::keyEvent(int key, int scancode, int action, int mods)
{
  switch (key)
  {
    case GLFW_KEY_ESCAPE:
      switch (action)
      {
        case GLFW_PRESS:
          glfwSetWindowShouldClose(getWindow(), GLFW_TRUE);
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}

void Window::mouseButtonEvent(int button, int action, int mods)
{
  if (button == GLFW_MOUSE_BUTTON_LEFT)
  {
    if (action == GLFW_PRESS)
    {
      assert(!m_tracking);
      glfwGetCursorPos(getWindow(), &m_trackStartX, &m_trackStartY);
      m_tracking = true;
      m_trackStartEyePos = m_eyePos;
      m_trackStartCenterPos = m_centerPos;
    }
    else
    {
      assert(m_tracking);
      m_tracking = false;
    }
  }
}

void Window::doPaint()
{
  // create a command pool for command buffer allocation
  std::shared_ptr<vkhlf::CommandPool> commandPool = getDevice()->createCommandPool(vk::CommandPoolCreateFlagBits::eResetCommandBuffer, getQueueFamilyIndex());
  std::shared_ptr<vkhlf::CommandBuffer> commandBuffer = commandPool->allocateCommandBuffer();

  // init uniform buffer
  glm::mat4 projection = glm::perspective(m_fovy, static_cast<float>(getFramebufferSwapchain()->getExtent().width) / static_cast<float>(getFramebufferSwapchain()->getExtent().height), 0.1f, 100.0f);
  m_view = glm::lookAt(m_eyePos, m_centerPos, m_upDir);
  glm::mat4 model = glm::mat4(1.0f);
  // Vulkan clip space has inverted Y and half Z.
  glm::mat4 clip = glm::mat4(1.0f, 0.0f, 0.0f, 0.0f,
                             0.0f, -1.0f, 0.0f, 0.0f,
                             0.0f, 0.0f, 0.5f, 0.0f,
                             0.0f, 0.0f, 0.5f, 1.0f);

  glm::mat4 mvpc = clip * projection * m_view * model;

  commandBuffer->begin();
  m_uniformBufferMVP->update<glm::mat4>(0, mvpc, commandBuffer);

  std::array<float, 4> ccv = { 0.462745f, 0.72549f, 0.0f };
  commandBuffer->beginRenderPass(getRenderPass(), getFramebufferSwapchain()->getFramebuffer(), vk::Rect2D({ 0, 0 }, getFramebufferSwapchain()->getExtent()),
                                     {vk::ClearValue(ccv), vk::ClearValue(vk::ClearDepthStencilValue(1.0f, 0))}, vk::SubpassContents::eInline);
  commandBuffer->bindPipeline(vk::PipelineBindPoint::eGraphics, m_pipeline);
  commandBuffer->bindDescriptorSets(vk::PipelineBindPoint::eGraphics, m_pipelineLayout, 0, {m_descriptorSet}, nullptr);
  commandBuffer->bindVertexBuffer(0, m_vertexBuffer, 0);
  commandBuffer->setViewport(0, vk::Viewport(0.0f, 0.0f, (float)getFramebufferSwapchain()->getExtent().width, (float)getFramebufferSwapchain()->getExtent().height, 0.0f, 1.0f));
  commandBuffer->setScissor(0, vk::Rect2D({ 0, 0 }, getFramebufferSwapchain()->getExtent()));
  commandBuffer->draw(sizeof(vertexData) / sizeof(vertexData[0]), 1, 0, 0);
  commandBuffer->endRenderPass();
  commandBuffer->end();

  getGraphicsQueue()->submit(vkhlf::SubmitInfo{ { getFramebufferSwapchain()->getPresentSemaphore() },{ vk::PipelineStageFlagBits::eColorAttachmentOutput }, commandBuffer, getRenderCompleteSemaphore() });
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
