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
#include <vkhlf/Types.h>

namespace vkhlf
{

  class QueryPool : public Reference<Device, Allocator>
  {
    public:
      VKHLF_API QueryPool(std::shared_ptr<Device> const& device, vk::QueryPoolCreateFlags flags, vk::QueryType queryType, uint32_t entryCount, vk::QueryPipelineStatisticFlags pipelineStatistics,
                       std::shared_ptr<Allocator> const& allocator);
      VKHLF_API virtual ~QueryPool();

      VKHLF_API std::vector<uint8_t> getResults(uint32_t startQuery, uint32_t queryCount, size_t dataSize, vk::DeviceSize stride, vk::QueryResultFlags flags);

      VKHLF_API operator vk::QueryPool() const;

#if !defined(NDEBUG)
      VKHLF_API vk::QueryType getQueryType() const;
#endif

      QueryPool(QueryPool const& rhs) = delete;
      QueryPool & operator=(QueryPool const& rhs) = delete;

    private:
      uint32_t      m_queryCount;
      vk::QueryPool m_query;

#if !defined(NDEBUG)
      vk::QueryType m_queryType;
#endif
  };

  inline QueryPool::operator vk::QueryPool() const
  {
    return m_query;
  }

} // namespace vk
