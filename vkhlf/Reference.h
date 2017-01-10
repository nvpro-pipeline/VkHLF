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

namespace vkhlf
{

  template<typename... Ts>
  class Reference
  {};

  template<typename T, typename... Ts>
  class Reference<T, Ts...>
  {
    public:
      Reference(std::shared_ptr<T> const& t, std::shared_ptr<Ts>... ts)
        : m_head(t)
        , m_tail(ts...)
      {}

      template<class U>
      std::shared_ptr<U> const& get() const { return m_tail.get<U>(); };

      template<>
      std::shared_ptr<T> const& get() const { return m_head; };

    protected:
      friend class Reference;
      template <class U> void set(std::shared_ptr<U> const& ptr) { m_tail.set<U>(ptr); }
      template <>        void set(std::shared_ptr<T> const& ptr) { m_head = ptr; }

    private:
      std::shared_ptr<T>  m_head;
      Reference<Ts...>    m_tail;
  };

} // namespace vk
