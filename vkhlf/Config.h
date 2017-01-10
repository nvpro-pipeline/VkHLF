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

#if defined(_WIN32)
#define VK_OS_WINDOWS
#endif

#if defined(__MINGW32__)
#define VK_OS_WINDOWS
#define VK_POSIX
#endif

#if defined(__linux__)
#define VK_OS_LINUX
#define VK_POSIX

# define _stricmp strcasecmp
#endif


#if defined(VK_OS_WINDOWS)
// microsoft specific storage-class defines
# if defined(VKHLF_EXPORTS)
#  define VKHLF_API __declspec(dllexport)
# else
#  define VKHLF_API __declspec(dllimport)
# endif
#else
# define VKHLF_API
#endif

#include <assert.h>
#include <sstream>
#include <vulkan/vulkan.hpp>

namespace vkhlf
{
  /** CheckLimit verifies that the value of type TIn does fit into type TOut
    *
    * signed bit loss:
    * in signed -> out signed: always okay
    * in signed -> out unsigned: might lose sign bit
    * in unsigned -> out signed: always okay
    * in unsigned -> out unsigned: always okay
    *
    * digits loss:
    *  InLessEqualThanOut false, out has always enough bits to hold in
    *  InLessEqualThanOut true, verify that all enabled bits from in fit into out.
  **/
  template <bool InLessEqualThanOut, bool outSigned, bool inSigned, typename TOut, typename TIn>
  struct CheckLimit
  {
    static bool checkLimit( TIn in );
  };

  template <bool outSigned, bool inSigned, typename TOut, typename TIn>
  struct CheckLimit<true, outSigned, inSigned, TOut, TIn>
  {
    static bool checkLimit( TIn /*in*/ )
    {
      return true;
    }
  };

  // Tin::digits < Tout::digits, out unsigned, in signed -> might lose sign bit
  template <typename TOut, typename TIn>
  struct CheckLimit<true, false, true, TOut, TIn>
  {
    static bool checkLimit( TIn in )
    {
      return in >= TIn( 0 );
    }
  };

  // TIn::digits > TOut::digits generic
  template <bool outSigned, bool inSigned, typename TOut, typename TIn>
  struct CheckLimit<false, outSigned, inSigned, TOut, TIn>
  {
    static bool checkLimit( TIn in )
    {
      // since TIn has more digits than TOut the sign bit of TIn will be killed always. Thus it's possible to skip the special case (in >= 0).
      TIn const mask = (TIn(1) << (std::numeric_limits<TOut>::digits)) - 1;

      return ( (in & mask) == in );
    }
  };

  // Tin::digits > TOut::digits, out signed, in signed, simple mask test is not enough because negative numbers have the most significant bits set
  template <typename TOut, typename TIn>
  struct CheckLimit<false, true, true, TOut, TIn>
  {
    static bool checkLimit( TIn in )
    {
      return std::numeric_limits<TOut>::min() <= in && in <= std::numeric_limits<TOut>::max();
    }
  };

  template <typename TOut, typename TIn>
  struct CheckRange
  {
    static bool checkLimit( TIn const & in )
    {
      return CheckLimit<std::numeric_limits<TIn>::digits <= std::numeric_limits<TOut>::digits
                        , std::numeric_limits<TOut>::is_signed, std::numeric_limits<TIn>::is_signed, TOut, TIn>::checkLimit( in );
    }
  };


  template<typename TOut, typename TIn>
  inline TOut checked_cast_integer( TIn in )
  {

    // this is exclusively for TOut and Tin being integer
    static_assert(std::numeric_limits<TOut>::is_integer && std::numeric_limits<TIn>::is_integer, "");

    if ( !CheckRange<TOut, TIn>::checkLimit( in ) )
    {
      throw std::runtime_error( "checked_cast detected that the range of the output type is not sufficient for the input value" );
    }

    return static_cast<TOut>(in);
  }

  // general implementation handle non-pure integer cases
  template <typename TOut, typename TIn, bool integer>
  struct Caster
  {
    TOut operator()(TIn in) {
      // more checks on non-pure integer to be done
      return static_cast<TOut>(in);
    }
  };

  // specialization for pure-integer conversion
  template <typename TOut, typename TIn>
  struct Caster<TOut, TIn, true>
  {
    TOut operator()(TIn in) { return checked_cast_integer<TOut>(in); }
  };

  template<typename TOut, typename TIn>
  inline TOut checked_cast( TIn in )
  {
    Caster<TOut, TIn, std::numeric_limits<TOut>::is_integer && std::numeric_limits<TIn>::is_integer> theCaster;
    return theCaster(in);
  }


  inline void verify( vk::Result result, char const* call, char const* file, unsigned int line )
  {
    if ( result != vk::Result::eSuccess )
    {
      std::ostringstream oss;
      oss << "Error on executing " << call << " in file <" << file << "> line " << line << ": " << vk::to_string( result ) << std::endl;
      throw std::runtime_error( oss.str() );
    }
  }

} // namespace vk

#if !defined(NDEBUG)
# define VK_VERIFY( fct )   vkhlf::verify( fct, #fct, __FILE__, __LINE__ )
#else
# define VK_VERIFY( fct )   fct
#endif
