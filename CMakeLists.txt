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

PROJECT( VkHLF )

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMake")

#GLFW3 doesn't add the Vulkan include directories by default. Add them before.
include_directories($ENV{VK_SDK_PATH}/include)

option(GLFW_BUILD_EXAMPLES "Build the GLFW example programs" OFF)
option(GLFW_BUILD_TESTS "Build the GLFW test programs" OFF)
option(GLFW_BUILD_DOCS "Build the GLFW documentation" OFF)
option(GLFW_INSTALL "Generate installation target" OFF)
add_subdirectory(3rdparty/glfw)

#add glslang
option(ENABLE_GLSLANG_BINARIES "Enable GLSLang Binaries" OFF)
add_subdirectory(3rdparty/glslang)


set(ENV{GLFW_ROOT} ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/glfw)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

if ( "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "armv7l")
  set (VK_ARCH "${CMAKE_SYSTEM_PROCESSOR}" CACHE STRING "CPU Architecture" )
elseif( CMAKE_SIZEOF_VOID_P EQUAL 8 )
  set (VK_ARCH "x64")
else ()
  set (VK_ARCH "x86")
endif()


if ("${CMAKE_GENERATOR}" MATCHES "^(Visual Studio).*")
  set (CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} /MP")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
  set (VK_CONFIG "$(ConfigurationName)")
else()
  if( "${CMAKE_BUILD_TYPE}" STREQUAL "" )
    set (VK_CONFIG "release")
  endif()
endif()

if ( ${VK_ARCH} STREQUAL "armv7l" )
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv7-a -mtune=cortex-a9 -mfloat-abi=hard -mfpu=neon" )
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mtune=cortex-a9 -mfloat-abi=hard -mfpu=neon")
endif()

if(UNIX)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

  if (VK_ARCH STREQUAL "x64")
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -msse4.1")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse4.1")
  endif()
endif()

#set binary directory
if(WIN32 AND "${CMAKE_GENERATOR}" MATCHES "^(Visual Studio).*")
    set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib" )
    set( CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin" )
    set( CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin" )
  else()
    set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib/${VK_CONFIG}" )
    set( CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin/${VK_CONFIG}" )
    set( CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin/${VK_CONFIG}" )
endif()

set(VK_3RDPARTY_PATH "${CMAKE_CURRENT_SOURCE_DIR}/3rdParty" CACHE STRING "CkSpp 3rdParty path")
set(VK_BINARY_PATH "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${VK_CONFIG}")
message( "VK_BINARY_PATH = ${VK_BINARY_PATH}" )

# some useful macros
MACRO(ADD_TARGET_PROPERTIES _target _name)

  SET(_properties)
  FOREACH(_prop ${ARGN})
    SET(_properties "${_properties} ${_prop}")
  ENDFOREACH(_prop)
  GET_TARGET_PROPERTY(_old_properties ${_target} ${_name})
  IF(NOT _old_properties)
    # in case it's NOTFOUND
    SET(_old_properties)
  ENDIF(NOT _old_properties)
  SET_TARGET_PROPERTIES(${_target} PROPERTIES ${_name} "${_old_properties} ${_properties}")

ENDMACRO(ADD_TARGET_PROPERTIES)

MACRO(TARGET_INCLUDE_SYMBOL target symbol)
  if (WIN32)
    if ( VK_ARCH STREQUAL "x64" )
      add_target_properties( ${target} LINK_FLAGS /include:${symbol} )
    elseif( VK_ARCH STREQUAL "x86" )
      add_target_properties( ${target} LINK_FLAGS /include:_${symbol} )
    endif()
  endif()

  if(UNIX)
    add_target_properties( ${target} LINK_FLAGS "-Wl,--undefined=${symbol}" )
  endif()

ENDMACRO()

include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

add_subdirectory( samples )
add_subdirectory( vkhlf )
