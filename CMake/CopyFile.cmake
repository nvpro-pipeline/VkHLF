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


MACRO(copy_file_if_changed target infile outfile)
  add_custom_command( TARGET ${target}
                      POST_BUILD
                      COMMAND ${CMAKE_COMMAND}
                      ARGS -E copy_if_different "${infile}" "${outfile}"
                      DEPENDS "${infile}"
                     )
ENDMACRO()

FUNCTION(copy_files_if_changed target)
  # get integer with last element index and target removed
  list(LENGTH ARGN list_count)
  math(EXPR list_max_index ${list_count}-2)
  if ( ${list_max_index} GREATER 0 )
    list(GET ARGN -1 target_folder )
    foreach(i RANGE ${list_max_index})
      list(GET ARGN ${i} source)
      get_filename_component(basename "${source}" NAME)
      copy_file_if_changed( target "${source}" "${target_folder}/${basename}" )
    endforeach()
  endif()
ENDFUNCTION()
