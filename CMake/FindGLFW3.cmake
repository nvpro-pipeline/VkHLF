# Locate the glfw library
# This module defines the following variables:
# GLFW_LIBRARY, the name of the library;
# GLFW_INCLUDE_DIR, where to find glfw include files.
# GLFW_FOUND, true if both the GLFW_LIBRARY and GLFW_INCLUDE_DIR have been found.
#
# To help locate the library and include file, you could define an environment variable called
# GLFW_ROOT which points to the root of the glfw library installation. This is pretty useful
# on a Windows platform.
#
#
# Usage example to compile an "executable" target to the glfw library:
#
# FIND_PACKAGE (glfw REQUIRED)
# INCLUDE_DIRECTORIES (${GLFW_INCLUDE_DIR})
# ADD_EXECUTABLE (executable ${EXECUTABLE_SRCS})
# TARGET_LINK_LIBRARIES (executable ${GLFW_LIBRARY})
#
# TODO:
# Allow the user to select to link to a shared library or to a static library.

#Search for the include file...
FIND_PATH(GLFW_INCLUDE_DIRS GLFW/glfw3.h DOC "Path to GLFW include directory."
  HINTS
  $ENV{GLFW_ROOT}/include
  PATH_SUFFIX include #For finding the include file under the root of the glfw expanded archive, typically on Windows.
  PATHS
  /usr/include/
  /usr/local/include/
  # By default headers are under GLFW subfolder
  /usr/include/GLFW
  /usr/local/include/GLFW
)

SET(GLFW_LIB_NAMES libglfw3.a glfw3 glfw GLFW3.lib)

#FIND_LIBRARY(GLFW_LIBRARIES DOC "Absolute path to GLFW library."
#  NAMES ${GLFW_LIB_NAMES}
#  HINTS
#  $ENV{GLFW_ROOT}/lib-vc2013
#  $ENV{GLFW_ROOT}/lib-vc2015
#  PATH_SUFFIXES lib/win32 #For finding the library file under the root of the glfw expanded archive, typically on Windows.
#  PATHS
#  /usr/local/lib
#  /usr/lib
#)
set(GLFW_LIBRARIES glfw CACHE PATH "glfw libraries")

IF( APPLE )
    find_library(IOKIT NAMES IOKit)
    #find_library(OpenGL NAMES OpenGL)
    find_library(COREVIDEO NAMES CoreVideo)
    find_library(COCOA NAMES Cocoa)
    SET(GLFW_LIBRARIES ${GLFW_LIBRARIES} ${IOKIT} ${COREVIDEO} ${COCOA})
endif( APPLE )

IF(GLFW_LIBRARIES AND GLFW_INCLUDE_DIRS)
  SET(GLFW_FOUND TRUE)
  message(STATUS "Found GLFW3: ${GLFW_LIBRARIES}")
ELSE()
  message(STATUS "GLFW3 NOT found!")
ENDIF(GLFW_LIBRARIES AND GLFW_INCLUDE_DIRS)

#if(GLFW_FOUND)
#  MARK_AS_ADVANCED(GLFW_INCLUDE_DIRS GLFW_LIBRARIES)
#endif(GLFW_FOUND)
