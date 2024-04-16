# MIT License
#
# Copyright (c) 2022 Meher Malladi, Luca Lobefaro, Ignacio Vizzo, Cyrill
# Stachniss, University of Bonn
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

set(BUILD_SHARED_LIBS
    OFF
    CACHE BOOL "TBB Shared")
set(TBBMALLOC_BUILD
    OFF
    CACHE BOOL "TBB Malloc")
set(TBB_EXAMPLES
    OFF
    CACHE BOOL "TBB Examples")
set(TBB_STRICT
    OFF
    CACHE BOOL "TBB Stric")
set(TBB_TEST
    OFF
    CACHE BOOL "TBB Test")

include(FetchContent)
if(${CMAKE_VERSION} GREATER_EQUAL 3.24)
  # version 3.24 introduces OVERRIDE_FIND_PACKAGE.
  FetchContent_Declare(
    tbb
    URL https://github.com/oneapi-src/oneTBB/archive/refs/tags/v2021.8.0.tar.gz
        OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    tbb
    URL https://github.com/oneapi-src/oneTBB/archive/refs/tags/v2021.8.0.tar.gz)
endif()

if(NOT tbb_POPULATED)
  FetchContent_Populate(tbb)
  if(${CMAKE_VERSION} GREATER_EQUAL 3.25)
    add_subdirectory(${tbb_SOURCE_DIR} ${tbb_BINARY_DIR} SYSTEM
                     EXCLUDE_FROM_ALL)
  else()
    # Emulate the SYSTEM flag introduced in CMake 3.25. Withouth this flag the
    # compiler will # consider this 3rdparty headers as source code and fail due
    # the -Werror flag.
    add_subdirectory(${tbb_SOURCE_DIR} ${tbb_BINARY_DIR} EXCLUDE_FROM_ALL)
    get_target_property(tbb_include_dirs tbb INTERFACE_INCLUDE_DIRECTORIES)
    set_target_properties(tbb PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES
                                         "${tbb_include_dirs}")
    if(${CMAKE_VERSION} LESS 3.24)
      # Emulate the OVERRIDE_FIND_PACKAGE behaviour in 3.24
      file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/fpRedirects")

      # dummy an empty config file. we add_subdirectory above anyways
      file(WRITE "${CMAKE_BINARY_DIR}/fpRedirects/tbb-config.cmake" "")

      # dummy a version file. same is done by OVERRIDE_FIND_PACKAGE by default
      file(
        WRITE "${CMAKE_BINARY_DIR}/fpRedirects/tbb-config-version.cmake"
        "set(PACKAGE_VERSION_COMPATIBLE TRUE)
      set(PACKAGE_VERSION_EXACT TRUE)
      ")
      # set the package dir for find_package calls downstream
      set(TBB_DIR "${CMAKE_BINARY_DIR}/fpRedirects")
      # prefer configs over module searches when find_packaging because of the
      # redirection
      set(CMAKE_FIND_PACKAGE_PREFER_CONFIG ON)
    endif()
  endif()
endif()
