# MIT License
#
# Copyright (c) 2022 Luca Lobefaro, Meher Malladi, Ignacio Vizzo, Cyrill
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
set(BUILD_STATIC ON CACHE BOOL "BLOSC built as static lib.")
set(BUILD_SHARED OFF CACHE BOOL "BLOSC built as shared lib.")
set(BUILD_TESTS OFF CACHE BOOL "BLOSC tests build.")
set(BUILD_BENCHMARKS OFF CACHE BOOL "BLOSC build benchmark programs.")
set(PREFER_EXTERNAL_COMPLIBS OFF CACHE BOOL "BLOSC prefers installed depenbdency instead of includes sources.")

include(FetchContent)
set(blosc_fetch_content_args URL https://github.com/Blosc/c-blosc/archive/refs/tags/v1.21.5.tar.gz)
if(${CMAKE_VERSION} GREATER_EQUAL 3.28)
  list(APPEND blosc_fetch_content_args SYSTEM EXCLUDE_FROM_ALL OVERRIDE_FIND_PACKAGE)
endif()

FetchContent_Declare(blosc ${blosc_fetch_content_args})

if(${CMAKE_VERSION} GREATER_EQUAL 3.28)
  FetchContent_MakeAvailable(blosc)
else()
  # we cannot use MakeAvailable because EXCLUDE_FROM_ALL in Declare is 3.28
  FetchContent_GetProperties(blosc)
  if(NOT blosc_POPULATED)
    FetchContent_Populate(blosc)
    if(${CMAKE_VERSION} GREATER_EQUAL 3.25)
      add_subdirectory(${blosc_SOURCE_DIR} ${blosc_BINARY_DIR} SYSTEM EXCLUDE_FROM_ALL)
    else()
      # Emulate the SYSTEM flag introduced in CMake 3.25. Withouth this flag the
      # compiler will consider this 3rdparty headers as source code and fail due
      # the -Werror flag.
      add_subdirectory(${blosc_SOURCE_DIR} ${blosc_BINARY_DIR} EXCLUDE_FROM_ALL)
      get_target_property(blosc_include_dirs blosc_static INTERFACE_INCLUDE_DIRECTORIES)
      set_target_properties(blosc_static PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${blosc_include_dirs}")
    endif()
    # Emulate the OVERRIDE_FIND_PACKAGE behaviour in 3.24. OVERRIDE creates dummy
    # config and config version files and then forces find_package to first
    # check the dir which has these files. Instead here we create a dummy
    # Find<package>.cmake file and modify CMAKE_MODULE_PATH since openvdb uses
    # the basic call signature and prefers modules first. Setting
    # CMAKE_FIND_PACKAGE_PREFER_CONFIG ON was for some reason inconsistent. This
    # is a bit simpler anyway. At least with this you dont have to set
    # package_DIR extra.
    file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/fpRedirects")

    # Dummy an empty find file. We add_subdirectory above anyways
    file(WRITE "${CMAKE_BINARY_DIR}/fpRedirects/FindBlosc.cmake" "")

    # Modify cmake module path only if it already doesnt have the directory
    list(FIND CMAKE_MODULE_PATH "${CMAKE_BINARY_DIR}/fpRedirects" _index)
    if(${_index} EQUAL -1)
      list(APPEND CMAKE_MODULE_PATH "${CMAKE_BINARY_DIR}/fpRedirects")
    endif()
  endif()
endif()

add_library(Blosc::blosc ALIAS blosc_static)
