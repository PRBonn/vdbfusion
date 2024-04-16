# MIT License
#
# Copyright (c) 2022 Luca Lobefaro, Meher Malladi, Ignacio Vizzo, Cyrill Stachniss, University of Bonn
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
set(BUILD_STATIC ON)
set(BUILD_TESTS OFF)
set(BUILD_BENCHMARKS OFF)
set(PREFER_EXTERNAL_COMPLIBS OFF)

include(FetchContent)
FetchContent_Declare(blosc URL https://github.com/Blosc/c-blosc/archive/refs/tags/v1.5.0.tar.gz)
if(NOT blosc_POPULATED)
  FetchContent_Populate(blosc)
  if(${CMAKE_VERSION} GREATER_EQUAL 3.25)
    add_subdirectory(${blosc_SOURCE_DIR} ${blosc_BINARY_DIR} SYSTEM EXCLUDE_FROM_ALL)
  else()
    # Emulate the SYSTEM flag introduced in CMake 3.25. Withouth this flag the compiler will
    # consider this 3rdparty headers as source code and fail due the -Werror flag.
    add_subdirectory(${blosc_SOURCE_DIR} ${blosc_BINARY_DIR} EXCLUDE_FROM_ALL)
    get_target_property(blosc_include_dirs blosc INTERFACE_INCLUDE_DIRECTORIES)
    set_target_properties(blosc PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${blosc_include_dirs}")
  endif()
endif()
