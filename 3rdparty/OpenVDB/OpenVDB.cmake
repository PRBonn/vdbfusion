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

include(${CMAKE_CURRENT_LIST_DIR}/../tbb/tbb.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/../blosc/blosc.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/../boost/boost.cmake)

if(BUILD_PYTHON_BINDINGS)
  # openvdb handles installing the pybind to site_packages as pyopenvdb.so
  set(OPENVDB_BUILD_PYTHON_MODULE ON CACHE BOOL "OpenVDB python module.")
else()
  set(OPENVDB_BUILD_PYTHON_MODULE OFF CACHE BOOL "OpenVDB python module.")
endif()
set(OPENVDB_CORE_SHARED OFF CACHE BOOL "OpenVDB shared lib.")
set(OPENVDB_CORE_STATIC ON CACHE BOOL "OpenVDB static lib.")
set(OPENVDB_USE_DELAYED_LOADING OFF CACHE BOOL "OpenVDB delayed loading.")
set(OPENVDB_CXX_STRICT OFF CACHE BOOL "OpenVDB CXX strict.")
set(OPENVDB_BUILD_VDB_PRINT OFF CACHE BOOL "OpenVDB build VDB print.")
set(OPENVDB_INSTALL_CMAKE_MODULES OFF CACHE BOOL "OpenVDB install cmake modules.")
set(USE_STATIC_DEPENDENCIES ON CACHE BOOL "OpenVDB use static deps.")
set(USE_AX OFF CACHE BOOL "OpenVDB use AX.")
set(USE_NANOVDB OFF CACHE BOOL "OpenVDB use nanovdb.")
set(USE_ZLIB OFF CACHE BOOL "OpenVDB use ZLib.")

include(FetchContent)
FetchContent_Declare(
  openvdb
  URL https://github.com/AcademySoftwareFoundation/openvdb/archive/refs/tags/v11.0.0.tar.gz
)
if(NOT openvdb_POPULATED)
  FetchContent_Populate(openvdb)
  if(${CMAKE_VERSION} GREATER_EQUAL 3.25)
    add_subdirectory(${openvdb_SOURCE_DIR} ${openvdb_BINARY_DIR} SYSTEM EXCLUDE_FROM_ALL)
  else()
    # Emulate the SYSTEM flag introduced in CMake 3.25. Withouth this flag the
    # compiler will consider this 3rdparty headers as source code and fail due
    # the -Werror flag.
    add_subdirectory(${openvdb_SOURCE_DIR} ${openvdb_BINARY_DIR} EXCLUDE_FROM_ALL)
    get_target_property(openvdb_include_dirs openvdb_static INTERFACE_INCLUDE_DIRECTORIES)
    set_target_properties(openvdb_static PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${openvdb_include_dirs}")
  endif()
endif()

add_library(OpenVDB::openvdb ALIAS openvdb_static)
