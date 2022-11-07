# MIT License
#
# # Copyright (c) 2022 Ignacio Vizzo, Cyrill Stachniss, University of Bonn
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

# CMake arguments for configuring ExternalProjects.
set(ExternalProject_CMAKE_ARGS
    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    -DCMAKE_CXX_COMPILER_LAUNCHER=${CMAKE_CXX_COMPILER_LAUNCHER}
    -DCMAKE_BUILD_TYPE=Release
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON)

if(SILENCE_WARNINGS)
  set(ExternalProject_CMAKE_CXX_FLAGS "-DCMAKE_CXX_FLAGS=-w")
endif()

if(USE_SYSTEM_EIGEN3)
  find_package(Eigen3 QUIET NO_MODULE)
endif()
if(NOT USE_SYSTEM_EIGEN3 OR NOT EIGEN3_FOUND)
  set(USE_SYSTEM_EIGEN3 OFF)
  include(${CMAKE_CURRENT_LIST_DIR}/eigen/eigen.cmake)
endif()

if(BUILD_PYTHON_BINDINGS)
  if(USE_SYSTEM_PYBIND11)
    find_package(pybind11 QUIET)
  endif()
  if(NOT USE_SYSTEM_PYBIND11 OR NOT pybind11_FOUND)
    set(USE_SYSTEM_PYBIND11 OFF)
    include(${CMAKE_CURRENT_LIST_DIR}/pybind11/pybind11.cmake)
  endif()
endif()

if(USE_SYSTEM_OPENVDB)
  # When OpenVDB is available on the system, we just go for the dynamic version of it
  include(GNUInstallDirs)
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_INSTALL_FULL_LIBDIR}/cmake/OpenVDB")
  find_package(OpenVDB QUIET)
  if(OpenVDB_FOUND AND OpenVDB_USES_BLOSC)
    # We need to get these hidden dependencies (if available) to static link them inside our library
    target_link_libraries(OpenVDB::openvdb INTERFACE Blosc::blosc)
  endif()
endif()
# When not using a pre installed version of OpenVDB we assume that no dependencies are installed and
# therefore build blos-c, tbb, and libboost from soruce
if(NOT USE_SYSTEM_OPENVDB OR NOT OpenVDB_FOUND)
  set(USE_SYSTEM_OPENVDB OFF)
  include(${CMAKE_CURRENT_LIST_DIR}/OpenVDB/OpenVDB.cmake)
endif()
