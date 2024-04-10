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

# Include superbuild dependencies for tbb and blos-c
include(${CMAKE_CURRENT_LIST_DIR}/../tbb/tbb.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/../blosc/blosc.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/../boost/boost.cmake)
include(ExternalProject)
include(GNUInstallDirs)

# Simulate a local installation of OpenVDB library
ExternalProject_Add(
  external_openvdb
  PREFIX openvdb
  GIT_REPOSITORY https://github.com/AcademySoftwareFoundation/openvdb.git
  GIT_TAG v11.0.0
  GIT_SHALLOW ON
  UPDATE_COMMAND ""
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
             ${ExternalProject_CMAKE_ARGS}
             ${ExternalProject_CMAKE_CXX_FLAGS}
             # Custom OpenVDB build settings
             -DCMAKE_POSITION_INDEPENDENT_CODE=ON
             -DOPENVDB_BUILD_PYTHON_MODULE=OFF
             -DOPENVDB_BUILD_VDB_PRINT=OFF
             -DOPENVDB_CORE_SHARED=OFF
             -DOPENVDB_CORE_STATIC=ON
             -DOPENVDB_CXX_STRICT=OFF
             -DUSE_CCACHE=ON
             -DUSE_STATIC_DEPENDENCIES=ON
             -DUSE_ZLIB=OFF
             # Specify super libraries root directories
             -DBOOST_ROOT=${BOOST_ROOT}
             -DTBB_ROOT=${TBB_ROOT}
             -DBLOSC_ROOT=${BLOSC_ROOT})

ExternalProject_Add_StepDependencies(external_openvdb build external_boost)
ExternalProject_Add_StepDependencies(external_openvdb build external_tbb)
ExternalProject_Add_StepDependencies(external_openvdb build external_blosc)

# Simulate importing OpenVDB::openvdb
ExternalProject_Get_Property(external_openvdb INSTALL_DIR)
add_library(OpenVDBHelper INTERFACE)
add_dependencies(OpenVDBHelper external_openvdb)
target_include_directories(OpenVDBHelper INTERFACE ${INSTALL_DIR}/${CMAKE_INSTALL_INCLUDEDIR})
target_link_directories(OpenVDBHelper INTERFACE ${INSTALL_DIR}/${CMAKE_INSTALL_LIBDIR})
target_link_libraries(OpenVDBHelper INTERFACE openvdb)

# Setup Visible dependencies
find_package(Threads REQUIRED)
set(_OPENVDB_DEPENDENCIES)
list(APPEND _OPENVDB_DEPENDENCIES Boost::iostreams)
list(APPEND _OPENVDB_DEPENDENCIES Threads::Threads)
list(APPEND _OPENVDB_DEPENDENCIES TBB::tbb)
list(APPEND _OPENVDB_DEPENDENCIES Blosc::blosc)
target_link_libraries(OpenVDBHelper INTERFACE ${_OPENVDB_DEPENDENCIES})
add_library(OpenVDB::openvdb ALIAS OpenVDBHelper)
