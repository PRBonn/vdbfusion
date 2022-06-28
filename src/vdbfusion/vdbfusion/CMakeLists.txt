cmake_minimum_required(VERSION 3.10)
project(VDBFusion VERSION 0.1.6 LANGUAGES CXX)

add_library(vdbfusion STATIC)
add_library(VDBFusion::vdbfusion ALIAS vdbfusion)

# In case we are building this as a standalone project, we should look here the dependencies
if(NOT TARGET Eigen3::Eigen OR NOT TARGET OpenVDB::openvdb)
  message(STATUS "Building only VDBFusion C++ module")
  include(GNUInstallDirs)
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_INSTALL_FULL_LIBDIR}/cmake/OpenVDB")
  set(VDBFUSION_STANDALONE ON)
  find_package(Eigen3 REQUIRED)
  find_package(OpenVDB REQUIRED)
endif()

add_compile_options(-gdwarf-4)
target_sources(vdbfusion PRIVATE MarchingCubes.cpp VDBVolume.cpp)
target_compile_features(vdbfusion PUBLIC cxx_std_11)
target_compile_options(vdbfusion PRIVATE -Wall -Wextra)
set_target_properties(vdbfusion PROPERTIES PUBLIC_HEADER "VDBVolume.h")
target_include_directories(
  vdbfusion PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
                                                       $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(vdbfusion PUBLIC Eigen3::Eigen OpenVDB::openvdb)

# NOTE: Installation is still work in progress, therefore we only support dev machines with all the 3rdparty
# dependencies locally installed
if((USE_SYSTEM_EIGEN3 AND USE_SYSTEM_OPENVDB) OR VDBFUSION_STANDALONE)
  include(cmake/Install.cmake)
endif()

# Google tests
add_subdirectory(tests)
