cmake_minimum_required(VERSION 3.10)
project(VDBFusion VERSION 0.1.6 LANGUAGES CXX)

set (CMAKE_CXX_STANDARD 17)

# Setup build options
option(USE_CCACHE "Build using Ccache if found on the path" ON)
option(USE_SYSTEM_EIGEN3 "Use system pre-installed eigen3" ON)
option(USE_SYSTEM_OPENVDB "Use system pre-installed OpenVDB" ON)
option(USE_SYSTEM_PYBIND11 "Use system pre-installed pybind11" ON)
option(SILENCE_WARNINGS "To build manylinux packages only, disable on the global scope" OFF)
option(BUILD_CXX_EXAMPLE "Build C++ examples" OFF)
option(BUILD_PYTHON_BINDINGS "Build the python module" OFF)

mark_as_advanced(USE_CCACHE USE_SYSTEM_OPENVDB SILENCE_WARNINGS BUILD_PYTHON_BINDINGS)

if(SILENCE_WARNINGS)
  message(WARNING "Disable all warnings for all targets in the build system, only for the CI/CD")
  add_compile_options(-w)
endif()

add_compile_options(-gdwarf-4)

# ccache setup
if(USE_CCACHE)
  find_program(CCACHE_PATH ccache)
  if(CCACHE_PATH)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
    message(STATUS "Using ccache: ${CCACHE_PATH}")
  endif()
endif()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find Dependencies from the 3rdparty directory.
include(3rdparty/find_dependencies.cmake)

if(BUILD_CXX_EXAMPLE)
  add_subdirectory(examples)
endif()
add_subdirectory(src)
