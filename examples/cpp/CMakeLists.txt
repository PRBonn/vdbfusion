cmake_minimum_required(VERSION 3.10)
project(VDBFusion_Examples)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
  find_package(VDBFusion REQUIRED)
endif()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Pull dependencies used only for the C++ examples
include(3rdparty/find_dependencies.cmake)
add_subdirectory(datasets)

add_compile_options(-gdwarf-4)
add_executable(kitti_pipeline kitti_pipeline.cpp)
target_compile_features(kitti_pipeline PUBLIC cxx_std_11)
target_link_libraries(kitti_pipeline PRIVATE VDBFusion::vdbfusion argparse::argparse fmt::fmt igl::core datasets)
set_target_properties(kitti_pipeline PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
