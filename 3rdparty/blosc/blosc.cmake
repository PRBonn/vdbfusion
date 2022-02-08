include(ExternalProject)
ExternalProject_Add(
  external_blosc
  PREFIX blosc
  URL https://github.com/Blosc/c-blosc/archive/refs/tags/v1.5.0.tar.gz
  URL_HASH SHA256=208ba4db0e5116421ed2fbbdf2adfa3e1d133d29a6324a0f47cf2d71f3810c92
  UPDATE_COMMAND ""
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
             ${ExternalProject_CMAKE_ARGS}
             ${ExternalProject_CMAKE_CXX_FLAGS}
             # Custom OpenVDB build settings
             -DBUILD_STATIC=ON
             -DBUILD_TESTS=OFF
             -DBUILD_BENCHMARKS=OFF
             -DPREFER_EXTERNAL_COMPLIBS=OFF)

# Simulate importing Blosc::blosc for OpenVDBHelper target
ExternalProject_Get_Property(external_blosc INSTALL_DIR)
set(BLOSC_ROOT ${INSTALL_DIR} CACHE INTERNAL "BLOSC_ROOT Install directory")
add_library(BloscHelper INTERFACE)
add_dependencies(BloscHelper external_blosc)
target_include_directories(BloscHelper INTERFACE ${INSTALL_DIR}/include)
target_link_directories(BloscHelper INTERFACE ${INSTALL_DIR}/lib)
target_link_libraries(BloscHelper INTERFACE blosc.a)
add_library(Blosc::blosc ALIAS BloscHelper)
