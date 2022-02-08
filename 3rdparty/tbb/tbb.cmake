include(ExternalProject)
ExternalProject_Add(
  external_tbb
  PREFIX tbb
  URL https://github.com/nachovizzo/tbb/archive/refs/tags/tbbstatic.tar.gz
  URL_HASH SHA256=db5ede77c4bd10ad12fab11ed38b7e8cf80aba85db16a57514073c383e6c8630
  UPDATE_COMMAND ""
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
             ${ExternalProject_CMAKE_ARGS}
             ${ExternalProject_CMAKE_CXX_FLAGS}
             # custom flags
             -DTBB_BUILD_TBBMALLOC=ON
             -DTBB_BUILD_SHARED=OFF
             -DTBB_BUILD_STATIC=ON
             -DTBB_BUILD_TESTS=OFF)

# Simulate importing TBB::tbb for OpenVDBHelper target
ExternalProject_Get_Property(external_tbb INSTALL_DIR)
set(TBB_ROOT ${INSTALL_DIR} CACHE INTERNAL "TBB_ROOT Install directory")
add_library(TBBHelper INTERFACE)
add_dependencies(TBBHelper external_tbb)
target_include_directories(TBBHelper INTERFACE ${INSTALL_DIR}/include)
target_link_directories(TBBHelper INTERFACE ${INSTALL_DIR}/lib)
target_link_libraries(TBBHelper INTERFACE tbb)
add_library(TBB::tbb ALIAS TBBHelper)
