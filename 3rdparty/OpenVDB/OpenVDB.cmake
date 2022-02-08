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
  # TODO: Update these once changes has been merged to upstream GIT_REPOSITORY
  # https://github.com/AcademySoftwareFoundation/openvdb.git
  GIT_REPOSITORY https://github.com/nachovizzo/openvdb.git
  GIT_TAG nacho/vdbfusion
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
