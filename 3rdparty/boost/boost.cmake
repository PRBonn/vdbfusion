set(BOOST_URL "https://boostorg.jfrog.io/artifactory/main/release/1.71.0/source/boost_1_71_0.tar.bz2")
set(BOOST_URL_SHA256 "d73a8da01e8bf8c7eda40b4c84915071a8c8a0df4a6734537ddde4a8580524ee")
set(BOOST_CONFIGURE <SOURCE_DIR>/bootstrap.sh --with-libraries=iostreams,regex)
set(BOOST_INSTALL
    <SOURCE_DIR>/b2
    install
    link=static
    warnings=off
    cxxflags=-fPIC
    cflags=-fPIC
    --prefix=<INSTALL_DIR>)

include(ExternalProject)
ExternalProject_Add(
  external_boost
  PREFIX boost
  URL ${BOOST_URL}
  URL_HASH SHA256=${BOOST_URL_SHA256}
  BUILD_IN_SOURCE true
  CONFIGURE_COMMAND "${BOOST_CONFIGURE}"
  BUILD_COMMAND ""
  INSTALL_COMMAND "${BOOST_INSTALL}")

# Simulate importing Boost::iostreams for OpenVDBHelper target
ExternalProject_Get_Property(external_boost INSTALL_DIR)
set(BOOST_ROOT ${INSTALL_DIR} CACHE INTERNAL "Boost libraries Install directory")
add_library(BoostIostreamsHelper INTERFACE)
add_dependencies(BoostIostreamsHelper external_boost_iostreams)
target_include_directories(BoostIostreamsHelper INTERFACE ${INSTALL_DIR}/include)
target_link_directories(BoostIostreamsHelper INTERFACE ${INSTALL_DIR}/lib)
target_link_libraries(BoostIostreamsHelper INTERFACE boost_iostreams.a)
add_library(Boost::iostreams ALIAS BoostIostreamsHelper)
