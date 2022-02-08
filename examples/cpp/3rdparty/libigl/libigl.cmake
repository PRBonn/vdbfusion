include(ExternalProject)
ExternalProject_Add(
  external_libigl
  PREFIX libigl
  GIT_REPOSITORY https://github.com/libigl/libigl.git
  GIT_TAG v2.3.0
  GIT_SHALLOW ON
  BUILD_COMMAND ""
  CONFIGURE_COMMAND ""
  INSTALL_COMMAND ""
  UPDATE_COMMAND "")

# Simulate importing libigl
ExternalProject_Get_Property(external_libigl SOURCE_DIR)
add_library(libiglHelper INTERFACE)
add_dependencies(libiglHelper external_libigl)
target_include_directories(libiglHelper INTERFACE ${SOURCE_DIR}/include)
add_library(igl::core ALIAS libiglHelper)
