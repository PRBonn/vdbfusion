include(ExternalProject)

ExternalProject_Add(
  external_eigen
  PREFIX eigen
  URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.bz2
  URL_HASH SHA256=b4c198460eba6f28d34894e3a5710998818515104d6e74e5cc331ce31e46e626
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND "")

ExternalProject_Get_Property(external_eigen SOURCE_DIR)
add_library(libEigenHelper INTERFACE)
add_dependencies(libEigenHelper external_eigen)
target_include_directories(libEigenHelper SYSTEM INTERFACE $<BUILD_INTERFACE:${SOURCE_DIR}>)
set_property(TARGET libEigenHelper PROPERTY EXPORT_NAME Eigen3::Eigen)
add_library(Eigen3::Eigen ALIAS libEigenHelper)
