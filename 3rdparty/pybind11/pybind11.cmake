include(FetchContent)
FetchContent_Declare(
  ext_pybind11
  PREFIX pybind11
  #TODO: Update to a release when this gets merged: https://github.com/pybind/pybind11/pull/3743
  GIT_REPOSITORY https://github.com/pybind/pybind11
  GIT_TAG master
  GIT_SHALLOW ON)

FetchContent_MakeAvailable(ext_pybind11)
