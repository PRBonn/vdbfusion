pybind11_add_module(vdbfusion_pybind vdbfusion_pybind.cpp)
target_compile_options(vdbfusion_pybind PRIVATE -Werror -Wall -Wextra)
target_link_libraries(vdbfusion_pybind PRIVATE VDBFusion::vdbfusion)

# PYOPENVDB_SUPPORT is defined only by the existence of the pyopenvdb library.
find_package(Python COMPONENTS Interpreter)
execute_process(COMMAND
                ${PYTHON_EXECUTABLE} "-c" "import pyopenvdb; print(True)"
                OUTPUT_VARIABLE PYOPENVDB_SUPPORT
                ERROR_QUIET)
if(PYOPENVDB_SUPPORT)
  find_package(Boost COMPONENTS python Required)
  target_compile_definitions(vdbfusion_pybind PRIVATE "$<$<COMPILE_LANGUAGE:CXX>:PYOPENVDB_SUPPORT>")
  target_link_libraries(vdbfusion_pybind PRIVATE Boost::python)
  message(STATUS "PYOPENVDB_SUPPORT enabled")
endif()
