# MIT License
#
# # Copyright (c) 2022 Ignacio Vizzo, Cyrill Stachniss, University of Bonn
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

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
  find_package(Boost COMPONENTS python REQUIRED)
  target_compile_definitions(vdbfusion_pybind PRIVATE "$<$<COMPILE_LANGUAGE:CXX>:PYOPENVDB_SUPPORT>")
  target_link_libraries(vdbfusion_pybind PRIVATE Boost::python)
  message(STATUS "PYOPENVDB_SUPPORT enabled")
endif()
