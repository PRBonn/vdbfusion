/*
 * MIT License
 *
 * # Copyright (c) 2022 Ignacio Vizzo, Cyrill Stachniss, University of Bonn
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include <openvdb/openvdb.h>

#include <boost/python.hpp>

// pollute namespace with py
#include <pybind11/pybind11.h>
namespace py = pybind11;

namespace pybind11::detail {
/// TODO: Try to remove boost::python dependency
template <>
struct type_caster<openvdb::FloatGrid::Ptr> {
public:
    /// Converts Python to C++
    bool load(handle src, bool /*unused*/) {
        PyObject* source = src.ptr();
        boost::python::extract<typename openvdb::FloatGrid::Ptr> grid_ptr(source);
        if (!grid_ptr.check()) return false;
        value = grid_ptr();
        return (value && (PyErr_Occurred() == nullptr));
    }

    /// Converts from C++ to Python
    static handle cast(openvdb::FloatGrid::Ptr src, return_value_policy, handle) {
        if (!src) return none().inc_ref();
        py::module::import("pyopenvdb").attr("FloatGrid");
        boost::python::object obj = boost::python::object(src);
        auto out = reinterpret_borrow<py::object>(obj.ptr());
        return out.release();
    }

    PYBIND11_TYPE_CASTER(openvdb::FloatGrid::Ptr, _("pyopenvdb.FloatGrid"));
};
}  // namespace pybind11::detail
