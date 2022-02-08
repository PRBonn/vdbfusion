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
