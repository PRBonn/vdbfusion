// MIT License
//
// # Copyright (c) 2022 Ignacio Vizzo, Cyrill Stachniss, University of Bonn
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// OpenVDB
#include <openvdb/openvdb.h>

// pybind11
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

// std stuff
#include <Eigen/Core>
#include <memory>
#include <vector>

#ifdef PYOPENVDB_SUPPORT
#include "pyopenvdb.h"
#endif

#include "stl_vector_eigen.h"
#include "vdbfusion/VDBVolume.h"

PYBIND11_MAKE_OPAQUE(std::vector<Eigen::Vector3d>);
PYBIND11_MAKE_OPAQUE(std::vector<Eigen::Vector3i>);

namespace py = pybind11;
using namespace py::literals;

namespace vdbfusion {

PYBIND11_MODULE(vdbfusion_pybind, m) {
    auto vector3dvector = pybind_eigen_vector_of_vector<Eigen::Vector3d>(
        m, "_VectorEigen3d", "std::vector<Eigen::Vector3d>",
        py::py_array_to_vectors_double<Eigen::Vector3d>);

    auto vector3ivector = pybind_eigen_vector_of_vector<Eigen::Vector3i>(
        m, "_VectorEigen3i", "std::vector<Eigen::Vector3i>",
        py::py_array_to_vectors_int<Eigen::Vector3i>);
    py::class_<VDBVolume, std::shared_ptr<VDBVolume>> vdb_volume(
        m, "_VDBVolume",
        "This is the low level C++ bindings, all the methods and "
        "constructor defined within this module (starting with a ``_`` "
        "should not be used. Please reffer to the python Procesor class to "
        "check how to use the API");
    vdb_volume
        .def(py::init<float, float, bool>(), "voxel_size"_a, "sdf_trunc"_a,
             "space_carving"_a = false)
        // TODO: add support for this
        .def("_integrate",
             py::overload_cast<const std::vector<Eigen::Vector3d>&, const Eigen::Matrix4d&,
                               const std::function<float(float)>&>(&VDBVolume::Integrate),
             "points"_a, "extrinsic"_a, "weighting_function"_a)
        .def("_integrate",
             py::overload_cast<const std::vector<Eigen::Vector3d>&, const Eigen::Vector3d&,
                               const std::function<float(float)>&>(&VDBVolume::Integrate),
             "points"_a, "origin"_a, "weighting_function"_a)
        .def(
            "_integrate",
            [](VDBVolume& self, const std::vector<Eigen::Vector3d>& points,
               const Eigen::Vector3d& origin) {
                self.Integrate(points, origin, [](float /*sdf*/) { return 1.0f; });
            },
            "points"_a, "origin"_a)
        .def(
            "_integrate",
            [](VDBVolume& self, const std::vector<Eigen::Vector3d>& points,
               const Eigen::Vector3d& origin, float weight) {
                self.Integrate(points, origin, [=](float /*sdf*/) { return weight; });
            },
            "points"_a, "origin"_a, "weight"_a)
        .def(
            "_integrate",
            [](VDBVolume& self, const std::vector<Eigen::Vector3d>& points,
               const Eigen::Matrix4d& extrinsics) {
                self.Integrate(points, extrinsics, [](float /*sdf*/) { return 1.0f; });
            },
            "points"_a, "extrinsic"_a)
        .def(
            "_integrate",
            [](VDBVolume& self, const std::vector<Eigen::Vector3d>& points,
               const Eigen::Matrix4d& extrinsics, float weight) {
                self.Integrate(points, extrinsics, [=](float /*sdf*/) { return weight; });
            },
            "points"_a, "origin"_a, "weight"_a)
#ifdef PYOPENVDB_SUPPORT
        .def("_integrate",
             py::overload_cast<openvdb::FloatGrid::Ptr, const std::function<float(float)>&>(
                 &VDBVolume::Integrate),
             "grid"_a, "weighting_function"_a)
        .def(
            "_integrate",
            [](VDBVolume& self, openvdb::FloatGrid::Ptr grid) {
                self.Integrate(grid, [](float /*sdf*/) { return 1.0f; });
            },
            "grid"_a)
        .def(
            "_integrate",
            [](VDBVolume& self, openvdb::FloatGrid::Ptr grid, float weight) {
                self.Integrate(grid, [=](float /*sdf*/) { return weight; });
            },
            "grid"_a, "weight"_a)
#endif
        .def(
            "_update_tsdf",
            [](VDBVolume& self, const float& sdf, std::vector<int>& ijk,
               const std::function<float(float)>& weighting_function) {
                self.UpdateTSDF(sdf, openvdb::Coord(ijk[0], ijk[1], ijk[2]), weighting_function);
            },
            "sdf"_a, "weighting_function"_a, "ijk"_a)
        .def(
            "_update_tsdf",
            // Same story here, we overload to skip the lambda passing between Python and C++
            [](VDBVolume& self, const float& sdf, std::vector<int>& ijk) {
                self.UpdateTSDF(sdf, openvdb::Coord(ijk[0], ijk[1], ijk[2]),
                                [](float /*sdf*/) { return 1.0f; });
            },
            "sdf"_a, "ijk"_a)
        .def("_extract_triangle_mesh", &VDBVolume::ExtractTriangleMesh, "fill_holes"_a,
             "min_weight"_a)
        .def(
            "_extract_vdb_grids",
            [](const VDBVolume& self, const std::string& filename) {
                openvdb::io::File(filename).write({self.tsdf_, self.weights_});
            },
            "filename"_a)
#ifndef PYOPENVDB_SUPPORT
        .def_property_readonly_static("PYOPENVDB_SUPPORT_ENABLED", [](py::object) { return false; })
#else
        .def_property_readonly_static("PYOPENVDB_SUPPORT_ENABLED", [](py::object) { return true; })
        .def("_prune", &VDBVolume::Prune, "min_weight"_a)
        .def_readwrite("_tsdf", &VDBVolume::tsdf_)
        .def_readwrite("_weights", &VDBVolume::weights_)
#endif
        .def_readwrite("_voxel_size", &VDBVolume::voxel_size_)
        .def_readwrite("_sdf_trunc", &VDBVolume::sdf_trunc_)
        .def_readwrite("_space_carving", &VDBVolume::space_carving_);
}

}  // namespace vdbfusion
