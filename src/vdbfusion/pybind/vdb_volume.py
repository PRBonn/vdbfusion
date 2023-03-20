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

from typing import Any, Optional, Tuple, Callable, overload

import numpy as np

from . import vdbfusion_pybind


class VDBVolume:
    def __init__(
        self,
        voxel_size: float,
        sdf_trunc: float,
        space_carving: bool = False,
    ):
        self._volume = vdbfusion_pybind._VDBVolume(
            voxel_size=np.float32(voxel_size),
            sdf_trunc=np.float32(sdf_trunc),
            space_carving=space_carving,
        )
        # Passthrough all data members from the C++ API
        self.voxel_size = self._volume._voxel_size
        self.sdf_trunc = self._volume._sdf_trunc
        self.space_carving = self._volume._space_carving
        self.pyopenvdb_support_enabled = self._volume.PYOPENVDB_SUPPORT_ENABLED
        if self.pyopenvdb_support_enabled:
            self.tsdf = self._volume._tsdf
            self.weights = self._volume._weights

    def __repr__(self) -> str:
        return (
            f"VDBVolume with:\n"
            f"voxel_size    = {self.voxel_size}\n"
            f"sdf_trunc     = {self.sdf_trunc}\n"
            f"space_carving = {self.space_carving}\n"
        )

    @overload
    def integrate(
        self,
        points: np.ndarray,
        extrinsic: np.ndarray,
        weighting_function: Callable[[float], float],
    ) -> None:
        ...

    @overload
    def integrate(self, grid, weighting_function: Callable[[float], float]) -> None:
        ...

    @overload
    def integrate(self, grid, weight: float) -> None:
        ...

    @overload
    def integrate(self, grid) -> None:
        ...

    def integrate(
        self,
        points: Optional[np.ndarray] = None,
        colors: Optional[np.ndarray] = None,
        extrinsic: Optional[np.ndarray] = None,
        grid: Optional[Any] = None,
        weight: Optional[float] = None,
        weighting_function: Optional[Callable[[float], float]] = None,
    ) -> None:

        assert (weighting_function is None) or (weight is None), (
            "Not both weighting_function and weight can be defined"
        )
        if grid is not None:
            if not self.pyopenvdb_support_enabled:
                raise NotImplementedError("Please compile with PYOPENVDB_SUPPORT_ENABLED")
            if weighting_function is not None:
                return self._volume._integrate(grid, weighting_function)
            if weight is not None:
                return self._volume._integrate(grid, weight)
            return self._volume._integrate(grid)
        else:
            assert isinstance(points, np.ndarray), "points must by np.ndarray(n, 3)"
            assert points.dtype == np.float64, "points dtype must be np.float64"
            assert isinstance(extrinsic, np.ndarray), "origin/extrinsic must by np.ndarray"
            assert extrinsic.dtype == np.float64, "origin/extrinsic dtype must be np.float64"
            if extrinsic.shape == (4, 4):
                extrinsic = extrinsic[:3, 3].copy()
            assert extrinsic.shape in [
                (3,),
                (3, 1),
            ], "origin/extrinsic must be a (3,) array or a (4,4) matrix"

            _points = vdbfusion_pybind._VectorEigen3d(points)
            if colors is not None:
                _colors = vdbfusion_pybind._VectorEigen3d(colors)
            else:
                _colors = vdbfusion_pybind._VectorEigen3d(np.zeros((0, 3), dtype=np.float64))
            if weighting_function is not None:
                return self._volume._integrate(_points, _colors, extrinsic, weighting_function)
            if weight is not None:
                return self._volume._integrate(_points, _colors, extrinsic, weight)
            return self._volume._integrate(_points, _colors, extrinsic)

    @overload
    def update_tsdf(
        self, sdf: float, ijk: np.ndarray, weighting_function: Optional[Callable[[float], float]]
    ) -> None:
        ...

    @overload
    def update_tsdf(self, sdf: float, ijk: np.ndarray) -> None:
        ...

    def update_tsdf(
        self,
        sdf: float,
        ijk: np.ndarray,
        weighting_function: Optional[Callable[[float], float]] = None,
    ) -> None:
        if weighting_function is not None:
            return self._volume._update_tsdf(sdf, ijk, weighting_function)
        return self._volume._update_tsdf(sdf, ijk)

    def extract_triangle_mesh(self, fill_holes: bool = True, min_weight: float = 0.0) -> Tuple:
        """Returns a the vertices and triangles representing the constructed the TriangleMesh.

        If you can afford to use Open3D as dependency just pass the output of this function to the
        TriangleMesh constructor from Open3d.

        vertices, triangles = integrator.extract_triangle_mesh()
        mesh = o3d.geometry.TriangleMesh(
            o3d.utility.Vector3dVector(vertices),
            o3d.utility.Vector3iVector(triangles),
        )
        mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
        """
        vertices, triangles, colors = self._volume._extract_triangle_mesh(fill_holes, min_weight)
        return np.asarray(vertices), np.asarray(triangles), np.asarray(colors)

    def write_vdb_grids(self, out_file: str) -> None:
        """Write the internal map representation to a file.

        Contains both D(x) and W(x) grids.
        """
        self._volume._write_vdb_grids(out_file)

    def prune(self, min_weight: float):
        """Use the W(x) weights grid to cleanup the generated signed distance field according to a
        minimum weight threshold.

        This function is ideal to cleanup the TSDF grid:D(x) before exporting it.
        """
        return self._volume._prune(min_weight)

    def get_tsdf(self):
        """Return the tsdf as a numpy array.

        Note: For large volumes this might not fit in memory.
        """
        return self._volume.get_tsdf()

    def get_weights(self):
        """Return the weights as a numpy array.

        Note: For large volumes this might not fit in memory.
        """
        return self._volume.get_weights()