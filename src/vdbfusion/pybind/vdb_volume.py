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
    def integrate(self, points: np.ndarray, extrinsic: np.ndarray, weight: float) -> None:
        ...

    @overload
    def integrate(self, points: np.ndarray, extrinsic: np.ndarray) -> None:
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
        extrinsic: Optional[np.ndarray] = None,
        grid: Optional[Any] = None,
        weight: Optional[float] = None,
        weighting_function: Optional[Callable[[float], float]] = None,
    ) -> None:
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
            assert extrinsic.shape in [
                (3,),
                (3, 1),
                (4, 4),
            ], "origin/extrinsic must be a (3,) array or a (4,4) matrix"

            _points = vdbfusion_pybind._VectorEigen3d(points)
            if weighting_function is not None:
                return self._volume._integrate(_points, extrinsic, weighting_function)
            if weight is not None:
                return self._volume._integrate(_points, extrinsic, weight)
            self._volume._integrate(_points, extrinsic)

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
        """
        vertices, triangles = self._volume._extract_triangle_mesh(fill_holes, min_weight)
        return np.asarray(vertices), np.asarray(triangles)

    def extract_vdb_grids(self, out_file: str) -> None:
        """For now, write the internal map representation to a file.

        Contains both D(x) and W(x) grids.
        """
        self._volume._extract_vdb_grids(out_file)

    def prune(self, min_weight: float):
        """Use the W(x) weights grid to cleanup the generated signed distance field according to a
        minimum weight threshold.

        This function is ideal to cleanup the TSDF grid:D(x) before exporting it.
        """
        return self._volume._prune(min_weight)
