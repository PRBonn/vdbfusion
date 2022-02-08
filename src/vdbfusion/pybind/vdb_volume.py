from typing import Any, Callable, Tuple

import numpy as np

from . import vdbfusion_pybind


class VDBVolume:
    """Wrapper class around the low level C++ python bindings.

    TODO: Complete class documentation and check for data types
    """

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
        # If PYOPENVDB_SUPPORT has been enabled then we can acccess those attributes
        if hasattr(self._volume, "_tsdf") and hasattr(self._volume, "_weights"):
            self.tsdf = self._volume._tsdf
            self.weights = self._volume._weights

    def __repr__(self) -> str:
        return (
            f"VDBVolume with:\n"
            f"voxel_size    = {self.voxel_size}\n"
            f"sdf_trunc     = {self.sdf_trunc}\n"
            f"space_carving = {self.space_carving}\n"
        )

    def integrate(
        self,
        points: np.ndarray,
        extrinsic: np.ndarray,  # or origin
        weighting_function: Callable[[float], float] = None,
    ) -> None:
        """Explain here how to use the function.

        TODO: Add tag dispatching for the `origin` case
        """
        assert isinstance(points, np.ndarray), "points must by np.ndarray(n, 3)"
        assert points.dtype == np.float64, "points dtype must be np.float64"
        assert isinstance(extrinsic, np.ndarray), "origin/extrinsic must by np.ndarray"
        assert extrinsic.dtype == np.float64, "origin/extrinsic dtype must be np.float64"
        assert extrinsic.shape in [
            (3,),
            (3, 1),
            (4, 4),
        ], "origin/extrinsic must be a (3,) array or a (4,4) matrix"
        # TODO: Fix this logic with singledispatchmethod
        if weighting_function:
            self._volume._integrate(
                vdbfusion_pybind._VectorEigen3d(points), extrinsic, weighting_function
            )
        self._volume._integrate(vdbfusion_pybind._VectorEigen3d(points), extrinsic)

    def update_tsdf(
        self,
        sdf: float,
        ijk: np.ndarray,
        weighting_function: Callable[[float], float] = None,
    ) -> None:
        assert isinstance(ijk, np.ndarray), "ijk must by np.ndarray(3,)"
        assert ijk.dtype == np.int32, "ijk dtype must be np.int32"
        if weighting_function:
            self._volume._update_tsdf(sdf, ijk, weighting_function)
        self._volume._update_tsdf(sdf, ijk)

    def extract_triangle_mesh(
        self,
        fill_holes: bool = True,
        min_weight: float = 1.0,
    ) -> Tuple[np.ndarray, np.ndarray]:
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
