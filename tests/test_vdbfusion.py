"""Test the C++/Python bindings."""
import unittest

import numpy as np

from vdbfusion import VDBVolume


class VDBFusionTest(unittest.TestCase):
    """Not the best unit testing framework, but at least makes sure that the API is not entirely
    broken."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.voxel_size: float = 0.1
        self.sdf_trunc: float = 0.3
        self.space_carving: bool = False
        self.n_points = 100
        self.points = np.random.rand(self.n_points, 3)
        self.pose = np.eye(4)
        self.tsdf_volume = VDBVolume(self.voxel_size, self.sdf_trunc, self.space_carving)

    def test_integrate(self) -> None:
        """We can't check anything meaningful from the python side, so move on."""
        self.tsdf_volume.integrate(points=self.points, extrinsic=self.pose)

    def test_extract_triangle_mesh(self) -> None:
        self.tsdf_volume.integrate(points=self.points, extrinsic=self.pose)
        vertices, triangles = self.tsdf_volume.extract_triangle_mesh()
        # We can't see what's inside, but at least should be not None objects
        self.assertIsNotNone(vertices)
        self.assertIsNotNone(triangles)


if __name__ == "__main__":
    unittest.main()
