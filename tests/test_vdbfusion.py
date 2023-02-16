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
        """We can't check nothing meaningfull from the python side, so move on."""
        self.tsdf_volume.integrate(points=self.points, extrinsic=self.pose)

    def test_extract_tirangle_mesh(self) -> None:
        self.tsdf_volume.integrate(points=self.points, extrinsic=self.pose)
        vertices, triangles = self.tsdf_volume.extract_triangle_mesh()
        # We can't see what's inside, but at least should be not None objects
        self.assertIsNotNone(vertices)
        self.assertIsNotNone(triangles)


if __name__ == "__main__":
    unittest.main()
