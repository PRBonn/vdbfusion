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

import os

import open3d as o3d
from tqdm import trange

from utils import load_config


class PointCloudPipeline:
    """Same as VDB but jsut aggregate points."""

    def __init__(self, dataset, config_file: str, map_name: str, jump: int = 0, n_scans: int = -1):
        self._dataset = dataset
        self._config = load_config(config_file)
        self._n_scans = len(dataset) if n_scans == -1 else n_scans
        self._jump = jump
        self._map_name = f"{map_name}_pcd_{self._n_scans}_scans"
        self._cloud_map = o3d.geometry.PointCloud()

    def run(self):
        self._run_pipeline()
        self._write_ply()

    def visualize(self):
        o3d.visualization.draw_geometries([self._cloud_map])

    def __len__(self):
        return len(self._dataset)

    def _run_pipeline(self):
        for idx in trange(self._jump, self._jump + self._n_scans, unit=" frames"):
            scan, _ = self._dataset[idx]
            self._cloud_map += o3d.geometry.PointCloud(o3d.utility.Vector3dVector(scan))

    def _write_ply(self):
        os.makedirs(self._config.out_dir, exist_ok=True)
        filename = os.path.join(self._config.out_dir, self._map_name) + ".ply"
        o3d.io.write_point_cloud(filename, self._cloud_map)
