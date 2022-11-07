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

from functools import reduce
import os
import sys
import time

import numpy as np
import open3d as o3d
from tqdm import trange

from utils import load_config, write_config
from vdbfusion import VDBVolume


class VDBFusionPipeline:
    """Abstract class that defines a Pipeline, derived classes must implement the dataset and config
    properties."""

    def __init__(self, dataset, config_file: str, map_name: str, jump: int = 0, n_scans: int = -1):
        self._dataset = dataset
        self._config = load_config(config_file)
        self._n_scans = len(dataset) if n_scans == -1 else n_scans
        self._jump = jump
        self._map_name = f"{map_name}_{self._n_scans}_scans"
        self._tsdf_volume = VDBVolume(
            self._config.voxel_size,
            self._config.sdf_trunc,
            self._config.space_carving,
        )
        self._res = {}

    def run(self):
        self._run_tsdf_pipeline()
        self._write_ply()
        self._write_cfg()
        self._write_vdb()
        self._print_tim()
        self._print_metrics()

    def visualize(self):
        o3d.visualization.draw_geometries([self._res["mesh"]])

    def __len__(self):
        return len(self._dataset)

    def _run_tsdf_pipeline(self):
        times = []
        for idx in trange(self._jump, self._jump + self._n_scans, unit=" frames"):
            scan, pose = self._dataset[idx]
            tic = time.perf_counter_ns()
            self._tsdf_volume.integrate(scan, pose)
            toc = time.perf_counter_ns()
            times.append(toc - tic)
        self._res = {"mesh": self._get_o3d_mesh(self._tsdf_volume, self._config), "times": times}

    def _write_vdb(self):
        os.makedirs(self._config.out_dir, exist_ok=True)
        filename = os.path.join(self._config.out_dir, self._map_name) + ".vdb"
        self._tsdf_volume.extract_vdb_grids(filename)

    def _write_ply(self):
        os.makedirs(self._config.out_dir, exist_ok=True)
        filename = os.path.join(self._config.out_dir, self._map_name) + ".ply"
        o3d.io.write_triangle_mesh(filename, self._res["mesh"])

    def _write_cfg(self):
        os.makedirs(self._config.out_dir, exist_ok=True)
        filename = os.path.join(self._config.out_dir, self._map_name) + ".yml"
        write_config(dict(self._config), filename)

    def _print_tim(self):
        total_time_ns = reduce(lambda a, b: a + b, self._res["times"])
        total_time = total_time_ns * 1e-9
        total_scans = self._n_scans - self._jump
        self.fps = float(total_scans / total_time)

    @staticmethod
    def _get_o3d_mesh(tsdf_volume, cfg):
        vertices, triangles = tsdf_volume.extract_triangle_mesh(cfg.fill_holes, cfg.min_weight)
        mesh = o3d.geometry.TriangleMesh(
            o3d.utility.Vector3dVector(vertices),
            o3d.utility.Vector3iVector(triangles),
        )
        mesh.compute_vertex_normals()
        return mesh

    def _print_metrics(self):
        # If PYOPENVDB_SUPPORT has not been enabled then we can't report any metrics
        if not self._tsdf_volume.pyopenvdb_support_enabled:
            print("No metrics available, please compile with PYOPENVDB_SUPPORT")
            return

        # Compute the dimensions of the volume mapped
        grid = self._tsdf_volume.tsdf
        bbox = grid.evalActiveVoxelBoundingBox()
        dim = np.abs(np.asarray(bbox[1]) - np.asarray(bbox[0]))
        volume_extent = np.ceil(self._config.voxel_size * dim).astype(np.int32)
        volume_extent = f"{volume_extent[0]} x {volume_extent[1]} x {volume_extent[2]}"

        # Compute memory footprint
        total_voxels = int(np.prod(dim))
        float_size = 4
        # Always 2 grids
        mem_footprint = 2 * grid.memUsage() / (1024 * 1024)
        dense_equivalent = 2 * (float_size * total_voxels) / (1024 * 1024 * 1024)  # GB

        # compute size of .vdb file
        filename = os.path.join(self._config.out_dir, self._map_name) + ".vdb"
        file_size = float(os.stat(filename).st_size) / (1024 * 1024)

        # print metrics
        trunc_voxels = int(np.ceil(self._config.sdf_trunc / self._config.voxel_size))

        filename = os.path.join(self._config.out_dir, self._map_name) + ".txt"
        with open(filename, "w") as f:
            stdout = sys.stdout
            sys.stdout = f  # Change the standard output to the file we created.
            print(f"--------------------------------------------------")
            print(f"Results for dataset {self._map_name}:")
            print(f"--------------------------------------------------")
            print(f"voxel size       = {self._config.voxel_size} [m]")
            print(f"truncation       = {trunc_voxels} [voxels]")
            print(f"space carving    = {self._config.space_carving}")
            print(f"Avg FPS          = {self.fps:.2f} [Hz]")
            print(f"--------------------------------------------------")
            print(f"volume extent    = {volume_extent} [m x m x m]")
            print(f"memory footprint = {mem_footprint:.2f} [MB]")
            print(f"dense equivalent = {dense_equivalent:.2f} [GB]")
            print(f"size on disk     = {file_size:.2f} [MB]")
            print(f"--------------------------------------------------")
            print(f"number of scans  = {len(self)}")
            print(f"points per scan  = {len(self._dataset[0][0])}")
            print(f"min range        = {self._config.min_range} [m]")
            print(f"max range        = {self._config.max_range} [m]")
            print(f"--------------------------------------------------")
            sys.stdout = stdout

        # Print it
        os.system(f"cat {filename}")
