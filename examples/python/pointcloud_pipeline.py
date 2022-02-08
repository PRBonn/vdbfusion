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
