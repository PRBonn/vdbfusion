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
import glob
import importlib
import os
import sys
from typing import List

import numpy as np
from trimesh import transform_points

sys.path.append("..")
from utils.cache import get_cache, memoize
from utils.config import load_config


class NuScenesDataset:
    def __init__(
        self,
        nusc_root_dir: str,
        sequence: int,
        config_file: str,
        *_,
        **__,
    ):
        try:
            importlib.import_module("nuscenes")
        except ModuleNotFoundError:
            print("nuscenes-devkit is not installed on your system")
            print('run "pip install nuscenes-devkit"')
            sys.exit(1)

        # TODO: If someone needs more splits from nuScenes expose this 2 parameters
        #  nusc_version: str = "v1.0-trainval"
        #  split: str = "train"
        nusc_version: str = "v1.0-mini"
        split: str = "mini_train"
        self.lidar_name: str = "LIDAR_TOP"

        # Lazy loading
        from nuscenes.nuscenes import NuScenes
        from nuscenes.utils.splits import create_splits_logs

        # Config stuff
        self.sequence_id = str(int(sequence)).zfill(4)
        self.config = load_config(config_file)

        self.nusc = NuScenes(dataroot=str(nusc_root_dir), version=nusc_version)
        self.scene_name = f"scene-{self.sequence_id}"
        if self.scene_name not in [s["name"] for s in self.nusc.scene]:
            print(f'[ERROR] Sequence "{self.sequence_id}" not available scenes')
            print("\nAvailable scenes:")
            self.nusc.list_scenes()
            sys.exit(1)

        # Load nuScenes read from file inside dataloader module
        self.load_point_cloud = importlib.import_module(
            "nuscenes.utils.data_classes"
        ).LidarPointCloud.from_file

        # Get assignment of scenes to splits.
        split_logs = create_splits_logs(split, self.nusc)

        # Use only the samples from the current split.
        scene_token = self._get_scene_token(split_logs)
        self.lidar_tokens = self._get_lidar_tokens(scene_token)
        self.poses = self._load_poses()

        # Cache
        self.use_cache = True
        self.cache = get_cache(directory="cache/nuscenes/")

    def __len__(self):
        return len(self.lidar_tokens)

    def __getitem__(self, idx):
        return self.read_point_cloud(self.lidar_tokens[idx], idx, self.config), self.poses[
            idx
        ].astype(np.float64)

    @memoize()
    def read_point_cloud(self, token: str, idx: int, config):
        filename = self.nusc.get("sample_data", token)["filename"]
        pcl = self.load_point_cloud(os.path.join(self.nusc.dataroot, filename))
        points = pcl.points.T[:, :3]
        points = points[np.linalg.norm(points, axis=1) <= config.max_range]
        points = points[np.linalg.norm(points, axis=1) >= config.min_range]
        points = transform_points(points, self.poses[idx]) if config.apply_pose else None
        return points.astype(np.float64)

    def _load_poses(self) -> np.ndarray:
        from nuscenes.utils.geometry_utils import transform_matrix
        from pyquaternion import Quaternion

        poses = np.empty((len(self), 4, 4), dtype=np.float32)
        for i, lidar_token in enumerate(self.lidar_tokens):
            sd_record_lid = self.nusc.get("sample_data", lidar_token)
            cs_record_lid = self.nusc.get(
                "calibrated_sensor", sd_record_lid["calibrated_sensor_token"]
            )
            ep_record_lid = self.nusc.get("ego_pose", sd_record_lid["ego_pose_token"])

            car_to_velo = transform_matrix(
                cs_record_lid["translation"],
                Quaternion(cs_record_lid["rotation"]),
            )
            pose_car = transform_matrix(
                ep_record_lid["translation"],
                Quaternion(ep_record_lid["rotation"]),
            )

            poses[i:, :] = pose_car @ car_to_velo

        # Convert from global coordinate poses to local poses
        first_pose = poses[0, :, :]
        poses = np.linalg.inv(first_pose) @ poses
        return poses

    def _get_scene_token(self, split_logs: List[str]) -> str:
        """
        Convenience function to get the samples in a particular split.
        :param split_logs: A list of the log names in this split.
        :return: The list of samples.
        """
        scene_tokens = [s["token"] for s in self.nusc.scene if s["name"] == self.scene_name][0]
        scene = self.nusc.get("scene", scene_tokens)
        log = self.nusc.get("log", scene["log_token"])
        return scene["token"] if log["logfile"] in split_logs else ""

    def _get_lidar_tokens(self, scene_token: str) -> List[str]:
        scene_rec = self.nusc.get("scene", scene_token)

        # Get records from DB.
        scene_rec = self.nusc.get("scene", scene_token)
        start_sample_rec = self.nusc.get("sample", scene_rec["first_sample_token"])
        sd_rec = self.nusc.get("sample_data", start_sample_rec["data"][self.lidar_name])

        # Make list of frames
        cur_sd_rec = sd_rec
        sd_tokens = []
        while cur_sd_rec["next"] != "":
            cur_sd_rec = self.nusc.get("sample_data", cur_sd_rec["next"])
            sd_tokens.append(cur_sd_rec["token"])
        return sd_tokens


class NUSCENESDataset:
    def __init__(self, nusc_root_dir: str, sequence: int, config_file: str):
        """Simple NUSCENES DataLoader to provide a ready-to-run example.

        Heavily inspired in PyLidar SLAM
        """
        # Config stuff
        self.sequence = str(int(sequence)).zfill(4)
        self.config = load_config(config_file)
        self.nusc_sequence_dir = os.path.join(nusc_root_dir, self.sequence)
        self.velodyne_dir = os.path.join(self.nusc_sequence_dir, "velodyne/")

        # Read stuff
        self.calibration = self.read_calib_file(os.path.join(self.nusc_sequence_dir, "calib.txt"))
        self.poses = self.load_poses(os.path.join(self.nusc_sequence_dir, "poses.txt"))
        self.scan_files = sorted(glob.glob(self.velodyne_dir + "*.bin"))

    def __len__(self):
        return len(self.scan_files)

    def scans(self, idx):
        return self.read_point_cloud(idx, self.scan_files, self.config)

    @memoize()
    def read_point_cloud(self, idx: int, scan_files: list, config: dict):
        file_path = scan_files[idx]
        points = np.fromfile(file_path, dtype=np.float32).reshape((-1, 4))[:, :3]
        points = points[np.linalg.norm(points, axis=1) <= config.max_range]
        points = points[np.linalg.norm(points, axis=1) >= config.min_range]
        points = transform_points(points, self.poses[idx]) if config.apply_pose else None
        return points
