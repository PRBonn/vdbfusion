#!/usr/bin/env python3
# Original implementation by Federico Magistri
import glob
import os
import sys
from typing import Dict

import numpy as np
import open3d as o3d

sys.path.append("..")
from utils.cache import get_cache, memoize
from utils.config import load_config


class CowDataset:
    def __init__(
        self,
        data_source,
        config_file: str,
        get_color: bool = False,
        apply_pose: bool = True,
    ):
        # Cache
        self.use_cache = True
        self.cache = get_cache(directory="cache/cow/")
        self.get_color = get_color
        self.apply_pose = apply_pose
        self.config = load_config(config_file)

        self.data_source = os.path.join(data_source, "")
        self.gt_list = self.read_gt_list(os.path.join(self.data_source, "poses.txt"))
        self.cloud_files = sorted(glob.glob(self.data_source + "*.ply"))

    @staticmethod
    def read_gt_list(filename):
        poses = np.loadtxt(filename, delimiter=",")
        return poses.reshape((len(poses), 4, 4))

    def __getitem__(self, idx):
        return self.getitem(idx, self.config)

    @memoize()
    def getitem(self, idx, config):
        pose = self.gt_list[idx]
        pcd = o3d.io.read_point_cloud(self.cloud_files[idx])
        pcd.transform(pose) if self.apply_pose else None
        colors = np.asarray(pcd.colors)
        points = np.asarray(pcd.points)
        points = points[np.linalg.norm(points, axis=1) <= config.max_range]
        points = points[np.linalg.norm(points, axis=1) >= config.min_range]
        if self.get_color:
            return points, colors, pose
        return points, pose

    def __len__(self):
        return len(self.cloud_files)
