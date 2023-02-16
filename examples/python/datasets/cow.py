#!/usr/bin/env python3
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

# Download the data from https://www.ipb.uni-bonn.de/html/software/vdbfusion/cow_and_lady_dataset.zip
# For more detailed information on this dataset please check https://github.com/PRBonn/vdbfusion_ros/issues/2

import glob
import os
import sys

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
        poses = np.loadtxt(filename, delimiter=" ")
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
