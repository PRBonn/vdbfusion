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

# Original implementation by Tiziano Guadagnino
import os
import re
import sys
import csv
from typing import Dict

import numpy as np
import open3d as o3d


sys.path.append("..")
from utils.cache import get_cache, memoize
from utils.config import load_config


def vec2transform(v):
    """Convert a pose from 7D vector format ( x y z qx qy qz qw) to transformation matrix form
    Args: v pose in 7D vector format
    Returns:
        T 4x4 transformation matrix

    $ rosrun tf tf_echo base os_lidar
      - Translation: [-0.084, -0.025, 0.050]
      - Rotation: in Quaternion [0.000, 0.000, 0.924, 0.383]
                  in RPY (radian) [0.000, -0.000, 2.356]
                  in RPY (degree) [0.000, -0.000, 135.000]

    Random quaternion sent by nived in Mattermost

      -Rotation: q_BL: [ 0.0122965, -0.002454, 0.9226886, 0.385342]

    """
    T_cam_to_os = np.eye(4)
    T_cam_to_os[:3, -1] = np.array([-0.084, -0.025, 0.050])
    T_cam_to_os[:3, :3] = o3d.geometry.Geometry3D.get_rotation_matrix_from_quaternion(
        np.array([0.383, 0.000, 0.000, 0.924])
    )

    T_os_to_cam = np.linalg.inv(T_cam_to_os)
    T = np.eye(4)
    T[:3, -1] = v[:3]
    T[:3, :3] = o3d.geometry.Geometry3D.get_rotation_matrix_from_quaternion(
        np.array([v[6], v[3], v[4], v[5]])
    )
    return T_os_to_cam @ T @ T_cam_to_os


def get_pcd_filenames(scans_folder):
    # cloud_1583836591_182590976.pcd
    regex = re.compile("^cloud_(\d*_\d*)")

    def get_cloud_timestamp(pcd_filename):
        m = regex.search(pcd_filename)
        secs, nsecs = m.groups()[0].split("_")
        return int(secs) * int(1e9) + int(nsecs)

    return sorted(os.listdir(scans_folder), key=get_cloud_timestamp)


def load_gt_poses(poses_csv):
    poses = []
    with open(poses_csv, "r") as f:
        reader = csv.reader(f, delimiter=",")
        # skip the first line, the lazy way
        reader.__next__()
        for l in reader:
            # a pose have format x y z qx qy qz qw
            pose = np.array([float(x) for x in l[2:]])
            poses.append(vec2transform(pose))
    return poses


class NewerCollegeDataset:
    def __init__(self, data_source, config_file: str):
        self.config = load_config(config_file)
        self.data_source = os.path.join(data_source, "")
        self.scan_folder = os.path.join(self.data_source, "raw_format/ouster_scan")
        self.pose_file = os.path.join(self.data_source, "ground_truth/registered_poses.csv")

        # Load scan files and poses
        self.scan_files = get_pcd_filenames(self.scan_folder)
        self.poses = load_gt_poses(self.pose_file)

        # Cache
        self.use_cache = True
        self.cache = get_cache(directory="cache/newer_college")

    def __len__(self):
        return len(self.scan_files)

    def __getitem__(self, idx):
        file_path = os.path.join(self.scan_folder, self.scan_files[idx])
        return self.getitem(file_path, idx, self.config)

    @memoize()
    def getitem(self, file_path: str, idx: int, config: Dict):
        pose = self.poses[idx]
        scan = o3d.io.read_point_cloud(file_path)

        points = np.asarray(scan.points)
        points = points[np.linalg.norm(points, axis=1) <= config.max_range]
        points = points[np.linalg.norm(points, axis=1) >= config.min_range]

        scan = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        scan.transform(pose) if config.apply_pose else None
        return np.asarray(scan.points), pose
