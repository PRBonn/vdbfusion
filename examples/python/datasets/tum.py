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


import numpy as np
import open3d as o3d
from pyquaternion import Quaternion
import sys

sys.path.append("..")
from utils.cache import get_cache, memoize


class TUMDataset:
    def __init__(self, data_source, get_color=False):
        # Cache
        self.use_cache = True
        self.cache = get_cache(directory="cache/tum/")

        self.get_color = get_color

        self.data_source = data_source
        self.depth_frames = np.loadtxt(fname=f"{self.data_source}/depth.txt", dtype=str)
        self.rgb_frames = np.loadtxt(fname=f"{self.data_source}/rgb.txt", dtype=str)

        self.matches = np.array(
            self.associate(
                self.rgb_frames[:, 0].astype(np.float64).tolist(),
                self.depth_frames[:, 0].astype(np.float64).tolist(),
            )
        )
        gt_list = np.loadtxt(fname=self.data_source + "/" + "groundtruth.txt", dtype=str)
        self.gt_poses = self.load_poses(gt_list)

    @memoize()
    def load_poses(self, gt_list):
        indices = np.abs(
            (
                np.subtract.outer(
                    gt_list[:, 0].astype(np.float64),
                    self.depth_frames[:, 0].astype(np.float64),
                )
            )
        ).argmin(0)

        xyz = gt_list[indices][:, 1:4]
        rotations = np.array(
            [
                Quaternion(x=x, y=y, z=z, w=w).rotation_matrix
                for x, y, z, w in gt_list[indices][:, 4:]
            ]
        )
        num_poses = rotations.shape[0]
        poses = np.eye(4, dtype=np.float64).reshape(1, 4, 4).repeat(num_poses, axis=0)
        poses[:, :3, :3] = rotations
        poses[:, :3, 3] = xyz
        return poses

    @memoize()
    def associate(self, first_list, second_list, offset=0.0, max_difference=0.2):
        matches = []
        for a in first_list:
            temp_matches = [
                (abs(a - (b + offset)), a, b)
                for b in second_list
                if abs(a - (b + offset)) < max_difference
            ]
            if len(temp_matches) != 0:
                diff, first, second = temp_matches[np.argmin(np.array(temp_matches)[:, 0])]
                matches.append((first, second))
        matches.sort()
        return matches

    @memoize()
    def __getitem__(self, idx):
        rgb_id, depth_id = self.matches[idx]
        pose = self.gt_poses[idx]

        rgb = o3d.io.read_image(f"{self.data_source}/rgb/{rgb_id:.6f}.png")
        depth = o3d.io.read_image(f"{self.data_source}/depth/{depth_id:.6f}.png")
        rgbd_image = o3d.geometry.RGBDImage.create_from_tum_format(rgb, depth)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image,
            o3d.camera.PinholeCameraIntrinsic(
                o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault
            ),
        )

        xyz = np.array(pcd.points)
        colors = np.array(pcd.colors)

        if self.get_color:
            return xyz, colors, pose
        return xyz, pose

    def __len__(self):
        return len(self.matches)
