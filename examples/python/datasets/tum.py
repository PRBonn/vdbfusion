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

# Original implementation by Federico Magistri

import os

import cv2 as cv
import numpy as np
import open3d as o3d

import sys

sys.path.append("..")
from utils.cache import get_cache, memoize


class TUMDataset:
    def __init__(self, data_source, get_color=False):
        # Cache
        self.use_cache = True
        self.cache = get_cache(directory="cache/tum/")

        self.offset = 0.0
        self.max_difference = 0.2
        self.get_color = get_color

        self.data_source = data_source

        self.rgb_txt = os.path.join(self.data_source, "rgb.txt")
        self.depth_txt = os.path.join(self.data_source, "depth.txt")
        self.pose_txt = os.path.join(self.data_source, "groundtruth.txt")

        self.gt_list = self.read_gt_list(self.pose_txt)
        self.rgb_list = self.read_file_list(self.rgb_txt)
        self.depth_list = self.read_file_list(self.depth_txt)
        self.matches = self.associate(self.rgb_list, self.depth_list)

    @memoize()
    def associate(self, first_keys, second_keys):
        """Associate two dictionaries of (stamp,data). As the time stamps never match exactly, we
        aim to find the closest match for every input tuple.

        Input:
        first_list -- first dictionary of (stamp,data) tuples
        second_list -- second dictionary of (stamp,data) tuples
        offset -- time offset between both dictionaries (e.g., to model the delay between the sensors)
        max_difference -- search radius for candidate generation

        Output:
        matches -- list of matched tuples ((stamp1,data1),(stamp2,data2))
        """
        potential_matches = [
            (abs(a - (b + self.offset)), a, b)
            for a in first_keys.keys()
            for a in first_keys.keys()
            for b in second_keys.keys()
            if abs(a - (b + self.offset)) < self.max_difference
        ]
        potential_matches.sort()
        matches = []
        for diff, a, b in potential_matches:
            if a in first_keys and b in second_keys:
                first_keys.pop(a)
                second_keys.pop(b)
                matches.append((a, b))

        matches.sort()
        return matches

    def find_closest_ts(self, ts):
        times = list(self.gt_list.keys())
        diff = np.abs(np.array(times) - ts)
        idx = diff.argmin()
        return times[idx]

    @memoize()
    def read_file_list(self, filename):
        """Reads a trajectory from a text file.

        File format:
        The file format is "stamp d1 d2 d3 ...", where stamp denotes the time stamp (to be matched)
        and "d1 d2 d3.." is arbitary data (e.g., a 3D position and 3D orientation) associated to this timestamp.

        Input:
        filename -- File name

        Output:
        dict -- dictionary of (stamp,data) tuples
        """
        file = open(filename)
        data = file.read()
        lines = data.replace(",", " ").replace("\t", " ").split("\n")
        # lines = lines[:100]
        list = [
            [v.strip() for v in line.split(" ") if v.strip() != ""]
            for line in lines
            if len(line) > 0 and line[0] != "#"
        ]
        list = [(float(l[0]), l[1:]) for l in list if len(l) > 1]
        return dict(list)

    @memoize()
    def read_gt_list(self, filename):
        """Reads a trajectory from a text file.

        File format:
        The file format is "stamp d1 d2 d3 ...", where stamp denotes the time stamp (to be matched)
        and "d1 d2 d3.." is arbitary data (e.g., a 3D position and 3D orientation) associated to this timestamp.

        Input:
        filename -- File name

        Output:
        dict -- dictionary of (stamp,data) tuples
        """
        file = open(filename)
        data = file.read()
        lines = data.replace(",", " ").replace("\t", " ").split("\n")
        # lines = lines[:100]
        list = [
            [v.strip() for v in line.split(" ") if v.strip() != ""]
            for line in lines
            if len(line) > 0 and line[0] != "#"
        ]
        list = [(float(l[0]), l[1:]) for l in list if len(l) > 1]
        return dict(list)

    @staticmethod
    def conver_to_homo(v):
        x, y, z = v[0], v[1], v[2]
        qx, qy, qz, qw = v[3], v[4], v[5], v[6]
        T = np.eye(4)
        T[:3, -1] = np.array([x, y, z])
        T[:3, :3] = o3d.geometry.Geometry3D.get_rotation_matrix_from_quaternion(
            np.array([qw, qx, qy, qz])
        )
        return T

    @memoize()
    def __getitem__(self, idx):
        rgb_id, depth_id = self.matches[idx]

        pose_timestamp = self.find_closest_ts(depth_id)
        pose = self.conver_to_homo(self.gt_list[pose_timestamp])
        pose_inv = np.linalg.inv(pose)

        bgr = cv.imread(os.path.join(self.data_source, "rgb", "{:.6f}".format(rgb_id) + ".png"))
        rgb = cv.cvtColor(bgr, cv.COLOR_BGR2RGB)

        depth = cv.imread(
            os.path.join(self.data_source, "depth", "{:.6f}".format(depth_id) + ".png"), -1
        )

        rgb = o3d.geometry.Image(rgb)
        depth = o3d.geometry.Image(depth)

        h, w, _ = bgr.shape

        intrinsic = o3d.camera.PinholeCameraIntrinsic()
        intrinsic.set_intrinsics(height=h, width=w, fx=525.0, fy=525.0, cx=319.5, cy=239.5)

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            rgb, depth, depth_scale=5000, depth_trunc=100.0, convert_rgb_to_intensity=False
        )

        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd, intrinsic, pose_inv, project_valid_depth_only=True
        )

        xyz = np.array(pcd.points)
        colors = np.array(pcd.colors)

        if self.get_color:
            return xyz, colors, np.array(pose)
        return xyz, np.array(pose)

    def __len__(self):
        return len(self.matches)
