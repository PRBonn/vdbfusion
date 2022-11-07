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


class ScanNet:
    def __init__(self, data_source, get_color=False, split=None):

        self.name = data_source.split("/")[-1]

        self.get_color = get_color
        self.data_source = data_source

        self.rgb_list = os.listdir(os.path.join(self.data_source, "color"))
        self.rgb_list.sort()

        self.depth_list = os.listdir(os.path.join(self.data_source, "depth"))
        self.depth_list.sort()

        self.pose_list = os.listdir(os.path.join(self.data_source, "pose"))
        self.pose_list.sort()

    def __len__(self):
        return len(self.pose_list)

    def __getitem__(self, idx):

        pose = np.loadtxt(os.path.join(self.data_source, "pose", self.pose_list[idx]))
        pose_inv = np.linalg.inv(pose)

        bgr = cv.imread(os.path.join(self.data_source, "color", self.rgb_list[idx]))
        rgb = cv.cvtColor(bgr, cv.COLOR_BGR2RGB)

        depth = cv.imread(os.path.join(self.data_source, "depth", self.depth_list[idx]), -1)

        K = np.loadtxt(os.path.join(self.data_source, "intrinsic_depth.txt"))

        rgb = o3d.geometry.Image(rgb)
        depth = o3d.geometry.Image(depth)

        h, w, _ = bgr.shape

        intrinsic = o3d.camera.PinholeCameraIntrinsic()
        intrinsic.set_intrinsics(
            height=h,
            width=w,
            fx=K[0, 0],
            fy=K[1, 1],
            cx=K[0, 2],
            cy=K[1, 2],
        )

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            rgb, depth, depth_scale=1000, depth_trunc=1000.0, convert_rgb_to_intensity=False
        )

        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd, intrinsic, pose_inv, project_valid_depth_only=True
        )

        xyz = np.array(pcd.points)
        colors = np.array(pcd.colors)

        xyz = np.array(xyz)
        colors = np.array(colors)

        if self.get_color:
            return xyz, colors, np.array(pose)

        return xyz, np.array(pose)
