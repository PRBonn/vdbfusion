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

import glob
import os

import numpy as np
import open3d as o3d


class BunnyDataset:
    """The bun.conf does not specify how to work with the trasnformation, after hours of tyring to
    debug how to use it, I couldn't find how to use the 1996 dataset.

        So I've created my own. Contact me if you feel curious on how I've obtianed it.

    ./mesh_to_dataset.py bunny.ply --scan-count 10
    """

    def __init__(self, bunny_root, apply_pose: bool = True):
        # Cache
        self.use_cache = True
        self.apply_pose = apply_pose

        self.data_dir = os.path.join(bunny_root, "generated")
        self.scan_dir = os.path.join(self.data_dir, "data/")
        self.poses = self.load_bunny_poses()
        self.scans = self.load_bunny_clouds()
        assert len(self.scans) == len(self.poses)

    def load_bunny_poses(self):
        filename = os.path.join(self.data_dir, "poses.txt")
        poses = np.loadtxt(filename).reshape(-1, 4, 4)
        return poses

    def load_bunny_clouds(self):
        scans = []
        scan_files = sorted(glob.glob(self.scan_dir + "*.ply"))
        for scan_file in scan_files:
            scan = o3d.io.read_point_cloud(scan_file)
            scans.append(np.asarray(scan.points))
        return scans

    def __getitem__(self, idx):
        return self.scans[idx], self.poses[idx]

    def __len__(self):
        return len(self.scans)
