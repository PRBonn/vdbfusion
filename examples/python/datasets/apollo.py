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

# Original implementation by Luiggi Kartofellone
import glob
import natsort
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

import sys

sys.path.append("..")
from utils.cache import get_cache, memoize


class ApolloDataset:
    def __init__(self, folder: str):
        super().__init__()
        self.scan_files = natsort.natsorted(glob.glob(f"{folder}/pcds/*.pcd"))
        self.poses = self.readPoses(f"{folder}/poses/gt_poses.txt")
        assert len(self.scan_files) == self.poses.shape[0]

        # Cache
        self.use_cache = True
        self.cache = get_cache(directory="cache/apollo/")

    def __getitem__(self, idx):
        pose = np.linalg.inv(self.poses[0, :, :]) @ self.poses[idx, :, :]
        points = self.get_scan(self.scan_files[idx], pose)
        return points, pose

    def __len__(self):
        return len(self.scan_files)

    @staticmethod
    def readPoses(file):
        data = np.loadtxt(file)
        id, time, t, q = np.split(data, [1, 2, 5], axis=1)
        r = R.from_quat(q).as_matrix()
        pose = np.zeros([r.shape[0], 4, 4])
        pose[:, :3, -1] = t
        pose[:, :3, :3] = r
        pose[:, -1, -1] = 1
        return pose

    @memoize()
    def get_scan(self, file: str, pose):
        pcd = o3d.io.read_point_cloud(file)
        pcd.transform(pose)
        return np.asarray(pcd.points)
