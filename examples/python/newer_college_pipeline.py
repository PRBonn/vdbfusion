#!/usr/bin/env python3
# @file      kitti_pipeline.py
# @author    Ignacio Vizzo     [ivizzo@uni-bonn.de]
#
# Copyright (c) 2021 Ignacio Vizzo, all rights reserved
import argh

from datasets import NewerCollegeDataset as Dataset
from vdbfusion_pipeline import VDBFusionPipeline as Pipeline


def main(
    ouster_scans: str,
    sequence: int = 0,
    config: str = "config/newer_college.yaml",
    n_scans: int = -1,
    jump: int = 0,
    visualize: bool = False,
):
    """Help here!"""
    dataset = Dataset(ouster_scans, config)
    pipeline = Pipeline(dataset, config, jump=jump, n_scans=n_scans, map_name="newer_college")
    pipeline.run()
    pipeline.visualize() if visualize else None


if __name__ == "__main__":
    argh.dispatch_command(main)
