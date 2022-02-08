#!/usr/bin/env python3
# @file      kitti_pipeline.py
# @author    Ignacio Vizzo     [ivizzo@uni-bonn.de]
#
# Copyright (c) 2021 Ignacio Vizzo, all rights reserved
import argh
from datasets import KITTIOdometryDataset as Dataset
from vdbfusion_pipeline import VDBFusionPipeline as Pipeline


def main(
    kitti_root_dir: str,
    sequence: int = 0,
    config: str = "config/kitti.yaml",
    n_scans: int = -1,
    jump: int = 0,
    visualize: bool = False,
):
    """Help here!"""
    dataset = Dataset(kitti_root_dir, sequence, config)
    pipeline = Pipeline(dataset, config, f"kitti_{str(sequence).zfill(2)}", jump, n_scans)
    pipeline.run()
    pipeline.visualize() if visualize else None


if __name__ == "__main__":
    argh.dispatch_command(main)
