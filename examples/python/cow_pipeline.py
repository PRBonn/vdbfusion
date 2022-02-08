#!/usr/bin/env python3
# @file      cow_pipeline.py
# @author    Ignacio Vizzo     [ivizzo@uni-bonn.de]
#
# Copyright (c) 2021 Ignacio Vizzo, all rights reserved
import argh

from datasets import CowDataset as Dataset
from vdbfusion_pipeline import VDBFusionPipeline as Pipeline


def main(
    data_source: str,
    config: str = "config/cow.yaml",
    n_scans: int = -1,
    jump: int = 0,
    visualize: bool = False,
):
    """Help here!"""
    dataset = Dataset(data_source, get_color=False, apply_pose=True, config_file=config)
    pipeline = Pipeline(dataset, config, jump=jump, n_scans=n_scans, map_name="cow")
    pipeline.run()
    pipeline.visualize() if visualize else None


if __name__ == "__main__":
    argh.dispatch_command(main)
