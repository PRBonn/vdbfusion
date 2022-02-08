#!/usr/bin/env python3
# @file      cow_pipeline.py
# @author    Ignacio Vizzo     [ivizzo@uni-bonn.de]
#
# Copyright (c) 2021 Ignacio Vizzo, all rights reserved
import argh

from datasets import BunnyGeneratedDataset as Dataset
from vdbfusion_pipeline import VDBFusionPipeline as Pipeline


def main(
    data_source: str,
    config: str = "config/bunny.yaml",
    visualize: bool = False,
):
    """Help here!"""
    dataset = Dataset(data_source, apply_pose=True)
    pipeline = Pipeline(dataset, config, map_name="bunny")
    pipeline.run()
    pipeline.visualize() if visualize else None


if __name__ == "__main__":
    argh.dispatch_command(main)
