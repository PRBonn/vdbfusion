#!/usr/bin/env python3
# @file      icl_pipeline.py
# @author    Ignacio Vizzo     [ivizzo@uni-bonn.de]
#
# Copyright (c) 2021 Ignacio Vizzo, all rights reserved
import argh

from datasets import ICLDataset as Dataset
from vdbfusion_pipeline import VDBFusionPipeline as Pipeline


def main(
    data_source: str,
    config: str = "config/icl.yaml",
    n_scans: int = -1,
    jump: int = 0,
    visualize: bool = False,
):
    """Help here!"""
    pipeline = Pipeline(Dataset(data_source), config, jump=jump, n_scans=n_scans, map_name="icl")
    pipeline.run()
    pipeline.visualize() if visualize else None


if __name__ == "__main__":
    argh.dispatch_command(main)
