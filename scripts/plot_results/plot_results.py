# -*- coding: utf-8 -*-
import click
import matplotlib.pyplot as plt
from pathlib import Path
import pdb
import numpy as np
from pyflann import *
import pdb
import re


title_dict = {
    'cpu_percents_enable_depth_to_rgb': "toggle : Mapper Depth To RGB Enabled",
    'cpu_percents_ir_dist': "toggle : IR Distortion Correction Enabled",
    'cpu_percents_rgb_dist': "toggle : RGB Distortion Correction Enabled",
    'cpu_percents_depth_dist': "toggle : Depth Distortion Correction Enabled",
    'cpu_percents_real_depth': "toggle : Compute Real Depth Correction Enabled",
    'cpu_percents_enable_rgb_to_depth': "toggle : Mapper Enabled RGB To Depth",
    'cpu_percents_smooth_filter':  "toggle : Smoothing Filter Enabled"
}


def reindex(timestamp, cpu_consumption):
    search_idx = FLANN()
    params = search_idx.build_index(timestamp.reshape(-1, 1))
    sec_vec = np.linspace(0, 60, 120)
    result, dists = search_idx.nn_index(
        sec_vec.reshape(-1, 1), num_neighbors=2)
    target_idx = result[:, 1]
    interval_nearest_measure = dists[:, 1]
    is_measured = interval_nearest_measure < 1.0
    cpu_consumption_arranged = [
        cpu_consumption[target_idx[i]] if is_measured[i] else np.nan for i, sec in enumerate(sec_vec)
    ]
    return sec_vec, cpu_consumption_arranged


@click.command()
@click.option("--data-dir", "-d", required=True)
def main(data_dir):
    data_path_list = list(Path(data_dir).glob("*.txt"))
    for data_path in data_path_list:
        data_str = data_path
        data = np.loadtxt(data_str)

        data_name = data_str.name[:-4]
        timestamp = np.round(data[:, 0], decimals=2)
        cpu_consumption = data[:, 1]
        timestamp, cpu_consumption = reindex(timestamp, cpu_consumption)
        plt.plot(timestamp, cpu_consumption)
        plt.title(title_dict[data_name])
        plt.xlim(0, 60)
        plt.ylim(0, 100)
        plt.xlabel("Elapsed time from sensor activation [s]")
        plt.ylabel("Total CPU Resource Consumption Rate [%]")
        plt.grid(True)

        plt.savefig('result/'+ data_name + ".png") 
        plt.cla()


if __name__ == "__main__":
    main()
