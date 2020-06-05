import multiprocessing as mp
import os.path as osp
import sys
import time

import click
import toml
from zense_pywrapper import PyPicoZenseManager
import pdb
import psutil
from enum import Enum, IntEnum
from datetime import datetime
import numpy as np


def get_timestamp_milliseconds():
    return float((datetime.now() - datetime.utcfromtimestamp(0)).total_seconds() * 1e3)


# Unique objects
zense_mng = PyPicoZenseManager(0, debug=True)
this_pid = filter(
    lambda p: psutil.Process(p).name() == "python", psutil.pids()
)[0]


class CameraFlags(IntEnum):
    EnableDepthDistCorrection = 0,
    EnableIRDistCorrection = 1,
    EnableRGBDistCorrection = 2,
    EnableComputeRealDepthFilter = 3,
    EnableSmoothingFilter = 4,
    EnabledRGBToDepth = 5,
    EnabledDepth2RGB = 6


class Setting:
    def __init__(self):
        EnableDepthDistCorrection = False
        EnableIRDistCorrection = False
        EnableRGBDistCorrection = False
        EnableComputeRealDepthFilter = False
        EnableSmoothingFilter = False
        EnabledRGBToDepth = False
        EnabledDepth2RGB = False
        self.flags = [
            EnableDepthDistCorrection,
            EnableIRDistCorrection,
            EnableRGBDistCorrection,
            EnableComputeRealDepthFilter,
            EnableSmoothingFilter,
            EnabledRGBToDepth,
            EnabledDepth2RGB
        ]

    def flip(self, idx):
        self.flags[idx] = not self.flags[idx]

    @property
    def flags(self):
        return self.flags


def set_setting(zense_mng, zense_setting):
    EnableDepthDistCorrection, \
        EnableIRDistCorrection, \
        EnableRGBDistCorrection, \
        EnableComputeRealDepthFilter, \
        EnableSmoothingFilter, \
        EnabledRGBToDepth, \
        EnabledDepth2RGB = zense_setting.flags

    zense_mng.setup_debug(
        EnableDepthDistCorrection,
        EnableIRDistCorrection,
        EnableRGBDistCorrection,
        EnableComputeRealDepthFilter,
        EnableSmoothingFilter,
        EnabledRGBToDepth,
        EnabledDepth2RGB
    )


def configure_flipping(switch_flag):
    zense_setting = Setting()
    zense_setting.flip(switch_flag)
    set_setting(zense_mng, zense_setting)


def monitor(switch_flags):
    zense_setting = Setting()
    set_setting(zense_mng, zense_setting)
    # log cpu usage of `worker_process` every 10 ms
    cpu_percents = []
    count = 1
    start_time = get_timestamp_milliseconds()
    while count < 500:
        if (count % 100) == 0:
            configure_flipping(switch_flags)
        cpu_percents.append(
            [(get_timestamp_milliseconds()-start_time)/1000, psutil.cpu_percent()])
        print("{}%".format(cpu_percents[-1]))
        time.sleep(0.05)
        count += 1
    return cpu_percents


cpu_percents_depth_dist = monitor(CameraFlags.EnableDepthDistCorrection)
np.savetxt("data/cpu_percents_depth_dist.txt" , cpu_percents_depth_dist)
cpu_percents_ir_dist = monitor(CameraFlags.EnableIRDistCorrection)
np.savetxt("data/cpu_percents_ir_dist.txt" , cpu_percents_ir_dist)
cpu_percents_rgb_dist = monitor(CameraFlags.EnableRGBDistCorrection)
np.savetxt("data/cpu_percents_rgb_dist.txt" , cpu_percents_rgb_dist)
cpu_percents_real_depth = monitor(CameraFlags.EnableComputeRealDepthFilter)
np.savetxt("data/cpu_percents_real_depth.txt" , cpu_percents_real_depth)
cpu_percents_smooth_filter = monitor(CameraFlags.EnableSmoothingFilter)
np.savetxt("data/cpu_percents_smooth_filter.txt" , cpu_percents_smooth_filter)
cpu_percents_enable_rgb_to_depth = monitor(CameraFlags.EnabledRGBToDepth)
np.savetxt("data/cpu_percents_enable_rgb_to_depth.txt" , cpu_percents_enable_rgb_to_depth)
cpu_percents_enable_depth_to_rgb = monitor(CameraFlags.EnabledDepth2RGB)
np.savetxt("data/cpu_percents_enable_depth_to_rgb.txt" , cpu_percents_enable_depth_to_rgb)
