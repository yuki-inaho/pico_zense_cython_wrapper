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


# Unique objects
zense_mng = PyPicoZenseManager(0, debug=True)
this_pid = filter(lambda p: psutil.Process(
    p).name() == "python", psutil.pids())[0]


class CameraFlags(IntEnum):
    EnableDepthDistCorrection = 1,
    EnableIRDistCorrection = 2,
    EnableRGBDistCorrection = 3,
    EnableComputeRealDepthFilter = 4,
    EnableSmoothingFilter = 5,
    EnabledRGBToDepth = 6,
    EnabledDepth2RGB = 7


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
    zense_setting = Setting(switch_flags)
    set_setting(zense_mng, zense_setting)
    process = psutil.Process(this_pid)

    # log cpu usage of `worker_process` every 10 ms
    cpu_percents = []
    count = 1
    while count < 1000:
        zense_mng.update()
        if (count % 100) == 0:
            configure_flipping()
        cpu_percents.append(process.cpu_percent())
        print("{}%".format(cpu_percents[-1]))
        time.sleep(0.05)
        count += 1

    return cpu_percents


zense_setting = Setting()
zense_setting.flip(CameraFlags.EnableComputeRealDepthFilter)
cpu_percents = monitor()
