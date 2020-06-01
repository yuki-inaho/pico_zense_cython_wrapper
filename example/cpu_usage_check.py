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
        pdb.set_trace()        
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


def configure_flipping():
    zense_mng = PyPicoZenseManager(0, debug=True)
    zense_setting = Setting()
    zense_setting.flip(CameraFlags.EnableComputeRealDepthFilter)
    set_setting(zense_mng, zense_setting)
    time.sleep(1)
    for _ in range(5):
        zense_setting.flip(CameraFlags.EnableComputeRealDepthFilter)
        set_setting(zense_mng, zense_setting)
        time.sleep(1)


def monitor(target):
    worker_process = mp.Process(target=target)
    worker_process.start()
    p = psutil.Process(worker_process.pid)

    # log cpu usage of `worker_process` every 10 ms
    cpu_percents = []
    while worker_process.is_alive():
        cpu_percents.append(p.cpu_percent())
        time.sleep(0.01)

    worker_process.join()
    return cpu_percents

    zense_setting = Setting()
    zense_setting.flip(CameraFlags.EnableComputeRealDepthFilter)

cpu_percents = monitor(target=configure_flipping)
