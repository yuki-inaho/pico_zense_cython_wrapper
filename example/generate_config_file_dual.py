#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os.path as osp
import glob
import sys
import toml
import click
import time
from six.moves import input
import numpy as np
from zense_pywrapper import PyPicoZenseManager
from collections import OrderedDict


SCRIPT_DIR = osp.dirname(osp.abspath(__file__))
is_py2 = sys.version_info.major == 2


def wait_for_zense_connection():
    count = 0
    while True:
        zense_path = glob.glob("/dev/v4l/by-id/usb-PicoZense*")
        if len(zense_path) == 0:
            sys.stdout.write("\rWaiting for device detection (pico zense)" +
                             "." * count)
            time.sleep(1)
            sys.stdout.flush()
            count += 1
        else:
            break


def wait_for_zense_disconnection():
    count = 0
    while True:
        zense_path = glob.glob("/dev/v4l/by-id/usb-PicoZense*")
        if len(zense_path) > 0:
            sys.stdout.write("\rWaiting for device disconnection (pico zense)" +
                             "." * count)
            time.sleep(1)
            sys.stdout.flush()
            count += 1
        else:
            break


@click.command()
@click.option('--out', '-o', default='{}/../cfg/dualzense.toml'.format(SCRIPT_DIR))
@click.option('--template', '-t', default='{}/../cfg/template_dual.toml'.format(SCRIPT_DIR))
def main(out, template):
    decoder = toml.TomlDecoder(_dict=OrderedDict)
    encoder = toml.TomlEncoder(_dict=OrderedDict)
    toml.TomlEncoder = encoder
    dict_toml = toml.load(open(template), _dict=OrderedDict, decoder=decoder)
    intrinsic_elems = ["fx", "fy", "cx", "cy",
                       "p1", "p2", "k1", "k2",
                       "k3", "k4", "k5", "k6"]
    rotation_extrinsic_elems = ["r11", "r12", "r13",
                                "r21", "r22", "r23",
                                "r31", "r32", "r33"]
    translation_extrinsic_elems = ["tx", "ty", "tz"]

    # Get video device information related with zense
    input("Please connect \"Left\" Zense and press ENTER")
    wait_for_zense_connection()
    time.sleep(0.5)
    zense = PyPicoZenseManager(0)
    if is_py2:
        dict_toml["Camera0"]["serial_no"] = zense.serial_number
    else:
        dict_toml["Camera0"]["serial_no"] = zense.serial_number.decode('utf-8')
    dict_toml["Camera0"]["serial_no"] = zense.serial_number

    intrinsic_depth_params = zense.camera_parameter_depth
    intrinsic_rgb_params = zense.camera_parameter_rgb
    for i, _elem in enumerate(intrinsic_elems):
        dict_toml["Camera0_Factory"][_elem] = intrinsic_depth_params[i]
        dict_toml["Camera0_RGB_Factory"][_elem] = intrinsic_rgb_params[i]

    ext_params = zense.extrinsic_parameter
    _rotation_ext = ext_params[0]
    _translation_ext = ext_params[1]
    for i, _elem in enumerate(rotation_extrinsic_elems):
        dict_toml["Camera0_Extrinsic_Factory"][_elem] = _rotation_ext[i]
    for i, _elem in enumerate(translation_extrinsic_elems):
        dict_toml["Camera0_Extrinsic_Factory"][_elem] = _translation_ext[i]

    zense.close()
    del zense

    input("Please disconnect \"Left\" Zense and press ENTER")
    wait_for_zense_disconnection()

    input("Please connect \"Right\" Zense and press ENTER")
    wait_for_zense_connection()
    time.sleep(0.5)
    zense = PyPicoZenseManager(0)
    if is_py2:
        dict_toml["Camera1"]["serial_no"] = zense.serial_number
    else:
        dict_toml["Camera1"]["serial_no"] = zense.serial_number.decode('utf-8')
    dict_toml["Camera1"]["serial_no"] = zense.serial_number

    intrinsic_depth_params = zense.camera_parameter_depth
    intrinsic_rgb_params = zense.camera_parameter_rgb
    for i, _elem in enumerate(intrinsic_elems):
        dict_toml["Camera1_Factory"][_elem] = intrinsic_depth_params[i]
        dict_toml["Camera1_RGB_Factory"][_elem] = intrinsic_rgb_params[i]

    ext_params = zense.extrinsic_parameter
    _rotation_ext = ext_params[0]
    _translation_ext = ext_params[1]
    for i, _elem in enumerate(rotation_extrinsic_elems):
        dict_toml["Camera1_Extrinsic_Factory"][_elem] = _rotation_ext[i]
    for i, _elem in enumerate(translation_extrinsic_elems):
        dict_toml["Camera1_Extrinsic_Factory"][_elem] = _translation_ext[i]

    zense.close()
    del zense

    with open(out, "w") as f:
        toml.encoder.dump(dict_toml, f)
        print("generated")


if __name__ == "__main__":
    main()
