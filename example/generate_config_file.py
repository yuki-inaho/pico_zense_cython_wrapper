#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os.path as osp
import sys
import toml
import click
from zense_pywrapper import PyPicoZenseManager
from collections import OrderedDict
import pdb
SCRIPT_PATH = osp.dirname(osp.abspath(sys.argv[0]))

@click.command()
@click.option('--out', '-o', default='{}/../cfg/camera_parameter.toml'.format(SCRIPT_PATH))
def main(out):
    zense = PyPicoZenseManager(0)

    decoder = toml.TomlDecoder(_dict=OrderedDict)
    encoder = toml.TomlEncoder(_dict=OrderedDict)
    toml.TomlEncoder = encoder
    dict_toml = toml.load(open('{}/../cfg/template.toml'.format(SCRIPT_PATH)),
                          _dict=OrderedDict, decoder=decoder)

    dict_toml["Camera0"]["serial_no"] = zense.serial_number
    intrinsic_depth_params = zense.camera_parameter_depth
    intrinsic_rgb_params = zense.camera_parameter_rgb

    intrinsic_elems = ["fx", "fy", "cx", "cy",
                       "p1", "p2", "k1", "k2",
                       "k3", "k4", "k5", "k6"]

    for i, _elem in enumerate(intrinsic_elems):
        dict_toml["Camera0_Factory"][_elem] = intrinsic_depth_params[i]

    for i, _elem in enumerate(intrinsic_elems):
        dict_toml["Camera0_RGB_Factory"][_elem] = intrinsic_rgb_params[i]

    ext_params = zense.extrinsic_parameter
    _rotation_ext = ext_params[0]
    _translation_ext = ext_params[1]
    rotation_extrinsic_elems = ["r11", "r12", "r13",
                                "r21", "r22", "r23",
                                "r31", "r32", "r33"]
    translation_extrinsic_elems = ["tx", "ty", "tz"]

    for i, _elem in enumerate(rotation_extrinsic_elems):
        dict_toml["Camera0_Extrinsic_Factory"][_elem] = _rotation_ext[i]
    for i, _elem in enumerate(translation_extrinsic_elems):
        dict_toml["Camera0_Extrinsic_Factory"][_elem] = _translation_ext[i]

    with open(out, "w") as f:
        toml.encoder.dump(dict_toml, f)
        print("generated")

    del zense


if __name__ == "__main__":
    main()
