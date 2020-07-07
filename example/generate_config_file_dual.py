#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os.path as osp
import sys
import toml
import click
from zense_pywrapper import PyPicoZenseManager
from collections import OrderedDict
SCRIPT_PATH = osp.dirname(osp.abspath(sys.argv[0]))


@click.command()
@click.option('--out', '-o', default='{}/../cfg/dualzense.toml'.format(SCRIPT_PATH))
def main(out):
    zense_0 = PyPicoZenseManager(0)
    zense_1 = PyPicoZenseManager(1)

    decoder = toml.TomlDecoder(_dict=OrderedDict)
    encoder = toml.TomlEncoder(_dict=OrderedDict)
    toml.TomlEncoder = encoder
    intrinsic_elems = ["fx", "fy", "cx", "cy",
                       "p1", "p2", "k1", "k2",
                       "k3", "k4", "k5", "k6"]
    dict_toml = toml.load(open('{}/../cfg/template_dual.toml'.format(SCRIPT_PATH)),
                          _dict=OrderedDict, decoder=decoder)

    dict_toml["Camera0"]["serial_no"] = zense_0.serial_number
    intrinsic_depth_params_0 = zense_0.camera_parameter_depth
    for i, _elem in enumerate(intrinsic_elems):
        dict_toml["Camera0_Factory"][_elem] = intrinsic_depth_params_0[i]

    dict_toml["Camera1"]["serial_no"] = zense_1.serial_number
    intrinsic_depth_params_1 = zense_1.camera_parameter_depth
    for i, _elem in enumerate(intrinsic_elems):
        dict_toml["Camera1_Factory"][_elem] = intrinsic_depth_params_1[i]

    with open(out, "w") as f:
        toml.encoder.dump(dict_toml, f)
        print("generated")

    del zense_0
    del zense_1


if __name__ == "__main__":
    main()
