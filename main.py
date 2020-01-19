#!/usr/bin/env python
# -*- coding: utf-8 -*-
import toml 
import click
from zense_pywrapper import PyPicoZenseManager
import pdb

@click.command()
@click.option('--out', '-o', default='camera_parameter.toml')
def main(out):
    zense = PyPicoZenseManager()
    dict_toml = toml.load(open('template.toml'))
    dict_toml["Camera0"]["serial_no"] = zense.getSerialNumber()
    params = zense.getCameraParameter()

    dict_toml["Camera0_Factory"]["fx"] = params[0]
    dict_toml["Camera0_Factory"]["fy"] = params[1]
    dict_toml["Camera0_Factory"]["cx"] = params[2]
    dict_toml["Camera0_Factory"]["cy"] = params[3]
    dict_toml["Camera0_Factory"]["p1"] = params[4]
    dict_toml["Camera0_Factory"]["p2"] = params[5]
    dict_toml["Camera0_Factory"]["k1"] = params[6]    
    dict_toml["Camera0_Factory"]["k2"] = params[7]    
    dict_toml["Camera0_Factory"]["k3"] = params[8]    
    dict_toml["Camera0_Factory"]["k4"] = params[9]    

    with open(out, "w") as f:
        toml.dump(dict_toml, f)
    del zense

if __name__ == "__main__":
    main()
