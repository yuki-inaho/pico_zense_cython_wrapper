#! /usr/bin/python
import os
import toml
import numpy as np
from zense_pywrapper import PyPicoZenseManager
import cv2

N_SENSORS = 4
WDR_RANGE1_PULSE_COUNT = 280
WDR_RANGE2_PULSE_COUNT = 600

drange_dict = {-1: "Undefined", 0: "Near", 1: "Mid", 2: "Far"}
laser_dict = {"Undefined": 160, "Near": 160, "Mid": 280, "Far": 300}

WINDOW_NAME = "WDR Test"
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CFG_PARAM_PATH = os.path.join(SCRIPT_DIR, "../cfg/dualzense.toml")


def colorize_depth_img(img, max_var):
    img_colorized = np.zeros([img.shape[0], img.shape[1], 3]).astype(np.uint8)
    img_colorized[:, :, 1] = 255
    img_colorized[:, :, 2] = 255
    img_hue = img.copy().astype(np.float32)
    img_hue[np.where(img_hue > max_var)] = 0
    zero_idx = np.where((img_hue > max_var) | (img_hue == 0))
    img_hue *= 255.0 / max_var
    img_colorized[:, :, 0] = img_hue.astype(np.uint8)
    img_colorized = cv2.cvtColor(img_colorized, cv2.COLOR_HSV2RGB)
    img_colorized[zero_idx[0], zero_idx[1], :] = 0
    return img_colorized


#
# main rootine
#

dict_toml = toml.load(CFG_PARAM_PATH)

zenses_for_serial = [PyPicoZenseManager(i) for i in range(N_SENSORS)]
serial_number_list = [zenses_for_serial[i].serial_number for i in range(N_SENSORS)]
del zenses_for_serial
left_zense_sn = dict_toml["Camera0"]["serial_no"]
right_zense_sn = dict_toml["Camera1"]["serial_no"]
left_zense_index = serial_number_list.index(left_zense_sn)
right_zense_index = serial_number_list.index(right_zense_sn)
other_zense_idxs = np.setdiff1d(np.arange(4), [left_zense_index, right_zense_index])


zense_mng_left = PyPicoZenseManager(left_zense_index, CFG_PARAM_PATH, "Camera0")
zense_mng_right = PyPicoZenseManager(right_zense_index, CFG_PARAM_PATH, "Camera1")
zense_mng_left.set_pulse_count_WDR(WDR_RANGE1_PULSE_COUNT, WDR_RANGE2_PULSE_COUNT)
zense_mng_right.set_pulse_count_WDR(WDR_RANGE1_PULSE_COUNT, WDR_RANGE2_PULSE_COUNT)
zense_mng_left.enable_external_trigger()
zense_mng_right.enable_external_trigger()

zense_mng_sidecar0 = PyPicoZenseManager(other_zense_idxs[0])
zense_mng_sidecar1 = PyPicoZenseManager(other_zense_idxs[1])

while True:
    status = zense_mng_left.update()
    status &= zense_mng_right.update()
    status &= zense_mng_sidecar0.update()
    status &= zense_mng_sidecar1.update()
    if status:
        pulcnt_wdr_left = zense_mng_left.get_pulse_count_WDR()
        pulcnt_wdr_right = zense_mng_right.get_pulse_count_WDR()
        print("zense(index=0) :range1={}, range2={}".format(pulcnt_wdr_left[0], pulcnt_wdr_left[1]))
        print("zense(index=1) :range1={}, range2={} \n".format(pulcnt_wdr_right[0], pulcnt_wdr_right[1]))
