#! /usr/bin/python
#  capture and visualize WDR-Depth images using zense (range1 = {>0, e.g. 0}, range2 = {>0, e.g. 0}, rgb_image != 1)
import os
import sys
import toml
import numpy as np
from zense_pywrapper import PyPicoZenseManager
import cv2
import cvui

drange_dict = {-1: "Undefined", 0: "Near", 1: "Mid", 2: "Far"}
laser_dict = {"Undefined": 160, "Near": 160, "Mid": 280, "Far": 300}

WINDOW_NAME = "WDR Test"
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CFG_PARAM_PATH = os.path.join(SCRIPT_DIR, "../cfg/camera_parameter.toml")

def is_rgbir_enabled():
    cfg_path = CFG_PARAM_PATH
    toml_dict = toml.load(open(cfg_path))
    isRGBIR = int(toml_dict["Camera0"]["range1"]) < 0 and int(toml_dict["Camera0"]["rgb_image"]) == 1
    do_exit = False
    if not isRGBIR:
        print("Current camera setting is not IR-RGB mode.")
        do_exit = True
    if do_exit:
        assert False


def ir_16UC1_to_8UC3(ir_img):
    _ir_img = ir_img.copy()
    slope = 2040
    over_pix = np.where(_ir_img > slope)
    _ir_img[over_pix[0], over_pix[1]] = 2040
    _ir_img = (_ir_img.astype(np.float)/2040*255).astype(np.uint8)
    _ir_img = cv2.cvtColor(_ir_img, cv2.COLOR_GRAY2RGB)
    return _ir_img

#
# main rootine
#

is_rgbir_enabled()
zense_mng = PyPicoZenseManager(0)
zense_mng.set_device_mode_from_config(CFG_PARAM_PATH, "Camera0")

cvui.init(WINDOW_NAME)
key = cv2.waitKey(10)
while ((key & 0xFF != ord('q')) or (key & 0xFF != 27)):
    status = zense_mng.update()
    if status:
        rgb_img = zense_mng.rgb_image
        ir_img = zense_mng.ir_image
        rgb_img_resized = cv2.resize(rgb_img,(640, 360))
        ir_8uc3 = ir_16UC1_to_8UC3(ir_img)

        frame = np.zeros((840, 640, 3), np.uint8)
        frame[0:360, 0:640, :] = rgb_img_resized
        frame[360:840, 0:640, :] = ir_8uc3

        cvui.update()
        cv2.imshow(WINDOW_NAME, frame)
        key = cv2.waitKey(20)
        if key == 27:
            break

cv2.destroyAllWindows()
