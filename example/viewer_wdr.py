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


def is_wdr_enabled():
    cfg_path = CFG_PARAM_PATH
    toml_dict = toml.load(open(cfg_path))
    isWDR = int(toml_dict["Camera0"]["range1"]) >= 0 and \
        int(toml_dict["Camera0"]["range2"]) >= 0
    isRGB = int(toml_dict["Camera0"]["rgb_image"]) == 1
    do_exit = False
    if not isWDR:
        print("Current camera setting is WDR disabled mode. This app can be executed under WDR enabled setting")
        do_exit = True
    if isRGB:
        print("Current camera setting is RGB enabled mode. This app can be executed under RGB disabled setting")
        do_exit = True
    if do_exit:
        assert False


def colorize_depth_img(img, max_var):
    img_colorized = np.zeros([img.shape[0], img.shape[1], 3]).astype(np.uint8)
    img_colorized[:, :, 1] = 255
    img_colorized[:, :, 2] = 255
    img_hue = img.copy().astype(np.float32)
    img_hue[np.where(img_hue > max_var)] = 0
    zero_idx = np.where((img_hue > max_var) | (img_hue == 0))
    img_hue *= 255.0/max_var
    img_colorized[:, :, 0] = img_hue.astype(np.uint8)
    img_colorized = cv2.cvtColor(img_colorized, cv2.COLOR_HSV2RGB)
    img_colorized[zero_idx[0], zero_idx[1], :] = 0
    return img_colorized


#
# main rootine
#

is_wdr_enabled()
zense_mng = PyPicoZenseManager(0, CFG_PARAM_PATH, "Camera0")
cvui.init(WINDOW_NAME)

# zense_mng.set_laser_intensity(600)
#status = zense_mng.update_laser_intensity()

key = cv2.waitKey(10)
while ((key & 0xFF != ord('q')) or (key & 0xFF != 27)):
    status = zense_mng.update()
    if status:
        depth_img_r1 = zense_mng.depth_image_range1
        depth_img_r2 = zense_mng.depth_image_range2
        depth_img_r1_colorized = colorize_depth_img(depth_img_r1, 2000)
        depth_img_r2_colorized = colorize_depth_img(depth_img_r2, 2000)

        depth_img_r1_resized = cv2.resize(depth_img_r1_colorized,
                                          (IMAGE_WIDTH, IMAGE_HEIGHT))
        depth_img_r2_resized = cv2.resize(depth_img_r2_colorized,
                                          (IMAGE_WIDTH, IMAGE_HEIGHT))
        frame = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH * 2, 3), np.uint8)
        frame[0:IMAGE_HEIGHT, 0:IMAGE_WIDTH, :] = depth_img_r1_resized
        frame[0:IMAGE_HEIGHT,
              IMAGE_WIDTH:IMAGE_WIDTH * 2, :] = depth_img_r2_resized

        cvui.update()
        cv2.imshow(WINDOW_NAME, frame)
        key = cv2.waitKey(20)
        if key == 27:
            break

cv2.destroyAllWindows()
