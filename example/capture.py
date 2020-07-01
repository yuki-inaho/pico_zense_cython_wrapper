import glob
import os
import sys
import os.path as osp
import shutil

import cv2
import cvui
import numpy as np
from zense_pywrapper import PyPicoZenseManager
import json

import pdb

DATA_SAVE_DIR = os.path.join(os.getcwd(), "./data")

WINDOW_NAME = "Capture"
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480


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


def ir_uc8_normalized_img(img, max_var):
    img_norm = img.copy().astype(np.float32)
    img_norm[np.where(img_norm > max_var)] = 0
    zero_idx = np.where((img_norm > max_var) | (img_norm == 0))
    img_norm *= 255.0/max_var
    img_norm_uc8 = np.zeros([img.shape[0], img.shape[1], 3])
    img_norm_uc8[:, :, 0] = img_norm.astype(np.uint8)
    img_norm_uc8[:, :, 1] = img_norm.astype(np.uint8)
    img_norm_uc8[:, :, 2] = img_norm.astype(np.uint8)
    img_norm_uc8[zero_idx[0], zero_idx[1], :] = 0
    return img_norm_uc8


def main():
    global TOML_PATH_ZENSE
    global DATA_SAVE_DIR
    global WINDOW_NAME
    global IMAGE_WIDTH
    global IMAGE_HEIGHT

    if not os.path.exists(DATA_SAVE_DIR):
        os.mkdir(DATA_SAVE_DIR)

    if not os.path.exists(osp.join(DATA_SAVE_DIR, "depth")):
        os.mkdir(osp.join(DATA_SAVE_DIR, "depth"))

    if not os.path.exists(osp.join(DATA_SAVE_DIR, "color")):
        os.mkdir(osp.join(DATA_SAVE_DIR, "color"))

    number_of_saved_frame = len(
        glob.glob(osp.join(DATA_SAVE_DIR, "depth", "*.png")))

    SCRIPT_DIR_PATH = os.path.dirname(os.path.abspath(__file__))
    CFG_PARAM_PATH = os.path.join(SCRIPT_DIR_PATH, "../cfg/camera_parameter.toml")
    zense_mng = PyPicoZenseManager(0, CFG_PARAM_PATH, "Camera0")

    cvui.init(WINDOW_NAME)
    key = cv2.waitKey(20)

    while ((key & 0xFF != ord('q')) or (key & 0xFF != 27)):
        status = zense_mng.update()
        if status:
            ir_img = zense_mng.ir_image
            depth_img = zense_mng.depth_image

            ir_img_resized = cv2.resize(ir_img, (IMAGE_WIDTH, IMAGE_HEIGHT))

            depth_img_colorized = colorize_depth_img(depth_img, 2000)
            ir_img_normd = ir_uc8_normalized_img(ir_img_resized, 3840)

            frame = np.zeros((IMAGE_HEIGHT*2, IMAGE_WIDTH*2, 3), np.uint8)
            frame[0:IMAGE_HEIGHT, 0:IMAGE_WIDTH, :] = ir_img_normd
            frame[0:IMAGE_HEIGHT, IMAGE_WIDTH:IMAGE_WIDTH *
                  2, :] = depth_img_colorized

            cvui.printf(frame, 50, IMAGE_HEIGHT+50, 0.8, 0x00ff00,
                        "Number of Captured Images : %d", number_of_saved_frame)
            if (cvui.button(frame, 100, IMAGE_HEIGHT+100, 200, 100,  "Capture")) or (key & 0xFF == ord('s')):
                cv2.imwrite(osp.join(DATA_SAVE_DIR, "depth", "%06d.png" %
                                     (number_of_saved_frame)), depth_img)
                cv2.imwrite(osp.join(DATA_SAVE_DIR, "color",
                                     "%06d.png" % (number_of_saved_frame)), ir_img)
                number_of_saved_frame += 1

            if cvui.button(frame, 350, IMAGE_HEIGHT+100, 200, 100,  "Erase Images"):
                shutil.rmtree(osp.join(DATA_SAVE_DIR, "depth"))
                os.mkdir(osp.join(DATA_SAVE_DIR, "depth"))
                shutil.rmtree(osp.join(DATA_SAVE_DIR, "color"))
                os.mkdir(osp.join(DATA_SAVE_DIR, "color"))
                number_of_saved_frame = 0

            cvui.update()
            cv2.imshow(WINDOW_NAME, frame)
            key = cv2.waitKey(20)
            if key == 27:
                break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
