import sys
import glob
import os
import os.path as osp
import shutil

import toml
import cv2
import cvui
import numpy as np
import time
from zense_pywrapper import PyPicoZenseManager
import json

import pdb

TOML_PATH_ZENSE = os.path.join(os.getcwd(), "./camera_parameter.toml")
DATA_SAVE_DIR = os.path.join(os.getcwd(), "./data")

WINDOW_NAME = "Capture"
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

class JsonEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        else:
            return super(JsonEncoder, self).default(obj)

class IntrinsicParam:
    def __init__(self):
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

def getIntrinsicParam(toml_dict, camera_name):
    intrinsic_param = IntrinsicParam()
    intrinsic_param.fx = toml_dict[camera_name+"_Factory"]["fx"]
    intrinsic_param.fy = toml_dict[camera_name+"_Factory"]["fy"]
    intrinsic_param.cx = toml_dict[camera_name+"_Factory"]["cx"]
    intrinsic_param.cy = toml_dict[camera_name+"_Factory"]["cy"]
    return intrinsic_param

###
### main routine
### 

#mkdir
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

    toml_dict_zense = toml.load(open(TOML_PATH_ZENSE))
    with open(osp.join(DATA_SAVE_DIR, osp.basename(TOML_PATH_ZENSE)), "w") as f:
        toml.dump(toml_dict_zense, f)

    fx = toml_dict_zense["Camera0_Factory"]["fx"]
    fy = toml_dict_zense["Camera0_Factory"]["fy"]
    cx = toml_dict_zense["Camera0_Factory"]["cx"]
    cy = toml_dict_zense["Camera0_Factory"]["cy"]

    with open(osp.join(DATA_SAVE_DIR, "camera_intrinsic.json"), 'w') as outfile:
        obj = json.dump(
            {
                'width':
                    640,
                'height':
                    480,
                'intrinsic_matrix': [
                    fx, 0, 0, 0, fy, 0, cx,
                    cy, 1
                ]
            },
            outfile,
            indent=4)

    number_of_saved_frame = len(glob.glob(osp.join(DATA_SAVE_DIR, "depth", "*.png")))

    zense_mng = PyPicoZenseManager()

    cvui.init(WINDOW_NAME)
    key = cv2.waitKey(20)

    while ((key & 0xFF != ord('q')) or (key & 0xFF != 27)):
        status = zense_mng.update()
        if status:
            rgb_img = zense_mng.getRGBImage()
            depth_img = zense_mng.getDepthImage()
            
            rgb_img_resized = cv2.resize(rgb_img, (IMAGE_WIDTH, IMAGE_HEIGHT))

            depth_img_colorized = np.zeros([IMAGE_HEIGHT, IMAGE_WIDTH, 3]).astype(np.uint8)
            depth_img_colorized[:,:,1]=255
            depth_img_colorized[:,:,2]=255

            _depth_img_zense_hue = depth_img.copy().astype(np.float32)
            _depth_img_zense_hue[np.where(_depth_img_zense_hue > 2000)] = 0
            zero_idx = np.where((_depth_img_zense_hue>2000) | (_depth_img_zense_hue == 0))
            _depth_img_zense_hue *= 255.0/2000.0

            depth_img_colorized[:,:,0]  =_depth_img_zense_hue.astype(np.uint8)
            depth_img_colorized = cv2.cvtColor(depth_img_colorized, cv2.COLOR_HSV2RGB)
            depth_img_colorized[zero_idx[0], zero_idx[1],:] = 0

            frame = np.zeros((IMAGE_HEIGHT*2, IMAGE_WIDTH*2, 3), np.uint8)
            frame[0:IMAGE_HEIGHT, 0:IMAGE_WIDTH,:] = rgb_img_resized
            frame[0:IMAGE_HEIGHT, IMAGE_WIDTH:IMAGE_WIDTH*2,:] = depth_img_colorized

            cvui.printf(frame, 50, IMAGE_HEIGHT+50, 0.8, 0x00ff00,  "Number of Captured Images : %d", number_of_saved_frame)
            if (cvui.button(frame, 100, IMAGE_HEIGHT+100, 200, 100,  "Capture")) or (key & 0xFF == ord('s')):
                cv2.imwrite(osp.join(DATA_SAVE_DIR, "depth", "%06d.png"%(number_of_saved_frame)), depth_img)            
                cv2.imwrite(osp.join(DATA_SAVE_DIR, "color", "%06d.png"%(number_of_saved_frame)), rgb_img)            
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
            if  key == 27:
                break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
