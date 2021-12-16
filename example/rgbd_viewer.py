import glob
import os
import sys
import os.path as osp
import shutil

import cv2
import cvui
import numpy as np
from zense_pywrapper import PyPicoZenseManager
from utils.zense_camera_param import CameraParam
from utils.projection_module import PixelProjectorRGBDepth
from utils.projection_utils import cvt_depth2pcl, cvt_numpy2open3d, colorize_depth
import toml
import json

import pdb


WINDOW_NAME = "RGBD"
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480


def rgbd_fusion(rgb_img, depth_img):
    pass


def main():
    global WINDOW_NAME
    global IMAGE_WIDTH
    global IMAGE_HEIGHT

    SCRIPT_DIR_PATH = os.path.dirname(os.path.abspath(__file__))
    CFG_PARAM_PATH = os.path.join(SCRIPT_DIR_PATH, "../cfg/camera_parameter.toml")
    zense_mng = PyPicoZenseManager(0, CFG_PARAM_PATH, "Camera0")
    zense_mng.disable_external_trigger()

    toml_dict = toml.load(open(CFG_PARAM_PATH))

    intrinsic_elems = ["fx", "fy", "cx", "cy"]

    zense_depth_camera_param = CameraParam()
    zense_depth_camera_param.set_intrinsic_parameter(*[toml_dict["Camera0_Factory"][elem] for elem in intrinsic_elems])
    zense_depth_camera_param.set_image_size(*[toml_dict["Camera0"][elem] for elem in ["width", "height"]])

    zense_rgb_camera_param = CameraParam()
    zense_rgb_camera_param.set_intrinsic_parameter(
        *[toml_dict["Camera0_RGB_Factory"][elem] for elem in intrinsic_elems]
    )
    rgb_width = 1920
    rgb_height = 1080
    zense_rgb_camera_param.set_image_size(rgb_width, rgb_height)

    rotation_extrinsic_elems = ["r11", "r12", "r13", "r21", "r22", "r23", "r31", "r32", "r33"]
    extrinsic_rot_dcm = np.array(
        [toml_dict["Camera0_Extrinsic_Factory"][elem] for elem in rotation_extrinsic_elems]
    ).reshape(3, 3)
    translation_extrinsic_elems = ["tx", "ty", "tz"]
    extrinsic_translation = np.array(
        [toml_dict["Camera0_Extrinsic_Factory"][elem] for elem in translation_extrinsic_elems]
    )
    transform_c2d = np.eye(4)
    transform_c2d[:3, :3] = extrinsic_rot_dcm
    transform_c2d[:3, 3] = extrinsic_translation / 1000
    transform_d2c = transform_c2d

    projector_d2c = PixelProjectorRGBDepth(zense_depth_camera_param, zense_rgb_camera_param, transform_d2c)

    cvui.init(WINDOW_NAME)
    key = cv2.waitKey(20)

    dict_toml = toml.load(open(CFG_PARAM_PATH))

    while (key & 0xFF != ord("q")) or (key & 0xFF != 27):
        status = zense_mng.update()
        if status:
            rgb_img = zense_mng.rgb_image.copy()
            depth_img = zense_mng.depth_image.copy()
            depth_img[depth_img == 65535] = 0
            depth_img_scaled = depth_img / 1000.0
            pcd_array = cvt_depth2pcl(depth_img_scaled, zense_depth_camera_param)
            depth_image_d2c = projector_d2c.get_projected_points_depth_to_color(pcd_array, scale=1000)
            depth_image_d2c_colorized = colorize_depth(depth_image_d2c, 2000)
            rgbd_img = rgb_img.copy()
            nonzero_idx = np.where(depth_image_d2c > 0)
            rgbd_img[nonzero_idx[0], nonzero_idx[1], :] = depth_image_d2c_colorized[nonzero_idx[0], nonzero_idx[1], :]
            rgbd_img_resized = cv2.resize(rgbd_img, (1280, 720))
            cv2.imshow(WINDOW_NAME, rgbd_img_resized)
            key = cv2.waitKey(20)
            if key == 27:
                break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
