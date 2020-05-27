import open3d as o3d
import os
import toml
import numpy as np
from attr import dataclass, ib
from projection.projection_utils import color2depth_align
import pdb

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
TOML_DIR = "../cfg/camera_parameter.toml"
WIDTH = 640
HEIGHT = 480

dict_toml = toml.load(open(TOML_DIR))
dict_toml_depth_intrinsic = dict_toml["Camera0_Factory"]
dict_toml_rgb_intrinsic = dict_toml["Camera0_Factory"]
dict_toml_extrinsic = dict_toml["Camera0_Extrinsic_Factory"]

# Load Intrinsic Parameters (RGB)
Intrinsic
camera_rgb_intrinsic = o3d.camera.PinholeCameraIntrinsic()
camera_rgb_intrinsic .set_intrinsics(
    width=WIDTH, height=HEIGHT,
    fx=dict_toml_rgb_intrinsic["fx"], 
    fy=dict_toml_rgb_intrinsic["fy"], 
    cx=dict_toml_rgb_intrinsic["cx"], 
    cy=dict_toml_rgb_intrinsic["cy"]
)

# Load Intrinsic Parameters (Depth)
camera_depth_intrinsic = o3d.camera.PinholeCameraIntrinsic()
camera_depth_intrinsic .set_intrinsics(
    width=WIDTH, height=HEIGHT,
    fx=dict_toml_depth_intrinsic["fx"], 
    fy=dict_toml_depth_intrinsic["fy"], 
    cx=dict_toml_depth_intrinsic["cx"], 
    cy=dict_toml_depth_intrinsic["cy"]
)

# Load Extrinsic Parameters
translation_elem = ["tx", "ty", "tz"]
rotation_elem = ["r11", "r12", "r13",
                 "r21", "r22", "r23",
                 "r31", "r32", "r33"]
translation = np.array([dict_toml_extrinsic[_elem]
                        for _elem in translation_elem])
rotation_dcm = np.asarray([dict_toml_extrinsic[_elem]
                           for _elem in rotation_elem]).reshape([3, 3])
extrinsic_mat = np.eye(4)
extrinsic_mat[:3, :3] = rotation_dcm
extrinsic_mat[:3, 3] = translation


color_raw = o3d.io.read_image("{}/data/color/000000.png".format(SCRIPT_DIR))
depth_raw = o3d.io.read_image("{}/data/depth/000000.png".format(SCRIPT_DIR))

rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw)


pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
    rgbd_image,
    intrinsic=camera_intrinsic,
    extrinsic=extrinsic_mat
)
o3d.visualization.draw_geometries([pcd])
