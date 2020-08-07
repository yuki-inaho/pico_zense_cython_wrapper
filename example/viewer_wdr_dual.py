import sys
import cv2
import click
import numpy as np
import os
import cvui
import toml
import shutil
import glob
from collections import deque

NAMESPACE = 'merge_near_far'
NEAR = 'depth_near'
FAR = 'depth_far'
DEPTH_SHAPE = (480, 640)
DEPTH_SIZE = DEPTH_SHAPE[0] * DEPTH_SHAPE[1]

sys.path.append("/app/scripts")
from zense_pywrapper import PyPicoZenseManager

N_ZENSE = 2
WDR_RANGE1_PULSE_COUNT = 160
WDR_RANGE2_PULSE_COUNT = 400

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


class IntrinsicParam:
    def __init__(self):
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None


class MergeDepthImage(object):
    def __init__(self, merge_queue_size=1):
        assert merge_queue_size > 0
        self.merge_queue_size = merge_queue_size
        self.merged_depth_img = None
        self.depth_deque = deque(maxlen=self.merge_queue_size)

    def get_queue_size(self):
        return self.merge_queue_size

    def clear_queue(self):
        self.depth_deque.clear()

    def get_median(self, _deque):
        depth_stk = np.array(list(_deque))
        sort_idx = np.argsort(depth_stk, axis=0)
        n_filled = (depth_stk > 0).sum(axis=0)
        median_idx_in_sorted = -(n_filled / 2) + (self.merge_queue_size - 1)
        median_idx = sort_idx.reshape(self.merge_queue_size, -1)[
            median_idx_in_sorted.reshape(-1).astype(np.uint64),
            np.arange(DEPTH_SIZE)
        ]
        depth_median = (depth_stk.reshape(self.merge_queue_size, -1)[
            median_idx, np.arange(DEPTH_SIZE)
        ]).reshape(DEPTH_SHAPE)
        return depth_median

    def update(self, depth_image):
        self.depth_deque.append(depth_image)
        n_contain_elem = len(self.depth_deque)
        if n_contain_elem == self.merge_queue_size:
            self.merged_depth_img = self.get_median(self.depth_deque)
            self.clear_queue()
            return True
        else:
            return False

    @property
    def merged_depth_image(self):
        assert self.merged_depth_img is not None
        return self.merged_depth_img


class WDRImageMerger(object):
    def __init__(self, merge_queue_size=1):
        assert merge_queue_size > 0
        self.merge_queue_size = merge_queue_size
        self.merged_depth_img_wdr = None
        self.range1_deque = deque(maxlen=self.merge_queue_size)
        self.range2_deque = deque(maxlen=self.merge_queue_size)

    def clear_queue(self):
        self.range1_deque.clear()
        self.range2_deque.clear()

    def get_median(self, _deque):
        depth_stk = np.array(list(_deque))
        sort_idx = np.argsort(depth_stk, axis=0)
        n_filled = (depth_stk > 0).sum(axis=0)
        median_idx_in_sorted = -(n_filled / 2) + (self.merge_queue_size - 1)
        median_idx = sort_idx.reshape(self.merge_queue_size, -1)[
            median_idx_in_sorted.reshape(-1).astype(np.uint64),
            np.arange(DEPTH_SIZE)
        ]
        depth_median = (depth_stk.reshape(self.merge_queue_size, -1)[
            median_idx, np.arange(DEPTH_SIZE)
        ]).reshape(DEPTH_SHAPE)
        return depth_median

    def get_merged(self, range1_img, range2_img):
        bflg_range1_empty = (range1_img == 0)
        depth_merged_median = range1_img.copy()
        depth_merged_median[bflg_range1_empty] = range2_img[bflg_range1_empty]
        return depth_merged_median

    def update(self, depth_image_range1, depth_image_range2):
        self.range1_deque.append(depth_image_range1)
        self.range2_deque.append(depth_image_range2)
        assert len(self.range1_deque) == len(self.range2_deque)
        n_contain_elem = len(self.range1_deque)
        if n_contain_elem == self.merge_queue_size:
            range1_depth_img_median = self.get_median(self.range1_deque)
            range2_depth_img_median = self.get_median(self.range2_deque)
            self.merged_depth_img_wdr = self.get_merged(range1_depth_img_median, range2_depth_img_median)
            self.clear_queue()
            return True
        else:
            return False

    @property
    def merged_depth_image(self):
        assert self.merged_depth_img_wdr is not None
        return self.merged_depth_img_wdr


def is_wdr_enabled(camera_name, cfg_path):
    toml_dict = toml.load(open(cfg_path))
    isWDR = int(toml_dict[camera_name]["range1"]) >= 0 and \
        int(toml_dict[camera_name]["range2"]) >= 0
    isRGB = int(toml_dict[camera_name]["rgb_image"]) == 1
    do_exit = False
    if not isWDR:
        print("Current camera setting is WDR disabled mode. This app can be executed under WDR enabled setting")
        do_exit = True
    if isRGB:
        print("Current camera setting is RGB enabled mode. This app can be executed under RGB disabled setting")
        do_exit = True
    if do_exit:
        assert False


def colorize_depth(img, max_var=2000):
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



def open_dualzense(toml_path):
    dict_toml = toml.load(toml_path)
    zense_manager_list = [PyPicoZenseManager(i) for i in range(2)]
    left_zense_sn = dict_toml["Camera0"]["serial_no"]
    right_zense_sn = dict_toml["Camera1"]["serial_no"]
    serial_number_list = [_mng.serial_number for _mng in zense_manager_list]
    left_zense_index = serial_number_list.index(left_zense_sn)
    right_zense_index = serial_number_list.index(right_zense_sn)
    
    zense_mng_left = zense_manager_list[left_zense_index]
    zense_mng_left.set_device_mode_from_config(toml_path, "Camera0")
    zense_mng_right = zense_manager_list[right_zense_index]
    zense_mng_right.set_device_mode_from_config(toml_path, "Camera1")
    return zense_mng_left, zense_mng_right


def get_wdr_depth_range(toml_path, camera_name):
    dict_toml = toml.load(toml_path)
    range1 = dict_toml[camera_name]["range1"]
    range2 = dict_toml[camera_name]["range2"]
    return range1, range2


def get_intrinsic_param_zense(toml_path, camera_name):
    dict_toml = toml.load(toml_path)
    fx = dict_toml["{}_Factory".format(camera_name)]["fx"]
    fy = dict_toml["{}_Factory".format(camera_name)]["fy"]
    cx = dict_toml["{}_Factory".format(camera_name)]["cx"]
    cy = dict_toml["{}_Factory".format(camera_name)]["cy"]
    intrinsic_zense_depth = IntrinsicParam()
    intrinsic_zense_depth.fx = fx
    intrinsic_zense_depth.fy = fy
    intrinsic_zense_depth.cx = cx
    intrinsic_zense_depth.cy = cy
    return intrinsic_zense_depth


@click.command()
@click.option("--toml-path", "-t", default="{}/../cfg/dualzense.toml".format(SCRIPT_DIR))
@click.option("--save_dir", "-s", default="{}/../data".format(SCRIPT_DIR))
def main(toml_path, save_dir):
    is_wdr_enabled("Camera0", toml_path)
    is_wdr_enabled("Camera1", toml_path)

    zense_mng_left, zense_mng_right = open_dualzense(toml_path)
    l_range1, l_range2 = get_wdr_depth_range(toml_path, "Camera0")
    r_range1, r_range2 = get_wdr_depth_range(toml_path, "Camera1")

    intrinsic_l = get_intrinsic_param_zense(toml_path, "Camera0")
    intrinsic_r = get_intrinsic_param_zense(toml_path, "Camera1")
    depth_merger_left = WDRImageMerger()
    depth_merger_right = WDRImageMerger()

    WINDOW_NAME = "Capture"
    cvui.init(WINDOW_NAME)
    while True:
        key = cv2.waitKey(10)
        frame = np.zeros((900, 1600, 3), np.uint8)
        frame[:] = (49, 52, 49)
        status = zense_mng_left.update()
        status &= zense_mng_right.update()
        cvui.text(frame, 650, 10, 'Zense Left', 0.5)
        cvui.text(frame, 1130, 10, 'Zense Right', 0.5)
        if status:
            depth_img_l1 = zense_mng_left.depth_image_range1.copy()
            depth_img_l2 = zense_mng_left.depth_image_range2.copy()
            depth_img_l1[depth_img_l1 == 65535] = 0
            depth_img_l2[depth_img_l2 == 65535] = 0
            depth_merger_left.update(depth_img_l1, depth_img_l2)
            merged_depth_l = depth_merger_left.merged_depth_image

            depth_img_r1 = zense_mng_right.depth_image_range1.copy()
            depth_img_r2 = zense_mng_right.depth_image_range2.copy()
            depth_img_r1[depth_img_r1 == 65535] = 0
            depth_img_r2[depth_img_r2 == 65535] = 0
            depth_merger_right.update(depth_img_r1, depth_img_r2)
            merged_depth_r = depth_merger_right.merged_depth_image

            depth_l_color = colorize_depth(merged_depth_l)
            depth_l_color = cv2.rotate(depth_l_color, cv2.ROTATE_90_CLOCKWISE)
            frame[30:670, 640:1120, :] = depth_l_color

            depth_r_color = colorize_depth(merged_depth_r)
            depth_r_color = cv2.rotate(depth_r_color, cv2.ROTATE_90_CLOCKWISE)
            frame[30:670, 1120:1600, :] = depth_r_color

        if key == 27:
            break
        if key == ord('q'):
            break
        cvui.update()
        cvui.imshow(WINDOW_NAME, frame)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()