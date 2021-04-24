# MIT License.
# Copyright (c) 2021 by BioicDL. All rights reserved.
# Created by LiuXb on 2021/1/31
# -*- coding:utf-8 -*-

"""
@Modified: 
@Description:
"""
import sys
import os

# append the DeepClawDev directory to python path and set it as working directory
_root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(_root_path)
os.chdir(_root_path)
print('work_dir: ', _root_path)

import time, cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

from deepclaw.driver.arms.ArmController import ArmController
from deepclaw.modules.end2end.yolov5.YOLO5 import Yolo5
from deepclaw.driver.sensors.camera.Realsense_L515 import Realsense

from deepclaw.modules.calibration.Calibration2D import Calibration2D
from deepclaw.driver.arms.Auboi5Controller import AuboController

import serial, time


# sudo chmod +777 ttyUSB0
class ComSwitch(object):
    def __init__(self, com="/dev/ttyUSB0"):
        self.__open_cmd = [0xA0, 0x01, 0x01, 0xA2]
        self.__close_cmd = [0xA0, 0x01, 0x00, 0xA1]
        self.sc = serial.Serial(port=com, baudrate=9600)

    def open(self):
        self.sc.write(self.__open_cmd)

    def close(self):
        self.sc.write(self.__close_cmd)


def pick_place(robot_server: ArmController, gripper_server: object, home_joint, pick_xyzrt, place_xyzrt):
    cs = ComSwitch()
    cs.open()
    # go to pick above
    up_pos = pick_xyzrt.copy()
    up_pos[2] = up_pos[2] + 0.1

    robot_server.move_p(up_pos, 1.5, 1.5)
    time.sleep(0.01)
    # go to pick
    robot_server.move_p(pick_xyzrt, 1.5, 1.5)
    # pick
    cs.close()
    time.sleep(1)
    # go up
    robot_server.move_p(up_pos, 1.5, 1.5)
    time.sleep(0.01)
    print('##############################')

    # go to release
    robot_server.move_p(place_xyzrt, 2.5, 2.5)
    print('##############################')
    # release
    cs.open()
    time.sleep(0.8)
    # go back home
    robot_server.move_j(home_joint, 1.5, 1.5)
    time.sleep(0.01)


def detectObject(detector_algo: Yolo5, crop_bounding=[172, 470, 360, 886]):
    region_class = detector_algo.detect(color)
    if region_class is None:
        return False, None, None, None
    print(crop_bounding)
    # object on the tray
    uv_roi = []
    cla_roi = []
    cfi_roi = []
    for i in range(len(region_class)):
        uv_temp = np.array(region_class[i][0:4], np.int16)
        cla_temp = int(region_class[i][5])
        cfi_temp = region_class[i][4]
        # col_min, row_min, col_max, row_max
        uv_mid_c = (uv_temp[0] + uv_temp[2]) / 2.0
        uv_mid_r = (uv_temp[1] + uv_temp[3]) / 2.0
        if uv_mid_c < crop_bounding[2] - 20 or uv_mid_c > crop_bounding[3] + 10 or uv_mid_r < crop_bounding[
            0] - 20 or uv_mid_r > crop_bounding[1] + 10:
            continue
        else:
            uv_roi.append(uv_temp)
            cla_roi.append(cla_temp)
            cfi_roi.append(cfi_temp)

    if len(uv_roi) == 0:
        return False, None, None, None

    # pick one object once
    uv = uv_roi[0]
    cla = cla_roi[0]
    cfi = cfi_roi[0]
    return True, uv, cla, cfi


if __name__ == '__main__':
    """ Initialization """
    # camera and robot driver
    robot = AuboController('./configs/basic_config/robot_auboi5.yaml')
    camera = Realsense('./configs/basic_config/camera_rs_d435.yaml')
    object_detector = Yolo5('./configs/basic_config/yolov5_cfg.yaml')

    # home_joints = [0, 0.7, 2.0, -0.2, 1.57, 0]
    home_joints = [-0.6788985133171082, 0.31800785660743713, 1.7671029567718506, -0.1122966781258583,
                   1.5715042352676392, 0.2295781970024109]
    robot.move_j(home_joints, 1.5, 1.5)

    """ start picking loop"""
    place_xyzrt = [0.5, -0.4, 0.45, 3.14, 0, 0]
    crop_bounding = [172, 470, 360, 886]
    cali_path = './configs/basic_config/cali2D.yaml'
    for n in range(20):
        frame = camera.get_frame()
        color = frame.color_image[0]
        # crop_image = color[crop_bounding[0]:crop_bounding[1], crop_bounding[2]:crop_bounding[3]]
        # cv2.imshow("crop image", crop_image)
        # cv2.waitKey()
        # exit()

        # 识别物体
        # region_class = object_detector.detect(color)
        ret, uv, cla, cfi = detectObject(object_detector, crop_bounding)
        if not ret:
            continue
        if cla not in [0, 1, 2, 3]:
            print('\033[1;35m Error Category \033[0m!')
            continue

        ret = cv2.rectangle(color, (uv[0], uv[1]), (uv[2], uv[3]), (255, 255, 0), 2)
        ret = cv2.putText(color, '{}'.format(n),
                          (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                          (255, 0, 255), 1)
        cv2.imshow('Result', color)
        # if cv2.waitKey(1) & 0xFF == ord('q'): break
        # continue
        """ generate grasping pose"""
        hand_eye = Calibration2D(cali_path)
        # col
        ux = (uv[0] + uv[2]) / 2.0
        # row
        vy = (uv[1] + uv[3]) / 2.0

        temp = hand_eye.cvt(ux, vy)
        z = 0.38
        if abs(uv[2] - uv[0]) >= abs(uv[3] - uv[1]):
            angle = 0.785
        else:
            angle = -0.785
        # grasp pose in euler angle
        temp_pose = [temp[0], temp[1], z, 3.14, 0, angle]
        # transfer to rotation vector
        # r = R.from_euler('xyz', temp_pose[3:6], degrees=False)
        # rvc = r.as_rotvec()0
        # pick_pose = [temp_pose[0], temp_pose[1], temp_pose[2], rvc[0], rvc[1], rvc[2] + 0.785]
        # grasping
        print(temp_pose, place_xyzrt)
        pick_place(robot, robot, home_joints, temp_pose, place_xyzrt)
        break
        if cv2.waitKey(1) & 0xFF == ord('q'): break

cv2.destroyAllWindows()
