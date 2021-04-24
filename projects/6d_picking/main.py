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
import serial

_root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(_root_path)
os.chdir(_root_path)
print('work_dir: ', _root_path)

import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

from deepclaw.driver.arms.ArmController import ArmController
from deepclaw.modules.end2end.yolov5.YOLO5 import Yolo5
from deepclaw.driver.sensors.camera.Realsense_L515 import Realsense
from deepclaw.driver.arms.Auboi5Controller import AuboController
from deepclaw.modules.grasp_planning.GeoGrasp import GeoGrasp

import serial, time


class ComSwitch(object):
    def __init__(self, com="/dev/ttyUSB0"):
        self.__open_cmd = [0xA0, 0x01, 0x01, 0xA2]
        self.__close_cmd = [0xA0, 0x01, 0x00, 0xA1]
        self.sc = serial.Serial(port=com, baudrate=9600)

    def open(self):
        self.sc.write(self.__open_cmd)

    def close(self):
        self.sc.write(self.__close_cmd)


def pick_place(robot_server: ArmController, gripper_server: object, home_joint, pick_xyzrt, pick_xyzrt_up, place_xyzrt):
    # go to pick above
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


def detectObject(detector_algo: Yolo5, color, crop_bounding=[220, 451, 393, 970]):
    region_class = detector_algo.detect(color)
    if region_class is None:
        return False, None, None, None

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
        print('location: ', uv_mid_c, uv_mid_r)
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

    home_joints = [0, 0.7, 2.0, -0.2, 1.57, 0]
    robot.move_j(home_joints, 1.5, 1.5)

    """ start picking loop"""
    place_xyzrt = [0.5, -0.4, 0.5, 3.14, 0, 0]
    crop_bounding = [220, 451, 393, 970]
    hand_eye_matrix = np.load('./configs/E_T_B.npy')
    while 1:
        frame = camera.get_frame()
        color = frame.color_image[0]
        depth_img = frame.depth_image[0]
        # 识别物体
        # region_class = object_detector.detect(color)
        ret, uv, cla, cfi = detectObject(object_detector, color, crop_bounding)
        if not ret:
            continue
        if cla not in [0, 1, 2, 3]:
            print('\033[1;35m Error Category \033[0m!')
            continue
        """ generate grasping pose"""
        # transfer to point cloud
        # depth intrinsics
        width = 1280
        hight = 720
        fx = 640.983
        fy = 640.983
        cx = 641.114
        cy = 368.461
        pc = []
        for i in range(uv[1], uv[3]):
            temp_pc = []
            for j in range(uv[0], uv[2]):
                z = depth_img[i][j]
                x = (j - cx) * z / fx
                y = (i - cy) * z / fy
                if z == 0:
                    temp_pc.append([np.nan, np.nan, np.nan])
                else:
                    temp_pc.append([x, y, z])
            pc.append(temp_pc)
        pc = np.array(pc).reshape((-1, 3))
        # filter plane

        plane_model = [0.02510424, 0.0505177, 0.62201613, -0.7809697]
        new_pc = []
        for i in range(len(pc)):
            if pc[i][0] * plane_model[0] + pc[i][1] * plane_model[1] + pc[i][2] * plane_model[2] + plane_model[3] < 0:
                new_pc.append(pc[i])

        new_pc = np.array(new_pc)
        show_flag = 1
        if show_flag:
            fig = plt.figure("pcs")
            ax = fig.add_subplot(111, projection="3d")
            ax.scatter(new_pc[:, 0], new_pc[:, 1], new_pc[:, 2], s=1, )
            ax.set_xlabel("x")
            ax.set_ylabel("y")
            ax.set_zlabel("z")
            ax.set_xlim(-1, 1)
            ax.set_ylim(-1, 1)
            ax.set_zlim(0.5, 2)
            ax.view_init(elev=-90, azim=-90)
            plt.show()
        # GeoGrasp
        if len(new_pc) == 0:
            continue
        m = GeoGrasp.run(new_pc)
        m = np.array(m)
        # row
        grasp_matrix = np.reshape(m, (4, 4), order='C')
        print('==============original grasp matrix=====================')
        print(grasp_matrix)
        if np.all(grasp_matrix == 0):
            continue

        temp_matrix = grasp_matrix.copy()
        grasp_matrix[0:3, 0] = temp_matrix[0:3, 1] / np.linalg.norm(temp_matrix[0:3, 1])  # 1
        grasp_matrix[0:3, 1] = temp_matrix[0:3, 2] / np.linalg.norm(temp_matrix[0:3, 2])  # 2
        grasp_matrix[0:3, 2] = temp_matrix[0:3, 0] / np.linalg.norm(temp_matrix[0:3, 0])  # 0
        print('==============modified axis grasp matrix=====================')
        print(grasp_matrix)

        # load eye to base matrix
        hand_eye_matrix = np.array(hand_eye_matrix)
        print('=========== hand_eye_calib ==============')
        print(hand_eye_matrix)
        grasping_in_base = np.dot(hand_eye_matrix, grasp_matrix)
        print('=========== grasping_in_base ==============')
        print(grasping_in_base)

        rot_matrix = grasping_in_base[0:3, 0:3]
        if rot_matrix[2, 2] > 0:
            rot_matrix[0:3, 2] = -rot_matrix[0:3, 2]
            rot_matrix[0:3, 1] = -rot_matrix[0:3, 1]
            grasping_in_base[0:3, 0:3] = rot_matrix
        print('=========== rot_matrix ==============')
        print(rot_matrix)
        print('=========== modified -z grasping_in_base ==============')
        print(grasping_in_base)
        # gripper to flange matrix
        E_T_F = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.133], [0, 0, 0, 1]])
        # grasping_in_base = np.dot(grasping_in_base, E_T_F)
        grasping_in_base = np.dot(grasping_in_base, np.linalg.inv(E_T_F))
        r = R.from_matrix(grasping_in_base[0: 3, 0: 3])
        rot = r.as_euler('xyz', degrees=False)
        transfer = grasping_in_base[0:3, 3]
        measure_z = 0.4
        # measure_endtransfer = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.4], [0, 0, 0, 1]])
        # rot_x = np.array([[1, 0, 0, 0], [0, np.cos(rot[0]), -np.sin(rot[0]), 0], [0, np.sin(rot[0]), np.cos(rot[0]), 0], [0, 0, 0, 1]])
        # rot_y = np.array([[np.cos(rot[1]), 0,  np.sin(rot[1]), 0], [0, 1, 0, 0], [-np.sin(rot[1]), 0, np.cos(rot[1]), 0], [0, 0, 0, 1]])
        # rot_z = np.array([[np.cos(rot[2]), -np.sin(rot[2]), 0, 0], [np.sin(rot[2]), np.cos(rot[2]), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        # measure_world = np.dot(rotzx, rot_y)
        #
        # measure_end = np.dot(measure_world, measure_endtransfer)
        # print('Origin', measure_world)
        # print('endshift', measure_end)

        temp_pose = [transfer[0], transfer[1], transfer[2]+0.25, rot[0], rot[1], rot[2]]
        # temp_pose = [measure_end[0][3], measure_end[1][3], measure_end[2][3], rot[0], rot[1], rot[2]]
        print('grabpose', temp_pose)
        # z offset ,  pick up
        E_T_F = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -0.03], [0, 0, 0, 1]])
        grasping_in_base = np.dot(grasping_in_base, E_T_F)
        r = R.from_matrix(grasping_in_base[0: 3, 0: 3])
        rot = r.as_euler('xyz', degrees=False)
        transfer = grasping_in_base[0:3, 3]
        temp_pose_up = [transfer[0], transfer[1], transfer[2] + 0.3, rot[0], rot[1], rot[2]]

        print('grasp pose: ', temp_pose)
        print('grasp pose: ', np.array(temp_pose[3:6]) * 180 / 3.14159)

        # # transfer to rotation vector
        # r = R.from_euler('xyz', temp_pose[3:6], degrees=False)
        # rvc = r.as_rotvec()
        # pick_pose = [temp_pose[0], temp_pose[1], temp_pose[2], rvc[0], rvc[1], rvc[2]]
        # pick_pose_up = [temp_pose_up[0], temp_pose_up[1], temp_pose_up[2], rvc[0], rvc[1], rvc[2]]
        # print(temp_pose)
        # print(temp_pose_up)

        # 抓取
        pick_place(robot, robot, home_joints, temp_pose, temp_pose_up, place_xyzrt)



# measure_endtransfer = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.4], [0, 0, 0, 1]])
        # rot_x = np.array([[1, 0, 0, 0], [0, np.cos(rot[0]), -np.sin(rot[0]), 0], [0, np.sin(rot[0]), np.cos(rot[0]), 0], [0, 0, 0, 1]])
        # rot_y = np.array([[np.cos(rot[1]), 0,  np.sin(rot[1]), 0], [0, 1, 0, 0], [-np.sin(rot[1]), 0, np.cos(rot[1]), 0], [0, 0, 0, 1]])
        # rot_z = np.array([[np.cos(rot[2]), -np.sin(rot[2]), 0, 0], [np.sin(rot[2]), np.cos(rot[2]), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        # measure_world = np.dot(rotzx, rot_y)
        #
        # measure_end = np.dot(measure_world, measure_endtransfer)
        # print('Origin', measure_world)
        # print('endshift', measure_end)



# # MIT License.
# # Copyright (c) 2021 by BioicDL. All rights reserved.
# # Created by LiuXb on 2021/1/31
# # -*- coding:utf-8 -*-
#
# """
# @Modified:
# @Description:
# """
# import time
# import numpy as np
# import matplotlib.pyplot as plt
# from scipy.spatial.transform import Rotation as R
#
# from deepclaw.driver.arms.ArmController import ArmController
# from deepclaw.modules.end2end.yolov5.YOLO5 import Yolo5
# from deepclaw.driver.sensors.camera.Realsense_L515 import Realsense
# from deepclaw.driver.arms.Auboi5Controller import AuboController
# from deepclaw.modules.grasp_planning.GeoGrasp import GeoGrasp
#
# import serial, time
#
#
# class ComSwitch(object):
#     def __init__(self, com="/dev/ttyUSB0"):
#         self.__open_cmd = [0xA0, 0x01, 0x01, 0xA2]
#         self.__close_cmd = [0xA0, 0x01, 0x00, 0xA1]
#         self.sc = serial.Serial(port=com, baudrate=9600)
#
#     def open(self):
#         self.sc.write(self.__open_cmd)
#
#     def close(self):
#         self.sc.write(self.__close_cmd)
#
#
# def pick_place(robot_server: ArmController, gripper_server: object, home_joint, pick_xyzrt, pick_xyzrt_up, place_xyzrt):
#     # go to pick above
#     cs = ComSwitch()
#     cs.open()
#     # go to pick above
#     up_pos = pick_xyzrt.copy()
#     up_pos[2] = up_pos[2] + 0.1
#
#     robot_server.move_p(up_pos, 1.5, 1.5)
#     time.sleep(0.01)
#     # go to pick
#     robot_server.move_p(pick_xyzrt, 1.5, 1.5)
#     # pick
#     cs.close()
#     time.sleep(1)
#     # go up
#     robot_server.move_p(up_pos, 1.5, 1.5)
#     time.sleep(0.01)
#     print('##############################')
#
#     # go to release
#     robot_server.move_p(place_xyzrt, 2.5, 2.5)
#     print('##############################')
#     # release
#     cs.open()
#     time.sleep(0.8)
#     # go back home
#     robot_server.move_j(home_joint, 1.5, 1.5)
#     time.sleep(0.01)
#
#
# def detectObject(detector_algo: Yolo5, color, crop_bounding=[220, 451, 393, 970]):
#     region_class = detector_algo.detect(color)
#     if region_class is None:
#         return False, None, None, None
#
#     # object on the tray
#     uv_roi = []
#     cla_roi = []
#     cfi_roi = []
#     for i in range(len(region_class)):
#         uv_temp = np.array(region_class[i][0:4], np.int16)
#         cla_temp = int(region_class[i][5])
#         cfi_temp = region_class[i][4]
#         # col_min, row_min, col_max, row_max
#         uv_mid_c = (uv_temp[0] + uv_temp[2]) / 2.0
#         uv_mid_r = (uv_temp[1] + uv_temp[3]) / 2.0
#         print('location: ', uv_mid_c, uv_mid_r)
#         if uv_mid_c < crop_bounding[2] - 20 or uv_mid_c > crop_bounding[3] + 10 or uv_mid_r < crop_bounding[
#             0] - 20 or uv_mid_r > crop_bounding[1] + 10:
#             continue
#         else:
#             uv_roi.append(uv_temp)
#             cla_roi.append(cla_temp)
#             cfi_roi.append(cfi_temp)
#
#     if len(uv_roi) == 0:
#         return False, None, None, None
#
#     # pick one object once
#     uv = uv_roi[0]
#     cla = cla_roi[0]
#     cfi = cfi_roi[0]
#     return True, uv, cla, cfi
#
#
# if __name__ == '__main__':
#     """ Initialization """
#     # camera and robot driver
#     robot = AuboController('./configs/basic_config/robot_auboi5.yaml')
#     camera = Realsense('./configs/basic_config/camera_rs_d435.yaml')
#     object_detector = Yolo5('./configs/basic_config/yolov5_cfg.yaml')
#
#     home_joints = [0, 0.7, 2.0, -0.2, 1.57, 0]
#     robot.move_j(home_joints, 1.5, 1.5)
#
#     """ start picking loop"""
#     place_xyzrt = [0.5, -0.4, 0.3, 3.14, 0, 0]
#     crop_bounding = [300, 720, 300, 1000]
#     hand_eye_matrix = np.load('./configs/ICRA2020-ur5-azure-rg6/E_T_B.npy')
#     while 1:
#         frame = camera.get_frame()
#         color = frame.color_image[0]
#         depth_img = frame.depth_image[0]
#         # 识别物体
#         # region_class = object_detector.detect(color)
#         ret, uv, cla, cfi = detectObject(object_detector, crop_bounding=[300, 720, 300, 1000])
#         if not ret:
#             continue
#         if cla not in [0, 1, 2, 3]:
#             print('\033[1;35m Error Category \033[0m!')
#             continue
#         """ generate grasping pose"""
#         # transfer to point cloud
#         # depth intrinsics
#         width = 1280
#         hight = 720
#         fx = 640.983
#         fy = 640.983
#         cx = 641.114
#         cy = 368.461
#         pc = []
#         for i in range(uv[1], uv[3]):
#             temp_pc = []
#             for j in range(uv[0], uv[2]):
#                 z = depth_img[i][j]
#                 x = (j - cx) * z / fx
#                 y = (i - cy) * z / fy
#                 if z == 0:
#                     temp_pc.append([np.nan, np.nan, np.nan])
#                 else:
#                     temp_pc.append([x, y, z])
#             pc.append(temp_pc)
#         pc = np.array(pc).reshape((-1, 3))
#         # filter plane
#         plane_model = [-0.07, 0.25, 0.97, -0.98]
#         new_pc = []
#         for i in range(len(pc)):
#             if pc[i][0]*plane_model[0] + pc[i][1]*plane_model[1] + pc[i][2]*plane_model[2] + plane_model[3] < 0:
#                 new_pc.append(pc[i])
#
#         new_pc = np.array(new_pc)
#         # if is_debug:
#         #     fig = plt.figure("pcs")
#         #     ax = fig.add_subplot(111, projection="3d")
#         #     ax.scatter(new_pc[:, 0], new_pc[:, 1], new_pc[:, 2], s=1,)
#         #     ax.set_xlabel("x")
#         #     ax.set_ylabel("y")
#         #     ax.set_zlabel("z")
#         #     ax.set_xlim(-1, 1)
#         #     ax.set_ylim(-1, 1)
#         #     ax.set_zlim(0.5, 2)
#         #     ax.view_init(elev=-90, azim=-90)
#         #     plt.show()
#         # GeoGrasp
#         if len(new_pc) == 0:
#             continue
#         m = GeoGrasp.run(new_pc)
#         m = np.array(m)
#         # row
#         grasp_matrix = np.reshape(m, (4, 4), order='C')
#         print('==============original grasp matrix=====================')
#         print(grasp_matrix)
#         if np.all(grasp_matrix == 0):
#             continue
#
#         temp_matrix = grasp_matrix.copy()
#         grasp_matrix[0:3, 0] = temp_matrix[0:3, 1] / np.linalg.norm(temp_matrix[0:3, 1]) # 1
#         grasp_matrix[0:3, 1] = temp_matrix[0:3, 2] / np.linalg.norm(temp_matrix[0:3, 2]) # 2
#         grasp_matrix[0:3, 2] = temp_matrix[0:3, 0] / np.linalg.norm(temp_matrix[0:3, 0]) # 0
#         print('==============modified axis grasp matrix=====================')
#         print(grasp_matrix)
#
#         # load eye to base matrix
#         hand_eye_matrix = np.array(hand_eye_matrix)
#         print('=========== hand_eye_calib ==============')
#         print(hand_eye_matrix)
#         grasping_in_base = np.dot(hand_eye_matrix, grasp_matrix)
#         print('=========== grasping_in_base ==============')
#         print(grasping_in_base)
#
#         rot_matrix = grasping_in_base[0:3, 0:3]
#         if rot_matrix[2, 2] > 0:
#             rot_matrix[0:3, 2] = -rot_matrix[0:3, 2]
#             rot_matrix[0:3, 1] = -rot_matrix[0:3, 1]
#             grasping_in_base[0:3, 0:3] = rot_matrix
#         print('=========== rot_matrix ==============')
#         print(rot_matrix)
#         print('=========== modified -z grasping_in_base ==============')
#         print(grasping_in_base)
#         # gripper to flange matrix
#         E_T_F = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.133], [0, 0, 0, 1]])
#         # grasping_in_base = np.dot(grasping_in_base, E_T_F)
#         grasping_in_base = np.dot(grasping_in_base, np.linalg.inv(E_T_F))
#
#         r = R.from_matrix(grasping_in_base[0: 3, 0: 3])
#         rot = r.as_euler('xyz', degrees=False)
#         transfer = grasping_in_base[0:3, 3]
#         temp_pose = [transfer[0], transfer[1], transfer[2] + 0.3, rot[0], rot[1], rot[2]]
#
#         # z offset ,  pick up
#         E_T_F = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -0.03], [0, 0, 0, 1]])
#         grasping_in_base = np.dot(grasping_in_base, E_T_F)
#         r = R.from_matrix(grasping_in_base[0: 3, 0: 3])
#         rot = r.as_euler('xyz', degrees=False)
#         transfer = grasping_in_base[0:3, 3]
#         temp_pose_up = [transfer[0], transfer[1], transfer[2], rot[0], rot[1], rot[2]]
#
#         print('grasp pose: ', temp_pose)
#         # # transfer to rotation vector
#         # r = R.from_euler('xyz', temp_pose[3:6], degrees=False)
#         # rvc = r.as_rotvec()
#         # pick_pose = [temp_pose[0], temp_pose[1], temp_pose[2], rvc[0], rvc[1], rvc[2]]
#         # pick_pose_up = [temp_pose_up[0], temp_pose_up[1], temp_pose_up[2], rvc[0], rvc[1], rvc[2]]
#         # 抓取
#         pick_place(robot, robot, home_joints, temp_pose, temp_pose_up, place_xyzrt)
#
#
