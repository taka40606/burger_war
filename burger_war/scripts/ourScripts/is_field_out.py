#! /usr/bin/env python
# -- coding: utf-8 --import rospy
import rospkg
import numpy as np

def isFieldOut(check_x, check_y, robot_radius = 0.12):
    clear_width = 0.05
    wall_length = 2.4 # [m]
    wall_length_half = wall_length / 2 # [m]
    wall_thickness = 0.05 # [m] 壁の厚さ(gazeboで確認)
    wall_thickness_half = wall_thickness / 2 # [m]    # 障害物の[位置x, 位置y, 横長さ, 縦長さ]　[m]
    obstacle_pos = np.array([[      0,     0, 0.35/2, 0.35/2],
                             [ 0.53,  0.53, 0.2/2,  0.15/2],
                             [-0.53,  0.53, 0.2/2,  0.15/2],
                             [ 0.53, -0.53, 0.2/2,  0.15/2],
                             [-0.53, -0.53, 0.2/2,  0.15/2]])    # 障害物の当たり判定
    for i in range(obstacle_pos.shape[0]):
        lower_x = obstacle_pos[i][0] - obstacle_pos[i][2] - robot_radius + clear_width
        upper_x = obstacle_pos[i][0] + obstacle_pos[i][2] + robot_radius - clear_width
        lower_y = obstacle_pos[i][1] - obstacle_pos[i][3] - robot_radius + clear_width
        upper_y = obstacle_pos[i][1] + obstacle_pos[i][3] + robot_radius - clear_width
        # 障害物にぶつかる場合
        if isClose(check_x, lower_x, upper_x) and isClose(check_y, lower_y, upper_y):
            return True
    
    # フィールドの範囲内にいるかどうか判定
    check_rot_x, check_rot_y = rotation(check_x, check_y, np.pi/4)
    
    wall_lower_x = -wall_length_half + wall_thickness_half + robot_radius + clear_width
    wall_upper_x =  wall_length_half - wall_thickness_half - robot_radius - clear_width
    wall_lower_y = -wall_length_half + wall_thickness_half + robot_radius + clear_width
    wall_upper_y =  wall_length_half - wall_thickness_half - robot_radius - clear_width
    if isClose(check_rot_x, wall_lower_x, wall_upper_x) and isClose(check_rot_y, wall_lower_y, wall_upper_y):
        return False
    else:
        return True
def isClose(check_val, lower, upper):
    if lower < check_val < upper:
        return True
    return False
def rotation(x, y, yaw_rad):
    rot_x = np.cos(yaw_rad) * x - np.sin(yaw_rad) * y
    rot_y = np.sin(yaw_rad) * x + np.cos(yaw_rad) * y
    return rot_x, rot_y
