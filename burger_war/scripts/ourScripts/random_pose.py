#! /usr/bin/env python
# -*- coding: utf-8 -*-


import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
import numpy as np



def isFieldOut(check_x, check_y, enemy_robot_x=0, enemy_robot_y=0, robot_radius = 0.1):
    wall_length = 2.4 # [m]
    wall_length_half = wall_length / 2 # [m]
    wall_thickness = 0.05 # [m] 壁の厚さ(gazeboで確認)
    wall_thickness_half = wall_thickness / 2 # [m]
    # 障害物の[位置x, 位置y, 横長さ, 縦長さ]
    obstacle_pos = np.array([[      0,     0, 0.35/2, 0.35/2],
                             [ 0.53,  0.53, 0.2/2,  0.15/2],
                             [-0.53,  0.53, 0.2/2,  0.15/2],
                             [ 0.53, -0.53, 0.2/2,  0.15/2],
                             [-0.53, -0.53, 0.2/2,  0.15/2],
                             [enemy_robot_x, enemy_robot_y, robot_radius,  robot_radius]])


    # 障害物の当たり判定
    for i in range(obstacle_pos.shape[0]):
        lower_x = obstacle_pos[i][0] - obstacle_pos[i][2] - robot_radius
        upper_x = obstacle_pos[i][0] + obstacle_pos[i][2] + robot_radius
        lower_y = obstacle_pos[i][1] - obstacle_pos[i][3] - robot_radius
        upper_y = obstacle_pos[i][1] + obstacle_pos[i][3] + robot_radius
        # 障害物にぶつかる場合
        if isClose(check_x, lower_x, upper_x) and isClose(check_y, lower_y, upper_y):
            return True
    
    # フィールドの範囲内にいるかどうか判定
    check_rot_x, check_rot_y = rotation(check_x, check_y, np.pi/4)
    
    wall_lower_x = -wall_length_half + wall_thickness_half + robot_radius
    wall_upper_x =  wall_length_half - wall_thickness_half - robot_radius
    wall_lower_y = -wall_length_half + wall_thickness_half + robot_radius
    wall_upper_y =  wall_length_half - wall_thickness_half - robot_radius
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


def randomRespown(service_name="/gazebo/set_model_state", model_name="red_bot"):
    rospy.init_node('set_pose')
    robot_radius = 0.2 # [m]
    wall_rength = 2.4 + 0.1 # [m]
    field_length_half = wall_rength / np.sqrt(2)

    # Model Angler
    rand_angle = (np.random.rand() - 0.5) * np.pi * 2 # [rad]
    # Model Position
    while True:
        pos_x = (np.random.rand() - 0.5) * 2 * field_length_half # [m]
        pos_y = (np.random.rand() - 0.5) * 2 * field_length_half # [m]
        if isFieldOut(pos_x, pos_y) == False:
            break

    respown(pos_x, pos_y, rand_angle, model_name=model_name)


def respown(pos_x, pos_y, yaw, service_name="/gazebo/set_model_state", model_name="red_bot"):
    state_msg = ModelState()
    state_msg.model_name = model_name
    state_msg.pose.position.x = pos_x
    state_msg.pose.position.y = pos_y
    state_msg.pose.position.z = 0.01
    #state_msg.pose.orientation.x = 0.0
    #state_msg.pose.orientation.y = 0.0

    #state_msg.twist.angular.z = yaw
    state_msg.pose.orientation.z = np.sin(yaw / 2.);
    state_msg.pose.orientation.w = np.cos(yaw / 2.);


    rospy.wait_for_service(service_name)
    try:
        set_state = rospy.ServiceProxy(service_name, SetModelState)
        resp = set_state( state_msg )

    except rospy.ServiceException, e:
        print("Service call failed: " + str(e))


def main():
    randomRespown("/gazebo/set_model_state", "red_bot")
    randomRespown("/gazebo/set_model_state", "blue_bot")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
