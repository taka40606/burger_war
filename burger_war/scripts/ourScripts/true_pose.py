#! /usr/bin/env python
# -*- coding: utf-8 -*-


import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import GetModelState
import numpy as np


def getStatus(service_name="/gazebo/get_model_state", model_name="red_bot"):
    rospy.init_node('get_pose')
    state = ModelState()

    rospy.wait_for_service(service_name)
    
    try:
        # cliant作成
        get_state_cliant = rospy.ServiceProxy(service_name, GetModelState)
        response = get_state_cliant(model_name, "")
        state = response

    except rospy.ServiceException, e:
        print("Service call failed: " + str(e))
    
    return state


def main():
    print("* red_bot *")
    print(getStatus(model_name="red_bot"))
    print("\n")
    print("* blue_bot *")
    print(getStatus(model_name="blue_bot"))


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
