#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int8
import math
import time

status=10

def movebaseStatusCallback(movebase_status):
    global status
    status=movebase_status.data
    print status

def InputPose(x,y,z):
    pose=Pose2D()
    pose.x=x
    pose.y=y
    pose.theta=z
    point_pub.publish(pose)

if __name__ == '__main__':
    point_pub = rospy.Publisher('/red_bot/point', Pose2D, queue_size=10)
    movebase_status_sub = rospy.Subscriber('/red_bot/move_base_state', Int8, movebaseStatusCallback)
    rospy.init_node('point_pub', anonymous=True)
    r = rospy.Rate(10) # 10hz
    Time=0

    InputPose(-1.1,0.0,0.0)
    OLDTime=time.time()
    print OLDTime
    while True:
        if status==2:
            print "fin"
            break
        Time=time.time()-OLDTime
        print "loop"
        print time.time()
        print OLDTime
        print Time
        r.sleep()

'''
    while not rospy.is_shutdown():
        try:
            InputPose(-1.1,0.0,0.0)
            while True:
                if status==2:
                    print "fin"
                    break
                r.sleep()
        except rospy.ROSInterruptException: pass
        r.sleep()
'''
