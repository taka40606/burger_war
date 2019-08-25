#!/usr/bin/env python
# license removed for brevity
import rospy
#from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
#from sensor_msgs.msg import PointCloud2
#from pyquaternion import Quaternion #pip install pyquaternion
import math
import numpy as np

Tx=0
Ty=0
th=0
enemy_pose=[0]*2

def odomCallback(my_pose_msg):
    global Tx
    global Ty
    global th
    Tx=my_pose_msg.pose.pose.position.x
    Ty=my_pose_msg.pose.pose.position.y
    z=my_pose_msg.pose.pose.orientation.z
    w=my_pose_msg.pose.pose.orientation.w
    #q = Quaternion(w,0,0,z)
    #tf::Quaternion q(0, 0, thz, thw);
    #tf::Matrix3x3 m(q);
    #double roll, pitch, yaw;
    #m.getRPY(roll, pitch, yaw);
    #print q
    #th=q[2]
    th=2*math.acos(w)*(math.asin(z)/abs(math.asin(z)))
    #print th


def scanCallback(scan):
    global Tx
    global Ty
    global th
    points=np.array([[0.0]*360,[0.0]*360])
    #points3=np.array([[0.0]*360,[0.0]*360])
    #rot=[[math.cos(3.141592/4),-1*math.sin(3.141592/4)],[math.sin(3.141592/4),math.cos(3.141592/4)]]
    #rot2=[[math.cos(-3.141592/4),-1*math.sin(-3.141592/4)],[math.sin(-3.141592/4),math.cos(-3.141592/4)]]
    rot=np.array([[math.cos(3.141592/4),math.sin(3.141592/4)],[-1*math.sin(3.141592/4),math.cos(3.141592/4)]])
    rot2=np.array([[math.cos(-3.141592/4),math.sin(-3.141592/4)],[-1*math.sin(-3.141592/4),math.cos(-3.141592/4)]])
    rot3=np.array([[math.cos(th),math.sin(th)],[-1*math.sin(th),math.cos(th)]])
    for i in range(360):
        points[0][i]=scan.ranges[i]*math.sin(2*3.141592*i/360)
        points[1][i]=scan.ranges[i]*math.cos(2*3.141592*i/360)
    points=np.dot(rot3,points)

    for i in range(360):
        points[0][i]+=Ty
        points[1][i]+=Tx
        if ((0.38<points[0][i]<0.68 and 0.405<points[1][i]<0.655) or
           (-0.68<points[0][i]<-0.38 and -0.655<points[1][i]<-0.405) or
           (0.38<points[0][i]<0.68 and -0.655<points[1][i]<-0.405) or
           (-0.68<points[0][i]<-0.38 and 0.405<points[1][i]<0.655) or
           (-0.225<points[0][i]<0.225 and -0.225<points[1][i]<0.225)):
               points[0][i]=0.0
               points[1][i]=0.0
    #print scan.ranges[1]
    #print points
    points2=np.dot(rot,points)
    #print points2
    for i in range(360):
        if (1.1<points2[0][i] or points2[0][i]<-1.1 or 1.1<points2[1][i] or points2[1][i]<-1.1):
               points2[0][i]=0.0
               points2[1][i]=0.0
    #print points2
    points3=np.dot(rot2,points2)
    #print points3
    count=0
    ave=[0.0]*2
    for i in range(360):
        if (0.001<points3[0][i] or points3[0][i]<-0.001 or 0.001<points3[1][i] or points3[1][i]<-0.001):
            count+=1
            ave[0]+=points3[0][i]
            ave[1]+=points3[1][i]
    ave[0]=ave[0]/count
    ave[1]=ave[1]/count


    while True:
        maxL=0
        maxi=-1
        for i in range(360):
            if ((0.001<points3[0][i] or points3[0][i]<-0.001 or 0.001<points3[1][i] or points3[1][i]<-0.001)
            and maxL < math.sqrt((ave[0]-points3[0][i])*(ave[0]-points3[0][i])+(ave[1]-points3[1][i])*(ave[1]-points3[1][i])) ):
                maxL=math.sqrt((ave[0]-points3[0][i])*(ave[0]-points3[0][i])+(ave[1]-points3[1][i])*(ave[1]-points3[1][i]))
                maxi=i

        points3[0][maxi]=0.0
        points3[1][maxi]=0.0

        ave[0]=0
        ave[1]=0
        count=0
        for i in range(360):
            if (0.001<points3[0][i] or points3[0][i]<-0.001 or 0.001<points3[1][i] or points3[1][i]<-0.001):
                count+=1
                ave[0]+=points3[0][i]
                ave[1]+=points3[1][i]
        ave[0]=ave[0]/count
        ave[1]=ave[1]/count
        print ("ave,maxL=")
        print ave
        print maxL
        if maxL < 0.15:
            print ("fin!!!!")
            print (" ")
            break
    #print points3
    global enemy_pose
    enemy_pose=ave

def enemy():
    #map_sub = rospy.Subscriber('map', OccupancyGrid, mapCallback)
    odom_sub = rospy.Subscriber('/red_bot/odom', Odometry, odomCallback)
    scan_sub = rospy.Subscriber('/red_bot/scan', LaserScan, scanCallback)
    rospy.init_node('pointcloud_processor', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        r.sleep()

if __name__ == '__main__':
    try:
        enemy()
    except rospy.ROSInterruptException: pass
