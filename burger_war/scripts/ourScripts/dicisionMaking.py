#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
import math
import time
from std_msgs.msg import Int32MultiArray,MultiArrayLayout,MultiArrayDimension
import is_field_out as fo #katano

status=10
enemy_x=0.0
enemy_y=0.0
enemy_th=0.0
pose_=[-1.3,0,0]
OLDpose_=[0,0,0]
Tx=0.0
Ty=0.0
th=0.0
my_position_num=0
my_position_num2=0
my_position_num_to=0
enemy_position_num=2 #my start:0 right:1 enemy start:2 left:3
position=[[-1.3,0,0],[0.0,-1.3,math.pi/2],[1.3,0.0,math.pi],[0.0,1.3,-1*math.pi/2],
[-0.72051407,-0.74551407,0.0],[0.72051407,-0.74551407,0.0],[0.72051407,0.74551407,0.0],[-0.72051407,0.74551407,0.0]]
my_point=0
enemy_point=0

def movebaseStatusCallback(movebase_status):
    global status
    status=movebase_status.data

def enemyPoseCallback(pose):
    global enemy_x
    global enemy_y
    global enemy_th
    global status
    global enemy_position_num
    enemy_x=pose.x
    enemy_y=pose.y
    enemy_th=pose.theta
    print 'enemy_pose'
    print pose
    #print status
    enemy_position_num=checkPosition(pose.x,pose.y)[0]
    print enemy_position_num

def pointsArrayPoseCallback(point):
    global enemy_point
    global my_point
    my_point=point.data[18]
    enemy_point=point.data[19]
    #print point.data

def odomCallback(my_pose_msg):
    global Tx
    global Ty
    global th
    global my_position_num
    global my_position_num2
    Ty=-1*my_pose_msg.pose.pose.position.x
    Tx=my_pose_msg.pose.pose.position.y
    th=2*math.acos(my_pose_msg.pose.pose.orientation.w)*(math.asin(my_pose_msg.pose.pose.orientation.z)/abs(math.asin(my_pose_msg.pose.pose.orientation.z))) - math.pi/2
    #print 'my_pose'
    #print Tx
    #print Ty
    print th
    my_position_num=checkPosition(Tx,Ty)[0]
    my_position_num2=checkPosition(Tx,Ty)[1]
    #print my_position_num
    #print my_position_num2

def InputPose(x,y,z):
    pose=Pose2D()
    pose.x=x
    pose.y=y
    pose.theta=z
    #print 'move pose:'
    #print pose
    point_pub.publish(pose)

def checkPosition(x,y):
    #print "x,y"
    #print x
    #print y
    position_num=0
    position_num2=0
    if x+y > 0 and x-y > 0: #C opposite
        position_num=2
    elif x+y < 0 and x-y > 0: #B right
        position_num=1
    elif x+y > 0 and x-y < 0: #D left
        position_num=3
    elif x+y < 0 and x-y < 0: #A start
        position_num=0

    if x-(0.455/0.63)*y < 0 and x-(0.605/0.43)*y > 0:
        position_num2=4 #E
    elif x-(0.455/0.63)*y > 0 and x-(0.605/0.43)*y < 0:
        position_num2=6 #G
    elif x+(0.455/0.63)*y < 0 and x+(0.605/0.43)*y > 0:
        position_num2=7 #H
    elif x+(0.455/0.63)*y > 0 and x+(0.605/0.43)*y < 0:
        position_num2=5 #F
    else:
        position_num2=0
    #print position_num
    #print position_num2
    return [position_num,position_num2]


if __name__ == '__main__':
    point_pub = rospy.Publisher('my_pose', Pose2D, queue_size=10)
    movebase_status_sub = rospy.Subscriber('move_base_state', Int8, movebaseStatusCallback)
    enemy_pose_sub = rospy.Subscriber('enemy_pose', Pose2D, enemyPoseCallback)
    points_array_sub = rospy.Subscriber('points_array', Int32MultiArray, pointsArrayPoseCallback)
    odom_sub = rospy.Subscriber('odom', Odometry, odomCallback)
    rospy.init_node('dicision_making', anonymous=True)
    r = rospy.Rate(10) # 10hz
    Time=0
    OLDTime=time.time()
    #global enemy_x
    #global enemy_y
    global my_position_num_to
    global my_position_num
    global enemy_position_num
    global enemy_point
    global my_point
    global Tx
    global Ty
    global th
    #get field points
    #whether get points or not

    i=0
    while i<5:
        i=i+1
        print i
        print "i"
        r.sleep()


    while True:
        if Tx!=0 and Ty!=0:
            print "Tx,Ty,th"
            print Tx
            print Ty
            print th
            break     
        r.sleep()

    L=0.15
    while fo.isFieldOut(Tx-L*math.cos(th), Ty-L*math.sin(th), 0.12) :
        print "L"
        print L
        x=Tx-L*math.cos(th)
        y=Ty-L*math.sin(th)
        print x
        print y
        InputPose(x,y,0.0)
        L=L-0.01


    rospy.spin()

"""
    InputPose(-1.3,0.0,10.0*math.pi/180) # 5deg
    r.sleep()
    r.sleep()
    r.sleep()
    r.sleep()
    r.sleep()
    '''
    while True:
        if status==2:
            print "arrive"
            break
        print "moving"      
        r.sleep()
    '''
    InputPose(-1.3,0.0,-10.0*math.pi/180) # -5deg
    r.sleep()
    r.sleep()
    r.sleep()
    r.sleep()
    r.sleep()
    '''
    while True:
        if status==2:
            print "arrive"
            break
        print "moving"      
        r.sleep()
    '''

    while True:
      print "point"
      print my_point
      print enemy_point
      if (Time > 10 and my_point<enemy_point):
        #move to enemy's back
        #get enemy's point
        #whether get points or not
        print "active mode"

      else:
        print "sniper mode"
        #print "move space"
        #search enemy's position and enemy change position
        #if pose_[0]!=OLDpose_[0] or pose_[1]!=OLDpose_[1] or pose_[2]!=OLDpose_[2]:

        if enemy_position_num==3:
            if my_position_num==0 and my_position_num2!=4:
                my_position_num_to=4
            elif my_position_num==0 and my_position_num2==4:
                my_position_num_to=1
            if my_position_num==2 and my_position_num2!=5:
                my_position_num_to=5
            elif my_position_num==2 and my_position_num2==5:
                my_position_num_to=1
            #if my_position_num==3:
                #turn to enemy direction
            if my_position_num==1:
                #stay
                my_position_num_to=1

        if enemy_position_num==2:
            if my_position_num==1 and my_position_num2!=4:
                my_position_num_to=4
            elif my_position_num==1 and my_position_num2==4:
                my_position_num_to=0
            if my_position_num==3 and my_position_num2!=7:
                my_position_num_to=7
            elif my_position_num==3 and my_position_num2==7:
                my_position_num_to=0
            #if my_position_num==2:
                #turn to enemy direction
            if my_position_num==0:
                #stay
                my_position_num_to=0

        if enemy_position_num==1:
            if my_position_num==2 and my_position_num2!=6:
                my_position_num_to=6
            elif my_position_num==2 and my_position_num2==6:
                my_position_num_to=3
            if my_position_num==0 and my_position_num2!=7:
                my_position_num_to=7
            elif my_position_num==0 and my_position_num2==7:
                my_position_num_to=3
            #if my_position_num==1:
                #turn to enemy direction
            if my_position_num==3:
                #stay
                my_position_num_to=3

        if enemy_position_num==0:
            if my_position_num==1 and my_position_num2!=5:
                my_position_num_to=5
            elif my_position_num==1 and my_position_num2==5:
                my_position_num_to=3
            if my_position_num==3 and my_position_num2!=6:
                my_position_num_to=6
            elif my_position_num==3 and my_position_num2==6:
                my_position_num_to=3
            #if my_position_num==0:
                #turn to enemy direction
            if my_position_num==2:
                #stay
                my_position_num_to=2
        
        #OLDpose_=pose_
        #OLDpose_=position[my_position_num_to]
        Time =time.time()-OLDTime
        print Time
        print "my_position_num"
        print my_position_num
        print "my_position_num2"
        print my_position_num2
        print "enemy_position_num"
        print enemy_position_num
        print "my_position_num_to"
        print my_position_num_to
        print " "
        InputPose(position[my_position_num_to][0],position[my_position_num_to][1],position[my_position_num_to][2])
      r.sleep()
"""
