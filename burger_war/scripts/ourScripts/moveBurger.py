#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray
from actionlib_msgs.msg import GoalStatus
import math
import numpy as np
import time
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose2D

class OnigiriRun(object):

	def __init__(self):
		self.my_bot_id = ""
		self.movebase_status = 2
		#ROS publisher
		self.goal_point_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
		self.movebase_status_pub = rospy.Publisher('move_base_state', Int8, queue_size=1)
		#ROS subscriber
		self.move_base_status_sub = rospy.Subscriber('move_base/status', GoalStatusArray, self.GoalStatusArrayCallback, queue_size=1)
		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.map_sub = rospy.Subscriber('map', OccupancyGrid, self.mapCallback)
		self.point_sub = rospy.Subscriber('my_pose', Pose2D, self.moveCallback)
        	self.odom_sub = rospy.Subscriber('odom', Odometry, self.odomCallback)
        	self.obstacle_direction_sub = rospy.Subscriber('obstacle_direction', Float32, self.obstacleDirectionCallback)
		self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.client.wait_for_server()

	def moveCallback(self,point_):
		self.point = [point_.x, point_.y, math.sin(point_.theta/2), math.cos(point_.theta/2)]
		#self.movebase_status=0
		#print("movebase_status::::%d" % self.movebase_status)

	def recoveryBehavior(self):
	        twist=Twist()
	        global back_direction
	        global th
                r = rospy.Rate(10)
	        #print "back_direction"
	        #print back_direction
	        #print "th"
	        #print th
	        turn_speed=1.35
	        th_tmp=th
	        back_direction_tmp=back_direction
	        while abs(th_tmp-th)<abs(back_direction_tmp):
	            twist.angular.z=turn_speed*back_direction_tmp/abs(back_direction_tmp)
	            self.cmd_vel_pub.publish(twist)
	            #print th
	            r.sleep()
	        twist.angular.z=0.0
	        self.cmd_vel_pub.publish(twist)
	        r.sleep()
	        twist.linear.x=0.2
	        self.cmd_vel_pub.publish(twist)
	        Tx_old=Tx
	        Ty_old=Ty
	        while 0.2>math.sqrt((Tx-Tx_old)*(Tx-Tx_old)+(Ty-Ty_old)*(Ty-Ty_old)):
	            r.sleep()
	        twist.linear.x=0.0
	        self.cmd_vel_pub.publish(twist)
	
	def obstacleDirectionCallback(self, direction_msg):
		global back_direction
	        #print "direction_msg"
	        #print direction_msg.data
	        back_direction=direction_msg.data
	
	def odomCallback(self, my_pose_msg):
	        global Tx
	        global Ty
	        global th
	        Ty=-1*my_pose_msg.pose.pose.position.x
	        Tx=my_pose_msg.pose.pose.position.y
	        th=2*math.acos(my_pose_msg.pose.pose.orientation.w)*(math.asin(my_pose_msg.pose.pose.orientation.z)/abs(math.asin(my_pose_msg.pose.pose.orientation.z))) - math.pi / 2.0
	
	
	def GoalStatusArrayCallback(self, data):
		#print("movebase_status:%d_______start" % self.movebase_status)
		status=Int8()
		status.data=self.movebase_status
		self.movebase_status_pub.publish(status)
		#self.recoveryBehavior()
		if self.movebase_status!=5: #2
			#print self.point
			status_id = 0
			#print self.movebase_status
			#uint8 PENDING         = 0  
			#uint8 ACTIVE          = 1 
			#uint8 PREEMPTED       = 2
			#uint8 SUCCEEDED       = 3
			#uint8 ABORTED         = 4
			#uint8 REJECTED        = 5
			#uint8 PREEMPTING      = 6
			#uint8 RECALLING       = 7
			#uint8 RECALLED        = 8
			#uint8 LOST            = 9
			if len(data.status_list) != 0:
				goalStatus = data.status_list[0]
				status_id = goalStatus.status
				#print status_id

			#moving
            		if self.movebase_status == 1:
				if status_id == 3 or status_id == 0:
					self.movebase_status = 2
					#time.sleep(2.0)
				elif status_id == 4:
					self.movebase_status = 4
				return
			#stop and goal
			elif self.movebase_status == 0 or self.movebase_status == 2:
				self.pose = PoseStamped()
				self.pose.pose.position.x = self.point[0]
				self.pose.pose.position.y = self.point[1]
				self.pose.pose.position.z = 0.0
				#quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
				self.pose.pose.orientation.x = 0
				self.pose.pose.orientation.y = 0
				self.pose.pose.orientation.z = self.point[2]
				self.pose.pose.orientation.w = self.point[3]
				self.pose.header.frame_id = self.my_bot_id #change
				self.pose.header.stamp = rospy.Time.now()
				self.goal_point_pub.publish(self.pose)
				self.movebase_status = 3
				return
			elif self.movebase_status == 3:

				if status_id == 0:
					self.movebase_status = 0
				elif status_id == 3:
					self.movebase_status = 2
					return
				else:
					self.movebase_status = 1
					return
			#elif self.movebase_status == 4:
			#	self.movebase_status = 0
			#	return
			elif self.movebase_status == 4:
				self.client.cancel_goal()
                		self.recoveryBehavior()
                		self.movebase_status = 0
                		return

		#print("movebase_status:%d_______end" % self.movebase_status)

	def mapCallback(self, map_info):
		self.my_bot_id = map_info.header.frame_id

if __name__=="__main__":
	rospy.init_node("move_burger")
	OnigiriRun()
	rospy.spin()
