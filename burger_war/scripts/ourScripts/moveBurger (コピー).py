#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
from std_msgs.msg import String
from std_msgs.msg import Int8
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from actionlib_msgs.msg import GoalStatus
import math
import numpy as np
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
		self.map_sub = rospy.Subscriber('map', OccupancyGrid, self.mapCallback)
		self.point_sub = rospy.Subscriber('my_pose', Pose2D, self.moveCallback)

	def moveCallback(self,point_):
		self.point = [point_.x, point_.y, math.sin(point_.theta/2), math.cos(point_.theta/2)]
		#self.movebase_status=0
		#print("movebase_status::::%d" % self.movebase_status)
		'''
		while True:
			if self.movebase_status==2:
				break
		'''

	def GoalStatusArrayCallback(self, data):
		#print("movebase_status:%d" % self.movebase_status)
		status=Int8()
		status.data=self.movebase_status
		self.movebase_status_pub.publish(status)

		status_id = 0;
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
			elif status_id == 4:
				self.movebase_status = 4
			return
		#stop
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
				return
			else:
				self.movebase_status = 1
				return
		elif self.movebase_status == 4:
			self.movebase_status = 0
			return


	def mapCallback(self, map_info):
		self.my_bot_id = map_info.header.frame_id


if __name__=="__main__":
	rospy.init_node("move_burger")
	OnigiriRun()
	rospy.spin()
