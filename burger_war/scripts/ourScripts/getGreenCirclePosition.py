#!/usr/bin/env python
# -*- coding: utf-8 -*-
# license removed for brevity

import re
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray,MultiArrayLayout,MultiArrayDimension
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
import math
import numpy as np
import cv2
import sys
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import time
import copy
import math
from getRedBallPosition import enemy_position_calc_red


# グローバル変数
## Camera Image
burger_cv_cam_img = []
enemyPose=[0.0]*3
my_bot_id = ""
myColor=0
enemyTime=0.0
OLDenemyTime=0.0
Tx=0.0
Ty=0.0
th=0.0
enemyPose_p=[0.0]*2

class OurSubscriber(object):

	def __init__(self):
		#ROS subscriber
		self.camcmprss_sub = rospy.Subscriber('image_raw/compressed', CompressedImage, self.GetCamCmprssImgCallback, queue_size=1)
		self.conv_start_time = time.time()
		self.conv_now_time = self.conv_start_time
		self.conv_elapsed_time = self.conv_now_time - self.conv_now_time
	
	def GetCamCmprssImgCallback(self, given_data):		# self, おsubした画像msg
		global burger_cv_cam_img
		global burger_cut_img
		global burger_cut_img2
		global burger_cut_img_num
		self.conv_now_time = time.time()
		self.conv_elapsed_time = self.conv_now_time - self.conv_start_time
		if self.conv_elapsed_time > 0.033:
			self.conv_start_time = self.conv_now_time
			np_arr = np.fromstring(given_data.data, np.uint8)
			burger_cv_cam_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


def odomCallback(my_pose_msg):
	global Tx
	global Ty
	global th
	Tx=my_pose_msg.pose.pose.position.x
	Ty=my_pose_msg.pose.pose.position.y
	#z=my_pose_msg.pose.pose.orientation.z
	#w=my_pose_msg.pose.pose.orientation.w
	#th=2*math.acos(w)*(math.asin(z)/abs(math.asin(z)))
	th=2*math.acos(my_pose_msg.pose.pose.orientation.w)*(math.asin(my_pose_msg.pose.pose.orientation.z)/abs(math.asin(my_pose_msg.pose.pose.orientation.z)))
	#print th

def enemy_position_calc(image):
	#hsv変換
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
	h = hsv[:, :, 0]
	s = hsv[:, :, 1]
	mask = np.zeros(h.shape, dtype=np.uint8)
	#画素数
	wo = 320
	ho = 240
	#緑色抽出マスク
	mask[(42 < h) & (h < 120) & (s > 100)] = 255
	#輪郭抽出
	image, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	circles = []
	try:
		
		maxCont=contours[0]
		for c in contours:
			if len(maxCont)<len(c):
				maxCont = c
		#緑色重心計算
		x,y,w,h = cv2.boundingRect(maxCont)
		img = cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),2)
		#cv2.imshow("rect_image",img)
		#距離計算
		x2 = float(x+w/2)
		#print("x2",x2)
		y2 = float(h)
		#距離計算
		height_distance_standard = ho/(np.tan(0.3927))
		height_tan = (y2/2)/height_distance_standard#高さtan(th)
		height_rad = np.arctan(height_tan)
		height_deg =height_rad*180/np.pi
		distance = 0.04/height_tan
		distance = 1.25*distance + 0.1
		
		#角度計算
		side_standard = wo/(np.tan(0.52359))
		side_tan = (x2-wo)/side_standard
		side_rad = np.arctan(side_tan)
		side_deg = side_rad*180/np.pi

		#distance = distance/np.cos(side_rad)

		#print("distance",distance)
		#print("side_deg",side_deg)

		return image,distance,side_rad
	except:
		distance = 10
		side_rad = 0
		return image,distance,side_rad
		print("ロスト")

def enemy_position(Tx,Ty,th,distance,side_rad):

	enemy_x = Tx + distance*(np.cos(th - side_rad))
	enemy_y = Ty + distance*(np.sin(th - side_rad))
	#print("x",Tx)
	#print("y",Ty)		
	if enemy_x < -1.45 or enemy_x > 1.45 or enemy_y < -1.45 or enemy_y > 1.45:
		enemy_x = 0
		enemy_y = 0

		#print("lost")
	#else:
		#print("x",Tx)
		#print("y",Ty)
		#print("th",th)
		#print("side_rad",side_rad)
		#print("th + side_rad",th + side_rad)
		#print("x_pos",enemy_x)
		#print("y_pos",enemy_y)
	return enemy_x,enemy_y

if __name__ == '__main__':
	rospy.init_node('getGreenCirclePosition', anonymous=True)
	OurSubscriber()			# センサ情報おsub用
	odom_sub = rospy.Subscriber('odom', Odometry, odomCallback)
	pose_pub = rospy.Publisher('green_position', Pose2D, queue_size=10)
	pose_pub_red = rospy.Publisher('red_ball_position', Pose2D, queue_size=10)
	prcssed_img = []			# マーカー認識位置重ね描き後のカメラ画像
	cut_img = []				# マーカー切り出し画像
	cut_img_resize = []			# リサイズ後のマーカー切り出し画像
	img_switch_counter = 0		# マーカを複数切り出すので描画やおpubの切り替えに使用

	#cv2.namedWindow("Cut Image2")			# カメラ画像（マーカ位置重ね書き後）描画用窓
	#cv2.namedWindow("Camera Image2")			# 切り出し画像（リサイズ前）描画用窓
	#cv2.namedWindow("Cut Image Contour")	# 切り出し画像（リサイズ後）描画用窓
	
	r = rospy.Rate(10)
	while True:
		r.sleep()
		#cv2.imshow("Camera Image2", burger_cv_cam_img)
		# カメラ画像（マーカ位置重ね書き後）描画
		distance = 0
		rad = 0
		distance_red = 0
		rad_red = 0
		if len(burger_cv_cam_img) > 0:
			prcssed_img, distance, rad = enemy_position_calc(burger_cv_cam_img)
			
			#cv2.imshow("Cut Image2", prcssed_img)
			position = enemy_position(Tx,Ty,th,distance,rad)
			pose=Pose2D()
			pose.x=position[1]
			pose.y=-position[0]
			pose.theta=0
			pose_pub.publish(pose)
			prcssed_img, distance_red, rad_red = enemy_position_calc_red(burger_cv_cam_img)
			position_red = enemy_position(Tx,Ty,th,distance_red,rad_red)
			pose_red=Pose2D()
			pose_red.x=position_red[1]
			pose_red.y=-position_red[0]
			pose_red.theta=0
			pose_pub_red.publish(pose_red)
	while not rospy.is_shutdown():
		r.sleep()
