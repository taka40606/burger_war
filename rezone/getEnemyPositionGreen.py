# -*- coding: utf-8 -*-
#!/usr/bin/env python
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
		#赤色抽出マスク
		mask[((h < 20) | (h > 200)) & (s > 128)] = 255
		#輪郭抽出
		image, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		circles = []
	try:
		maxCont=contours[0]
		for c in contours:
			if len(maxCont)<len(c):
				maxCont = c
		#赤色重心計算
		mu = cv2.moments(maxCont)
		x,y= int(mu["m10"]/mu["m00"]) , int(mu["m01"]/mu["m00"])
		#距離計算
		x2 = float(w)
		y2 = float(h)
		#距離計算
		height_distance_standard = ho/(np.tan(0.3927))
		height_tan = (y2/2)/height_distance_standard#高さtan(th)
		height_rad = np.arctan(height_tan)
		height_deg =height_rad*180/np.pi
		distance = 0.04/height_tan
		distance = 0.92764*distance+0.088126
		#角度計算
		side_standard = wo/(np.tan(0.52359))
		side_tan = (x2/2)/side_standard
		side_rad = np.arctan(side_tan)
		side_deg = side_rad*180/np.pi
		
		
		print(distance,"mm")
		print(side_deg,"deg")
		return image,distance,side_rad
	except:
		distance = 0
		rad = 0
		return image,distance,side_rad
		print("ロスト")

def enemy_position(Tx,Ty,Th,distance,side_rad)
	y = -Tx
	x = Ty
	Tx = x
	Ty = y
	enemy_x = distance*(cos(Th + side_rad))
	enemy_y = distance*(sin(Th + side_rad))
	print("敵x座標",enemy_x)
	print("敵y座標",enemy_y)

	return enemy_x,enemy_y

if __name__ == '__main__':
	rospy.init_node("find_marker")
	OurSubscriber()			# センサ情報おsub用
	odom_sub = rospy.Subscriber('odom', Odometry, odomCallback)
	prcssed_img = []			# マーカー認識位置重ね描き後のカメラ画像
	cut_img = []				# マーカー切り出し画像
	cut_img_resize = []			# リサイズ後のマーカー切り出し画像
	img_switch_counter = 0		# マーカを複数切り出すので描画やおpubの切り替えに使用

	start_time = time.time()				# 画像描画・おpub用タイマー
	cv2.namedWindow("Camera Image2")			# カメラ画像（マーカ位置重ね書き後）描画用窓
	#cv2.namedWindow("Cut Image_")			# 切り出し画像（リサイズ前）描画用窓
	#cv2.namedWindow("Cut Image Contour")	# 切り出し画像（リサイズ後）描画用窓
	rospy.init_node('get_enemy_position', anonymous=True)
    r = rospy.Rate(10) # 10hz
	while True:
		# カメラ画像（マーカ位置重ね書き後）描画
		if len(burger_cv_cam_img) > 0:
			prcssed_img, distance, rad = enemy_position_calc(burger_cv_cam_img)
			cv2.imshow("Camera Image2", prcssed_img)
			if distance == 0:
				print(lost)
			else:
				position = enemy_position(a,b,c,d,e)

    while not rospy.is_shutdown():
        r.sleep()
