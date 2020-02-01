#! /usr/bin/env python
# -*- coding: utf-8 -*-

# z axis flipping現象
import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

from geometry_msgs.msg import Vector3

import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import csv
import os.path
import cv2
from cv2 import aruco
from nav_msgs.msg import Odometry
import tf
import math

from sensor_msgs.msg import LaserScan

import is_field_out as myFunc

from aruco_msgs.msg import MarkerArray, Marker

# テスト用
#import getState

np.set_printoptions(suppress=True)


def getClose(val, lower, upper):
	interval = np.abs(upper - lower)
	while val < lower:
		val += interval
	while val > upper:
		val -= interval
	return val

def quaternion2Euler(quaternion):
	e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
	roll  = e[0] # [rad]
	pitch = e[1] # [rad]
	yaw   = e[2] # [rad]
	return roll, pitch, yaw

def sharpnessImg(img):
	kernel = np.array([[-1, -2, -1],
					   [-2, 12, -2],
					   [-1, -2, -1]])
	sharpness_img = cv2.filter2D(img, -1, kernel)
	return sharpness_img



class ARPoseEstimation():
	def __init__(self):
		# 先にやらないとすぐsubされてエラー出る
		self.bridge = CvBridge()
		# ROS関連
		rospy.init_node('AR_pose_estimation', anonymous=True) # ノード立ち上げ
		# publisher
		self.pub_mybot_pose = rospy.Publisher('ar_mybot_pose', Vector3, queue_size=1)
		self.pub_enemybot_pose = rospy.Publisher('ar_enemybot_pose', Vector3, queue_size=1)
		# subscriber
		#self.sub_mybot_odom = rospy.Subscriber('/red_bot/odom', Odometry, self.SubOdom)
		#self.image_sub = rospy.Subscriber("/red_bot/image_raw", Image, self.callback)
		#self.scan_sub = rospy.Subscriber("/red_bot/scan", LaserScan, self.scan)
		self.sub_mybot_odom = rospy.Subscriber('odom', Odometry, self.SubOdom, queue_size=1)
		self.image_sub = rospy.Subscriber("image_raw", Image, self.callback, queue_size=1)
		self.scan_sub = rospy.Subscriber("scan", LaserScan, self.scan, queue_size=1)
		self.pub_target_id = rospy.Publisher('target_id', MarkerArray, queue_size=1)
		
		
		self.marker_array_num_first_field = 6 
		# x y yaw_rad
		self.marker_pose = np.array([[   0, -0.7, np.pi/2], # blue bot back
									 [-0.7,    0,   np.pi], # blue bot left
									 [ 0.7,    0,       0], # blue bot right
									 [   0, -0.7, np.pi/2], # red bot back
									 [-0.7,    0,   np.pi], # red bot left
									 [ 0.7,    0,       0], # red bot right
									 [-0.53,  0.53 + 0.075, np.pi/2], # tomato n
									 [-0.53,  0.53 - 0.075,-np.pi/2], # tomato s
									 [ 0.53,  0.53 + 0.075, np.pi/2], # omelette n
									 [ 0.53,  0.53 - 0.075,-np.pi/2], # omelette s
									 [-0.53, -0.53 + 0.075, np.pi/2], # pudding n
									 [-0.53, -0.53 - 0.075,-np.pi/2], # pudding s
									 [ 0.53, -0.53 + 0.075, np.pi/2], # octopuswiener n
									 [ 0.53, -0.53 - 0.075,-np.pi/2], # octopuswiener s
									 [     0,  0.175, np.pi/2], # friedshrimp n
									 [ 0.175,      0,       0], # friedshrimp e
									 [-0.175,      0,  -np.pi], # friedshrimp w
									 [     0, -0.175,-np.pi/2]]) # friedshrimp s

		self.array_marker_ids = []

		# parameter
		self.robot_radius = 0.07 # [m]
		self.camera_2_robot_dist_y = 0.058 # [m]
		#aruco = cv2.aruco
		self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
		# dist camera 2 root center
		self.marker_length = 0.020 # [m]
		
		# シミュレーション
		#self.camera_matrix = np.array([  [694.1500723673269,			   0.0, 320.5],
		#								 [				0.0, 694.1500723673269, 240.5],
		#								 [				0.0, 			   0.0,   1.0]])
		# imageサイズはカメラパラメータに含まれてる？
		#self.distortion_coeff = np.array([[ 0., 0., 0., 0., 0.]])
		
		# 実機読み込み
		#self.camera_matrix = np.load("/home/reo/Dropbox/workspace/python/pythonAruco/burger_mtx1.npy")
		#self.distortion_coeff = np.load("/home/reo/Dropbox/workspace/python/pythonAruco/burger_dist1.npy")
		# 実機直接
		self.camera_matrix = np.array([[ 816.42612515,            0, 329.49462233],
		 					  		   [            0, 813.65731686, 217.97573101],
		  					 		   [            0,            0,            1]])
		self.distortion_coeff = np.array([-0.09409554, 0.86805124, -0.00568206, 0.00305165, -3.07839237])

		
		self.marker_xy_dist_mybot_cs = []
		self.marker_yaw_mybot_cs = []
		# mark number取得
		self.getMarkNumber()
		
		self.detected_marker_flag = False
		
		# rider
		self.scan_laser = []
		
		# ar param
		#para = aruco.DetectorParameters_create()
		#para.doCornerRefinement = True
		#para.cornerRefinementWinSize = 10
		
		# 予測結果(x,y,yaw,更新フラグ)
		# mybotはフィールド座標系,enemybotはmybot座標系なことに注意
		self.mybot_x_field_cs = 0 # [m]
		self.mybot_y_field_cs = 0 # [m]
		self.mybot_angle_field_cs = 0 # [rad]
		self.enemybot_x_field_cs = 0 # [m]
		self.enemybot_y_field_cs = 0 # [m]
		self.enemybot_angle_field_cs = 0 # [rad]
		# odometory subscribe
		self.mybot_x_odom = 0
		self.mybot_y_odom = 0
		self.mybot_yaw_odom = 0

		# for Publishing Marker ID
		self.my_target_id_msg = Marker()
		self.my_target_id_msg.id = 0
		self.my_target_ids_msg = MarkerArray()
		self.my_target_ids_msg.markers.append(self.my_target_id_msg)

	# self.marker_yaw_mybot_cs[i]
	# レーザースキャンでマーカーの両端くらいの長さ見て角度補正
	def correctMarkerAngle(self, marker_x, marker_y, marker_self_angle_robot_cs):
		if len(self.scan_laser) < 1:
			return marker_self_angle_robot_cs
		#print("correctMarkerAngle")
		#print(marker_x, marker_y)
		robot_2_mark_angle = np.rad2deg(math.atan2(marker_x, marker_y))
		marker_center_laser_num = 360 - int(getClose(robot_2_mark_angle, 0, 359))
		# 0 ~ 360度へ変換
		marker_left_laser_num  = getClose(marker_center_laser_num + 10 , 0, 359)
		marker_right_laser_num = getClose(marker_center_laser_num - 10 , 0, 359)
		#print(marker_left_laser_num, marker_center_laser_num, marker_right_laser_num)
		#print(self.scan_laser[marker_left_laser_num], self.scan_laser[marker_right_laser_num])
		
		range_diff = self.scan_laser[marker_left_laser_num] - self.scan_laser[marker_right_laser_num]

		if marker_self_angle_robot_cs >= 0:
			if range_diff < 0:
				marker_self_angle_robot_cs *= -1
		if marker_self_angle_robot_cs < 0:
			if range_diff > 0:
				marker_self_angle_robot_cs *= -1
		#print("return")
		return marker_self_angle_robot_cs


	# 推定したマーカーの姿勢から，相手・自己の姿勢推定
	def changeMarkerPose2RobotPose(self):
		# 相手(自己ロボット座標系)
		enemybot_x_array = []
		enemybot_y_array = []
		enemybot_yaw_array = []
		enemybot_diagonal_array = []
		# 自己(マップ座標系)
		mybot_x_array = []
		mybot_y_array = []
		mybot_yaw_array = []
		mybot_diagonal_array = []
		
		self.mybot_x_field_cs = 0
		self.mybot_y_field_cs = 0
		self.mybot_angle_field_cs = 0
		self.enemybot_x_field_cs = 0
		self.enemybot_y_field_cs = 0
		self.enemybot_angle_field_cs = 0

		field_marker_diagonal = 2.0 # [m] 適当
		enemy_marker_diagonal = 2.0 # [m]

		if self.detected_marker_flag == True:
			for i in range(len(self.detected_ids)):
				marker_array_number = np.where(self.array_marker_ids == self.detected_ids[i])
				# 誤認識
				if len(marker_array_number[0]) < 1:
					continue

				marker_array_number = marker_array_number[0][0] # int型に
				# マーカーのフィールド座標
				set_marker_x     = self.marker_pose[marker_array_number][0]
				set_marker_y     = self.marker_pose[marker_array_number][1]
				set_marker_angle = self.marker_pose[marker_array_number][2]
				# ロボットから見たマーカーの位置
				marker_x_mybot_cs = self.marker_xy_dist_mybot_cs[i][0]
				marker_y_mybot_cs = self.marker_xy_dist_mybot_cs[i][1]
				# 相対距離(norm)
				diagonal = np.sqrt(marker_x_mybot_cs**2 + \
								   marker_y_mybot_cs**2 ) + self.robot_radius

				# 距離が一番近いマーカーを選定する
				if marker_array_number < self.marker_array_num_first_field:
					# 敵マーカー
					if enemy_marker_diagonal > diagonal:
						enemy_marker_diagonal = diagonal
						enemy_marker_x = self.robot_radius * np.cos(self.marker_yaw_mybot_cs[i] - np.pi/2) + marker_x_mybot_cs
						enemy_marker_y = self.robot_radius * np.sin(self.marker_yaw_mybot_cs[i] - np.pi/2) + marker_y_mybot_cs
						enemy_marker_angle = getClose(-self.marker_yaw_mybot_cs[i] + set_marker_angle, -np.pi, np.pi)
				
						# 姿勢をmybot座標系からフィールド座標系に変換する
						# テスト用
						#red_state = getState.getPose(model_name="red_bot")
						#self.enemybot_x_field_cs, self.enemybot_y_field_cs, self.enemybot_angle_field_cs = self.changeEnemyPose_Mybot2FieldCS(red_state[0], red_state[1], red_state[2], enemy_marker_x, enemy_marker_y, enemy_marker_angle)
						# 実践用
						self.enemybot_x_field_cs, self.enemybot_y_field_cs, self.enemybot_angle_field_cs = self.changeEnemyPose_Mybot2FieldCS(self.mybot_x_odom, self.mybot_y_odom, self.mybot_yaw_odom, enemy_marker_x, enemy_marker_y, enemy_marker_angle)
				
				
				else:
					# フィールドマーカー
					if field_marker_diagonal > diagonal:
						# レーザースキャンでz axis flipping補正
						corrected_marker_angle = self.correctMarkerAngle(marker_x_mybot_cs, marker_y_mybot_cs, self.marker_yaw_mybot_cs[i])
		
						# フィールド座標系の xとy
						angle = set_marker_angle + corrected_marker_angle
						rot_x, rot_y = myFunc.rotation(marker_x_mybot_cs, marker_y_mybot_cs, \
																			set_marker_angle + corrected_marker_angle + np.pi/2)
						
						self.mybot_x_field_cs = set_marker_x - rot_x
						self.mybot_y_field_cs = set_marker_y - rot_y
		
						# なぜかここのマーカーだけ角度ずれるから例外処理
						if marker_array_number == 15:
							self.mybot_angle_field_cs = corrected_marker_angle + np.pi
						elif marker_array_number == 16:
							self.mybot_angle_field_cs = corrected_marker_angle
						else:
							self.mybot_angle_field_cs = -set_marker_angle + corrected_marker_angle

						self.mybot_angle_field_cs = getClose(self.mybot_angle_field_cs, -np.pi, np.pi)


			# フィールドの内外判定	
			if myFunc.isFieldOut(self.mybot_x_field_cs, self.mybot_y_field_cs) == True:
				self.mybot_x_field_cs = 0
				self.mybot_y_field_cs = 0
				self.mybot_angle_field_cs = 0
			else:
				# navigation座標系に変更
				self.mybot_x_field_cs, self.mybot_y_field_cs = myFunc.rotation(self.mybot_x_field_cs, self.mybot_y_field_cs, np.pi/2)
				self.mybot_angle_field_cs += np.pi/2 
				self.mybot_angle_field_cs = getClose(self.mybot_angle_field_cs, -np.pi, np.pi)
	
			#print("-------mybot--------")
			#print(self.mybot_x_field_cs)
			#print(self.mybot_y_field_cs)
			#print(self.mybot_angle_field_cs)
	

			# フィールドの内外判定	
			if myFunc.isFieldOut(self.enemybot_x_field_cs, self.enemybot_y_field_cs) == True:
				self.enemybot_x_field_cs = 0
				self.enemybot_y_field_cs = 0
				self.enemybot_angle_field_cs = 0
			else:
				# navigation座標系に変更
				self.enemybot_x_field_cs, self.enemybot_y_field_cs = myFunc.rotation(self.enemybot_x_field_cs, self.enemybot_y_field_cs, np.pi/2)
				self.enemybot_angle_field_cs += np.pi/2 
				self.enemybot_angle_field_cs = getClose(self.enemybot_angle_field_cs, -np.pi, np.pi)
			
			#print("-------enemy--------")
			#print(self.enemybot_x_field_cs)
			#print(self.enemybot_y_field_cs)
			#print(self.enemybot_angle_field_cs)
			

	def changeEnemyPose_Mybot2FieldCS(self, my_x, my_y, my_angle, enemy_marker_x, enemy_marker_y, enemy_marker_angle):
		enemy_rot_x, enemy_rot_y = myFunc.rotation(enemy_marker_x, enemy_marker_y, my_angle - np.pi/2)
		enemy_x = my_x + enemy_rot_x
		enemy_y = my_y + enemy_rot_y
		enemy_angle = my_angle + enemy_marker_angle - np.pi/2
		# -pi ~ piに納める
		enemy_angle = getClose(enemy_angle, -np.pi, np.pi)
		return enemy_x, enemy_y, enemy_angle


	def getMarkNumber(self):
		self.array_marker_ids = [52, 50, 51, 42, 40, 41, 9, 10, 5, 6, 11, 12, 7, 8, 1, 3, 4, 2]


	def publishEstimatedPose(self):
		# 自分の姿勢
		vec3 = Vector3(x=self.mybot_x_field_cs, y=self.mybot_y_field_cs, z=self.mybot_angle_field_cs)
		self.pub_mybot_pose.publish(vec3)
		
		# 相手の姿勢
		vec3 = Vector3(x=self.enemybot_x_field_cs, y=self.enemybot_y_field_cs, z=self.enemybot_angle_field_cs)
		self.pub_enemybot_pose.publish(vec3)
	
		# Publish Marker ID
		if self.detected_marker_flag == True:
			for i in range(len(self.detected_ids)):
				self.my_target_ids_msg.markers[0].id = self.detected_ids[i][0]
				self.pub_target_id.publish(self.my_target_ids_msg)


	# マーカーを見つけてマーカーの姿勢を推定する
	def callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print('Cv_Bridge_Error')

		# サイズ大きくすると安定する?
		#size_k = 2
		#self.img = cv2.resize(self.img, (0,0), fx=size_k, fy=size_k, interpolation=cv2.INTER_NEAREST)
		gray_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

		# エッジを強く(要検討)
		#gray_img = sharpnessImg(gray_img)
		#self.resize_img = cv2.filter2D(self.resize_img, -1, kernel)
		
		# マーカー検出
		self.corners, self.detected_ids, rejectedImgPoints = aruco.detectMarkers(gray_img, self.dictionary)
		
		#aruco.drawDetectedMarkers(self.img, self.corners, self.detected_ids, (0,255,0))
		
		#criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 1, 0.001)
		#for corner, in self.corners:
		#	cv2.cornerSubPix(gray_img, corner, (11,11), (-1,-1), criteria)
		
		#aruco.drawDetectedMarkers(self.img, self.corners, self.detected_ids, (0,255,0))

		if len(self.corners) > 0:
			self.detected_marker_flag = True
		else:
			self.detected_marker_flag = False
		

		if self.detected_marker_flag == True:

			if len(self.corners) > 0:
				# 見つけたマーカーのオイラー角の集合
				self.marker_yaw_mybot_cs = []
				self.marker_xy_dist_mybot_cs = []
				
				# マーカーごとに処理
				for corner in enumerate(self.corners):
					#corner = np.squeeze(corner[1])
					corner = corner[1]
					# rvec -> rotation vector, tvec -> translation vector
					rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corner, self.marker_length, self.camera_matrix, self.distortion_coeff)

					# < rodoriguesからeuluerへの変換 >

					# 不要なaxisを除去
					tvec = np.squeeze(tvec)
					rvec = np.squeeze(rvec)
					# 回転ベクトルからrodoriguesへ変換
					rvec_matrix = cv2.Rodrigues(rvec)
					rvec_matrix = rvec_matrix[0] # rodoriguesから抜き出し
					# 並進ベクトルの転置
					transpose_tvec = tvec[np.newaxis, :].T
					# 合成
					proj_matrix = np.hstack((rvec_matrix, transpose_tvec))
					# オイラー角への変換
					euler_angle = cv2.decomposeProjectionMatrix(proj_matrix)[6] # [deg]
					

					marker_yaw_rad = np.deg2rad(euler_angle[1]) # [rad]
					
					self.marker_yaw_mybot_cs.append(marker_yaw_rad)
					
					self.marker_xy_dist_mybot_cs.append(np.array([tvec[0], tvec[2] + self.camera_2_robot_dist_y])) # ロボットからカメラまでの距離も考慮

					# 描画
					#self.img = aruco.drawAxis(self.img, self.camera_matrix, self.distortion_coeff, rvec, tvec, 0.05)

		self.changeMarkerPose2RobotPose()
		
		# 結果をpublish
		self.publishEstimatedPose()

		# 可視化
		#self.img = cv2.resize(self.img, (0,0), fx=0.5, fy=0.5)
		#cv2.imshow('image', self.img)
		#cv2.waitKey(1)
		

	def SubOdom(self, data):
		self.mybot_x_odom = data.pose.pose.position.x
		self.mybot_y_odom = data.pose.pose.position.y
		_, _, self.mybot_yaw_odom = quaternion2Euler(data.pose.pose.orientation)
	
	def scan(self, data):
		self.scan_laser = data.ranges





def main():
	pose_estimation = ARPoseEstimation()

	rospy.spin()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass





# reference
# 可視化
# http://sgrsn1711.hatenablog.com/entry/2018/02/15/224615




# reference
# http://answers.gazebosim.org/question/6503/call-gazeboset_model_state-without-resetting-position/

# angler to orientation
# https://qiita.com/srs/items/93d7cc671d206a07deae
# こっちの方参考にした
# https://qiita.com/TaroYamada/items/e3f3d0ea4ecc0a832fac
