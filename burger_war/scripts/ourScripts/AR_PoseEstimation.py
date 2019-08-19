#! /usr/bin/env python
# -*- coding: utf-8 -*-


import rospy 
import rospkg 
#from gazebo_msgs.msg import ModelState 
#from gazebo_msgs.srv import SetModelState

from geometry_msgs.msg import Vector3

import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import csv
import os.path
import cv2
from cv2 import aruco
#import getState
from nav_msgs.msg import Odometry
import tf

np.set_printoptions(suppress=True)


def rotation(x, y, yaw_rad):
	rot_x = np.cos(yaw_rad) * x - np.sin(yaw_rad) * y
	rot_y = np.sin(yaw_rad) * x + np.cos(yaw_rad) * y
	return rot_x, rot_y

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
	#return Vector3(x=e[0], y=e[1], z=e[2])



class ARPoseEstimation():
	def __init__(self):
		# ROS関連
		rospy.init_node('AR_pose_estimation', anonymous=True) # ノード立ち上げ
		# publisher
		self.pub_mybot_pose = rospy.Publisher('ar_mybot_pose', Vector3)
		self.pub_enemybot_pose = rospy.Publisher('ar_enemybot_pose', Vector3)
		# subscriber
		self.sub_mybot_odom = rospy.Subscriber('odom', Odometry, self.SubOdom)
		self.image_sub = rospy.Subscriber("image_raw", Image, self.callback)
		
		self.bridge = CvBridge()
		
		self.field_marker_init_number = 6 
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
									 [ 0.175,      0,  -np.pi], # friedshrimp e
									 [-0.175,      0,       0], # friedshrimp w
									 [     0, -0.175,-np.pi/2]]) # friedshrimp s

		self.marker_ids = []

		# parameter
		self.robot_radius = 0.07 # [m]
		self.dist_mybot2camera = 0.1 # ? [m]
		#aruco = cv2.aruco
		self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
		# camera parameter
		self.marker_length = 0.020 # [m]
		self.camera_matrix = np.array([  [694.1500723673269,			   0.0, 320.5],
										 [				0.0, 694.1500723673269, 240.5],
										 [				0.0, 			   0.0,   1.0]])
		# imageサイズはカメラパラメータに含まれてる？
		self.distortion_coeff = np.array([[ 0., 0., 0., 0., 0.]])

		self.marker_yaw_mybot_cs = []
		self.marker_xy_dist_mybot_cs = []

		self.getMarkNumber()
		
		self.detected_marker_flag = False
		
		# 予測結果(x,y,yaw,更新フラグ)
		# mybotはフィールド座標系,enemybotはmybot座標系なことに注意
		self.mybot_x = 0 # [m]
		self.mybot_y = 0 # [m]
		self.mybot_yaw = 0 # [rad]
		self.enemybot_x = 0 # [m]
		self.enemybot_y = 0 # [m]
		self.enemybot_yaw = 0 # [rad]
		self.mybot_pose_update_flag = True
		self.enemybot_pose_update_flag = True
		# odometory subscribe
		self.mybot_x_odom = 0
		self.mybot_y_odom = 0
		self.mybot_yaw_odom = 0

	# 推定したマーカーの姿勢から，相手・自己の姿勢推定
	def getMybotPose(self):
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

		if self.detected_marker_flag == True:
			for i in range(len(self.detected_ids)):
				marker_array_number = np.where(self.marker_ids == self.detected_ids[i])
				# 誤認識
				if len(marker_array_number[0]) < 1:
					continue
			
				marker_array_number = marker_array_number[0][0] # int型に
				# マーカーのフィールド座標
				marker_x_field_cs   = self.marker_pose[marker_array_number][0]
				marker_y_field_cs   = self.marker_pose[marker_array_number][1]
				marker_yaw_field_cs = self.marker_pose[marker_array_number][2]
				# ロボットから見たマーカーの位置
				marker_x_mybot_cs = self.marker_xy_dist_mybot_cs[i][0]
				marker_y_mybot_cs = self.marker_xy_dist_mybot_cs[i][1]
				# 相対距離
				diagonal = np.sqrt(marker_x_mybot_cs**2 + \
								   marker_y_mybot_cs**2 ) + self.robot_radius
				# 相対角度
				related_angle = marker_yaw_field_cs + self.marker_yaw_mybot_cs[i]

				# 相手の姿勢推定
				if marker_array_number < self.field_marker_init_number:
					# mybot座標系の相手の角度
					#enemybot_yaw_array.append(self.marker_yaw_mybot_cs[i] + marker_yaw_field_cs)
					enemy_angle = getClose(-self.marker_yaw_mybot_cs[i] + marker_yaw_field_cs, -np.pi, np.pi)
					enemybot_yaw_array.append(enemy_angle)
					x = self.robot_radius * np.cos(self.marker_yaw_mybot_cs[i] - np.pi/2) \
																	+ marker_x_mybot_cs
					y = self.dist_mybot2camera + self.robot_radius * np.sin(self.marker_yaw_mybot_cs[i] - np.pi/2) \
																	+ marker_y_mybot_cs
					enemybot_x_array.append(x)
					enemybot_y_array.append(y)
					enemybot_diagonal_array.append(-(diagonal))
			
				# 自己姿勢推定
				else:
					# フィールド座標系の角度
					mybot_yaw_array.append(-(marker_yaw_field_cs - self.marker_yaw_mybot_cs[i]))
					# フィールド座標系の x&y
					angle = marker_yaw_field_cs + self.marker_yaw_mybot_cs[i]
					rot_x, rot_y = rotation(marker_x_mybot_cs, marker_y_mybot_cs + self.dist_mybot2camera, \
																		marker_yaw_field_cs + self.marker_yaw_mybot_cs[i] + np.pi/2)
					mybot_x_array.append(marker_x_field_cs - rot_x)
					mybot_y_array.append(marker_y_field_cs - rot_y)
					mybot_diagonal_array.append(diagonal)
			
			if len(mybot_x_array) > 0:
				mybot_x_array   = np.array(mybot_x_array)[:, 0]
				mybot_y_array   = np.array(mybot_y_array)[:, 0]
				mybot_yaw_array = np.array(mybot_yaw_array)[:, 0]
				mybot_diagonal_array = np.array(mybot_diagonal_array)
				# 相対距離が一番短いマーカーからの情報を用いる
				minimum_index = np.argmin(mybot_diagonal_array)
				
				self.mybot_x   = mybot_x_array[minimum_index]
				self.mybot_y   = mybot_y_array[minimum_index]
				self.mybot_yaw = mybot_yaw_array[minimum_index]
				self.mybot_pose_update_flag = True
			else:
				self.mybot_pose_update_flag = False

			
			# 相手
			if len(enemybot_x_array) > 0:
				enemybot_x_array   = np.array(enemybot_x_array)[:, 0]
				enemybot_y_array   = np.array(enemybot_y_array)[:, 0]
				enemybot_yaw_array = np.array(enemybot_yaw_array)[:, 0]
				enemybot_diagonal_array = np.array(enemybot_diagonal_array)
				# 相対距離が一番短いマーカーからの情報を用いる
				minimum_index = np.argmin(enemybot_diagonal_array)

				self.enemybot_x   = enemybot_x_array[minimum_index]
				self.enemybot_y   = enemybot_y_array[minimum_index]
				self.enemybot_yaw = enemybot_yaw_array[minimum_index]
				self.enemybot_pose_update_flag = True
			else:
				self.enemybot_pose_update_flag = False
			
			# とりあえず結果をここで出力
			
			print("-------mybot--------")
			print("Update:" + str(self.mybot_pose_update_flag))
			print(self.mybot_x)
			print(self.mybot_y)
			print(self.mybot_yaw)
			print("-------enemy--------")
			print("Update:" + str(self.enemybot_pose_update_flag))
			

			#red_state = getState.getPose(model_name="red_bot")

			# 今回はとりあえず真値とってきてテスト
			self.enemybot_x, self.enemybot_y, self.enemybot_yaw = self.getEnemyPoseFieldCS(self.mybot_x_odom, self.mybot_y_odom, self.mybot_yaw_odom)
			print("convert")
			print(self.enemybot_x)
			print(self.enemybot_y)
			print(self.enemybot_yaw)
			#print("anser")
			#print(getState.getPose(model_name="blue_bot"))
			# publish
			self.publishEstimatedPose()

	def getEnemyPoseFieldCS(self, my_x, my_y, my_angle):
		rot_x, rot_y = rotation(self.enemybot_x, self.enemybot_y, my_angle - np.pi/2)
		enemy_x = my_x + rot_x
		enemy_y = my_y + rot_y
		enemy_angle = my_angle + self.enemybot_yaw - np.pi/2
		# -pi ~ piに納める
		enemy_angle = getClose(enemy_angle, -np.pi, np.pi)
		return enemy_x, enemy_y, enemy_angle


	def getMarkNumber(self):
		csv_path = os.path.expanduser('~') + "/catkin_ws/src/burger_war/judge/marker_set/sim.csv"
		#csv_path = os.path.expanduser('~') + "/catkin_ws/src/burger_war/judge/marker_set/marker_set_default.csv"

		with open(csv_path, 'r') as f:
			reader = csv.reader(f)
			for row in reader:
				self.marker_ids.append(int(row[2]))

			self.marker_ids = np.array(self.marker_ids)

	def publishEstimatedPose(self):
		if self.mybot_pose_update_flag == True:
			vec3 = Vector3(x=self.mybot_x, y=self.mybot_y, z=self.mybot_yaw)
			self.pub_mybot_pose.publish(vec3)
		
		if self.enemybot_pose_update_flag == True:
			vec3 = Vector3(x=self.enemybot_x, y=self.enemybot_y, z=self.enemybot_yaw)
			self.pub_enemybot_pose.publish(vec3)
	

	# マーカーを見つけてマーカーの姿勢を推定する
	def callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print('Cv_Bridge_Error')
		# サイズ大きくしないと安定しない
		size_k = 4
		self.resize_img = cv2.resize(self.img, (0,0), fx=size_k, fy=size_k)
		# マーカー検出
		self.corners, self.detected_ids, rejectedImgPoints = aruco.detectMarkers(self.resize_img, self.dictionary)

		print(len(self.corners))
		if len(self.corners) > 0:
			self.corners[0] /= size_k

		# 描画
		aruco.drawDetectedMarkers(self.img, self.corners, self.detected_ids, (0,255,0))

		if len(self.corners) > 0:
			self.detected_marker_flag = True
		else:
			self.detected_marker_flag = False

		if self.detected_marker_flag == True:
			rvec, tvec, obj_points = aruco.estimatePoseSingleMarkers(self.corners, self.marker_length, self.camera_matrix, self.distortion_coeff)
			# 見つけたマーカーのオイラー角の集合
			self.marker_yaw_mybot_cs = []
			self.marker_xy_dist_mybot_cs = []

			for i in range(rvec.shape[0]):
				# rodoriguesからeuluerへの変換
				rvec_matrix = cv2.Rodrigues(rvec[np.newaxis, i, :,:])[0]
				proj_matrix = np.hstack((rvec_matrix, tvec[i, :,:].T))
				euler_angle = cv2.decomposeProjectionMatrix(proj_matrix)[6] # [deg]

				marker_yaw_rad = np.deg2rad(euler_angle[1]) # [rad]

				self.marker_yaw_mybot_cs.append(marker_yaw_rad)
				self.marker_xy_dist_mybot_cs.append(np.array([tvec[i,0,0], tvec[i,0,2]]))

				# 描画
				self.img = aruco.drawAxis(self.img, self.camera_matrix, self.distortion_coeff, rvec[i], tvec[i], 0.05)

		self.getMybotPose()


		# 可視化
		self.img = cv2.resize(self.img, (0,0), fx=0.5, fy=0.5)
		cv2.imshow('image', self.img)
		cv2.waitKey(1)


	def SubOdom(self, data):
		self.mybot_x_odom = data.pose.pose.position.x
		self.mybot_y_odom = data.pose.pose.position.y
		_, _, self.mybot_yaw_odom = quaternion2Euler(data.pose.pose.orientation)



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
