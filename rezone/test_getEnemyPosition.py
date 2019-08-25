# -*- coding: utf-8 -*-
#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import sys
from cv_bridge import CvBridge, CvBridgeError


def enemy_position_calc(image):
	try:
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

		maxCont=contours[0]
		for c in contours:
			if len(maxCont)<len(c):
				maxCont = c
		#赤色重心計算
		mu = cv2.moments(maxCont)
		x,y= int(mu["m10"]/mu["m00"]) , int(mu["m01"]/mu["m00"])
		x2 = float(x)
		y2 = float(y)
		#距離計算
		height_distance_standard = 126/(np.tan(0.432))
		height_tan = (126-y2)/height_distance_standard#高さtan(th)
		height_rad = np.arctan(height_tan)
		height_deg =height_rad*180/np.pi
		distance = 200/height_tan
		distance = ((distance-100)/650)*1000
		#角度計算
		side_standard = 168*np.sqrt(3)
		side_tan = (x2-168)/side_standard
		side_rad = np.arctan(side_tan)
		side_deg = side_rad*180/np.pi
		
		print(distance,"mm")
		print(side_deg,"deg")
		return distance,side_rad
	except:
		distance = 0
		rad = 0
		return distance,side_rad
		print("ロスト")


if __name__ == "__main__":
	#img = cv2.imread("image_raw_screenshot_09.08.2019.png",cv2.IMREAD_COLOR)
	odom_sub = rospy.Subscriber('odom', Odometry, odomCallback)
	img = cv2.imread("2.0dis.png")
	r = enemy_position_calc(img)
	print(r)
  	#cv2.imshow('rezone',r)
  	#cv2.waitKey(0)
  	#cv2.destroyALLWIndows()


