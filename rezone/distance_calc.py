# -*- coding: utf-8 -*-
#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import sys
from cv_bridge import CvBridge, CvBridgeError

def distance_calc(image):
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
		y2 = float(y)
		y_standard = 126/(np.tan(0.432))
		a = (126-y2)/y_standard#tan(th)
		rada = np.arctan(a)
		dega =rada*180/np.pi
		distance = 200/a
		distance = ((distance-100)/650)*1000
		print(y2)
		print(dega,"deg")
		print(distance,"mm")
		return image
	except:
		print("例外")

if __name__ == "__main__":
	#img = cv2.imread("image_raw_screenshot_09.08.2019.png",cv2.IMREAD_COLOR)
	img = cv2.imread("2.0dis.png")
	r = distance_calc(img)
  	cv2.imshow('rezone',r)
  	cv2.waitKey(0)
  	cv2.destroyALLWIndows()

