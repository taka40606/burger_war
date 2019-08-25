# -*- coding: utf-8 -*-
#!/usr/bin/env python


import cv2
import numpy as np
import rospy
import sys
from cv_bridge import CvBridge, CvBridgeError

def enemy_degree_calc(image):
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
	h = hsv[:, :, 0]
	s = hsv[:, :, 1]
	mask = np.zeros(h.shape, dtype=np.uint8)
	mask2 = np.zeros(h.shape, dtype=np.uint8)
	mask[(42 < h) & (h < 120) & (s > 100)] = 255

	image2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	area1 = cv2.contourArea(contours[0])
	area2 = cv2.contourArea(contours[1])

	maxCont=contours[0]
	for c1 in contours:
		if len(maxCont)<len(c1):
			maxCont = c1
	ellipse1 = cv2.fitEllipse(maxCont)
	x = ellipse1[1][0]
	y = ellipse1[1][1]
	theta = np.arccos(x/y)
	cont = contours[0]
	flag = 0
	area_c = x*y*np.pi/4
	ellipse2 = cv2.fitEllipse(contours[1])
	x2 = ellipse2[1][0]
	y2 = ellipse2[1][1]
	area_d = x2*y2*np.pi/4
	area_b = x*np.sin(theta)*y*np.pi*0.75/4

	return area1,area2,area_b,area_c,area_d

if __name__ == "__main__":
	img = cv2.imread("30deg.png")
	r = enemy_degree_calc(img)
	#cv2.imshow('rezone',r)
  	#cv2.waitKey(0)
  	#cv2.destroyALLWIndows()
	print(r)
