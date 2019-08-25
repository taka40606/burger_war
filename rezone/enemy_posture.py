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
	maxCont=contours[0]
	for c1 in contours:
		if len(maxCont)<len(c1):
			maxCont = c1
	ellipse = cv2.fitEllipse(maxCont)
	max_area = cv2.contourArea(maxCont)
	x = ellipse[1][0]
	y = ellipse[1][1]
	theta = np.arccos(x/y)
	
	flag = 0
	for c2 in contours:
		area = cv2.contourArea(c2)
		area_b = x*np.sin(theta)*y*np.pi*0.75/4
		area_c = max_area*0.9
		if area > area_b and area < max_area:
			flag = 1

	#if flag == 1:
	#	theta = -theta
	#deg =theta*180/np.pi
	return flag

