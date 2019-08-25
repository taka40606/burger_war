#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import sys
from cv_bridge import CvBridge, CvBridgeError

def degree_calc(image):
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
	h = hsv[:, :, 0]
	s = hsv[:, :, 1]
	mask = np.zeros(h.shape, dtype=np.uint8)
	
	mask[((h < 20) | (h > 200)) & (s > 128)] = 255

	image, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	circles = []

	maxCont=contours[0]
	for c in contours:
		if len(maxCont)<len(c):
			maxCont = c

	mu = cv2.moments(maxCont)
	x,y= int(mu["m10"]/mu["m00"]) , int(mu["m01"]/mu["m00"])
	x = x-168

	return x

