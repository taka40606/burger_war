#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import sys
from cv_bridge import CvBridge, CvBridgeError

def areacalc(image):
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
	area = cv2.contourArea(maxCont)
	mu = cv2.moments(maxCont)
	x,y= int(mu["m10"]/mu["m00"]) , int(mu["m01"]/mu["m00"])
	image2 = cv2.circle(image, (x,y), 4, 100, 2, 4)

	return area

if __name__ == "__main__":
	
	img = cv2.imread("image_raw_screenshot_09.08.2019.png")
	r = areacalc(img)
	print(r)
