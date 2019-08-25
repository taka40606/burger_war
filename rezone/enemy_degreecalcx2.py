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

	_, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	#maxCont=contours[0]
	#for c in contours:
	#	if len(maxCont)<len(contours[c]):
	#		maxCont = c
	#for c in contours:
	#	if 
	#ellipse = cv2.fitEllipse(maxCont)
	#x = ellipse[1][0]
	#y = ellipse[1][1]
	#theta = np.arccos(x/y)
	#image2 = cv2.ellipse(mask2,ellipse,(255,255,255),-1)
	#mu = cv2.moments(maxCont)
	#x,y= int(mu["m10"]/mu["m00"]) , int(mu["m01"]/mu["m00"])
	#x2 = float(x)
	#y_standard = 168*np.sqrt(3)
	#a = (x2-168)/y_standard
	#rad = np.arctan(a)
	#deg =theta*180/np.pi
	#return image2
	return contours

if __name__ == "__main__":
	#img = cv2.imread("image_raw_screenshot_09.08.2019.png",cv2.IMREAD_COLOR)
	img = cv2.imread("45deg.png")
	r = enemy_degree_calc(img)
  	#cv2.imshow('rezone',r)
  	#cv2.waitKey(0)
  	#cv2.destroyALLWIndows()
	print(r)

