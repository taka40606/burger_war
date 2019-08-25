#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import sys
from cv_bridge import CvBridge, CvBridgeError

def find_rect_of_target_color(image):
  hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
  h = hsv[:, :, 0]
  s = hsv[:, :, 1]
  mask = np.zeros(h.shape, dtype=np.uint8)
  mask[((h < 20) | (h > 200)) & (s > 128)] = 255
  image, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  rects = []
  for contour in contours:
     approx = cv2.convexHull(contour)
     rect = cv2.boundingRect(approx)
     rects.append(np.array(rect))
  return rects

if __name__ == "__main__":
	#img = cv2.imread("image_raw_screenshot_09.08.2019.png",cv2.IMREAD_COLOR)
	img = cv2.imread("image_raw_screenshot_09.08.2019.png")
	r = find_rect_of_target_color(img)
  	#cv2.imshow('rezone',r)
  	#cv2.waitKey(0)
  	#cv2.destroyALLWIndows()
	print(r)
