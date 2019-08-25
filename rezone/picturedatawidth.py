#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import sys
from cv_bridge import CvBridge, CvBridgeError

img = cv2.imread("image_raw_screenshot_09.08.2019.png",cv2.IMREAD_COLOR)
height, width, channels = img.shape[:3]
print("width: " + str(width))
print("height: " + str(height))
