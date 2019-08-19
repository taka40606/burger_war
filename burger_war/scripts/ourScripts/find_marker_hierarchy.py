#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import CompressedImage
import time
import copy
import math

## Camera Image
burger_cv_cam_img = []

class OurSubscriber(object):

	def __init__(self):
		#ROS subscriber
		self.camcmprss_sub = rospy.Subscriber('image_raw/compressed', CompressedImage, self.GetCamCmprssImgCallback, queue_size=1)
		self.conv_start_time = time.time()
		self.conv_now_time = self.conv_start_time
		self.conv_elapsed_time = self.conv_now_time - self.conv_now_time

	def GetCamCmprssImgCallback(self, given_data):
		global burger_cv_cam_img
		global burger_cut_img
		global burger_cut_img2
		global burger_cut_img_num
		self.conv_now_time = time.time()
		self.conv_elapsed_time = self.conv_now_time - self.conv_start_time
		if self.conv_elapsed_time > 0.033:
			self.conv_start_time = self.conv_now_time
			np_arr = np.fromstring(given_data.data, np.uint8)
			burger_cv_cam_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


def find_marker(src_img):
	gray_img = cv2.cvtColor(src_img, cv2.COLOR_BGR2GRAY)
	cut_img_temp = []
	cut_img_temp_resize = []
	output_img = []
	output_img_resize = []

	#prcssed_img, boxes_pos, bounding_boxes_shape = find_circ_of_target(src_img)
	prcssed_img, boxes_pos, bounding_boxes_shape = find_rect_of_target_color(src_img)

	for i in range(len(boxes_pos)):
		if boxes_pos[i][1] >= bounding_boxes_shape[i][1] / 2  and gray_img.shape[0] > boxes_pos[i][1] + bounding_boxes_shape[i][1] / 2 and boxes_pos[i][0] >= bounding_boxes_shape[i][0] / 2 and gray_img.shape[1] > boxes_pos[i][0] + bounding_boxes_shape[i][0] / 2:
			cut_img_temp = copy.deepcopy(gray_img[boxes_pos[i][1] - bounding_boxes_shape[i][1] / 2 : boxes_pos[i][1] + bounding_boxes_shape[i][1] / 2, boxes_pos[i][0] - bounding_boxes_shape[i][0] / 2 : boxes_pos[i][0] + bounding_boxes_shape[i][0] / 2])
			#kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]], np.float32)
			kernel = np.array([[0,-1,0], [-1,5,-1], [0,-1,0]], np.float32)
			cut_img_temp = cv2.filter2D(cut_img_temp, -1, kernel)
			ret,thresh = cv2.threshold(cut_img_temp,127,255,0)

			#cv2.imwrite('result_'+str(i)+'.png', cut_img_temp)
			output_img.append(cut_img_temp)
			cut_img_temp_resize = cv2.resize(cut_img_temp, (src_img.shape[1], src_img.shape[0]))
			cut_img_temp_resize = make_marker_clear(cut_img_temp_resize)
			output_img_resize.append(cut_img_temp_resize)

	return [prcssed_img, output_img, output_img_resize]

"""
def contrast(image, a):
  lut = [ np.uint8(255.0 / (1 + math.exp(-a * (i - 128.) / 255.))) for i in range(256)]
  result_image = np.array( [ lut[value] for value in image.flat], dtype=np.uint8 )
  result_image = result_image.reshape(image.shape)
  return result_image
"""

def make_marker_clear(src_img):
	prcssed_img = copy.deepcopy(src_img)
	prcssed_img = cv2.bilateralFilter(prcssed_img, 9, 75, 75)
	#prcssed_img = cv2.GaussianBlur(prcssed_img,(5,5),0)
	ret_temp, prcssed_img = cv2.threshold(prcssed_img,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
	#clahe = cv2.createCLAHE(clipLimit=1.0, tileGridSize=(8,8))
	#prcssed_img = clahe.apply(prcssed_img)
	#prcssed_img = cv2.equalizeHist(prcssed_img)
	#prcssed_img = contrast(prcssed_img, 20)
	#ret_temp, prcssed_img = cv2.threshold(prcssed_img, 50, 255, cv2.THRESH_BINARY)
	return prcssed_img

def find_circ_of_target(src_img):
	circles_pos = []
	bounding_boxes_shape = []
	prcssed_img = cv2.bilateralFilter(src_img, 9, 75, 75)
	canny_img = cv2.Canny(src_img, 50, 150)
	dst, contours, hierarchy = cv2.findContours(canny_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	kernel = np.ones((5,5),np.uint8)
	dst_bold = cv2.dilate(dst,kernel,iterations = 1)
	circles = cv2.HoughCircles(dst_bold,cv2.HOUGH_GRADIENT,dp=1,minDist=20,param1=80,param2=40,minRadius=0,maxRadius=240) #エラーになる場合はHOUGH_GRADIENTをcv.CV_HOUGH_GRADIENT  にする

	if circles != None:
		circles = np.uint16(np.around(circles))
		for i in circles[0,:]:
			circles_pos.append([i[0], i[1]])
			bounding_boxes_shape.append([i[2]*2, i[2]*2])
			cv2.circle(prcssed_img,(i[0],i[1]),i[2],(0,255,0),2)

	return [prcssed_img, circles_pos, bounding_boxes_shape]


def find_rect_of_target_color(src_img):
	boxes_pos = []
	bounding_boxes_shape = []
	prcssed_img = copy.deepcopy(src_img)
	hsv = cv2.cvtColor(src_img, cv2.COLOR_BGR2HSV_FULL)
	h = hsv[:, :, 0]
	s = hsv[:, :, 1]
	#青認識
	mask = np.zeros(h.shape, dtype=np.uint8)
	mask[(((h < 230) & (h > 150)) & (s > 70)) | (((h < 120) & (h > 70)) & (s > 70))] = 255
	kernel = np.ones((5,5),np.uint8)
	mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
	dst, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	rects = []
	for i, contour in enumerate(contours):
		#外側の輪郭のみ
		if hierarchy[0][i][3] != -1:
			continue
		approx = cv2.convexHull(contour)
		area = cv2.contourArea(approx)
		if area < 600:#600:
			continue
		x,y,w,h = cv2.boundingRect(approx)
		#あまりに細長い領域は除外
		if (w>3.0*h or 3.0*w<h):
			continue
		boxes_pos.append([x+w/2, y+h/2])
		bounding_boxes_shape.append([2*w, 2*h])
		prcssed_img = cv2.rectangle(src_img,(x-w/2,y-h/2),(x+3*w/2,y+3*h/2),(0,255,0),2)


	return [prcssed_img, boxes_pos, bounding_boxes_shape]
	#return rects



def shoot_marker(src_img, img_pub):
	our_cvbridge = CvBridge()
	prcssed_img_msg = our_cvbridge.cv2_to_imgmsg(src_img, encoding="bgr8")
	img_pub.publish(prcssed_img_msg)


if __name__ == '__main__':
	rospy.init_node("find_marker")
	OurSubscriber()
	prcssimg_pub = rospy.Publisher('image_prcssed', Image, queue_size=1)


	prcssed_img = []
	cut_img = []
	cut_img_resize = []
	img_switch_counter = 0

	start_time = time.time()
	cv2.namedWindow("Camera Image")
	cv2.namedWindow("Cut Image_")
	#cv2.namedWindow("Cut Image2")

	num_img=0


	while True:
		now_time = time.time()
		elapsed_time = now_time - start_time
		if elapsed_time > 0.033:
			start_time = now_time
			if len(burger_cv_cam_img) > 0:
				prcssed_img, cut_img, cut_img_resize = find_marker(burger_cv_cam_img)
				cv2.imshow("Camera Image", prcssed_img)
			if len(cut_img) > 0:

				if img_switch_counter < len(cut_img):
					if len(cut_img[img_switch_counter]) > 0:
						if cut_img[img_switch_counter].shape[0] > 0 and cut_img[img_switch_counter].shape[1] > 0:
							cut_img_rgb = cv2.cvtColor(cut_img[img_switch_counter],cv2.COLOR_GRAY2RGB)
							cv2.imshow("Cut Image_", cut_img_rgb)
				#			prcssimg_msg = our_cvbridge.cv2_to_imgmsg(cut_img_rgb, encoding="bgr8")
				#			prcssimg_pub.publish(prcssimg_msg)

				if img_switch_counter < len(cut_img_resize):
					if len(cut_img_resize[img_switch_counter]) > 0:
						if cut_img_resize[img_switch_counter].shape[0] > 0 and cut_img_resize[img_switch_counter].shape[1] > 0:
							cut_img_resize_rgb = cv2.cvtColor(cut_img_resize[img_switch_counter],cv2.COLOR_GRAY2RGB)
							cv2.imshow("Cut Image2", cut_img_resize_rgb)
							#cv2.imwrite('result_'+str(num_img)+'.png', cut_img_resize_rgb)
							num_img=num_img+1
							shoot_marker(cut_img_resize_rgb, prcssimg_pub)

					img_switch_counter += 1
				else :
					img_switch_counter = 0

			cv2.waitKey(1)
