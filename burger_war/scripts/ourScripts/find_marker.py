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
from aruco_msgs.msg import MarkerArray


# グローバル変数
## Camera Image
burger_cv_cam_img = []


class OurSubscriber(object):

	def __init__(self):
		#ROS subscriber
		self.camcmprss_sub = rospy.Subscriber('image_raw/compressed', CompressedImage, self.GetCamCmprssImgCallback, queue_size=1)
		self.conv_start_time = time.time()
		self.conv_now_time = self.conv_start_time
		self.conv_elapsed_time = self.conv_now_time - self.conv_now_time
	
	def GetCamCmprssImgCallback(self, given_data):		# self, おsubした画像msg
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


# マーカを探して切り出し
def find_marker(src_img):		# カメラ画像
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

# 画像のマーカー部分を認識しやすいように処理
# ※ 画像処理については手法・パラメータ含めて検討中
def make_marker_clear(src_img):			# カメラ画像
	prcssed_img = copy.deepcopy(src_img)
	iteration_num = 1
	"""
	#gssblr_img = cv2.GaussianBlur(prcssed_img,(5,5),0)
	#eqhist_img = cv2.equalizeHist(biltrl_img)
	#cntrst_img = contrast(biltrl_img, 20)
	biltrl_img = cv2.bilateralFilter(prcssed_img, 9, 75, 75)
	clahe = cv2.createCLAHE(clipLimit=1.0, tileGridSize=(8,8))
	clahe_img = clahe.apply(biltrl_img)
	ret_temp, thrshld_img = cv2.threshold(clahe_img,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
	prcssed_img = thrshld_img
	"""
	biltrl_img = cv2.bilateralFilter(prcssed_img, 9, 20, 20)
	for i in range(iteration_num-1):
		biltrl_img_temp = cv2.bilateralFilter(biltrl_img, 9, 20, 20)
		biltrl_img = copy.deepcopy(biltrl_img_temp)
	ret_temp, thrshld_img = cv2.threshold(biltrl_img,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
	prcssed_img = thrshld_img
	#prcssed_img = biltrl_img

	return prcssed_img

# 画像中の円を探して位置と大きさを取得（認識した円を重ね書きした画像も返します）
def find_circ_of_target(src_img):		# カメラ画像
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

# 嶋ちゃん関数より　青円を探して四角で囲うぜ。四角の位置と縦横辺の長さを返すぜ（認識箇所に四角を重ね書きした画像も返します）
def find_rect_of_target_color(src_img):		# カメラ画像
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

	"""
	for contour in contours:
		approx = cv2.convexHull(contour)
		area = cv2.contourArea(approx)
		if area < 600:
			continue
		x,y,w,h = cv2.boundingRect(approx)
		#あまりに細長い領域は除外
		if (w>3.0*h or 3.0*w<h):
			continue
		boxes_pos.append([x+w/2, y+h/2])
		bounding_boxes_shape.append([2*w, 2*h])
		prcssed_img = cv2.rectangle(src_img,(x-w/2,y-h/2),(x+3*w/2,y+3*h/2),(0,255,0),2)
	"""

	return [prcssed_img, boxes_pos, bounding_boxes_shape]


# マーカー画像をシュゥウウウウウト! エキサイティイイイイイング！！ バトルd
def shoot_marker(src_img, img_pub):		# マーカー画像, おpub先
	our_cvbridge = CvBridge()	# cv <==> img msg 変換ブリッジ
	prcssed_img_msg = our_cvbridge.cv2_to_imgmsg(src_img, encoding="bgr8")
	img_pub.publish(prcssed_img_msg)


if __name__ == '__main__':
	rospy.init_node("find_marker")
	OurSubscriber()			# センサ情報おsub用
	prcssimg_pub = rospy.Publisher('image_prcssed', Image, queue_size=1)	# aruco サーバへマーカ画像をおpubする用
	markerid_pub = rospy.Publisher('target_id', MarkerArray, queue_size=1)	# cv2.aruco.detectMarkers で取得したidをおpubする用

	our_cvbridge = CvBridge()

	prcssed_img = []			# マーカー認識位置重ね描き後のカメラ画像
	cut_img = []				# マーカー切り出し画像
	cut_img_resize = []			# リサイズ後のマーカー切り出し画像
	img_switch_counter = 0		# マーカを複数切り出すので描画やおpubの切り替えに使用
	marker_switch_counter = 0	# マーカ複数描画やおpubの切り替えに使用

	start_time = time.time()				# 画像描画・おpub用タイマー
	"""
	cv2.namedWindow("Camera Image")			# カメラ画像（マーカ位置重ね書き後）描画用窓
	cv2.namedWindow("Cut Image_")			# 切り出し画像（リサイズ前）描画用窓
	cv2.namedWindow("Cut Image Resized")	# 切り出し画像（リサイズ後）描画用窓
	cv2.namedWindow("Marker Image")			# マーカー画像描画用窓
	"""
	
	# cv2.aruco.detectMarkers 用辞書を用意
	aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
	gotten_targetid_msg = MarkerArray()

	marker_img_size = 35	# 35pxx35px (7マスx7マス --> 1マス5px)

	loop_timer = rospy.Rate(30)     # ループの時間調整用　30[Hz]

	while not rospy.is_shutdown():
		now_time = time.time()
		elapsed_time = now_time - start_time
		if elapsed_time > 0.033:	# fps = 30 （のつもり）
			start_time = now_time
			# カメラ画像（マーカ位置重ね書き後）描画
			if len(burger_cv_cam_img) > 0:
				prcssed_img, cut_img, cut_img_resize = find_marker(burger_cv_cam_img)
				#cv2.imshow("Camera Image", prcssed_img)
			# 切り出し画像（リサイズ前）描画
			if len(cut_img) > 0:
				if img_switch_counter < len(cut_img):
					if len(cut_img[img_switch_counter]) > 0:
						if cut_img[img_switch_counter].shape[0] > 0 and cut_img[img_switch_counter].shape[1] > 0:
							cut_img_rgb = cv2.cvtColor(cut_img[img_switch_counter],cv2.COLOR_GRAY2RGB)
				#			cv2.imshow("Cut Image_", cut_img_rgb)
				#			prcssimg_msg = our_cvbridge.cv2_to_imgmsg(cut_img_rgb, encoding="bgr8")
				#			prcssimg_pub.publish(prcssimg_msg)
			# 切り出し画像（リサイズ後）描画 & おpub
			if len(cut_img_resize) > 0:
				if img_switch_counter < len(cut_img_resize):
					if len(cut_img_resize[img_switch_counter]) > 0:
						if cut_img_resize[img_switch_counter].shape[0] > 0 and cut_img_resize[img_switch_counter].shape[1] > 0:
							cut_img_resize_rgb = cv2.cvtColor(cut_img_resize[img_switch_counter],cv2.COLOR_GRAY2RGB)
				#			cv2.imshow("Cut Image Resized", cut_img_resize_rgb)
							shoot_marker(cut_img_resize_rgb, prcssimg_pub)
					img_switch_counter += 1
				else :
					img_switch_counter = 0
			
			if len(burger_cv_cam_img) > 0:
				marker_corners, marker_ids, rejectedImgPoints = cv2.aruco.detectMarkers(burger_cv_cam_img, aruco_dictionary)
				if marker_ids != None:
					if marker_switch_counter < len(marker_ids):
						marker_img = np.full((marker_img_size * 3, marker_img_size * 3), 255, dtype=np.uint8)
						marker_img[marker_img_size : marker_img_size * 2, marker_img_size : marker_img_size * 2] = cv2.aruco.drawMarker(aruco_dictionary, marker_ids[marker_switch_counter][0], marker_img_size)
						marker_img_bgr = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)						
				#		cv2.imshow("Marker Image", marker_img_bgr)
						prcssimg_msg = our_cvbridge.cv2_to_imgmsg(marker_img_bgr, encoding="bgr8")
						prcssimg_pub.publish(prcssimg_msg)
						marker_switch_counter += 1
					else :
						marker_switch_counter = 0
					"""
					for gotten_marker_id in marker_ids:
						marker_img = np.full((marker_img_size * 3, marker_img_size * 3), 255, dtype=np.uint8)
						marker_img[marker_img_size : marker_img_size * 2, marker_img_size : marker_img_size * 2] = cv2.aruco.drawMarker(aruco_dictionary, gotten_marker_id[0], marker_img_size)
						marker_img_bgr = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)
						#cv2.imshow("Marker Image", marker_img_bgr)
						prcssimg_msg = our_cvbridge.cv2_to_imgmsg(marker_img_bgr, encoding="bgr8")
						prcssimg_pub.publish(prcssimg_msg)
					"""

		cv2.waitKey(1)
		loop_timer.sleep()


