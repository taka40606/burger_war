# -*- coding: utf-8 -*-
#!/usr/bin/env python


import cv2
import numpy as np
import rospy
import sys
from cv_bridge import CvBridge, CvBridgeError

def green_recognition(image):
	#画素数
	wo = 126#480
	ho = 168#640

	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
	h = hsv[:, :, 0]
	s = hsv[:, :, 1]
	mask = np.zeros(h.shape, dtype=np.uint8)
	mask2 = np.zeros(h.shape, dtype=np.uint8)
	mask[(42 < h) & (h < 120) & (s > 100)] = 255#緑認識HSV

	image2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	area1 = cv2.contourArea(contours[0])
	#area2 = cv2.contourArea(contours[1])

	maxCont=contours[0]
	for c1 in contours:
		if len(maxCont)<len(c1):
			maxCont = c1
	x,y,w,h = cv2.boundingRect(maxCont)
	img = cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),2)
	cv2.imshow("rect_image",img)
	#距離計算
	x2 = float(w)
	y2 = float(h)
	#距離計算
	height_distance_standard = ho/(np.tan(0.3927))
	height_tan = (y2/2)/height_distance_standard#高さtan(th)
	height_rad = np.arctan(height_tan)
	height_deg =height_rad*180/np.pi
	distance = 0.04/height_tan
	distance = 0.92764*distance+0.088126
	#角度計算
	side_standard = wo/(np.tan(0.52359))
	side_tan = (x2/2)/side_standard
	side_rad = np.arctan(side_tan)
	side_deg = side_rad*180/np.pi
		
	
	print(distance,"mm")
	print(side_deg,"deg")
	return w,h,distance,side_deg


if __name__ == '__main__':
	#rospy.init_node("find_marker")
	#OurSubscriber()			# センサ情報おsub用
	#odom_sub = rospy.Subscriber('odom', Odometry, odomCallback)
	#prcssed_img = []			# マーカー認識位置重ね描き後のカメラ画像
	#cut_img = []				# マーカー切り出し画像
	#cut_img_resize = []			# リサイズ後のマーカー切り出し画像
	#img_switch_counter = 0		# マーカを複数切り出すので描画やおpubの切り替えに使用

	#start_time = time.time()				# 画像描画・おpub用タイマー
	#cv2.namedWindow("Camera Image2")			# カメラ画像（マーカ位置重ね書き後）描画用窓
	#cv2.namedWindow("Cut Image_")			# 切り出し画像（リサイズ前）描画用窓
	#cv2.namedWindow("Cut Image Contour")	# 切り出し画像（リサイズ後）描画用窓
	#rospy.init_node('get_enemy_position', anonymous=True)
    	#r = rospy.Rate(10) # 10hz
	
	#while True:
		# カメラ画像（マーカ位置重ね書き後）描画
		#if len(burger_cv_cam_img) > 0:
			#prcssed_img, distance, rad = enemy_position_calc(burger_cv_cam_img)
			#cv2.imshow("Camera Image2", prcssed_img)
			#if distance == 0:
				#print(lost)
			#else:
				#position = enemy_position(a,b,c,d,e)

	#while not rospy.is_shutdown():
		#r.sleep()
		#img = cv2.imread("opencv-findcontours_01.png")
	img = cv2.imread("0.5green.png")
	r = green_recognition(img)
  	#cv2.imshow('rezone',r)
	print(r)
  	cv2.waitKey(0)
  	cv2.destroyALLWIndows()
