#! /usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import csv
import os.path

# rosでopencvが競合する場合
#import sys
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

import cv2
from cv2 import aruco


# テスト用
#import getState

np.set_printoptions(suppress=True)



def sharpness(img, kernel_num):
	if kernel_num == 0:
		return img
	elif kernel_num == 1:
		kernel = np.array([[0, -1, 0],
						   [0,  3, 0],
						   [0, -1, 0]])
	elif kernel_num == 2:
		kernel = np.array([[ 0, -1,  0],
						   [-1,  5, -1],
						   [ 0, -1,  0]])
	elif kernel_num == 3:
		kernel = np.array([[-1, -1, -1],
						   [-1,  9, -1],
						   [-1, -1, -1]])
	elif kernel_num == 4:
		kernel = np.array([[-1, -2, -1],
						   [-2, 12, -2],
						   [-1, -2, -1]])
	elif kernel_num == 5:
		kernel = np.array([[1,  4,    6,  4, 1],
						   [4, 16,   24, 16, 4],
						   [6, 24, -476, 24, 6],
						   [4, 16,   24, 16, 4],
						   [1,  4,    6,  4, 1]])
	else:
		return img

	sharpness_img = cv2.filter2D(img, -1, kernel)
	return sharpness_img


def drawMarker(img):
	dictionary = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
	corners, detected_ids, rejectedImgPoints = aruco.detectMarkers(img, dictionary)
	aruco.drawDetectedMarkers(img, corners, detected_ids, (0,255,0))
	return img


def main():
	img = cv2.imread('45deg_nagai.png')
	#draw_img = drawMarker(sharpness_img)
	cv2.imshow("origin", img)
	#cv2.imshow("sharpness", sharpness_img)
	
	for i in range(1,6):
		sharpness_img = sharpness(img, i)
		draw_img = drawMarker(sharpness_img)
		name = "kernel" + str(i)
		cv2.imshow(name, draw_img)

	cv2.waitKey(0)


if __name__ == '__main__':
	main()





# reference

# https://qiita.com/TaroYamada/items/e3f3d0ea4ecc0a832fac
# https://blog.capilano-fw.com/?p=1990
