#!/usr/bin/env python
import cv2
img = cv2.imread("opencv-findcontours_01.png")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
image, contours, hierarchy = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

cv2.imshow('image',image)
cv2.waitKey(0)
cv2.destroyAllWindows()