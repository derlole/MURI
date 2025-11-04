import cv2 as cv
import numpy as np

img = cv.imread('drawing.png')

img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

# Maskenerstellung
lower_green = np.array([ 40,  50,  50])
upper_green = np.array([ 80, 255, 255])
mask_green = cv.inRange(img_hsv, lower_green, upper_green)


lower_red = cv.inRange(img_hsv, np.array([  0, 50, 50]), np.array([ 10,255,255]))
upper_red = cv.inRange(img_hsv, np.array([170, 50, 50]), np.array([180,255,255]))
mask_red = lower_red | upper_red

contours_green = cv.findContours(mask_green, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
contours_red = cv.findContours(mask_green, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)