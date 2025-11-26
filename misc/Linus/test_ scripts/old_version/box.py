import cv2 as cv
import numpy as np

img = cv.imread('data/final möglich.png')

img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

# Maskenerstellung
#lower_green = np.array([ 40,  50,  50])
#upper_green = np.array([ 80, 255, 255])
#mask_green = cv.inRange(img_hsv, lower_green, upper_green)


lower_red = cv.inRange(img_hsv, np.array([  0, 50, 50]), np.array([ 10,255,255]))
upper_red = cv.inRange(img_hsv, np.array([170, 50, 50]), np.array([180,255,255]))
mask_red = lower_red | upper_red

# Konturen finden
#contours_green, _ = cv.findContours(mask_green, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
contours_red, _ = cv.findContours(mask_red, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

#print(f"Grüne Konturen gefunden: {len(contours_green)}")
print(f"Rote Konturen gefunden: {len(contours_red)}")


#if len(contours_green) > 0:
    #largest_green = max(contours_green, key=cv.contourArea)
    
    #x_g, y_g, w_g, h_g = cv.boundingRect(largest_green)
    
    #print(f"Grüner Balken gefunden: ")
    #print(f"Position: x={x_g}, y={y_g}")
    #print(f"Größe: width={w_g}, height={h_g}")

if len(contours_red) > 0:
    largest_red = max(contours_red, key=cv.contourArea)

    x_r, y_r, w_r, h_r = cv.boundingRect(largest_red)
    
    print(f"Rotes Quadrat gefunden: ")
    print(f"Position: x={x_r}, y={y_r}")
    print(f"Größe: width={w_r}, height={h_r}")


center_x = x_r + w_r // 2 
center_y = y_r + h_r // 2

print(f"Mitte des roten Markers ({center_x}, {center_y})")

picture_height, picture_width = img.shape[:2]
pixel_to_mid = x_r - (picture_width)

print(f"pixel_to_mid: {pixel_to_mid}")