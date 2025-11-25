import cv2 as cv
import numpy as np

img= cv.imread('data/final möglich.png')


img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

# Maskenerstellung
lower_green = np.array([35, 50, 50])      # untere Grenze
upper_green = np.array([85, 255, 255])    # obere Grenze

mask_green = cv.inRange(img_hsv, lower_green, upper_green)
# Konturen finden

contours, hierarchy = cv.findContours(mask_green, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

print(f"Grüne Konturen gefunden: {len(contours)}")

if len(contours) > 0:
        largest_green = max(contours, key=cv.contourArea)
        x_r, y_r, w_r, h_r = cv.boundingRect(largest_green)
        rectangle_center_x = x_r + w_r // 2
        rectangle_center_y = y_r + h_r // 2
        
        print(f"Günes Rechteck gefunden: ")
        print(f"Position: x={x_r}, y={y_r}")
        print(f"Größe: width={w_r}, height={h_r}")
        print(f"Mitte des Bereichs ({rectangle_center_x}, {rectangle_center_y})")

        picture_height, picture_width = frame.shape[:2]
        picture_mid_x = picture_width // 2
        pixel_to_mid_x = rectangle_center_x - picture_mid_x
        print(f"pixel_to_mid: {pixel_to_mid_x}")

        print(f"Höhe des grünen Vierecks: {h_r}")

else:
    print("Kein Rechteck gefunden...")
    