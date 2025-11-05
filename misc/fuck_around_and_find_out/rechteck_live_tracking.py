import cv2 as cv
import numpy as np

cam = cv.VideoCapture(0)

while True:
    img = cam.read()

    img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    # Maskenerstellung
    lower_red = cv.inRange(img_hsv, np.array([  0, 50, 50]), np.array([ 10,255,255]))
    upper_red = cv.inRange(img_hsv, np.array([170, 50, 50]), np.array([180,255,255]))
    mask_red = lower_red | upper_red

    # Konturen finden
    contours_red, _ = cv.findContours(mask_red, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    print(f"Rote Konturen gefunden: {len(contours_red)}")

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

    print(f"Höhe des roten Vierecks: {h_r}")

    cv.rectangle(img, (x_r, y_r), (x_r + w_r, y_r + h_r), (0, 255, 0), 2)
    cv.circle(img, (center_x, center_y), 4, (255, 0, 0), -1)

    cv.imshow(winname="rectangle", mat=img)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cam.release()
cv.destroyAllWindows()