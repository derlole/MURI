import cv2 as cv
import matplotlib.pyplot as plt

cam = cv.VideoCapture(0)
success, img = cam.read()
if success:
    plt.imshow(cv.cvtColor(img, cv.COLOR_RGB2BGR))
    plt.axis('off')
    plt.show()
else:
    print("Failed")
