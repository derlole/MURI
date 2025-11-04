import cv2 as cv
import numpy as np
import matplotlib

cam = cv.VideoCapture(0)

while True:
    success, frame = cam.read()

    #cv.imshow(winname="circle", mat=frame)
    #cv.waitKey(0)
    #cv.destroyAllWindows()

    print("Type: " + str(type(frame)))

    img_conv = cv.cvtColor(src = frame, code = cv.COLOR_RGB2GRAY)
    print("Shape: " + str(img_conv.shape) + " Size: " + str(img_conv.size))

    img_conv_blur = cv.GaussianBlur(src=img_conv, ksize=(9,9), sigmaX=0)
    print("Shape: " + str(img_conv_blur.shape) + " Size: " + str(img_conv_blur.size))


    circles = cv.HoughCircles(image=img_conv_blur,
                    method=cv.HOUGH_GRADIENT, 
                    dp=1, 
                    minDist= 150, # Mindestabstand zwischen Kreisen
                    param1= 120, # Canny Edge Detection Schwellwert
                                #    Intern nutzt HoughCircles den Canny-Algorithmus zur Kantenerkennung                    
                                #    Intern nutzt HoughCircles den Canny-Algorithmus zur Kantenerkennung
                                #   Hoher Wert (z.B. 200): Nur starke Kanten → weniger Rauschen, aber evtl. Kreise übersehen
                                #  Niedriger Wert (z.B. 50): Auch schwache Kanten → mehr Kreise, aber mehr Falsch-Positive
                                # Standard: 100 ist ein guter Startwert

                    param2= 35, # Bestimmt, wie "sicher" ein Kreis sein muss, um erkannt zu werden UMSO GRÖẞER UMSO GENAUER
                    minRadius=2, #radius der Kreise in Pixeln 
                    maxRadius=80) #maximaler Radius der Kreise in Pixeln

    print("Circles: " + str(circles))

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            # Zeichne den äußeren Kreis
            cv.circle(img=frame, center=(i[0], i[1]), radius=i[2], color=(0, 255, 0), thickness=2)
            # Zeichne das Zentrum des Kreises
            cv.circle(img=frame, center=(i[0], i[1]), radius=2, color=(0, 0, 255), thickness=3)

    cv.imshow(winname="circle", mat=frame)
    if cv.waitKey(1) & 0xFF == ord('q'):  # Bei 'q' Taste beenden
        break
cam.release()
cv.destroyAllWindows()


