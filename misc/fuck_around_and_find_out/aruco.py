import cv2 as cv
import cv2.aruco as aruco
import numpy as np

# Initialisieren der Kamera
img = cv.VideoCapture(0)

while True:
    # Lesen eines Frames
    success, frame = img.read()

    # Umwandeln des Frames in Graustufen
    frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # Erkennen des ArUco-Markers
    corners, ids, rejectedImgPoints = cv.aruco.detectMarkers(
        frame_gray, aruco.DICT_5X5_1000, parameters=None)

    # Bestimmen der Entfernung und des Winkels
    if len(corners) > 0:
        rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(
            corners, 100, np.eye(3), np.zeros(4))

        # Ausgeben der Ergebnisse
        print("Entfernung:", tvecs)
        print("Winkel:", rvecs)

    # Anzeigen des Frames
    cv.imshow('Frame', frame)

    # Beenden des Programms
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# Schließen der Kamera
img.release()
cv.destroyAllWindows()