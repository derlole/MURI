import numpy as np
import cv2
import yaml

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

CHECKERBOARD = (6, 9)  # 6x9 innere Ecken bei 7x10 Feldern
SQUARE_SIZE = 24  # mm

objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp = objp * SQUARE_SIZE  # mm

objpoints = []  # 3d point world
imgpoints = []  # 2d points image
cap = cv2.VideoCapture(0)
found = 0

while found < 20:
    ret, img = cap.read()
    if not ret:
        print("Fehler beim Lesen der Kamera")
        break
        
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
    
    if ret == True:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        
        img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        found += 1
        print(f'Found: {found}')
    
    #cv2.imshow('img', img)
    #cv2.waitKey(1000)
    #if cv2.waitKey(10) & 0xFF == ord('q'):  # 'q' Abbruch
    #    break

cap.release()
#cv2.destroyAllWindows()
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

data = {
    'camera_matrix': np.asarray(mtx).tolist(),
    'dist_coeff': np.asarray(dist).tolist()
}

with open("calibration4.yaml", "w") as f:
    yaml.dump(data, f)

print("Kalibrierung abgeschlossen!")