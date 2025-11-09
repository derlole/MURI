import cv2 as cv
import numpy as np

# Marker size in mm
marker_size = 100

# Initialisieren der Kamera
img = cv.VideoCapture(0)

# Define the ArUco dictionary
aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_100)
aruco_params = cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(aruco_dict, aruco_params)

# Camera matrix and distortion coefficients
camera_matrix = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((4, 1), dtype=np.float32)

while True:
    # Lesen eines Frames
    success, frame = img.read()
    if not success:
        print("Fehler beim Lesen des Frames")
        break
    
    # Umwandeln des Frames in Graustufen
    frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    
    # Erkennen des ArUco-Markers
    corners, ids, rejectedImgPoints = detector.detectMarkers(frame_gray)
    
    if ids is not None and len(corners) > 0:
        # Für jeden erkannten Marker
        for i in range(len(ids)):
            # Define 3D object points for the marker corners
            obj_points = np.array([
                [-marker_size/2,  marker_size/2, 0],
                [ marker_size/2,  marker_size/2, 0],
                [ marker_size/2, -marker_size/2, 0],
                [-marker_size/2, -marker_size/2, 0]
            ], dtype=np.float32)
            
            # Estimate pose using solvePnP
            success, rvec, tvec = cv.solvePnP(
                obj_points, 
                corners[i][0], 
                camera_matrix, 
                dist_coeffs,
                flags=cv.SOLVEPNP_IPPE_SQUARE
            )
            
            if success:
                # Ausgeben der Ergebnisse
                print(f"Marker ID {ids[i][0]}:")
                print(f"  Entfernung (tvec): {tvec.flatten()}")
                print(f"  Rotation (rvec): {rvec.flatten()}")
                
                # Optional: Draw axis on the marker
                cv.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, marker_size/2)
        
        # Draw detected markers
        cv.aruco.drawDetectedMarkers(frame, corners, ids)
    
    # Anzeigen des Frames
    cv.imshow('Frame', frame)
    
    # Beenden des Programms
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# Schließen der Kamera
img.release()
cv.destroyAllWindows()