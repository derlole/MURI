import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import pickle
import math
class AMD():
    def __init__(self, data_img=None, calibration_file='camera_calibration.pkl'):
        self.marker_size = 100  # mm
        
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
        aruco_params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(aruco_dict, aruco_params)
        self.aruco_dict = aruco_dict
        
        self.camera_matrix = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros((4, 1), dtype=np.float32)

        # Feld das für die Kalibrierung verwendet wird
        self.charuco_board = aruco.CharucoBoard(
            (7, 5),      # 7x5 Felder
            40.0,        # mm Feldgröße
            30.0,        # mm Marker
            aruco_dict
        )
        
    def calibrate_from_image(self, image_path):
        pass #TODO Kalibrierung einfügen
        
    def aruco_detection(self, img):
        frame = cv.imread(img)
        
        frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(frame_gray)

        if ids is not None and len(corners) > 0:
            for i in range(len(ids)):
                obj_points = np.array([
                    [-self.marker_size/2,  self.marker_size/2, 0],
                    [ self.marker_size/2,  self.marker_size/2, 0],
                    [ self.marker_size/2, -self.marker_size/2, 0],
                    [-self.marker_size/2, -self.marker_size/2, 0]
                ], dtype=np.float32)

                success, rvec, tvec = cv.solvePnP(
                    obj_points,
                    corners[i][0],
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv.SOLVEPNP_IPPE_SQUARE
                )
                if success:
                    return tvec[2], rvec[2]

        return -1.0, math.pi