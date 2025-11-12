import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import pickle
import math

class AMD():
    def __init__(self):
        self.marker_size = 200  # mm
        
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
        aruco_params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(aruco_dict, aruco_params)
        self.aruco_dict = aruco_dict
        
        self.camera_matrix = np.array([[604.1946504636386, 
                                        0.0, 311.1787729991119], 
                                        [0.0, 604.2044268889094, 
                                         237.6933460968677], 
                                         [0, 0, 1]], dtype=np.float32)
        
        self.dist_coeffs = np.array([[0.19090119661845203],
                              [-1.8264546666600625],
                              [0.006275358413548046],
                              [-0.0008178156020032578],
                              [7.176765962359053]], dtype=np.float32)
        
    def aruco_detection(self, img):
        frame = img
        
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