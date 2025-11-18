import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import math

class AMD():
    """
    ArUco marker detection

    The class encapsulates an OpenCV ArUco detector, the camera parameters
    and the logic to obtain the depth (Z-coordinate) and Y-rotation of a
    detected marker from a single image.
    """

    def __init__(self):
        """
        Initialise the ArUco detector and the camera parameters.

        The marker dictionary is a predefined 5x5 dictionary containing
        1000 possible IDs. The camera matrix and distortion coefficients
        are hard-coded and correspond to a previously run script 
        calibrating the camera of the used device.
        """
        self.marker_size = 175  # mm
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
        """Detect ArUco marker and return its depth and rotation.

        Parameters
        ----------
        img : numpy.ndarray
            Input image in BGR format.

        Returns
        -------
        tuple
            (z_pos, y_rot) where
            - z_pos is the depth (Z-coordinate) in millimeters,
            - y_rot is the Y-rotation in radians.
            If no marker is detected, returns (-1.0, math.pi)."""
        frame = img
        frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(frame_gray)

        if ids is not None and len(corners) > 0:
            for i in range(len(ids)):
                obj_points = np.array([[-self.marker_size / 2,  self.marker_size / 2, 0],
                                       [ self.marker_size / 2,  self.marker_size / 2, 0],
                                       [ self.marker_size / 2, -self.marker_size / 2, 0],
                                       [-self.marker_size / 2, -self.marker_size / 2, 0]],
                                      dtype=np.float32)

                success, rvec, tvec = cv.solvePnP(
                    obj_points,
                    corners[i][0],
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv.SOLVEPNP_IPPE_SQUARE)

                if success:
                    return tvec[2], rvec[2]

        return -1.0, math.pi    # Fehler-Return