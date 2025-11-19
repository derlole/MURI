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
        
        
        self.camera_matrix = np.array([
            [1856.5594482378056, 0.0, 971.2508020385866],
            [0.0, 1854.2218241989967, 472.4387993168578],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)

        self.dist_coeffs = np.array([
            [0.4258768890441897],
            [-2.58573901335018],
            [-0.029226787102926456],
            [-0.005191425823853011],
            [ 6.748360238575704]
        ], dtype=np.float32)

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
        
        frame_gray = img
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
                    angle_rad = self.calculate_angle_to_marker(corners[i])
                    
                    print(f'tvec: 0:{tvec[0]}    1:{tvec[1]}     2:{tvec[2]}')
                    print(f'Berechneter Winkel: {angle_rad} rad ({math.degrees(angle_rad):.2f}°)')
                    
                    return tvec[2][0], angle_rad
    
        return -1000.0, math.pi
    
    def calculate_angle_to_marker(self, corners):
        """
        Berechnet den horizontalen Winkel vom Kameramittelpunkt zum Markermittelpunkt.
        
        Parameters
        ----------
        corners : numpy.ndarray
            Die Ecken des detektierten Markers (von detectMarkers).
        
        Returns
        -------
        float
            Winkel in Radiant. 
            - 0 rad = Marker ist in Bildmitte
            - Positiv = Marker ist rechts von der Mitte
            - Negativ = Marker ist links von der Mitte
        """
        marker_corners = corners[0]
        marker_center_x = np.mean(marker_corners[:, 0])
        marker_center_y = np.mean(marker_corners[:, 1])
        
        cx = self.camera_matrix[0, 2]  # 971.25
        cy = self.camera_matrix[1, 2]  # 472.44
        
        fx = self.camera_matrix[0, 0]  # 1856.56
        
        delta_x = marker_center_x - cx
        
        angle_rad = math.atan2(delta_x, fx)
        
        return angle_rad