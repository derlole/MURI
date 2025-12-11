import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import math
import config

class AMD():
    '''
    ArUco marker detection

    The class encapsulates an OpenCV ArUco detector, the camera parameters
    and the logic to obtain the depth (Z-coordinate) and Y-rotation of a
    detected marker from a single image.
    '''
    def __init__(self):
        # Marker-Größen aus der Konfiguration laden
        self.marker_sizes = config.MARKER_SIZES
        
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
        aruco_params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(aruco_dict, aruco_params)

        self.aruco_dict = aruco_dict

        self.camera_matrix = np.array(config.CAMERA_MATRIX_RAW, dtype=np.float32)
        self.dist_coeffs = np.array(config.DISTANCE_COEFFICIENT, dtype=np.float32)

    def aruco_detection(self, img):
        ''' Detect ArUco marker and return its depth, rotation, and ID.

        Parameters
        ----------
        img : numpy.ndarray

        Returns
        ----------
        tuple
            (z_pos, y_rot, marker_id) where
            - z_pos is the depth (Z-coordinate) in millimeters,
            - y_rot is the Y-rotation in radians,
            - marker_id is the ID of the detected marker.
            If no marker is detected, returns (-1000.0, math.pi, 9999).
            If both markers 0 and 69 are detected, returns marker 69.
        '''
        frame_gray = img
        corners, ids, _ = self.detector.detectMarkers(frame_gray)
        
        if ids is not None and len(corners) > 0: 
            index_69 = None
            index_0 = None
            
            for i in range(len(ids)):
                if ids[i][0] == 69:
                    index_69 = i
                elif ids[i][0] == 0:
                    index_0 = i
            
            index_to_use = index_69 if index_69 is not None else index_0 # Wähle Marker 69, wenn er existiert, sonst Marker 0
            
            if index_to_use is not None:
                # Bestimme die Markergröße basierend auf der ID
                marker_id = ids[index_to_use][0]
                marker_size = self.marker_sizes.get(marker_id)
                
                # Wenn die Marker-ID nicht konfiguriert ist, Fehler zurückgeben
                if marker_size is None:
                    return -1000.0, math.pi, 9999
                
                obj_points = np.array([[-marker_size / 2,  marker_size / 2, 0],   # Obere linke Ecke
                                        [ marker_size / 2,  marker_size / 2, 0],  # Obere rechte Ecke
                                        [ marker_size / 2, -marker_size / 2, 0],  # Untere rechte Ecke
                                        [-marker_size / 2, -marker_size / 2, 0]], # Untere linke Ecke
                                    dtype=np.float32)
                
                success, _, self.tvec = cv.solvePnP(
                    obj_points,
                    corners[index_to_use][0],
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv.SOLVEPNP_IPPE_SQUARE)
                
                if success:
                    angle_rad = self.calculate_angle_to_marker(corners[index_to_use])
                    marker_id = ids[index_to_use][0]
                    return self.tvec[2][0], angle_rad, marker_id
    
        return -1000.0, math.pi, 9999
    
    def calculate_angle_to_marker(self):
        ''' Compute the yaw angle of a detected marker using 3D position data.
        
        Returns
        ----------
        float
            angle_rad in radians.
                Positive → marker right of camera,
                Negative → marker left of camera.
        '''
        distance = self.tvec[2][0]
        x_offset = self.tvec[0][0]
        
        # x_offset Gegenkathete; distance Ankathete
        angle_rad = math.atan2(x_offset, distance) 
        
        # Rückgabe in rad
        return angle_rad