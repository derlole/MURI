import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import math
import config

class AMD():
    """Detektiert ArUco-Marker und berechnet 3D-Position (Distanz, Winkel)."""
    
    def __init__(self):
        """Initialisiert Detektor mit Config-Parametern aus config.py.
        
        Lädt: MARKER_SIZES, CAMERA_MATRIX_RAW, DISTANCE_COEFFICIENT
        Nutzt: DICT_5X5_1000, DetectorParameters()
        """
        # Marker-Größen aus der Konfiguration laden
        self.marker_sizes = config.MARKER_SIZES
        
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
        aruco_params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(aruco_dict, aruco_params)

        self.aruco_dict = aruco_dict

        self.camera_matrix = np.array(config.CAMERA_MATRIX_RAW, dtype=np.float32)
        self.dist_coeffs = np.array(config.DISTANCE_COEFFICIENT, dtype=np.float32)

    def aruco_detection(self, img):
        """Detektiert Marker und berechnet Position (Distanz, Winkel, ID).
        
        Args:
            img (numpy.ndarray): Grayscale-Bild
        
        Returns:
            tuple: (distanz_mm, winkel_rad, marker_id)
            
        Fehlerfall: (-1000.0, π, 9999) bei Nicht-Detektion/Fehler
        Priorität: Marker 69 > Marker 0
        """
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
                    angle_rad = self.calculate_angle_to_marker()
                    marker_id = ids[index_to_use][0]
                    return self.tvec[2][0], angle_rad, marker_id
    
        return -1000.0, math.pi, 9999
    
    def calculate_angle_to_marker(self):
        """Berechnet Yaw-Winkel (atan2 von X-Offset und Z-Distanz).
        
        Returns:
            float: Winkel in Rad [-π, π]
                Positiv=rechts, Negativ=links, 0≈zentriert
        """
        distance = self.tvec[2][0]
        x_offset = self.tvec[0][0]
        
        # x_offset Gegenkathete; distance Ankathete
        angle_rad = math.atan2(x_offset, distance) 
        
        # Rückgabe in rad
        return angle_rad