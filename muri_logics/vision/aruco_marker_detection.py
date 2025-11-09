import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import pickle

class AMD():
    def __init__(self, data_img=None, calibration_file='camera_calibration.pkl'):
        self.marker_size = 100  # mm
        
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
        aruco_params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(aruco_dict, aruco_params)
        self.aruco_dict = aruco_dict
        
        self.charuco_board = aruco.CharucoBoard(
            (7, 5),      # 7x5 Felder
            40.0,        # mm Feldgröße
            30.0,        # mm Marker
            aruco_dict
        )
        
        # Kameraparameter laden oder Standardwerte
        self.calibration_file = calibration_file
        if self.load_calibration():
            print("Kalibrierung geladen!")
        else:
            print("Keine Kalibrierung gefunden - verwende Standardwerte")
            self.camera_matrix = np.array([[1000, 0, 320], 
                                          [0, 1000, 240], 
                                          [0, 0, 1]], dtype=np.float32)
            self.dist_coeffs = np.zeros((4, 1), dtype=np.float32)
        
        # Frame laden falls angegeben
        if data_img:
            self.frame = cv.imread(data_img)
    
    def calibrate_from_image(self, image_path):
        img = cv.imread(image_path)
        if img is None:
            print(f"Fehler: Bild nicht gefunden: {image_path}")
            return False
        
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        image_size = gray.shape[::-1]
        
        # Aruco Marker erkennen
        corners, ids, _ = self.detector.detectMarkers(gray)
        
        if ids is None or len(ids) < 4:
            print(f"Fehler: Zu wenige Marker erkannt ({len(ids) if ids is not None else 0}). Mindestens 4 benötigt.")
            return False
        
        ret, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
            corners, ids, gray, self.charuco_board
        )
        
        if ret < 4:
            print(f"Fehler: Zu wenige Charuco-Ecken ({ret}). Mindestens 4 benötigt.")
            return False
        
        print(f"✓ {ret} Charuco-Ecken erkannt")
        
        # Kalibrierung
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
            [charuco_corners],
            [charuco_ids],
            self.charuco_board,
            image_size,
            None,
            None
        )
        
        if ret:
            self.camera_matrix = camera_matrix
            self.dist_coeffs = dist_coeffs
            
            print("\n=== Kalibrierung erfolgreich! ===")
            print(f"Kamera-Matrix:\n{camera_matrix}")
            print(f"Verzerrung:\n{dist_coeffs.ravel()}")
            
            self.save_calibration()
            return True
        else:
            print("Fehler bei der Kalibrierung!")
            return False
    
    def save_calibration(self):
        """Speichert Kalibrierungsparameter"""
        data = {
            'camera_matrix': self.camera_matrix,
            'dist_coeffs': self.dist_coeffs
        }
        with open(self.calibration_file, 'wb') as f:
            pickle.dump(data, f)
        print(f"Kalibrierung gespeichert: {self.calibration_file}")
    
    def load_calibration(self):
        """Lädt Kalibrierungsparameter"""
        try:
            with open(self.calibration_file, 'rb') as f:
                data = pickle.load(f)
            self.camera_matrix = data['camera_matrix']
            self.dist_coeffs = data['dist_coeffs']
            return True
        except FileNotFoundError:
            return False
    
    def aruco_detection(self, frame):
        """Erkennt ArUco-Marker und berechnet Pose"""
        frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(frame_gray)
        
        rvecs = []
        tvecs = []
        
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
                    rvecs.append(rvec)
                    tvecs.append(tvec)
        
        return rvecs, tvecs, corners, ids