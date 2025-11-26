import cv2 as cv
import numpy as np
import math

# --- HILFSFUNKTION (Deine Implementierung) ---
def calculate_angle_to_marker(marker_corners, camera_matrix):
    """
    Berechnet den horizontalen Winkel vom Kameramittelpunkt zum Markermittelpunkt.
    
    Parameters
    ----------
    marker_corners : numpy.ndarray
        Die Ecken des spezifischen Markers (z.B. corners[i]).
    camera_matrix : numpy.ndarray
        Die Kameramatrix.
        
    Returns
    -------
    float
        Winkel in Radiant.
    """
    # Berechne den Mittelpunkt des Markers in Pixel-Koordinaten
    # marker_corners hat shape (1, 4, 2)
    c = marker_corners[0]
    marker_center_x = np.mean(c[:, 0])
    
    # Bildmittelpunkt aus der Kameramatrix (Principal Point)
    cx = camera_matrix[0, 2] 
    
    # Brennweite in x-Richtung
    fx = camera_matrix[0, 0] 
    
    # Berechne die Pixel-Abweichung von der Bildmitte
    delta_x = marker_center_x - cx
    
    # Berechne den Winkel über Arcustangens
    angle_rad = math.atan2(delta_x, fx)
    
    return angle_rad

# ---------------------------------------------

# Marker size in mm
marker_size = 175

# Initialisieren der Kamera
img = cv.VideoCapture(0)
# Hinweis: Nicht alle Kameras unterstützen jede Auflösung. 
# Falls das Bild schwarz bleibt, hier Standardwerte (640x480) testen.
#img.set(cv.CAP_PROP_FRAME_WIDTH, 1920)
#img.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)

# Define the ArUco dictionary
aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_1000)
aruco_params = cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(aruco_dict, aruco_params)

# Camera matrix and distortion coefficients (DEINE WERTE aus der Test-Datei)
camera_matrix = np.array([
    [485.98256352079204, 0.0,           326.8156596599855],
    [0.0,                481.9114474146188, 231.52240485863123],
    [0.0,                0.0,           1.0]
], dtype=np.float32)

dist_coeffs = np.array([
    [-0.28206812200227427],
    [ 2.61448314076083],
    [ 0.020889912770543827],
    [ 0.017394594564323452],
    [-7.331368052181177]
], dtype=np.float32)


while True:
    # Lesen eines Frames
    success, frame = img.read()
    if not success:
        print("Fehler beim Lesen des Frames")
        break
    
    # Optional: Ausgabe der Shape nur einmalig oder seltener, sonst spammt es die Konsole voll
    # print(f'-----{frame.shape}')
    
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
            success_pnp, rvec, tvec = cv.solvePnP(
                obj_points, 
                corners[i][0], 
                camera_matrix, 
                dist_coeffs,
                flags=cv.SOLVEPNP_IPPE_SQUARE
            )
            
            if success_pnp:
                # --- HIER IST DEINE NEUE BERECHNUNG ---
                angle_rad = calculate_angle_to_marker(corners[i], camera_matrix)
                dist_mm = tvec[2][0]
                
                # Ausgeben der Ergebnisse in Konsole
                print(f"ID {ids[i][0]} | Dist: {dist_mm:.1f}mm | Winkel: {angle_rad:.4f} rad ({math.degrees(angle_rad):.1f}°)")
                
                # Visualisierung im Bild (Overlay)
                # Zeichne Achsen
                cv.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, marker_size/2)
                
                # Zeichne Text ins Bild (Links oben vom Marker)
                top_left_corner = corners[i][0][0].astype(int)
                cv.putText(frame, f"Dist: {int(dist_mm)}mm", (top_left_corner[0], top_left_corner[1] - 20), 
                           cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv.putText(frame, f"Ang: {math.degrees(angle_rad):.1f} deg", (top_left_corner[0], top_left_corner[1] - 50), 
                           cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # Draw detected markers borders
        cv.aruco.drawDetectedMarkers(frame, corners, ids)
    
    # Anzeigen des Frames
    # Frame evtl. verkleinern für Anzeige, falls 2600px zu breit für deinen Bildschirm ist
    disp_frame = cv.resize(frame, (1280, 960)) if frame.shape[1] > 1920 else frame
    cv.imshow('Frame', disp_frame)
    
    # Beenden des Programms mit 'q'
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# Schließen der Kamera
img.release()
cv.destroyAllWindows()