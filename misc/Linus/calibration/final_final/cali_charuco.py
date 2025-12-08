import numpy as np
import cv2
import yaml
import time

# ChArUco Board Parameter
SQUARES_X = 7  # Anzahl Quadrate in X-Richtung
SQUARES_Y = 5  # Anzahl Quadrate in Y-Richtung
BOARD_WIDTH = 170  # mm
BOARD_HEIGHT = 119  # mm

# Berechne Quadratgröße
SQUARE_LENGTH = BOARD_WIDTH / SQUARES_X  # mm pro Quadrat
MARKER_LENGTH = SQUARE_LENGTH * 0.75  # ArUco Marker sind typisch 75% der Quadratgröße

# ChArUco Board erstellen
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard(
    (SQUARES_X, SQUARES_Y),
    SQUARE_LENGTH,
    MARKER_LENGTH,
    aruco_dict
)
charuco_detector = cv2.aruco.CharucoDetector(board)

# Datensammlungen
all_charuco_corners = []
all_charuco_ids = []
image_size = None

# Kamera initialisieren
cap = cv2.VideoCapture(0)
# Optional: Höhere Auflösung
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

found = 0
required_images = 20

print(f"Sammle {required_images} Kalibrierungsbilder...")
print("Bewege das ChArUco Board in verschiedene Positionen und Winkel")

while found < required_images:
    ret, img = cap.read()
    if not ret:
        print("Fehler beim Lesen der Kamera")
        break
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    if image_size is None:
        image_size = gray.shape[::-1]
    
    # ChArUco Ecken detektieren
    charuco_corners, charuco_ids, marker_corners, marker_ids = charuco_detector.detectBoard(gray)
    
    # Mindestens 6 Ecken für gute Kalibrierung
    if charuco_corners is not None and len(charuco_corners) > 6:
        all_charuco_corners.append(charuco_corners)
        all_charuco_ids.append(charuco_ids)
        found += 1
        print(f'Bild {found}/{required_images} erfasst - {len(charuco_corners)} Ecken gefunden')

        # img_with_markers = cv2.aruco.drawDetectedMarkers(img.copy(), marker_corners, marker_ids)
        # img_with_corners = cv2.aruco.drawDetectedCornersCharuco(img_with_markers, charuco_corners, charuco_ids)
        # cv2.imshow('ChArUco Detection', img_with_corners)
        
        time.sleep(1.0)  # Warte 1 Sekunde zwischen Aufnahmen
    
    # if cv2.waitKey(10) & 0xFF == ord('q'):
    #     break

cap.release()
# cv2.destroyAllWindows()

print("\nStarte Kalibrierung...")

# Kalibrierung durchführen
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
    all_charuco_corners,
    all_charuco_ids,
    board,
    image_size,
    None,
    None
)

if ret:
    print(f"\nKalibrierung erfolgreich!")
    print(f"Reprojection Error: {ret:.4f} Pixel")
    
    # Ergebnisse speichern
    data = {
        'camera_matrix': np.asarray(camera_matrix).tolist(),
        'dist_coeff': np.asarray(dist_coeffs).tolist(),
        'reprojection_error': float(ret),
        'board_config': {
            'squares_x': SQUARES_X,
            'squares_y': SQUARES_Y,
            'square_length': float(SQUARE_LENGTH),
            'marker_length': float(MARKER_LENGTH)
        }
    }
    
    with open("charuco_cali.yaml", "w") as f:
        yaml.dump(data, f)
    
    print("\nKamera-Matrix:")
    print(camera_matrix)
    print("\nVerzerrungskoeffizienten:")
    print(dist_coeffs)
    print("\nKalibrierung in 'charuco_cali.yaml' gespeichert!")
else:
    print("Kalibrierung fehlgeschlagen!")