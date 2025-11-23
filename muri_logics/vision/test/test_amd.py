import pathlib
import cv2 as cv
import numpy as np
import pytest
from aruco_marker_detection import AMD


# test_amd.py
import cv2 as cv
import math
import numpy as np
import unittest

from aruco_marker_detection import AMD   # <-- Pfad ggf. anpassen


class TestAMD(unittest.TestCase):
    """Test‑Suite für die AMD‑Klasse."""

    @classmethod
    def setUpClass(cls):
        """Erstelle ein AMD‑Objekt, das für alle Tests verwendet wird."""
        cls.amd = AMD()

    def test_no_marker(self):
        """Bild ohne ArUco‑Marker → erwartete Rückgabe (-1000.0, π)."""
        # Schwarzes Bild (keine Marker)
        img = np.zeros((480, 640, 3), dtype=np.uint8)

        z, y_rot = self.amd.aruco_detection(img)

        self.assertAlmostEqual(z, -1000.0, places=3)
        self.assertAlmostEqual(y_rot, math.pi, places=5)

    def test_with_marker(self):
        """Bild mit einem bekannten Marker → prüfe, dass ein Ergebnis zurückkommt."""
        # 1. Erstelle ein Test‑Marker‑Bild (ID = 0)
        marker_id = 0
        marker_size_px = 200
        aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_1000)
        marker_img = cv.aruco.generateImageMarker(
            aruco_dict, marker_id, marker_size_px, 1)

        # 2. Lege das Marker‑Bild in ein größeres Bild ein (Simulation einer Kameraaufnahme)
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        x_offset, y_offset = 220, 140
        img[y_offset:y_offset+marker_size_px,
            x_offset:x_offset+marker_size_px] = cv.cvtColor(marker_img, cv.COLOR_GRAY2BGR)

        # 3. Aufruf der Erkennungsfunktion
        z, y_rot = self.amd.aruco_detection(img)

        # 4. Prüfe, dass ein plausibles Ergebnis zurückkommt
        #    (Tiefe > 0 und Winkel im Bereich [-π/2, π/2])
        self.assertGreater(z, 0.0, "Tiefe sollte positiv sein")
        self.assertGreaterEqual(y_rot, -math.pi/2)
        self.assertLessEqual(y_rot, math.pi/2)

        # Optional: Ausgabe für manuelle Kontrolle
        print(f"\nErkannt – Tiefe: {z:.2f} mm, Winkel: {math.degrees(y_rot):.2f}°")

if __name__ == "__main__":
    unittest.main()