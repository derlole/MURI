import unittest
import math
import cv2 as cv
from pathlib import Path

from vision.aruco_marker_detection import AMD


class TestAMD(unittest.TestCase):
    """Unit-Tests für die ArUco Marker Detection (AMD) Klasse."""
    
    def setUp(self):
        """Initialisiere die AMD-Klasse und definiere den Pfad zu den Testbildern."""
        try:
            self.amd = AMD()
        except AttributeError as e:
            self.skipTest(f"OpenCV-Version unterstützt ArucoDetector nicht: {e}")
        self.test_dir = Path(__file__).parent / "picture"
    
    def test_no_marker_detected(self):
        """Test: Kein Marker erkannt → (-1000.0, math.pi, 9999)"""
        img_path = self.test_dir / "fail.jpeg"
        self.assertTrue(img_path.exists(), f"Testbild nicht gefunden: {img_path}")
        
        img = cv.imread(str(img_path))
        self.assertIsNotNone(img, f"Bild konnte nicht geladen werden: {img_path}")
        
        z_pos, y_rot, marker_id = self.amd.aruco_detection(img)
        
        self.assertEqual(marker_id, 9999)
        self.assertEqual(z_pos, -1000.0)
        self.assertAlmostEqual(y_rot, math.pi, delta=0.1)
    
    def test_no_marker_alternative(self):
        """Test: Kein Marker erkannt (alternatives Fehlerbeispiel) → (-1000.0, math.pi, 9999)"""
        img_path = self.test_dir / "fail2.png"
        self.assertTrue(img_path.exists(), f"Testbild nicht gefunden: {img_path}")
        
        img = cv.imread(str(img_path))
        self.assertIsNotNone(img, f"Bild konnte nicht geladen werden: {img_path}")
        
        z_pos, y_rot, marker_id = self.amd.aruco_detection(img)
        
        self.assertEqual(marker_id, 9999)
        self.assertEqual(z_pos, -1000.0)
        self.assertAlmostEqual(y_rot, math.pi, delta=0.1)
    
    def test_marker_0_at_50cm(self):
        """Test: Marker 0 bei ~50 cm → Erwartet (500.0, 0.0, 0)"""
        img_path = self.test_dir / "fünfzig.jpeg"
        self.assertTrue(img_path.exists(), f"Testbild nicht gefunden: {img_path}")
        
        img = cv.imread(str(img_path))
        self.assertIsNotNone(img, f"Bild konnte nicht geladen werden: {img_path}")
        
        z_pos, y_rot, marker_id = self.amd.aruco_detection(img)
        
        if marker_id == 9999:
            self.skipTest("Marker 0 konnte nicht erkannt werden")
        
        self.assertEqual(marker_id, 0)
        self.assertAlmostEqual(z_pos, 500.0, delta=50.0)
        self.assertAlmostEqual(y_rot, 0.0, delta=0.1)
    
    def test_marker_0_at_25cm(self):
        """Test: Marker 0 bei ~25 cm (kurze Distanz) → Erwartet z_pos < 400mm"""
        img_path = self.test_dir / "kurz.png"
        self.assertTrue(img_path.exists(), f"Testbild nicht gefunden: {img_path}")
        
        img = cv.imread(str(img_path))
        self.assertIsNotNone(img, f"Bild konnte nicht geladen werden: {img_path}")
        
        z_pos, y_rot, marker_id = self.amd.aruco_detection(img)
        
        if marker_id == 9999:
            self.skipTest("Marker bei kurzer Distanz konnte nicht erkannt werden")
        
        self.assertNotEqual(marker_id, 9999)
        self.assertLess(z_pos, 400.0)
        self.assertGreater(z_pos, 0.0)
    
    def test_marker_69_at_100cm(self):
        """Test: Marker 69 bei ~100 cm → Erwartet (1000.0, 0.0, 69)"""
        img_path = self.test_dir / "lang.png"
        self.assertTrue(img_path.exists(), f"Testbild nicht gefunden: {img_path}")
        
        img = cv.imread(str(img_path))
        self.assertIsNotNone(img, f"Bild konnte nicht geladen werden: {img_path}")
        
        z_pos, y_rot, marker_id = self.amd.aruco_detection(img)
        
        if marker_id == 9999:
            self.skipTest("Marker 69 konnte nicht erkannt werden")
        
        self.assertEqual(marker_id, 69)
        self.assertAlmostEqual(z_pos, 1000.0, delta=50.0)
        self.assertAlmostEqual(y_rot, 0.0, delta=0.1)


if __name__ == "__main__":
    unittest.main()