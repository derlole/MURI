import unittest
import cv2 as cv
import numpy as np
import math
import os
from ..aruco_marker_detection import AMD

TEST_DIR = os.path.dirname(os.path.abspath(__file__))


class TestAMD(unittest.TestCase):
    """Unit-Tests für die AMD (ArUco Marker Detection) Klasse"""
    
    @classmethod
    def setUpClass(cls):
        """Einmalige Initialisierung für alle Tests"""
        print("\n=== ArUco Marker Detection Tests ===")
        print("Marker-ID: 0")
        print("Marker-Größe: 175mm")
        print("Toleranzen: ±100mm (Distanz), ±5° (Winkel)\n")
    
    def setUp(self):
        """Initialisierung vor jedem Test"""
        self.amd = AMD()
        
        # Erwartete Werte
        self.expected_long_distance = 5300.0  # mm
        self.expected_short_distance = 400.0  # mm
        self.expected_angle = 0.0  # radians
        
        # Toleranzen
        self.distance_tolerance = 100.0  
        self.angle_tolerance = math.radians(5)
    
    def test_initialization(self):
        """Test: Korrekte Initialisierung der AMD-Klasse"""
        self.assertEqual(self.amd.marker_size, 175, 
                        "Marker-Größe wurde nicht korrekt gesetzt")
        
        self.assertIsNotNone(self.amd.detector, 
                            "ArUco-Detektor wurde nicht initialisiert")
        
        self.assertEqual(self.amd.camera_matrix.shape, (3, 3), 
                        "Kameramatrix hat falsche Dimensionen")
        
        self.assertIsNotNone(self.amd.dist_coeffs, 
                            "Verzerrungskoeffizienten wurden nicht gesetzt")
    
    def test_long_distance_detection(self):
        """Test: Marker-Erkennung bei ~5.5m Distanz (long.png)"""
        img = cv.imread(os.path.join(TEST_DIR, 'picture/long.png'), cv.IMREAD_GRAYSCALE)
        self.assertIsNotNone(img, 
                            "Bild 'picture/long.png' konnte nicht geladen werden")
        
        z_pos, y_rot = self.amd.aruco_detection(img)
        
        # Marker sollte erkannt werden (nicht Standardwert)
        self.assertNotEqual(z_pos, -1000.0, 
                           "Marker wurde nicht erkannt (long.png)")
        self.assertNotEqual(y_rot, math.pi, 
                           "Marker-Winkel wurde nicht berechnet (long.png)")
        
        # Distanz prüfen: ~5500mm ±100mm
        distance_error = abs(z_pos - self.expected_long_distance)
        self.assertLessEqual(distance_error, self.distance_tolerance,
                            f"Distanz {z_pos:.1f}mm weicht um {distance_error:.1f}mm "
                            f"von erwarteten {self.expected_long_distance:.1f}mm ab "
                            f"(Toleranz: ±{self.distance_tolerance:.1f}mm)")
        
        # Winkel prüfen: ~0° ±5°
        angle_error = abs(y_rot - self.expected_angle)
        self.assertLessEqual(angle_error, self.angle_tolerance,
                            f"Winkel {math.degrees(y_rot):.2f}° weicht um "
                            f"{math.degrees(angle_error):.2f}° von erwarteten 0° ab "
                            f"(Toleranz: ±{math.degrees(self.angle_tolerance):.1f}°)")
        
        print(f"✓ Long distance: z={z_pos:.1f}mm, angle={math.degrees(y_rot):.2f}°")
    
    def test_short_distance_detection(self):
        """Test: Marker-Erkennung bei ~40cm Distanz (short.png)"""
        img = cv.imread(os.path.join(TEST_DIR, 'picture/short.png'), cv.IMREAD_GRAYSCALE)
        self.assertIsNotNone(img, 
                            "Bild 'picture/short.png' konnte nicht geladen werden")
        
        z_pos, y_rot = self.amd.aruco_detection(img)
        
        # Marker sollte erkannt werden
        self.assertNotEqual(z_pos, -1000.0, 
                           "Marker wurde nicht erkannt (short.png)")
        self.assertNotEqual(y_rot, math.pi, 
                           "Marker-Winkel wurde nicht berechnet (short.png)")
        
        # Distanz prüfen: ~400mm ±100mm
        distance_error = abs(z_pos - self.expected_short_distance)
        self.assertLessEqual(distance_error, self.distance_tolerance,
                            f"Distanz {z_pos:.1f}mm weicht um {distance_error:.1f}mm "
                            f"von erwarteten {self.expected_short_distance:.1f}mm ab "
                            f"(Toleranz: ±{self.distance_tolerance:.1f}mm)")
        
        # Winkel prüfen: ~0° ±5°
        angle_error = abs(y_rot - self.expected_angle)
        self.assertLessEqual(angle_error, self.angle_tolerance,
                            f"Winkel {math.degrees(y_rot):.2f}° weicht um "
                            f"{math.degrees(angle_error):.2f}° von erwarteten 0° ab "
                            f"(Toleranz: ±{math.degrees(self.angle_tolerance):.1f}°)")
        
        print(f"✓ Short distance: z={z_pos:.1f}mm, angle={math.degrees(y_rot):.2f}°")
    
    def test_no_marker_detection(self):
        """Test: Verhalten bei fehlendem Marker (failure.png)"""
        img = cv.imread(os.path.join(TEST_DIR, 'picture/failure.png'), cv.IMREAD_GRAYSCALE)
        self.assertIsNotNone(img, 
                            "Bild 'picture/failure.png' konnte nicht geladen werden")
        
        z_pos, y_rot = self.amd.aruco_detection(img)
        
        # Sollte Standardwerte zurückgeben
        self.assertEqual(z_pos, -1000.0, 
                        "Falscher Standardwert für Distanz bei fehlendem Marker")
        self.assertEqual(y_rot, math.pi, 
                        "Falscher Standardwert für Winkel bei fehlendem Marker")
        
        print(f"✓ No marker (failure.png): z={z_pos:.1f}mm, angle={math.degrees(y_rot):.2f}°")
    
    def test_angle_calculation_center(self):
        """Test: Winkelberechnung für Marker im Bildzentrum"""
        # Simulierte Ecken eines Markers im Bildzentrum
        cx = self.amd.camera_matrix[0, 2]
        cy = self.amd.camera_matrix[1, 2]
        
        # Marker zentriert um cx, cy
        corners = np.array([[[cx - 20, cy - 20],
                             [cx + 20, cy - 20],
                             [cx + 20, cy + 20],
                             [cx - 20, cy + 20]]], dtype=np.float32)
        
        angle = self.amd.calculate_angle_to_marker(corners)
        
        # Winkel sollte nahe 0 sein
        self.assertAlmostEqual(angle, 0.0, places=3,
                              msg=f"Winkel für zentrierten Marker sollte ~0 sein, "
                                  f"ist aber {math.degrees(angle):.2f}°")
    
    def test_angle_calculation_right(self):
        """Test: Winkelberechnung für Marker rechts vom Zentrum"""
        cx = self.amd.camera_matrix[0, 2]
        cy = self.amd.camera_matrix[1, 2]
        
        # Marker rechts vom Zentrum
        offset = 100
        corners = np.array([[[cx + offset - 20, cy - 20],
                             [cx + offset + 20, cy - 20],
                             [cx + offset + 20, cy + 20],
                             [cx + offset - 20, cy + 20]]], dtype=np.float32)
        
        angle = self.amd.calculate_angle_to_marker(corners)
        
        # Winkel sollte positiv sein (rechts = positiv)
        self.assertGreater(angle, 0.0, 
                          "Winkel sollte positiv sein für Marker rechts vom Zentrum")
        
        print(f"✓ Right offset: angle={math.degrees(angle):.2f}°")
    
    def test_angle_calculation_left(self):
        """Test: Winkelberechnung für Marker links vom Zentrum"""
        cx = self.amd.camera_matrix[0, 2]
        cy = self.amd.camera_matrix[1, 2]
        
        # Marker links vom Zentrum
        offset = -100
        corners = np.array([[[cx + offset - 20, cy - 20],
                             [cx + offset + 20, cy - 20],
                             [cx + offset + 20, cy + 20],
                             [cx + offset - 20, cy + 20]]], dtype=np.float32)
        
        angle = self.amd.calculate_angle_to_marker(corners)
        
        # Winkel sollte negativ sein (links = negativ)
        self.assertLess(angle, 0.0, 
                       "Winkel sollte negativ sein für Marker links vom Zentrum")
        
        print(f"✓ Left offset: angle={math.degrees(angle):.2f}°")
    
    def test_camera_matrix_properties(self):
        """Test: Überprüfung der Kameramatrix-Eigenschaften"""
        # Hauptpunkt (cx, cy) sollte nahe der Bildmitte sein
        cx = self.amd.camera_matrix[0, 2]
        cy = self.amd.camera_matrix[1, 2]
        
        # Für 640x480 Auflösung
        self.assertTrue(200 < cx < 440, 
                       f"cx={cx:.1f} scheint nicht plausibel für 640x480 Auflösung")
        self.assertTrue(100 < cy < 380, 
                       f"cy={cy:.1f} scheint nicht plausibel für 640x480 Auflösung")
        
        # Brennweiten sollten positiv und ähnlich sein
        fx = self.amd.camera_matrix[0, 0]
        fy = self.amd.camera_matrix[1, 1]
        
        self.assertGreater(fx, 0, "Brennweite fx sollte positiv sein")
        self.assertGreater(fy, 0, "Brennweite fy sollte positiv sein")
        
        # fx und fy sollten ähnlich sein (Abweichung < 5%)
        ratio = fx / fy
        self.assertTrue(0.95 < ratio < 1.05, 
                       f"fx/fy={ratio:.3f} - Brennweiten weichen stark ab")


def run_tests_with_verbose_output():
    """Führt die Tests mit ausführlicher Ausgabe aus"""
    # Test-Suite erstellen
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Tests hinzufügen
    suite.addTests(loader.loadTestsFromTestCase(TestAMD))
    
    # Tests ausführen mit verbosity=2 für detaillierte Ausgabe
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # Zusammenfassung
    print("\n" + "="*70)
    print("ZUSAMMENFASSUNG")
    print("="*70)
    print(f"Tests durchgeführt: {result.testsRun}")
    print(f"Erfolgreich: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"Fehlgeschlagen: {len(result.failures)}")
    print(f"Fehler: {len(result.errors)}")
    print("="*70)
    
    return result.wasSuccessful()


if __name__ == '__main__':
    # Tests mit ausführlicher Ausgabe ausführen
    success = run_tests_with_verbose_output()
    
    # Exit-Code setzen (wichtig für CI/CD)
    exit(0 if success else 1)