import pathlib
import cv2 as cv
import numpy as np
import pytest
from aruco_marker_detection import AMD

# Hilfsfunktion: Bild laden (kann später durch Fixtures ersetzt werden)
def load_image(rel_path: str) -> np.ndarray:
    """Lädt ein Bild aus dem Test‑Verzeichnis und gibt es als BGR‑Array zurück."""
    img_path = pathlib.Path(__file__).parent / rel_path
    img = cv.imread(str(img_path))
    if img is None:
        raise FileNotFoundError(f"Bild nicht gefunden: {img_path}")
    return img

# Fixtures für die beiden Testbilder
@pytest.fixture(scope="module")
def img_max():
    """Bild, das die maximale Distanz zum Marker erzeugt."""
    return load_image("images/max_distance.jpg")

@pytest.fixture(scope="module")
def img_min():
    """Bild, das die minimale Distanz zum Marker erzeugt."""
    return load_image("images/min_distance.jpg")

# Grundlegender Setup‑Fixture
@pytest.fixture
def detector():
    """Instanziiert die AMD‑Klasse mit den vordefinierten Kameraparametern."""
    return AMD()

# Test für die maximale Distanz
def test_max_distance(detector, img_max):
    """
    Erwartet, dass die ermittelte Z‑Position (Tiefe) im Bereich
    der maximal erwarteten Distanz liegt.
    """
    z_pos, _ = detector.aruco_detection(img_max)

    # Beispiel‑Grenzwerte – an dein Setup anpassen
    max_expected = 2000.0   # mm
    min_expected = 1500.0   # mm

    assert min_expected <= z_pos <= max_expected, (
        f"Max‑Distanz‑Test fehlgeschlagen: {z_pos:.1f} mm "
        f"liegt nicht im erwarteten Bereich [{min_expected}, {max_expected}]"
    )

# Test für die minimale Distanz
def test_min_distance(detector, img_min):
    """
    Erwartet, dass die ermittelte Z‑Position (Tiefe) im Bereich
    der minimalen erwarteten Distanz liegt.
    """
    z_pos, _ = detector.aruco_detection(img_min)

    # Beispiel‑Grenzwerte – an dein Setup anpassen
    max_expected = 500.0    # mm
    min_expected = 100.0    # mm

    assert min_expected <= z_pos <= max_expected, (
        f"Min‑Distanz‑Test fehlgeschlagen: {z_pos:.1f} mm "
        f"liegt nicht im erwarteten Bereich [{min_expected}, {max_expected}]"
    )