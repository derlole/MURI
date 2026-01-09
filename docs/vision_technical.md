# Vision-System: Technische Details & API-Dokumentation

<!-- Written, maintained and owned by Linus Braun (MURI DEVELOPMENT TEAM) -->

## Überblick: System-Architektur

```
┌──────────────────────┐
│   Hardware: Kamera   │  /dev/video0 (30 Hz)
│   (USB, 640×480)     │
└──────────┬───────────┘
           │
           ▼
┌──────────────────────────────┐
│  CameraReadOut (ROS2 Node)   │  Liest BGR, wandelt in Grayscale
│  - Timer: 30 Hz              │  - publiziert auf /muri_image_raw
└──────────┬────────────────────┘
           │ Topic: /muri_image_raw
           │ Typ: sensor_msgs/Image (mono8)
           ▼
┌──────────────────────────────┐
│ ImageProcessing (ROS2 Node)  │  Verarbeitet Bilder
│ - Subscriber                 │  - Ruft AMD auf
│ - AMD-Detektor               │  - Filtert Ergebnisse
│ - Buffer + Error-Counter     │  - publiziert auf /muri_picture_data
└──────────┬────────────────────┘
           │ Topic: /muri_picture_data
           │ Typ: muri_dev_interfaces/PictureData
           ▼
┌──────────────────────────────┐
│  Logik-Module                │  Steuert Roboter
│  (Drive, Turn, Init, Follow) │  basierend auf Position
└──────────────────────────────┘
```

---

## CameraReadOut Node: Detaillierter Ablauf

### Initialisierung (`__init__`)

```
1. Erstelle ROS2 Node mit Name 'camera_read_out'
2. Erstelle Publisher für '/muri_image_raw'
   ├─ Message-Typ: sensor_msgs/Image
   └─ Queue-Size: 10
3. Öffne /dev/video0 mit cv.VideoCapture(0)
4. Setze Buffer-Size auf 1 (verhindert Verzögerungen)
5. Starte Timer mit Frequenz 1/30 Sekunde (30 Hz)
6. Log: "CameraReadOut-Node gestartet"
```

### Timer-Callback: `timer_callback()` (alle 33.3ms)

```
EINGABE: Keine (Timer getriggert)

ABLAUF:
1. CvBridge-Instanz erstellen
2. read_camera() aufrufen
   ├─ Liest Frame von /dev/video0
   ├─ BGR → Grayscale konvertieren (cv.COLOR_BGR2GRAY)
   └─ Returns: numpy.ndarray oder None
3. cv2_to_imgmsg(bild, encoding='mono8') aufrufen
   ├─ OpenCV-Array → ROS Image-Message
   └─ mono8 = 8-Bit Grayscale, 1 Kanal
4. publish() aufrufen
   └─ Sende auf '/muri_image_raw'

FEHLERBEHANDLUNG:
─ AttributeError: None von read_camera()
  └─ Wird gefangen, Error geloggt
  └─ Publikation schlägt fehl → ImageProcessing bemerkt Fehler

RÜCKGABE: Keine (asynchron)
```

### Bild-Erfassung: `read_camera()`

```
EINGABE: Keine (Instanzvariable self.img)

ABLAUF:
1. success, frame = self.img.read()
2. if not success:
   ├─ Fehler geloggt: "Bild konnte nicht gelesen werden"
   ├─ return None
   └─ timer_callback() fängt AttributeError

3. frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
   ├─ BGR (OpenCV Standard) → Grayscale
   ├─ Shape: (480, 640, 3) → (480, 640)
   └─ dtype: uint8 (0-255)

4. return frame_gray

RÜCKGABE:
─ Success: numpy.ndarray mit dtype=uint8, shape=(height, width)
─ Fehler: None

FEHLER-SZENARIEN:
─ Kamera nicht vorhanden → success=False
─ USB-Kabel gezogen → success=False
─ Kamera-Treiber Fehler → Exception (wird in __init__ gefangen)
```

---

## ImageProcessing Node: Detaillierter Ablauf

### Initialisierung (`__init__`)

```
1. Erstelle ROS2 Node mit Name 'image_processing'
2. Erstelle CvBridge-Instanz
3. Initialisiere Instanzvariablen:
   ├─ Detektionsergebnisse: distance_in_milimeters, angle_in_rad, marker_id=9999
   ├─ Buffer: first_data=-1.0, second_data=-1.0, third_data=-1.0
   └─ Error-Tracking: error=False, error_counter=0
4. Erstelle AMD-Detektor: self.proc_AMD = AMD()
5. Erstelle Subscriber für '/muri_image_raw'
   ├─ Message-Typ: sensor_msgs/Image
   ├─ Callback: listener_callback()
   └─ Queue-Size: 10
6. Erstelle Publisher für '/muri_picture_data'
   ├─ Message-Typ: muri_dev_interfaces/PictureData
   └─ Queue-Size: 10
```

### Message-Empfang: `listener_callback(msg)` (30 Hz)

```
EINGABE: msg (sensor_msgs/Image)
─ encoding: 'mono8'
─ data: Pixelwerte (0-255)

ABLAUF:
1. try:
   ├─ cv_raw_image = bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
   └─ Returns: numpy.ndarray oder None
2. except Exception:
   └─ Fehler geloggt, aber fortgesetzt (Robustheit!)

3. pic_to_data(cv_raw_image)
   ├─ AMD-Detektion
   ├─ Einheiten-Konvertierung
   ├─ Filterung
   └─ Ergebnisse in Instanzvariablen speichern

4. Erstelle PictureData-Message:
   ├─ error: self.error (Boolean)
   ├─ angle_in_rad: self.angle_in_rad (float32)
   ├─ distance_in_meters: self.distance_in_meters_filtered (float32)
   └─ dominant_aruco_id: self.marker_id (int32)

5. publish() aufrufen
   └─ Sende auf '/muri_picture_data'

RÜCKGABE: Keine (asynchron)
```

### Detektion & Filterung: `pic_to_data(data_img)`

```
EINGABE: data_img (numpy.ndarray oder None)

ABLAUF:
1. Fehlerüberwachung:
   ├─ if data_img is None:
   │  ├─ error_counter += 1
   │  └─ Log: "Kein Frame erhalten!"
   │
   └─ if error_counter > 10:
      └─ error = True (permanenter Fehler signalisiert)

2. ArUco-Detektion:
   ├─ distance_in_milimeters, angle_in_rad, marker_id = AMD.aruco_detection(data_img)
   │
   └─ Returns bei Erfolg: (z_mm, y_rad, id)
      oder Fehler: (-1000.0, π, 9999)

3. Einheiten-Konvertierung:
   ├─ distance_in_meters_unfiltered = distance_in_milimeters / 1000
   └─ Umrechnung: mm → m (z.B. 500 mm → 0.5 m)

4. Distanz-Filterung:
   ├─ distance_in_meters_filtered = filter_distance()
   └─ Siehe: filter_distance() Ablauf

RÜCKGABE: Keine (Ergebnisse in Instanzvariablen)
```

### Distanz-Filter: `filter_distance()`

```
EINGABE: Keine (Instanzvariable self.distance_in_meters_unfiltered)

ABLAUF:
1. Buffer-Shift (FIFO: neuste nach alte):
   ├─ third_data = second_data  (ältester Wert)
   ├─ second_data = first_data  (mittlerer Wert)
   └─ first_data = distance_in_meters_unfiltered  (neuster Wert)

2. Finde neuesten gültigen Wert:
   ├─ for v in (first_data, second_data, third_data):
   │  ├─ if v != -1.0:
   │  │  └─ return v  ← RÜCKGABE: neuester gültiger Wert
   │  └─ weiter
   │
   └─ return -1.0  ← Alle drei Werte sind ungültig

BEISPIEL-SZENARIO:
─ Ungefilterter Stream: 0.5, -1.0, -1.0, 0.6, 0.65, ...
─ Nach Filter:           0.5, 0.5,  0.5,  0.6, 0.65, ...
  (Fehler wird überbrückt, dann langsam aktualisiert)

RÜCKGABE:
─ float: Neueste gültige Distanz [m]
─ oder -1.0 wenn alle Puffer leer

FEHLERTOLERANZ:
─ Kurzzeitige Fehler (1-2 Frames): Überbrückt
─ Permanente Fehler: Bleibt bei letzer gültigem Wert hängen
─ Alternative: error_counter erkennt Permanentfehler
```

---

## AMD: ArUco-Detektor - Detaillierter Ablauf

### Initialisierung

```
EINGABE: config.py mit folgenden Parametern:

MARKER_SIZES: {
    0: 175,    ← Standard-Zielmarker (175 mm)
    69: 75     ← Anderer Roboter (75 mm)
}

CAMERA_MATRIX_RAW: 3×3 Intrinsik-Matrix
[
  [649.9, 0.0, 320.8],    ← Brennweite (fx, fy), optischer Mittelpunkt (cx)
  [0.0, 657.6, 240.5],
  [0.0, 0.0, 1.0]
]

DISTANCE_COEFFICIENT: 5-Komponenten Verzerrungsvektor
[
  [0.0480],     ← k1 (radial)
  [-0.1371],    ← k2 (radial)
  [-0.0233],    ← p1 (tangential)
  [-0.0260],    ← p2 (tangential)
  [2.1354]      ← k3 (radial, höhere Ordnung)
]

ABLAUF:
1. Lade MARKER_SIZES in self.marker_sizes
2. Erstelle ArUco-Dictionary: DICT_5X5_1000
   └─ 5×5-Marker, bis zu 1000 verschiedene IDs
3. Erstelle ArucoDetector mit Standard-Parametern
4. Speichere Kamera-Matrix und Verzerrungskoeffizienten als numpy-Arrays (float32)

ERGEBNIS:
─ self.detector: Ready für detectMarkers()
─ self.camera_matrix: Ready für solvePnP()
─ self.dist_coeffs: Ready für solvePnP()
```

### Marker-Detektion: `aruco_detection(img)`

```
EINGABE: img (numpy.ndarray Grayscale, dtype=uint8)
─ Shape: (height, width), z.B. (480, 640)
─ KEIN RGB, sondern Grayscale (zum Sparen von Bandbreite)

ABLAUF:

═══ SCHRITT 1: Marker-Erkennung ═══
1. corners, ids, rejected = self.detector.detectMarkers(img)
   ├─ corners: Liste von Ecken-Koordinaten (2D-Pixel) pro erkannter Marker
   ├─ ids: Erkannte Marker-IDs (z.B. [0], [69], [0, 69])
   └─ rejected: Kandidaten, die nicht erkannt wurden

2. Fehlerfall 1:
   ├─ if ids is None or len(corners) == 0:
   └─ return (-1000.0, π, 9999)  ← Keine Marker erkannt

═══ SCHRITT 2: Marker-Priorität ═══
3. Suche nach ID 69 und ID 0:
   ├─ for i in range(len(ids)):
   │  ├─ if ids[i][0] == 69: index_69 = i
   │  └─ elif ids[i][0] == 0: index_0 = i
   │
   └─ index_to_use = index_69 if index_69 is not None else index_0

4. Fehlerfall 2:
   ├─ if index_to_use is None:
   │  ├─ (Keine ID 69 und keine ID 0 erkannt)
   └─ return (-1000.0, π, 9999)  ← Keine relevante ID

═══ SCHRITT 3: Marker-Größe ═══
5. Lade Marker-Größe:
   ├─ marker_id = ids[index_to_use][0]
   ├─ marker_size = self.marker_sizes.get(marker_id)
   │
   └─ Fehlerfall 3:
      ├─ if marker_size is None:
      │  ├─ (Marker-ID nicht in config konfiguriert)
      └─ return (-1000.0, π, 9999)

═══ SCHRITT 4: 3D-Objektpunkte ═══
6. Erstelle 3D-Koordinaten des Markers (Marker-Zentrum ist Ursprung):
   ├─ obj_points = [
   │  [-marker_size/2,  marker_size/2, 0],  ← Obere linke Ecke
   │  [ marker_size/2,  marker_size/2, 0],  ← Obere rechte Ecke
   │  [ marker_size/2, -marker_size/2, 0],  ← Untere rechte Ecke
   │  [-marker_size/2, -marker_size/2, 0]   ← Untere linke Ecke
   │  ]
   │
   └─ Liegt in XY-Ebene (Z=0), Zentrum bei (0,0,0)
      Beispiel (marker_size=100 mm):
      ├─ (-50, 50, 0) ← links oben
      ├─ (50, 50, 0)  ← rechts oben
      ├─ (50, -50, 0) ← rechts unten
      └─ (-50, -50, 0)← links unten

═══ SCHRITT 5: Pose-Estimation ═══
7. Berechne 3D-Position:
   ├─ success, rvec, tvec = cv.solvePnP(
   │  ├─ objectPoints: obj_points (3D im Marker-Koordinatensystem)
   │  ├─ imagePoints: corners[index_to_use][0] (2D Pixel im Bild)
   │  ├─ cameraMatrix: self.camera_matrix (3×3 Intrinsik)
   │  ├─ distCoeffs: self.dist_coeffs (5 Verzerrungskoeffizienten)
   │  └─ flags: cv.SOLVEPNP_IPPE_SQUARE
   │  )
   │
   └─ Returns:
      ├─ success: Boolean (Konvergenz erreicht?)
      ├─ rvec: Rotationsvektor (3×1, wird NICHT verwendet)
      └─ tvec: Translationsvektor (3×1, wird verwendet)
         ├─ tvec[0][0]: X-Position (horizontal, positiv=rechts)
         ├─ tvec[1][0]: Y-Position (vertikal, positiv=oben)
         └─ tvec[2][0]: Z-Position (Tiefe, positiv=weg von Kamera)

8. Fehlerfall 4:
   ├─ if not success:
   └─ return (-1000.0, π, 9999)

═══ SCHRITT 6: Winkelberechnung ═══
9. Berechne Yaw-Winkel:
   ├─ angle_rad = calculate_angle_to_marker()
   │  └─ Nutzt tvec[0][0] und tvec[2][0]
   └─ returns: float in [-π, π]

═══ ERFOLGREICHE RÜCKGABE ═══
10. return (tvec[2][0], angle_rad, marker_id)
    ├─ tvec[2][0]: Distanz zum Marker in Millimetern (Z-Koordinate)
    ├─ angle_rad: Horizontaler Yaw-Winkel in Radiant
    └─ marker_id: Erkannte Marker-ID (0 oder 69)

FEHLERFALL-ÜBERSICHT:
┌─────────────────────────────────┬──────────────────┐
│ Szenario                        │ Rückgabe         │
├─────────────────────────────────┼──────────────────┤
│ 1. Keine Marker erkannt         │ (-1000, π, 9999) │
│ 2. ID ≠ 0 und ≠ 69              │ (-1000, π, 9999) │
│ 3. Marker-ID nicht konfiguriert │ (-1000, π, 9999) │
│ 4. solvePnP schlägt fehl        │ (-1000, π, 9999) │
│ 5. Nur ID 69 erkannt            │ (z, θ, 69)       │
│ 6. Nur ID 0 erkannt             │ (z, θ, 0)        │
│ 7. Beide IDs erkannt            │ (z, θ, 69)       │ ← 69 bevorzugt
└─────────────────────────────────┴──────────────────┘

RÜCKGABE-TYP: tuple (float, float, int)
```

### Winkelberechnung: `calculate_angle_to_marker()`

```
EINGABE: Keine (Instanzvariable self.tvec gesetzt von aruco_detection())

GEOMETRIE:
─ Rechtwinkliges Dreieck im Kamera-Koordinatensystem:
                      Kamera (optischer Mittelpunkt)
                          *
                         /|
                        / | Z (Tiefe)
                       /  |
      X (Horizont)    /α  | (Ankathete)
      (Gegenkathete) /    |
                    /     |
                   /______|
                  Marker

ABLAUF:
1. distance = self.tvec[2][0]  ← Z-Achse: Distanz zum Marker
2. x_offset = self.tvec[0][0]  ← X-Achse: Horizontaler Versatz

3. angle_rad = math.atan2(x_offset, distance)
   ├─ atan2(Gegenkathete, Ankathete)
   └─ Berechne Winkel in [-π, π]

WINKELKONVENTION:
─ angle_rad > 0:  Marker rechts
                  └─ Roboter ist links vom Marker
                  └─ Muss nach rechts drehen
─ angle_rad < 0:  Marker links
                  └─ Roboter ist rechts vom Marker
                  └─ Muss nach links drehen
─ angle_rad ≈ 0:  Marker zentriert
                  └─ Roboter zeigt direkt auf Marker

WARUM ATAN2 STATT ATAN?
┌──────────────────────┬────────────────┬────────────────┐
│ Eigenschaft          │ atan()          │ atan2()        │
├──────────────────────┼────────────────┼────────────────┤
│ Wertebereich         │ [-π/2, π/2]    │ [-π, π]        │
│ Division by Zero     │ Problem @ x=0  │ Kein Problem   │
│ Quadrant-Info        │ Nein           │ Ja (beide Args)│
│ Vorzeichenbehandlung │ Manuell        │ Automatisch    │
│ Genauigkeit          │ Gut            │ Besser         │
└──────────────────────┴────────────────┴────────────────┘

RÜCKGABE: float in [-π, π] Radian
```

---

## ROS2 Message-Formate

### `/muri_image_raw` (CameraReadOut Publisher)

```
Type: sensor_msgs/Image

Felder:
├─ header.seq: uint32 (Sequenznummer)
├─ header.stamp.sec: uint32 (Sekunden seit Epoch)
├─ header.stamp.nsec: uint32 (Nanosekunden)
├─ header.frame_id: string (z.B. "camera")
├─ height: uint32 (480)
├─ width: uint32 (640)
├─ encoding: string ("mono8")
├─ is_bigendian: bool (false)
├─ step: uint32 (Bytes pro Zeile = 640)
└─ data: uint8[] (Pixelwerte, 480×640 = 307200 Bytes)

Bandbreite:
─ mono8: ~300 KB/Frame @ 30 Hz = ~9 MB/s
─ RGB: ~900 KB/Frame @ 30 Hz = ~27 MB/s (3× größer!)
```

### `/muri_picture_data` (ImageProcessing Publisher)

```
Type: muri_dev_interfaces/PictureData

Felder:
├─ header: std_msgs/Header (Timestamp)
├─ error: bool
│  └─ True: >10 konsekutive Frame-Fehler erkannt
│  └─ False: Normal
├─ angle_in_rad: float32
│  └─ Bereich: [-π, π]
│  └─ Positiv: Marker rechts
│  └─ Negativ: Marker links
├─ distance_in_meters: float32
│  └─ Gefilterte Distanz
│  └─ Bei Fehler: -1.0
└─ dominant_aruco_id: int32
   └─ 0: Standard-Zielmarker
   └─ 69: Anderer Roboter
   └─ 9999: Fehler

Publikations-Frequenz: 30 Hz (async zu Bildempfang)
```

---

## Troubleshooting & Debugging

### Problem: Keine Marker erkannt

**Ursachen & Lösungen**:

| Ursache | Test | Lösung |
|---|---|---|
| Kamera nicht angeschlossen | `ls /dev/video*` | USB-Kabel prüfen |
| Schlechte Lichtverhältnisse | Bild live prüfen | Lichter anmachen |
| Marker zu klein/weit weg | Berechne: Marker-Größe ÷ Distanz | Marker vergrößern oder näher kommen |
| Marker beschädigt | Visuelle Inspektion | Marker neu drucken |
| ArUco-Dictionary falsch | Code überprüfen | DICT_5X5_1000 verwenden |

### Problem: Marker erkannt, aber falsche Distanz

**Ursachen**:

| Ursache | Test | Lösung |
|---|---|---|
| Kamera nicht kalibriert | DISTANCE_COEFFICIENT prüfen | CharUco-Kalibrierung durchführen |
| Marker-Größe falsch in config | Physische Größe messen | config.py aktualisieren |
| solvePnP konvergiert nicht | Check success-Flag | IPPE_SQUARE Flag verwenden |

### Problem: Springende Distanzwerte

**Root Cause**: solvePnP rvec-Werte sind instabil
**Lösung**: Ist bereits implementiert → händische Winkelberechnung, Buffer-Filter

### Problem: Error-Flag wird gesetzt

**Bedeutung**: >10 konsekutive Frame-Fehler

**Überprüfen**:
```bash
# In ROS2 Terminal:
ros2 topic echo /muri_picture_data

# Prüfe: error: true oder false?
# Falls true: Kamera-Problem oder ImageProcessing-Fehler
```

---

## Performance & Optimierungen

### Rechenzeiten (ungefähr)

| Operation | Zeit |
|---|---|
| Kameraframe lesen | 33.3 ms (30 Hz) |
| BGR → Grayscale | 1-2 ms |
| detectMarkers() | 5-10 ms |
| solvePnP() | 2-3 ms |
| Filterung | <1 ms |
| ROS-Publikation | 1-2 ms |
| **Gesamt** | **~44-51 ms** (< 1 Frame) |

### Bandbreite

| Format | Pro Frame | @ 30 Hz |
|---|---|---|
| Grayscale (mono8) | 300 KB | 9 MB/s |
| RGB | 900 KB | 27 MB/s |
| **Einsparung** | **600 KB** | **18 MB/s** |

### Optimierungs-Potential

- ✓ Grayscale statt RGB: 66% Datenreduktion
- ✓ Buffer-Size=1: Keine Verzögerung durch Backlog
- ✓ Marker-Priorität vor solvePnP: ~40% Rechenzeit-Einsparung bei Dual-Marker
- ✓ Manuelle Winkelberechnung: Stabiler als rvec-basiert
- ⚠ Serielles (nicht paralleles) Processing: Könnte in Zukunft parallelisiert werden

