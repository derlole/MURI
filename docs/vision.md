# Vision-System: Implementierungs-Details

<!-- Written, maintained and owned by Linus Braun (MURI DEVELOPMENT TEAM) -->

## Übersicht

Diese Dokumentation beschreibt die detaillierte Implementierung der Vision-Pipeline ohne Design-Entscheidungen. Für Design-Begründungen siehe [vision_entscheidungen_technical.md](vision_entscheidungen_technical.md).

---

## ArUco Marker Detection (AMD) - Detaillierte Implementierung

### Konstruktor `__init__()`

#### Geladene Konfigurationen

**Aus `config.py`**:
- `MARKER_SIZES`: Dictionary mit Marker-IDs → physische Größen (mm)
  - Beispiel: `{0: 175, 69: 75}`
- `CAMERA_MATRIX_RAW`: 3×3 Kamera-Intrinsik-Matrix (Brennweite, optischer Mittelpunkt)
- `DISTANCE_COEFFICIENT`: 5er-Array mit Verzerrungskoeffizienten (k1, k2, p1, p2, k3)

#### Initialisierte Komponenten

| Komponente | Typ | Zweck |
|---|---|---|
| `self.detector` | `cv.ArucoDetector` | Marker-Erkennung im Bild |
| `self.aruco_dict` | `DICT_5X5_1000` | 5×5-Marker, bis 1000 IDs |
| `self.camera_matrix` | `numpy.ndarray` (3×3) | Intrinsische Kameraparameter |
| `self.dist_coeffs` | `numpy.ndarray` (5,) | Verzerrungskoeffizienten |
| `self.marker_sizes` | `dict` | Größen-Lookup nach Marker-ID |

---

### Methode `aruco_detection(img)`

#### Eingabe
- **`img`** (numpy.ndarray): Grayscale-Bild (single-channel, dtype=uint8)
  - Größe: Standard (640x480)
  - Format: Muss Grayscale sein (externe Konvertierung!)

#### Detektions-Schritte

**1. Marker-Erkennung**
```python
corners, ids, _ = self.detector.detectMarkers(frame_gray)
```
- **corners**: Liste mit Konturen aller erkannten Marker
- **ids**: Array mit Marker-IDs (oder None wenn keine)
- **_**: Wird ignoriert

**2. Marker-Priorisierung**
```python
index_69 = None
index_0 = None

for i in range(len(ids)):
    if ids[i][0] == 69:
        index_69 = i
    elif ids[i][0] == 0:
        index_0 = i

index_to_use = index_69 if index_69 is not None else index_0
```

**Prioritäts-Logik**:
- Wenn Marker 69 existiert → verwende Marker 69
- Sonst wenn Marker 0 existiert → verwende Marker 0
- Sonst → Fehlerwert zurückgeben

**3. Marker-Größe laden**
```python
marker_id = ids[index_to_use][0]
marker_size = self.marker_sizes.get(marker_id)
```

Falls `marker_size is None` → Fehler zurückgeben (ID nicht konfiguriert)

**4. 3D-Objektpunkte definieren**
```python
obj_points = np.array([
    [-marker_size / 2,  marker_size / 2, 0],   # Top-left
    [ marker_size / 2,  marker_size / 2, 0],   # Top-right
    [ marker_size / 2, -marker_size / 2, 0],   # Bottom-right
    [-marker_size / 2, -marker_size / 2, 0]    # Bottom-left
], dtype=np.float32)
```

**Koordinaten-System**:
- Ursprung: Marker-Zentrum
- XY-Ebene: Marker-Oberfläche
- Z-Achse: Senkrecht zur Ebene (immer 0)
- Einheit: Millimeter

**5. Pose-Estimation mit solvePnP**
```python
success, _, self.tvec = cv.solvePnP(
    obj_points,
    corners[index_to_use][0],
    self.camera_matrix,
    self.dist_coeffs,
    flags=cv.SOLVEPNP_IPPE_SQUARE
)
```

**Eingaben**:
- `obj_points`: 3D-Punkte (4 Ecken des Markers) in mm
- `corners[index_to_use][0]`: 2D-Bild-Koordinaten der Ecken (Pixel)
- `camera_matrix`: Intrinsische Kameraparameter
- `dist_coeffs`: Linsen-Verzerrungskoeffizienten
- `flags=SOLVEPNP_IPPE_SQUARE`: Algorithmus (siehe [vision_entscheidungen_technical.md](vision_entscheidungen_technical.md))

**Ausgabe**:
- `self.tvec`: Translationsvektor [x, y, z] vom Kamera-Koordinatensystem zum Marker
  - `tvec[0][0]`: X-Position (horizontal, mm)
  - `tvec[1][0]`: Y-Position (vertikal, mm)
  - `tvec[2][0]`: Z-Position (Tiefe/Distanz, mm)

**6. Winkel berechnen**
```python
angle_rad = self.calculate_angle_to_marker()
```

Siehe Methode unten.

#### Rückgabewerte

**Bei Erfolg**: `(z_distanz_mm, y_winkel_rad, marker_id)`
```python
return self.tvec[2][0], angle_rad, marker_id
```

**Bei Fehler**: `(-1000.0, math.pi, 9999)`
- **-1000.0**: Ungültige Distanz (negativ = unmöglich)
- **math.pi**: Ungültiger Winkel (180° = physikalisch nicht erreichbar)
- **9999**: Fehler-ID (nicht im verwendeten Dictionary)

#### Fehlerszenarien

| Szenario | Bedingung | Rückgabe |
|---|---|---|
| Keine Marker | `ids is None \| len(corners)==0` | (-1000.0, π, 9999) |
| Falsche ID | Weder 69 noch 0 erkannt | (-1000.0, π, 9999) |
| Nicht konfiguriert | `marker_size is None` | (-1000.0, π, 9999) |
| solvePnP-Fehler | `success == False` | (-1000.0, π, 9999) |

---

### Methode `calculate_angle_to_marker()`

#### Mathematische Grundlage

**Rechtwinkliges Dreieck im Kamera-Koordinatensystem**:

```
            Kamera
             *
            /|
           / |
          /  | tvec[2] (Z-Distanz)
      tvec[0]|
     (X-Vers)|
      /      |
     /       |
    ---------+
    Marker
```

**Trigonometrie**:
- **Gegenkathete**: `x_offset = tvec[0][0]`
- **Ankathete**: `distance = tvec[2][0]`
- **Winkel α**: `angle = atan2(x_offset, distance)`

#### Berechnung

```python
distance = self.tvec[2][0]
x_offset = self.tvec[0][0]
angle_rad = math.atan2(x_offset, distance)
```

**Warum `atan2` statt `atan`**:
- ✓ Vollständiger Wertebereich: [-π, π] (alle Quadranten)
- ✓ Division-by-Zero-Sicherheit
- ✓ Vorzeichenerhaltung automatisch
- ✓ Genauere Berechnung mit beiden Komponenten

#### Vorzeichenkonvention

| Winkel | Marker-Position | Aktion |
|---|---|---|
| **> 0** | Rechts von Bildmitte | Roboter dreht **rechts** |
| **< 0** | Links von Bildmitte | Roboter dreht **links** |
| **≈ 0** | Zentriert | Keine Drehung |

#### Rückgabewert

- **Typ**: `float` (Radiant)
- **Bereich**: [-π, π]
- **Einheit**: Radiant (nicht Grad!)

---

## CameraReadOut Node - Detaillierte Implementierung

### ROS2-Komponenten

| Komponente | Topic/Device | Konfiguration |
|---|---|---|
| **Publisher** | `/muri_image_raw` | `sensor_msgs/Image`, Queue=10 |
| **Timer** | - | 1/30 Sekunden (30 Hz) |
| **Kamera** | `/dev/video0` | OpenCV VideoCapture(0) |

### Initialisierung

```python
self.img = cv.VideoCapture(path_camera)  # /dev/video0
self.img.set(cv.CAP_PROP_BUFFERSIZE, 1)  # Minimaler Buffer
self.data = self.create_timer(1/30, self.timer_callback)
```

**Buffer-Size = 1**:
- Verhindert Zwischenspeicherung mehrerer Frames
- Gewährleistet immer das aktuellste Bild
- Reduziert Latenz

### Ablauf `timer_callback()`

**Alle 1/30 Sekunden**:

1. **CvBridge erstellen** (für OpenCV ↔ ROS-Konvertierung)
2. **Frame erfassen** via `read_camera()`
3. **In Image-Message konvertieren** mit `cv2_to_imgmsg(encoding='mono8')`
4. **Auf `/muri_image_raw` publishen**

### Ablauf `read_camera()`

1. **Frame erfassen**
   ```python
   success, frame = self.img.read()
   ```
   - `success`: Boolean (True = erfolgreich)
   - `frame`: BGR-Bild (numpy.ndarray) oder None

2. **Fehlerbehandlung**
   ```python
   if not success:
       self.get_logger().error('Bild konnte nicht gelesen werden')
       return None
   ```

3. **BGR → Grayscale konvertieren**
   ```python
   frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
   ```
   - **BGR2GRAY**: OpenCV's BGR-Format zu Grayscale
   - **Nicht RGB2GRAY**: RGB ist OpenCV-intern nicht das native Format!

4. **Rückgabewert**
   - ✓ `numpy.ndarray`: Shape (height, width), dtype=uint8
   - ✗ `None`: Bei Lesefehler → führt zu AttributeError im timer_callback()

---

## ImageProcessing Node - Detaillierte Implementierung

### ROS2-Komponenten

| Komponente | Topic | Typ | Konfiguration |
|---|---|---|---|
| **Subscriber** | `/muri_image_raw` | `sensor_msgs/Image` | Queue=10 |
| **Publisher** | `/muri_picture_data` | `PictureData` | Queue=10 |

### Initialisierung

**Instanzvariablen**:

| Variable | Typ | Init-Wert | Zweck |
|---|---|---|---|
| `distance_in_milimeters` | float | None | Roh-Distanz von AMD (mm) |
| `distance_in_meters_unfiltered` | float | None | Konvertiert (m) |
| `distance_in_meters_filtered` | float | None | Nach Buffer-Filter (m) |
| `angle_in_rad` | float | None | Yaw-Winkel zum Marker |
| `marker_id` | int | 9999 | Marker-ID zu den Werten |
| `error` | bool | False | Wird nach 10 konsekutiven Fehlern auf TRUE gesetzt |
| `error_counter` | int | 0 | Zählt die Anzahl Frame-Verluste |
| `first_data` | float | -1.0 | Neuester Puffer-Wert |
| `second_data` | float | -1.0 | Mittlerer Puffer-Wert |
| `third_data` | float | -1.0 | Ältester Puffer-Wert |
| `proc_AMD` | AMD | Instanz | ArUco-Detektor |

**ROS2-Komponenten**:
```python
self.bridge = CvBridge()  # Image-Konvertierung
self.subscription = create_subscription(Image, '/muri_image_raw', ...)
self.publisher = create_publisher(PictureData, '/muri_picture_data', ...)
```

### Ablauf `listener_callback(msg)`

**Eingabe**: `sensor_msgs/Image` (Grayscale, mono8)

1. **Bildkonvertierung**
   ```python
   cv_raw_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
   ```
   - ROS Image-Message → OpenCV numpy.ndarray

2. **Detektion und Filterung**
   ```python
   self.pic_to_data(cv_raw_image)
   ```

3. **PictureData-Message erstellen**
   ```python
   pub_pic_data = PictureData()
   pub_pic_data.error = self.error
   pub_pic_data.angle_in_rad = float(self.angle_in_rad)
   pub_pic_data.distance_in_meters = float(self.distance_in_meters_filtered)
   pub_pic_data.dominant_aruco_id = int(self.marker_id)
   ```

4. **Publishen**
   ```python
   self.publisher.publish(pub_pic_data)
   ```

**Wichtig**: Gefilterte Distanz wird publiziert (nicht ungefiltert)!

### Ablauf `pic_to_data(data_img)`

**Eingabe**: `numpy.ndarray` oder `None`

1. **Fehlerüberwachung**
   ```python
   if data_img is None:
       self.error_counter += 1
   ```

2. **Error-Flag setzen**
   ```python
   if self.error_counter > 10:
       self.error = True
   ```
   - **Schwellwert 10**: ~333 ms @ 30 Hz
   - **Kein Reset**: Counter wird nie zurückgesetzt (permanente Fehler)

3. **ArUco-Detektion**
   ```python
   self.distance_in_milimeters, self.angle_in_rad, self.marker_id = \
       self.proc_AMD.aruco_detection(data_img)
   ```

4. **Einheitenkonvertierung**
   ```python
   self.distance_in_meters_unfiltered = self.distance_in_milimeters / 1000
   ```

5. **Filterung**
   ```python
   self.distance_in_meters_filtered = self.filter_distance()
   ```

### Ablauf `filter_distance()`

**Last-Valid-Value-Filter mit 3-Wert-Buffer**:

1. **Buffer-Shift**
   ```python
   self.third_data  = self.second_data
   self.second_data = self.first_data
   self.first_data  = self.distance_in_meters_unfiltered
   ```

2. **Neuesten gültigen Wert finden**
   ```python
   for v in (self.first_data, self.second_data, self.third_data):
       if v != -1.0:
           return v
   return -1.0
   ```

**Logik**:
- Iteriert von neu nach alt
- Gibt ersten Wert ≠ -1.0 zurück
- Falls alle -1.0 (ungültig) → gibt -1.0 zurück

---

## PictureData Message-Definition

```python
std_msgs/Header header
bool error
float32 angle_in_rad
float32 distance_in_meters
int32 dominant_aruco_id
```

| Feld | Typ | Wertebereich | Bedeutung |
|---|---|---|---|
| `error` | bool | True/False | True nach >10 Frame-Verlusten |
| `angle_in_rad` | float | [-π, π] | Yaw-Winkel zum Marker |
| `distance_in_meters` | float | [0 --> ...]* | Gefilterte Distanz |
| `dominant_aruco_id` | int32 | 0, 69, 9999 | Erkannte Marker-ID |


---



## Abhängigkeiten und Imports

### Externe Bibliotheken

```python
import cv2 as cv              # OpenCV Hauptbibliothek
import cv2.aruco as aruco     # ArUco-Modul
import numpy as np            # Array-Operationen
import math                   # atan2, pi
import rclpy                  # ROS2 Python API
from rclpy.node import Node   # ROS2 Node-Klasse
```

### ROS2-Interfaces

```python
from sensor_msgs.msg import Image     # Kamera-Bilder
from muri_dev_interfaces.msg import PictureData  # Detektionsergebnisse
from cv_bridge import CvBridge        # OpenCV ↔ ROS Bridge
```

### Konfiguration

```python
import config  # Externe Datei mit:
               # - MARKER_SIZES
               # - CAMERA_MATRIX_RAW
               # - DISTANCE_COEFFICIENT
```

---

