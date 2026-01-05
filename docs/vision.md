# ArUco Marker Detection Dokumentation

## Übersicht

Die `AMD` (ArUco Marker Detection) Klasse kapselt die OpenCV ArUco-Erkennungslogik zur Detektion von ArUco-Markern und Berechnung ihrer räumlichen Position relativ zur Roboter-Kamera. Die Klasse wird extern aufgerufen, um pro Detektionsaufruf folgende Informationen zu liefern:

- **Marker-ID**: Identifikation des erkannten Markers
- **Distanz**: Entfernung vom Roboter zum Marker (Z-Koordinate)
- **Winkel**: Horizontaler Winkelversatz vom Roboter zum Marker (Y-Rotation)

Die Klasse unterstützt mehrere Marker mit unterschiedlichen physischen Größen und verwendet vorkalibrierte Kameraparameter. Weitere Marker können in der config.py eingetragen werden.

## Systemintegration

Die AMD-Klasse wird in einem dedizierten ROS2-Node aufgerufen, der die detektierten Aruco-Daten (Marker-ID, Distanz, Winkel) über Topics publisht. Diese publizierten Daten werden dann von anderen Ros2 teilen verwendet.

Die Klasse selbst ist unabhängig von ROS2 und dient ausschließlich der Bildverarbeitung, Positionsberechnung und ID-Erkennung.

## Klasseninitialisierung

### Konstruktor `__init__()`

**Zweck**: Initialisiert den ArUco-Detektor mit vorkonfigurierten Kameraparametern und Marker-Größen.

```python
def init(self):
    # Marker-Größen aus der Konfiguration laden
    self.marker_sizes = config.MARKER_SIZES
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
    aruco_params = aruco.DetectorParameters()
    self.detector = aruco.ArucoDetector(aruco_dict, aruco_params)
    self.aruco_dict = aruco_dict
    self.camera_matrix = np.array(config.CAMERA_MATRIX_RAW, dtype=np.float32)
    self.dist_coeffs = np.array(config.DISTANCE_COEFFICIENT, dtype=np.float32)
```

### Initialisierte Komponenten

**ArUco-Dictionary (`aruco.DICT_5X5_1000`)**:
- Verwendet 5×5-Marker für optimalen Kompromiss zwischen Erkennungsrobustheit und ID-Vielfalt
- Bessere Erkennbarkeit aus größerer Entfernung und bei niedrigerer Auflösung als 6×6-Marker

**Marker-Größen (`self.marker_sizes`)**:
- Dictionary mit Marker-ID als Schlüssel und physischer Größe als Wert
- Unterstützt mehrere Marker mit unterschiedlichen Dimensionen

- Beispiel: `{0: 50.0, 69: 30.0}` (id: size in mm)

**Kamera-Kalibrierungsparameter**:

`self.camera_matrix`:
- 3×3-Matrix mit intrinsischen Kameraparametern (Brennweite, optischer Mittelpunkt)
- Wurde durch separate Kamera-Kalibrierung für die verwendete Kamera ermittelt

`self.dist_coeffs`:
- Verzerrungskoeffizienten der Kamera (radiale und tangentiale Verzerrung)

- Wurde durch separate Kamera-Kalibrierung ermittelt

**Detector-Objekt (`self.detector`)**:
- OpenCV `ArucoDetector`-Instanz mit Standard-Detektionsparametern
- Verwendet für die eigentliche Marker-Erkennung im Bild

**ArUco-Dictionary-Referenz (`self.aruco_dict`)**:
- Gespeicherte Referenz für einfacheres Handling in anderen Methoden
- Ermöglicht Zugriff auf Dictionary-Informationen ohne Neuinitialisierung

### Konfigurationsabhängigkeiten

Die Klasse benötigt folgende Parameter aus dem `config`-Modul:
- `MARKER_SIZES`: Dictionary mit Marker-IDs und ihren physischen Größen [mm]
- `CAMERA_MATRIX_RAW`: 3×3-Kamera-Intrinsik-Matrix
- `DISTANCE_COEFFICIENT`: Linsenverzerrungskoeffizienten

---

## Methode `aruco_detection()`

**Zweck**: Detektiert ArUco-Marker im Bild und gibt Distanz, Winkel und Marker-ID zurück.

### Methodensignatur

```python
def aruco_detection(self, img):


```

### Eingabeparameter

**`img` (numpy.ndarray)**:
- Erwartet ein **Graustufenbild** (single-channel)
- Grayscale-Format wird aus Übertragungsoptimierungsgründen verwendet (reduzierte Bandbreite, höhere Geschwindigkeit, mehr Bilder)
- Kein RGB-zu-Grayscale-Konvertierung innerhalb der Methode notwendig
- Die Konvertierung erfolgt extern vor dem Methodenaufruf

### Marker-Prioritätslogik

**Unterstützte Marker-IDs**:
- **ID 69**: Roboter-Marker (höchste Priorität)
- **ID 0**: Standard-Zielmarker (niedrigere Priorität)

**Priorisierungsregel**:
```python
index_69 = None
index_0 = None

for i in range(len(ids)):
    if ids[i] == 69:
    index_69 = i
    elif ids[i] == 0:
    index_0 = i

index_to_use = index_69 if index_69 is not None else index_0

```

**Entscheidungslogik**:
- Wenn **nur Marker 69** erkannt wird → Marker 69 wird weitergegeben
- Wenn **nur Marker 0** erkannt wird → Marker 0 wird weitergegeben
- Wenn **beide Marker gleichzeitig** erkannt werden → Marker 69 wird priorisiert, Marker 0 wird ignoriert
- Wenn **kein relevanter Marker** erkannt wird → Fehlerwert wird zurückgegeben


### Fehlerbehandlung

**Fehlerwerte bei keiner Detektion**: `(-1000.0, math.pi, 9999)`

Diese Werte signalisieren verschiedenen Logik-Modulen einen Detektionsfehler:
- **z_pos = -1000.0**: Ungültige Distanz (offensichtlich außerhalb des physikalischen Bereichs)
- **y_rot = math.pi**: Ungültiger Winkel (π rad = 180°),da er bei 180° physikalisch keinen Aruco Marker erkennen kann.
- **marker_id = 9999**: Fehler-ID (in Steuerungslogik als Abbruch-Signal verwendet), Diese ID ist keine existierende im verwendeten DIctionary

**Fehlerszenarien**:
1. Kein Marker im Bild erkannt (`ids is None`)
2. Marker erkannt, aber weder ID 69 noch ID 0 (`index_to_use is None`)
3. Marker-ID nicht in `config.MARKER_SIZES` konfiguriert
4. `cv.solvePnP()` schlägt fehl (`success == False`)

### Marker-Größen-Handling

**Dynamische Marker-Größen**:
```python
marker_id = ids[index_to_use]
marker_size = self.marker_sizes.get(marker_id)
```

Wenn die Marker-ID nicht konfiguriert ist, Fehler zurückgeben

```python
if marker_size is None:
return -1000.0, math.pi, 9999
```

**Zweck**:
- Unterstützt Marker mit unterschiedlichen physischen Abmessungen
- Ermöglicht präzise Distanzberechnung durch Verwendung der tatsächlichen Marker-Größe
- Flexibilität bei der Marker-Platzierung auf verschiedenen Robotern (Platzprobleme)

**3D-Objektpunkte**:

```python
obj_points = np.array([[-marker_size / 2, marker_size / 2, 0], # Obere linke Ecke
                        [ marker_size / 2, marker_size / 2, 0], # Obere rechte Ecke
                        [ marker_size / 2, -marker_size / 2, 0], # Untere rechte Ecke
                        [-marker_size / 2, -marker_size / 2, 0]], # Untere linke Ecke
dtype=np.float32)
```

- Definiert die 3D-Koordinaten der vier Marker-Ecken im Marker-Koordinatensystem
- Marker liegt in der XY-Ebene (Z=0)
- Ursprung im Marker-Zentrum

### Pose-Estimation mit solvePnP

**Methode**: `cv.solvePnP()` mit `SOLVEPNP_IPPE_SQUARE`-Flag

```python
success, _, self.tvec = cv.solvePnP(obj_points,
                                    corners[index_to_use],
                                    self.camera_matrix,
                                    self.dist_coeffs,
                                    flags=cv.SOLVEPNP_IPPE_SQUARE)

```

**Eingaben**:
- `obj_points`: 3D-Koordinaten der Marker-Ecken in Millimetern
- `corners[index_to_use][0]`: Detektierte 2D-Bildkoordinaten der Marker-Ecken

**Ausgabe**:
- `self.tvec`: Translationsvektor [x, y, z] vom Kamera-Koordinatensystem zum Marker
  - `tvec[0][0]`: X-Position (horizontal, positiv = rechts)
  - `tvec[1][0]`: Y-Position (vertikal, positiv = oben)
  - `tvec[2][0]`: Z-Position (Tiefe/Distanz, positiv = vom Kamera weg)

**SOLVEPNP_IPPE_SQUARE**:
- Optimiert für quadratische planare Objekte (ArUco-Marker)
- Infinitesimal Plane-based Pose Estimation (IPPE)
- Höhere Genauigkeit und Robustheit als Standard-Algorithmen

### Rückgabewerte

**Bei erfolgreicher Detektion**:

```python
if success:
    angle_rad = self.calculate_angle_to_marker()
    marker_id = ids[index_to_use]
    return self.tvec, angle_rad, marker_id
```

- **z_pos** (`self.tvec[2][0]`): Distanz zum Marker in Millimetern
- **y_rot** (`angle_rad`): Winkel zum Marker in Radiant (berechnet mit `calculate_angle_to_marker()`)
- **marker_id**: Erkannte Marker-ID (69 oder 0)

**Bei Fehler**:

```python
return -1000.0, math.pi, 9999
```

### Ablaufdiagramm

---

Eingabe: Graustufenbild
↓
detectMarkers() → corners, ids
↓
ids vorhanden?
├─ Nein → Rückgabe: (-1000.0, π, 9999)
└─ Ja → Suche nach ID 69 und ID 0
↓
Priorisiere ID 69, sonst ID 0
↓
Marker-ID gefunden?
├─ Nein → Rückgabe: (-1000.0, π, 9999)
└─ Ja → Lade marker_size aus Config
↓
marker_size konfiguriert?
├─ Nein → Rückgabe: (-1000.0, π, 9999)
└─ Ja → Erstelle obj_points
↓
solvePnP() → success, tvec
↓
success?
├─ Nein → Rückgabe: (-1000.0, π, 9999)
└─ Ja → Berechne angle_rad
↓
Rückgabe: (tvec, angle_rad, marker_id)

---

## Methode `calculate_angle_to_marker()`

**Zweck**: Berechnet den horizontalen Yaw-Winkel vom Roboter zum erkannten Marker mithilfe der 3D-Positionsdaten.

### Methodensignatur

```python
def calculate_angle_to_marker(self):
```

### Geometrische Grundlage

**Rechtwinkliges Dreieck**:

Die Methode konstruiert ein rechtwinkliges Dreieck im Kamera-Koordinatensystem:

                     Kamera
                      *
                     /|
                    /α|  
         x_offset  /  | distance (Z-Achse)
    (Gegenkathete)/   | (Ankathete) 
                 /    |  
                /     |  
    Marker     --------  


**Koordinatensystem**:
- **Z-Achse** (distance): Geradeaus von der Kamera, optische Achse (Bildmitte)
- **X-Achse** (x_offset): Horizontal, positiv = rechts von der Kamera
- **Y-Achse**: Vertikal (nicht verwendet für Yaw-Berechnung)

**Trigonometrische Beziehung**:

- **Gegenkathete**: `x_offset` (horizontaler Abstand vom Bildmittelpunkt)
- **Ankathete**: `distance` (Entfernung entlang der optischen Achse)
- **Winkel α**: Yaw-Winkel vom Roboter zum Marker
### Berechnungslogik

```python
distance = self.tvec # Z-Koordinate: Tiefe/Distanz zum Marker​
x_offset = self.tvec # X-Koordinate: Horizontaler Versatz

angle_rad = math.atan2(x_offset, distance)
```

### Verwendung von `atan2` statt `atan`

**Vorteile von `math.atan2(y, x)`**:

1. **Vollständiger Wertebereich**: Gibt Winkel im Bereich \([-\pi, \pi]\) zurück (alle vier Quadranten)
2. **Division-by-Zero-Sicherheit**: Kein Problem wenn `distance = 0`
3. **Vorzeichenerhaltung**: Berücksichtigt automatisch das Vorzeichen von `x_offset` und `distance`
4. **Genauigkeit**: Präzisere Berechnung durch Nutzung beider Komponenten

### Vorzeichenkonvention

**Positiver Winkel** (`angle_rad > 0`):
- Marker befindet sich **rechts** von der Bildmitte
- Roboter ist **links versetzt** vom Marker
- Roboter muss **nach rechts drehen**, um auf den Marker auszurichten

**Negativer Winkel** (`angle_rad < 0`):
- Marker befindet sich **links** von der Bildmitte
- Roboter ist **rechts versetzt** vom Marker
- Roboter muss **nach links drehen**, um auf den Marker auszurichten

**Null-Winkel** (`angle_rad ≈ 0`):
- Marker liegt in der Bildmitte
- Roboter ist bereits korrekt ausgerichtet
- Keine Drehung erforderlich

### Integration mit Steuerlogik

Der von `calculate_angle_to_marker()` zurückgegebene Winkel wird direkt als Winkelfehler in den P-Reglern der Logik-Module verwendet:

- Positiver Winkelfehler → Positive Winkelgeschwindigkeit → Rechtsdrehung
- Negativer Winkelfehler → Negative Winkelgeschwindigkeit → Linksdrehung


---

## Abhängigkeiten

### Externe Bibliotheken
- `cv2` (OpenCV): Bildverarbeitung und ArUco-Detektion
- `cv2.aruco`: ArUco-Marker-Modul
- `numpy`: Array-Operationen für Kameraparameter
- `math`: Mathematische Funktionen (atan2, pi)

### Konfigurationsmodul
```python
import config
```

---------------------------------------------------------------------------------------

# Camera Read Out Dokumentation

## Übersicht

Der `CameraReadOut`-Node erfasst kontinuierlich Bilder von der Roboter-Kamera und publiziert sie als Grayscale-Images auf `/muri_image_raw`. Der Node dient als erste Stufe der Vision-Pipeline und versorgt nachgelagerte Nodes (z.B. ArUco-Detektion, Bildverarbeitung) mit Kamerabildern.

**Hauptfunktionen**:
- Kamera-Stream von `/dev/video0` mit 30 Hz
- RGB zu Grayscale-Konvertierung (Bandbreitenoptimierung)
- Publikation auf `/muri_image_raw` (sensor_msgs/Image)

---

## Klasse `CameraReadOut`

### Konstruktor `__init__()`

```python
def init(self):
    super().init('camera_read_out')
    self.publisher = self.create_publisher(Image, '/muri_image_raw', 10)
    timer_time = 1/30 # 30 Hz
    path_camera = 0 # /dev/video0

    try:
        self.img = cv.VideoCapture(path_camera)
    except Exception as e:
        self.get_logger().error(f'Fehler beim Initialisieren der Kamera: {str(e)}')
        raise e

    self.img.set(cv.CAP_PROP_BUFFERSIZE, 1)
    self.data = self.create_timer(timer_time, self.timer_callback)
    self.get_logger().info('CameraReadOut-Node gestartet')
```


### Konfiguration

**Timer-Frequenz (30 Hz)**:
- Maximale Bildrate der verwendeten Kamera
- Optimale Balance zwischen Aktualität und Performance

**Kamera-Device (`path_camera = 0`)**:
- Entspricht `/dev/video0` (erste Kamera am System)
- Die Roboter-Kamera ist an diesem Device angeschlossen

**Buffer-Size = 1**:
- Minimalsterr OpenCV-Frame-Buffer
- Verhindert veraltete/verzögerte Bilder
- Notwendig für fehlerfreie ArUco-Detektion

**Grayscale-Publikation**:
- Encoding: `mono8` (1 Kanal statt 3)
- 66% weniger Daten als RGB
- Höhere Übertragungsrate bei gleicher Bandbreite
- ArUco-Detektion benötigt nur Grayscale

### Publisher

- **Topic**: `/muri_image_raw`
- **Message-Typ**: `sensor_msgs/Image`
- **Queue-Size**: 10

---

## Methoden

### `timer_callback()`

**Zweck**: Timer-Callback für periodische Bilderfassung und Publikation (30 Hz).

```python
def timer_callback(self):
    try:
        bridge = CvBridge()
        msg = bridge.cv2_to_imgmsg(self.read_camera(), encoding='mono8')
        self.publisher.publish(msg)
    except AttributeError as e:
        self.get_logger().error("Falscher Wert wurde versucht über die Bridge zu senden")
```


**Ablauf**:
1. `CvBridge()`-Instanz erstellen für OpenCV-zu-ROS-Konvertierung
2. Bild von Kamera lesen via `read_camera()`
3. OpenCV-Array zu ROS2-Image-Message konvertieren (encoding: `mono8`)
4. Message auf `/muri_image_raw` publishen

**CvBridge-Instanziierung**:
- Wird in jedem Callback neu erstellt
- Alternative wäre Instanzvariable in `__init__`

**Fehlerbehandlung**:
- `AttributeError`: Tritt auf wenn falscher/ungültiger Wert an Bridge übergeben wird
- Beispiel: `None` statt numpy-Array führt zu ROS2-internem Fehler
- ROS2 erlaubt keine `None`-Werte in Image-Messages

**Auskommentiertes Logging**:
- `#self.get_logger().info('Bild wird verschickt...')` 
- Kann für schnelles Debugging aktiviert werden
- Bei 30 Hz würde permanentes Logging die Console fluten

---

### `read_camera()`

**Zweck**: Erfasst ein einzelnes Frame von der Kamera und konvertiert zu Grayscale.

```python
def read_camera(self):
    success, frame = self.img.read()
    if not success:
        self.get_logger().error('Bild konnte nicht gelesen werden')
        return None
    frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    return frame_gray
```

**Ablauf**:
1. Frame von `cv.VideoCapture` lesen
2. Bei Fehler: Error-Log und `None` zurückgeben
3. BGR zu Grayscale konvertieren
4. Grayscale-Frame zurückgeben

**BGR zu Grayscale**:
- OpenCV verwendet intern **BGR** statt RGB als Farbformat
- `cv.COLOR_BGR2GRAY` konvertiert korrekt von OpenCV-Format zu Grayscale
- Verwendung von `cv.COLOR_RGB2GRAY` würde zu falschen Grauwerten führen

**Fehlerbehandlung**:
- `success = False`: Kamera nicht verfügbar, Verbindung verloren, oder Device-Fehler
- Rückgabe von `None` führt zu `AttributeError` in `timer_callback()`
- ROS2 `cv2_to_imgmsg()` akzeptiert keine `None`-Werte

**Return-Typ**:
- `numpy.ndarray` (Grayscale, single-channel)
- Shape: `(height, width)` z.B. `(480, 640)`
- dtype: `uint8` (0-255)

---

## Abhängigkeiten

```python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
import rclpy
from rclpy.node import Node
```


**ROS2-Pakete**:
- `rclpy`: ROS2 Python-Client-Library
- `sensor_msgs`: Image-Message-Definition
- `cv_bridge`: Konvertierung zwischen OpenCV und ROS2-Messages

**Externe Bibliotheken**:
- `cv2` (OpenCV): Kamera-Zugriff und Bildverarbeitung
- `numpy`: Array-Operationen (indirekt über OpenCV)


## Main-Funktion
Standard ROS2-Node-Lifecycle:

```python
def main(args=None):
    rclpy.init(args=args)
    camera_read_out = CameraReadOut()
    rclpy.spin(camera_read_out)
    camera_read_out.destroy_node()
    rclpy.shutdown()
```

---------------------------------------------------------------------------------------

# Image Data Processing Dokumentation

## Übersicht

Der `ImageProcessing`-Node ist die zentrale Bildverarbeitungs-Pipeline zwischen Kamera und Steuerungslogik. Der Node empfängt Grayscale-Bilder vom `CameraReadOut`-Node, führt ArUco-Marker-Detektion durch und publiziert verarbeitete Daten (Distanz, Winkel, Marker-ID) für die Logik-Module.

**Hauptfunktionen**:
- Subscription auf `/muri_image_raw` (Kamerabilder)
- ArUco-Marker-Detektion via AMD-Klasse
- Distanzfilterung mit Ring-Buffer (3 Werte)
- Fehlerüberwachung (10-Frame-Schwellwert)
- Publikation auf `/muri_picture_data` (PictureData)

## Systemintegration

CameraReadOut ──> /muri_image_raw ──> ImageProcessing ──> /muri_picture_data
│
AMD-Klasse
(ArUco-Detektion)
│
↓
Subscriber


Der Node bildet die Brücke zwischen Kamera-Hardware und Steuerungslogik.

---

## Klasse `ImageProcessing`

### Konstruktor `__init__()`

```python
def init(self):
    super().init('image_processing')
    self.bridge = CvBridge()
    self.distance_in_meters_unfiltered = None
    self.distance_in_meters_filtered = None
    self.distance_in_milimeters = None
    self.angle_in_rad = None
    self.marker_id = 9999
    self.error = False
    self.error_counter = 0
    self.first_data = -1.0
    self.second_data = -1.0
    self.third_data = -1.0
```

```python
self.proc_AMD = AMD()

self.subscription = self.create_subscription(
    Image, '/muri_image_raw', self.listener_callback, 10)

self.publisher = self.create_publisher(
    PictureData, '/muri_picture_data', 10)
```

### Instanzvariablen

**ArUco-Detektion**:
- `self.proc_AMD`: Instanz der AMD-Klasse (ArUco Marker Detection)
- `self.marker_id`: Erkannte Marker-ID (Default: 9999 = Fehlercode)
- `self.angle_in_rad`: Horizontaler Winkel zum Marker [rad]
- `self.distance_in_milimeters`: Rohdistanz in Millimetern
- `self.distance_in_meters_unfiltered`: Konvertierte ungefilterte Distanz [m]
- `self.distance_in_meters_filtered`: Gefilterte Distanz nach Ring-Buffer [m]

**Fehlerüberwachung**:
- `self.error`: Boolean-Flag für kritische Fehler
- `self.error_counter`: Zähler für konsekutive Frame-Verluste
- **Schwellwert**: 10 konsekutive Fehler → `error = True`
- **Zweck**: Erkennung von Kamera-/Verbindungsproblemen

**Ring-Buffer für Distanzfilterung**:
- `self.first_data`: Neuester Distanzwert [m]
- `self.second_data`: Vorletzter Distanzwert [m]
- `self.third_data`: Drittletzter Distanzwert [m]
- **Initialisierung**: `-1.0` (ungültiger Wert)
- **Zweck**: Stabilisierung bei kurzzeitigen Detektionsfehlern (schlechte Lichtverhältnisse, Marker kurzzeitig verdeckt)

**ROS2-Komponenten**:
- `self.bridge`: CvBridge-Instanz für Image-Konvertierung (als Instanzvariable für Performance)
- `self.subscription`: Subscriber auf `/muri_image_raw`
- `self.publisher`: Publisher auf `/muri_picture_data`

### Subscriber

- **Topic**: `/muri_image_raw`
- **Message-Typ**: `sensor_msgs/Image` (Grayscale, mono8)
- **Callback**: `listener_callback()`
- **Queue-Size**: 10

### Publisher

- **Topic**: `/muri_picture_data`
- **Message-Typ**: `muri_dev_interfaces/PictureData`
- **Queue-Size**: 10

**PictureData-Message-Definition**:

std_msgs/Header header
bool error
float32 angle_in_rad
float32 distance_in_meters
int32 dominant_aruco_id


**Felder**:
- `error`: Fehler-Flag (True nach 10 konsekutiven Frame-Verlusten)
- `angle_in_rad`: Winkel zum Marker [rad] (von AMD)
- `distance_in_meters`: **Gefilterte** Distanz [m] (nach Ring-Buffer)
- `dominant_aruco_id`: Marker-ID (69, 0, oder 9999 bei Fehler)

### Default-Werte

**Marker-ID = 9999**:
- Entspricht dem Fehlercode aus der AMD-Klasse
- Signalisiert: Kein Marker erkannt oder Detektionsfehler
- Garantiert kein Konflikt mit verwendeten IDs (0, 69)

**Error-Counter-Schwellwert = 10**:
- Empirisch ermittelter Wert
- Balance zwischen Robustheit und Fehler-Reaktionszeit
- Bei 30 Hz entspricht das ~333 ms Fehlertoleranz

**Ring-Buffer-Initialisierung = -1.0**:
- Ungültiger Wert (negative Distanz unmöglich)

---

## Methoden

### `listener_callback()`

**Zweck**: Callback für eingehende Kamerabilder, führt ArUco-Detektion durch und publiziert verarbeitete Daten.

```python
def listener_callback(self, msg):
    try:
        cv_raw_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
    except Exception as e:
        self.get_logger().error("Fehler bei der Bildübertragung")

    self.pic_to_data(cv_raw_image)

    pub_pic_data = PictureData()
    pub_pic_data.error = self.error
    pub_pic_data.angle_in_rad = float(self.angle_in_rad)
    pub_pic_data.distance_in_meters = float(self.distance_in_meters_filtered)
    pub_pic_data.dominant_aruco_id = int(self.marker_id)
    self.publisher.publish(pub_pic_data)
```

**Ablauf**:
1. ROS2-Image-Message zu OpenCV-Array konvertieren (Grayscale)
2. `pic_to_data()` aufrufen für ArUco-Detektion und Filterung
3. `PictureData`-Message erstellen und Felder befüllen
4. Message auf `/muri_picture_data` publishen

**Wichtig**:
- **Gefilterte Distanz** wird publiziert (`distance_in_meters_filtered`)
- Alle Instanzvariablen werden bei jedem Callback aktualisiert
- Bei Fehler in Konvertierung wird Error geloggt, aber Verarbeitung fortgesetzt

---

### `pic_to_data()`

**Zweck**: Führt ArUco-Detektion durch, konvertiert Einheiten und wendet Distanzfilterung an.

```python

def pic_to_data(self, data_img):
    if data_img is None:
        self.get_logger().info('Kein Frame erhalten!')
        self.error_counter += 1
    if self.error_counter > 10:
        self.error = True

    self.distance_in_milimeters, self.angle_in_rad, self.marker_id = \
    self.proc_AMD.aruco_detection(data_img)
    self.distance_in_meters_unfiltered = self.distance_in_milimeters/1000
    self.distance_in_meters_filtered = self.filter_distance()

```

**Ablauf**:
1. **Fehlerüberwachung**: Bei `None`-Input Error-Counter inkrementieren
2. **ArUco-Detektion**: AMD-Klasse aufrufen (gibt mm, rad, ID zurück)
3. **Einheitenkonvertierung**: Millimeter → Meter
4. **Distanzfilterung**: Ring-Buffer-Filter anwenden

**Error-Counter-Logik**:
- Bei `data_img is None`: Counter +1
- Bei Counter > 10: `self.error = True`
- **Kein Reset**: Counter wird nie zurückgesetzt (nur bei Node-Neustart)
- **Zweck**: Permanente Fehler erkennen (z.B. Kamera-Ausfall)

**Fehlerfall-Verhalten**:
- Auch bei `data_img is None` wird `aruco_detection()` aufgerufen
- AMD verwendet dann den alten/vorherigen Wert weiter
- Ring-Buffer sorgt für Stabilität bei kurzzeitigen Ausfällen

---

### `filter_distance()` 

**Zweck**: Buffer-Filter für Distanzwerte (Last-Valid-Value-Filter).

```python
def filter_distance(self):
    self.third_data = self.second_data
    self.second_data = self.first_data
    self.first_data = self.distance_in_meters_unfiltered

    for v in (self.first_data, self.second_data, self.third_data):
        if v != -1.0:
            return v
    return -1.0

```

**Funktionsweise**:
1. **Buffer-Shift**: Werte wandern durch den Buffer (newest → oldest)
2. **Neuesten gültigen Wert finden**: Iteriere von neu nach alt
3. **Rückgabe**: Ersten Wert ≠ -1.0, oder -1.0 falls alle ungültig

## Abhängigkeiten

```python
from muri_dev_interfaces.msg import PictureData
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
import rclpy
from rclpy.node import Node
from vision.aruco_marker_detection import AMD
```


**Custom Interfaces**:
- `muri_dev_interfaces`: Package mit `PictureData`-Message

**Lokale Module**:
- `vision.aruco_marker_detection`: AMD-Klasse für ArUco-Detektion

**ROS2-Pakete**:
- `rclpy`, `sensor_msgs`, `cv_bridge`

**Externe Bibliotheken**:
- `cv2` (OpenCV), `numpy`

---

## Main-Funktion

Standard ROS2-Node-Lifecycle.

```python
def main(args=None):
    rclpy.init(args=args)
    camera_read_out = ImageProcessing()
    rclpy.spin(camera_read_out)
    camera_read_out.destroy_node()
    rclpy.shutdown()
```
