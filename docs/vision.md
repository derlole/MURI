
## Systemarchitektur

### Komponenten

1. **CameraReadOut** - Kamera-Akquisition und Raw-Image-Streaming
2. **ImageProcessing** - Bildverarbeitung und Datenextraktion
3. **AMD (ArUco Marker Detection)** - OpenCV-basierte Marker-Erkennung

### Kommunikationsdiagramm

```
┌─────────────────────────────────────────────────────────────┐
│                     CameraReadOut Node                      │
│                    (camera_read_out)                        │
│                                                             │
│  ┌──────────────┐         ┌──────────────┐                  │
│  │  cv.VideoCapture       │   Timer      │                  │
│  │  (30 Hz)     │         │   (30 Hz)    │                  │
│  └──────┬───────┘         └──────┬───────┘                  │
│         │                        │                          │
│         └────────────┬───────────┘                          │
│                      ▼                                      │
│              Publisher: /muri_image_raw                     │
└──────────────────────┼──────────────────────────────────────┘
                       │ (sensor_msgs/Image, mono8)
                       ▼
┌─────────────────────────────────────────────────────────────┐
│                  ImageProcessing Node                       │
│                  (image_processing)                         │
│                                                             │
│  Subscriber: /muri_image_raw                                │
│         │                                                   │
│         ▼                                                   │
│  ┌──────────────┐         ┌──────────────┐                  │
│  │  CvBridge    │────────▶│     AMD      │                  │
│  │  Conversion  │         │  (ArUco)     │                  │
│  └──────────────┘         └──────┬───────┘                  │
│                                  │                          │
│                                  ▼                          │
│                      Distance & Angle Filter                │
│                                  │                          │
│                                  ▼                          │
│              Publisher: /muri_picture_data                  │
└──────────────────────┼──────────────────────────────────────┘
                       │ (PictureData)
                       ▼
                Navigation/Control System
```

---

## Detaillierte Komponentenbeschreibung

### 1. CameraReadOut Node

**Zweck:** Erfasst kontinuierlich Kamerabilder und publiziert diese als ROS2-Messages für die Weiterverarbeitung.

#### Konfiguration

- **Node-Name:** `camera_read_out`
- **Publisher:** `/muri_image_raw` (sensor_msgs/Image)
- **Timer-Rate:** 30 Hz (0.0333s)
- **Kamera-Auflösung:** 1920x1080 Pixel
- **Encoding:** mono8 (Graustufen)

#### Klassenstruktur

```python
class CameraReadOut(Node):
    def __init__(self)
```

#### Kamera-Konfiguration

```python
path_camera = 0  # Default: /dev/video0
self.img = cv.VideoCapture(path_camera)
self.img.set(cv.CAP_PROP_FRAME_WIDTH, 1920)
self.img.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)
self.img.set(cv.CAP_PROP_BUFFERSIZE, 1)
```

**Parameter:**
- Frame Width: 1920 Pixel
- Frame Height: 1080 Pixel
- Buffer Size: 1 Frame (minimale Latenz)
- Capture Device: /dev/video0 (Standard-Webcam)

#### Wichtige Methoden

##### `__init__()`

Initialisiert die Node und konfiguriert die Kamera:
- Erstellt Publisher für `/muri_image_raw`
- Öffnet Video-Capture-Device
- Setzt Auflösung und Buffer-Größe
- Erstellt Timer mit 30 Hz für kontinuierliche Bildakquisition

##### `timer_callback()`

Hauptschleife (30 Hz):

```python
def timer_callback(self):
    bridge = CvBridge()
    msg = bridge.cv2_to_imgmsg(self.read_camera(), encoding='mono8')
    self.publisher.publish(msg)
    self.get_logger().info('Bild wird verschickt...')
```

**Ablauf:**
1. Erstellt CvBridge-Instanz für Konvertierung
2. Liest Frame mit `read_camera()`
3. Konvertiert OpenCV-Bild zu ROS2-Message
4. Publiziert Message auf `/muri_image_raw`
5. Loggt Versand-Status

##### `read_camera()`

Liest einzelnes Frame von der Kamera:

```python
def read_camera(self):
    success, frame = self.img.read()
    frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    
    if not success:
        self.get_logger().info('Bild konnte nicht gelesen werden')
        return None
    return frame_gray
```

**Verarbeitung:**
- Liest RGB-Frame von VideoCapture
- Konvertiert zu Graustufen (BGR → GRAY)
- Gibt Graustufenbild zurück oder `None` bei Fehler

#### Nachrichtenformat

**Output: sensor_msgs/Image**

```
header:
  stamp: <timestamp>
  frame_id: ""
height: 1080
width: 1920
encoding: "mono8"
is_bigendian: 0
step: 1920
data: [uint8 array]
```

#### Besonderheiten

- **Minimaler Buffer:** Buffer-Größe 1 minimiert Latenz für Echtzeit-Verarbeitung
- **Graustufen-Konvertierung:** Erfolgt bereits in der Akquisition für reduzierte Bandbreite
- **Keine Fehlerbehandlung bei Kamera-Ausfall:** Node terminiert nicht, gibt None zurück
- **Logging bei jedem Frame:** Kann zu hoher Log-Last führen (30 msg/s)

---

### 2. ImageProcessing Node

**Zweck:** Empfängt Raw-Images, führt ArUco-Marker-Detektion durch und publiziert extrahierte Positionsdaten (Distanz und Winkel).

#### Konfiguration

- **Node-Name:** `image_processing`
- **Subscriber:** `/muri_image_raw` (sensor_msgs/Image)
- **Publisher:** `/muri_picture_data` (muri_dev_interfaces/PictureData)
- **Queue Size:** 10 (Subscriber & Publisher)

#### Klassenstruktur

```python
class ImageProcessing(Node):
    def __init__(self)
```

#### Interne Zustandsvariablen

```python
# Distanz-Daten
self.distance_in_meters_unfiltered: float   # Rohdistanz in Metern
self.distance_in_meters_filtered: float     # Gefilterte Distanz
self.distance_in_milimeters: float          # Distanz in Millimetern

# Winkel-Daten
self.angle_in_rad: float                    # Winkel zum Marker in Radiant

# Fehlerbehandlung
self.error: bool                            # Error-Flag für >10 verlorene Frames
self.error_counter: int                     # Zähler für verlorene Frames

# Ringpuffer für Filter
self.first_data: float                      # Neuster Wert
self.second_data: float                     # Vorletzter Wert
self.third_data: float                      # Drittletzter Wert

# Prozessor
self.proc_AMD: AMD                          # ArUco-Detektor-Instanz
```

#### Wichtige Methoden

##### `__init__()`

Initialisiert die Node und alle Komponenten:

```python
def __init__(self):
    super().__init__('image_processing')
    
    self.bridge = CvBridge()
    # ... Zustandsvariablen initialisieren ...
    self.proc_AMD = AMD()
    
    # Subscriber erstellen
    self.subscription = self.create_subscription(
        Image,
        '/muri_image_raw',
        self.listener_callback,
        10)
    
    # Publisher erstellen
    self.publisher = self.create_publisher(
        PictureData,
        '/muri_picture_data',
        10)
```

##### `listener_callback(msg)`

ROS2-Callback für eingehende Bilder:

```python
def listener_callback(self, msg):
    self.data = msg
    cv_raw_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
    self.get_logger().info('Bild empfangen!')
    self.get_logger().info(f'tvec_unfiltered: {self.distance_in_meters_unfiltered}    rvec: {self.angle_in_rad}')
    
    self.pic_to_data(cv_raw_image)
    
    self.get_logger().info(f'tvec_filtered: {self.distance_in_meters_filtered}')
    
    pub_pic_data = PictureData()
    pub_pic_data.error = self.error
    pub_pic_data.angle_in_rad = float(self.angle_in_rad)
    pub_pic_data.distance_in_meters = float(self.distance_in_meters_unfiltered)
    
    self.publisher.publish(pub_pic_data)
    self.get_logger().info('OpenCV-Daten werden gepublished...')
```

**Ablauf:**
1. Konvertiert ROS2-Image zu OpenCV-Format (mono8)
2. Loggt Empfang und aktuelle ungefilterte Werte
3. Ruft `pic_to_data()` für Marker-Detektion auf
4. Loggt gefilterte Distanz
5. Erstellt PictureData-Message mit Error-Flag, Winkel und Distanz
6. Publiziert verarbeitete Daten
7. Loggt Versand-Status

##### `pic_to_data(data_img)`

Verarbeitet OpenCV-Bild und extrahiert Marker-Daten:

```python
def pic_to_data(self, data_img):
    if data_img is None:
        self.get_logger().info('Kein Frame erhalten!')
        self.error_counter += 1

    if self.error_counter > 10:
        self.error = True

    self.distance_in_milimeters, self.angle_in_rad = self.proc_AMD.aruco_detection(data_img)
    self.distance_in_meters_unfiltered = self.distance_in_milimeters/1000
    self.distance_in_meters_filtered = self.daf()
```

**Verarbeitung:**
1. Prüft auf None-Input, inkrementiert Error-Counter
2. Setzt Error-Flag nach 10 verlorenen Frames
3. Ruft ArUco-Detektion auf
4. Konvertiert Millimeter zu Meter
5. Wendet Distanz-Filter an (DAF)

##### `daf()`

Einfacher Ringpuffer-Filter:

```python
def daf(self):
    # 1. Ring-Buffer aktualisieren
    self.third_data  = self.second_data
    self.second_data = self.first_data
    self.first_data  = self.distance_in_meters_unfiltered

    # 2. Das *neuste* gültige Ergebnis zurückgeben
    for v in (self.first_data, self.second_data, self.third_data):
        if v != -1.0:          # gibt den neuesten Wert zurück ungleich -1.0
            return v
    return -1.0                # alle drei == -1.0
```

**Funktionsweise:**
- Schiebt neue Werte durch 3-Element-Ringpuffer
- Gibt ersten gefundenen gültigen Wert zurück (neuester zuerst)
- -1.0 signalisiert "kein Marker erkannt"

#### Nachrichtenformat

**Output: muri_dev_interfaces/PictureData**

```python
error: bool                    # True wenn >10 Frames verloren
angle_in_rad: float32          # Winkel zum Marker in Radiant
distance_in_meters: float32    # Distanz zum Marker in Metern (ungefiltert)
```

---

### 3. AMD (ArUco Marker Detection)

**Zweck:** Kapselt OpenCV ArUco-Detektion mit Kamera-Kalibrierung und liefert Distanz und Winkel zum erkannten Marker.

#### Klassenstruktur

```python
class AMD():
    def __init__(self)
```

#### Konfiguration

```python
self.marker_size = config.MARKER_SIZE  # Marker-Größe in mm
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
aruco_params = aruco.DetectorParameters()
self.detector = aruco.ArucoDetector(aruco_dict, aruco_params)

self.camera_matrix = np.array(config.CAMERA_MATRIX_RAW, dtype=np.float32)
self.dist_coeffs = np.array(config.DISTANCE_COEFFICIENT, dtype=np.float32)
```

**Parameter:**
- **ArUco Dictionary:** DICT_5X5_1000 (5x5 Bit, 1000 einzigartige IDs)
- **Marker Size:** Aus config.MARKER_SIZE (in Millimetern)
- **Camera Matrix:** 3x3 intrinsische Kameramatrix aus Kalibrierung
- **Distortion Coefficients:** Verzerrungskoeffizienten der Kamera

#### Camera Matrix Format

```python
camera_matrix = [
    [fx,  0, cx],
    [ 0, fy, cy],
    [ 0,  0,  1]
]
```

#### Wichtige Methoden

##### `__init__()`

Initialisiert Detektor und Kamera-Parameter:
- Lädt ArUco Dictionary (5x5, 1000 IDs)
- Erstellt Detektor mit Default-Parametern
- Lädt intrinsische Kameramatrix aus Config
- Lädt Distortion Coefficients aus Config

##### `aruco_detection(img)`

Hauptmethode für Marker-Erkennung:

```python
def aruco_detection(self, img):
    frame_gray = img
    corners, ids, _ = self.detector.detectMarkers(frame_gray)
    
    if ids is not None and len(corners) > 0:
        for i in range(len(ids)):
            obj_points = np.array([
                [-self.marker_size / 2,  self.marker_size / 2, 0],
                [ self.marker_size / 2,  self.marker_size / 2, 0],
                [ self.marker_size / 2, -self.marker_size / 2, 0],
                [-self.marker_size / 2, -self.marker_size / 2, 0]
            ], dtype=np.float32)
            
            success, rvec, tvec = cv.solvePnP(
                obj_points,
                corners[i][0],
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv.SOLVEPNP_IPPE_SQUARE)
            
            if success:
                angle_rad = self.calculate_angle_to_marker(corners[i])
                
                print(f'tvec: 0:{tvec[0]}    1:{tvec[1]}     2:{tvec[2]}')
                print(f'Berechneter Winkel: {angle_rad} rad ({math.degrees(angle_rad):.2f}°)')
                
                return tvec[2][0], angle_rad
    
    return -1000.0, math.pi
```

**Rückgabewerte:**
- **Bei Erfolg:** `(z_position_mm, angle_rad)`
  - z_position_mm: Distanz in Millimetern (tvec[2])
  - angle_rad: Winkel in Radiant
- **Bei Fehler:** `(-1000.0, math.pi)`

**Ablauf:**
1. Marker-Detektion: Ruft `detectMarkers()` auf Graustufenbild auf
2. Validierung: Prüft ob Marker gefunden wurden (ids is not None)
3. 3D-Punktdefinition: Definiert Marker-Ecken im 3D-Raum (Objektkoordinaten)
4. Pose-Estimation: Nutzt `cv.solvePnP()` mit IPPE_SQUARE-Algorithmus
5. Winkel-Berechnung: Ruft `calculate_angle_to_marker()` auf
6. Logging: Gibt Translation- und Rotationsvektor aus
7. Rückgabe: Z-Koordinate (Tiefe) und berechneter Winkel

##### `calculate_angle_to_marker(corners)`

Berechnet Gierwinkel (Yaw) zum Marker:

```python
def calculate_angle_to_marker(self, corners):
    marker_corners = corners[0]
    marker_center_x = np.mean(marker_corners[:, 0])
    marker_center_y = np.mean(marker_corners[:, 1])
    
    cx = self.camera_matrix[0, 2]  # 971.25
    cy = self.camera_matrix[1, 2]  # 472.44
    
    fx = self.camera_matrix[0, 0]  # 1856.56
    
    delta_x = marker_center_x - cx
    
    angle_rad = math.atan2(delta_x, fx)
    
    return angle_rad
```

**Berechnungsschritte:**
1. Extrahiert Marker-Ecken aus Ergebnis
2. Berechnet Marker-Zentrum (Mittelwert x/y-Koordinaten)
3. Extrahiert Principal Point (cx, cy) aus Kameramatrix
4. Extrahiert Brennweite (fx) aus Kameramatrix
5. Berechnet Horizontal-Offset zum Bildzentrum: `delta_x = marker_center_x - cx`
6. Berechnet Winkel: `angle = atan2(delta_x, fx)`

**Winkel-Konvention:**
- **Positiver Winkel:** Marker rechts vom Bildzentrum
- **Negativer Winkel:** Marker links vom Bildzentrum
- **0 Radiant:** Marker im Bildzentrum

#### 3D-Pose-Estimation Details

**Object Points Definition:**

```python
obj_points = [
    [-size/2,  size/2, 0],  # Obere linke Ecke
    [ size/2,  size/2, 0],  # Obere rechte Ecke
    [ size/2, -size/2, 0],  # Untere rechte Ecke
    [-size/2, -size/2, 0]   # Untere linke Ecke
]
```

Definiert Marker im Marker-Koordinatensystem:
- Ursprung im Marker-Zentrum
- Z=0 (Marker liegt in XY-Ebene)
- Größe aus `config.MARKER_SIZE`

**solvePnP() Methode:**
- **Algorithmus:** `SOLVEPNP_IPPE_SQUARE`
  - Optimiert für planare Marker
  - "Infinitesimal Plane-based Pose Estimation"
  - Schnell und robust für quadratische Marker
- **Input:**
  - 3D-Objektpunkte (Marker-Ecken)
  - 2D-Bildpunkte (detektierte Ecken)
  - Kameramatrix (intrinsische Parameter)
  - Distortion Coefficients
- **Output:**
  - `rvec`: Rotationsvektor (Rodrigues-Format)
  - `tvec`: Translationsvektor [X, Y, Z]

**Translation Vector (tvec):**

```
tvec[0] = X-Position (horizontal)
tvec[1] = Y-Position (vertikal)
tvec[2] = Z-Position (Tiefe/Distanz) ← wird verwendet
```

#### Besonderheiten

- **Nur erster Marker:** Bei mehreren Markern wird nur der erste verarbeitet
- **Keine Marker-ID-Filterung:** Akzeptiert alle ArUco-IDs aus DICT_5X5_1000
- **Print-Statements:** Nutzt `print()` statt ROS-Logger (nicht in ROS-Logs sichtbar)
- **Hardcoded Fallback-Werte:** Bei Fehler wird `(-1000.0, math.pi)` zurückgegeben
- **Keine Fehlerbehandlung:** Keine spezifische Behandlung bei solvePnP-Fehler

---

## Datenfluss und Verarbeitung

### Kompletter Pipeline-Ablauf

```
1. Kamera-Hardware
        │
        ▼
2. CameraReadOut.read_camera()
   ├─ cv.VideoCapture.read()
   ├─ BGR → GRAY Konvertierung
   └─ Return: numpy.ndarray (1920x1080, mono8)
        │
        ▼
3. CameraReadOut.timer_callback()
   ├─ cv2_to_imgmsg() Konvertierung
   └─ Publish: /muri_image_raw
        │
        ▼
4. ImageProcessing.listener_callback()
   ├─ imgmsg_to_cv2() Konvertierung
   └─ Call: pic_to_data()
        │
        ▼
5. ImageProcessing.pic_to_data()
   ├─ Call: AMD.aruco_detection()
   ├─ mm → m Konvertierung
   └─ Call: daf() Filter
        │
        ▼
6. AMD.aruco_detection()
   ├─ detectMarkers()
   ├─ solvePnP() für Pose
   ├─ calculate_angle_to_marker()
   └─ Return: (distance_mm, angle_rad)
        │
        ▼
7. ImageProcessing.listener_callback()
   ├─ Erstelle PictureData Message
   └─ Publish: /muri_picture_data
        │
        ▼
8. Navigation/Control System
```

### Koordinatensysteme

#### Kamera-Koordinatensystem

```
    Y (nach unten)
    │
    │
    └───── X (nach rechts)
   ╱
  ╱
 Z (in Blickrichtung, Tiefe)
```

#### Marker-Koordinatensystem

```
    Y (nach oben)
    │
    │
    └───── X (nach rechts)
   ╱
  ╱
 Z (senkrecht aus Marker, zu Kamera)
```

#### Transformation

- `tvec` gibt Position des Markers im Kamera-Koordinatensystem
- `tvec[2]` entspricht Distanz entlang optischer Achse
- `angle_rad` ist Rotation um Y-Achse (Gierwinkel)

## Version

Dokumentiert am: 2025-11-25  
ROS2 Version: Humble Hawksbill  
Python Version: 3.10.12